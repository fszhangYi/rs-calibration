from __future__ import annotations

import os
import threading
import time
from pathlib import Path
from typing import Any, Optional

import cv2
import numpy as np
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from rs_calib.app_services.arm_service import ArmService
from rs_calib.app_services.camera_service import CameraService
from rs_calib.app_services.live_calib_service import LiveCalibService
from rs_calib.app_services.matrix_path_service import MatrixPathService
from rs_calib.app_services.offline_calib_service import OfflineCalibService
from rs_calib.app_services.record_service import RecordService
from rs_calib.app_services.session_report_service import build_session_report
from rs_calib.config.store import load_config
from rs_calib.domain.aruco import aruco_meta_dict, detect_target
from rs_calib.domain.matrix_path import MatrixPathParams
from rs_calib.errors import Ok
from rs_calib.infra import session_store
from rs_calib.paths import resolve_frontend_dist
from rs_calib.version import __version__


class ConnectArmBody(BaseModel):
    robot_ip: Optional[str] = None
    dry_run: bool = False


class OpenCameraBody(BaseModel):
    serial: Optional[str] = None
    serials: Optional[list[str]] = None
    width: Optional[int] = None
    height: Optional[int] = None
    fps: Optional[int] = None
    preview_serial: Optional[str] = None


class JogJointBody(BaseModel):
    joint: int = Field(ge=0, le=5)
    sign: int = Field(description="+1 or -1")


class JogCartBody(BaseModel):
    axis: str
    sign: int


class TcpBody(BaseModel):
    tcp_xyz: list[float]


class JogStepsBody(BaseModel):
    jog_joint_step_deg: Optional[float] = None
    jog_cart_step_m: Optional[float] = None
    jog_cart_step_deg: Optional[float] = None
    cart_frame: Optional[str] = None
    max_delta_deg: Optional[float] = None
    v_norm_rad_s: Optional[float] = None
    t_min_s: Optional[float] = None
    t_max_s: Optional[float] = None


class ArucoBody(BaseModel):
    board_type: Optional[str] = None
    dict: Optional[str] = None
    marker_id: Optional[int] = None
    marker_length_m: Optional[float] = None
    squares_x: Optional[int] = None
    squares_y: Optional[int] = None
    square_length_m: Optional[float] = None
    charuco_marker_length_m: Optional[float] = None
    min_charuco_corners: Optional[int] = None


class OfflineBody(BaseModel):
    session_dir: str


class SessionReportBody(BaseModel):
    session_dir: Optional[str] = None


class HandeyeSetupBody(BaseModel):
    setup: str = "eye_to_hand"
    method: Optional[str] = None
    dual_backends: Optional[bool] = None


class MatrixPlanBody(BaseModel):
    nx: Optional[int] = None
    ny: Optional[int] = None
    nz: Optional[int] = None
    step_x_m: Optional[float] = None
    step_y_m: Optional[float] = None
    step_z_m: Optional[float] = None
    n_rx: Optional[int] = None
    n_ry: Optional[int] = None
    n_rz: Optional[int] = None
    step_rx_deg: Optional[float] = None
    step_ry_deg: Optional[float] = None
    step_rz_deg: Optional[float] = None
    snake: Optional[bool] = None
    dwell_s: Optional[float] = None
    frame: Optional[str] = None
    seed_xyzrpy: Optional[list[float]] = None


def create_app() -> FastAPI:
    cfg = load_config()
    session_root = (os.environ.get("RS_CALIB_SESSION_ROOT") or "").strip()
    if session_root:
        cfg.app.session_root = session_root
    camera = CameraService()
    arm = ArmService(cfg)
    record = RecordService(cfg, camera, arm)
    live = LiveCalibService(cfg)
    offline = OfflineCalibService(cfg)
    matrix = MatrixPathService(cfg, arm)
    last_session_dir: dict[str, Optional[Path]] = {"path": None}

    state_lock = threading.Lock()
    latest_frame: dict[str, Any] = {"img": None, "ts": 0.0, "jpeg": None}
    grab_thread: dict[str, Optional[threading.Thread]] = {"t": None, "stop": False}
    matrix_state: dict[str, Any] = {
        "waypoints": [],
        "seed": None,
        "params": None,
        "running": False,
        "progress": None,
        "last_error": None,
        "last_result": None,
        "thread": None,
        "stopped_for_converge": False,
        "session": None,
    }

    def _restart_record_for_matrix() -> dict[str, Any]:
        """Stop any active session and start a fresh one for matrix-path calib."""
        if camera.n_open <= 0:
            raise HTTPException(400, "矩阵走位标定请先打开相机")
        if record.recording:
            stop_res = record.stop()
            if not isinstance(stop_res, Ok):
                raise HTTPException(400, f"无法结束上一录制: {stop_res.error}")
        start_res = record.start()
        if not isinstance(start_res, Ok):
            raise HTTPException(400, f"无法开始录制: {start_res.error}")
        live_on = False
        if not camera.is_multi:
            live.reset()
            live.set_enabled(True)
            live_on = True
        else:
            live.set_enabled(False)
        return {
            "session": str(start_res.value),
            "live": live_on,
            "camera_count": camera.n_open,
            "serials": camera.open_serials,
        }

    def _stop_record_after_matrix() -> Optional[str]:
        if not record.recording:
            return None
        try:
            if live.last_result is not None and record.session_dir is not None:
                live.persist(record.session_dir)
        except Exception:  # noqa: BLE001
            pass
        stop_res = record.stop()
        if isinstance(stop_res, Ok):
            return str(stop_res.value)
        return None

    frontend = resolve_frontend_dist()
    index_html = frontend / "index.html"

    app = FastAPI(title="rs-calibration", version=__version__)

    def _stop_grab() -> None:
        grab_thread["stop"] = True
        t = grab_thread["t"]
        if t is not None and t.is_alive():
            t.join(timeout=2.0)
        grab_thread["t"] = None

    def _grab_loop() -> None:
        last_detect_t = 0.0
        detect_period_s = 0.12  # throttle board detect — full-rate Charuco stalls the stream
        while not grab_thread["stop"]:
            if camera.n_open <= 0:
                time.sleep(0.05)
                continue
            grabbed = camera.grab_all(1000)
            if not grabbed:
                time.sleep(0.01)
                continue
            images = {s: im for s, (_t, im) in grabbed.items()}
            preview = camera.preview_serial
            if preview and preview in grabbed:
                ts = grabbed[preview][0]
            else:
                ts = next(iter(grabbed.values()))[0]
            sample = record.on_frames(ts, images)
            serial = preview if preview in images else next(iter(images))
            img = images[serial]
            show = img
            now = time.time()
            intr = camera.current_intrinsics(serial)
            do_detect = intr is not None and (now - last_detect_t) >= detect_period_s
            if do_detect:
                last_detect_t = now
                try:
                    _, dbg = detect_target(img, intr, cfg.aruco, draw=True)
                    if dbg is not None:
                        show = dbg
                except Exception:  # noqa: BLE001
                    show = img
                if sample is not None and live.enabled and not camera.is_multi and len(images) == 1:
                    try:
                        live.offer_frame(
                            img,
                            sample,
                            intr,
                            session_dir=record.session_dir,
                        )
                    except Exception:  # noqa: BLE001
                        pass
            with state_lock:
                latest_frame["img"] = show
                latest_frame["ts"] = ts
                latest_frame["jpeg"] = None  # invalidate cached jpeg
            # Do NOT arm.poll() here — TCP round-trips stall the camera grab/stream.

    def _ensure_grab() -> None:
        if grab_thread["t"] is not None and grab_thread["t"].is_alive():
            return
        grab_thread["stop"] = False
        t = threading.Thread(target=_grab_loop, daemon=True)
        grab_thread["t"] = t
        t.start()

    @app.on_event("shutdown")
    def _shutdown() -> None:
        _stop_grab()
        camera.close()
        arm.disconnect()

    @app.get("/", response_class=HTMLResponse)
    def index() -> HTMLResponse:
        if index_html.is_file():
            return HTMLResponse(index_html.read_text(encoding="utf-8"))
        return HTMLResponse("<h1>rs-calibration</h1><p>frontend-dist/index.html missing</p>")

    # Optional static assets next to index.html (CSS/JS splits later)
    if frontend.is_dir():
        app.mount("/assets", StaticFiles(directory=str(frontend)), name="assets")

    @app.get("/api/health")
    def health() -> dict:
        return {
            "ok": True,
            "version": __version__,
            "camera_open": camera.n_open > 0,
            "camera_count": camera.n_open,
            "camera_serials": camera.open_serials,
            "multi_camera": camera.is_multi,
            "preview_serial": camera.preview_serial,
            "arm_connected": arm.adapter.is_connected,
            "armed": arm.adapter.armed,
            "recording": record.recording,
            "live": live.enabled and not camera.is_multi,
            "has_session": record.session_dir is not None,
            "has_live_result": live.last_result is not None,
            "live_converged": bool(live.last_status.converged) if live.last_status else False,
            "live_saved": str(live._saved_path) if live._saved_path else None,
            "live_solve_idx": live._solve_idx,
            "live_history": str(live._history_path) if live._history_path else None,
            # Button: enable only after live convergence (status kept after stop).
            "can_session_report": bool(live.last_status.converged) if live.last_status else False,
            "last_session": str(
                record.session_dir or last_session_dir["path"] or ""
            )
            or None,
        }

    @app.get("/api/config")
    def get_config() -> dict:
        from rs_calib.domain.handeye import opencv_handeye_status

        return {
            "tcp_xyz": cfg.tcp_xyz,
            "arm": {
                "robot_ip": cfg.arm.robot_ip,
                "cart_frame": cfg.arm.cart_frame,
                "jog_joint_step_deg": cfg.arm.jog_joint_step_deg,
                "jog_cart_step_m": cfg.arm.jog_cart_step_m,
                "jog_cart_step_deg": cfg.arm.jog_cart_step_deg,
                "max_delta_deg": cfg.arm.max_delta_deg,
                "v_norm_rad_s": cfg.arm.v_norm_rad_s,
                "t_min_s": cfg.arm.t_min_s,
                "t_max_s": cfg.arm.t_max_s,
            },
            "camera": {
                "width": cfg.camera.width,
                "height": cfg.camera.height,
                "fps": cfg.camera.fps,
            },
            "aruco": aruco_meta_dict(cfg.aruco),
            "handeye": {
                "setup": getattr(cfg.handeye, "setup", "eye_to_hand"),
                "method": cfg.handeye.method,
                "compare_methods": cfg.handeye.compare_methods,
                "dual_backends": bool(getattr(cfg.handeye, "dual_backends", False)),
            },
            "opencv_handeye": opencv_handeye_status(),
        }

    @app.post("/api/handeye/setup")
    def set_handeye_setup(body: HandeyeSetupBody) -> dict:
        from rs_calib.config.store import save_config
        from rs_calib.domain.handeye import normalize_setup, opencv_handeye_status

        try:
            setup = normalize_setup(body.setup)
        except ValueError as e:
            raise HTTPException(400, str(e)) from e
        cfg.handeye.setup = setup
        if body.method:
            cfg.handeye.method = str(body.method).strip().upper() or cfg.handeye.method
        if body.dual_backends is not None:
            cfg.handeye.dual_backends = bool(body.dual_backends)
        live.reset()
        try:
            save_config(cfg)
        except Exception:  # noqa: BLE001
            pass
        return {
            "ok": True,
            "setup": cfg.handeye.setup,
            "method": cfg.handeye.method,
            "dual_backends": bool(cfg.handeye.dual_backends),
            "opencv_handeye": opencv_handeye_status(),
        }

    @app.post("/api/shutdown")
    def shutdown_server() -> dict:
        """Safe exit: stop grab / camera / arm, then terminate the process."""
        import os
        import threading
        import time

        def _die() -> None:
            time.sleep(0.35)
            try:
                _stop_grab()
            except Exception:  # noqa: BLE001
                pass
            try:
                camera.close()
            except Exception:  # noqa: BLE001
                pass
            try:
                arm.disconnect()
            except Exception:  # noqa: BLE001
                pass
            os._exit(0)

        threading.Thread(target=_die, name="rs-calib-shutdown", daemon=True).start()
        return {"ok": True, "message": "shutting down"}

    @app.post("/api/tcp")
    def set_tcp(body: TcpBody) -> dict:
        if len(body.tcp_xyz) != 3:
            raise HTTPException(400, "tcp_xyz must have 3 floats")
        cfg.tcp_xyz = [float(x) for x in body.tcp_xyz]
        return {"ok": True, "tcp_xyz": cfg.tcp_xyz}

    @app.post("/api/jog_steps")
    def set_jog_steps(body: JogStepsBody) -> dict:
        if body.jog_joint_step_deg is not None:
            v = float(body.jog_joint_step_deg)
            if not (0.01 <= v <= 30.0):
                raise HTTPException(400, "jog_joint_step_deg 范围 0.01~30°")
            cfg.arm.jog_joint_step_deg = v
        if body.jog_cart_step_m is not None:
            v = float(body.jog_cart_step_m)
            if not (1e-5 <= v <= 0.1):
                raise HTTPException(400, "jog_cart_step_m 范围 0.01mm~100mm")
            cfg.arm.jog_cart_step_m = v
        if body.jog_cart_step_deg is not None:
            v = float(body.jog_cart_step_deg)
            if not (0.01 <= v <= 30.0):
                raise HTTPException(400, "jog_cart_step_deg 范围 0.01~30°")
            cfg.arm.jog_cart_step_deg = v
        if body.cart_frame is not None:
            frame = str(body.cart_frame).strip().lower()
            if frame not in ("flange", "tcp"):
                raise HTTPException(400, "cart_frame 须为 flange 或 tcp")
            cfg.arm.cart_frame = frame
        if body.max_delta_deg is not None:
            res = arm.set_max_delta_deg(float(body.max_delta_deg))
            if not isinstance(res, Ok):
                raise HTTPException(400, str(res.error))
        if body.v_norm_rad_s is not None:
            res = arm.set_v_norm_rad_s(float(body.v_norm_rad_s))
            if not isinstance(res, Ok):
                raise HTTPException(400, str(res.error))
        if body.t_min_s is not None:
            v = float(body.t_min_s)
            if not (0.02 <= v <= 30.0):
                raise HTTPException(400, "t_min_s 范围 0.02~30")
            cfg.arm.t_min_s = v
        if body.t_max_s is not None:
            v = float(body.t_max_s)
            if not (0.1 <= v <= 30.0):
                raise HTTPException(400, "t_max_s 范围 0.1~30")
            cfg.arm.t_max_s = v
        return {
            "ok": True,
            "jog_joint_step_deg": cfg.arm.jog_joint_step_deg,
            "jog_cart_step_m": cfg.arm.jog_cart_step_m,
            "jog_cart_step_deg": cfg.arm.jog_cart_step_deg,
            "cart_frame": cfg.arm.cart_frame,
            "max_delta_deg": cfg.arm.max_delta_deg,
            "v_norm_rad_s": cfg.arm.v_norm_rad_s,
            "t_min_s": cfg.arm.t_min_s,
            "t_max_s": cfg.arm.t_max_s,
            "driver_max_delta_deg": arm.adapter.get_max_delta_deg(),
        }

    @app.post("/api/aruco")
    def set_aruco(body: ArucoBody) -> dict:
        a = cfg.aruco
        if body.board_type is not None:
            bt = str(body.board_type).strip().lower()
            if bt not in ("charuco", "aruco", "single", "marker"):
                raise HTTPException(400, "board_type 须为 charuco 或 aruco")
            a.board_type = "aruco" if bt in ("aruco", "single", "marker") else "charuco"
        if body.dict is not None:
            a.dict = str(body.dict).strip() or a.dict
        if body.marker_id is not None:
            a.marker_id = int(body.marker_id)
        if body.marker_length_m is not None:
            v = float(body.marker_length_m)
            if not (0.001 <= v <= 2.0):
                raise HTTPException(400, "marker_length_m 范围 0.001~2 m")
            a.marker_length_m = v
        if body.squares_x is not None:
            a.squares_x = max(2, int(body.squares_x))
        if body.squares_y is not None:
            a.squares_y = max(2, int(body.squares_y))
        if body.square_length_m is not None:
            v = float(body.square_length_m)
            if not (0.001 <= v <= 1.0):
                raise HTTPException(400, "square_length_m 范围 0.001~1 m")
            a.square_length_m = v
        if body.charuco_marker_length_m is not None:
            v = float(body.charuco_marker_length_m)
            if not (0.001 <= v <= 1.0):
                raise HTTPException(400, "charuco_marker_length_m 范围 0.001~1 m")
            a.charuco_marker_length_m = v
        if body.min_charuco_corners is not None:
            a.min_charuco_corners = max(4, int(body.min_charuco_corners))
        if a.charuco_marker_length_m >= a.square_length_m:
            raise HTTPException(400, "charuco 码边长须小于棋盘格边长")
        return {"ok": True, "aruco": aruco_meta_dict(a)}

    @app.get("/api/cameras")
    def list_cameras() -> Any:
        res = camera.scan()
        if not isinstance(res, Ok):
            raise HTTPException(500, str(res.error))
        return [
            {"serial": d.serial, "name": d.name, "firmware": d.firmware} for d in res.value
        ]

    @app.post("/api/camera/open")
    def open_camera(body: OpenCameraBody) -> dict:
        serials = list(body.serials or [])
        if body.serial:
            serials.insert(0, body.serial)
        # unique preserve order
        seen = set()
        ordered = []
        for s in serials:
            if s and s not in seen:
                seen.add(s)
                ordered.append(s)
        if not ordered:
            raise HTTPException(400, "serial 或 serials 必填")
        w = body.width or cfg.camera.width
        h = body.height or cfg.camera.height
        fps = body.fps or cfg.camera.fps
        if len(ordered) == 1:
            res = camera.open(ordered[0], w, h, fps, replace_all=True)
            if not isinstance(res, Ok):
                raise HTTPException(400, str(res.error))
            opened = {ordered[0]: res.value}
        else:
            res = camera.open_many(ordered, w, h, fps)
            if not isinstance(res, Ok):
                raise HTTPException(400, str(res.error))
            opened = res.value
        if body.preview_serial:
            camera.set_preview_serial(body.preview_serial)
        _ensure_grab()
        # disable live when multi
        if camera.is_multi and live.enabled:
            live.set_enabled(False)
        intr = camera.current_intrinsics()
        return {
            "ok": True,
            "serials": list(opened.keys()),
            "multi_camera": camera.is_multi,
            "preview_serial": camera.preview_serial,
            "intrinsics": None
            if intr is None
            else {
                "fx": intr.fx,
                "fy": intr.fy,
                "cx": intr.cx,
                "cy": intr.cy,
                "dist": intr.dist,
                "width": intr.width,
                "height": intr.height,
                "fps": intr.fps,
            },
        }

    @app.post("/api/camera/preview")
    def set_preview(serial: str) -> dict:
        res = camera.set_preview_serial(serial)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        intr = res.value
        return {
            "ok": True,
            "preview_serial": serial,
            "intrinsics": {
                "fx": intr.fx,
                "fy": intr.fy,
                "cx": intr.cx,
                "cy": intr.cy,
                "dist": intr.dist,
                "width": intr.width,
                "height": intr.height,
                "fps": intr.fps,
            },
        }

    @app.post("/api/camera/close")
    def close_camera() -> dict:
        camera.close()
        with state_lock:
            latest_frame["img"] = None
        return {"ok": True}

    @app.get("/api/camera/intrinsics")
    def get_intrinsics() -> dict:
        intr = camera.current_intrinsics()
        if intr is None:
            raise HTTPException(400, "相机未打开")
        return {
            "fx": intr.fx,
            "fy": intr.fy,
            "cx": intr.cx,
            "cy": intr.cy,
            "dist": intr.dist,
            "width": intr.width,
            "height": intr.height,
            "fps": intr.fps,
        }

    @app.post("/api/arm/connect")
    def arm_connect(body: ConnectArmBody) -> dict:
        ip = body.robot_ip or cfg.arm.robot_ip
        cfg.arm.robot_ip = ip
        res = arm.connect(ip, dry_run=body.dry_run)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        st = res.value
        return {
            "ok": True,
            "endpoint": st.endpoint,
            "armed": st.armed,
            "joints_rad": st.joints_rad,
            "error": st.error,
        }

    @app.post("/api/arm/disconnect")
    def arm_disconnect() -> dict:
        arm.disconnect()
        return {"ok": True}

    @app.post("/api/arm/arm")
    def arm_enable() -> dict:
        res = arm.arm()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "result": res.value}

    @app.post("/api/arm/disarm")
    def arm_disarm() -> dict:
        res = arm.disarm()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "result": res.value}

    @app.post("/api/arm/stop")
    def arm_stop() -> dict:
        res = arm.stop()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "result": res.value}

    @app.get("/api/arm/state")
    def arm_state() -> dict:
        if not arm.adapter.is_connected:
            return {"ok": False, "connected": False}
        st = arm.poll()
        pose = None
        if st.ok and st.joints_rad is not None:
            _, xyzrpy = arm.flange_from_joints(st.joints_rad)
            pose = xyzrpy
        return {
            "ok": st.ok,
            "connected": True,
            "armed": st.armed,
            "endpoint": st.endpoint,
            "joints_rad": st.joints_rad,
            "pose_xyzrpy": pose,
            "error": st.error,
        }

    @app.post("/api/arm/jog_joint")
    def jog_joint(body: JogJointBody) -> dict:
        res = arm.jog_joint(body.joint, body.sign)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "result": res.value}

    @app.post("/api/arm/jog_cart")
    def jog_cart(body: JogCartBody) -> dict:
        res = arm.jog_cartesian(body.axis, body.sign)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "result": res.value}

    @app.post("/api/record/start")
    def record_start() -> dict:
        if camera.is_multi and live.enabled:
            live.set_enabled(False)
        res = record.start()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        if live.enabled and not camera.is_multi:
            live.reset()
        last_session_dir["path"] = Path(res.value)
        return {
            "ok": True,
            "session": str(res.value),
            "camera_count": camera.n_open,
            "serials": camera.open_serials,
            "live_disabled": camera.is_multi,
        }

    @app.post("/api/record/stop")
    def record_stop() -> dict:
        saved = None
        if live.last_result is not None and record.session_dir is not None:
            try:
                saved = live.persist(record.session_dir)
            except Exception:  # noqa: BLE001
                saved = None
        res = record.stop()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        last_session_dir["path"] = Path(res.value)
        return {
            "ok": True,
            "session": str(res.value),
            "live_extrinsics": str(saved) if saved else None,
        }

    @app.post("/api/session/report")
    def session_report(body: Optional[SessionReportBody] = None) -> dict:
        """Build session conclusion text from meta / joints / live_solves / extrinsics."""
        body = body or SessionReportBody()
        raw = (body.session_dir or "").strip()
        if raw:
            sess = Path(raw)
        else:
            sess = record.session_dir or last_session_dir["path"]
        if sess is None:
            raise HTTPException(400, "无 session：请先录制/走位，或传入 session_dir")
        res = build_session_report(sess)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        data = res.value
        last_session_dir["path"] = Path(data["session"])
        try:
            (Path(data["session"]) / "session_report.txt").write_text(
                data["report"] + "\n", encoding="utf-8"
            )
            data["report_path"] = str(Path(data["session"]) / "session_report.txt")
        except Exception:  # noqa: BLE001
            data["report_path"] = None
        return {"ok": True, **data}

    @app.post("/api/live")
    def set_live(enabled: bool = True) -> dict:
        if enabled and camera.is_multi:
            raise HTTPException(400, "多相机模式下不支持实时求解，请录制后离线计算")
        live.set_enabled(enabled)
        if enabled:
            live.reset()
        return {"ok": True, "live": live.enabled}

    @app.get("/api/live/status")
    def live_status() -> dict:
        st = live.last_status
        r = live.last_result
        return {
            "enabled": live.enabled,
            "n_samples": len(live.samples),
            "converged": st.converged if st else False,
            "delta_t_mm": st.delta_t_mm if st else None,
            "delta_R_deg": st.delta_R_deg if st else None,
            "stable_streak": st.stable_streak if st else 0,
            "warning": st.warning if st else None,
            "rejected_after_converge": live._rejected_since_lock,
            "saved_path": str(live._saved_path) if live._saved_path else None,
            "solve_idx": live._solve_idx,
            "history_path": str(live._history_path) if live._history_path else None,
            "method": r.method if r else None,
            "setup": getattr(r, "setup", None) if r else getattr(cfg.handeye, "setup", "eye_to_hand"),
            "dual_backends": bool(getattr(cfg.handeye, "dual_backends", False)),
            "dual": (r.metrics or {}).get("dual") if r and r.metrics else None,
            "T_base_cam": r.T_base_cam.tolist() if r else None,
            "T_flange_cam": r.T_flange_cam.tolist() if r and r.T_flange_cam is not None else None,
        }

    @app.post("/api/live/export")
    def live_export() -> dict:
        if live.last_result is None or record.session_dir is None:
            raise HTTPException(400, "无外参或 session")
        path = live.persist(record.session_dir)
        return {"ok": True, "path": str(path)}

    @app.post("/api/offline")
    def run_offline(body: OfflineBody) -> dict:
        session = Path(body.session_dir)
        try:
            meta = session_store.read_meta(session)
            cams = session_store.list_cameras_from_meta(meta)
        except Exception as e:  # noqa: BLE001
            raise HTTPException(400, f"无法读取 meta: {e}") from e
        if len(cams) > 1:
            bundle = offline.run_all(session)
            if not isinstance(bundle, Ok):
                raise HTTPException(400, str(bundle.error))
            data = bundle.value
            return {
                "ok": True,
                "multi_camera": True,
                "n_ok": data["n_ok"],
                "n_fail": data["n_fail"],
                "results": [
                    {
                        "camera_serial": one["camera_serial"],
                        "setup": getattr(one["result"], "setup", "eye_to_hand"),
                        "method": one["result"].method,
                        "n_samples": one["result"].n_samples,
                        "extrinsics_path": one["extrinsics_path"],
                        "warnings": one.get("warnings") or [],
                    }
                    for one in data["results"]
                ],
                "errors": data["errors"],
            }
        res = offline.run(session, camera_serial=cams[0]["serial"] if cams else None)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        data = res.value
        r = data["result"]
        out = {
            "ok": True,
            "multi_camera": False,
            "camera_serial": data.get("camera_serial"),
            "setup": getattr(r, "setup", "eye_to_hand"),
            "method": r.method,
            "n_samples": r.n_samples,
            "extrinsics_path": data["extrinsics_path"],
            "warnings": data.get("warnings"),
            "T_base_cam": r.T_base_cam.tolist(),
            "filter_report": data["report"].__dict__,
        }
        if getattr(r, "T_flange_cam", None) is not None:
            out["T_flange_cam"] = r.T_flange_cam.tolist()
        if getattr(r, "T_cam_flange", None) is not None:
            out["T_cam_flange"] = r.T_cam_flange.tolist()
        return out

    def _encode_preview_jpeg() -> bytes:
        with state_lock:
            cached = latest_frame.get("jpeg")
            if cached is not None:
                return cached
            img = latest_frame["img"]
            ts = latest_frame.get("ts")
        if img is None:
            blank = np.zeros((360, 640, 3), dtype=np.uint8)
            cv2.putText(
                blank,
                "No frame",
                (200, 180),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (180, 180, 180),
                2,
            )
            img = blank
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
        if not ok:
            return b""
        data = buf.tobytes()
        with state_lock:
            # Only cache if frame ts unchanged (avoid stale overwrite races)
            if latest_frame.get("ts") == ts or img is latest_frame.get("img"):
                latest_frame["jpeg"] = data
        return data

    @app.get("/api/preview.jpg")
    def preview_jpeg():
        from fastapi.responses import Response

        _ensure_grab()
        data = _encode_preview_jpeg()
        if not data:
            raise HTTPException(503, "encode failed")
        return Response(
            content=data,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

    @app.get("/api/stream.mjpeg")
    def stream_mjpeg() -> StreamingResponse:
        _ensure_grab()

        def gen():
            last_sent = None
            while True:
                data = _encode_preview_jpeg()
                if not data:
                    time.sleep(0.05)
                    continue
                # Skip duplicate payloads to reduce browser MJPEG buffer stalls
                if data is last_sent:
                    time.sleep(0.04)
                    continue
                last_sent = data
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
                )
                time.sleep(0.06)

        return StreamingResponse(
            gen(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    def _matrix_params_from_body(body: MatrixPlanBody) -> MatrixPathParams:
        base = cfg.matrix_path.to_dict()
        overrides = {k: v for k, v in body.model_dump().items() if v is not None and k != "seed_xyzrpy"}
        base.update(overrides)
        return MatrixPathParams.from_dict(base)

    @app.post("/api/matrix/plan")
    def matrix_plan(body: MatrixPlanBody) -> dict:
        params = _matrix_params_from_body(body)
        res = matrix.plan(params, seed_xyzrpy=body.seed_xyzrpy)
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        data = res.value
        matrix_state["waypoints"] = data["waypoints"]
        matrix_state["seed"] = data["seed_xyzrpy"]
        matrix_state["params"] = data["params"]
        matrix_state["last_error"] = None
        return {
            "ok": True,
            "seed_xyzrpy": data["seed_xyzrpy"],
            "summary": data["summary"],
            "params": data["params"],
            "waypoints": [
                {"index": w.index, "label": w.label, "xyzrpy": w.xyzrpy}
                for w in data["waypoints"]
            ],
        }

    @app.post("/api/matrix/start")
    def matrix_start() -> dict:
        # Recover stuck flags if prior worker died uncleanly
        t_old = matrix_state.get("thread")
        if matrix_state["running"] and t_old is not None and not t_old.is_alive():
            matrix_state["running"] = False
            matrix_state["thread"] = None
            matrix.reset_cancel()
        if matrix_state["running"] or matrix.running:
            raise HTTPException(400, "矩阵走位正在进行")
        wps = matrix_state.get("waypoints") or []
        if not wps:
            raise HTTPException(400, "请先调用 /api/matrix/plan 生成点位")
        if not arm.adapter.is_connected:
            raise HTTPException(400, "请先连接机械臂")
        if not arm.adapter.armed:
            raise HTTPException(400, "请先 Arm 使能")
        # Fresh session + live solve for this matrix run
        rec_info = _restart_record_for_matrix()
        last_session_dir["path"] = Path(rec_info["session"])
        params = matrix_state.get("params") or cfg.matrix_path.to_dict()
        wps_run = list(wps)
        matrix.reset_cancel()
        matrix_state["running"] = True
        matrix_state["last_error"] = None
        matrix_state["last_result"] = None
        matrix_state["stopped_for_converge"] = False
        matrix_state["session"] = rec_info["session"]
        matrix_state["progress"] = {"i": 0, "n": len(wps_run), "label": "", "phase": "starting"}

        def _live_converged() -> bool:
            st = live.last_status
            return bool(st is not None and st.converged)

        def _run() -> None:
            def cb(i, n, wp, phase):
                matrix_state["progress"] = {
                    "i": i,
                    "n": n,
                    "label": wp.label,
                    "phase": phase,
                }

            try:
                res = matrix.execute(
                    wps_run,
                    frame=str(params.get("frame", "flange")),
                    dwell_s=float(params.get("dwell_s", 0.8)),
                    progress_cb=cb,
                    extra_cancel=_live_converged,
                )
                if isinstance(res, Ok):
                    matrix_state["last_result"] = res.value
                    matrix_state["last_error"] = None
                else:
                    err = res.error
                    matrix_state["last_error"] = str(err)
                    matrix_state["last_result"] = None
                    if getattr(err, "code", None) == "PATH_CONVERGED" or matrix.cancel_reason == "converge":
                        matrix_state["stopped_for_converge"] = True
            except Exception as e:  # noqa: BLE001
                matrix_state["last_error"] = str(e)
                matrix_state["last_result"] = None
            finally:
                # Converge → stop path (already done) and stop recording.
                if matrix_state.get("stopped_for_converge") or _live_converged():
                    matrix_state["stopped_for_converge"] = True
                    stopped = _stop_record_after_matrix()
                    if stopped:
                        matrix_state["session"] = stopped
                matrix_state["running"] = False
                matrix_state["thread"] = None

        t = threading.Thread(target=_run, daemon=True, name="matrix-path")
        matrix_state["thread"] = t
        t.start()
        return {
            "ok": True,
            "n": len(wps_run),
            "session": rec_info["session"],
            "live": rec_info["live"],
            "recording_restarted": True,
            "camera_count": rec_info["camera_count"],
            "serials": rec_info["serials"],
        }

    @app.post("/api/matrix/stop")
    def matrix_stop() -> dict:
        matrix.request_cancel("user")
        return {"ok": True, "cancelling": True}

    @app.get("/api/matrix/status")
    def matrix_status() -> dict:
        return {
            "running": bool(matrix_state["running"]),
            "progress": matrix_state.get("progress"),
            "n_waypoints": len(matrix_state.get("waypoints") or []),
            "seed_xyzrpy": matrix_state.get("seed"),
            "last_error": matrix_state.get("last_error"),
            "last_result": matrix_state.get("last_result"),
            "stopped_for_converge": bool(matrix_state.get("stopped_for_converge")),
            "session": matrix_state.get("session"),
            "recording": record.recording,
        }

    return app


def main(host: str = "0.0.0.0", port: int = 6006) -> None:
    import os
    import threading
    import time
    import urllib.error
    import urllib.request
    import webbrowser

    import uvicorn

    browse_host = "127.0.0.1" if host in {"0.0.0.0", "::"} else host
    url = f"http://{browse_host}:{port}/"
    health = f"http://{browse_host}:{port}/api/health"
    no_browser = (os.environ.get("RS_CALIB_NO_BROWSER") or "").strip().lower() in {
        "1",
        "true",
        "yes",
    }

    def _open_browser_when_ready() -> None:
        if no_browser:
            return
        for _ in range(90):
            try:
                with urllib.request.urlopen(health, timeout=1.5) as resp:
                    if getattr(resp, "status", 200) == 200:
                        webbrowser.open(url)
                        print(f"[rs-calib] opened browser {url}", flush=True)
                        return
            except (urllib.error.URLError, TimeoutError, OSError):
                pass
            time.sleep(1.0)
        print(f"[rs-calib] health wait timeout; open manually: {url}", flush=True)

    threading.Thread(
        target=_open_browser_when_ready, name="rs-calib-open-browser", daemon=True
    ).start()

    uvicorn.run(
        "rs_calib.web.server:create_app",
        factory=True,
        host=host,
        port=port,
        reload=False,
        log_level="info",
    )


if __name__ == "__main__":
    main()
