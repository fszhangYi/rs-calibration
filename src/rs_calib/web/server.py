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
from rs_calib.config.store import load_config
from rs_calib.domain.aruco import detect_marker
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


class OfflineBody(BaseModel):
    session_dir: str


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

    state_lock = threading.Lock()
    latest_frame: dict[str, Any] = {"img": None, "ts": 0.0}
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
    }

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
        while not grab_thread["stop"]:
            if camera.n_open <= 0:
                time.sleep(0.05)
                continue
            grabbed = camera.grab_all(1000)
            if not grabbed:
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
            intr = camera.current_intrinsics(serial)
            if intr is not None:
                _, dbg = detect_marker(
                    img,
                    intr,
                    dict_name=cfg.aruco.dict,
                    marker_id=cfg.aruco.marker_id,
                    marker_length_m=cfg.aruco.marker_length_m,
                    draw=True,
                )
                if dbg is not None:
                    show = dbg
                # 多相机禁用实时求解
                if sample is not None and live.enabled and not camera.is_multi and len(images) == 1:
                    live.offer_frame(img, sample, intr)
            with state_lock:
                latest_frame["img"] = show
                latest_frame["ts"] = ts
            if arm.adapter.is_connected:
                arm.poll()

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
        }

    @app.get("/api/config")
    def get_config() -> dict:
        return {
            "tcp_xyz": cfg.tcp_xyz,
            "arm": {
                "robot_ip": cfg.arm.robot_ip,
                "cart_frame": cfg.arm.cart_frame,
                "jog_joint_step_deg": cfg.arm.jog_joint_step_deg,
                "jog_cart_step_m": cfg.arm.jog_cart_step_m,
                "jog_cart_step_deg": cfg.arm.jog_cart_step_deg,
            },
            "camera": {
                "width": cfg.camera.width,
                "height": cfg.camera.height,
                "fps": cfg.camera.fps,
            },
            "aruco": {
                "dict": cfg.aruco.dict,
                "marker_id": cfg.aruco.marker_id,
                "marker_length_m": cfg.aruco.marker_length_m,
            },
        }

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
        return {
            "ok": True,
            "jog_joint_step_deg": cfg.arm.jog_joint_step_deg,
            "jog_cart_step_m": cfg.arm.jog_cart_step_m,
            "jog_cart_step_deg": cfg.arm.jog_cart_step_deg,
            "cart_frame": cfg.arm.cart_frame,
        }

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
        return {
            "ok": True,
            "session": str(res.value),
            "camera_count": camera.n_open,
            "serials": camera.open_serials,
            "live_disabled": camera.is_multi,
        }

    @app.post("/api/record/stop")
    def record_stop() -> dict:
        res = record.stop()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "session": str(res.value)}

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
            "method": r.method if r else None,
            "T_base_cam": r.T_base_cam.tolist() if r else None,
        }

    @app.post("/api/live/export")
    def live_export() -> dict:
        if live.last_result is None or record.session_dir is None:
            raise HTTPException(400, "无外参或 session")
        path = session_store.save_extrinsics(record.session_dir, live.last_result)
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
        return {
            "ok": True,
            "multi_camera": False,
            "camera_serial": data.get("camera_serial"),
            "method": r.method,
            "n_samples": r.n_samples,
            "extrinsics_path": data["extrinsics_path"],
            "warnings": data.get("warnings"),
            "T_base_cam": r.T_base_cam.tolist(),
            "filter_report": data["report"].__dict__,
        }

    @app.get("/api/stream.mjpeg")
    def stream_mjpeg() -> StreamingResponse:
        _ensure_grab()

        def gen():
            while True:
                with state_lock:
                    img = latest_frame["img"]
                if img is None:
                    # placeholder
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
                ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if not ok:
                    time.sleep(0.05)
                    continue
                data = buf.tobytes()
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
                )
                time.sleep(0.05)

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
        if matrix_state["running"]:
            raise HTTPException(400, "矩阵走位正在进行")
        wps = matrix_state.get("waypoints") or []
        if not wps:
            raise HTTPException(400, "请先调用 /api/matrix/plan 生成点位")
        if not arm.adapter.is_connected:
            raise HTTPException(400, "请先连接机械臂")
        if not arm.adapter.armed:
            raise HTTPException(400, "请先 Arm 使能")
        params = matrix_state.get("params") or cfg.matrix_path.to_dict()
        matrix.reset_cancel()
        matrix_state["running"] = True
        matrix_state["last_error"] = None
        matrix_state["last_result"] = None
        matrix_state["progress"] = {"i": 0, "n": len(wps), "label": "", "phase": "starting"}

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
                    wps,
                    frame=str(params.get("frame", "flange")),
                    dwell_s=float(params.get("dwell_s", 0.8)),
                    progress_cb=cb,
                )
                if isinstance(res, Ok):
                    matrix_state["last_result"] = res.value
                else:
                    matrix_state["last_error"] = str(res.error)
            except Exception as e:  # noqa: BLE001
                matrix_state["last_error"] = str(e)
            finally:
                matrix_state["running"] = False
                matrix_state["thread"] = None

        t = threading.Thread(target=_run, daemon=True)
        matrix_state["thread"] = t
        t.start()
        return {"ok": True, "n": len(wps)}

    @app.post("/api/matrix/stop")
    def matrix_stop() -> dict:
        matrix.request_cancel()
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
        }

    return app


def main(host: str = "0.0.0.0", port: int = 6006) -> None:
    import uvicorn

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
