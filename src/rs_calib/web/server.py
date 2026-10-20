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
from rs_calib.app_services.offline_calib_service import OfflineCalibService
from rs_calib.app_services.record_service import RecordService
from rs_calib.config.store import load_config
from rs_calib.domain.aruco import detect_marker
from rs_calib.errors import Ok
from rs_calib.infra import session_store
from rs_calib.paths import resolve_frontend_dist
from rs_calib.version import __version__


class ConnectArmBody(BaseModel):
    robot_ip: Optional[str] = None
    dry_run: bool = False


class OpenCameraBody(BaseModel):
    serial: str
    width: Optional[int] = None
    height: Optional[int] = None
    fps: Optional[int] = None


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

    state_lock = threading.Lock()
    latest_frame: dict[str, Any] = {"img": None, "ts": 0.0}
    grab_thread: dict[str, Optional[threading.Thread]] = {"t": None, "stop": False}

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
            if not camera.adapter.is_open:
                time.sleep(0.05)
                continue
            grabbed = camera.adapter.grab(1000)
            if grabbed is None:
                continue
            ts, img = grabbed
            sample = record.on_frame(ts, img)
            show = img
            intr = camera.current_intrinsics()
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
                if sample is not None and live.enabled:
                    live.offer_frame(img, sample, intr)
            with state_lock:
                latest_frame["img"] = show
                latest_frame["ts"] = ts
            # light poll arm so record alignment works without separate worker
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
            "camera_open": camera.adapter.is_open,
            "arm_connected": arm.adapter.is_connected,
            "armed": arm.adapter.armed,
            "recording": record.recording,
            "live": live.enabled,
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
        res = camera.open(
            body.serial,
            body.width or cfg.camera.width,
            body.height or cfg.camera.height,
            body.fps or cfg.camera.fps,
        )
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        _ensure_grab()
        intr = res.value
        return {
            "ok": True,
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
        res = record.start()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        if live.enabled:
            live.reset()
        return {"ok": True, "session": str(res.value)}

    @app.post("/api/record/stop")
    def record_stop() -> dict:
        res = record.stop()
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        return {"ok": True, "session": str(res.value)}

    @app.post("/api/live")
    def set_live(enabled: bool = True) -> dict:
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
        res = offline.run(Path(body.session_dir))
        if not isinstance(res, Ok):
            raise HTTPException(400, str(res.error))
        data = res.value
        r = data["result"]
        return {
            "ok": True,
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
