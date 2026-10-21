from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from rs_calib.app_services.arm_service import ArmService
from rs_calib.app_services.camera_service import CameraService
from rs_calib.config.schema import AppConfig
from rs_calib.domain.types import FlangeSample
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra import session_store
from rs_calib.infra.video_io import SessionVideoWriter


class RecordService:
    def __init__(self, cfg: AppConfig, camera: CameraService, arm: ArmService) -> None:
        self.cfg = cfg
        self.camera = camera
        self.arm = arm
        self.recording = False
        self.session_dir: Optional[Path] = None
        self._writers: dict[str, SessionVideoWriter] = {}
        self._frame_idx = 0

    @property
    def camera_serials(self) -> list[str]:
        return list(self._writers.keys()) if self._writers else self.camera.open_serials

    def start(self) -> Result[Path]:
        if self.recording:
            return Err(AppError("RECORD_IO", "已在录制中"))
        serials = self.camera.open_serials
        if not serials:
            return Err(AppError("RECORD_IO", "请先打开至少一台相机"))
        for s in serials:
            if self.camera.current_intrinsics(s) is None:
                return Err(AppError("RECORD_IO", f"相机 {s} 无内参"))
        try:
            session = session_store.new_session_dir(self.cfg.app.session_root)
            writers: dict[str, SessionVideoWriter] = {}
            cam_entries = []
            multi = len(serials) > 1
            for serial in serials:
                intr = self.camera.current_intrinsics(serial)
                assert intr is not None
                if multi:
                    rel = f"cameras/{serial}"
                    media = session / "cameras" / serial
                else:
                    rel = ""
                    media = session
                writer = SessionVideoWriter(
                    media,
                    intr.width,
                    intr.height,
                    intr.fps,
                    fourcc=self.cfg.record.video_fourcc,
                    rel_path_prefix=rel,
                )
                writers[serial] = writer
                cam_entries.append(
                    session_store.camera_entry(
                        serial=serial,
                        name=self.camera.device_names.get(serial, ""),
                        intrinsics=intr,
                        video_info=writer.info,
                    )
                )
            meta = session_store.build_meta(
                cameras=cam_entries,
                robot_ip=self.cfg.arm.robot_ip,
                tcp_xyz=list(self.cfg.tcp_xyz),
                aruco={
                    "dict": self.cfg.aruco.dict,
                    "marker_id": self.cfg.aruco.marker_id,
                    "marker_length_m": self.cfg.aruco.marker_length_m,
                },
                incomplete=True,
                frame_count=0,
                T_flange_marker=self.cfg.aruco.T_flange_marker,
            )
            session_store.write_meta(session, meta)
            self.session_dir = session
            self._writers = writers
            self._frame_idx = 0
            self.recording = True
            return Ok(session)
        except Exception as e:  # noqa: BLE001
            for w in self._writers.values():
                try:
                    w.close()
                except Exception:  # noqa: BLE001
                    pass
            self._writers = {}
            return Err(AppError("RECORD_IO", "创建 session 失败", str(e)))

    def stop(self) -> Result[Path]:
        if not self.recording or self.session_dir is None:
            return Err(AppError("RECORD_IO", "当前未在录制"))
        for w in self._writers.values():
            w.close()
        try:
            meta = session_store.read_meta(self.session_dir)
            meta["incomplete"] = False
            meta["frame_count"] = self._frame_idx
            # refresh per-camera video info
            cams = []
            for serial, writer in self._writers.items():
                intr = self.camera.current_intrinsics(serial)
                if intr is None:
                    continue
                cams.append(
                    session_store.camera_entry(
                        serial=serial,
                        name=self.camera.device_names.get(serial, ""),
                        intrinsics=intr,
                        video_info=writer.info,
                    )
                )
            if cams:
                meta["cameras"] = cams
                meta["multi_camera"] = len(cams) > 1
                meta["schema_version"] = 2 if len(cams) > 1 else 1
                meta["camera"] = {
                    "serial": cams[0]["serial"],
                    "name": cams[0]["name"],
                    "width": cams[0]["width"],
                    "height": cams[0]["height"],
                    "fps": cams[0]["fps"],
                    "intrinsics": cams[0]["intrinsics"],
                }
                meta["video"] = cams[0]["video"]
            session_store.write_meta(self.session_dir, meta)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("RECORD_IO", "结束录制写 meta 失败", str(e)))
        path = self.session_dir
        self.recording = False
        self._writers = {}
        return Ok(path)

    def on_frame(self, ts_cam: float, image: np.ndarray) -> Optional[FlangeSample]:
        """Legacy single-image path (preview camera). Prefer on_frames for multi-cam."""
        serial = self.camera.preview_serial or (self.camera.open_serials[0] if self.camera.open_serials else None)
        if serial is None:
            return None
        return self.on_frames(ts_cam, {serial: image})

    def on_frames(self, ts_cam: float, images: dict[str, np.ndarray]) -> Optional[FlangeSample]:
        """Align shared arm snapshot with a multi-camera frame bundle."""
        st = self.arm.last_state
        if st is None or not st.ok or st.joints_rad is None:
            return None
        T, xyzrpy = self.arm.flange_from_joints(st.joints_rad)
        sample = FlangeSample(
            ts_cam=ts_cam,
            ts_arm=st.ts,
            frame_idx=self._frame_idx,
            joints_rad=list(st.joints_rad),
            T_base_flange=T,
            pose_xyzrpy=xyzrpy,
        )
        if self.recording and self._writers and self.session_dir is not None:
            # only count frame if we can write all expected camera streams
            missing = [s for s in self._writers if s not in images]
            if missing:
                return None
            for serial, writer in self._writers.items():
                writer.write(images[serial])
            session_store.append_flange_sample(self.session_dir, sample)
            self._frame_idx += 1
        else:
            self._frame_idx += 1
        return sample
