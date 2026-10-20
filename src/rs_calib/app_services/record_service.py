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
        self._writer: Optional[SessionVideoWriter] = None
        self._frame_idx = 0

    def start(self) -> Result[Path]:
        if self.recording:
            return Err(AppError("RECORD_IO", "已在录制中"))
        intr = self.camera.current_intrinsics()
        if intr is None or not self.camera.adapter.is_open:
            return Err(AppError("RECORD_IO", "请先打开相机"))
        try:
            session = session_store.new_session_dir(self.cfg.app.session_root)
            self._writer = SessionVideoWriter(
                session,
                intr.width,
                intr.height,
                intr.fps,
                fourcc=self.cfg.record.video_fourcc,
            )
            meta = session_store.build_meta(
                camera_serial=self.camera.selected_serial or "",
                camera_name=self.camera.device_name,
                intrinsics=intr,
                robot_ip=self.cfg.arm.robot_ip,
                tcp_xyz=list(self.cfg.tcp_xyz),
                aruco={
                    "dict": self.cfg.aruco.dict,
                    "marker_id": self.cfg.aruco.marker_id,
                    "marker_length_m": self.cfg.aruco.marker_length_m,
                },
                video_info=self._writer.info,
                incomplete=True,
                frame_count=0,
                T_flange_marker=self.cfg.aruco.T_flange_marker,
            )
            session_store.write_meta(session, meta)
            self.session_dir = session
            self._frame_idx = 0
            self.recording = True
            return Ok(session)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("RECORD_IO", "创建 session 失败", str(e)))

    def stop(self) -> Result[Path]:
        if not self.recording or self.session_dir is None:
            return Err(AppError("RECORD_IO", "当前未在录制"))
        if self._writer is not None:
            self._writer.close()
        # rewrite meta complete
        try:
            meta = session_store.read_meta(self.session_dir)
            meta["incomplete"] = False
            meta["frame_count"] = self._frame_idx
            if self._writer is not None:
                meta["video"] = self._writer.info
            session_store.write_meta(self.session_dir, meta)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("RECORD_IO", "结束录制写 meta 失败", str(e)))
        path = self.session_dir
        self.recording = False
        self._writer = None
        return Ok(path)

    def on_frame(self, ts_cam: float, image: np.ndarray) -> Optional[FlangeSample]:
        """Align camera frame with latest arm snapshot; optionally record."""
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
        if self.recording and self._writer is not None and self.session_dir is not None:
            self._writer.write(image)
            session_store.append_flange_sample(self.session_dir, sample)
            self._frame_idx += 1
        else:
            # preview-only: still advance idx so live-calib samples stay unique
            self._frame_idx += 1
        return sample
