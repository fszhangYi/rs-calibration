from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import detect_marker
from rs_calib.domain.types import Intrinsics, VerifyFrameMetrics
from rs_calib.domain.verify_geom import overlay_verify_frame
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra import session_store
from rs_calib.infra.video_io import SessionVideoReader


class VerifyService:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.session_dir: Optional[Path] = None
        self.reader: Optional[SessionVideoReader] = None
        self.samples_by_idx: dict[int, object] = {}
        self.intrinsics: Optional[Intrinsics] = None
        self.T_cam_base: Optional[np.ndarray] = None
        self.T_flange_marker: Optional[np.ndarray] = None
        self.tcp_xyz = list(cfg.tcp_xyz)
        self.frame_count = 0

    def load(
        self,
        session_dir: Path,
        extrinsics_path: Path,
        *,
        intrinsics_override: Optional[Intrinsics] = None,
    ) -> Result[int]:
        try:
            meta = session_store.read_meta(session_dir)
            T_base_cam, T_cam_base, _ = session_store.load_extrinsics(extrinsics_path)
            _ = T_base_cam
            samples = session_store.load_flange_samples(session_dir)
            self.samples_by_idx = {s.frame_idx: s for s in samples}
            self.intrinsics = intrinsics_override or session_store.intrinsics_from_meta(meta)
            self.T_cam_base = T_cam_base
            self.tcp_xyz = list(meta.get("tcp_xyz") or self.cfg.tcp_xyz)
            tfm = meta.get("T_flange_marker")
            self.T_flange_marker = np.asarray(tfm, dtype=np.float64) if tfm else None
            self.reader = SessionVideoReader(session_dir, meta)
            self.session_dir = session_dir
            self.frame_count = max(self.reader.frame_count, len(samples))
            self._meta = meta
            return Ok(self.frame_count)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("VERIFY_IO", "加载验证数据失败", str(e)))

    def render(self, frame_idx: int) -> tuple[Optional[np.ndarray], Optional[VerifyFrameMetrics]]:
        if self.reader is None or self.intrinsics is None or self.T_cam_base is None:
            return None, None
        img = self.reader.read_index(frame_idx)
        sample = self.samples_by_idx.get(frame_idx)
        if img is None or sample is None:
            return img, None
        meta = getattr(self, "_meta", {})
        T_cm, _ = detect_marker(
            img,
            self.intrinsics,
            dict_name=meta.get("aruco", {}).get("dict", self.cfg.aruco.dict),
            marker_id=int(meta.get("aruco", {}).get("marker_id", self.cfg.aruco.marker_id)),
            marker_length_m=float(
                meta.get("aruco", {}).get("marker_length_m", self.cfg.aruco.marker_length_m)
            ),
        )
        out, metrics = overlay_verify_frame(
            img,
            self.intrinsics,
            T_cam_base=self.T_cam_base,
            T_base_flange=sample.T_base_flange,  # type: ignore[attr-defined]
            T_cam_marker=T_cm,
            tcp_xyz=self.tcp_xyz,
            T_flange_marker=self.T_flange_marker,
            axis_length_m=self.cfg.verify.axis_length_m,
            show_tcp_axes=self.cfg.verify.show_tcp_axes,
            frame_idx=frame_idx,
        )
        return out, metrics

    def close(self) -> None:
        if self.reader is not None:
            self.reader.close()
            self.reader = None
