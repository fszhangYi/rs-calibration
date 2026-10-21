from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import aruco_meta_dict, detect_target
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
        self.T_cam_flange: Optional[np.ndarray] = None
        self.T_base_marker: Optional[np.ndarray] = None
        self.T_flange_marker: Optional[np.ndarray] = None
        self.setup: str = "eye_to_hand"
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
            T_base_cam, T_cam_base, ext = session_store.load_extrinsics(extrinsics_path)
            _ = T_base_cam
            samples = session_store.load_flange_samples(session_dir)
            self.samples_by_idx = {s.frame_idx: s for s in samples}
            self.intrinsics = intrinsics_override or session_store.intrinsics_from_meta(meta)
            self.T_cam_base = T_cam_base
            self.setup = str(ext.get("setup") or meta.get("handeye", {}).get("setup") or "eye_to_hand")
            tcf = ext.get("T_cam_flange")
            tfc = ext.get("T_flange_cam")
            if tcf is not None:
                self.T_cam_flange = np.asarray(tcf, dtype=np.float64)
            elif tfc is not None:
                self.T_cam_flange = np.linalg.inv(np.asarray(tfc, dtype=np.float64))
            else:
                self.T_cam_flange = None
            self.tcp_xyz = list(meta.get("tcp_xyz") or self.cfg.tcp_xyz)
            tfm = meta.get("T_flange_marker")
            self.T_flange_marker = np.asarray(tfm, dtype=np.float64) if tfm else None
            self.reader = SessionVideoReader(session_dir, meta)
            self.session_dir = session_dir
            self.frame_count = max(self.reader.frame_count, len(samples))
            self._meta = meta
            self.T_base_marker = None
            if self.setup == "eye_in_hand" and self.T_cam_flange is not None and self.intrinsics is not None:
                self.T_base_marker = self._estimate_base_marker(samples)
            return Ok(self.frame_count)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("VERIFY_IO", "加载验证数据失败", str(e)))

    def _estimate_base_marker(self, samples: list) -> Optional[np.ndarray]:
        """Average T_base_marker = T_base_flange @ T_flange_cam @ T_cam_marker over detections."""
        assert self.reader is not None and self.intrinsics is not None and self.T_cam_flange is not None
        meta = getattr(self, "_meta", {})
        T_flange_cam = np.linalg.inv(self.T_cam_flange)
        acc: list[np.ndarray] = []
        for s in samples[:40]:
            img = self.reader.read_index(s.frame_idx)
            if img is None:
                continue
            aruco_params = {**aruco_meta_dict(self.cfg.aruco), **(meta.get("aruco") or {})}
            T_cm, _ = detect_target(img, self.intrinsics, aruco_params, draw=False)
            if T_cm is None:
                continue
            acc.append(s.T_base_flange @ T_flange_cam @ T_cm)
            if len(acc) >= 8:
                break
        if not acc:
            return None
        # naive average of translation + SVD mean rotation
        t = np.mean([T[:3, 3] for T in acc], axis=0)
        R_sum = sum(T[:3, :3] for T in acc)
        U, _, Vt = np.linalg.svd(R_sum)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            U[:, -1] *= -1
            R = U @ Vt
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = t
        return T

    def render(self, frame_idx: int) -> tuple[Optional[np.ndarray], Optional[VerifyFrameMetrics]]:
        ready = self.reader is not None and self.intrinsics is not None
        if not ready:
            return None, None
        if self.setup == "eye_to_hand" and self.T_cam_base is None:
            return None, None
        if self.setup == "eye_in_hand" and self.T_cam_flange is None:
            return None, None
        img = self.reader.read_index(frame_idx)
        sample = self.samples_by_idx.get(frame_idx)
        if img is None or sample is None:
            return img, None
        meta = getattr(self, "_meta", {})
        aruco_params = {**aruco_meta_dict(self.cfg.aruco), **(meta.get("aruco") or {})}
        T_cm, _ = detect_target(img, self.intrinsics, aruco_params, draw=False)
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
            setup=self.setup,
            T_cam_flange=self.T_cam_flange,
            T_base_marker=self.T_base_marker,
        )
        return out, metrics

    def close(self) -> None:
        if self.reader is not None:
            self.reader.close()
            self.reader = None
