from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import aruco_meta_dict, detect_target
from rs_calib.domain.math3d import se3_delta, xyzrpy_from_T
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
        self.base_marker_n: int = 0
        self.base_marker_spread_mm: float = float("nan")
        self.aruco_params: dict[str, Any] = {}
        self.load_info: dict[str, Any] = {}

    def peek_meta(self, session_dir: Path) -> Result[dict[str, Any]]:
        """Read session meta fields used to populate the verify UI (no video open)."""
        try:
            meta = session_store.read_meta(Path(session_dir))
            intr = session_store.intrinsics_from_meta(meta)
            he = meta.get("handeye") or {}
            ar = meta.get("aruco") or {}
            tfm = meta.get("T_flange_marker") or getattr(self.cfg.aruco, "T_flange_marker", None)
            tfm_xyzrpy_mm_deg = None
            if tfm is not None:
                try:
                    T = np.asarray(tfm, dtype=np.float64)
                    xyzrpy = xyzrpy_from_T(T)
                    tfm_xyzrpy_mm_deg = [
                        xyzrpy[0] * 1000.0,
                        xyzrpy[1] * 1000.0,
                        xyzrpy[2] * 1000.0,
                        float(np.rad2deg(xyzrpy[3])),
                        float(np.rad2deg(xyzrpy[4])),
                        float(np.rad2deg(xyzrpy[5])),
                    ]
                except Exception:  # noqa: BLE001
                    tfm = None
                    tfm_xyzrpy_mm_deg = None
            return Ok(
                {
                    "meta": meta,
                    "setup": str(he.get("setup") or "eye_to_hand"),
                    "aruco": {**aruco_meta_dict(self.cfg.aruco), **ar},
                    "intrinsics": {
                        "fx": intr.fx,
                        "fy": intr.fy,
                        "cx": intr.cx,
                        "cy": intr.cy,
                        "dist": list(intr.dist),
                        "width": intr.width,
                        "height": intr.height,
                        "fps": intr.fps,
                    },
                    "tcp_xyz": list(meta.get("tcp_xyz") or self.cfg.tcp_xyz),
                    "T_flange_marker": tfm,
                    "T_flange_marker_xyzrpy_mm_deg": tfm_xyzrpy_mm_deg,
                    "default_extrinsics": str(Path(session_dir) / "extrinsics.json"),
                }
            )
        except Exception as e:  # noqa: BLE001
            return Err(AppError("VERIFY_IO", "读取 session meta 失败", str(e)))

    def set_T_flange_marker(self, T: Optional[np.ndarray]) -> None:
        """Update mounting transform used only for absolute overlay / error (not re-solve)."""
        self.T_flange_marker = None if T is None else np.asarray(T, dtype=np.float64)

    def load(
        self,
        session_dir: Path,
        extrinsics_path: Path,
        *,
        setup_override: Optional[str] = None,
        aruco_override: Optional[dict[str, Any]] = None,
        intrinsics_override: Optional[Intrinsics] = None,
        T_flange_marker_override: Optional[np.ndarray] = None,
        T_flange_marker_override_set: bool = False,
    ) -> Result[int]:
        try:
            meta = session_store.read_meta(session_dir)
            T_base_cam, T_cam_base, ext = session_store.load_extrinsics(extrinsics_path)
            _ = T_base_cam
            samples = session_store.load_flange_samples(session_dir)
            self.samples_by_idx = {s.frame_idx: s for s in samples}

            if intrinsics_override is not None:
                self.intrinsics = intrinsics_override
                intr_src = "ui_override"
            else:
                self.intrinsics = session_store.intrinsics_from_meta(meta)
                intr_src = "meta.json"

            meta_setup = str(
                ext.get("setup") or (meta.get("handeye") or {}).get("setup") or "eye_to_hand"
            )
            if setup_override:
                self.setup = str(setup_override)
                setup_src = "ui_override"
            else:
                self.setup = meta_setup
                setup_src = "extrinsics/meta"

            self.T_cam_base = T_cam_base
            tcf = ext.get("T_cam_flange")
            tfc = ext.get("T_flange_cam")
            if tcf is not None:
                self.T_cam_flange = np.asarray(tcf, dtype=np.float64)
            elif tfc is not None:
                self.T_cam_flange = np.linalg.inv(np.asarray(tfc, dtype=np.float64))
            else:
                self.T_cam_flange = None

            self.tcp_xyz = list(meta.get("tcp_xyz") or self.cfg.tcp_xyz)
            if T_flange_marker_override_set:
                self.T_flange_marker = (
                    None
                    if T_flange_marker_override is None
                    else np.asarray(T_flange_marker_override, dtype=np.float64)
                )
                tfm_src = "ui_override"
            else:
                tfm = meta.get("T_flange_marker") or getattr(self.cfg.aruco, "T_flange_marker", None)
                self.T_flange_marker = np.asarray(tfm, dtype=np.float64) if tfm else None
                tfm_src = "meta/config" if tfm else "identity"

            # ArUco: defaults ← cfg, then meta, then explicit UI override (highest priority).
            merged = {**aruco_meta_dict(self.cfg.aruco), **(meta.get("aruco") or {})}
            if aruco_override:
                merged.update({k: v for k, v in aruco_override.items() if v is not None})
            self.aruco_params = merged
            aruco_src = "ui_override" if aruco_override else (
                "meta.json" if meta.get("aruco") else "app_config"
            )

            if self.reader is not None:
                self.reader.close()
            self.reader = SessionVideoReader(session_dir, meta)
            self.session_dir = session_dir
            self.frame_count = max(self.reader.frame_count, len(samples))
            self._meta = meta
            self.T_base_marker = None
            self.base_marker_n = 0
            self.base_marker_spread_mm = float("nan")

            if self.setup == "eye_in_hand":
                if self.T_cam_flange is None:
                    return Err(
                        AppError(
                            "VERIFY_EIH",
                            "眼在手上需要 extrinsics 中的 T_flange_cam / T_cam_flange",
                        )
                    )
                used = ext.get("used_frame_indices") or []
                self.T_base_marker = self._estimate_base_marker(samples, used_indices=used)
                if self.T_base_marker is None:
                    return Err(
                        AppError(
                            "VERIFY_EIH",
                            "眼在手上验证需要估计固定板位姿 T_base_marker，但未能从视频中检出足够标定板"
                            f"（当前板参数 board_type={merged.get('board_type')} "
                            f"marker_length_m={merged.get('marker_length_m')}）",
                        )
                    )

            assert self.intrinsics is not None
            self.load_info = {
                "setup": self.setup,
                "setup_source": setup_src,
                "meta_setup": meta_setup,
                "intrinsics_source": intr_src,
                "aruco_source": aruco_src,
                "T_flange_marker_source": tfm_src,
                "aruco": dict(self.aruco_params),
                "fx": self.intrinsics.fx,
                "fy": self.intrinsics.fy,
                "cx": self.intrinsics.cx,
                "cy": self.intrinsics.cy,
                "tcp_xyz": list(self.tcp_xyz),
                "extrinsics": str(extrinsics_path),
                "n_samples": len(samples),
                "frame_count": self.frame_count,
            }
            return Ok(self.frame_count)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("VERIFY_IO", "加载验证数据失败", str(e)))

    def _estimate_base_marker(
        self,
        samples: list,
        *,
        used_indices: Optional[list] = None,
        max_n: int = 24,
    ) -> Optional[np.ndarray]:
        assert self.reader is not None and self.intrinsics is not None and self.T_cam_flange is not None
        T_flange_cam = np.linalg.inv(self.T_cam_flange)
        aruco_params = self.aruco_params

        by_idx = {s.frame_idx: s for s in samples}
        ordered: list = []
        if used_indices:
            for fi in used_indices:
                s = by_idx.get(int(fi))
                if s is not None:
                    ordered.append(s)
        if len(ordered) < 8:
            step = max(1, len(samples) // max(40, max_n * 2))
            ordered = list(samples[::step])
        if len(ordered) > max_n * 3:
            step = max(1, len(ordered) // (max_n * 2))
            ordered = ordered[::step]

        acc: list[np.ndarray] = []
        for s in ordered:
            img = self.reader.read_index(s.frame_idx)
            if img is None:
                continue
            T_cm, _ = detect_target(img, self.intrinsics, aruco_params, draw=False)
            if T_cm is None:
                continue
            acc.append(s.T_base_flange @ T_flange_cam @ T_cm)
            if len(acc) >= max_n:
                break
        if len(acc) < 3:
            return None

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

        spreads = [se3_delta(T, Ti)[0] for Ti in acc]
        self.base_marker_n = len(acc)
        self.base_marker_spread_mm = float(np.median(spreads)) if spreads else float("nan")
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
        T_cm, _ = detect_target(img, self.intrinsics, self.aruco_params, draw=False)
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
