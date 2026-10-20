from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

import numpy as np

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import detect_marker
from rs_calib.domain.filter_select import FilterParams, filter_and_select
from rs_calib.domain.handeye import available_methods, solve_handeye
from rs_calib.domain.math3d import se3_delta
from rs_calib.domain.types import CalibSample, ExtrinsicsResult, Intrinsics
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra import session_store
from rs_calib.infra.video_io import SessionVideoReader


class OfflineCalibService:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg

    def run(
        self,
        session_dir: Path,
        *,
        intrinsics_override: Optional[Intrinsics] = None,
        progress_cb=None,
    ) -> Result[dict[str, Any]]:
        session_dir = Path(session_dir)
        try:
            meta = session_store.read_meta(session_dir)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", "无法读取 meta.json", str(e)))

        intr = intrinsics_override or session_store.intrinsics_from_meta(meta)
        samples_flange = session_store.load_flange_samples(session_dir)
        if not samples_flange:
            return Err(AppError("CALIB_FAIL", "joints_poses.jsonl 为空"))

        by_idx = {s.frame_idx: s for s in samples_flange}
        reader = SessionVideoReader(session_dir, meta)
        detected: list[CalibSample] = []
        joints_map: dict[int, list[float]] = {}
        n = max(reader.frame_count, len(samples_flange))
        for i, fs in enumerate(samples_flange):
            if progress_cb:
                progress_cb(i + 1, len(samples_flange))
            img = reader.read_index(fs.frame_idx)
            if img is None:
                continue
            T_cm, _ = detect_marker(
                img,
                intr,
                dict_name=meta.get("aruco", {}).get("dict", self.cfg.aruco.dict),
                marker_id=int(meta.get("aruco", {}).get("marker_id", self.cfg.aruco.marker_id)),
                marker_length_m=float(
                    meta.get("aruco", {}).get("marker_length_m", self.cfg.aruco.marker_length_m)
                ),
            )
            if T_cm is None:
                continue
            detected.append(
                CalibSample(
                    frame_idx=fs.frame_idx,
                    T_base_flange=fs.T_base_flange,
                    T_cam_marker=T_cm,
                )
            )
            joints_map[fs.frame_idx] = list(fs.joints_rad)
        reader.close()

        fp = FilterParams(
            dup_joint_eps_deg=self.cfg.convergence.dup_joint_eps_deg,
            dup_pose_eps_mm=self.cfg.convergence.dup_pose_eps_mm,
            dup_pose_eps_deg=self.cfg.convergence.dup_pose_eps_deg,
            outlier_jump_mm=self.cfg.convergence.outlier_jump_mm,
            outlier_jump_deg=self.cfg.convergence.outlier_jump_deg,
            min_translation_span_m=self.cfg.convergence.min_translation_span_m,
            min_rotation_span_deg=self.cfg.convergence.min_rotation_span_deg,
            min_samples=self.cfg.convergence.min_samples,
        )
        selected, report = filter_and_select(detected, joints_by_idx=joints_map, params=fp)
        if len(selected) < 3:
            return Err(AppError("CALIB_FAIL", f"有效样本不足: {len(selected)}", str(report)))

        try:
            result = solve_handeye(selected, method=self.cfg.handeye.method)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", "HandEye 求解失败", str(e)))

        warnings = list(report.warnings)
        if self.cfg.handeye.compare_methods:
            for m in available_methods():
                if m == result.method:
                    continue
                try:
                    other = solve_handeye(selected, method=m)
                    dt, dR = se3_delta(result.T_base_cam, other.T_base_cam)
                    if dt > 5.0 or dR > 2.0:
                        warnings.append(f"{m} 与主方法偏差 Δt={dt:.1f}mm ΔR={dR:.2f}°")
                except Exception:  # noqa: BLE001
                    pass

        out_path = session_store.save_extrinsics(
            session_dir,
            result,
            extra={"filter_report": report.__dict__, "warnings": warnings},
        )
        return Ok(
            {
                "result": result,
                "report": report,
                "warnings": warnings,
                "extrinsics_path": str(out_path),
            }
        )
