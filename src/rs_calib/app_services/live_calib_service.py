from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import detect_marker
from rs_calib.domain.convergence import ConvergenceParams, ConvergenceTracker
from rs_calib.domain.filter_select import FilterParams, is_near_duplicate
from rs_calib.domain.handeye import solve_handeye
from rs_calib.domain.types import CalibSample, ConvergenceStatus, ExtrinsicsResult, FlangeSample, Intrinsics


class LiveCalibService:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.enabled = False
        self.samples: list[CalibSample] = []
        self._joints: dict[int, list[float]] = {}
        self._last_kept: Optional[CalibSample] = None
        self.tracker = ConvergenceTracker(
            ConvergenceParams(
                min_samples=cfg.convergence.min_samples,
                max_delta_t_mm=cfg.convergence.max_delta_t_mm,
                max_delta_R_deg=cfg.convergence.max_delta_R_deg,
                stable_count=cfg.convergence.stable_count,
            )
        )
        self.last_result: Optional[ExtrinsicsResult] = None
        self.last_status: Optional[ConvergenceStatus] = None
        self._since_recompute = 0

    def reset(self) -> None:
        self.samples.clear()
        self._joints.clear()
        self._last_kept = None
        self.tracker.reset()
        self.last_result = None
        self.last_status = None
        self._since_recompute = 0

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        if not enabled:
            return

    def offer_frame(
        self,
        image: np.ndarray,
        sample: FlangeSample,
        intrinsics: Intrinsics,
        on_result: Optional[Callable[[ExtrinsicsResult, ConvergenceStatus], None]] = None,
    ) -> None:
        if not self.enabled:
            return
        T_cm, _ = detect_marker(
            image,
            intrinsics,
            dict_name=self.cfg.aruco.dict,
            marker_id=self.cfg.aruco.marker_id,
            marker_length_m=self.cfg.aruco.marker_length_m,
            draw=False,
        )
        if T_cm is None:
            return
        cand = CalibSample(
            frame_idx=sample.frame_idx,
            T_base_flange=sample.T_base_flange.copy(),
            T_cam_marker=T_cm,
        )
        fp = FilterParams(
            dup_joint_eps_deg=self.cfg.convergence.dup_joint_eps_deg,
            dup_pose_eps_mm=self.cfg.convergence.dup_pose_eps_mm,
            dup_pose_eps_deg=self.cfg.convergence.dup_pose_eps_deg,
        )
        if self._last_kept is not None:
            ja = self._joints.get(self._last_kept.frame_idx)
            jb = sample.joints_rad
            if is_near_duplicate(self._last_kept, cand, joints_a=ja, joints_b=jb, params=fp):
                return
        self.samples.append(cand)
        self._joints[sample.frame_idx] = list(sample.joints_rad)
        self._last_kept = cand
        self._since_recompute += 1
        n = self.cfg.convergence.recompute_every_n
        if len(self.samples) >= 3 and self._since_recompute >= n:
            self._since_recompute = 0
            try:
                result = solve_handeye(
                    self.samples,
                    method=self.cfg.handeye.method,
                    setup=getattr(self.cfg.handeye, "setup", "eye_to_hand"),
                )
                status = self.tracker.update(result)
                result.metrics["convergence"] = {
                    "delta_t_mm": status.delta_t_mm,
                    "delta_R_deg": status.delta_R_deg,
                    "converged": status.converged,
                    "stable_streak": status.stable_streak,
                }
                self.last_result = result
                self.last_status = status
                if on_result is not None:
                    on_result(result, status)
            except Exception:  # noqa: BLE001
                pass
