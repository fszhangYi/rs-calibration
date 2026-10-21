from __future__ import annotations

from pathlib import Path
from typing import Callable, Optional

import numpy as np

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import detect_target
from rs_calib.domain.convergence import ConvergenceParams, ConvergenceTracker
from rs_calib.domain.filter_select import FilterParams, is_near_duplicate
from rs_calib.domain.handeye import solve_handeye
from rs_calib.domain.math3d import se3_delta
from rs_calib.domain.types import CalibSample, ConvergenceStatus, ExtrinsicsResult, FlangeSample, Intrinsics
from rs_calib.infra import session_store


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
        self._saved_path: Optional[Path] = None
        self._history_path: Optional[Path] = None
        self._solve_idx = 0
        self._rejected_since_lock = 0

    def reset(self) -> None:
        self.samples.clear()
        self._joints.clear()
        self._last_kept = None
        self.tracker.reset()
        self.last_result = None
        self.last_status = None
        self._since_recompute = 0
        self._saved_path = None
        self._history_path = None
        self._solve_idx = 0
        self._rejected_since_lock = 0

    def set_enabled(self, enabled: bool) -> None:
        self.enabled = enabled
        if not enabled:
            return

    def persist_latest(self, session_dir: Optional[Path]) -> Optional[Path]:
        """Write current last_result as session ``extrinsics.json`` (latest snapshot)."""
        if self.last_result is None or session_dir is None:
            return None
        extra = {
            "source": "live",
            "solve_idx": self._solve_idx,
            "rejected_after_converge": self._rejected_since_lock,
            "history": "live_solves.jsonl",
        }
        if self.last_status is not None:
            extra["convergence"] = {
                "converged": self.last_status.converged,
                "delta_t_mm": self.last_status.delta_t_mm,
                "delta_R_deg": self.last_status.delta_R_deg,
                "stable_streak": self.last_status.stable_streak,
                "n_valid": self.last_status.n_valid,
                "warning": self.last_status.warning,
            }
        path = session_store.save_extrinsics(session_dir, self.last_result, extra=extra)
        self._saved_path = path
        return path

    # Back-compat alias used by UI / web
    def persist(self, session_dir: Optional[Path]) -> Optional[Path]:
        return self.persist_latest(session_dir)

    def _save_solve(
        self,
        session_dir: Optional[Path],
        result: ExtrinsicsResult,
        status: ConvergenceStatus,
        *,
        basis_samples: list[CalibSample],
        basis_joints: dict[int, list[float]],
        rejected: bool = False,
    ) -> None:
        """Persist one solve: full basis samples + result (+ optional rejection note)."""
        if session_dir is None:
            return
        self._solve_idx += 1
        extra = {
            "rejected_sample": rejected,
            "rejected_after_converge": self._rejected_since_lock,
        }
        try:
            self._history_path = session_store.append_live_solve(
                session_dir,
                solve_idx=self._solve_idx,
                samples=basis_samples,
                joints_by_idx=basis_joints,
                result=result,
                status=status,
                extra=extra,
            )
            # Refresh latest snapshot for accepted solves (offline/verify pick-up).
            if not rejected:
                self.persist_latest(session_dir)
        except Exception:  # noqa: BLE001
            pass

    def offer_frame(
        self,
        image: np.ndarray,
        sample: FlangeSample,
        intrinsics: Intrinsics,
        on_result: Optional[Callable[[ExtrinsicsResult, ConvergenceStatus], None]] = None,
        *,
        session_dir: Optional[Path] = None,
    ) -> None:
        if not self.enabled:
            return
        T_cm, _ = detect_target(image, intrinsics, self.cfg.aruco, draw=False)
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

        prev_kept = self._last_kept
        self.samples.append(cand)
        self._joints[sample.frame_idx] = list(sample.joints_rad)
        self._last_kept = cand
        self._since_recompute += 1
        n = self.cfg.convergence.recompute_every_n
        if len(self.samples) < 3 or self._since_recompute < n:
            return
        self._since_recompute = 0
        try:
            result = solve_handeye(
                self.samples,
                method=self.cfg.handeye.method,
                setup=getattr(self.cfg.handeye, "setup", "eye_to_hand"),
            )
        except Exception:  # noqa: BLE001
            return

        # Snapshot the exact basis that produced this result (before any reject/pop).
        basis_samples = list(self.samples)
        basis_joints = {k: list(v) for k, v in self._joints.items()}

        # Already converged: drop samples that would pull the estimate off the lock.
        locked = self.tracker.locked_result
        if locked is not None and self.tracker.would_destabilize(result):
            self.samples.pop()
            self._joints.pop(sample.frame_idx, None)
            self._last_kept = prev_kept
            self._rejected_since_lock += 1
            dt, dR = se3_delta(locked.primary_T(), result.primary_T())
            status = ConvergenceStatus(
                n_valid=locked.n_samples,
                delta_t_mm=dt,
                delta_R_deg=dR,
                stable_streak=self.tracker.params.stable_count,
                converged=True,
                warning=f"已舍弃破坏收敛的样本 (累计{self._rejected_since_lock})",
            )
            self.last_result = locked
            self.last_status = status
            # Audit: keep the would-be result + the samples that produced it.
            self._save_solve(
                session_dir,
                result,
                status,
                basis_samples=basis_samples,
                basis_joints=basis_joints,
                rejected=True,
            )
            if on_result is not None:
                on_result(locked, status)
            return

        status = self.tracker.update(result)
        result.metrics["convergence"] = {
            "delta_t_mm": status.delta_t_mm,
            "delta_R_deg": status.delta_R_deg,
            "converged": status.converged,
            "stable_streak": status.stable_streak,
            "rejected_after_converge": self._rejected_since_lock,
        }
        self.last_result = result
        self.last_status = status
        self._save_solve(
            session_dir,
            result,
            status,
            basis_samples=basis_samples,
            basis_joints=basis_joints,
            rejected=False,
        )
        if on_result is not None:
            on_result(result, status)
