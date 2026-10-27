from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

from rs_calib.domain.math3d import se3_delta
from rs_calib.domain.types import ConvergenceStatus, ExtrinsicsResult


@dataclass
class ConvergenceParams:
    min_samples: int = 12
    max_delta_t_mm: float = 1.5
    max_delta_R_deg: float = 0.5
    stable_count: int = 5


def _window_diameter_mm_deg(results: Sequence[ExtrinsicsResult]) -> tuple[float, float]:
    """Max pairwise SE(3) delta inside ``results`` (mm, deg)."""
    if len(results) < 2:
        return 0.0, 0.0
    Ts = [r.primary_T() for r in results]
    max_dt = 0.0
    max_dR = 0.0
    for i in range(len(Ts)):
        for j in range(i + 1, len(Ts)):
            dt, dR = se3_delta(Ts[i], Ts[j])
            if dt > max_dt:
                max_dt = float(dt)
            if dR > max_dR:
                max_dR = float(dR)
    return max_dt, max_dR


class ConvergenceTracker:
    """Online extrinsic stability.

    Convergence is **not** “last step vs previous step”. A slow drift of 1.4 mm
    per solve would still look stable under consecutive-only checks.

    Instead we keep a short history of accepted solves and require the last
    ``stable_count`` results to form a tight cluster: the **pairwise diameter**
    (max Δt / ΔR among them) must stay within ``max_delta_*``. That is equivalent
    to asking the newest solve to agree with the recent record window, not only
    with its immediate predecessor.

    Early session solves (before ``min_samples``) are ignored for streak so a
    bad first estimate does not poison the criterion forever.
    """

    def __init__(self, params: Optional[ConvergenceParams] = None):
        self.params = params or ConvergenceParams()
        self._history: list[ExtrinsicsResult] = []
        self._streak = 0
        self._locked: Optional[ExtrinsicsResult] = None

    @property
    def locked(self) -> bool:
        return self._locked is not None

    @property
    def locked_result(self) -> Optional[ExtrinsicsResult]:
        return self._locked

    def reset(self) -> None:
        self._history = []
        self._streak = 0
        self._locked = None

    def would_destabilize(self, result: ExtrinsicsResult) -> bool:
        """True if already converged and ``result`` drifts past thresholds vs lock."""
        if self._locked is None:
            return False
        dt, dR = se3_delta(self._locked.primary_T(), result.primary_T())
        return dt > self.params.max_delta_t_mm or dR > self.params.max_delta_R_deg

    def _trailing_cluster_len(self) -> int:
        """Largest ``k`` such that diameter(history[-k:]) ≤ thresholds."""
        hist = self._history
        if not hist:
            return 0
        thr_t = self.params.max_delta_t_mm
        thr_r = self.params.max_delta_R_deg
        # Cap search: no need beyond a few× stable_count for UI streak.
        k_max = min(len(hist), max(self.params.stable_count * 3, self.params.stable_count))
        best = 1
        for k in range(2, k_max + 1):
            dt, dR = _window_diameter_mm_deg(hist[-k:])
            if dt <= thr_t and dR <= thr_r:
                best = k
            else:
                # Larger windows only grow diameter → stop early.
                break
        return best

    def update(self, result: ExtrinsicsResult) -> ConvergenceStatus:
        n = result.n_samples
        warning = None
        if n < self.params.min_samples:
            # Do not accumulate pre-threshold noise into the stability window.
            self._history = [result]
            self._streak = 0
            return ConvergenceStatus(
                n_valid=n,
                delta_t_mm=0.0,
                delta_R_deg=0.0,
                stable_streak=0,
                converged=False,
                warning="样本不足",
            )

        self._history.append(result)
        keep = max(self.params.stable_count * 4, 24)
        if len(self._history) > keep:
            self._history = self._history[-keep:]

        self._streak = self._trailing_cluster_len()
        # Status deltas: diameter of the candidate window (last stable_count, or all if fewer).
        win_n = min(len(self._history), self.params.stable_count)
        dt, dR = _window_diameter_mm_deg(self._history[-win_n:])

        converged = (
            self._streak >= self.params.stable_count
            and n >= self.params.min_samples
            and dt <= self.params.max_delta_t_mm
            and dR <= self.params.max_delta_R_deg
        )
        if converged and self._locked is None:
            self._locked = result
        return ConvergenceStatus(
            n_valid=n,
            delta_t_mm=dt,
            delta_R_deg=dR,
            stable_streak=min(self._streak, self.params.stable_count),
            converged=converged or self._locked is not None,
            warning=warning,
        )
