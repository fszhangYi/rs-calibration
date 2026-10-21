from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from rs_calib.domain.math3d import se3_delta
from rs_calib.domain.types import ConvergenceStatus, ExtrinsicsResult


@dataclass
class ConvergenceParams:
    min_samples: int = 12
    max_delta_t_mm: float = 1.5
    max_delta_R_deg: float = 0.5
    stable_count: int = 5


class ConvergenceTracker:
    def __init__(self, params: Optional[ConvergenceParams] = None):
        self.params = params or ConvergenceParams()
        self._prev: Optional[ExtrinsicsResult] = None
        self._streak = 0

    def reset(self) -> None:
        self._prev = None
        self._streak = 0

    def update(self, result: ExtrinsicsResult) -> ConvergenceStatus:
        n = result.n_samples
        warning = None
        if self._prev is None or n < self.params.min_samples:
            self._prev = result
            self._streak = 0
            return ConvergenceStatus(
                n_valid=n,
                delta_t_mm=0.0,
                delta_R_deg=0.0,
                stable_streak=0,
                converged=False,
                warning="样本不足" if n < self.params.min_samples else None,
            )

        dt, dR = se3_delta(self._prev.primary_T(), result.primary_T())
        if dt <= self.params.max_delta_t_mm and dR <= self.params.max_delta_R_deg:
            self._streak += 1
        else:
            self._streak = 0
        self._prev = result
        converged = self._streak >= self.params.stable_count and n >= self.params.min_samples
        return ConvergenceStatus(
            n_valid=n,
            delta_t_mm=dt,
            delta_R_deg=dR,
            stable_streak=self._streak,
            converged=converged,
            warning=warning,
        )
