"""ConvergenceTracker: window diameter, not consecutive-only."""

from __future__ import annotations

import numpy as np

from rs_calib.domain.convergence import ConvergenceParams, ConvergenceTracker
from rs_calib.domain.types import ExtrinsicsResult


def _T_translate(x_m: float) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[0, 3] = float(x_m)
    return T


def _result(x_mm: float, n: int) -> ExtrinsicsResult:
    return ExtrinsicsResult(
        setup="eye_to_hand",
        method="test",
        n_samples=n,
        used_indices=list(range(n)),
        T_base_cam=_T_translate(x_mm / 1000.0),
        T_cam_base=_T_translate(-x_mm / 1000.0),
        metrics={},
    )


def test_slow_drift_does_not_converge():
    """Each step 1.2 mm — consecutive-only would pass; window diameter must not."""
    tr = ConvergenceTracker(
        ConvergenceParams(
            min_samples=12,
            max_delta_t_mm=1.5,
            max_delta_R_deg=0.5,
            stable_count=5,
        )
    )
    # 5 steps of 1.2 mm → window diameter ≈ 4.8 mm > 1.5
    for i in range(5):
        st = tr.update(_result(1.2 * i, n=12 + i))
    assert st.converged is False
    assert tr.locked_result is None
    assert st.delta_t_mm > 1.5


def test_tight_cluster_converges():
    tr = ConvergenceTracker(
        ConvergenceParams(
            min_samples=12,
            max_delta_t_mm=1.5,
            max_delta_R_deg=0.5,
            stable_count=5,
        )
    )
    # Tiny jitter around the same pose.
    xs = [0.0, 0.3, -0.2, 0.4, 0.1]
    st = None
    for i, x in enumerate(xs):
        st = tr.update(_result(x, n=12 + i))
    assert st is not None
    assert st.converged is True
    assert tr.locked_result is not None
    assert st.stable_streak >= 5
    assert st.delta_t_mm <= 1.5


def test_below_min_samples_never_converges():
    tr = ConvergenceTracker(
        ConvergenceParams(min_samples=12, max_delta_t_mm=1.5, max_delta_R_deg=0.5, stable_count=5)
    )
    for i in range(8):
        st = tr.update(_result(0.0, n=4 + i))  # n = 4..11 < 12
    assert st.converged is False
    assert st.warning == "样本不足"
    assert tr.locked_result is None
