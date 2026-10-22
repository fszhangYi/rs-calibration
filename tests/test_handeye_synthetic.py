"""Synthetic eye-to-hand / eye-in-hand recovery tests (OpenCV input convention)."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from rs_calib.domain.handeye import solve_handeye
from rs_calib.domain.math3d import inv_T, se3_delta
from rs_calib.domain.types import CalibSample


def _rand_T(rng: np.random.Generator) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(rng.normal(size=3) * 0.5).as_matrix()
    T[:3, 3] = rng.uniform(-0.3, 0.3, size=3)
    return T


def _eth_samples(rng: np.random.Generator, n: int = 20) -> tuple[list[CalibSample], np.ndarray]:
    T_base_cam_gt = _rand_T(rng)
    T_flange_marker = _rand_T(rng)  # unknown fixed; should not matter
    T_cam_base = inv_T(T_base_cam_gt)
    samples: list[CalibSample] = []
    for i in range(n):
        T_base_flange = _rand_T(rng)
        T_cam_marker = T_cam_base @ T_base_flange @ T_flange_marker
        samples.append(
            CalibSample(
                frame_idx=i,
                T_base_flange=T_base_flange,
                T_cam_marker=T_cam_marker,
            )
        )
    return samples, T_base_cam_gt


def test_handeye_recovers_T_base_cam():
    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(42))
    result = solve_handeye(samples, method="TSAI", setup="eye_to_hand")
    assert result.setup == "eye_to_hand"
    dt, dR = se3_delta(result.T_base_cam, T_base_cam_gt)
    assert dt < 1.0, f"translation error {dt} mm"
    assert dR < 0.1, f"rotation error {dR} deg"


def test_handeye_park_recovers_T_base_cam():
    """Park had a swapped outer(a,b) bug; must recover GT on its own."""
    from rs_calib.domain.handeye import (
        _filter_pairs,
        _motion_pairs_axxb,
        _solve_ax_xb_park,
    )

    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(99), n=24)
    A, B = _motion_pairs_axxb(samples, setup="eye_to_hand")
    Af, Bf = _filter_pairs(A, B, min_rot_deg=0.5)
    X = _solve_ax_xb_park(Af, Bf)
    dt, dR = se3_delta(X, T_base_cam_gt)
    assert dt < 1.0, f"Park translation error {dt} mm"
    assert dR < 0.1, f"Park rotation error {dR} deg"


def test_handeye_tsai_recovers_T_base_cam():
    """Tsai must use OpenCV-style P=2q_xyz, not raw rotvec."""
    from rs_calib.domain.handeye import (
        _filter_pairs,
        _motion_pairs_axxb,
        _solve_ax_xb_tsai,
    )

    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(123), n=24)
    A, B = _motion_pairs_axxb(samples, setup="eye_to_hand")
    Af, Bf = _filter_pairs(A, B, min_rot_deg=0.5)
    X = _solve_ax_xb_tsai(Af, Bf)
    dt, dR = se3_delta(X, T_base_cam_gt)
    assert dt < 2.0, f"Tsai translation error {dt} mm"
    assert dR < 0.5, f"Tsai rotation error {dR} deg"


def test_handeye_native_backend_and_dual():
    from rs_calib.domain.handeye import opencv_handeye_available, solve_handeye

    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(3), n=20)
    native = solve_handeye(samples, method="PARK", setup="eye_to_hand", backend="native")
    assert native.metrics.get("backend") == "native"
    dt, dR = se3_delta(native.T_base_cam, T_base_cam_gt)
    assert dt < 1.0 and dR < 0.1

    dual = solve_handeye(samples, method="PARK", setup="eye_to_hand", dual_backends=True)
    assert "dual" in dual.metrics
    if opencv_handeye_available():
        assert dual.metrics["dual"].get("primary") == "opencv"
        assert "OPENCV" in dual.method or "DUAL" in dual.method
        dt2, dR2 = se3_delta(dual.T_base_cam, T_base_cam_gt)
        assert dt2 < 1.0 and dR2 < 0.1
    else:
        assert dual.metrics["dual"].get("primary") == "native"
        assert "DUAL_NO_CV" in dual.method


def test_handeye_recovers_T_flange_cam():
    rng = np.random.default_rng(7)
    T_flange_cam_gt = _rand_T(rng)
    T_base_marker = _rand_T(rng)  # board fixed in base
    T_cam_flange = inv_T(T_flange_cam_gt)

    samples: list[CalibSample] = []
    for i in range(24):
        T_base_flange = _rand_T(rng)
        # cam_marker = cam_flange · inv(base_flange) · base_marker
        T_cam_marker = T_cam_flange @ inv_T(T_base_flange) @ T_base_marker
        samples.append(
            CalibSample(
                frame_idx=i,
                T_base_flange=T_base_flange,
                T_cam_marker=T_cam_marker,
            )
        )

    result = solve_handeye(samples, method="TSAI", setup="eye_in_hand")
    assert result.setup == "eye_in_hand"
    assert result.T_flange_cam is not None
    dt, dR = se3_delta(result.T_flange_cam, T_flange_cam_gt)
    assert dt < 1.0, f"EIH translation error {dt} mm"
    assert dR < 0.1, f"EIH rotation error {dR} deg"
    dt2, dR2 = se3_delta(result.primary_T(), T_flange_cam_gt)
    assert dt2 < 1.0 and dR2 < 0.1


if __name__ == "__main__":
    test_handeye_recovers_T_base_cam()
    test_handeye_recovers_T_flange_cam()
    print("ok")
