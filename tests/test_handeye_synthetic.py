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


def test_handeye_recovers_T_base_cam():
    rng = np.random.default_rng(42)
    T_base_cam_gt = _rand_T(rng)
    T_flange_marker = _rand_T(rng)  # unknown fixed; should not matter
    T_cam_base = inv_T(T_base_cam_gt)

    samples: list[CalibSample] = []
    for i in range(20):
        T_base_flange = _rand_T(rng)
        T_cam_marker = T_cam_base @ T_base_flange @ T_flange_marker
        samples.append(
            CalibSample(
                frame_idx=i,
                T_base_flange=T_base_flange,
                T_cam_marker=T_cam_marker,
            )
        )

    result = solve_handeye(samples, method="TSAI", setup="eye_to_hand")
    assert result.setup == "eye_to_hand"
    dt, dR = se3_delta(result.T_base_cam, T_base_cam_gt)
    assert dt < 1.0, f"translation error {dt} mm"
    assert dR < 0.1, f"rotation error {dR} deg"


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
