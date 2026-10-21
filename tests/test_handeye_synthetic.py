"""Synthetic eye-to-hand recovery test (nails OpenCV input convention)."""

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

    result = solve_handeye(samples, method="TSAI")
    dt, dR = se3_delta(result.T_base_cam, T_base_cam_gt)
    assert dt < 1.0, f"translation error {dt} mm"
    assert dR < 0.1, f"rotation error {dR} deg"
