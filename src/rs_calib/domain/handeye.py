from __future__ import annotations

from typing import Sequence

import numpy as np

from rs_calib.domain.math3d import inv_T
from rs_calib.domain.types import CalibSample, ExtrinsicsResult

_METHOD_CONSTANTS = {
    "TSAI": "CALIB_HAND_EYE_TSAI",
    "PARK": "CALIB_HAND_EYE_PARK",
    "HORAUD": "CALIB_HAND_EYE_HORAUD",
    "ANDREFF": "CALIB_HAND_EYE_ANDREFF",
    "DANIILIDIS": "CALIB_HAND_EYE_DANIILIDIS",
}


def available_methods() -> list[str]:
    return list(_METHOD_CONSTANTS.keys())


def _solve_ax_xb_svd(
    T_A: list[np.ndarray],
    T_B: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Solve AX=XB for relative motions via Kronecker SVD + LS translation."""
    mats = []
    for Ta, Tb in zip(T_A, T_B):
        Ra, Rb = Ta[:3, :3], Tb[:3, :3]
        mats.append(np.kron(np.eye(3), Ra) - np.kron(Rb.T, np.eye(3)))
    M = np.vstack(mats)
    _, _, Vt = np.linalg.svd(M)
    rx = Vt[-1].reshape(3, 3)
    U, _, Vt2 = np.linalg.svd(rx)
    Rx = U @ Vt2
    if np.linalg.det(Rx) < 0:
        Rx = U @ np.diag([1.0, 1.0, -1.0]) @ Vt2

    At, bt = [], []
    for Ta, Tb in zip(T_A, T_B):
        Ra, ta = Ta[:3, :3], Ta[:3, 3]
        tb = Tb[:3, 3]
        At.append(np.eye(3) - Ra)
        bt.append(ta - Rx @ tb)
    tx, *_ = np.linalg.lstsq(np.vstack(At), np.concatenate(bt), rcond=None)
    return Rx, tx.reshape(3)


def _absolute_to_relative(T_list: list[np.ndarray]) -> list[np.ndarray]:
    rel = []
    for i in range(len(T_list) - 1):
        # A_i = T_{i}^{-1} T_{i+1}
        rel.append(inv_T(T_list[i]) @ T_list[i + 1])
    return rel


def _solve_eye_to_hand_fallback(samples: Sequence[CalibSample]) -> np.ndarray:
    """Recover T_base_cam without OpenCV.

    Eye-to-hand AX=XB with:
      A_i = inv(G_i) @ G_{i+1}   (G = T_base_flange)
      B_i = C_i @ inv(C_{i+1})   (C = T_cam_marker)
      X   = T_base_cam
    """
    G = [s.T_base_flange.astype(np.float64) for s in samples]
    C = [s.T_cam_marker.astype(np.float64) for s in samples]
    A = _absolute_to_relative(G)
    B = []
    for i in range(len(C) - 1):
        B.append(C[i] @ inv_T(C[i + 1]))
    Rx, tx = _solve_ax_xb_svd(A, B)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rx
    T[:3, 3] = tx
    return T


def solve_handeye(
    samples: Sequence[CalibSample],
    method: str = "TSAI",
) -> ExtrinsicsResult:
    """Solve eye-to-hand extrinsic ``T_base_cam``.

    Prefer ``cv2.calibrateHandEye`` (eye-to-hand input convention: feed
    ``inv(T_base_flange)`` as gripper2base). Fall back to SVD AX=XB if OpenCV
    build lacks the API (e.g. some OpenCV 5 wheels).

    Does **not** use tcp_xyz or T_flange_marker.
    """
    if len(samples) < 3:
        raise ValueError(f"need at least 3 samples, got {len(samples)}")
    key = method.upper()
    if key not in _METHOD_CONSTANTS:
        raise ValueError(f"unknown handeye method: {method}")

    indices = [int(s.frame_idx) for s in samples]
    T_base_cam: np.ndarray | None = None
    used = key

    try:
        import cv2

        if hasattr(cv2, "calibrateHandEye"):
            R_gripper2base: list[np.ndarray] = []
            t_gripper2base: list[np.ndarray] = []
            R_target2cam: list[np.ndarray] = []
            t_target2cam: list[np.ndarray] = []
            for s in samples:
                T_base2flange = inv_T(s.T_base_flange)
                R_gripper2base.append(T_base2flange[:3, :3].astype(np.float64))
                t_gripper2base.append(T_base2flange[:3, 3].reshape(3, 1).astype(np.float64))
                R_target2cam.append(s.T_cam_marker[:3, :3].astype(np.float64))
                t_target2cam.append(s.T_cam_marker[:3, 3].reshape(3, 1).astype(np.float64))
            flag = getattr(cv2, _METHOD_CONSTANTS[key])
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_gripper2base,
                t_gripper2base,
                R_target2cam,
                t_target2cam,
                method=flag,
            )
            T_base_cam = np.eye(4, dtype=np.float64)
            T_base_cam[:3, :3] = R_cam2base
            T_base_cam[:3, 3] = t_cam2base.reshape(3)
    except Exception:  # noqa: BLE001
        T_base_cam = None

    if T_base_cam is None:
        T_base_cam = _solve_eye_to_hand_fallback(samples)
        used = f"{key}_FALLBACK_SVD"

    T_cam_base = inv_T(T_base_cam)
    return ExtrinsicsResult(
        T_base_cam=T_base_cam,
        T_cam_base=T_cam_base,
        method=used,
        n_samples=len(samples),
        used_indices=indices,
        metrics={},
    )
