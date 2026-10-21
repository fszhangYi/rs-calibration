from __future__ import annotations

from typing import Literal, Sequence

import numpy as np

from rs_calib.domain.math3d import inv_T
from rs_calib.domain.types import CalibSample, ExtrinsicsResult

SetupName = Literal["eye_to_hand", "eye_in_hand"]

_METHOD_CONSTANTS = {
    "TSAI": "CALIB_HAND_EYE_TSAI",
    "PARK": "CALIB_HAND_EYE_PARK",
    "HORAUD": "CALIB_HAND_EYE_HORAUD",
    "ANDREFF": "CALIB_HAND_EYE_ANDREFF",
    "DANIILIDIS": "CALIB_HAND_EYE_DANIILIDIS",
}

_SETUPS = ("eye_to_hand", "eye_in_hand")


def available_methods() -> list[str]:
    return list(_METHOD_CONSTANTS.keys())


def available_setups() -> list[str]:
    return list(_SETUPS)


def normalize_setup(setup: str) -> SetupName:
    key = (setup or "eye_to_hand").strip().lower().replace("-", "_")
    aliases = {
        "eth": "eye_to_hand",
        "eih": "eye_in_hand",
        "eye_to_hand": "eye_to_hand",
        "eye_in_hand": "eye_in_hand",
        "eyetohand": "eye_to_hand",
        "eyeinhand": "eye_in_hand",
    }
    out = aliases.get(key, key)
    if out not in _SETUPS:
        raise ValueError(f"unknown handeye setup: {setup}")
    return out  # type: ignore[return-value]


def _solve_ax_xb_svd(
    T_A: list[np.ndarray],
    T_B: list[np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    """Solve AX=XB for relative motions via Kronecker SVD + LS translation.

    Kronecker form uses column-major ``vec``; reshape must use ``order='F'``.
    """
    mats = []
    for Ta, Tb in zip(T_A, T_B):
        Ra, Rb = Ta[:3, :3], Tb[:3, :3]
        mats.append(np.kron(np.eye(3), Ra) - np.kron(Rb.T, np.eye(3)))
    M = np.vstack(mats)
    _, _, Vt = np.linalg.svd(M)
    rx = Vt[-1].reshape(3, 3, order="F")
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


def _left_rel(T_list: list[np.ndarray]) -> list[np.ndarray]:
    """A_i = T_{i+1} @ inv(T_i)."""
    return [T_list[i + 1] @ inv_T(T_list[i]) for i in range(len(T_list) - 1)]


def _right_rel(T_list: list[np.ndarray]) -> list[np.ndarray]:
    """A_i = inv(T_i) @ T_{i+1}."""
    return [inv_T(T_list[i]) @ T_list[i + 1] for i in range(len(T_list) - 1)]


def _pack_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _solve_eye_to_hand_fallback(samples: Sequence[CalibSample]) -> np.ndarray:
    """Recover T_base_cam without OpenCV.

    From G_i @ T_flange_marker = X @ C_i (X = T_base_cam):
      A_i = G_{i+1} @ inv(G_i)
      B_i = C_{i+1} @ inv(C_i)
      A X = X B
    """
    G = [s.T_base_flange.astype(np.float64) for s in samples]
    C = [s.T_cam_marker.astype(np.float64) for s in samples]
    Rx, tx = _solve_ax_xb_svd(_left_rel(G), _left_rel(C))
    return _pack_T(Rx, tx)


def _solve_eye_in_hand_fallback(samples: Sequence[CalibSample]) -> np.ndarray:
    """Recover T_flange_cam without OpenCV.

    From G_i @ X @ C_i = T_base_marker (X = T_flange_cam):
      A_i = inv(G_i) @ G_{i+1}
      B_i = C_i @ inv(C_{i+1})
      A X = X B
    """
    G = [s.T_base_flange.astype(np.float64) for s in samples]
    C = [s.T_cam_marker.astype(np.float64) for s in samples]
    B = [C[i] @ inv_T(C[i + 1]) for i in range(len(C) - 1)]
    Rx, tx = _solve_ax_xb_svd(_right_rel(G), B)
    return _pack_T(Rx, tx)


def _opencv_calibrate(
    samples: Sequence[CalibSample],
    *,
    setup: SetupName,
    method_key: str,
) -> np.ndarray | None:
    try:
        import cv2
    except Exception:  # noqa: BLE001
        return None
    if not hasattr(cv2, "calibrateHandEye"):
        return None

    R_gripper2base: list[np.ndarray] = []
    t_gripper2base: list[np.ndarray] = []
    R_target2cam: list[np.ndarray] = []
    t_target2cam: list[np.ndarray] = []
    for s in samples:
        if setup == "eye_to_hand":
            # OpenCV ETH: feed inv(T_base_flange) as gripper2base → returns cam2base
            T_g2b = inv_T(s.T_base_flange)
        else:
            # OpenCV EIH: feed T_base_flange as gripper2base → returns cam2gripper
            T_g2b = s.T_base_flange
        R_gripper2base.append(T_g2b[:3, :3].astype(np.float64))
        t_gripper2base.append(T_g2b[:3, 3].reshape(3, 1).astype(np.float64))
        R_target2cam.append(s.T_cam_marker[:3, :3].astype(np.float64))
        t_target2cam.append(s.T_cam_marker[:3, 3].reshape(3, 1).astype(np.float64))

    flag = getattr(cv2, _METHOD_CONSTANTS[method_key])
    try:
        R_out, t_out = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=flag,
        )
    except Exception:  # noqa: BLE001
        return None
    return _pack_T(R_out, t_out)


def solve_handeye(
    samples: Sequence[CalibSample],
    method: str = "TSAI",
    *,
    setup: str = "eye_to_hand",
) -> ExtrinsicsResult:
    """Solve hand-eye extrinsic for the selected setup.

    - ``eye_to_hand`` → ``T_base_cam`` (scene camera fixed in base)
    - ``eye_in_hand`` → ``T_flange_cam`` (wrist camera fixed on flange)

    Prefer ``cv2.calibrateHandEye``; fall back to SVD AX=XB if OpenCV lacks the API.
    """
    if len(samples) < 3:
        raise ValueError(f"need at least 3 samples, got {len(samples)}")
    key = method.upper()
    if key not in _METHOD_CONSTANTS:
        raise ValueError(f"unknown handeye method: {method}")
    setup_n = normalize_setup(setup)

    indices = [int(s.frame_idx) for s in samples]
    T_solved = _opencv_calibrate(samples, setup=setup_n, method_key=key)
    used = key
    if T_solved is None:
        if setup_n == "eye_to_hand":
            T_solved = _solve_eye_to_hand_fallback(samples)
        else:
            T_solved = _solve_eye_in_hand_fallback(samples)
        used = f"{key}_FALLBACK_SVD"

    if setup_n == "eye_to_hand":
        T_base_cam = T_solved
        T_cam_base = inv_T(T_base_cam)
        return ExtrinsicsResult(
            T_base_cam=T_base_cam,
            T_cam_base=T_cam_base,
            method=used,
            n_samples=len(samples),
            used_indices=indices,
            metrics={},
            setup=setup_n,
            T_flange_cam=None,
            T_cam_flange=None,
        )

    T_flange_cam = T_solved
    T_cam_flange = inv_T(T_flange_cam)
    # Placeholder base↔cam (pose varies with arm); primary extrinsic is flange↔cam.
    eye = np.eye(4, dtype=np.float64)
    return ExtrinsicsResult(
        T_base_cam=eye.copy(),
        T_cam_base=eye.copy(),
        method=used,
        n_samples=len(samples),
        used_indices=indices,
        metrics={"note": "eye_in_hand: primary is T_flange_cam"},
        setup=setup_n,
        T_flange_cam=T_flange_cam,
        T_cam_flange=T_cam_flange,
    )
