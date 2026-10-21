from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from rs_calib.domain.math3d import T_flange_tcp, draw_axes, inv_T, se3_delta
from rs_calib.domain.types import Intrinsics, VerifyFrameMetrics


def marker_pred_T_eth(
    T_cam_base: np.ndarray,
    T_base_flange: np.ndarray,
    T_flange_marker: Optional[np.ndarray] = None,
) -> np.ndarray:
    T_fm = np.eye(4) if T_flange_marker is None else T_flange_marker
    return T_cam_base @ T_base_flange @ T_fm


def marker_pred_T_eih(
    T_cam_flange: np.ndarray,
    T_base_flange: np.ndarray,
    T_base_marker: np.ndarray,
) -> np.ndarray:
    """Eye-in-hand: board fixed in base → cam_marker = cam_flange · inv(base_flange) · base_marker."""
    return T_cam_flange @ inv_T(T_base_flange) @ T_base_marker


# backward-compatible alias
def marker_pred_T(
    T_cam_base: np.ndarray,
    T_base_flange: np.ndarray,
    T_flange_marker: Optional[np.ndarray] = None,
) -> np.ndarray:
    return marker_pred_T_eth(T_cam_base, T_base_flange, T_flange_marker)


def tcp_A(
    T_cam_base: np.ndarray,
    T_base_flange: np.ndarray,
    tcp_xyz: Sequence[float],
) -> np.ndarray:
    return T_cam_base @ T_base_flange @ T_flange_tcp(tcp_xyz)


def tcp_A_eih(T_cam_flange: np.ndarray, tcp_xyz: Sequence[float]) -> np.ndarray:
    return T_cam_flange @ T_flange_tcp(tcp_xyz)


def tcp_B(
    T_cam_marker: np.ndarray,
    tcp_xyz: Sequence[float],
    T_flange_marker: Optional[np.ndarray] = None,
) -> np.ndarray:
    T_fm = np.eye(4) if T_flange_marker is None else np.asarray(T_flange_marker)
    return T_cam_marker @ inv_T(T_fm) @ T_flange_tcp(tcp_xyz)


def overlay_verify_frame(
    image_bgr: np.ndarray,
    intrinsics: Intrinsics,
    *,
    T_cam_base: Optional[np.ndarray],
    T_base_flange: np.ndarray,
    T_cam_marker: Optional[np.ndarray],
    tcp_xyz: Sequence[float],
    T_flange_marker: Optional[np.ndarray] = None,
    axis_length_m: float = 0.05,
    show_tcp_axes: bool = True,
    frame_idx: int = 0,
    setup: str = "eye_to_hand",
    T_cam_flange: Optional[np.ndarray] = None,
    T_base_marker: Optional[np.ndarray] = None,
) -> tuple[np.ndarray, VerifyFrameMetrics]:
    out = image_bgr.copy()
    K = intrinsics.K()
    dist = intrinsics.dist_coeffs()
    is_eih = setup == "eye_in_hand"

    if show_tcp_axes:
        if is_eih and T_cam_flange is not None:
            Ta = tcp_A_eih(T_cam_flange, tcp_xyz)
        elif T_cam_base is not None:
            Ta = tcp_A(T_cam_base, T_base_flange, tcp_xyz)
        else:
            Ta = None
        if Ta is not None:
            draw_axes(
                out,
                K,
                dist,
                Ta,
                axis_length_m,
                colors_bgr=((0, 0, 255), (0, 255, 0), (255, 0, 0)),
                thickness=3,
            )

    has = T_cam_marker is not None
    err_t = err_r = float("nan")
    if has:
        assert T_cam_marker is not None
        T_pred = None
        if is_eih and T_cam_flange is not None and T_base_marker is not None:
            T_pred = marker_pred_T_eih(T_cam_flange, T_base_flange, T_base_marker)
        elif T_cam_base is not None:
            T_pred = marker_pred_T_eth(T_cam_base, T_base_flange, T_flange_marker)
        if T_pred is not None:
            err_t, err_r = se3_delta(T_pred, T_cam_marker)
        if show_tcp_axes and not is_eih:
            Tb = tcp_B(T_cam_marker, tcp_xyz, T_flange_marker)
            draw_axes(
                out,
                K,
                dist,
                Tb,
                axis_length_m,
                colors_bgr=((255, 255, 0), (255, 255, 0), (255, 255, 0)),
                thickness=2,
            )
        elif show_tcp_axes and is_eih and T_pred is not None:
            draw_axes(
                out,
                K,
                dist,
                T_pred,
                axis_length_m,
                colors_bgr=((255, 255, 0), (255, 255, 0), (255, 255, 0)),
                thickness=2,
            )

    metrics = VerifyFrameMetrics(
        frame_idx=frame_idx,
        marker_err_t_mm=err_t,
        marker_err_R_deg=err_r,
        has_detect=has,
    )
    return out, metrics
