from __future__ import annotations

from typing import Optional, Sequence

import numpy as np

from rs_calib.domain.math3d import draw_axes, inv_T, se3_delta, T_flange_tcp
from rs_calib.domain.types import Intrinsics, VerifyFrameMetrics


def marker_pred_T(
    T_cam_base: np.ndarray,
    T_base_flange: np.ndarray,
    T_flange_marker: Optional[np.ndarray] = None,
) -> np.ndarray:
    T_fm = np.eye(4) if T_flange_marker is None else T_flange_marker
    return T_cam_base @ T_base_flange @ T_fm


def tcp_A(
    T_cam_base: np.ndarray,
    T_base_flange: np.ndarray,
    tcp_xyz: Sequence[float],
) -> np.ndarray:
    return T_cam_base @ T_base_flange @ T_flange_tcp(tcp_xyz)


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
    T_cam_base: np.ndarray,
    T_base_flange: np.ndarray,
    T_cam_marker: Optional[np.ndarray],
    tcp_xyz: Sequence[float],
    T_flange_marker: Optional[np.ndarray] = None,
    axis_length_m: float = 0.05,
    show_tcp_axes: bool = True,
    frame_idx: int = 0,
) -> tuple[np.ndarray, VerifyFrameMetrics]:
    out = image_bgr.copy()
    K = intrinsics.K()
    dist = intrinsics.dist_coeffs()

    # Color A: robot TCP via extrinsic (RGB axes)
    if show_tcp_axes:
        Ta = tcp_A(T_cam_base, T_base_flange, tcp_xyz)
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
        T_pred = marker_pred_T(T_cam_base, T_base_flange, T_flange_marker)
        err_t, err_r = se3_delta(T_pred, T_cam_marker)
        # Color B: vision TCP (cyan-ish)
        if show_tcp_axes:
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

    metrics = VerifyFrameMetrics(
        frame_idx=frame_idx,
        marker_err_t_mm=err_t,
        marker_err_R_deg=err_r,
        has_detect=has,
    )
    return out, metrics
