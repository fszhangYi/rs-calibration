from __future__ import annotations

from typing import Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R


def inv_T(T: np.ndarray) -> np.ndarray:
    T = np.asarray(T, dtype=np.float64)
    out = np.eye(4, dtype=np.float64)
    Rm = T[:3, :3]
    t = T[:3, 3]
    out[:3, :3] = Rm.T
    out[:3, 3] = -Rm.T @ t
    return out


def T_from_xyzrpy(xyzrpy: Sequence[float]) -> np.ndarray:
    x, y, z, rx, ry, rz = [float(v) for v in xyzrpy[:6]]
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R.from_euler("xyz", [rx, ry, rz]).as_matrix()
    T[:3, 3] = [x, y, z]
    return T


def xyzrpy_from_T(T: np.ndarray) -> list[float]:
    T = np.asarray(T, dtype=np.float64)
    rpy = R.from_matrix(T[:3, :3]).as_euler("xyz").tolist()
    return [float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), *rpy]


def T_flange_tcp(tcp_xyz: Sequence[float]) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, 3] = [float(tcp_xyz[0]), float(tcp_xyz[1]), float(tcp_xyz[2])]
    return T


def rotation_angle_deg(Ra: np.ndarray, Rb: np.ndarray) -> float:
    Rrel = np.asarray(Ra, dtype=np.float64).T @ np.asarray(Rb, dtype=np.float64)
    # clip for numerical safety
    cos_theta = (np.trace(Rrel) - 1.0) * 0.5
    cos_theta = float(np.clip(cos_theta, -1.0, 1.0))
    return float(np.degrees(np.arccos(cos_theta)))


def se3_delta(Ta: np.ndarray, Tb: np.ndarray) -> tuple[float, float]:
    """Return (translation_mm, rotation_deg) between two poses."""
    Ta = np.asarray(Ta, dtype=np.float64)
    Tb = np.asarray(Tb, dtype=np.float64)
    t_mm = float(np.linalg.norm(Ta[:3, 3] - Tb[:3, 3]) * 1000.0)
    r_deg = rotation_angle_deg(Ta[:3, :3], Tb[:3, :3])
    return t_mm, r_deg


def project_axes(
    K: np.ndarray,
    dist: np.ndarray,
    T_cam_obj: np.ndarray,
    axis_len: float = 0.05,
) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int], tuple[int, int]]:
    """Project object-frame axes into image pixels (origin, x, y, z)."""
    import cv2

    pts = np.float32(
        [
            [0, 0, 0],
            [axis_len, 0, 0],
            [0, axis_len, 0],
            [0, 0, axis_len],
        ]
    )
    rvec, _ = cv2.Rodrigues(T_cam_obj[:3, :3])
    tvec = T_cam_obj[:3, 3].reshape(3, 1)
    img_pts, _ = cv2.projectPoints(pts, rvec, tvec, K, dist)
    img_pts = img_pts.reshape(-1, 2)
    out = []
    for i in range(4):
        out.append((int(round(img_pts[i, 0])), int(round(img_pts[i, 1]))))
    return out[0], out[1], out[2], out[3]  # type: ignore[return-value]


def draw_axes(
    image: np.ndarray,
    K: np.ndarray,
    dist: np.ndarray,
    T_cam_obj: np.ndarray,
    axis_len: float,
    colors_bgr: tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]],
    thickness: int = 2,
) -> np.ndarray:
    import cv2

    origin, x, y, z = project_axes(K, dist, T_cam_obj, axis_len)
    cv2.line(image, origin, x, colors_bgr[0], thickness)
    cv2.line(image, origin, y, colors_bgr[1], thickness)
    cv2.line(image, origin, z, colors_bgr[2], thickness)
    return image
