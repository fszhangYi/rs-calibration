from __future__ import annotations

from typing import Optional

import cv2
import numpy as np

from rs_calib.domain.types import Intrinsics


def _get_dict(name: str):
    key = name if name.startswith("DICT_") else f"DICT_{name}"
    if not hasattr(cv2.aruco, key):
        raise ValueError(f"unknown ArUco dict: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, key))


def detect_marker(
    image_bgr: np.ndarray,
    intrinsics: Intrinsics,
    *,
    dict_name: str = "DICT_5X5_50",
    marker_id: int = 0,
    marker_length_m: float = 0.15,
    draw: bool = False,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Detect configured ArUco marker.

    Returns:
        (T_cam_marker 4x4 or None, debug_image or None)
    """
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    dictionary = _get_dict(dict_name)
    params = cv2.aruco.DetectorParameters()
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    debug = image_bgr.copy() if draw else None
    if ids is None or len(ids) == 0:
        return None, debug

    ids_flat = ids.flatten().tolist()
    if marker_id not in ids_flat:
        if draw and debug is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        return None, debug

    idx = ids_flat.index(marker_id)
    K = intrinsics.K()
    dist = intrinsics.dist_coeffs()
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners[idx], float(marker_length_m), K, dist
    )
    rvec = rvecs[0, 0]
    tvec = tvecs[0, 0]
    Rm, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = tvec.reshape(3)

    if draw and debug is not None:
        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        cv2.drawFrameAxes(debug, K, dist, rvec, tvec, float(marker_length_m) * 0.5)

    return T, debug
