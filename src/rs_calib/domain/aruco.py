from __future__ import annotations

from typing import Any, Optional

import cv2
import numpy as np

from rs_calib.domain.types import Intrinsics


def _get_dict(name: str):
    key = name if name.startswith("DICT_") else f"DICT_{name}"
    if not hasattr(cv2.aruco, key):
        raise ValueError(f"unknown ArUco dict: {name}")
    return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, key))


def _marker_object_points(marker_length_m: float) -> np.ndarray:
    """ArUco corner order matches OpenCV detectMarkers (TL, TR, BR, BL), Z=0 plane."""
    half = float(marker_length_m) * 0.5
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def _estimate_pose(
    corners: np.ndarray,
    marker_length_m: float,
    K: np.ndarray,
    dist: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return (rvec 3,), (tvec 3,) for one marker. Works on OpenCV 4 and 5."""
    if hasattr(cv2.aruco, "estimatePoseSingleMarkers"):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, float(marker_length_m), K, dist
        )
        return rvecs[0, 0].reshape(3), tvecs[0, 0].reshape(3)

    img_pts = np.asarray(corners, dtype=np.float64).reshape(-1, 2)
    obj_pts = _marker_object_points(marker_length_m)
    ok, rvec, tvec = cv2.solvePnP(
        obj_pts,
        img_pts,
        K,
        dist,
        flags=cv2.SOLVEPNP_IPPE_SQUARE if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE") else cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed for ArUco marker")
    return np.asarray(rvec, dtype=np.float64).reshape(3), np.asarray(tvec, dtype=np.float64).reshape(3)


def _rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    Rm, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64).reshape(3))
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return T


def detect_marker(
    image_bgr: np.ndarray,
    intrinsics: Intrinsics,
    *,
    dict_name: str = "DICT_5X5_50",
    marker_id: int = 0,
    marker_length_m: float = 0.15,
    draw: bool = False,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Detect configured single ArUco marker.

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
    try:
        rvec, tvec = _estimate_pose(corners[idx], float(marker_length_m), K, dist)
    except Exception:  # noqa: BLE001
        if draw and debug is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        return None, debug

    T = _rvec_tvec_to_T(rvec, tvec)

    if draw and debug is not None:
        cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        if hasattr(cv2, "drawFrameAxes"):
            cv2.drawFrameAxes(debug, K, dist, rvec, tvec, float(marker_length_m) * 0.5)

    return T, debug


def _make_charuco_board(
    *,
    dict_name: str,
    squares_x: int,
    squares_y: int,
    square_length_m: float,
    marker_length_m: float,
):
    dictionary = _get_dict(dict_name)
    size = (int(squares_x), int(squares_y))
    return cv2.aruco.CharucoBoard(
        size,
        float(square_length_m),
        float(marker_length_m),
        dictionary,
    )


def detect_charuco(
    image_bgr: np.ndarray,
    intrinsics: Intrinsics,
    *,
    dict_name: str = "DICT_5X5_50",
    squares_x: int = 5,
    squares_y: int = 7,
    square_length_m: float = 0.04,
    marker_length_m: float = 0.03,
    min_corners: int = 6,
    draw: bool = False,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Detect ChArUco board pose (T_cam_board). OpenCV 4/5."""
    if not hasattr(cv2.aruco, "CharucoBoard"):
        raise RuntimeError("当前 OpenCV 无 CharucoBoard，请升级 opencv-contrib-python")

    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    board = _make_charuco_board(
        dict_name=dict_name,
        squares_x=squares_x,
        squares_y=squares_y,
        square_length_m=square_length_m,
        marker_length_m=marker_length_m,
    )
    debug = image_bgr.copy() if draw else None
    K = intrinsics.K()
    dist = intrinsics.dist_coeffs()

    charuco_corners = None
    charuco_ids = None
    marker_corners = None
    marker_ids = None

    if hasattr(cv2.aruco, "CharucoDetector"):
        detector = cv2.aruco.CharucoDetector(board)
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(gray)
    else:
        # OpenCV 4.x fallback
        dictionary = board.getDictionary() if hasattr(board, "getDictionary") else _get_dict(dict_name)
        params = cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "ArucoDetector"):
            det = cv2.aruco.ArucoDetector(dictionary, params)
            marker_corners, marker_ids, _ = det.detectMarkers(gray)
        else:
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                gray, dictionary, parameters=params
            )
        if marker_ids is not None and len(marker_ids) > 0:
            _n, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board
            )

    if draw and debug is not None:
        if marker_ids is not None and len(marker_ids) > 0:
            cv2.aruco.drawDetectedMarkers(debug, marker_corners, marker_ids)
        if charuco_corners is not None and charuco_ids is not None and len(charuco_ids) > 0:
            if hasattr(cv2.aruco, "drawDetectedCornersCharuco"):
                cv2.aruco.drawDetectedCornersCharuco(debug, charuco_corners, charuco_ids)

    if charuco_ids is None or charuco_corners is None or len(charuco_ids) < int(min_corners):
        return None, debug

    try:
        obj_pts, img_pts = board.matchImagePoints(charuco_corners, charuco_ids)
    except Exception:  # noqa: BLE001
        return None, debug
    if obj_pts is None or img_pts is None or len(obj_pts) < int(min_corners):
        return None, debug

    ok, rvec, tvec = cv2.solvePnP(
        np.asarray(obj_pts, dtype=np.float64),
        np.asarray(img_pts, dtype=np.float64),
        K,
        dist,
    )
    if not ok:
        return None, debug

    T = _rvec_tvec_to_T(rvec, tvec)
    if draw and debug is not None and hasattr(cv2, "drawFrameAxes"):
        axis_len = float(square_length_m) * max(2.0, min(squares_x, squares_y) * 0.35)
        cv2.drawFrameAxes(debug, K, dist, rvec, tvec, axis_len)
    return T, debug


def detect_target(
    image_bgr: np.ndarray,
    intrinsics: Intrinsics,
    aruco_cfg: Any,
    *,
    draw: bool = False,
) -> tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Dispatch ArUco / ChArUco detection from config section or dict-like."""
    get = aruco_cfg.get if isinstance(aruco_cfg, dict) else lambda k, d=None: getattr(aruco_cfg, k, d)
    board_type = str(get("board_type", "charuco") or "charuco").strip().lower()
    dict_name = str(get("dict", "DICT_5X5_50") or "DICT_5X5_50")

    if board_type in ("aruco", "single", "marker"):
        return detect_marker(
            image_bgr,
            intrinsics,
            dict_name=dict_name,
            marker_id=int(get("marker_id", 0) or 0),
            marker_length_m=float(get("marker_length_m", 0.15) or 0.15),
            draw=draw,
        )

    return detect_charuco(
        image_bgr,
        intrinsics,
        dict_name=dict_name,
        squares_x=int(get("squares_x", 5) or 5),
        squares_y=int(get("squares_y", 7) or 7),
        square_length_m=float(get("square_length_m", 0.04) or 0.04),
        marker_length_m=float(get("charuco_marker_length_m", 0.03) or 0.03),
        min_corners=int(get("min_charuco_corners", 6) or 6),
        draw=draw,
    )


def aruco_meta_dict(aruco_cfg: Any) -> dict[str, Any]:
    """Serialize board params into session meta."""
    get = aruco_cfg.get if isinstance(aruco_cfg, dict) else lambda k, d=None: getattr(aruco_cfg, k, d)
    return {
        "board_type": str(get("board_type", "charuco") or "charuco"),
        "dict": str(get("dict", "DICT_5X5_50") or "DICT_5X5_50"),
        "marker_id": int(get("marker_id", 0) or 0),
        "marker_length_m": float(get("marker_length_m", 0.15) or 0.15),
        "squares_x": int(get("squares_x", 5) or 5),
        "squares_y": int(get("squares_y", 7) or 7),
        "square_length_m": float(get("square_length_m", 0.04) or 0.04),
        "charuco_marker_length_m": float(get("charuco_marker_length_m", 0.03) or 0.03),
        "min_charuco_corners": int(get("min_charuco_corners", 6) or 6),
    }
