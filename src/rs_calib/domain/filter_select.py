from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np

from rs_calib.domain.math3d import rotation_angle_deg, se3_delta
from rs_calib.domain.types import CalibSample, FilterReport


@dataclass
class FilterParams:
    dup_joint_eps_deg: float = 0.3
    dup_pose_eps_mm: float = 3.0
    dup_pose_eps_deg: float = 1.0
    outlier_jump_mm: float = 50.0
    outlier_jump_deg: float = 30.0
    min_translation_span_m: float = 0.15
    min_rotation_span_deg: float = 25.0
    min_samples: int = 12
    max_select: int = 40


def _joint_dup(a: Sequence[float], b: Sequence[float], eps_deg: float) -> bool:
    da = np.degrees(np.abs(np.asarray(a) - np.asarray(b)))
    return bool(np.all(da <= eps_deg))


def _pose_dup(Ta: np.ndarray, Tb: np.ndarray, eps_mm: float, eps_deg: float) -> bool:
    t_mm, r_deg = se3_delta(Ta, Tb)
    return t_mm <= eps_mm and r_deg <= eps_deg


def filter_and_select(
    samples: Sequence[CalibSample],
    joints_by_idx: Optional[dict[int, list[float]]] = None,
    params: Optional[FilterParams] = None,
) -> tuple[list[CalibSample], FilterReport]:
    """Drop near-duplicates / jump outliers, then diversity-select."""
    p = params or FilterParams()
    n_raw = len(samples)
    warnings: list[str] = []
    if n_raw == 0:
        return [], FilterReport(0, 0, 0, 0, 0, [], ["no samples"])

    # assume caller already dropped no-detect; here samples are valid detections
    kept: list[CalibSample] = []
    for s in samples:
        if not kept:
            kept.append(s)
            continue
        prev = kept[-1]
        j_prev = (joints_by_idx or {}).get(prev.frame_idx)
        j_cur = (joints_by_idx or {}).get(s.frame_idx)
        if j_prev is not None and j_cur is not None:
            if _joint_dup(j_prev, j_cur, p.dup_joint_eps_deg):
                continue
        if _pose_dup(prev.T_base_flange, s.T_base_flange, p.dup_pose_eps_mm, p.dup_pose_eps_deg):
            continue
        # outlier jump vs previous kept
        t_mm, r_deg = se3_delta(prev.T_base_flange, s.T_base_flange)
        if t_mm > p.outlier_jump_mm or r_deg > p.outlier_jump_deg:
            continue
        kept.append(s)

    n_after_dup = len(kept)
    # outlier already applied inline; n_after_outlier == n_after_dup for this pipeline
    n_after_outlier = n_after_dup

    # span check
    if len(kept) >= 2:
        ts = np.array([s.T_base_flange[:3, 3] for s in kept])
        span = float(np.linalg.norm(ts.max(axis=0) - ts.min(axis=0)))
        if span < p.min_translation_span_m:
            warnings.append(f"translation span {span:.3f}m < {p.min_translation_span_m}")
        R0 = kept[0].T_base_flange[:3, :3]
        max_r = max(rotation_angle_deg(R0, s.T_base_flange[:3, :3]) for s in kept)
        if max_r < p.min_rotation_span_deg:
            warnings.append(f"rotation span {max_r:.1f}deg < {p.min_rotation_span_deg}")

    selected = _select_diverse(kept, target=max(p.min_samples * 2, 24), hard_max=p.max_select)
    report = FilterReport(
        n_raw=n_raw,
        n_after_detect=n_raw,
        n_after_dup=n_after_dup,
        n_after_outlier=n_after_outlier,
        n_selected=len(selected),
        indices=[s.frame_idx for s in selected],
        warnings=warnings,
    )
    return selected, report


def _select_diverse(
    samples: list[CalibSample],
    *,
    target: int,
    hard_max: int,
) -> list[CalibSample]:
    if len(samples) <= hard_max:
        return list(samples)
    k = min(target, hard_max, len(samples))
    # greedy farthest-point in (t_mm, R_deg) space
    selected = [samples[0]]
    remaining = list(samples[1:])
    while len(selected) < k and remaining:
        best_i = 0
        best_d = -1.0
        for i, cand in enumerate(remaining):
            dmin = min(
                se3_delta(cand.T_base_flange, s.T_base_flange)[0]
                + 10.0 * se3_delta(cand.T_base_flange, s.T_base_flange)[1]
                for s in selected
            )
            if dmin > best_d:
                best_d = dmin
                best_i = i
        selected.append(remaining.pop(best_i))
    return selected


def is_near_duplicate(
    a: CalibSample,
    b: CalibSample,
    *,
    joints_a: Optional[Sequence[float]] = None,
    joints_b: Optional[Sequence[float]] = None,
    params: Optional[FilterParams] = None,
) -> bool:
    p = params or FilterParams()
    if joints_a is not None and joints_b is not None:
        if _joint_dup(joints_a, joints_b, p.dup_joint_eps_deg):
            return True
    return _pose_dup(a.T_base_flange, b.T_base_flange, p.dup_pose_eps_mm, p.dup_pose_eps_deg)
