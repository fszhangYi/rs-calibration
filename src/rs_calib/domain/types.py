from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Optional

import numpy as np


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    dist: list[float]
    width: int
    height: int
    fps: int = 15

    def K(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

    def dist_coeffs(self) -> np.ndarray:
        d = list(self.dist) + [0.0] * 5
        return np.asarray(d[:5], dtype=np.float64)


@dataclass
class DeviceInfo:
    serial: str
    name: str = ""
    firmware: str = ""


@dataclass
class ArmState:
    ok: bool
    joints_rad: Optional[list[float]]
    armed: bool
    endpoint: str
    error: Optional[str] = None
    ts: float = 0.0


@dataclass
class FlangeSample:
    ts_cam: float
    ts_arm: float
    frame_idx: int
    joints_rad: list[float]
    T_base_flange: np.ndarray
    pose_xyzrpy: list[float]

    def to_json_dict(self) -> dict[str, Any]:
        return {
            "ts_cam": self.ts_cam,
            "ts_arm": self.ts_arm,
            "frame_idx": self.frame_idx,
            "joints_rad": [float(x) for x in self.joints_rad],
            "T_base_flange": self.T_base_flange.tolist(),
            "pose_xyzrpy": [float(x) for x in self.pose_xyzrpy],
        }

    @staticmethod
    def from_json_dict(d: dict[str, Any]) -> "FlangeSample":
        return FlangeSample(
            ts_cam=float(d["ts_cam"]),
            ts_arm=float(d["ts_arm"]),
            frame_idx=int(d["frame_idx"]),
            joints_rad=[float(x) for x in d["joints_rad"]],
            T_base_flange=np.asarray(d["T_base_flange"], dtype=np.float64),
            pose_xyzrpy=[float(x) for x in d["pose_xyzrpy"]],
        )


@dataclass
class CalibSample:
    frame_idx: int
    T_base_flange: np.ndarray
    T_cam_marker: np.ndarray


@dataclass
class ExtrinsicsResult:
    T_base_cam: np.ndarray
    T_cam_base: np.ndarray
    method: str
    n_samples: int
    used_indices: list[int] = field(default_factory=list)
    metrics: dict[str, Any] = field(default_factory=dict)
    # eye_to_hand | eye_in_hand — primary matrix differs by setup
    setup: str = "eye_to_hand"
    T_flange_cam: Optional[np.ndarray] = None
    T_cam_flange: Optional[np.ndarray] = None

    def primary_T(self) -> np.ndarray:
        """Extrinsic used for convergence / display for the active setup."""
        if self.setup == "eye_in_hand" and self.T_flange_cam is not None:
            return self.T_flange_cam
        return self.T_base_cam


@dataclass
class ConvergenceStatus:
    n_valid: int
    delta_t_mm: float
    delta_R_deg: float
    stable_streak: int
    converged: bool
    warning: Optional[str] = None


@dataclass
class FilterReport:
    n_raw: int
    n_after_detect: int
    n_after_dup: int
    n_after_outlier: int
    n_selected: int
    indices: list[int]
    warnings: list[str] = field(default_factory=list)


@dataclass
class VerifyFrameMetrics:
    frame_idx: int
    marker_err_t_mm: float
    marker_err_R_deg: float
    has_detect: bool
