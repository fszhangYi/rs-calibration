"""Generate a calibration-friendly 6-DoF matrix (xyz + rpy) of poses."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Any, Optional, Sequence


@dataclass
class MatrixPathParams:
    """Offsets are applied in the **base** frame relative to the seed pose.

    Full pose matrix = translation grid (nx×ny×nz) × orientation grid (n_rx×n_ry×n_rz).
    Orientation samples are XYZ-euler additives on the seed rpy (radians after conversion).
    """

    # translation grid
    nx: int = 3
    ny: int = 3
    nz: int = 1
    step_x_m: float = 0.05
    step_y_m: float = 0.05
    step_z_m: float = 0.03
    # orientation grid (centered on seed; 1 means keep seed angle on that axis)
    n_rx: int = 1
    n_ry: int = 3
    n_rz: int = 1
    step_rx_deg: float = 10.0
    step_ry_deg: float = 12.0
    step_rz_deg: float = 10.0
    # visiting order
    snake: bool = True
    dwell_s: float = 0.8
    frame: str = "flange"  # flange | tcp

    # legacy aliases (still accepted via from_dict)
    include_tilts: Optional[bool] = None
    tilt_rx_deg: Optional[float] = None
    tilt_ry_deg: Optional[float] = None
    tilt_rz_deg: Optional[float] = None

    def clamp(self) -> "MatrixPathParams":
        # migrate legacy tilt fields if present
        if self.include_tilts is False:
            self.n_rx = self.n_ry = self.n_rz = 1
        elif self.include_tilts is True or any(
            v is not None for v in (self.tilt_rx_deg, self.tilt_ry_deg, self.tilt_rz_deg)
        ):
            # old UI: ±tilt on axes with nonzero deg → 3 samples
            if self.tilt_rx_deg is not None and abs(self.tilt_rx_deg) > 1e-6:
                self.n_rx = max(self.n_rx, 3)
                self.step_rx_deg = abs(float(self.tilt_rx_deg))
            if self.tilt_ry_deg is not None and abs(self.tilt_ry_deg) > 1e-6:
                self.n_ry = max(self.n_ry, 3)
                self.step_ry_deg = abs(float(self.tilt_ry_deg))
            if self.tilt_rz_deg is not None and abs(self.tilt_rz_deg) > 1e-6:
                self.n_rz = max(self.n_rz, 3)
                self.step_rz_deg = abs(float(self.tilt_rz_deg))
            if self.include_tilts is True and self.n_rx == 1 and self.n_ry == 1 and self.n_rz == 1:
                self.n_ry = 3
                self.step_ry_deg = abs(float(self.tilt_ry_deg or self.step_ry_deg or 12.0))

        self.nx = max(1, min(int(self.nx), 11))
        self.ny = max(1, min(int(self.ny), 11))
        self.nz = max(1, min(int(self.nz), 5))
        self.n_rx = max(1, min(int(self.n_rx), 7))
        self.n_ry = max(1, min(int(self.n_ry), 7))
        self.n_rz = max(1, min(int(self.n_rz), 7))
        self.step_x_m = float(max(0.0, min(self.step_x_m, 0.3)))
        self.step_y_m = float(max(0.0, min(self.step_y_m, 0.3)))
        self.step_z_m = float(max(0.0, min(self.step_z_m, 0.3)))
        self.step_rx_deg = float(max(0.0, min(self.step_rx_deg, 45.0)))
        self.step_ry_deg = float(max(0.0, min(self.step_ry_deg, 45.0)))
        self.step_rz_deg = float(max(0.0, min(self.step_rz_deg, 45.0)))
        self.dwell_s = float(max(0.0, min(self.dwell_s, 10.0)))
        self.frame = "tcp" if str(self.frame).lower() == "tcp" else "flange"
        # clear legacy so asdict stays clean for persistence
        self.include_tilts = None
        self.tilt_rx_deg = None
        self.tilt_ry_deg = None
        self.tilt_rz_deg = None
        return self

    def to_dict(self) -> dict[str, Any]:
        self.clamp()
        d = asdict(self)
        for k in ("include_tilts", "tilt_rx_deg", "tilt_ry_deg", "tilt_rz_deg"):
            d.pop(k, None)
        return d

    @staticmethod
    def from_dict(d: Optional[dict[str, Any]]) -> "MatrixPathParams":
        p = MatrixPathParams()
        if not d:
            return p.clamp()
        for k, v in d.items():
            if hasattr(p, k):
                setattr(p, k, v)
        return p.clamp()

    def expected_count(self) -> int:
        self.clamp()
        return self.nx * self.ny * self.nz * self.n_rx * self.n_ry * self.n_rz


@dataclass
class MatrixWaypoint:
    index: int
    ix: int
    iy: int
    iz: int
    irx: int
    iry: int
    irz: int
    xyzrpy: list[float]  # meters, radians
    label: str = ""
    # backward-compatible alias used by older logs
    tilt_tag: str = ""


def _centered_offsets(n: int, step: float) -> list[float]:
    if n <= 1:
        return [0.0]
    return [(i - (n - 1) / 2.0) * step for i in range(n)]


def generate_matrix_waypoints(
    seed_xyzrpy: Sequence[float],
    params: MatrixPathParams,
) -> list[MatrixWaypoint]:
    """Build xyz×rpy grid around ``seed_xyzrpy`` (base-frame).

    Each waypoint = seed_xyz + Δxyz, seed_rpy + Δrpy.
    Snake order applies to the translation (ix,iy) loops to shorten travel.
    """
    params = params.clamp()
    seed = [float(x) for x in list(seed_xyzrpy)[:6]]
    while len(seed) < 6:
        seed.append(0.0)

    oxs = _centered_offsets(params.nx, params.step_x_m)
    oys = _centered_offsets(params.ny, params.step_y_m)
    ozs = _centered_offsets(params.nz, params.step_z_m)
    orxs = _centered_offsets(params.n_rx, math.radians(params.step_rx_deg))
    orys = _centered_offsets(params.n_ry, math.radians(params.step_ry_deg))
    orzs = _centered_offsets(params.n_rz, math.radians(params.step_rz_deg))

    waypoints: list[MatrixWaypoint] = []
    idx = 0

    for iz, oz in enumerate(ozs):
        for ix, ox in enumerate(oxs):
            y_indices = list(range(params.ny))
            if params.snake and (ix % 2 == 1):
                y_indices = list(reversed(y_indices))
            for iy in y_indices:
                oy = oys[iy]
                for irx, orx in enumerate(orxs):
                    for iry, ory in enumerate(orys):
                        for irz, orz in enumerate(orzs):
                            xyzrpy = [
                                seed[0] + ox,
                                seed[1] + oy,
                                seed[2] + oz,
                                seed[3] + orx,
                                seed[4] + ory,
                                seed[5] + orz,
                            ]
                            tag = (
                                f"rpy[{irx},{iry},{irz}] "
                                f"Δ=({math.degrees(orx):+.1f},{math.degrees(ory):+.1f},{math.degrees(orz):+.1f})°"
                            )
                            label = f"xyz[{ix},{iy},{iz}] {tag}"
                            waypoints.append(
                                MatrixWaypoint(
                                    index=idx,
                                    ix=ix,
                                    iy=iy,
                                    iz=iz,
                                    irx=irx,
                                    iry=iry,
                                    irz=irz,
                                    xyzrpy=xyzrpy,
                                    label=label,
                                    tilt_tag=tag,
                                )
                            )
                            idx += 1
    return waypoints


def spans_from_params(params: MatrixPathParams) -> dict[str, float]:
    """Grid spans implied by N×step (centered), independent of seed."""
    p = params.clamp()
    return {
        "x_span_m": float(max(0, p.nx - 1) * p.step_x_m),
        "y_span_m": float(max(0, p.ny - 1) * p.step_y_m),
        "z_span_m": float(max(0, p.nz - 1) * p.step_z_m),
        "rx_span_deg": float(max(0, p.n_rx - 1) * p.step_rx_deg),
        "ry_span_deg": float(max(0, p.n_ry - 1) * p.step_ry_deg),
        "rz_span_deg": float(max(0, p.n_rz - 1) * p.step_rz_deg),
    }


def summarize_waypoints(waypoints: Sequence[MatrixWaypoint]) -> dict[str, Any]:
    if not waypoints:
        return {"n": 0}
    xs = [w.xyzrpy[0] for w in waypoints]
    ys = [w.xyzrpy[1] for w in waypoints]
    zs = [w.xyzrpy[2] for w in waypoints]
    rxs = [w.xyzrpy[3] for w in waypoints]
    rys = [w.xyzrpy[4] for w in waypoints]
    rzs = [w.xyzrpy[5] for w in waypoints]
    return {
        "n": len(waypoints),
        "x_span_m": max(xs) - min(xs),
        "y_span_m": max(ys) - min(ys),
        "z_span_m": max(zs) - min(zs),
        "rx_span_deg": math.degrees(max(rxs) - min(rxs)),
        "ry_span_deg": math.degrees(max(rys) - min(rys)),
        "rz_span_deg": math.degrees(max(rzs) - min(rzs)),
        "labels_preview": [w.label for w in waypoints[:8]],
    }
