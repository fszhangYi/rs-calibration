from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator, Optional

import numpy as np

from rs_calib.domain.math3d import xyzrpy_from_T
from rs_calib.domain.types import ExtrinsicsResult, FlangeSample, Intrinsics
from rs_calib.version import __version__


def new_session_dir(root: str | Path) -> Path:
    root_p = Path(root)
    root_p.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = root_p / f"session_{stamp}"
    path.mkdir(parents=True, exist_ok=False)
    return path


def write_meta(session_dir: Path, meta: dict[str, Any]) -> None:
    path = session_dir / "meta.json"
    with path.open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)


def read_meta(session_dir: Path) -> dict[str, Any]:
    with (session_dir / "meta.json").open("r", encoding="utf-8") as f:
        return json.load(f)


def append_flange_sample(session_dir: Path, sample: FlangeSample) -> None:
    path = session_dir / "joints_poses.jsonl"
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(sample.to_json_dict(), ensure_ascii=False) + "\n")


def iter_flange_samples(session_dir: Path) -> Iterator[FlangeSample]:
    path = session_dir / "joints_poses.jsonl"
    if not path.is_file():
        return
        yield  # pragma: no cover
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            yield FlangeSample.from_json_dict(json.loads(line))


def load_flange_samples(session_dir: Path) -> list[FlangeSample]:
    return list(iter_flange_samples(session_dir))


def build_meta(
    *,
    camera_serial: str,
    camera_name: str,
    intrinsics: Intrinsics,
    robot_ip: str,
    tcp_xyz: list[float],
    aruco: dict[str, Any],
    video_info: dict[str, Any],
    incomplete: bool = False,
    frame_count: int = 0,
    T_flange_marker: Optional[list[list[float]]] = None,
) -> dict[str, Any]:
    return {
        "schema_version": 1,
        "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "software_version": __version__,
        "incomplete": incomplete,
        "camera": {
            "serial": camera_serial,
            "name": camera_name,
            "width": intrinsics.width,
            "height": intrinsics.height,
            "fps": intrinsics.fps,
            "intrinsics": {
                "fx": intrinsics.fx,
                "fy": intrinsics.fy,
                "cx": intrinsics.cx,
                "cy": intrinsics.cy,
                "dist": list(intrinsics.dist),
            },
        },
        "arm": {"robot_ip": robot_ip, "model": "EC616"},
        "tcp_xyz": list(tcp_xyz),
        "aruco": aruco,
        "T_flange_marker": T_flange_marker,
        "video": video_info,
        "frame_count": frame_count,
    }


def save_extrinsics(session_dir: Path, result: ExtrinsicsResult, extra: Optional[dict] = None) -> Path:
    path = session_dir / "extrinsics.json"
    payload = {
        "schema_version": 1,
        "T_base_cam": result.T_base_cam.tolist(),
        "T_cam_base": result.T_cam_base.tolist(),
        "xyzrpy_base_cam": xyzrpy_from_T(result.T_base_cam),
        "method": result.method,
        "n_samples": result.n_samples,
        "used_frame_indices": result.used_indices,
        "convergence": result.metrics.get("convergence"),
        "source_session": session_dir.name,
        "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
    }
    if extra:
        payload.update(extra)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    return path


def load_extrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    T_base_cam = np.asarray(data["T_base_cam"], dtype=np.float64)
    T_cam_base = np.asarray(data.get("T_cam_base") or np.linalg.inv(T_base_cam), dtype=np.float64)
    return T_base_cam, T_cam_base, data


def intrinsics_from_meta(meta: dict[str, Any]) -> Intrinsics:
    cam = meta["camera"]
    intr = cam["intrinsics"]
    return Intrinsics(
        fx=float(intr["fx"]),
        fy=float(intr["fy"]),
        cx=float(intr["cx"]),
        cy=float(intr["cy"]),
        dist=[float(x) for x in intr.get("dist", [0, 0, 0, 0, 0])],
        width=int(cam["width"]),
        height=int(cam["height"]),
        fps=int(cam.get("fps", 15)),
    )
