from __future__ import annotations

import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterator, Optional

import numpy as np

from rs_calib.domain.math3d import xyzrpy_from_T
from rs_calib.domain.types import CalibSample, ConvergenceStatus, ExtrinsicsResult, FlangeSample, Intrinsics
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


def camera_entry(
    *,
    serial: str,
    name: str,
    intrinsics: Intrinsics,
    video_info: dict[str, Any],
) -> dict[str, Any]:
    return {
        "serial": serial,
        "name": name,
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
        "video": video_info,
    }


def build_meta(
    *,
    cameras: list[dict[str, Any]],
    robot_ip: str,
    tcp_xyz: list[float],
    aruco: dict[str, Any],
    incomplete: bool = False,
    frame_count: int = 0,
    T_flange_marker: Optional[list[list[float]]] = None,
    handeye: Optional[dict[str, Any]] = None,
    # legacy single-cam kwargs (optional)
    camera_serial: str = "",
    camera_name: str = "",
    intrinsics: Optional[Intrinsics] = None,
    video_info: Optional[dict[str, Any]] = None,
) -> dict[str, Any]:
    cams = list(cameras)
    if not cams and intrinsics is not None:
        cams = [
            camera_entry(
                serial=camera_serial,
                name=camera_name,
                intrinsics=intrinsics,
                video_info=video_info or {"path": "video.mp4", "fourcc": "mp4v", "fallback_frames": False},
            )
        ]
    primary = cams[0] if cams else {}
    return {
        "schema_version": 2 if len(cams) > 1 else 1,
        "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
        "software_version": __version__,
        "incomplete": incomplete,
        "multi_camera": len(cams) > 1,
        "cameras": cams,
        # legacy single-cam mirror (first camera) for older tools
        "camera": {
            "serial": primary.get("serial", camera_serial),
            "name": primary.get("name", camera_name),
            "width": primary.get("width"),
            "height": primary.get("height"),
            "fps": primary.get("fps"),
            "intrinsics": primary.get("intrinsics"),
        },
        "arm": {"robot_ip": robot_ip, "model": "EC616"},
        "tcp_xyz": list(tcp_xyz),
        "aruco": aruco,
        "handeye": handeye or {"setup": "eye_to_hand"},
        "T_flange_marker": T_flange_marker,
        "video": primary.get("video") or video_info,
        "frame_count": frame_count,
    }


def list_cameras_from_meta(meta: dict[str, Any]) -> list[dict[str, Any]]:
    cams = meta.get("cameras")
    if isinstance(cams, list) and cams:
        return cams
    cam = meta.get("camera")
    if isinstance(cam, dict) and cam.get("serial"):
        entry = dict(cam)
        if "video" not in entry and meta.get("video"):
            entry["video"] = meta["video"]
        return [entry]
    return []


def _result_payload(result: ExtrinsicsResult) -> dict[str, Any]:
    setup = getattr(result, "setup", None) or "eye_to_hand"
    payload: dict[str, Any] = {
        "setup": setup,
        "method": result.method,
        "n_samples": result.n_samples,
        "used_frame_indices": list(result.used_indices),
        "T_base_cam": result.T_base_cam.tolist(),
        "T_cam_base": result.T_cam_base.tolist(),
        "xyzrpy_base_cam": xyzrpy_from_T(result.T_base_cam),
    }
    if result.T_flange_cam is not None:
        payload["T_flange_cam"] = result.T_flange_cam.tolist()
        payload["xyzrpy_flange_cam"] = xyzrpy_from_T(result.T_flange_cam)
    if result.T_cam_flange is not None:
        payload["T_cam_flange"] = result.T_cam_flange.tolist()
    if result.metrics:
        payload["metrics"] = dict(result.metrics)
    return payload


def _sample_payload(
    sample: CalibSample,
    joints_rad: Optional[list[float]] = None,
) -> dict[str, Any]:
    out: dict[str, Any] = {
        "frame_idx": int(sample.frame_idx),
        "T_base_flange": np.asarray(sample.T_base_flange, dtype=float).tolist(),
        "T_cam_marker": np.asarray(sample.T_cam_marker, dtype=float).tolist(),
        "xyzrpy_base_flange": xyzrpy_from_T(sample.T_base_flange),
        "xyzrpy_cam_marker": xyzrpy_from_T(sample.T_cam_marker),
    }
    if joints_rad is not None:
        out["joints_rad"] = [float(x) for x in joints_rad]
    return out


def append_live_solve(
    session_dir: Path,
    *,
    solve_idx: int,
    samples: list[CalibSample],
    joints_by_idx: dict[int, list[float]],
    result: ExtrinsicsResult,
    status: Optional[ConvergenceStatus] = None,
    extra: Optional[dict[str, Any]] = None,
) -> Path:
    """Append one live HandEye solve (basis samples + result) to ``live_solves.jsonl``."""
    session_dir = Path(session_dir)
    session_dir.mkdir(parents=True, exist_ok=True)
    path = session_dir / "live_solves.jsonl"
    record: dict[str, Any] = {
        "solve_idx": int(solve_idx),
        "ts": datetime.now(timezone.utc).astimezone().isoformat(),
        "source": "live",
        "basis": {
            "n_samples": len(samples),
            "samples": [
                _sample_payload(s, joints_by_idx.get(s.frame_idx)) for s in samples
            ],
        },
        "result": _result_payload(result),
    }
    if status is not None:
        record["convergence"] = {
            "converged": bool(status.converged),
            "delta_t_mm": float(status.delta_t_mm),
            "delta_R_deg": float(status.delta_R_deg),
            "stable_streak": int(status.stable_streak),
            "n_valid": int(status.n_valid),
            "warning": status.warning,
        }
    if extra:
        record["extra"] = extra
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")
    return path


def save_extrinsics(
    session_dir: Path,
    result: ExtrinsicsResult,
    extra: Optional[dict] = None,
    *,
    camera_serial: Optional[str] = None,
) -> Path:
    if camera_serial:
        cam_dir = session_dir / "cameras" / camera_serial
        cam_dir.mkdir(parents=True, exist_ok=True)
        path = cam_dir / "extrinsics.json"
        # also write a flat convenience copy
        flat = session_dir / f"extrinsics_{camera_serial}.json"
    else:
        path = session_dir / "extrinsics.json"
        flat = None
    payload = {
        "schema_version": 2,
        "camera_serial": camera_serial,
        "convergence": result.metrics.get("convergence"),
        "source_session": session_dir.name,
        "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
        **_result_payload(result),
    }
    if extra:
        payload.update(extra)
    with path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2, ensure_ascii=False)
    if flat is not None:
        with flat.open("w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
    return path


def load_extrinsics(path: Path) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    T_base_cam = np.asarray(data["T_base_cam"], dtype=np.float64)
    T_cam_base = np.asarray(data.get("T_cam_base") or np.linalg.inv(T_base_cam), dtype=np.float64)
    return T_base_cam, T_cam_base, data


def intrinsics_from_camera_entry(cam: dict[str, Any]) -> Intrinsics:
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


def intrinsics_from_meta(meta: dict[str, Any], camera_serial: Optional[str] = None) -> Intrinsics:
    cams = list_cameras_from_meta(meta)
    if camera_serial:
        for c in cams:
            if c.get("serial") == camera_serial:
                return intrinsics_from_camera_entry(c)
        raise KeyError(f"camera serial not in meta: {camera_serial}")
    if cams:
        return intrinsics_from_camera_entry(cams[0])
    cam = meta["camera"]
    return Intrinsics(
        fx=float(cam["intrinsics"]["fx"]),
        fy=float(cam["intrinsics"]["fy"]),
        cx=float(cam["intrinsics"]["cx"]),
        cy=float(cam["intrinsics"]["cy"]),
        dist=[float(x) for x in cam["intrinsics"].get("dist", [0, 0, 0, 0, 0])],
        width=int(cam["width"]),
        height=int(cam["height"]),
        fps=int(cam.get("fps", 15)),
    )


def resolve_camera_media_dir(session_dir: Path, cam: dict[str, Any]) -> Path:
    """Directory that contains video.mp4 or frames/ for this camera."""
    video = cam.get("video") or {}
    rel = video.get("path") or ""
    if rel in ("video.mp4", "frames", "", None):
        # legacy single-cam at session root
        return session_dir
    # path like cameras/<serial>/video.mp4 or cameras/<serial>
    p = Path(str(rel))
    if p.suffix:  # file
        return session_dir / p.parent
    return session_dir / p
