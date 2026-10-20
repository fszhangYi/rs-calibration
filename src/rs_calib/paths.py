"""Frozen / desktop path helpers (Scheme A compat)."""

from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

APP_ID = "rs-calib"
APP_TITLE = "RS Eye-to-Hand 标定"


def is_frozen() -> bool:
    return bool(getattr(sys, "frozen", False) and hasattr(sys, "_MEIPASS"))


def meipass() -> Path | None:
    if is_frozen():
        return Path(getattr(sys, "_MEIPASS"))
    return None


def project_root() -> Path:
    mp = meipass()
    if mp is not None:
        return mp
    # src/rs_calib/paths.py -> repo root
    return Path(__file__).resolve().parents[2]


def resolve_frontend_dist() -> Path:
    root = project_root()
    candidates = []
    if os.environ.get("FRONTEND_DIST"):
        candidates.append(Path(os.environ["FRONTEND_DIST"]))
    candidates.extend(
        [
            root / "frontend-dist",
            root / "frontend" / "dist",
            Path(__file__).resolve().parent / "web" / "static",
        ]
    )
    for candidate in candidates:
        if candidate.is_dir() and (candidate / "index.html").is_file():
            return candidate
    # last resort
    return Path(__file__).resolve().parent / "web" / "static"


def user_data_dir() -> Path:
    if sys.platform == "win32":
        base = Path(os.environ.get("APPDATA") or (Path.home() / "AppData" / "Roaming"))
        return base / APP_ID
    if sys.platform == "darwin":
        return Path.home() / "Library" / "Application Support" / APP_ID
    xdg = os.environ.get("XDG_DATA_HOME")
    if xdg:
        return Path(xdg) / APP_ID
    return Path.home() / ".local" / "share" / APP_ID


def sensors_src_dir() -> Path:
    """Directory that contains the `sensors` package."""
    root = project_root()
    for candidate in (
        root / "sensors_src",
        root / "vendor",
        root / "sensors" / "src",
        root.parent / "sensors" / "src",
    ):
        if (candidate / "sensors").is_dir():
            return candidate
        # vendor/sensors is the package itself (symlink)
        if candidate.name == "vendor" and (candidate / "sensors" / "__init__.py").is_file():
            return candidate
    return root / "vendor"


def ensure_runtime_env(*, desktop: bool = False) -> Path:
    data = user_data_dir()
    data.mkdir(parents=True, exist_ok=True)
    (data / "sessions").mkdir(parents=True, exist_ok=True)
    (data / "configs").mkdir(parents=True, exist_ok=True)
    (data / "logs").mkdir(parents=True, exist_ok=True)

    # seed default.yaml once
    src_cfg = project_root() / "configs" / "default.yaml"
    if not src_cfg.is_file():
        src_cfg = Path(__file__).resolve().parents[2] / "configs" / "default.yaml"
    dst_cfg = data / "configs" / "default.yaml"
    if src_cfg.is_file() and not dst_cfg.is_file():
        shutil.copy2(src_cfg, dst_cfg)

    frontend = resolve_frontend_dist()
    os.environ.setdefault("FRONTEND_DIST", str(frontend.resolve()))
    os.environ.setdefault("RS_CALIB_USER_DATA", str(data.resolve()))
    os.environ.setdefault("RS_CALIB_SESSION_ROOT", str((data / "sessions").resolve()))
    if dst_cfg.is_file():
        os.environ.setdefault("RS_CALIB_CONFIG", str(dst_cfg.resolve()))

    src = sensors_src_dir()
    if src.is_dir() and str(src) not in sys.path:
        mp = meipass()
        if mp is not None:
            mps = str(mp)
            if mps in sys.path:
                sys.path.remove(mps)
            sys.path.insert(0, mps)
            sys.path.insert(1, str(src))
        else:
            sys.path.insert(0, str(src))

    if desktop or is_frozen():
        work = data / "run"
        work.mkdir(parents=True, exist_ok=True)
        os.chdir(work)
    return data
