# -*- mode: python ; coding: utf-8 -*-
"""Windows / Wine onedir spec for rs-calib (FastAPI web UI desktop)."""

from pathlib import Path

from PyInstaller.utils.hooks import collect_submodules

# SPECPATH = directory containing this .spec
ROOT = Path(SPECPATH).resolve().parent  # noqa: F821
SRC = ROOT / "src"

SENSORS_SRC = (ROOT / "vendor").resolve()
if not (SENSORS_SRC / "sensors").is_dir():
    SENSORS_SRC = (ROOT.parent / "sensors" / "src").resolve()

frontend_dist = ROOT / "frontend" / "dist"
if not (frontend_dist / "index.html").is_file():
    frontend_dist = ROOT / "src" / "rs_calib" / "web" / "static"

datas = [
    (str(frontend_dist), "frontend-dist"),
    (str(ROOT / "configs"), "configs"),
]
binaries: list = []
if (SENSORS_SRC / "sensors").is_dir():
    datas.append((str(SENSORS_SRC / "sensors"), "sensors_src/sensors"))

hiddenimports = [
    "uvicorn",
    "uvicorn.logging",
    "uvicorn.loops",
    "uvicorn.loops.auto",
    "uvicorn.protocols",
    "uvicorn.protocols.http",
    "uvicorn.protocols.http.auto",
    "uvicorn.protocols.websockets",
    "uvicorn.protocols.websockets.auto",
    "uvicorn.lifespan",
    "uvicorn.lifespan.on",
    "fastapi",
    "starlette",
    "pydantic",
    "yaml",
    "numpy",
    "cv2",
    "scipy",
    "scipy.spatial",
    "scipy.spatial.transform",
    "multipart",
    "pkgutil",
    "rs_calib",
    "rs_calib.desktop_main",
    "rs_calib.paths",
    "rs_calib.web",
    "rs_calib.web.server",
    "rs_calib.app_services",
    "rs_calib.domain",
    "rs_calib.infra",
    "rs_calib.config",
    "sensors",
    "sensors.core",
    "sensors.core.base",
    "sensors.core.config",
    "sensors.core.health",
    "sensors.core.kinds",
    "sensors.core.registry",
    "sensors.drivers",
    "sensors.drivers.arm",
    "sensors.drivers.arm.elite_write",
    "sensors.drivers.arm.elite_read",
    "sensors.drivers.camera",
    "sensors.drivers.camera.realsense",
    "sensors.kinematics",
]
try:
    hiddenimports += collect_submodules("sensors.drivers")
    hiddenimports += collect_submodules("sensors.kinematics")
    hiddenimports += collect_submodules("rs_calib")
except Exception:
    pass

a = Analysis(  # noqa: F821
    [str(SRC / "rs_calib" / "desktop_main.py")],
    pathex=[str(SRC)] + ([str(SENSORS_SRC)] if SENSORS_SRC.is_dir() else []),
    binaries=binaries,
    datas=datas,
    hiddenimports=hiddenimports,
    hookspath=[],
    hooksconfig={},
    runtime_hooks=[],
    excludes=["tkinter", "matplotlib", "PySide2", "PySide6", "PyQt5", "PyQt6", "IPython"],
    noarchive=False,
    module_collection_mode={
        "numpy": "py",
    },
)

pyz = PYZ(a.pure)  # noqa: F821

exe = EXE(  # noqa: F821
    pyz,
    a.scripts,
    [],
    exclude_binaries=True,
    name="rs-calib",
    debug=False,
    bootloader_ignore_signals=False,
    strip=False,
    upx=False,
    console=True,
    disable_windowed_traceback=False,
    argv_emulation=False,
    target_arch=None,
    codesign_identity=None,
    entitlements_file=None,
)

coll = COLLECT(  # noqa: F821
    exe,
    a.binaries,
    a.zipfiles,
    a.datas,
    strip=False,
    upx=False,
    upx_exclude=[],
    name="rs-calib",
)
