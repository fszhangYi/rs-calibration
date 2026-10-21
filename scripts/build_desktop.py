#!/usr/bin/env python3
"""Build rs-calib desktop packages (Windows via Wine on Linux host)."""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
APP_ID = "rs-calib"
APP_EXE = "rs-calib"
PIP_INDEX = os.environ.get("PIP_INDEX_URL", "https://pypi.tuna.tsinghua.edu.cn/simple")


def _utc_stamp() -> str:
    return time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())


def _run(cmd: list[str], *, env: dict | None = None, cwd: Path | None = None) -> None:
    print("+", " ".join(cmd), flush=True)
    subprocess.check_call(cmd, env=env, cwd=str(cwd or ROOT))


def _linux_to_wine_path(path: Path) -> str:
    p = path.resolve()
    return "Z:" + str(p).replace("/", "\\")


def prepare_frontend() -> Path:
    """Copy web static UI into frontend/dist (Scheme A frontend-dist)."""
    src = ROOT / "src" / "rs_calib" / "web" / "static"
    dst = ROOT / "frontend" / "dist"
    if not (src / "index.html").is_file():
        raise SystemExit(f"missing UI: {src / 'index.html'}")
    if dst.exists():
        shutil.rmtree(dst)
    shutil.copytree(src, dst)
    print(f"[frontend] prepared {dst}", flush=True)
    return dst


def _ensure_win_python(tools: Path) -> Path:
    py = tools / "win-python"
    python_exe = py / "python.exe"
    if python_exe.is_file():
        return py

    tools.mkdir(parents=True, exist_ok=True)
    ver = "3.10.11"
    urls = [
        f"https://registry.npmmirror.com/-/binary/python/{ver}/python-{ver}-embed-amd64.zip",
        f"https://www.python.org/ftp/python/{ver}/python-{ver}-embed-amd64.zip",
    ]
    zip_path = tools / f"python-{ver}-embed-amd64.zip"
    if not zip_path.is_file() or zip_path.stat().st_size < 1_000_000:
        ok = False
        for url in urls:
            try:
                _run(["curl", "-L", "--retry", "3", "-o", str(zip_path), url])
                if zip_path.is_file() and zip_path.stat().st_size > 1_000_000:
                    ok = True
                    break
            except Exception:
                continue
        if not ok:
            raise SystemExit("failed to download embeddable CPython")
    if py.exists():
        shutil.rmtree(py)
    py.mkdir(parents=True)
    _run(["unzip", "-o", str(zip_path), "-d", str(py)])

    pth = next(py.glob("python*._pth"))
    lines = pth.read_text(encoding="utf-8").splitlines()
    out = []
    for line in lines:
        s = line.strip()
        if s.startswith("#") and "import site" in s:
            out.append("import site")
        elif s == "import site":
            out.append("import site")
        else:
            out.append(line)
    if "import site" not in "\n".join(out):
        out.append("import site")
    if "Lib\\site-packages" not in "\n".join(out) and "Lib/site-packages" not in "\n".join(out):
        out.append("Lib\\site-packages")
    pth.write_text("\n".join(out) + "\n", encoding="utf-8")
    (py / "Lib" / "site-packages").mkdir(parents=True, exist_ok=True)

    get_pip = tools / "get-pip.py"
    if not get_pip.is_file():
        _run(["curl", "-L", "-o", str(get_pip), "https://bootstrap.pypa.io/get-pip.py"])
    return py


def _wine_env(prefix: Path) -> dict[str, str]:
    env = os.environ.copy()
    env["WINEPREFIX"] = str(prefix)
    env["WINEARCH"] = "win64"
    env["WINEDEBUG"] = "-all"
    env["PIP_INDEX_URL"] = PIP_INDEX
    env["PYTHONNOUSERSITE"] = "1"
    return env


def _wine_bin() -> str:
    for cand in ("wine64", "wine"):
        if shutil.which(cand):
            return cand
    raise SystemExit("wine64/wine not found — install wine (win64) first")


def _postprocess_windows(built: Path, py_dir: Path) -> None:
    internal = built / "_internal"
    internal.mkdir(parents=True, exist_ok=True)
    embed_zip = py_dir / "python310.zip"
    if embed_zip.is_file():
        shutil.copy2(embed_zip, internal / "python310.zip")

    base = internal / "base_library.zip"
    hits = list(internal.glob("base_library.zip"))
    if hits:
        base = hits[0]
    if not base.is_file() or not embed_zip.is_file():
        print("[warn] skip base_library inject (missing zip)")
        return

    import zipfile as zfmod

    need_prefixes = (
        "pkgutil",
        "inspect",
        "copy",
        "pathlib",
        "typing",
        "contextlib",
        "dataclasses",
        "importlib",
        "json",
        "logging",
        "collections",
        "urllib",
        "email",
        "html",
        "xml",
        "encodings",
        "asyncio",
        "concurrent",
        "multiprocessing",
        "zoneinfo",
    )
    with zfmod.ZipFile(embed_zip, "r") as src, zfmod.ZipFile(base, "a") as dst:
        existing = set(dst.namelist())
        for name in src.namelist():
            top = name.split("/")[0].split(".")[0]
            if any(top == p or name.startswith(p + "/") or name.startswith(p + ".") for p in need_prefixes):
                if name not in existing:
                    dst.writestr(name, src.read(name))


def build_windows(*, skip_frontend: bool = False) -> Path:
    if not skip_frontend:
        prepare_frontend()
    else:
        if not (ROOT / "frontend" / "dist" / "index.html").is_file():
            prepare_frontend()

    wine = _wine_bin()
    tools = ROOT / ".tools"
    prefix = Path(os.environ.get("WINEPREFIX") or (ROOT / ".wine-rs-calib"))
    prefix.parent.mkdir(parents=True, exist_ok=True)
    py_dir = _ensure_win_python(tools)
    env = _wine_env(prefix)

    _run([wine, "wineboot", "--init"], env=env)

    py_wine = _linux_to_wine_path(py_dir / "python.exe")
    get_pip = _linux_to_wine_path(tools / "get-pip.py")
    _run([wine, py_wine, get_pip, "-i", PIP_INDEX], env=env)
    try:
        _run([wine, py_wine, "-m", "pip", "--version"], env=env)
    except subprocess.CalledProcessError as e:
        raise SystemExit("pip not available in Wine embed Python after get-pip") from e

    # Core + desktop deps (Windows wheels). Skip pyrealsense2/elite in Wine.
    pkgs = [
        "fastapi>=0.110",
        "uvicorn[standard]>=0.27",
        "PyYAML>=6.0",
        "opencv-python>=4.8,<5",
        "scipy>=1.10",
        "-r",
        _linux_to_wine_path(ROOT / "requirements-desktop.txt"),
    ]
    _run([wine, py_wine, "-m", "pip", "install", "-i", PIP_INDEX, *pkgs], env=env)
    _run([wine, py_wine, "-m", "pip", "install", "-i", PIP_INDEX, "numpy==1.23.5"], env=env)

    # Ensure sensors is importable during analysis via Z: vendor path
    vendor = ROOT / "vendor"
    if not (vendor / "sensors").is_dir():
        vendor.mkdir(parents=True, exist_ok=True)
        target = Path("/root/autodl-tmp/sensors/src/sensors")
        link = vendor / "sensors"
        if target.is_dir() and not link.exists():
            link.symlink_to(target)

    # Prefer data disk on AutoDL (overlay /tmp is often tight).
    scratch = Path(os.environ.get("RS_CALIB_BUILD_SCRATCH") or "/root/autodl-tmp/tmp")
    scratch.mkdir(parents=True, exist_ok=True)
    dist = scratch / "rs-calib-dist-win"
    work = scratch / "rs-calib-work-win"
    for d in (dist, work):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)

    runner = _linux_to_wine_path(ROOT / "packaging" / "run_pyinstaller_wine.py")
    spec = _linux_to_wine_path(ROOT / "packaging" / "rs-calib-desktop-win.spec")
    dist_w = _linux_to_wine_path(dist)
    work_w = _linux_to_wine_path(work)
    _run(
        [
            wine,
            py_wine,
            runner,
            "--noconfirm",
            "--clean",
            "--distpath",
            dist_w,
            "--workpath",
            work_w,
            spec,
        ],
        env=env,
    )

    built = dist / APP_EXE
    if not built.is_dir():
        raise SystemExit(f"PyInstaller output missing: {built}")

    _postprocess_windows(built, py_dir)

    # Sanity: frontend-dist
    fe = built / "_internal" / "frontend-dist" / "index.html"
    if not fe.is_file():
        # sometimes datas land differently
        alt = list(built.rglob("frontend-dist/index.html"))
        if not alt:
            raise SystemExit("missing _internal/frontend-dist/index.html in bundle")

    stamp = _utc_stamp()
    release = ROOT / "release" / f"{APP_ID}-desktop-windows-x64-{stamp}"
    if release.exists():
        shutil.rmtree(release)
    shutil.copytree(built, release)

    info = {
        "app_id": APP_ID,
        "platform": "windows-x64",
        "self_contained": True,
        "built_at": stamp,
        "entry": f"{APP_EXE}.exe",
        "notes": "Built on Linux via Wine + embeddable CPython + PyInstaller",
        "scheme": "A-adapted (FastAPI + static web UI)",
    }
    (release / "BUILD_INFO.json").write_text(json.dumps(info, indent=2) + "\n", encoding="utf-8")
    (release / "README.txt").write_text(
        "\n".join(
            [
                "rs-calib desktop (Windows x64) — Eye-to-Hand 场景相机标定",
                "",
                "1. 解压本目录",
                "2. 双击 rs-calib.exe",
                "3. 无需安装 Python / Node",
                "4. 用户数据：%APPDATA%\\rs-calib\\",
                "5. 桌面窗口需要 Microsoft Edge WebView2 Runtime",
                "   若窗口空白：安装 WebView2，或设置 RS_CALIB_BROWSER=1 用系统浏览器",
                "6. RealSense：目标机需安装 Intel RealSense SDK / 驱动，并 pip/侧载 pyrealsense2（本包未强制捆绑 USB 驱动）",
                "7. Elite EC616：需可用的 elite/elirobots Windows 包在 PYTHONPATH 或放入 _internal",
                "8. 默认会话目录：%APPDATA%\\rs-calib\\sessions\\",
                "",
            ]
        ),
        encoding="utf-8",
    )

    zip_path = Path(str(release) + ".zip")
    if zip_path.exists():
        zip_path.unlink()
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for path in release.rglob("*"):
            if path.is_file():
                zf.write(path, arcname=str(path.relative_to(release.parent)))
    print(f"release: {release}")
    print(f"zip: {zip_path}")
    return release


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Build rs-calib desktop package")
    p.add_argument("--target", choices=("windows",), default="windows")
    p.add_argument("--skip-frontend", action="store_true")
    args = p.parse_args(argv)
    if args.target == "windows":
        build_windows(skip_frontend=args.skip_frontend)
        return 0
    raise SystemExit(f"unsupported target: {args.target}")


if __name__ == "__main__":
    raise SystemExit(main())
