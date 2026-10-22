"""Ensure ``cv2.calibrateHandEye`` works (OpenCV 4.x). Auto-repair common OpenCV 5 gap."""

from __future__ import annotations

import subprocess
import sys
from typing import Any


def _probe() -> dict[str, Any]:
    try:
        import cv2
    except Exception as e:  # noqa: BLE001
        return {"ok": False, "version": None, "has_handeye": False, "error": str(e)}
    fn = getattr(cv2, "calibrateHandEye", None)
    return {
        "ok": True,
        "version": str(getattr(cv2, "__version__", "?")),
        "has_handeye": callable(fn),
        "error": None,
    }


def _pip_reinstall_opencv4() -> tuple[bool, str]:
    cmds = [
        [sys.executable, "-m", "pip", "uninstall", "-y",
         "opencv-python", "opencv-python-headless",
         "opencv-contrib-python", "opencv-contrib-python-headless"],
        [sys.executable, "-m", "pip", "install", "opencv-python>=4.8,<5"],
    ]
    logs: list[str] = []
    for cmd in cmds:
        try:
            p = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                encoding="utf-8",
                errors="replace",
                check=False,
            )
            logs.append(f"$ {' '.join(cmd)}\nexit={p.returncode}\n{p.stdout}\n{p.stderr}")
            # uninstall may return non-zero if package absent — continue
            if "install" in cmd and p.returncode != 0:
                return False, "\n".join(logs)
        except Exception as e:  # noqa: BLE001
            return False, f"{' '.join(cmd)} failed: {e}\n" + "\n".join(logs)
    # Drop cached import so next probe sees the new wheel.
    for name in list(sys.modules):
        if name == "cv2" or name.startswith("cv2."):
            del sys.modules[name]
    return True, "\n".join(logs)


def ensure_opencv_handeye(*, repair: bool = True) -> dict[str, Any]:
    """Return status dict; if ``repair`` and HandEye missing, reinstall OpenCV 4.x.

    Keys: ok, version, has_handeye, repaired, message, log
    """
    probe = _probe()
    if probe.get("has_handeye"):
        return {
            "ok": True,
            "version": probe.get("version"),
            "has_handeye": True,
            "repaired": False,
            "message": f"OpenCV {probe.get('version')} · calibrateHandEye OK",
            "log": "",
        }

    if not repair:
        return {
            "ok": False,
            "version": probe.get("version"),
            "has_handeye": False,
            "repaired": False,
            "message": (
                f"OpenCV {probe.get('version')} 无 calibrateHandEye"
                f"（{probe.get('error') or '常见于 5.0 轮子'}）"
            ),
            "log": "",
        }

    print(
        f"[rs-calib] OpenCV {probe.get('version')} 无 calibrateHandEye，"
        "正在安装 opencv-python>=4.8,<5 …",
        flush=True,
    )
    ok_pip, log = _pip_reinstall_opencv4()
    probe2 = _probe()
    if ok_pip and probe2.get("has_handeye"):
        return {
            "ok": True,
            "version": probe2.get("version"),
            "has_handeye": True,
            "repaired": True,
            "message": f"已修复为 OpenCV {probe2.get('version')} · calibrateHandEye OK",
            "log": log,
        }
    return {
        "ok": False,
        "version": probe2.get("version"),
        "has_handeye": bool(probe2.get("has_handeye")),
        "repaired": False,
        "message": (
            "无法自动修复 OpenCV HandEye。请手动执行: "
            'pip install "opencv-python>=4.8,<5"'
        ),
        "log": log,
    }


def main() -> int:
    st = ensure_opencv_handeye(repair=True)
    print(st["message"])
    if st.get("log") and not st.get("ok"):
        print(st["log"][-2000:])
    return 0 if st.get("ok") else 1


if __name__ == "__main__":
    raise SystemExit(main())
