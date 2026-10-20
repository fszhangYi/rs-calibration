"""Desktop entry: local uvicorn + system browser / pywebview."""

from __future__ import annotations

import os
import socket
import sys
import threading
import time
import urllib.request
import webbrowser


def _pick_port(preferred: int = 6006) -> int:
    env = os.environ.get("RS_CALIB_PORT") or os.environ.get("EOAT_PORT")
    if env:
        return int(env)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        try:
            s.bind(("127.0.0.1", preferred))
            return preferred
        except OSError:
            s.bind(("127.0.0.1", 0))
            return int(s.getsockname()[1])


def _wait_health(url: str, timeout: float = 45.0) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=1.5) as resp:
                if 200 <= getattr(resp, "status", 200) < 500:
                    return True
        except Exception:
            time.sleep(0.25)
    return False


def _start_webview(url: str) -> bool:
    try:
        import webview  # type: ignore
    except Exception as exc:  # noqa: BLE001
        print(f"[rs-calib] pywebview unavailable ({exc}); falling back to browser")
        return False

    from rs_calib.paths import APP_TITLE, user_data_dir

    storage = user_data_dir() / "webview"
    storage.mkdir(parents=True, exist_ok=True)
    window_kwargs = {
        "title": APP_TITLE,
        "url": url,
        "width": 1360,
        "height": 860,
        "background_color": "#0f1419",
        "text_select": True,
    }
    start_kwargs: dict = {"private_mode": False, "storage_path": str(storage)}
    gui = (os.environ.get("RS_CALIB_WEBVIEW_GUI") or "").strip().lower()
    if not gui and sys.platform == "win32":
        gui = "edgechromium"
    if gui:
        start_kwargs["gui"] = gui
    try:
        webview.create_window(**window_kwargs)
        webview.start(**start_kwargs)
        return True
    except Exception as exc:  # noqa: BLE001
        print(f"[rs-calib] webview start failed ({exc}); falling back to browser")
        try:
            start_kwargs.pop("gui", None)
            webview.create_window(**window_kwargs)
            webview.start(**start_kwargs)
            return True
        except Exception as exc2:  # noqa: BLE001
            print(f"[rs-calib] webview retry failed ({exc2})")
            return False


def main() -> None:
    if getattr(sys, "frozen", False):
        try:
            import numpy as _np  # noqa: F401
        except Exception as exc:  # noqa: BLE001
            print(f"[rs-calib] numpy import failed: {exc!r}", file=sys.stderr)
            raise SystemExit(2) from exc

    from rs_calib.paths import ensure_runtime_env

    ensure_runtime_env(desktop=True)

    import uvicorn

    from rs_calib.web.server import create_app

    app = create_app()
    host = "127.0.0.1"
    port = _pick_port(6006)
    url = f"http://{host}:{port}/"
    health = f"http://{host}:{port}/api/health"

    def _serve() -> None:
        uvicorn.run(app, host=host, port=port, log_level="info")

    thread = threading.Thread(target=_serve, name="rs-calib-uvicorn", daemon=True)
    thread.start()

    if not _wait_health(health):
        print(f"[rs-calib] health check failed: {health}", file=sys.stderr)
        sys.exit(1)

    print(f"[rs-calib] open {url}")
    force_browser = (os.environ.get("RS_CALIB_BROWSER") or "").strip().lower() in {
        "1",
        "true",
        "yes",
        "browser",
    }
    opened = False
    if not force_browser:
        opened = _start_webview(url)
    if not opened:
        webbrowser.open(url)
        try:
            while thread.is_alive():
                time.sleep(0.5)
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
