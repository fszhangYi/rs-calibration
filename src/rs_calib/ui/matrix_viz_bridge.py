"""Serve Cell-A EC616 matrix-path Three.js canvas for the desktop dialog.

Uses the same static assets as the web UI. Prefers an embedded QWebEngineView
when PyQtWebEngine is installed; otherwise opens the system browser against a
local ThreadingHTTPServer.
"""

from __future__ import annotations

import json
import socket
import threading
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Optional
from urllib.parse import urlparse

from rs_calib.paths import resolve_frontend_dist
from rs_calib.qt_compat import QHBoxLayout, QLabel, QPushButton, QVBoxLayout, QWidget, Qt


def _pick_free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


class _VizState:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.payload: dict[str, Any] = {
            "waypoints": [],
            "seed_xyzrpy": None,
            "joints_rad": None,
            "progress": None,
        }

    def update(self, **kwargs: Any) -> None:
        with self.lock:
            self.payload.update(kwargs)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return dict(self.payload)


class _Handler(SimpleHTTPRequestHandler):
    static_root: Path = Path(".")
    state: _VizState = _VizState()

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(self.static_root), **kwargs)

    def log_message(self, format: str, *args) -> None:  # noqa: A003
        return

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if path in ("/waypoints.json", "waypoints.json"):
            body = json.dumps(self.state.snapshot(), ensure_ascii=False).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if path in ("/", "/index.html"):
            self.path = "/matrix_path_viz.html"
        return super().do_GET()


class MatrixVizBridge:
    """Background static server + optional embedded WebEngine widget."""

    def __init__(self) -> None:
        self.state = _VizState()
        self._httpd: Optional[ThreadingHTTPServer] = None
        self._thread: Optional[threading.Thread] = None
        self.port: Optional[int] = None
        self.static_root = resolve_frontend_dist()

    @property
    def url(self) -> str:
        if not self.port:
            raise RuntimeError("viz server not started")
        return f"http://127.0.0.1:{self.port}/matrix_path_viz.html"

    def start(self) -> str:
        if self._httpd is not None:
            return self.url
        if not self.static_root.is_dir():
            raise FileNotFoundError(f"frontend static missing: {self.static_root}")
        port = _pick_free_port()
        _Handler.static_root = self.static_root
        _Handler.state = self.state
        httpd = ThreadingHTTPServer(("127.0.0.1", port), _Handler)
        self._httpd = httpd
        self.port = port
        t = threading.Thread(target=httpd.serve_forever, daemon=True)
        self._thread = t
        t.start()
        return self.url

    def stop(self) -> None:
        httpd = self._httpd
        self._httpd = None
        if httpd is not None:
            try:
                httpd.shutdown()
            except Exception:  # noqa: BLE001
                pass
            try:
                httpd.server_close()
            except Exception:  # noqa: BLE001
                pass
        self.port = None

    def publish(
        self,
        *,
        waypoints: Optional[list] = None,
        seed_xyzrpy: Optional[list[float]] = None,
        joints_rad: Optional[list[float]] = None,
        progress: Optional[dict] = None,
    ) -> None:
        kw: dict[str, Any] = {}
        if waypoints is not None:
            kw["waypoints"] = [
                {
                    "index": getattr(w, "index", i),
                    "label": getattr(w, "label", str(i)),
                    "xyzrpy": list(getattr(w, "xyzrpy", w if isinstance(w, (list, tuple)) else [])),
                }
                for i, w in enumerate(waypoints)
            ]
        if seed_xyzrpy is not None:
            kw["seed_xyzrpy"] = list(seed_xyzrpy)
        if joints_rad is not None:
            kw["joints_rad"] = list(joints_rad)
        if progress is not None:
            kw["progress"] = progress
        if kw:
            self.state.update(**kw)


def try_make_webengine_view(parent=None):
    """Return (view, load_url_callable) or (None, None) if WebEngine unavailable."""
    try:
        from PyQt5.QtCore import QUrl  # type: ignore
        from PyQt5.QtWebEngineWidgets import QWebEngineView  # type: ignore
    except ImportError:
        try:
            from PyQt6.QtCore import QUrl  # type: ignore
            from PyQt6.QtWebEngineWidgets import QWebEngineView  # type: ignore
        except ImportError:
            return None, None

    view = QWebEngineView(parent)

    def load_url(url: str) -> None:
        view.load(QUrl(url))

    return view, load_url


class MatrixVizPanel(QWidget):
    """Embed EC616 canvas when possible; otherwise offer browser open."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.bridge = MatrixVizBridge()
        self._url: Optional[str] = None
        self._view = None
        self._load_url = None

        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        tip = QLabel("三维画布 · Cell A Elite EC616（生成点位后刷新）")
        tip.setWordWrap(True)
        root.addWidget(tip)

        row = QHBoxLayout()
        self.btn_open = QPushButton("在浏览器打开画布")
        self.btn_open.clicked.connect(self.open_browser)
        row.addWidget(self.btn_open)
        root.addLayout(row)

        self._view, self._load_url = try_make_webengine_view(self)
        if self._view is not None:
            self._view.setMinimumHeight(280)
            root.addWidget(self._view, 1)
            self._status = QLabel("已嵌入 WebEngine · Cell A EC616")
            root.addWidget(self._status)
        else:
            self._status = QLabel("未安装 PyQtWebEngine — 生成点位后点「在浏览器打开画布」")
            align = getattr(Qt, "AlignCenter", None) or getattr(Qt, "AlignmentFlag").AlignCenter
            self._status.setAlignment(align)
            self._status.setMinimumHeight(120)
            self._status.setStyleSheet(
                "background:#070c14;color:#8b9bb4;border:1px solid #2a3542;border-radius:6px;"
            )
            root.addWidget(self._status, 1)

    def ensure_started(self) -> str:
        if not self._url:
            self._url = self.bridge.start()
            if self._load_url is not None:
                self._load_url(self._url)
        return self._url

    def open_browser(self) -> None:
        from rs_calib.qt_compat import QApplication

        url = self.ensure_started()
        # Prefer Qt openUrl if available
        try:
            from PyQt5.QtCore import QUrl  # type: ignore
            from PyQt5.QtGui import QDesktopServices  # type: ignore

            QDesktopServices.openUrl(QUrl(url))
            return
        except ImportError:
            pass
        try:
            from PyQt6.QtCore import QUrl  # type: ignore
            from PyQt6.QtGui import QDesktopServices  # type: ignore

            QDesktopServices.openUrl(QUrl(url))
            return
        except ImportError:
            pass
        import webbrowser

        webbrowser.open(url)
        _ = QApplication.instance()

    def show_waypoints(
        self,
        waypoints: list,
        seed_xyzrpy: Optional[list[float]] = None,
        joints_rad: Optional[list[float]] = None,
    ) -> None:
        self.ensure_started()
        self.bridge.publish(
            waypoints=waypoints,
            seed_xyzrpy=seed_xyzrpy,
            joints_rad=joints_rad,
        )
        if self._view is None:
            self._status.setText(f"已发布 {len(waypoints)} 点 — 点「在浏览器打开画布」查看 EC616")

    def set_progress(self, i: int, n: int, label: str = "", phase: str = "") -> None:
        self.bridge.publish(progress={"i": i, "n": n, "label": label, "phase": phase})

    def set_joints(self, joints_rad: Optional[list[float]]) -> None:
        if joints_rad is not None:
            self.bridge.publish(joints_rad=joints_rad)

    def shutdown(self) -> None:
        self.bridge.stop()
