from __future__ import annotations

import time

from rs_calib.app_services.camera_service import CameraService
from rs_calib.qt_compat import QThread, pyqtSignal


class CameraWorker(QThread):
    """Poll all open cameras; emit frame bundle for recording + preview serial image."""

    # images: dict[serial, ndarray], ts_ref: float, preview_serial: str|None
    frames_ready = pyqtSignal(object, float, object)
    # backward-compat alias used if old slots still connected
    frame_ready = pyqtSignal(object, float)  # ndarray, ts (preview only)
    failed = pyqtSignal(str)

    def __init__(self, camera: CameraService, parent=None):
        super().__init__(parent)
        self.camera = camera
        self._running = False

    @property
    def adapter(self):
        return self.camera.adapter

    def run(self) -> None:
        self._running = True
        while self._running:
            if self.camera.n_open <= 0:
                time.sleep(0.05)
                continue
            grabbed = self.camera.grab_all(1000)
            if not grabbed:
                continue
            images = {s: img for s, (_ts, img) in grabbed.items()}
            # reference timestamp: preview cam if present, else mean/first
            preview = self.camera.preview_serial
            if preview and preview in grabbed:
                ts = grabbed[preview][0]
            else:
                ts = next(iter(grabbed.values()))[0]
            self.frames_ready.emit(images, ts, preview)
            if preview and preview in images:
                self.frame_ready.emit(images[preview], ts)
            elif images:
                first_s = next(iter(images))
                self.frame_ready.emit(images[first_s], ts)

    def stop(self) -> None:
        self._running = False
