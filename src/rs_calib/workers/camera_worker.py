from __future__ import annotations

import time

from rs_calib.infra.realsense_adapter import RealSenseAdapter
from rs_calib.qt_compat import QThread, pyqtSignal


class CameraWorker(QThread):
    frame_ready = pyqtSignal(object, float)  # ndarray, ts
    failed = pyqtSignal(str)

    def __init__(self, adapter: RealSenseAdapter, parent=None):
        super().__init__(parent)
        self.adapter = adapter
        self._running = False

    def run(self) -> None:
        self._running = True
        while self._running:
            if not self.adapter.is_open:
                time.sleep(0.05)
                continue
            grabbed = self.adapter.grab(1000)
            if grabbed is None:
                continue
            ts, img = grabbed
            self.frame_ready.emit(img, ts)

    def stop(self) -> None:
        self._running = False
