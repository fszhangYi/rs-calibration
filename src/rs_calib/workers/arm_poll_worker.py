from __future__ import annotations

import time

from rs_calib.app_services.arm_service import ArmService
from rs_calib.qt_compat import QThread, pyqtSignal


class ArmPollWorker(QThread):
    state_ready = pyqtSignal(object)

    def __init__(self, arm: ArmService, hz: float = 50.0, parent=None):
        super().__init__(parent)
        self.arm = arm
        self.hz = max(1.0, float(hz))
        self._running = False

    def run(self) -> None:
        self._running = True
        period = 1.0 / self.hz
        while self._running:
            t0 = time.perf_counter()
            if self.arm.adapter.is_connected:
                st = self.arm.poll()
                self.state_ready.emit(st)
            elapsed = time.perf_counter() - t0
            time.sleep(max(0.0, period - elapsed))

    def stop(self) -> None:
        self._running = False
