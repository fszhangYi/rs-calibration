from __future__ import annotations

from typing import Callable, Optional, Sequence

from rs_calib.app_services.matrix_path_service import MatrixPathService
from rs_calib.domain.matrix_path import MatrixWaypoint
from rs_calib.errors import Ok
from rs_calib.qt_compat import QThread, pyqtSignal


class MatrixPathWorker(QThread):
    progress = pyqtSignal(int, int, str, str)  # i, n, label, phase
    finished_ok = pyqtSignal(object)  # result dict
    failed = pyqtSignal(str)
    converged_stop = pyqtSignal()

    def __init__(
        self,
        service: MatrixPathService,
        waypoints: Sequence[MatrixWaypoint],
        *,
        frame: str = "flange",
        dwell_s: float = 0.8,
        extra_cancel: Optional[Callable[[], bool]] = None,
        parent=None,
    ):
        super().__init__(parent)
        self.service = service
        self.waypoints = list(waypoints)
        self.frame = frame
        self.dwell_s = dwell_s
        self.extra_cancel = extra_cancel

    def run(self) -> None:
        def cb(i: int, n: int, wp: MatrixWaypoint, phase: str) -> None:
            self.progress.emit(i, n, wp.label, phase)

        res = self.service.execute(
            self.waypoints,
            frame=self.frame,
            dwell_s=self.dwell_s,
            progress_cb=cb,
            extra_cancel=self.extra_cancel,
        )
        if isinstance(res, Ok):
            self.finished_ok.emit(res.value)
        else:
            err = res.error
            if getattr(err, "code", None) == "PATH_CONVERGED" or self.service.cancel_reason == "converge":
                self.converged_stop.emit()
            self.failed.emit(str(err))

    def request_cancel(self) -> None:
        self.service.request_cancel("user")
