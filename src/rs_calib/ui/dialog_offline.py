from __future__ import annotations

from pathlib import Path

from rs_calib.app_services.offline_calib_service import OfflineCalibService
from rs_calib.config.schema import AppConfig
from rs_calib.errors import Ok
from rs_calib.qt_compat import (
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QTextEdit,
    QVBoxLayout,
)


class OfflineDialog(QDialog):
    def __init__(self, cfg: AppConfig, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.setWindowTitle("离线智能计算")
        self.resize(640, 420)
        layout = QVBoxLayout(self)

        row = QHBoxLayout()
        self.path_edit = QLineEdit()
        btn = QPushButton("选择 session 目录")
        btn.clicked.connect(self._browse)
        row.addWidget(self.path_edit)
        row.addWidget(btn)
        layout.addLayout(row)

        self.run_btn = QPushButton("开始计算")
        self.run_btn.clicked.connect(self._run)
        layout.addWidget(self.run_btn)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)
        layout.addWidget(QLabel("图例：滤波统计与外参将写入 session/extrinsics.json"))

    def _browse(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "选择 session 目录")
        if d:
            self.path_edit.setText(d)

    def _run(self) -> None:
        path = self.path_edit.text().strip()
        if not path:
            self.log.append("请先选择目录")
            return
        self.log.append(f"运行: {path}")
        svc = OfflineCalibService(self.cfg)

        def progress(i, n):
            if i == n or i % 10 == 0:
                self.log.append(f"进度 {i}/{n}")

        result = svc.run(Path(path), progress_cb=progress)
        if isinstance(result, Ok):
            data = result.value
            r = data["result"]
            rep = data["report"]
            self.log.append(
                f"成功 method={r.method} n={r.n_samples} selected={rep.n_selected} "
                f"raw_detect={rep.n_after_detect}"
            )
            self.log.append(f"写出: {data['extrinsics_path']}")
            for w in data.get("warnings") or []:
                self.log.append(f"警告: {w}")
            T = r.T_base_cam
            self.log.append(f"T_base_cam:\n{T}")
        else:
            self.log.append(str(result.error))
