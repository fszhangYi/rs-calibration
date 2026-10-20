from __future__ import annotations

from pathlib import Path

from rs_calib.app_services.verify_service import VerifyService
from rs_calib.config.schema import AppConfig
from rs_calib.errors import Ok
from rs_calib.qt_compat import (
    QDialog,
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSlider,
    QTimer,
    QVBoxLayout,
    Qt,
)
from rs_calib.ui.preview_label import PreviewLabel


class VerifyDialog(QDialog):
    def __init__(self, cfg: AppConfig, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.svc = VerifyService(cfg)
        self.setWindowTitle("验证（A=运动学+外参 TCP / B=视觉 TCP；主误差=预测板 vs 检测板）")
        self.resize(960, 640)
        layout = QVBoxLayout(self)

        def path_row(label: str):
            row = QHBoxLayout()
            edit = QLineEdit()
            btn = QPushButton(label)
            row.addWidget(edit)
            row.addWidget(btn)
            layout.addLayout(row)
            return edit, btn

        self.session_edit, b1 = path_row("session 目录")
        self.ext_edit, b2 = path_row("extrinsics.json")
        b1.clicked.connect(lambda: self._browse_dir(self.session_edit))
        b2.clicked.connect(lambda: self._browse_file(self.ext_edit))

        load_btn = QPushButton("加载")
        load_btn.clicked.connect(self._load)
        layout.addWidget(load_btn)

        self.preview = PreviewLabel()
        layout.addWidget(self.preview, stretch=1)

        ctrl = QHBoxLayout()
        self.play_btn = QPushButton("播放")
        self.play_btn.clicked.connect(self._toggle_play)
        self.slider = QSlider(Qt.Horizontal if hasattr(Qt, "Horizontal") else Qt.Orientation.Horizontal)
        self.slider.valueChanged.connect(self._on_slider)
        self.err_label = QLabel("误差: -")
        ctrl.addWidget(self.play_btn)
        ctrl.addWidget(self.slider)
        ctrl.addWidget(self.err_label)
        layout.addLayout(ctrl)

        self._timer = QTimer(self)
        self._timer.setInterval(66)
        self._timer.timeout.connect(self._tick)
        self._playing = False

    def _browse_dir(self, edit: QLineEdit) -> None:
        d = QFileDialog.getExistingDirectory(self, "选择目录")
        if d:
            edit.setText(d)

    def _browse_file(self, edit: QLineEdit) -> None:
        f, _ = QFileDialog.getOpenFileName(self, "选择外参", filter="JSON (*.json)")
        if f:
            edit.setText(f)

    def _load(self) -> None:
        res = self.svc.load(Path(self.session_edit.text()), Path(self.ext_edit.text()))
        if isinstance(res, Ok):
            n = res.value
            self.slider.setMaximum(max(0, n - 1))
            self.slider.setValue(0)
            self._render(0)
        else:
            self.err_label.setText(str(res.error))

    def _on_slider(self, v: int) -> None:
        self._render(v)

    def _render(self, idx: int) -> None:
        img, metrics = self.svc.render(idx)
        if img is not None:
            self.preview.show_bgr(img)
        if metrics is not None and metrics.has_detect:
            self.err_label.setText(
                f"帧{idx} 板误差 Δt={metrics.marker_err_t_mm:.2f}mm ΔR={metrics.marker_err_R_deg:.2f}°"
            )
        elif metrics is not None:
            self.err_label.setText(f"帧{idx} 未检测到板")

    def _toggle_play(self) -> None:
        self._playing = not self._playing
        self.play_btn.setText("暂停" if self._playing else "播放")
        if self._playing:
            self._timer.start()
        else:
            self._timer.stop()

    def _tick(self) -> None:
        v = self.slider.value() + 1
        if v > self.slider.maximum():
            v = 0
        self.slider.setValue(v)

    def closeEvent(self, event) -> None:  # noqa: N802
        self._timer.stop()
        self.svc.close()
        super().closeEvent(event)
