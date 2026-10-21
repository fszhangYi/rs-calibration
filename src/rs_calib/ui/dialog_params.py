from __future__ import annotations

from rs_calib.config.schema import AppConfig
from rs_calib.qt_compat import (
    QComboBox,
    QDialog,
    QDialogButtonBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)


class ParamsDialog(QDialog):
    def __init__(self, cfg: AppConfig, parent=None):
        super().__init__(parent)
        self.setWindowTitle("参数")
        self.cfg = cfg
        self._widgets: dict = {}
        layout = QVBoxLayout(self)
        form = QFormLayout()

        def add_float(key, label, value, step=0.1, vmin=-1e6, vmax=1e6):
            sp = QDoubleSpinBox()
            sp.setRange(vmin, vmax)
            sp.setDecimals(4)
            sp.setSingleStep(step)
            sp.setValue(float(value))
            form.addRow(label, sp)
            self._widgets[key] = sp
            return sp

        def add_int(key, label, value, vmin=0, vmax=100000):
            sp = QSpinBox()
            sp.setRange(vmin, vmax)
            sp.setValue(int(value))
            form.addRow(label, sp)
            self._widgets[key] = sp
            return sp

        # tcp
        self.tcp_edits = []
        row = QHBoxLayout()
        for i, name in enumerate("xyz"):
            sp = QDoubleSpinBox()
            sp.setRange(-2, 2)
            sp.setDecimals(4)
            sp.setSingleStep(0.01)
            sp.setValue(float(cfg.tcp_xyz[i]))
            row.addWidget(sp)
            self.tcp_edits.append(sp)
        form.addRow("tcp_xyz (m)", row)

        c = cfg.convergence
        add_int("min_samples", "min_samples", c.min_samples, 3, 1000)
        add_int("recompute_every_n", "recompute_every_n", c.recompute_every_n, 1, 100)
        add_float("max_delta_t_mm", "max_delta_t_mm", c.max_delta_t_mm)
        add_float("max_delta_R_deg", "max_delta_R_deg", c.max_delta_R_deg)
        add_int("stable_count", "stable_count", c.stable_count, 1, 100)
        add_float("dup_joint_eps_deg", "dup_joint_eps_deg", c.dup_joint_eps_deg)
        add_float("dup_pose_eps_mm", "dup_pose_eps_mm", c.dup_pose_eps_mm)
        add_float("dup_pose_eps_deg", "dup_pose_eps_deg", c.dup_pose_eps_deg)

        self.setup = QComboBox()
        self.setup.addItem("eye_to_hand（眼在手外）", "eye_to_hand")
        self.setup.addItem("eye_in_hand（眼在手上）", "eye_in_hand")
        si = self.setup.findData(getattr(cfg.handeye, "setup", "eye_to_hand"))
        self.setup.setCurrentIndex(si if si >= 0 else 0)
        form.addRow("handeye.setup", self.setup)

        self.method = QLineEdit(cfg.handeye.method)
        form.addRow("handeye.method", self.method)

        layout.addLayout(form)
        reset_btn = QPushButton("恢复收敛推荐值")
        reset_btn.clicked.connect(self._reset_conv)
        layout.addWidget(reset_btn)

        try:
            flags = QDialogButtonBox.Ok | QDialogButtonBox.Cancel
        except AttributeError:  # PyQt6
            flags = QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        buttons = QDialogButtonBox(flags)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

    def _reset_conv(self) -> None:
        self.cfg.reset_convergence_defaults()
        c = self.cfg.convergence
        mapping = {
            "min_samples": c.min_samples,
            "recompute_every_n": c.recompute_every_n,
            "max_delta_t_mm": c.max_delta_t_mm,
            "max_delta_R_deg": c.max_delta_R_deg,
            "stable_count": c.stable_count,
            "dup_joint_eps_deg": c.dup_joint_eps_deg,
            "dup_pose_eps_mm": c.dup_pose_eps_mm,
            "dup_pose_eps_deg": c.dup_pose_eps_deg,
        }
        for k, v in mapping.items():
            w = self._widgets[k]
            w.setValue(v)

    def apply(self) -> None:
        self.cfg.tcp_xyz = [sp.value() for sp in self.tcp_edits]
        c = self.cfg.convergence
        c.min_samples = int(self._widgets["min_samples"].value())
        c.recompute_every_n = int(self._widgets["recompute_every_n"].value())
        c.max_delta_t_mm = float(self._widgets["max_delta_t_mm"].value())
        c.max_delta_R_deg = float(self._widgets["max_delta_R_deg"].value())
        c.stable_count = int(self._widgets["stable_count"].value())
        c.dup_joint_eps_deg = float(self._widgets["dup_joint_eps_deg"].value())
        c.dup_pose_eps_mm = float(self._widgets["dup_pose_eps_mm"].value())
        c.dup_pose_eps_deg = float(self._widgets["dup_pose_eps_deg"].value())
        setup = self.setup.currentData()
        if setup:
            self.cfg.handeye.setup = str(setup)
        self.cfg.handeye.method = self.method.text().strip().upper() or "TSAI"
