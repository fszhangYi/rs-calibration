from __future__ import annotations

from rs_calib.config.schema import AppConfig
from rs_calib.qt_compat import (
    QCheckBox,
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

        def add_float(key, label, value, step=0.1, vmin=-1e6, vmax=1e6, decimals=4):
            sp = QDoubleSpinBox()
            sp.setRange(vmin, vmax)
            sp.setDecimals(decimals)
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

        self.max_delta = QDoubleSpinBox()
        self.max_delta.setRange(0.01, 45.0)
        self.max_delta.setDecimals(2)
        self.max_delta.setSingleStep(0.5)
        self.max_delta.setSuffix(" °")
        self.max_delta.setValue(float(cfg.arm.max_delta_deg))
        self.max_delta.setToolTip("单次关节指令 |Δq| 上限；矩阵走位会按此值自动切段")
        form.addRow("arm.max_delta_deg", self.max_delta)

        self.v_norm = QDoubleSpinBox()
        self.v_norm.setRange(0.001, 5.0)
        self.v_norm.setDecimals(3)
        self.v_norm.setSingleStep(0.01)
        self.v_norm.setSuffix(" rad/s")
        self.v_norm.setValue(float(getattr(cfg.arm, "v_norm_rad_s", 0.02)))
        self.v_norm.setToolTip("同 sensors-arm：T=clamp(peakΔq/v_norm, t_min, t_max)")
        form.addRow("arm.v_norm_rad_s", self.v_norm)

        add_float("t_min_s", "arm.t_min_s", getattr(cfg.arm, "t_min_s", 0.1), 0.05, 0.02, 30.0, 2)
        add_float("t_max_s", "arm.t_max_s", getattr(cfg.arm, "t_max_s", 30.0), 1.0, 0.1, 30.0, 2)

        # board / charuco — show fields matching selected board_type
        a = cfg.aruco
        self.board_type = QComboBox()
        self.board_type.addItem("ChArUco 板", "charuco")
        self.board_type.addItem("单 ArUco 码", "aruco")
        bi = self.board_type.findData(getattr(a, "board_type", "charuco"))
        self.board_type.setCurrentIndex(bi if bi >= 0 else 0)
        self.board_type.currentIndexChanged.connect(self._sync_board_rows)
        form.addRow("aruco.board_type", self.board_type)

        self.dict_edit = QLineEdit(str(a.dict))
        form.addRow("aruco.dict", self.dict_edit)

        self._board_form = form
        self._board_row_keys = {
            "aruco": ["marker_id", "marker_length_m"],
            "charuco": [
                "squares_x",
                "squares_y",
                "square_length_m",
                "charuco_marker_length_m",
                "min_charuco_corners",
            ],
        }
        add_int("marker_id", "aruco.marker_id", a.marker_id, 0, 10000)
        add_float(
            "marker_length_m",
            "aruco.marker_length_m (m)",
            a.marker_length_m,
            0.005,
            0.001,
            2.0,
            4,
        )
        add_int("squares_x", "charuco.squares_x", getattr(a, "squares_x", 5), 2, 50)
        add_int("squares_y", "charuco.squares_y", getattr(a, "squares_y", 7), 2, 50)
        add_float(
            "square_length_m",
            "charuco.square_length_m",
            getattr(a, "square_length_m", 0.04),
            0.001,
            0.001,
            1.0,
            4,
        )
        add_float(
            "charuco_marker_length_m",
            "charuco.marker_length_m",
            getattr(a, "charuco_marker_length_m", 0.03),
            0.001,
            0.001,
            1.0,
            4,
        )
        add_int(
            "min_charuco_corners",
            "charuco.min_corners",
            getattr(a, "min_charuco_corners", 6),
            4,
            200,
        )
        self._sync_board_rows()

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

        self.dual_backends = QCheckBox("同时跑 OpenCV calibrateHandEye + 原生（主解=OpenCV）")
        self.dual_backends.setChecked(bool(getattr(cfg.handeye, "dual_backends", False)))
        try:
            from rs_calib.domain.handeye import opencv_handeye_status

            st = opencv_handeye_status()
            self.dual_backends.setToolTip(
                f"OpenCV {st.get('cv_version')}: {st.get('detail')}\n"
                "建议 opencv-python≥4.8,<5"
            )
        except Exception:  # noqa: BLE001
            pass
        form.addRow("handeye.dual_backends", self.dual_backends)

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

    def _sync_board_rows(self, _idx: int = 0) -> None:
        bt = self.board_type.currentData() or "charuco"
        show = set(self._board_row_keys.get(bt, []))
        all_keys = self._board_row_keys["aruco"] + self._board_row_keys["charuco"]
        for key in all_keys:
            w = self._widgets.get(key)
            if w is None:
                continue
            visible = key in show
            w.setVisible(visible)
            label = self._board_form.labelForField(w)
            if label is not None:
                label.setVisible(visible)

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
        self.cfg.arm.max_delta_deg = float(self.max_delta.value())
        self.cfg.arm.v_norm_rad_s = float(self.v_norm.value())
        self.cfg.arm.t_min_s = float(self._widgets["t_min_s"].value())
        self.cfg.arm.t_max_s = float(self._widgets["t_max_s"].value())

        bt = self.board_type.currentData()
        if bt:
            self.cfg.aruco.board_type = str(bt)
        self.cfg.aruco.dict = self.dict_edit.text().strip() or "DICT_5X5_50"
        self.cfg.aruco.marker_id = int(self._widgets["marker_id"].value())
        self.cfg.aruco.marker_length_m = float(self._widgets["marker_length_m"].value())
        self.cfg.aruco.squares_x = int(self._widgets["squares_x"].value())
        self.cfg.aruco.squares_y = int(self._widgets["squares_y"].value())
        self.cfg.aruco.square_length_m = float(self._widgets["square_length_m"].value())
        self.cfg.aruco.charuco_marker_length_m = float(
            self._widgets["charuco_marker_length_m"].value()
        )
        self.cfg.aruco.min_charuco_corners = int(self._widgets["min_charuco_corners"].value())

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
        self.cfg.handeye.dual_backends = bool(self.dual_backends.isChecked())
