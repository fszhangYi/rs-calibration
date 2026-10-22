from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np

from rs_calib.app_services.verify_service import VerifyService
from rs_calib.config.schema import AppConfig
from rs_calib.domain.math3d import T_from_xyzrpy, xyzrpy_from_T
from rs_calib.domain.types import Intrinsics
from rs_calib.errors import Ok
from rs_calib.qt_compat import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QTimer,
    QVBoxLayout,
    QWidget,
    Qt,
)
from rs_calib.ui.preview_label import PreviewLabel

# Default flange→board from Park re-solve of session_20261022_193810 (ETH fixture).
_DEFAULT_TFM_XYZ_MM = (2.3, -43.4, 157.9)
_DEFAULT_TFM_RPY_DEG = (90.7, 2.5, -2.0)


class VerifyDialog(QDialog):
    """Verify extrinsics against session video.

    Intrinsics / ArUco / setup are loaded from ``meta.json`` + ``extrinsics.json``
    and shown in editable fields (overrides applied on 加载).
    """

    def __init__(self, cfg: AppConfig, parent=None):
        super().__init__(parent)
        self.cfg = cfg
        self.svc = VerifyService(cfg)
        self.setWindowTitle("验证 · 双色坐标轴")
        self.resize(1040, 720)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 8, 8, 8)

        # Scrollable form (tip / paths / params / load buttons) so short screens fit.
        form_host = QWidget()
        form_l = QVBoxLayout(form_host)
        form_l.setContentsMargins(0, 0, 8, 0)

        tip = QLabel(
            "默认从 session 的 meta.json / extrinsics.json 读取内参、ArUco、手眼模式；"
            "下方可核对并覆盖后点「加载」。\n"
            "主画面：A=外参预测板(RGB)  B=检测板(CMY)，应叠在 ArUco 上；"
            "主误差=二者位姿差。\n"
            "T_flange_marker 只影响绝对画图/误差，不改已求外参；"
            "平移填法兰坐标系下板心相对法兰原点（mm），板平放时常需 Rx≈90°；"
            "改完拖动滑条即刷新。"
        )
        tip.setWordWrap(True)
        form_l.addWidget(tip)

        paths = QGroupBox("数据路径")
        paths_l = QVBoxLayout(paths)

        def path_row(label: str):
            row = QHBoxLayout()
            edit = QLineEdit()
            btn = QPushButton(label)
            row.addWidget(edit, 1)
            row.addWidget(btn)
            paths_l.addLayout(row)
            return edit, btn

        self.session_edit, b1 = path_row("session 目录")
        self.ext_edit, b2 = path_row("extrinsics.json")
        b1.clicked.connect(self._browse_session)
        b2.clicked.connect(lambda: self._browse_file(self.ext_edit))
        self.session_edit.editingFinished.connect(self._on_session_path_edited)
        form_l.addWidget(paths)

        params = QGroupBox("验证参数（来自 meta，可改）")
        form = QFormLayout(params)

        self.setup_combo = QComboBox()
        self.setup_combo.addItem("眼在手外 eye_to_hand", "eye_to_hand")
        self.setup_combo.addItem("眼在手上 eye_in_hand", "eye_in_hand")
        form.addRow("手眼模式", self.setup_combo)

        self.board_type = QComboBox()
        self.board_type.addItem("单 ArUco", "aruco")
        self.board_type.addItem("ChArUco", "charuco")
        self.board_type.currentIndexChanged.connect(self._sync_board_rows)
        form.addRow("板类型", self.board_type)

        self.dict_edit = QLineEdit("DICT_5X5_50")
        form.addRow("字典", self.dict_edit)

        self.spin_marker_id = QSpinBox()
        self.spin_marker_id.setRange(0, 999)
        form.addRow("ArUco ID", self.spin_marker_id)

        self.spin_marker_len = QDoubleSpinBox()
        self.spin_marker_len.setDecimals(4)
        self.spin_marker_len.setRange(0.001, 1.0)
        self.spin_marker_len.setSingleStep(0.005)
        self.spin_marker_len.setSuffix(" m")
        form.addRow("标记边长", self.spin_marker_len)

        self.spin_sx = QSpinBox()
        self.spin_sx.setRange(2, 20)
        self.spin_sy = QSpinBox()
        self.spin_sy.setRange(2, 20)
        self.spin_sq = QDoubleSpinBox()
        self.spin_sq.setDecimals(4)
        self.spin_sq.setRange(0.001, 1.0)
        self.spin_sq.setSuffix(" m")
        self.spin_cm = QDoubleSpinBox()
        self.spin_cm.setDecimals(4)
        self.spin_cm.setRange(0.001, 1.0)
        self.spin_cm.setSuffix(" m")
        form.addRow("ChArUco squares_x", self.spin_sx)
        form.addRow("ChArUco squares_y", self.spin_sy)
        form.addRow("ChArUco square_len", self.spin_sq)
        form.addRow("ChArUco marker_len", self.spin_cm)
        self._charuco_rows = (
            self.spin_sx,
            self.spin_sy,
            self.spin_sq,
            self.spin_cm,
        )

        self.spin_fx = QDoubleSpinBox()
        self.spin_fy = QDoubleSpinBox()
        self.spin_cx = QDoubleSpinBox()
        self.spin_cy = QDoubleSpinBox()
        for sp in (self.spin_fx, self.spin_fy, self.spin_cx, self.spin_cy):
            sp.setDecimals(4)
            sp.setRange(0.0, 20000.0)
        form.addRow("fx", self.spin_fx)
        form.addRow("fy", self.spin_fy)
        form.addRow("cx", self.spin_cx)
        form.addRow("cy", self.spin_cy)

        self.src_label = QLabel("尚未读取 session")
        self.src_label.setWordWrap(True)
        form.addRow("来源", self.src_label)
        form_l.addWidget(params)

        tfm_box = QGroupBox("T_flange_marker（法兰→板中心，仅画图）")
        tfm_form = QFormLayout(tfm_box)
        self.chk_tfm = QCheckBox("启用安装偏置（关闭则按法兰≈板）")
        self.chk_tfm.setChecked(True)
        tfm_form.addRow(self.chk_tfm)

        def _mm_spin(v: float) -> QDoubleSpinBox:
            sp = QDoubleSpinBox()
            sp.setDecimals(2)
            sp.setRange(-2000.0, 2000.0)
            sp.setSingleStep(1.0)
            sp.setSuffix(" mm")
            sp.setValue(v)
            return sp

        def _deg_spin(v: float = 0.0) -> QDoubleSpinBox:
            sp = QDoubleSpinBox()
            sp.setDecimals(2)
            sp.setRange(-180.0, 180.0)
            sp.setSingleStep(1.0)
            sp.setSuffix(" °")
            sp.setValue(v)
            return sp

        self.spin_tfm_x = _mm_spin(_DEFAULT_TFM_XYZ_MM[0])
        self.spin_tfm_y = _mm_spin(_DEFAULT_TFM_XYZ_MM[1])
        self.spin_tfm_z = _mm_spin(_DEFAULT_TFM_XYZ_MM[2])
        self.spin_tfm_rx = _deg_spin(_DEFAULT_TFM_RPY_DEG[0])
        self.spin_tfm_ry = _deg_spin(_DEFAULT_TFM_RPY_DEG[1])
        self.spin_tfm_rz = _deg_spin(_DEFAULT_TFM_RPY_DEG[2])
        row_t = QHBoxLayout()
        for lab, sp in (
            ("X", self.spin_tfm_x),
            ("Y", self.spin_tfm_y),
            ("Z", self.spin_tfm_z),
        ):
            row_t.addWidget(QLabel(lab))
            row_t.addWidget(sp)
        tfm_form.addRow("平移", row_t)
        row_r = QHBoxLayout()
        for lab, sp in (
            ("Rx", self.spin_tfm_rx),
            ("Ry", self.spin_tfm_ry),
            ("Rz", self.spin_tfm_rz),
        ):
            row_r.addWidget(QLabel(lab))
            row_r.addWidget(sp)
        tfm_form.addRow("RPY(xyz)", row_r)
        self.btn_tfm_default = QPushButton(
            f"恢复默认 {_DEFAULT_TFM_XYZ_MM[0]:.0f}/{_DEFAULT_TFM_XYZ_MM[1]:.0f}/"
            f"{_DEFAULT_TFM_XYZ_MM[2]:.0f} mm · Rx≈{_DEFAULT_TFM_RPY_DEG[0]:.0f}°"
        )
        self.btn_tfm_default.clicked.connect(self._reset_tfm_default)
        tfm_form.addRow(self.btn_tfm_default)
        self._tfm_spins = (
            self.spin_tfm_x,
            self.spin_tfm_y,
            self.spin_tfm_z,
            self.spin_tfm_rx,
            self.spin_tfm_ry,
            self.spin_tfm_rz,
        )
        self.chk_tfm.toggled.connect(self._on_tfm_changed)
        for sp in self._tfm_spins:
            sp.valueChanged.connect(self._on_tfm_changed)
        form_l.addWidget(tfm_box)

        row_btn = QHBoxLayout()
        self.btn_fill = QPushButton("从 session 填入参数")
        self.btn_fill.clicked.connect(self._fill_from_session)
        self.btn_load = QPushButton("加载 / 按当前参数重算")
        self.btn_load.clicked.connect(self._load)
        row_btn.addWidget(self.btn_fill)
        row_btn.addWidget(self.btn_load)
        form_l.addLayout(row_btn)
        form_l.addStretch(0)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(0)  # QFrame.NoFrame
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setWidget(form_host)
        scroll.setMinimumHeight(160)
        # Cap the form pane so preview keeps room on short displays.
        scroll.setMaximumHeight(360)
        layout.addWidget(scroll, stretch=0)

        self.preview = PreviewLabel()
        self.preview.setMinimumHeight(220)
        layout.addWidget(self.preview, stretch=1)

        ctrl = QHBoxLayout()
        self.play_btn = QPushButton("播放")
        self.play_btn.clicked.connect(self._toggle_play)
        self.slider = QSlider(
            Qt.Horizontal if hasattr(Qt, "Horizontal") else Qt.Orientation.Horizontal
        )
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
        self._dist = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._wh_fps = (1280, 720, 15)

        self._sync_board_rows()
        self._seed_from_cfg()

    def _seed_from_cfg(self) -> None:
        a = self.cfg.aruco
        self._apply_aruco_dict(
            {
                "board_type": getattr(a, "board_type", "aruco"),
                "dict": a.dict,
                "marker_id": a.marker_id,
                "marker_length_m": a.marker_length_m,
                "squares_x": a.squares_x,
                "squares_y": a.squares_y,
                "square_length_m": a.square_length_m,
                "charuco_marker_length_m": a.charuco_marker_length_m,
            }
        )
        setup = getattr(self.cfg.handeye, "setup", "eye_to_hand")
        i = self.setup_combo.findData(setup)
        if i >= 0:
            self.setup_combo.setCurrentIndex(i)
        tfm = getattr(a, "T_flange_marker", None)
        if tfm is not None:
            self._apply_tfm_matrix(np.asarray(tfm, dtype=np.float64), enabled=True)
        else:
            self._reset_tfm_default()

    def _reset_tfm_default(self) -> None:
        self.chk_tfm.blockSignals(True)
        for sp in self._tfm_spins:
            sp.blockSignals(True)
        self.chk_tfm.setChecked(True)
        self.spin_tfm_x.setValue(_DEFAULT_TFM_XYZ_MM[0])
        self.spin_tfm_y.setValue(_DEFAULT_TFM_XYZ_MM[1])
        self.spin_tfm_z.setValue(_DEFAULT_TFM_XYZ_MM[2])
        self.spin_tfm_rx.setValue(_DEFAULT_TFM_RPY_DEG[0])
        self.spin_tfm_ry.setValue(_DEFAULT_TFM_RPY_DEG[1])
        self.spin_tfm_rz.setValue(_DEFAULT_TFM_RPY_DEG[2])
        self.chk_tfm.blockSignals(False)
        for sp in self._tfm_spins:
            sp.blockSignals(False)
        self._on_tfm_changed()

    def _apply_tfm_matrix(self, T: np.ndarray, *, enabled: bool = True) -> None:
        xyzrpy = xyzrpy_from_T(T)
        self.chk_tfm.blockSignals(True)
        for sp in self._tfm_spins:
            sp.blockSignals(True)
        self.chk_tfm.setChecked(enabled)
        self.spin_tfm_x.setValue(xyzrpy[0] * 1000.0)
        self.spin_tfm_y.setValue(xyzrpy[1] * 1000.0)
        self.spin_tfm_z.setValue(xyzrpy[2] * 1000.0)
        self.spin_tfm_rx.setValue(float(np.rad2deg(xyzrpy[3])))
        self.spin_tfm_ry.setValue(float(np.rad2deg(xyzrpy[4])))
        self.spin_tfm_rz.setValue(float(np.rad2deg(xyzrpy[5])))
        self.chk_tfm.blockSignals(False)
        for sp in self._tfm_spins:
            sp.blockSignals(False)
        self._on_tfm_changed()

    def _apply_tfm_xyzrpy_mm_deg(self, vals: list[float], *, enabled: bool = True) -> None:
        self.chk_tfm.blockSignals(True)
        for sp in self._tfm_spins:
            sp.blockSignals(True)
        self.chk_tfm.setChecked(enabled)
        self.spin_tfm_x.setValue(float(vals[0]))
        self.spin_tfm_y.setValue(float(vals[1]))
        self.spin_tfm_z.setValue(float(vals[2]))
        self.spin_tfm_rx.setValue(float(vals[3]) if len(vals) > 3 else 0.0)
        self.spin_tfm_ry.setValue(float(vals[4]) if len(vals) > 4 else 0.0)
        self.spin_tfm_rz.setValue(float(vals[5]) if len(vals) > 5 else 0.0)
        self.chk_tfm.blockSignals(False)
        for sp in self._tfm_spins:
            sp.blockSignals(False)
        self._on_tfm_changed()

    def _collect_T_flange_marker(self) -> Optional[np.ndarray]:
        if not self.chk_tfm.isChecked():
            return None
        xyz_m = [
            float(self.spin_tfm_x.value()) / 1000.0,
            float(self.spin_tfm_y.value()) / 1000.0,
            float(self.spin_tfm_z.value()) / 1000.0,
        ]
        rpy_rad = [
            float(np.deg2rad(self.spin_tfm_rx.value())),
            float(np.deg2rad(self.spin_tfm_ry.value())),
            float(np.deg2rad(self.spin_tfm_rz.value())),
        ]
        return T_from_xyzrpy([*xyz_m, *rpy_rad])

    def _on_tfm_changed(self, *_args) -> None:
        for sp in self._tfm_spins:
            sp.setEnabled(self.chk_tfm.isChecked())
        if self.svc.reader is not None:
            self.svc.set_T_flange_marker(self._collect_T_flange_marker())
            self._render(self.slider.value())

    def _sync_board_rows(self) -> None:
        is_charuco = (self.board_type.currentData() or "aruco") == "charuco"
        for w in self._charuco_rows:
            w.setEnabled(is_charuco)
        self.spin_marker_id.setEnabled(not is_charuco)
        self.spin_marker_len.setEnabled(not is_charuco)

    def _apply_aruco_dict(self, ar: dict) -> None:
        bt = str(ar.get("board_type") or "aruco")
        i = self.board_type.findData(bt if bt in ("aruco", "charuco") else "aruco")
        self.board_type.setCurrentIndex(i if i >= 0 else 0)
        self.dict_edit.setText(str(ar.get("dict") or "DICT_5X5_50"))
        self.spin_marker_id.setValue(int(ar.get("marker_id") or 0))
        self.spin_marker_len.setValue(float(ar.get("marker_length_m") or 0.03))
        self.spin_sx.setValue(int(ar.get("squares_x") or 5))
        self.spin_sy.setValue(int(ar.get("squares_y") or 7))
        self.spin_sq.setValue(float(ar.get("square_length_m") or 0.04))
        self.spin_cm.setValue(float(ar.get("charuco_marker_length_m") or 0.03))
        self._sync_board_rows()

    def _apply_intrinsics_dict(self, intr: dict) -> None:
        self.spin_fx.setValue(float(intr.get("fx") or 0))
        self.spin_fy.setValue(float(intr.get("fy") or 0))
        self.spin_cx.setValue(float(intr.get("cx") or 0))
        self.spin_cy.setValue(float(intr.get("cy") or 0))
        self._dist = [float(x) for x in (intr.get("dist") or [0, 0, 0, 0, 0])]
        while len(self._dist) < 5:
            self._dist.append(0.0)
        self._wh_fps = (
            int(intr.get("width") or 1280),
            int(intr.get("height") or 720),
            int(intr.get("fps") or 15),
        )

    def _browse_session(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "选择 session 目录")
        if d:
            self.session_edit.setText(d)
            self._fill_from_session()

    def _browse_file(self, edit: QLineEdit) -> None:
        f, _ = QFileDialog.getOpenFileName(self, "选择外参", filter="JSON (*.json)")
        if f:
            edit.setText(f)

    def _on_session_path_edited(self) -> None:
        if self.session_edit.text().strip():
            self._fill_from_session(silent=True)

    def _fill_from_session(self, silent: bool = False) -> None:
        sess = self.session_edit.text().strip()
        if not sess:
            if not silent:
                QMessageBox.information(self, "验证", "请先选择 session 目录")
            return
        res = self.svc.peek_meta(Path(sess))
        if not isinstance(res, Ok):
            if not silent:
                QMessageBox.warning(self, "验证", str(res.error))
            return
        info = res.value
        i = self.setup_combo.findData(info["setup"])
        if i >= 0:
            self.setup_combo.setCurrentIndex(i)
        self._apply_aruco_dict(info["aruco"])
        self._apply_intrinsics_dict(info["intrinsics"])
        ext_default = info["default_extrinsics"]
        if Path(ext_default).is_file():
            self.ext_edit.setText(ext_default)
        tfm_mm = info.get("T_flange_marker_xyzrpy_mm_deg")
        if tfm_mm:
            self._apply_tfm_xyzrpy_mm_deg(list(tfm_mm), enabled=True)
            tfm_note = (
                f"T_fm=[{tfm_mm[0]:.1f},{tfm_mm[1]:.1f},{tfm_mm[2]:.1f}]mm "
                f"RPY=[{tfm_mm[3]:.1f},{tfm_mm[4]:.1f},{tfm_mm[5]:.1f}]°"
            )
        else:
            # Keep current UI values (default 165/0/40) when meta has no mount.
            tfm_note = (
                f"T_fm(UI)=[{self.spin_tfm_x.value():.1f},"
                f"{self.spin_tfm_y.value():.1f},{self.spin_tfm_z.value():.1f}]mm"
            )
        ar = info["aruco"]
        self.src_label.setText(
            f"已从 meta 填入：setup={info['setup']}  "
            f"board={ar.get('board_type')}  marker_len={ar.get('marker_length_m')}  "
            f"fx={info['intrinsics']['fx']:.2f}  {tfm_note}  "
            f"默认外参={ext_default}"
        )

    def _collect_overrides(self) -> tuple[str, dict, Intrinsics]:
        setup = str(self.setup_combo.currentData() or "eye_to_hand")
        aruco = {
            "board_type": str(self.board_type.currentData() or "aruco"),
            "dict": self.dict_edit.text().strip() or "DICT_5X5_50",
            "marker_id": int(self.spin_marker_id.value()),
            "marker_length_m": float(self.spin_marker_len.value()),
            "squares_x": int(self.spin_sx.value()),
            "squares_y": int(self.spin_sy.value()),
            "square_length_m": float(self.spin_sq.value()),
            "charuco_marker_length_m": float(self.spin_cm.value()),
        }
        w, h, fps = self._wh_fps
        intr = Intrinsics(
            fx=float(self.spin_fx.value()),
            fy=float(self.spin_fy.value()),
            cx=float(self.spin_cx.value()),
            cy=float(self.spin_cy.value()),
            dist=list(self._dist[:5]),
            width=w,
            height=h,
            fps=fps,
        )
        return setup, aruco, intr

    def _load(self) -> None:
        sess = self.session_edit.text().strip()
        ext = self.ext_edit.text().strip()
        if not sess:
            QMessageBox.warning(self, "验证", "请选择 session 目录")
            return
        if not ext:
            cand = Path(sess) / "extrinsics.json"
            if cand.is_file():
                ext = str(cand)
                self.ext_edit.setText(ext)
            else:
                QMessageBox.warning(self, "验证", "请选择 extrinsics.json")
                return

        setup, aruco, intr = self._collect_overrides()
        T_fm = self._collect_T_flange_marker()
        res = self.svc.load(
            Path(sess),
            Path(ext),
            setup_override=setup,
            aruco_override=aruco,
            intrinsics_override=intr,
            T_flange_marker_override=T_fm,
            T_flange_marker_override_set=True,
        )
        if isinstance(res, Ok):
            n = res.value
            self.slider.setMaximum(max(0, n - 1))
            self.slider.setValue(0)
            info = self.svc.load_info
            extra = ""
            if setup == "eye_in_hand":
                extra = (
                    f" | T_base_marker←{self.svc.base_marker_n}帧 "
                    f"散度≈{self.svc.base_marker_spread_mm:.1f}mm"
                )
            if T_fm is None:
                tfm_txt = "T_fm=I"
            else:
                tfm_txt = (
                    f"T_fm=[{self.spin_tfm_x.value():.0f},"
                    f"{self.spin_tfm_y.value():.0f},"
                    f"{self.spin_tfm_z.value():.0f}]mm"
                )
            self.err_label.setText(
                f"已加载 {n} 帧 · {setup} · "
                f"板={aruco['board_type']}/{aruco.get('marker_length_m')}m · "
                f"fx={intr.fx:.1f} · {tfm_txt}{extra}"
            )
            self.src_label.setText(
                f"生效：setup={info.get('setup')}({info.get('setup_source')})  "
                f"内参={info.get('intrinsics_source')}  "
                f"ArUco={info.get('aruco_source')}  "
                f"T_fm={info.get('T_flange_marker_source')}  "
                f"board={aruco['board_type']} len={aruco.get('marker_length_m')}"
            )
            self.setWindowTitle(f"验证 · {setup} · A=预测板(RGB) / B=检测板(CMY)")
            self._render(0)
        else:
            self.err_label.setText(str(res.error))
            QMessageBox.warning(self, "验证加载失败", str(res.error))

    def _on_slider(self, v: int) -> None:
        self._render(v)

    def _render(self, idx: int) -> None:
        self.svc.set_T_flange_marker(self._collect_T_flange_marker())
        img, metrics = self.svc.render(idx)
        if img is not None:
            self.preview.show_bgr(img)
        if metrics is not None and metrics.has_detect:
            self.err_label.setText(
                f"帧{idx} 板误差 Δt={metrics.marker_err_t_mm:.2f}mm "
                f"ΔR={metrics.marker_err_R_deg:.2f}°  "
                f"[{self.svc.setup}]"
            )
        elif metrics is not None:
            self.err_label.setText(f"帧{idx} 未检测到板  [{self.svc.setup}]")

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
