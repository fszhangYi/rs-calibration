from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from rs_calib.app_services.arm_service import ArmService
from rs_calib.app_services.camera_service import CameraService
from rs_calib.app_services.live_calib_service import LiveCalibService
from rs_calib.app_services.record_service import RecordService
from rs_calib.config.schema import AppConfig
from rs_calib.config.store import save_config
from rs_calib.domain.aruco import detect_target
from rs_calib.domain.types import ArmState, ConvergenceStatus, ExtrinsicsResult
from rs_calib.errors import Ok
from rs_calib.infra import session_store
from rs_calib.app_services.session_report_service import build_session_report
from rs_calib.qt_compat import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QSplitter,
    QStatusBar,
    QTextEdit,
    Qt,
    QVBoxLayout,
    QWidget,
)
from rs_calib.ui.dialog_matrix_path import MatrixPathDialog
from rs_calib.ui.dialog_offline import OfflineDialog
from rs_calib.ui.dialog_params import ParamsDialog
from rs_calib.ui.dialog_verify import VerifyDialog
from rs_calib.ui.preview_label import PreviewLabel
from rs_calib.workers.arm_poll_worker import ArmPollWorker
from rs_calib.workers.camera_worker import CameraWorker


class MainWindow(QMainWindow):
    def __init__(self, cfg: AppConfig):
        super().__init__()
        self.cfg = cfg
        self.setWindowTitle("RS Eye-to-Hand 标定上位机")
        self.resize(1280, 800)

        self.camera = CameraService()
        self.arm = ArmService(cfg)
        self.record = RecordService(cfg, self.camera, self.arm)
        self.live = LiveCalibService(cfg)

        self.cam_worker = CameraWorker(self.camera)
        self.cam_worker.frames_ready.connect(self._on_frames)
        self.arm_worker = ArmPollWorker(self.arm, hz=cfg.arm.poll_hz)
        self.arm_worker.state_ready.connect(self._on_arm_state)

        self._build_ui()
        self.cam_worker.start()
        self.arm_worker.start()
        self.statusBar().showMessage("就绪")

    def _build_ui(self) -> None:
        root = QWidget()
        self.setCentralWidget(root)
        outer = QHBoxLayout(root)
        outer.setContentsMargins(6, 6, 6, 6)
        splitter = QSplitter()
        outer.addWidget(splitter)

        left = QWidget()
        left_l = QVBoxLayout(left)
        left_l.setContentsMargins(4, 4, 4, 4)
        left_l.setSpacing(6)

        # Camera (支持多选开流)
        cam_box = QGroupBox("相机（可多选）")
        cam_l = QVBoxLayout(cam_box)
        self.device_list = QListWidget()
        # 2 == QAbstractItemView.ExtendedSelection (PyQt5/6 compatible)
        self.device_list.setSelectionMode(2)
        self.device_list.setMaximumHeight(110)
        cam_l.addWidget(self.device_list)
        cam_l.addWidget(QLabel("提示：多选后点「开流」；多相机录制写多个视频，禁用实时求解"))
        row = QHBoxLayout()
        self.btn_scan = QPushButton("扫描")
        self.btn_scan.clicked.connect(self._scan_cameras)
        self.btn_open_cam = QPushButton("开流")
        self.btn_open_cam.clicked.connect(self._open_camera)
        self.btn_close_cam = QPushButton("关流")
        self.btn_close_cam.clicked.connect(self._close_camera)
        row.addWidget(self.btn_scan)
        row.addWidget(self.btn_open_cam)
        row.addWidget(self.btn_close_cam)
        cam_l.addLayout(row)
        prev_row = QHBoxLayout()
        prev_row.addWidget(QLabel("预览相机"))
        self.preview_cam_combo = QComboBox()
        self.preview_cam_combo.currentIndexChanged.connect(self._on_preview_cam_changed)
        prev_row.addWidget(self.preview_cam_combo, 1)
        cam_l.addLayout(prev_row)
        self.cam_mode_label = QLabel("模式: 未开流")
        cam_l.addWidget(self.cam_mode_label)

        form = QFormLayout()
        self.spin_fx = QDoubleSpinBox()
        self.spin_fy = QDoubleSpinBox()
        self.spin_cx = QDoubleSpinBox()
        self.spin_cy = QDoubleSpinBox()
        for sp in (self.spin_fx, self.spin_fy, self.spin_cx, self.spin_cy):
            sp.setRange(0, 10000)
            sp.setDecimals(3)
        form.addRow("fx", self.spin_fx)
        form.addRow("fy", self.spin_fy)
        form.addRow("cx", self.spin_cx)
        form.addRow("cy", self.spin_cy)
        reread = QPushButton("重新读取内参")
        reread.clicked.connect(self._reread_intrinsics)
        cam_l.addLayout(form)
        cam_l.addWidget(reread)
        left_l.addWidget(cam_box)

        # Calibration board (ChArUco / ArUco) — same as web, not under arm
        board_box = QGroupBox("标定板 ChArUco / ArUco")
        board_l = QVBoxLayout(board_box)
        board_row1 = QHBoxLayout()
        self.board_type_combo = QComboBox()
        self.board_type_combo.addItem("ChArUco 板", "charuco")
        self.board_type_combo.addItem("单 ArUco 码", "aruco")
        bt = getattr(self.cfg.aruco, "board_type", "charuco")
        bi = self.board_type_combo.findData(bt)
        self.board_type_combo.setCurrentIndex(bi if bi >= 0 else 0)
        self.board_type_combo.currentIndexChanged.connect(self._on_board_type_changed)
        board_row1.addWidget(self.board_type_combo)
        self.dict_edit = QLineEdit(str(self.cfg.aruco.dict))
        self.dict_edit.setPlaceholderText("DICT_5X5_50")
        board_row1.addWidget(self.dict_edit, 1)
        board_l.addLayout(board_row1)

        self.board_charuco_row = QWidget()
        board_row2 = QHBoxLayout(self.board_charuco_row)
        board_row2.setContentsMargins(0, 0, 0, 0)
        self.spin_sq_x = QSpinBox()
        self.spin_sq_x.setRange(2, 50)
        self.spin_sq_x.setValue(int(getattr(self.cfg.aruco, "squares_x", 5)))
        self.spin_sq_y = QSpinBox()
        self.spin_sq_y.setRange(2, 50)
        self.spin_sq_y.setValue(int(getattr(self.cfg.aruco, "squares_y", 7)))
        board_row2.addWidget(self.spin_sq_x)
        board_row2.addWidget(QLabel("×"))
        board_row2.addWidget(self.spin_sq_y)
        self.spin_sq_mm = QDoubleSpinBox()
        self.spin_sq_mm.setRange(1.0, 500.0)
        self.spin_sq_mm.setDecimals(1)
        self.spin_sq_mm.setSuffix(" mm格")
        self.spin_sq_mm.setValue(float(getattr(self.cfg.aruco, "square_length_m", 0.04)) * 1000.0)
        board_row2.addWidget(self.spin_sq_mm)
        board_row2.addStretch(1)
        board_l.addWidget(self.board_charuco_row)

        board_row3 = QHBoxLayout()
        self.spin_mk_mm = QDoubleSpinBox()
        self.spin_mk_mm.setRange(1.0, 500.0)
        self.spin_mk_mm.setDecimals(1)
        self.spin_mk_mm.setSuffix(" mm码")
        a = self.cfg.aruco
        if str(getattr(a, "board_type", "charuco")).lower() in ("aruco", "single", "marker"):
            self.spin_mk_mm.setValue(float(a.marker_length_m) * 1000.0)
        else:
            self.spin_mk_mm.setValue(float(getattr(a, "charuco_marker_length_m", 0.03)) * 1000.0)
        board_row3.addWidget(self.spin_mk_mm)
        self.spin_marker_id = QSpinBox()
        self.spin_marker_id.setRange(0, 10000)
        self.spin_marker_id.setPrefix("id ")
        self.spin_marker_id.setValue(int(a.marker_id))
        board_row3.addWidget(self.spin_marker_id)
        btn_board = QPushButton("应用板")
        btn_board.clicked.connect(self._apply_board)
        board_row3.addWidget(btn_board)
        board_row3.addStretch(1)
        board_l.addLayout(board_row3)
        self.board_hint = QLabel("")
        self.board_hint.setWordWrap(True)
        board_l.addWidget(self.board_hint)
        self._refresh_board_hint()
        left_l.addWidget(board_box)

        # Arm
        arm_box = QGroupBox("机械臂 EC616")
        arm_l = QVBoxLayout(arm_box)
        ip_row = QHBoxLayout()
        self.ip_edit = QLineEdit(self.cfg.arm.robot_ip)
        self.btn_arm_conn = QPushButton("连接")
        self.btn_arm_conn.clicked.connect(self._connect_arm)
        self.btn_arm_disc = QPushButton("断开")
        self.btn_arm_disc.clicked.connect(self._disconnect_arm)
        ip_row.addWidget(self.ip_edit)
        ip_row.addWidget(self.btn_arm_conn)
        ip_row.addWidget(self.btn_arm_disc)
        arm_l.addLayout(ip_row)
        self.arm_status = QLabel("未连接")
        arm_l.addWidget(self.arm_status)
        gate = QHBoxLayout()
        self.btn_arm_on = QPushButton("Arm")
        self.btn_arm_on.clicked.connect(self._arm_enable)
        self.btn_arm_off = QPushButton("Disarm")
        self.btn_arm_off.clicked.connect(self._arm_disarm)
        self.btn_arm_stop = QPushButton("Stop")
        self.btn_arm_stop.clicked.connect(self._arm_stop)
        for b in (self.btn_arm_on, self.btn_arm_off, self.btn_arm_stop):
            gate.addWidget(b)
        arm_l.addLayout(gate)
        self._jog_joint_btns: list[QPushButton] = []
        self._jog_cart_btns: list[QPushButton] = []

        # 关节 / 笛卡尔并排，减少纵向高度，避免最大化后被挤没
        jog_row = QHBoxLayout()
        joint_box = QGroupBox("关节点动")
        jog = QGridLayout(joint_box)
        jog.setContentsMargins(4, 4, 4, 4)
        jog.setSpacing(2)
        self.joint_value_labels: list[QLabel] = []
        for i in range(6):
            jm = QPushButton(f"J{i+1}-")
            jp = QPushButton(f"J{i+1}+")
            jv = QLabel("—")
            jv.setMinimumWidth(64)
            for b in (jm, jp):
                b.setMaximumHeight(26)
            jm.clicked.connect(lambda _=False, idx=i: self._jog_joint(idx, -1))
            jp.clicked.connect(lambda _=False, idx=i: self._jog_joint(idx, +1))
            jog.addWidget(jm, i, 0)
            jog.addWidget(jp, i, 1)
            jog.addWidget(jv, i, 2)
            self.joint_value_labels.append(jv)
            self._jog_joint_btns.extend([jm, jp])
        jog_row.addWidget(joint_box)

        cart_box = QGroupBox("笛卡尔点动")
        cart = QGridLayout(cart_box)
        cart.setContentsMargins(4, 4, 4, 4)
        cart.setSpacing(2)
        axes = [("X", "x"), ("Y", "y"), ("Z", "z"), ("Rx", "rx"), ("Ry", "ry"), ("Rz", "rz")]
        for r, (lab, ax) in enumerate(axes):
            bm = QPushButton(f"{lab}-")
            bp = QPushButton(f"{lab}+")
            for b in (bm, bp):
                b.setMaximumHeight(26)
            bm.clicked.connect(lambda _=False, a=ax: self._jog_cart(a, -1))
            bp.clicked.connect(lambda _=False, a=ax: self._jog_cart(a, +1))
            cart.addWidget(bm, r, 0)
            cart.addWidget(bp, r, 1)
            self._jog_cart_btns.extend([bm, bp])
        jog_row.addWidget(cart_box)
        arm_l.addLayout(jog_row)

        self.joints_label = QLabel("关节(deg): —")
        self.joints_label.setWordWrap(True)
        arm_l.addWidget(self.joints_label)

        frame_row = QHBoxLayout()
        frame_row.addWidget(QLabel("笛卡尔参考"))
        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["flange", "tcp"])
        self.frame_combo.setCurrentText(self.cfg.arm.cart_frame)
        self.frame_combo.currentTextChanged.connect(self._on_frame_changed)
        frame_row.addWidget(self.frame_combo, 1)
        arm_l.addLayout(frame_row)

        delta_row = QHBoxLayout()
        delta_row.addWidget(QLabel("单步最大Δq"))
        self.spin_max_delta = QDoubleSpinBox()
        self.spin_max_delta.setRange(0.01, 45.0)
        self.spin_max_delta.setDecimals(2)
        self.spin_max_delta.setSingleStep(0.5)
        self.spin_max_delta.setSuffix(" °")
        self.spin_max_delta.setValue(float(self.cfg.arm.max_delta_deg))
        self.spin_max_delta.setToolTip(
            "驱动单次关节指令 |Δq| 上限（arm.max_delta_deg）。\n"
            "矩阵走位会按此值自动切段；改完点「应用」写入驱动（无需重连）。"
        )
        delta_row.addWidget(self.spin_max_delta, 1)
        btn_max_delta = QPushButton("应用")
        btn_max_delta.clicked.connect(self._apply_max_delta)
        delta_row.addWidget(btn_max_delta)
        arm_l.addLayout(delta_row)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel("速度 v_norm"))
        self.spin_v_norm = QDoubleSpinBox()
        self.spin_v_norm.setRange(0.001, 5.0)
        self.spin_v_norm.setDecimals(3)
        self.spin_v_norm.setSingleStep(0.01)
        self.spin_v_norm.setSuffix(" rad/s")
        self.spin_v_norm.setValue(float(getattr(self.cfg.arm, "v_norm_rad_s", 0.02)))
        self.spin_v_norm.setToolTip(
            "关节空间速度（同 sensors-arm arm_abs_ramp.v_norm_rad_s）。\n"
            "走位时长 T=clamp(peakΔq/v_norm, t_min, t_max)。越大越快。"
        )
        speed_row.addWidget(self.spin_v_norm, 1)
        btn_v_norm = QPushButton("应用")
        btn_v_norm.clicked.connect(self._apply_v_norm)
        speed_row.addWidget(btn_v_norm)
        arm_l.addLayout(speed_row)

        self.pose_label = QLabel("法兰 pose: -")
        self.pose_label.setWordWrap(True)
        arm_l.addWidget(self.pose_label)
        left_l.addWidget(arm_box)

        # Record / live
        rec_box = QGroupBox("录制 / 实时标定")
        rec_l = QVBoxLayout(rec_box)
        setup_row = QHBoxLayout()
        setup_row.addWidget(QLabel("手眼模式"))
        self.setup_combo = QComboBox()
        self.setup_combo.addItem("眼在手外 · T_base_cam", "eye_to_hand")
        self.setup_combo.addItem("眼在手上 · T_flange_cam", "eye_in_hand")
        cur_setup = getattr(self.cfg.handeye, "setup", "eye_to_hand")
        idx = self.setup_combo.findData(cur_setup)
        self.setup_combo.setCurrentIndex(idx if idx >= 0 else 0)
        self.setup_combo.currentIndexChanged.connect(self._on_setup_changed)
        self.setup_combo.setToolTip(
            "眼在手外：场景相机固定，标定板在末端 → 求 T_base_cam\n"
            "眼在手上：腕部相机随法兰，标定板固定 → 求 T_flange_cam"
        )
        setup_row.addWidget(self.setup_combo, 1)
        rec_l.addLayout(setup_row)
        self.live_cb = QCheckBox("实时计算（仅单相机）")
        self.live_cb.toggled.connect(self._toggle_live)
        self.live_cb.setToolTip("多相机模式下不支持实时求解，请录制后离线计算")
        rec_l.addWidget(self.live_cb)
        row2 = QHBoxLayout()
        self.btn_rec_start = QPushButton("开始录制")
        self.btn_rec_stop = QPushButton("停止录制")
        self.btn_rec_start.clicked.connect(self._start_record)
        self.btn_rec_stop.clicked.connect(self._stop_record)
        row2.addWidget(self.btn_rec_start)
        row2.addWidget(self.btn_rec_stop)
        rec_l.addLayout(row2)
        self.session_label = QLabel("session: -")
        self.calib_label = QLabel("标定: 样本 0")
        self.calib_label.setWordWrap(True)
        self.btn_export_live = QPushButton("导出当前外参到 session")
        self.btn_export_live.clicked.connect(self._export_live)
        self.btn_session_report = QPushButton("生成结论到日志")
        self.btn_session_report.setToolTip("收敛后可用：根据 meta/joints/live_solves/extrinsics 生成分析结论")
        self.btn_session_report.clicked.connect(self._session_report)
        rec_l.addWidget(self.session_label)
        rec_l.addWidget(self.calib_label)
        rec_l.addWidget(self.btn_export_live)
        rec_l.addWidget(self.btn_session_report)
        left_l.addWidget(rec_box)

        # Menus-ish buttons
        tools = QHBoxLayout()
        self.btn_params = QPushButton("参数")
        self.btn_params.clicked.connect(self._open_params)
        self.btn_matrix = QPushButton("矩阵走位")
        self.btn_matrix.clicked.connect(self._open_matrix_path)
        self.btn_offline = QPushButton("离线计算")
        self.btn_offline.clicked.connect(self._open_offline)
        self.btn_verify = QPushButton("验证")
        self.btn_verify.clicked.connect(self._open_verify)
        for b in (self.btn_params, self.btn_matrix, self.btn_offline, self.btn_verify):
            tools.addWidget(b)
        left_l.addLayout(tools)
        left_l.addStretch(0)
        self._refresh_action_state()

        # 左侧可滚动：最大化/矮屏时也能滚到点动与 pose
        left_scroll = QScrollArea()
        left_scroll.setWidgetResizable(True)
        left_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        left_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        left_scroll.setFrameShape(0)  # QFrame.NoFrame
        left.setMinimumWidth(360)
        left_scroll.setWidget(left)
        left_scroll.setMinimumWidth(380)

        center = QWidget()
        cl = QVBoxLayout(center)
        self.preview = PreviewLabel()
        cl.addWidget(self.preview)

        splitter.addWidget(left_scroll)
        splitter.addWidget(center)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([420, 860])
        self.setStatusBar(QStatusBar())

    # ---- camera ----
    def _scan_cameras(self) -> None:
        res = self.camera.scan()
        self.device_list.clear()
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "扫描失败", str(res.error))
            return
        for d in res.value:
            item = QListWidgetItem(f"{d.serial}  |  {d.name}  |  fw={d.firmware}")
            item.setData(256, d.serial)  # Qt.UserRole ~ 256
            item.setData(257, d.name)
            self.device_list.addItem(item)
        self.statusBar().showMessage(f"发现 {len(res.value)} 台相机（可 Ctrl/Shift 多选）")

    def _selected_device_items(self) -> list:
        items = self.device_list.selectedItems()
        if items:
            return items
        cur = self.device_list.currentItem()
        return [cur] if cur is not None else []

    def _open_camera(self) -> None:
        items = self._selected_device_items()
        if not items:
            QMessageBox.information(self, "提示", "请先选择一台或多台相机")
            return
        serials = [it.data(256) for it in items]
        names = {it.data(256): (it.data(257) or "") for it in items}
        res = self.camera.open_many(
            serials,
            self.cfg.camera.width,
            self.cfg.camera.height,
            self.cfg.camera.fps,
            names=names,
        )
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "开流失败", str(res.error))
            return
        self._refresh_preview_cam_combo()
        intr = self.camera.current_intrinsics()
        if intr is not None:
            self._fill_intrinsics(intr)
        self._update_cam_mode_ui()
        self.statusBar().showMessage(f"已开流 {len(serials)} 台: {', '.join(serials)}")

    def _close_camera(self) -> None:
        self.camera.close()
        self.preview_cam_combo.blockSignals(True)
        self.preview_cam_combo.clear()
        self.preview_cam_combo.blockSignals(False)
        self._update_cam_mode_ui()
        self.statusBar().showMessage("相机已关闭")

    def _refresh_preview_cam_combo(self) -> None:
        self.preview_cam_combo.blockSignals(True)
        self.preview_cam_combo.clear()
        for s in self.camera.open_serials:
            name = self.camera.device_names.get(s, "")
            self.preview_cam_combo.addItem(f"{s}" + (f" ({name})" if name else ""), s)
        if self.camera.preview_serial:
            idx = self.preview_cam_combo.findData(self.camera.preview_serial)
            if idx >= 0:
                self.preview_cam_combo.setCurrentIndex(idx)
        self.preview_cam_combo.blockSignals(False)

    def _on_preview_cam_changed(self, _index: int = 0) -> None:
        serial = self.preview_cam_combo.currentData()
        if not serial:
            return
        res = self.camera.set_preview_serial(serial)
        if isinstance(res, Ok):
            self._fill_intrinsics(res.value)

    def _update_cam_mode_ui(self) -> None:
        n = self.camera.n_open
        if n <= 0:
            self.cam_mode_label.setText("模式: 未开流")
        elif n == 1:
            self.cam_mode_label.setText("模式: 单相机（可用实时求解）")
        else:
            self.cam_mode_label.setText(f"模式: 多相机 ×{n}（实时求解已禁用，请离线计算）")
            if self.live_cb.isChecked():
                self.live_cb.setChecked(False)
            self.live.set_enabled(False)
        self._refresh_action_state()

    def _refresh_action_state(self) -> None:
        """Enable/disable buttons by current camera / arm / record / live state."""
        cam_on = self.camera.n_open > 0
        multi = self.camera.is_multi
        recording = bool(self.record.recording)
        arm_on = bool(self.arm.adapter.is_connected)
        armed = bool(self.arm.adapter.armed) if arm_on else False
        has_live = self.live.last_result is not None
        has_session = self.record.session_dir is not None
        live_converged = bool(self.live.last_status and self.live.last_status.converged)
        key = (
            cam_on,
            multi,
            recording,
            arm_on,
            armed,
            has_live,
            has_session,
            self.live.enabled,
            live_converged,
        )
        if getattr(self, "_action_state_key", None) == key:
            return
        self._action_state_key = key

        self.btn_close_cam.setEnabled(cam_on and not recording)
        self.btn_open_cam.setEnabled(not recording)
        self.btn_scan.setEnabled(not recording)

        self.btn_arm_conn.setEnabled(not arm_on)
        self.btn_arm_disc.setEnabled(arm_on and not recording)
        self.btn_arm_on.setEnabled(arm_on and not armed)
        self.btn_arm_off.setEnabled(arm_on and armed)
        self.btn_arm_stop.setEnabled(arm_on)
        can_jog = arm_on and armed
        for b in self._jog_joint_btns + self._jog_cart_btns:
            b.setEnabled(can_jog)

        self.btn_rec_start.setEnabled(cam_on and arm_on and not recording)
        self.btn_rec_stop.setEnabled(recording)
        # Live may stay on while recording; only block enable when multi-cam.
        self.live_cb.setEnabled(cam_on and (not multi))
        self.btn_export_live.setEnabled(has_live and has_session)
        self.btn_session_report.setEnabled(live_converged and has_session)
        self.btn_matrix.setEnabled(arm_on and armed)
        # Highlight active primary actions
        self.btn_rec_start.setStyleSheet("" if not (cam_on and arm_on and not recording) else "font-weight:600;")
        self.btn_rec_stop.setStyleSheet("" if not recording else "font-weight:600; color:#b00020;")
        self.btn_arm_on.setStyleSheet("" if not (arm_on and not armed) else "font-weight:600;")
        self.btn_export_live.setStyleSheet(
            "" if not (has_live and has_session) else "font-weight:600; color:#0a7a4a;"
        )
        self.btn_session_report.setStyleSheet(
            "" if not (live_converged and has_session) else "font-weight:600; color:#0a7a4a;"
        )

    def _reread_intrinsics(self) -> None:
        res = self.camera.reread_intrinsics()
        if isinstance(res, Ok):
            self._fill_intrinsics(res.value)
        else:
            QMessageBox.warning(self, "内参", str(res.error))

    def _fill_intrinsics(self, intr) -> None:
        self.spin_fx.setValue(intr.fx)
        self.spin_fy.setValue(intr.fy)
        self.spin_cx.setValue(intr.cx)
        self.spin_cy.setValue(intr.cy)
        self._sync_intrinsics_from_spins()

    def _sync_intrinsics_from_spins(self) -> None:
        intr = self.camera.current_intrinsics()
        if intr is None:
            return
        intr.fx = self.spin_fx.value()
        intr.fy = self.spin_fy.value()
        intr.cx = self.spin_cx.value()
        intr.cy = self.spin_cy.value()

    # ---- arm ----
    def _connect_arm(self) -> None:
        self.cfg.arm.robot_ip = self.ip_edit.text().strip()
        res = self.arm.connect(self.cfg.arm.robot_ip)
        if isinstance(res, Ok):
            st = res.value
            self.arm_status.setText(f"已连接 {st.endpoint}")
            self.statusBar().showMessage(self.arm_status.text())
            if st.ok and st.joints_rad is not None:
                self._update_joint_display(st.joints_rad, armed=st.armed, endpoint=st.endpoint)
        else:
            self.arm_status.setText("连接失败")
            QMessageBox.warning(self, "机械臂", str(res.error))
        self._refresh_action_state()

    def _disconnect_arm(self) -> None:
        self.arm.disconnect()
        self.arm_status.setText("未连接")
        self._clear_joint_display()
        self._refresh_action_state()

    def _clear_joint_display(self) -> None:
        self.joints_label.setText("关节(deg): —")
        for lab in self.joint_value_labels:
            lab.setText("—")
        self.pose_label.setText("法兰 pose: -")

    def _update_joint_display(self, joints_rad: list[float], *, armed: bool, endpoint: str) -> None:
        deg = [math.degrees(float(x)) for x in joints_rad]
        for i, lab in enumerate(self.joint_value_labels):
            if i < len(deg):
                lab.setText(f"{deg[i]:+.2f}°")
        self.joints_label.setText(
            "关节(deg): " + ", ".join(f"J{i+1}={d:+.2f}" for i, d in enumerate(deg))
        )
        armed_txt = "Armed" if armed else "Connected"
        self.arm_status.setText(f"{armed_txt} | {endpoint}")
        T, xyzrpy = self.arm.flange_from_joints(joints_rad)
        _ = T
        self.pose_label.setText(
            "法兰 xyzrpy: " + ", ".join(f"{v:.3f}" for v in xyzrpy)
        )

    def _arm_enable(self) -> None:
        res = self.arm.arm()
        if isinstance(res, Ok):
            self.statusBar().showMessage("Arm 成功")
        else:
            QMessageBox.warning(self, "Arm", str(res.error))
        self._refresh_action_state()

    def _arm_disarm(self) -> None:
        res = self.arm.disarm()
        self.statusBar().showMessage("Disarm" if isinstance(res, Ok) else str(res.error))
        self._refresh_action_state()

    def _arm_stop(self) -> None:
        res = self.arm.stop()
        self.statusBar().showMessage("Stop" if isinstance(res, Ok) else str(res.error))
        self._refresh_action_state()

    def _jog_joint(self, idx: int, sign: int) -> None:
        res = self.arm.jog_joint(idx, sign)
        if not isinstance(res, Ok):
            self.statusBar().showMessage(f"关节点动失败: {res.error}")

    def _jog_cart(self, axis: str, sign: int) -> None:
        res = self.arm.jog_cartesian(axis, sign)
        if not isinstance(res, Ok):
            # Avoid modal popup on every click; detail stays in status bar.
            self.statusBar().showMessage(f"笛卡尔点动失败: {res.error}")
        else:
            self.statusBar().showMessage(f"笛卡尔点动 {axis}{'+' if sign>=0 else '-'} OK")

    def _on_frame_changed(self, text: str) -> None:
        self.cfg.arm.cart_frame = text

    def _on_arm_state(self, st: ArmState) -> None:
        if not st.ok or st.joints_rad is None:
            if st.error:
                self.arm_status.setText(f"读状态失败: {st.error}")
            return
        self._update_joint_display(list(st.joints_rad), armed=st.armed, endpoint=st.endpoint)
        self._refresh_action_state()

    # ---- record / live ----
    def _toggle_live(self, checked: bool) -> None:
        if checked and self.camera.is_multi:
            self.live_cb.blockSignals(True)
            self.live_cb.setChecked(False)
            self.live_cb.blockSignals(False)
            QMessageBox.information(self, "实时计算", "多相机模式下不支持实时求解，请录制后使用「离线计算」。")
            self.live.set_enabled(False)
            self._refresh_action_state()
            return
        self.live.set_enabled(checked)
        if checked:
            self.live.reset()
        self._refresh_action_state()

    def _start_record(self) -> None:
        self._sync_intrinsics_from_spins()
        if self.camera.is_multi and self.live_cb.isChecked():
            self.live_cb.setChecked(False)
        res = self.record.start()
        if isinstance(res, Ok):
            n = self.camera.n_open
            self.session_label.setText(f"session: {res.value}  ({n} 路视频)")
            if self.live_cb.isChecked() and not self.camera.is_multi:
                self.live.reset()
                self.live.set_enabled(True)
            else:
                self.live.set_enabled(False)
            self.statusBar().showMessage(f"录制中… ({n} 相机)")
        else:
            QMessageBox.warning(self, "录制", str(res.error))
        self._refresh_action_state()

    def _stop_record(self) -> None:
        # Persist live result before closing session writers/meta.
        if self.live.last_result is not None and self.record.session_dir is not None:
            path = self.live.persist(self.record.session_dir)
            if path is not None:
                self.statusBar().showMessage(f"实时外参已写入 {path.name}")
        res = self.record.stop()
        if isinstance(res, Ok):
            self.statusBar().showMessage(f"已保存 {res.value}")
        else:
            QMessageBox.warning(self, "录制", str(res.error))
        self._refresh_action_state()

    def _export_live(self) -> None:
        if self.camera.is_multi:
            QMessageBox.information(self, "导出", "多相机请用「离线计算」为每台相机导出外参")
            return
        if self.live.last_result is None or self.record.session_dir is None:
            QMessageBox.information(self, "导出", "无可用外参或 session")
            return
        path = self.live.persist(self.record.session_dir)
        QMessageBox.information(self, "导出", f"已写入 {path}")
        self._refresh_action_state()

    def _session_report(self) -> None:
        sess = self.record.session_dir
        if sess is None:
            QMessageBox.information(self, "结论", "无 session")
            return
        res = build_session_report(sess)
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "结论", str(res.error))
            return
        report = res.value["report"]
        out = Path(sess) / "session_report.txt"
        try:
            out.write_text(report + "\n", encoding="utf-8")
        except Exception:  # noqa: BLE001
            out = None
        dlg = QDialog(self)
        dlg.setWindowTitle(f"Session 结论 · {Path(sess).name}")
        dlg.resize(720, 520)
        lay = QVBoxLayout(dlg)
        te = QTextEdit()
        te.setReadOnly(True)
        te.setPlainText(report)
        lay.addWidget(te)
        btn = QPushButton("关闭")
        btn.clicked.connect(dlg.accept)
        lay.addWidget(btn)
        dlg.exec_()
        tip = "结论已生成" + (f"并写入 {out.name}" if out else "")
        self.statusBar().showMessage(tip)
        self.calib_label.setText(self.calib_label.text() + " | 结论已生成")

    def _on_setup_changed(self, _idx: int = 0) -> None:
        setup = self.setup_combo.currentData()
        if not setup:
            return
        self.cfg.handeye.setup = str(setup)
        self.live.reset()
        tip = "眼在手上" if setup == "eye_in_hand" else "眼在手外"
        self.statusBar().showMessage(f"手眼模式: {tip} ({setup})")
        try:
            save_config(self.cfg)
        except Exception:  # noqa: BLE001
            pass
        self._refresh_action_state()

    def _on_live_result(self, result: ExtrinsicsResult, status: ConvergenceStatus) -> None:
        flag = "已收敛" if status.converged else "未收敛"
        primary = "T_flange_cam" if result.setup == "eye_in_hand" else "T_base_cam"
        extra = f" | {status.warning}" if status.warning else ""
        hist = ""
        if self.live._history_path is not None:
            hist = f" | 求解#{self.live._solve_idx}→{self.live._history_path.name}"
        self.calib_label.setText(
            f"标定[{result.setup}→{primary}]: 样本 {status.n_valid} | {flag} | "
            f"Δt={status.delta_t_mm:.2f}mm ΔR={status.delta_R_deg:.2f}° | "
            f"streak={status.stable_streak} | method={result.method}{extra}{hist}"
        )
        self._refresh_action_state()

    def _on_frames(self, images: dict, ts: float, preview_serial: object) -> None:
        self._sync_intrinsics_from_spins()
        sample = self.record.on_frames(ts, images)
        serial = preview_serial if isinstance(preview_serial, str) else self.camera.preview_serial
        if not serial or serial not in images:
            serial = next(iter(images), None)
        if serial is None:
            return
        image = images[serial]
        show = image
        intr = self.camera.current_intrinsics(serial)
        if intr is not None:
            try:
                _, dbg = detect_target(image, intr, self.cfg.aruco, draw=True)
                if dbg is not None:
                    show = dbg
            except Exception as e:  # noqa: BLE001
                self.statusBar().showMessage(f"标定板: {e}")
            # 实时求解：仅单相机
            if (
                sample is not None
                and self.live.enabled
                and not self.camera.is_multi
                and len(images) == 1
            ):
                try:
                    self.live.offer_frame(
                        image,
                        sample,
                        intr,
                        on_result=self._on_live_result,
                        session_dir=self.record.session_dir,
                    )
                except Exception as e:  # noqa: BLE001
                    self.statusBar().showMessage(f"实时标定: {e}")
        self.preview.show_bgr(show)

    # ---- dialogs ----
    def _apply_max_delta(self) -> None:
        deg = float(self.spin_max_delta.value())
        res = self.arm.set_max_delta_deg(deg)
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "max_delta", str(res.error))
            return
        self.spin_max_delta.setValue(float(res.value))
        try:
            save_config(self.cfg)
        except Exception:  # noqa: BLE001
            pass
        live = self.arm.adapter.get_max_delta_deg()
        live_s = f"{live:.2f}°" if live == live else "未连接"
        self.statusBar().showMessage(f"max_delta_deg={res.value:.2f}°（驱动={live_s}）")

    def _apply_v_norm(self) -> None:
        res = self.arm.set_v_norm_rad_s(float(self.spin_v_norm.value()))
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "v_norm", str(res.error))
            return
        self.spin_v_norm.setValue(float(res.value))
        try:
            save_config(self.cfg)
        except Exception:  # noqa: BLE001
            pass
        self.statusBar().showMessage(f"v_norm_rad_s={res.value:.3f} rad/s")

    def _on_board_type_changed(self, _idx: int = 0) -> None:
        bt = self.board_type_combo.currentData() or "charuco"
        a = self.cfg.aruco
        if bt == "charuco":
            self.spin_mk_mm.setValue(float(getattr(a, "charuco_marker_length_m", 0.03)) * 1000.0)
        else:
            self.spin_mk_mm.setValue(float(a.marker_length_m) * 1000.0)
        self._refresh_board_hint()

    def _refresh_board_hint(self) -> None:
        bt = self.board_type_combo.currentData() or "charuco"
        is_charuco = bt == "charuco"
        self.board_charuco_row.setVisible(is_charuco)
        self.spin_marker_id.setVisible(not is_charuco)
        if is_charuco:
            self.board_hint.setText("ChArUco：格数 × 格边长(mm) + 码边长(mm，须小于格边长)")
        else:
            self.board_hint.setText("单 ArUco：字典 + marker id + 码边长(mm)")

    def _sync_board_widgets_from_cfg(self) -> None:
        a = self.cfg.aruco
        bi = self.board_type_combo.findData(getattr(a, "board_type", "charuco"))
        self.board_type_combo.blockSignals(True)
        self.board_type_combo.setCurrentIndex(bi if bi >= 0 else 0)
        self.board_type_combo.blockSignals(False)
        self.dict_edit.setText(str(a.dict))
        self.spin_sq_x.setValue(int(getattr(a, "squares_x", 5)))
        self.spin_sq_y.setValue(int(getattr(a, "squares_y", 7)))
        self.spin_sq_mm.setValue(float(getattr(a, "square_length_m", 0.04)) * 1000.0)
        self.spin_marker_id.setValue(int(a.marker_id))
        if str(getattr(a, "board_type", "charuco")).lower() in ("aruco", "single", "marker"):
            self.spin_mk_mm.setValue(float(a.marker_length_m) * 1000.0)
        else:
            self.spin_mk_mm.setValue(float(getattr(a, "charuco_marker_length_m", 0.03)) * 1000.0)
        self._refresh_board_hint()

    def _apply_board(self) -> None:
        a = self.cfg.aruco
        bt = self.board_type_combo.currentData() or "charuco"
        a.board_type = str(bt)
        a.dict = self.dict_edit.text().strip() or "DICT_5X5_50"
        mk_m = float(self.spin_mk_mm.value()) / 1000.0
        if mk_m < 0.001:
            QMessageBox.warning(self, "标定板", "码边长过小")
            return
        if a.board_type == "charuco":
            a.squares_x = int(self.spin_sq_x.value())
            a.squares_y = int(self.spin_sq_y.value())
            a.square_length_m = float(self.spin_sq_mm.value()) / 1000.0
            a.charuco_marker_length_m = mk_m
            if a.charuco_marker_length_m >= a.square_length_m:
                QMessageBox.warning(self, "标定板", "ChArUco 码边长必须小于格边长")
                return
        else:
            a.marker_id = int(self.spin_marker_id.value())
            a.marker_length_m = mk_m
        try:
            save_config(self.cfg)
        except Exception:  # noqa: BLE001
            pass
        self._refresh_board_hint()
        tip = (
            f"ChArUco {a.squares_x}×{a.squares_y} "
            f"格{a.square_length_m*1000:.1f}mm 码{a.charuco_marker_length_m*1000:.1f}mm"
            if a.board_type == "charuco"
            else f"ArUco id={a.marker_id} 边长{a.marker_length_m*1000:.1f}mm"
        )
        self.statusBar().showMessage(f"标定板已应用: {a.dict} · {tip}")

    def _open_params(self) -> None:
        dlg = ParamsDialog(self.cfg, self)
        if dlg.exec_():
            dlg.apply()
            self.live.tracker.params.min_samples = self.cfg.convergence.min_samples
            self.live.tracker.params.max_delta_t_mm = self.cfg.convergence.max_delta_t_mm
            self.live.tracker.params.max_delta_R_deg = self.cfg.convergence.max_delta_R_deg
            self.live.tracker.params.stable_count = self.cfg.convergence.stable_count
            self.spin_max_delta.setValue(float(self.cfg.arm.max_delta_deg))
            self.spin_v_norm.setValue(float(self.cfg.arm.v_norm_rad_s))
            self._sync_board_widgets_from_cfg()
            pushed = self.arm.set_max_delta_deg(self.cfg.arm.max_delta_deg)
            if not isinstance(pushed, Ok) and self.arm.adapter.is_connected:
                QMessageBox.warning(self, "max_delta", str(pushed.error))
            self.arm.set_v_norm_rad_s(self.cfg.arm.v_norm_rad_s)
            try:
                save_config(self.cfg)
            except Exception:  # noqa: BLE001
                pass

    def _open_matrix_path(self) -> None:
        MatrixPathDialog(
            self.cfg,
            self.arm,
            self,
            record=self.record,
            live=self.live,
        ).exec_()

    def _open_offline(self) -> None:
        OfflineDialog(self.cfg, self).exec_()

    def _open_verify(self) -> None:
        VerifyDialog(self.cfg, self).exec_()

    def closeEvent(self, event) -> None:  # noqa: N802
        self.cam_worker.stop()
        self.arm_worker.stop()
        self.cam_worker.wait(1000)
        self.arm_worker.wait(1000)
        self.camera.close()
        self.arm.disconnect()
        super().closeEvent(event)
