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
from rs_calib.domain.aruco import detect_marker
from rs_calib.domain.types import ArmState, ConvergenceStatus, ExtrinsicsResult
from rs_calib.errors import Ok
from rs_calib.infra import session_store
from rs_calib.qt_compat import (
    QCheckBox,
    QComboBox,
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
    QSplitter,
    QStatusBar,
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
        scan_btn = QPushButton("扫描")
        scan_btn.clicked.connect(self._scan_cameras)
        open_btn = QPushButton("开流")
        open_btn.clicked.connect(self._open_camera)
        close_btn = QPushButton("关流")
        close_btn.clicked.connect(self._close_camera)
        row.addWidget(scan_btn)
        row.addWidget(open_btn)
        row.addWidget(close_btn)
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

        # Arm
        arm_box = QGroupBox("机械臂 EC616")
        arm_l = QVBoxLayout(arm_box)
        ip_row = QHBoxLayout()
        self.ip_edit = QLineEdit(self.cfg.arm.robot_ip)
        conn_btn = QPushButton("连接")
        conn_btn.clicked.connect(self._connect_arm)
        disc_btn = QPushButton("断开")
        disc_btn.clicked.connect(self._disconnect_arm)
        ip_row.addWidget(self.ip_edit)
        ip_row.addWidget(conn_btn)
        ip_row.addWidget(disc_btn)
        arm_l.addLayout(ip_row)
        self.arm_status = QLabel("未连接")
        arm_l.addWidget(self.arm_status)
        gate = QHBoxLayout()
        for text, slot in (
            ("Arm", self._arm_enable),
            ("Disarm", self._arm_disarm),
            ("Stop", self._arm_stop),
        ):
            b = QPushButton(text)
            b.clicked.connect(slot)
            gate.addWidget(b)
        arm_l.addLayout(gate)

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
        start_btn = QPushButton("开始录制")
        stop_btn = QPushButton("停止录制")
        start_btn.clicked.connect(self._start_record)
        stop_btn.clicked.connect(self._stop_record)
        row2.addWidget(start_btn)
        row2.addWidget(stop_btn)
        rec_l.addLayout(row2)
        self.session_label = QLabel("session: -")
        self.calib_label = QLabel("标定: 样本 0")
        self.calib_label.setWordWrap(True)
        export_btn = QPushButton("导出当前外参到 session")
        export_btn.clicked.connect(self._export_live)
        rec_l.addWidget(self.session_label)
        rec_l.addWidget(self.calib_label)
        rec_l.addWidget(export_btn)
        left_l.addWidget(rec_box)

        # Menus-ish buttons
        tools = QHBoxLayout()
        for text, slot in (
            ("参数", self._open_params),
            ("矩阵走位", self._open_matrix_path),
            ("离线计算", self._open_offline),
            ("验证", self._open_verify),
        ):
            b = QPushButton(text)
            b.clicked.connect(slot)
            tools.addWidget(b)
        left_l.addLayout(tools)
        left_l.addStretch(0)

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
            self.live_cb.setEnabled(True)
            return
        if n == 1:
            self.cam_mode_label.setText("模式: 单相机（可用实时求解）")
            self.live_cb.setEnabled(True)
        else:
            self.cam_mode_label.setText(f"模式: 多相机 ×{n}（实时求解已禁用，请离线计算）")
            if self.live_cb.isChecked():
                self.live_cb.setChecked(False)
            self.live.set_enabled(False)
            self.live_cb.setEnabled(False)

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

    def _disconnect_arm(self) -> None:
        self.arm.disconnect()
        self.arm_status.setText("未连接")
        self._clear_joint_display()

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

    def _arm_disarm(self) -> None:
        res = self.arm.disarm()
        self.statusBar().showMessage("Disarm" if isinstance(res, Ok) else str(res.error))

    def _arm_stop(self) -> None:
        res = self.arm.stop()
        self.statusBar().showMessage("Stop" if isinstance(res, Ok) else str(res.error))

    def _jog_joint(self, idx: int, sign: int) -> None:
        res = self.arm.jog_joint(idx, sign)
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "关节点动", str(res.error))

    def _jog_cart(self, axis: str, sign: int) -> None:
        res = self.arm.jog_cartesian(axis, sign)
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "笛卡尔点动", str(res.error))

    def _on_frame_changed(self, text: str) -> None:
        self.cfg.arm.cart_frame = text

    def _on_arm_state(self, st: ArmState) -> None:
        if not st.ok or st.joints_rad is None:
            if st.error:
                self.arm_status.setText(f"读状态失败: {st.error}")
            return
        self._update_joint_display(list(st.joints_rad), armed=st.armed, endpoint=st.endpoint)

    # ---- record / live ----
    def _toggle_live(self, checked: bool) -> None:
        if checked and self.camera.is_multi:
            self.live_cb.blockSignals(True)
            self.live_cb.setChecked(False)
            self.live_cb.blockSignals(False)
            QMessageBox.information(self, "实时计算", "多相机模式下不支持实时求解，请录制后使用「离线计算」。")
            self.live.set_enabled(False)
            return
        self.live.set_enabled(checked)
        if checked:
            self.live.reset()

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

    def _stop_record(self) -> None:
        res = self.record.stop()
        if isinstance(res, Ok):
            self.statusBar().showMessage(f"已保存 {res.value}")
        else:
            QMessageBox.warning(self, "录制", str(res.error))

    def _export_live(self) -> None:
        if self.camera.is_multi:
            QMessageBox.information(self, "导出", "多相机请用「离线计算」为每台相机导出外参")
            return
        if self.live.last_result is None or self.record.session_dir is None:
            QMessageBox.information(self, "导出", "无可用外参或 session")
            return
        path = session_store.save_extrinsics(self.record.session_dir, self.live.last_result)
        QMessageBox.information(self, "导出", f"已写入 {path}")

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

    def _on_live_result(self, result: ExtrinsicsResult, status: ConvergenceStatus) -> None:
        flag = "已收敛" if status.converged else "未收敛"
        primary = "T_flange_cam" if result.setup == "eye_in_hand" else "T_base_cam"
        self.calib_label.setText(
            f"标定[{result.setup}→{primary}]: 样本 {status.n_valid} | {flag} | "
            f"Δt={status.delta_t_mm:.2f}mm ΔR={status.delta_R_deg:.2f}° | "
            f"streak={status.stable_streak} | method={result.method}"
        )

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
            _, dbg = detect_marker(
                image,
                intr,
                dict_name=self.cfg.aruco.dict,
                marker_id=self.cfg.aruco.marker_id,
                marker_length_m=self.cfg.aruco.marker_length_m,
                draw=True,
            )
            if dbg is not None:
                show = dbg
            # 实时求解：仅单相机
            if (
                sample is not None
                and self.live.enabled
                and not self.camera.is_multi
                and len(images) == 1
            ):
                self.live.offer_frame(image, sample, intr, on_result=self._on_live_result)
        self.preview.show_bgr(show)

    # ---- dialogs ----
    def _open_params(self) -> None:
        dlg = ParamsDialog(self.cfg, self)
        if dlg.exec_():
            dlg.apply()
            self.live.tracker.params.min_samples = self.cfg.convergence.min_samples
            self.live.tracker.params.max_delta_t_mm = self.cfg.convergence.max_delta_t_mm
            self.live.tracker.params.max_delta_R_deg = self.cfg.convergence.max_delta_R_deg
            self.live.tracker.params.stable_count = self.cfg.convergence.stable_count
            try:
                save_config(self.cfg)
            except Exception:  # noqa: BLE001
                pass

    def _open_matrix_path(self) -> None:
        MatrixPathDialog(self.cfg, self.arm, self).exec_()

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
