from __future__ import annotations

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
    QSplitter,
    QStatusBar,
    QVBoxLayout,
    QWidget,
)
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

        self.cam_worker = CameraWorker(self.camera.adapter)
        self.cam_worker.frame_ready.connect(self._on_frame)
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
        splitter = QSplitter()
        outer.addWidget(splitter)

        left = QWidget()
        left_l = QVBoxLayout(left)

        # Camera
        cam_box = QGroupBox("相机")
        cam_l = QVBoxLayout(cam_box)
        self.device_list = QListWidget()
        cam_l.addWidget(self.device_list)
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

        jog = QGridLayout()
        for i in range(6):
            jm = QPushButton(f"J{i+1}-")
            jp = QPushButton(f"J{i+1}+")
            jm.clicked.connect(lambda _=False, idx=i: self._jog_joint(idx, -1))
            jp.clicked.connect(lambda _=False, idx=i: self._jog_joint(idx, +1))
            jog.addWidget(jm, i, 0)
            jog.addWidget(jp, i, 1)
        arm_l.addLayout(jog)

        cart = QGridLayout()
        axes = [("X", "x"), ("Y", "y"), ("Z", "z"), ("Rx", "rx"), ("Ry", "ry"), ("Rz", "rz")]
        for r, (lab, ax) in enumerate(axes):
            bm = QPushButton(f"{lab}-")
            bp = QPushButton(f"{lab}+")
            bm.clicked.connect(lambda _=False, a=ax: self._jog_cart(a, -1))
            bp.clicked.connect(lambda _=False, a=ax: self._jog_cart(a, +1))
            cart.addWidget(bm, r, 0)
            cart.addWidget(bp, r, 1)
        arm_l.addLayout(cart)
        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["flange", "tcp"])
        self.frame_combo.setCurrentText(self.cfg.arm.cart_frame)
        self.frame_combo.currentTextChanged.connect(self._on_frame_changed)
        arm_l.addWidget(QLabel("笛卡尔参考"))
        arm_l.addWidget(self.frame_combo)
        self.pose_label = QLabel("法兰 pose: -")
        self.pose_label.setWordWrap(True)
        arm_l.addWidget(self.pose_label)
        left_l.addWidget(arm_box)

        # Record / live
        rec_box = QGroupBox("录制 / 实时标定")
        rec_l = QVBoxLayout(rec_box)
        self.live_cb = QCheckBox("实时计算")
        self.live_cb.toggled.connect(self._toggle_live)
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
            ("离线计算", self._open_offline),
            ("验证", self._open_verify),
        ):
            b = QPushButton(text)
            b.clicked.connect(slot)
            tools.addWidget(b)
        left_l.addLayout(tools)
        left_l.addStretch(1)

        center = QWidget()
        cl = QVBoxLayout(center)
        self.preview = PreviewLabel()
        cl.addWidget(self.preview)

        splitter.addWidget(left)
        splitter.addWidget(center)
        splitter.setStretchFactor(1, 1)
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
        self.statusBar().showMessage(f"发现 {len(res.value)} 台相机")

    def _open_camera(self) -> None:
        item = self.device_list.currentItem()
        if item is None:
            QMessageBox.information(self, "提示", "请先选择相机")
            return
        serial = item.data(256)
        self.camera.device_name = item.data(257) or ""
        res = self.camera.open(
            serial,
            self.cfg.camera.width,
            self.cfg.camera.height,
            self.cfg.camera.fps,
        )
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "开流失败", str(res.error))
            return
        self._fill_intrinsics(res.value)
        self.statusBar().showMessage(f"已开流 {serial}")

    def _close_camera(self) -> None:
        self.camera.close()
        self.statusBar().showMessage("相机已关闭")

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
        # sync editable ui into service
        self._sync_intrinsics_from_spins()

    def _sync_intrinsics_from_spins(self) -> None:
        intr = self.camera.ui_intrinsics
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
            self.arm_status.setText(f"已连接 {res.value.endpoint}")
            self.statusBar().showMessage(self.arm_status.text())
        else:
            self.arm_status.setText("连接失败")
            QMessageBox.warning(self, "机械臂", str(res.error))

    def _disconnect_arm(self) -> None:
        self.arm.disconnect()
        self.arm_status.setText("未连接")

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
            return
        T, xyzrpy = self.arm.flange_from_joints(st.joints_rad)
        _ = T
        armed = "Armed" if st.armed else "Connected"
        self.arm_status.setText(f"{armed} | {st.endpoint}")
        self.pose_label.setText(
            "法兰 xyzrpy: " + ", ".join(f"{v:.3f}" for v in xyzrpy)
        )

    # ---- record / live ----
    def _toggle_live(self, checked: bool) -> None:
        self.live.set_enabled(checked)
        if checked:
            self.live.reset()

    def _start_record(self) -> None:
        self._sync_intrinsics_from_spins()
        res = self.record.start()
        if isinstance(res, Ok):
            self.session_label.setText(f"session: {res.value}")
            if self.live_cb.isChecked():
                self.live.reset()
                self.live.set_enabled(True)
            self.statusBar().showMessage("录制中…")
        else:
            QMessageBox.warning(self, "录制", str(res.error))

    def _stop_record(self) -> None:
        res = self.record.stop()
        if isinstance(res, Ok):
            self.statusBar().showMessage(f"已保存 {res.value}")
        else:
            QMessageBox.warning(self, "录制", str(res.error))

    def _export_live(self) -> None:
        if self.live.last_result is None or self.record.session_dir is None:
            QMessageBox.information(self, "导出", "无可用外参或 session")
            return
        path = session_store.save_extrinsics(self.record.session_dir, self.live.last_result)
        QMessageBox.information(self, "导出", f"已写入 {path}")

    def _on_live_result(self, result: ExtrinsicsResult, status: ConvergenceStatus) -> None:
        flag = "已收敛" if status.converged else "未收敛"
        self.calib_label.setText(
            f"标定: 样本 {status.n_valid} | {flag} | "
            f"Δt={status.delta_t_mm:.2f}mm ΔR={status.delta_R_deg:.2f}° | "
            f"streak={status.stable_streak} | method={result.method}"
        )

    def _on_frame(self, image: np.ndarray, ts: float) -> None:
        self._sync_intrinsics_from_spins()
        sample = self.record.on_frame(ts, image)
        show = image
        intr = self.camera.current_intrinsics()
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
            if sample is not None and self.live.enabled:
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
