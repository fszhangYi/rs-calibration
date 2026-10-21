from __future__ import annotations

from pathlib import Path
from typing import Optional

from rs_calib.app_services.arm_service import ArmService
from rs_calib.app_services.live_calib_service import LiveCalibService
from rs_calib.app_services.matrix_path_service import MatrixPathService
from rs_calib.app_services.record_service import RecordService
from rs_calib.config.schema import AppConfig
from rs_calib.domain.matrix_path import MatrixPathParams, MatrixWaypoint
from rs_calib.errors import Ok
from rs_calib.qt_compat import (
    QCheckBox,
    QComboBox,
    QDialog,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QSplitter,
    QTextEdit,
    QVBoxLayout,
    QWidget,
    Qt,
)
from rs_calib.ui.matrix_viz_bridge import MatrixVizPanel
from rs_calib.workers.matrix_path_worker import MatrixPathWorker


class MatrixPathDialog(QDialog):
    """Generate a pose grid around the current arm pose and execute it."""

    def __init__(
        self,
        cfg: AppConfig,
        arm: ArmService,
        parent=None,
        *,
        record: Optional[RecordService] = None,
        live: Optional[LiveCalibService] = None,
    ):
        super().__init__(parent)
        self.cfg = cfg
        self.arm = arm
        self.record = record
        self.live = live
        self.svc = MatrixPathService(cfg, arm)
        self._waypoints: list[MatrixWaypoint] = []
        self._seed: Optional[list[float]] = None
        self._worker: Optional[MatrixPathWorker] = None
        self._stopped_for_converge = False

        self.setWindowTitle("矩阵走位 · Cell A EC616")
        self.resize(1100, 720)

        splitter = QSplitter()
        try:
            splitter.setOrientation(Qt.Horizontal)
        except Exception:  # noqa: BLE001
            splitter.setOrientation(Qt.Orientation.Horizontal)

        left = QWidget()
        root = QVBoxLayout(left)

        setup = getattr(cfg.handeye, "setup", "eye_to_hand")
        board_tip = (
            "眼在手上：标定板固定在桌上，腕部相机随臂运动采点。"
            if setup == "eye_in_hand"
            else "眼在手外：标定板装在末端，场景相机固定采点。"
        )
        tip = QLabel(
            "先手动点动到标定板视野中心，再「捕获当前位姿」作为 6 维网格中心。\n"
            f"{board_tip}\n"
            "点位 = 平移网格(Nx×Ny×Nz) × 姿态网格(Nrx×Nry×Nrz)，相对种子位姿的 Δxyz + Δrpy。\n"
            "「开始走位」会自动重新开录；实时求解收敛后自动停走位并停录。\n"
            "右侧三维画布借用 sensors-arm Cell A · Elite EC616 URDF；生成点位后自动刷新。"
        )
        tip.setWordWrap(True)
        root.addWidget(tip)

        form = QFormLayout()
        mp = cfg.matrix_path

        self.spin_nx = QSpinBox()
        self.spin_nx.setRange(1, 11)
        self.spin_nx.setValue(mp.nx)
        self.spin_ny = QSpinBox()
        self.spin_ny.setRange(1, 11)
        self.spin_ny.setValue(mp.ny)
        self.spin_nz = QSpinBox()
        self.spin_nz.setRange(1, 5)
        self.spin_nz.setValue(mp.nz)
        form.addRow("平移 Nx", self.spin_nx)
        form.addRow("平移 Ny", self.spin_ny)
        form.addRow("平移 Nz", self.spin_nz)

        self.spin_sx = QDoubleSpinBox()
        self.spin_sx.setRange(0, 300)
        self.spin_sx.setDecimals(1)
        self.spin_sx.setSuffix(" mm")
        self.spin_sx.setValue(mp.step_x_m * 1000)
        self.spin_sy = QDoubleSpinBox()
        self.spin_sy.setRange(0, 300)
        self.spin_sy.setDecimals(1)
        self.spin_sy.setSuffix(" mm")
        self.spin_sy.setValue(mp.step_y_m * 1000)
        self.spin_sz = QDoubleSpinBox()
        self.spin_sz.setRange(0, 300)
        self.spin_sz.setDecimals(1)
        self.spin_sz.setSuffix(" mm")
        self.spin_sz.setValue(mp.step_z_m * 1000)
        form.addRow("步进 ΔX", self.spin_sx)
        form.addRow("步进 ΔY", self.spin_sy)
        form.addRow("步进 ΔZ", self.spin_sz)

        self.spin_nrx = QSpinBox()
        self.spin_nrx.setRange(1, 7)
        self.spin_nrx.setValue(getattr(mp, "n_rx", 1))
        self.spin_nry = QSpinBox()
        self.spin_nry.setRange(1, 7)
        self.spin_nry.setValue(getattr(mp, "n_ry", 3))
        self.spin_nrz = QSpinBox()
        self.spin_nrz.setRange(1, 7)
        self.spin_nrz.setValue(getattr(mp, "n_rz", 1))
        form.addRow("姿态 Nrx", self.spin_nrx)
        form.addRow("姿态 Nry", self.spin_nry)
        form.addRow("姿态 Nrz", self.spin_nrz)

        self.spin_srx = QDoubleSpinBox()
        self.spin_srx.setRange(0, 45)
        self.spin_srx.setDecimals(1)
        self.spin_srx.setSuffix(" °")
        self.spin_srx.setValue(getattr(mp, "step_rx_deg", 10.0))
        self.spin_sry = QDoubleSpinBox()
        self.spin_sry.setRange(0, 45)
        self.spin_sry.setDecimals(1)
        self.spin_sry.setSuffix(" °")
        self.spin_sry.setValue(getattr(mp, "step_ry_deg", 12.0))
        self.spin_srz = QDoubleSpinBox()
        self.spin_srz.setRange(0, 45)
        self.spin_srz.setDecimals(1)
        self.spin_srz.setSuffix(" °")
        self.spin_srz.setValue(getattr(mp, "step_rz_deg", 10.0))
        form.addRow("步进 ΔRx", self.spin_srx)
        form.addRow("步进 ΔRy", self.spin_sry)
        form.addRow("步进 ΔRz", self.spin_srz)

        self.spin_dwell = QDoubleSpinBox()
        self.spin_dwell.setRange(0, 10)
        self.spin_dwell.setDecimals(2)
        self.spin_dwell.setSuffix(" s")
        self.spin_dwell.setValue(mp.dwell_s)
        form.addRow("到位停顿", self.spin_dwell)

        self.frame_combo = QComboBox()
        self.frame_combo.addItems(["flange", "tcp"])
        self.frame_combo.setCurrentText(mp.frame)
        form.addRow("参考系", self.frame_combo)

        self.cb_snake = QCheckBox("蛇形走位（缩短平移行程）")
        self.cb_snake.setChecked(mp.snake)
        form.addRow(self.cb_snake)
        root.addLayout(form)

        self.seed_label = QLabel("种子位姿: （尚未捕获）")
        self.seed_label.setWordWrap(True)
        root.addWidget(self.seed_label)

        row1 = QHBoxLayout()
        self.btn_capture = QPushButton("捕获当前位姿")
        self.btn_capture.clicked.connect(self._capture)
        self.btn_plan = QPushButton("生成点位")
        self.btn_plan.clicked.connect(self._plan)
        row1.addWidget(self.btn_capture)
        row1.addWidget(self.btn_plan)
        root.addLayout(row1)

        self.summary = QLabel("点位: 0")
        root.addWidget(self.summary)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        root.addWidget(self.log, 1)

        log_row = QHBoxLayout()
        self.btn_log_clear = QPushButton("清除日志")
        self.btn_log_clear.clicked.connect(self._clear_log)
        self.btn_log_export = QPushButton("导出日志")
        self.btn_log_export.clicked.connect(self._export_log)
        log_row.addWidget(self.btn_log_clear)
        log_row.addWidget(self.btn_log_export)
        log_row.addStretch(1)
        root.addLayout(log_row)

        row2 = QHBoxLayout()
        self.btn_start = QPushButton("开始走位")
        self.btn_start.clicked.connect(self._start)
        self.btn_stop = QPushButton("停止")
        self.btn_stop.clicked.connect(self._stop)
        self.btn_stop.setEnabled(False)
        self.btn_close = QPushButton("关闭")
        self.btn_close.clicked.connect(self.reject)
        row2.addWidget(self.btn_start)
        row2.addWidget(self.btn_stop)
        row2.addWidget(self.btn_close)
        root.addLayout(row2)

        self.status = QLabel("就绪")
        root.addWidget(self.status)

        self.viz_panel = MatrixVizPanel()
        splitter.addWidget(left)
        splitter.addWidget(self.viz_panel)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        try:
            splitter.setSizes([420, 680])
        except Exception:  # noqa: BLE001
            pass

        outer = QVBoxLayout(self)
        outer.setContentsMargins(8, 8, 8, 8)
        outer.addWidget(splitter)

        try:
            self.viz_panel.ensure_started()
        except Exception as e:  # noqa: BLE001
            self.log.append(f"三维画布启动失败: {e}")

    def _current_joints(self) -> Optional[list[float]]:
        try:
            st = self.arm.poll()
            if st is not None and getattr(st, "ok", False) and st.joints_rad is not None:
                return list(st.joints_rad)
        except Exception:  # noqa: BLE001
            pass
        return None

    def _params_from_ui(self) -> MatrixPathParams:
        return MatrixPathParams(
            nx=self.spin_nx.value(),
            ny=self.spin_ny.value(),
            nz=self.spin_nz.value(),
            step_x_m=self.spin_sx.value() / 1000.0,
            step_y_m=self.spin_sy.value() / 1000.0,
            step_z_m=self.spin_sz.value() / 1000.0,
            n_rx=self.spin_nrx.value(),
            n_ry=self.spin_nry.value(),
            n_rz=self.spin_nrz.value(),
            step_rx_deg=self.spin_srx.value(),
            step_ry_deg=self.spin_sry.value(),
            step_rz_deg=self.spin_srz.value(),
            snake=self.cb_snake.isChecked(),
            dwell_s=self.spin_dwell.value(),
            frame=self.frame_combo.currentText(),
        ).clamp()

    def _capture(self) -> None:
        frame = self.frame_combo.currentText()
        res = self.svc.capture_seed(frame)
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "捕获失败", str(res.error))
            return
        self._seed = res.value
        x, y, z, rx, ry, rz = self._seed
        self.seed_label.setText(
            f"种子位姿 ({frame}): xyz=({x:.3f},{y:.3f},{z:.3f}) m  "
            f"rpy=({rx:.3f},{ry:.3f},{rz:.3f}) rad"
        )
        self.log.append("已捕获当前位姿作为网格中心")
        joints = self._current_joints()
        if joints:
            self.viz_panel.set_joints(joints)

    def _plan(self) -> None:
        params = self._params_from_ui()
        res = self.svc.plan(params, seed_xyzrpy=self._seed)
        if not isinstance(res, Ok):
            QMessageBox.warning(self, "生成失败", str(res.error))
            return
        data = res.value
        self._seed = data["seed_xyzrpy"]
        self._waypoints = list(data["waypoints"])
        s = data["summary"]
        self.summary.setText(
            f"点位: {s['n']}  |  xyz跨度 "
            f"X={s['x_span_m']*1000:.0f} Y={s['y_span_m']*1000:.0f} Z={s['z_span_m']*1000:.0f} mm  |  "
            f"rpy跨度 Rx={s.get('rx_span_deg',0):.1f} Ry={s.get('ry_span_deg',0):.1f} "
            f"Rz={s.get('rz_span_deg',0):.1f} °"
        )
        self.log.append(
            f"已生成 {s['n']} 个点位 "
            f"(平移 {params.nx}×{params.ny}×{params.nz} × 姿态 {params.n_rx}×{params.n_ry}×{params.n_rz})"
        )
        for lab in s.get("labels_preview") or []:
            self.log.append(f"  · {lab}")
        if s["n"] > 8:
            self.log.append(f"  … 共 {s['n']} 点")
        p = params
        mp = self.cfg.matrix_path
        mp.nx, mp.ny, mp.nz = p.nx, p.ny, p.nz
        mp.step_x_m, mp.step_y_m, mp.step_z_m = p.step_x_m, p.step_y_m, p.step_z_m
        mp.n_rx, mp.n_ry, mp.n_rz = p.n_rx, p.n_ry, p.n_rz
        mp.step_rx_deg, mp.step_ry_deg, mp.step_rz_deg = p.step_rx_deg, p.step_ry_deg, p.step_rz_deg
        mp.dwell_s, mp.frame, mp.snake = p.dwell_s, p.frame, p.snake

        joints = self._current_joints()
        self.viz_panel.show_waypoints(self._waypoints, self._seed, joints)
        self.log.append("三维画布已更新（琥珀=种子，青=路径，蓝=当前）")

    def _clear_log(self) -> None:
        self.log.clear()

    def _export_log(self) -> None:
        text = self.log.toPlainText()
        if not text.strip():
            QMessageBox.information(self, "导出日志", "日志为空")
            return
        path, _ = QFileDialog.getSaveFileName(
            self,
            "导出日志",
            "matrix_path_log.txt",
            "Text (*.txt);;All (*.*)",
        )
        if not path:
            return
        try:
            Path(path).write_text(text, encoding="utf-8")
            self.log.append(f"日志已导出: {path}")
        except Exception as e:  # noqa: BLE001
            QMessageBox.warning(self, "导出日志", str(e))

    def _restart_record(self) -> bool:
        """Fresh session for this matrix run; enable live when single-cam."""
        if self.record is None:
            self.log.append("未绑定录制服务，跳过自动录制")
            return True
        cam = getattr(self.record, "camera", None)
        if cam is None or getattr(cam, "n_open", 0) <= 0:
            QMessageBox.warning(self, "走位", "请先打开相机（矩阵走位会自动重新录制）")
            return False
        if self.record.recording:
            stop_res = self.record.stop()
            if not isinstance(stop_res, Ok):
                QMessageBox.warning(self, "走位", f"无法结束上一录制: {stop_res.error}")
                return False
            self.log.append(f"已结束上一录制: {stop_res.value}")
        start_res = self.record.start()
        if not isinstance(start_res, Ok):
            QMessageBox.warning(self, "走位", f"无法开始录制: {start_res.error}")
            return False
        self.log.append(f"已重新开录: {start_res.value}")
        mw = self.parent()
        if self.live is not None:
            multi = bool(getattr(cam, "is_multi", False))
            if multi:
                self.live.set_enabled(False)
                if mw is not None and hasattr(mw, "live_cb"):
                    self.live_cb_sync(False)
            else:
                self.live.reset()
                self.live.set_enabled(True)
                self.live_cb_sync(True)
                self.log.append("已开启实时求解（收敛后自动停走位/停录）")
        if mw is not None and hasattr(mw, "session_label"):
            n = getattr(cam, "n_open", 1)
            mw.session_label.setText(f"session: {start_res.value}  ({n} 路视频)")
        if mw is not None and hasattr(mw, "_refresh_action_state"):
            try:
                mw._refresh_action_state()
            except Exception:  # noqa: BLE001
                pass
        return True

    def live_cb_sync(self, on: bool) -> None:
        mw = self.parent()
        if mw is not None and hasattr(mw, "live_cb"):
            try:
                mw.live_cb.blockSignals(True)
                mw.live_cb.setChecked(on)
                mw.live_cb.blockSignals(False)
            except Exception:  # noqa: BLE001
                pass

    def _stop_record_on_converge(self) -> None:
        if self.record is None or not self.record.recording:
            return
        try:
            if self.live is not None and self.live.last_result is not None:
                path = self.live.persist(self.record.session_dir)
                if path is not None:
                    self.log.append(f"收敛外参已写入 {path.name}")
        except Exception as e:  # noqa: BLE001
            self.log.append(f"写入外参失败: {e}")
        stop_res = self.record.stop()
        if isinstance(stop_res, Ok):
            self.log.append(f"收敛后已停录: {stop_res.value}")
        else:
            self.log.append(f"停录失败: {stop_res.error}")
        mw = self.parent()
        if mw is not None and hasattr(mw, "_refresh_action_state"):
            try:
                mw._refresh_action_state()
            except Exception:  # noqa: BLE001
                pass

    def _live_converged(self) -> bool:
        if self.live is None:
            return False
        st = self.live.last_status
        return bool(st is not None and st.converged)

    def _start(self) -> None:
        if self._worker is not None and self._worker.isRunning():
            return
        if not self._waypoints:
            self._plan()
            if not self._waypoints:
                return
        if not self.arm.adapter.is_connected:
            QMessageBox.warning(self, "走位", "请先连接机械臂")
            return
        if not self.arm.adapter.armed:
            QMessageBox.warning(self, "走位", "请先点击 Arm 使能")
            return
        n = len(self._waypoints)
        reply = QMessageBox.question(
            self,
            "确认走位",
            f"将依次运动到 {n} 个点位（含停顿）。\n"
            "开始后会自动重新录制；收敛后自动停止走位并停录。\n"
            "请确认工作空间无障碍。继续？",
        )
        yes = getattr(QMessageBox, "Yes", None)
        if yes is not None and reply != yes:
            return
        if not self._restart_record():
            return

        params = self._params_from_ui()
        self.svc.reset_cancel()
        self._stopped_for_converge = False
        self._worker = MatrixPathWorker(
            self.svc,
            self._waypoints,
            frame=params.frame,
            dwell_s=params.dwell_s,
            extra_cancel=self._live_converged,
            parent=self,
        )
        self._worker.progress.connect(self._on_progress)
        self._worker.finished_ok.connect(self._on_finished)
        self._worker.failed.connect(self._on_failed)
        self._worker.converged_stop.connect(self._on_converged_stop)
        self.btn_start.setEnabled(False)
        self.btn_stop.setEnabled(True)
        self.btn_capture.setEnabled(False)
        self.btn_plan.setEnabled(False)
        self.status.setText("走位中…")
        self.log.append("—— 开始矩阵走位 ——")
        self._worker.start()

    def _stop(self) -> None:
        if self._worker is not None:
            self._worker.request_cancel()
            self.status.setText("正在停止…")
            self.log.append("用户请求停止")

    def _on_progress(self, i: int, n: int, label: str, phase: str) -> None:
        self.status.setText(f"{phase} {i}/{n}: {label}")
        if phase == "moving":
            self.log.append(f"→ ({i}/{n}) {label}")
        self.viz_panel.set_progress(i, n, label, phase)
        joints = self._current_joints()
        if joints:
            self.viz_panel.set_joints(joints)

    def _on_converged_stop(self) -> None:
        self._stopped_for_converge = True
        self._stop_record_on_converge()

    def _on_finished(self, result: object) -> None:
        self.log.append(f"完成: {result}")
        self.status.setText("走位完成")
        self._unlock_ui()
        QMessageBox.information(self, "矩阵走位", "全部点位已执行完成（未触发收敛停录）")

    def _on_failed(self, msg: str) -> None:
        if self._stopped_for_converge or "收敛" in msg:
            self.log.append(f"收敛停止: {msg}")
            self.status.setText("收敛已停走位/停录")
            self._unlock_ui()
            QMessageBox.information(self, "矩阵走位", f"已收敛，走位与录制已停止。\n{msg}")
            return
        self.log.append(f"失败/中止: {msg}")
        self.status.setText(msg)
        self._unlock_ui()
        QMessageBox.warning(self, "矩阵走位", msg)

    def _unlock_ui(self) -> None:
        self.btn_start.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.btn_capture.setEnabled(True)
        self.btn_plan.setEnabled(True)
        self._worker = None

    def closeEvent(self, event) -> None:  # noqa: N802
        if self._worker is not None and self._worker.isRunning():
            self._worker.request_cancel()
            self._worker.wait(2000)
        try:
            self.viz_panel.shutdown()
        except Exception:  # noqa: BLE001
            pass
        super().closeEvent(event)
