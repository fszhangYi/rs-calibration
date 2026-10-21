from __future__ import annotations

from pathlib import Path

from rs_calib.app_services.offline_calib_service import OfflineCalibService
from rs_calib.config.schema import AppConfig
from rs_calib.errors import Ok
from rs_calib.infra import session_store
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
        self.resize(720, 480)
        layout = QVBoxLayout(self)

        row = QHBoxLayout()
        self.path_edit = QLineEdit()
        btn = QPushButton("选择 session 目录")
        btn.clicked.connect(self._browse)
        row.addWidget(self.path_edit)
        row.addWidget(btn)
        layout.addLayout(row)

        self.run_btn = QPushButton("开始计算（自动处理 session 内全部相机）")
        self.run_btn.clicked.connect(self._run)
        layout.addWidget(self.run_btn)

        self.log = QTextEdit()
        self.log.setReadOnly(True)
        layout.addWidget(self.log)
        layout.addWidget(
            QLabel(
                "按当前 handeye.setup 求解（眼在手外→T_base_cam，眼在手上→T_flange_cam）。\n"
                "单相机 → session/extrinsics.json；"
                "多相机 → cameras/<serial>/extrinsics.json 与 extrinsics_<serial>.json"
            )
        )

    def _browse(self) -> None:
        d = QFileDialog.getExistingDirectory(self, "选择 session 目录")
        if d:
            self.path_edit.setText(d)

    def _run(self) -> None:
        path = self.path_edit.text().strip()
        if not path:
            self.log.append("请先选择目录")
            return
        session = Path(path)
        self.log.append(f"运行: {session}")
        try:
            meta = session_store.read_meta(session)
            cams = session_store.list_cameras_from_meta(meta)
            self.log.append(f"检测到 {len(cams)} 台相机: " + ", ".join(c.get("serial", "?") for c in cams))
        except Exception as e:  # noqa: BLE001
            self.log.append(f"读取 meta 失败: {e}")
            return

        svc = OfflineCalibService(self.cfg)

        def progress(serial, i, n):
            if i == n or i % 10 == 0:
                self.log.append(f"[{serial}] 进度 {i}/{n}")

        if len(cams) <= 1:
            serial = cams[0]["serial"] if cams else None
            result = svc.run(session, camera_serial=serial, progress_cb=lambda s, i, n: progress(s, i, n))
            if isinstance(result, Ok):
                data = result.value
                r = data["result"]
                rep = data["report"]
                self.log.append(
                    f"成功 [{data.get('camera_serial')}] method={r.method} n={r.n_samples} "
                    f"selected={rep.n_selected} raw_detect={rep.n_after_detect}"
                )
                self.log.append(f"写出: {data['extrinsics_path']}")
                for w in data.get("warnings") or []:
                    self.log.append(f"警告: {w}")
                setup = getattr(r, "setup", "eye_to_hand")
                self.log.append(f"setup={setup}")
                if setup == "eye_in_hand" and getattr(r, "T_flange_cam", None) is not None:
                    self.log.append(f"T_flange_cam:\n{r.T_flange_cam}")
                else:
                    self.log.append(f"T_base_cam:\n{r.T_base_cam}")
            else:
                self.log.append(str(result.error))
            return

        bundle = svc.run_all(session, progress_cb=progress)
        if not isinstance(bundle, Ok):
            self.log.append(str(bundle.error))
            return
        data = bundle.value
        self.log.append(f"完成: 成功 {data['n_ok']} / 失败 {data['n_fail']}")
        for one in data["results"]:
            r = one["result"]
            rep = one["report"]
            setup = getattr(r, "setup", "eye_to_hand")
            self.log.append(
                f"  ✓ [{one['camera_serial']}] setup={setup} method={r.method} n={r.n_samples} "
                f"selected={rep.n_selected} → {one['extrinsics_path']}"
            )
            for w in one.get("warnings") or []:
                self.log.append(f"    警告: {w}")
        for err in data["errors"]:
            self.log.append(f"  ✗ [{err['camera_serial']}] {err['error']}")
