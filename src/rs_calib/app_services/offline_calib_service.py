from __future__ import annotations

from pathlib import Path
from typing import Any, Optional

from rs_calib.config.schema import AppConfig
from rs_calib.domain.aruco import aruco_meta_dict, detect_target
from rs_calib.domain.filter_select import FilterParams, filter_and_select
from rs_calib.domain.handeye import available_methods, solve_handeye
from rs_calib.domain.math3d import se3_delta
from rs_calib.domain.types import CalibSample, Intrinsics
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra import session_store
from rs_calib.infra.video_io import SessionVideoReader


class OfflineCalibService:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg

    def run(
        self,
        session_dir: Path,
        *,
        camera_serial: Optional[str] = None,
        intrinsics_override: Optional[Intrinsics] = None,
        progress_cb=None,
    ) -> Result[dict[str, Any]]:
        """Calibrate one camera. If serial is None, uses first/only camera in meta."""
        session_dir = Path(session_dir)
        try:
            meta = session_store.read_meta(session_dir)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", "无法读取 meta.json", str(e)))

        cams = session_store.list_cameras_from_meta(meta)
        if not cams:
            return Err(AppError("CALIB_FAIL", "meta 中无相机信息"))
        if camera_serial is None:
            camera_serial = str(cams[0]["serial"])
        cam = next((c for c in cams if c.get("serial") == camera_serial), None)
        if cam is None:
            return Err(AppError("CALIB_FAIL", f"未找到相机 {camera_serial}"))

        try:
            intr = intrinsics_override or session_store.intrinsics_from_meta(meta, camera_serial)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", "无法解析内参", str(e)))

        samples_flange = session_store.load_flange_samples(session_dir)
        if not samples_flange:
            return Err(AppError("CALIB_FAIL", "joints_poses.jsonl 为空"))

        try:
            reader = SessionVideoReader(session_dir, meta, camera_serial=camera_serial)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", f"无法打开相机视频: {camera_serial}", str(e)))

        detected: list[CalibSample] = []
        joints_map: dict[int, list[float]] = {}
        for i, fs in enumerate(samples_flange):
            if progress_cb:
                progress_cb(camera_serial, i + 1, len(samples_flange))
            img = reader.read_index(fs.frame_idx)
            if img is None:
                continue
            aruco_params = {**aruco_meta_dict(self.cfg.aruco), **(meta.get("aruco") or {})}
            T_cm, _ = detect_target(img, intr, aruco_params, draw=False)
            if T_cm is None:
                continue
            detected.append(
                CalibSample(
                    frame_idx=fs.frame_idx,
                    T_base_flange=fs.T_base_flange,
                    T_cam_marker=T_cm,
                )
            )
            joints_map[fs.frame_idx] = list(fs.joints_rad)
        reader.close()

        fp = FilterParams(
            dup_joint_eps_deg=self.cfg.convergence.dup_joint_eps_deg,
            dup_pose_eps_mm=self.cfg.convergence.dup_pose_eps_mm,
            dup_pose_eps_deg=self.cfg.convergence.dup_pose_eps_deg,
            outlier_jump_mm=self.cfg.convergence.outlier_jump_mm,
            outlier_jump_deg=self.cfg.convergence.outlier_jump_deg,
            min_translation_span_m=self.cfg.convergence.min_translation_span_m,
            min_rotation_span_deg=self.cfg.convergence.min_rotation_span_deg,
            min_samples=self.cfg.convergence.min_samples,
        )
        selected, report = filter_and_select(detected, joints_by_idx=joints_map, params=fp)
        if len(selected) < 3:
            return Err(
                AppError(
                    "CALIB_FAIL",
                    f"相机 {camera_serial} 有效样本不足: {len(selected)}",
                    str(report),
                )
            )

        # Prefer setup recorded in session meta (what was intended at capture time).
        meta_he = meta.get("handeye") if isinstance(meta.get("handeye"), dict) else {}
        setup = str(meta_he.get("setup") or getattr(self.cfg.handeye, "setup", "eye_to_hand"))
        method = str(meta_he.get("method") or self.cfg.handeye.method)
        try:
            result = solve_handeye(
                selected,
                method=method,
                setup=setup,
                dual_backends=bool(getattr(self.cfg.handeye, "dual_backends", False)),
            )
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", f"HandEye 求解失败 ({camera_serial})", str(e)))

        warnings = list(report.warnings)
        if setup != getattr(self.cfg.handeye, "setup", setup):
            warnings.append(
                f"使用 session meta 手眼模式 {setup}（当前界面为 {self.cfg.handeye.setup}）"
            )
        dual = (result.metrics or {}).get("dual") if result.metrics else None
        if isinstance(dual, dict):
            if dual.get("warning"):
                warnings.append(str(dual["warning"]))
            elif dual.get("enabled") and dual.get("primary") == "opencv":
                warnings.append(
                    f"双算法: OpenCV vs 原生 Δt={float(dual.get('delta_t_mm', float('nan'))):.2f}mm "
                    f"ΔR={float(dual.get('delta_R_deg', float('nan'))):.2f}° "
                    f"(主解=OpenCV)"
                )
        if self.cfg.handeye.compare_methods:
            for m in available_methods():
                if m == result.method.split("_")[0]:
                    continue
                try:
                    other = solve_handeye(selected, method=m, setup=setup)
                    dt, dR = se3_delta(result.primary_T(), other.primary_T())
                    if dt > 5.0 or dR > 2.0:
                        warnings.append(f"{m} 与主方法偏差 Δt={dt:.1f}mm ΔR={dR:.2f}°")
                except Exception:  # noqa: BLE001
                    pass

        out_path = session_store.save_extrinsics(
            session_dir,
            result,
            extra={
                "filter_report": report.__dict__,
                "warnings": warnings,
                "camera_serial": camera_serial,
                "setup": result.setup,
            },
            camera_serial=camera_serial if len(cams) > 1 else None,
        )
        # single-cam also keep classic extrinsics.json (save_extrinsics already does when serial None)
        if len(cams) == 1 and camera_serial:
            # ensure root extrinsics.json exists for verify UI
            session_store.save_extrinsics(
                session_dir,
                result,
                extra={"filter_report": report.__dict__, "warnings": warnings, "camera_serial": camera_serial},
                camera_serial=None,
            )
        return Ok(
            {
                "result": result,
                "report": report,
                "warnings": warnings,
                "extrinsics_path": str(out_path),
                "camera_serial": camera_serial,
            }
        )

    def run_all(
        self,
        session_dir: Path,
        *,
        progress_cb=None,
    ) -> Result[dict[str, Any]]:
        """Calibrate every camera listed in meta (multi-cam offline)."""
        session_dir = Path(session_dir)
        try:
            meta = session_store.read_meta(session_dir)
        except Exception as e:  # noqa: BLE001
            return Err(AppError("CALIB_FAIL", "无法读取 meta.json", str(e)))
        cams = session_store.list_cameras_from_meta(meta)
        if not cams:
            return Err(AppError("CALIB_FAIL", "meta 中无相机信息"))

        results = []
        errors = []
        for cam in cams:
            serial = str(cam["serial"])

            def _prog(s, i, n, _serial=serial):
                if progress_cb:
                    progress_cb(_serial, i, n)

            one = self.run(session_dir, camera_serial=serial, progress_cb=_prog)
            if isinstance(one, Ok):
                results.append(one.value)
            else:
                errors.append({"camera_serial": serial, "error": str(one.error)})

        if not results:
            return Err(AppError("CALIB_FAIL", "全部相机标定失败", str(errors)))
        return Ok(
            {
                "results": results,
                "errors": errors,
                "n_ok": len(results),
                "n_fail": len(errors),
            }
        )
