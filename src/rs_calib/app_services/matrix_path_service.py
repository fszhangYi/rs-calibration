from __future__ import annotations

import json
import math
import re
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Optional, Sequence

from rs_calib.app_services.arm_service import ArmService
from rs_calib.config.schema import AppConfig
from rs_calib.domain.matrix_path import (
    MatrixPathParams,
    MatrixWaypoint,
    generate_matrix_waypoints,
    spans_from_params,
    summarize_waypoints,
)
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.paths import project_root


def _safe_preset_name(name: str) -> str:
    s = re.sub(r"[^\w\-.]+", "_", (name or "").strip(), flags=re.UNICODE)
    s = s.strip("._-") or "matrix_preset"
    return s[:80]


class MatrixPathService:
    """Plan and execute matrix (grid) paths for eye-to-hand calibration coverage."""

    def __init__(self, cfg: AppConfig, arm: ArmService) -> None:
        self.cfg = cfg
        self.arm = arm
        self._cancel = False
        self._running = False
        self._cancel_reason: Optional[str] = None

    def preset_dir(self) -> Path:
        d = project_root() / "presets" / "matrix_path"
        d.mkdir(parents=True, exist_ok=True)
        return d

    @property
    def running(self) -> bool:
        return self._running

    @property
    def cancel_reason(self) -> Optional[str]:
        return self._cancel_reason

    def request_cancel(self, reason: str = "user") -> None:
        self._cancel = True
        if self._cancel_reason is None:
            self._cancel_reason = reason

    def reset_cancel(self) -> None:
        self._cancel = False
        self._cancel_reason = None

    def default_params(self) -> MatrixPathParams:
        return MatrixPathParams.from_dict(
            self.cfg.matrix_path.to_dict() if hasattr(self.cfg, "matrix_path") else None
        )

    def capture_seed(self, frame: str = "flange") -> Result[list[float]]:
        return self.arm.current_xyzrpy(frame=frame)

    def plan(
        self,
        params: MatrixPathParams,
        *,
        seed_xyzrpy: Optional[Sequence[float]] = None,
    ) -> Result[dict]:
        params = params.clamp()
        if seed_xyzrpy is None:
            seed_res = self.capture_seed(params.frame)
            if not isinstance(seed_res, Ok):
                return Err(seed_res.error)
            seed = seed_res.value
        else:
            seed = [float(x) for x in list(seed_xyzrpy)[:6]]
        waypoints = generate_matrix_waypoints(seed, params)
        if not waypoints:
            return Err(AppError("PATH_EMPTY", "未生成任何点位"))
        return Ok(
            {
                "seed_xyzrpy": seed,
                "params": params.to_dict(),
                "waypoints": waypoints,
                "summary": summarize_waypoints(waypoints),
                "spans": spans_from_params(params),
            }
        )

    def build_preset_payload(
        self,
        params: MatrixPathParams,
        seed_xyzrpy: Sequence[float],
        *,
        name: str = "",
        note: str = "",
    ) -> dict[str, Any]:
        params = params.clamp()
        seed = [float(x) for x in list(seed_xyzrpy)[:6]]
        while len(seed) < 6:
            seed.append(0.0)
        spans = spans_from_params(params)
        return {
            "schema_version": 1,
            "kind": "matrix_path_preset",
            "name": name or "matrix_preset",
            "note": note or "",
            "created_at": datetime.now(timezone.utc).astimezone().isoformat(),
            "params": params.to_dict(),
            "spans": spans,
            # Convenient display copies
            "spans_mm_deg": {
                "x_span_mm": spans["x_span_m"] * 1000.0,
                "y_span_mm": spans["y_span_m"] * 1000.0,
                "z_span_mm": spans["z_span_m"] * 1000.0,
                "rx_span_deg": spans["rx_span_deg"],
                "ry_span_deg": spans["ry_span_deg"],
                "rz_span_deg": spans["rz_span_deg"],
            },
            "seed_xyzrpy": seed,  # m, rad
            "seed_xyzrpy_mm_deg": [
                seed[0] * 1000.0,
                seed[1] * 1000.0,
                seed[2] * 1000.0,
                math.degrees(seed[3]),
                math.degrees(seed[4]),
                math.degrees(seed[5]),
            ],
            "n_waypoints": params.expected_count(),
        }

    def list_presets(self) -> list[dict[str, Any]]:
        out: list[dict[str, Any]] = []
        for path in sorted(self.preset_dir().glob("*.json")):
            try:
                data = json.loads(path.read_text(encoding="utf-8"))
            except Exception:  # noqa: BLE001
                data = {}
            out.append(
                {
                    "name": data.get("name") or path.stem,
                    "file": path.name,
                    "path": str(path),
                    "created_at": data.get("created_at"),
                    "n_waypoints": data.get("n_waypoints"),
                    "spans_mm_deg": data.get("spans_mm_deg"),
                    "note": data.get("note") or "",
                }
            )
        return out

    def save_preset(
        self,
        name: str,
        params: MatrixPathParams,
        seed_xyzrpy: Sequence[float],
        *,
        note: str = "",
    ) -> Result[dict]:
        safe = _safe_preset_name(name)
        payload = self.build_preset_payload(params, seed_xyzrpy, name=safe, note=note)
        path = self.preset_dir() / f"{safe}.json"
        try:
            path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        except Exception as e:  # noqa: BLE001
            return Err(AppError("PRESET_SAVE_FAIL", f"保存失败: {path}", str(e)))
        return Ok({"ok": True, "path": str(path), "file": path.name, "preset": payload})

    def load_preset(self, name_or_file: str) -> Result[dict]:
        raw = (name_or_file or "").strip()
        if not raw:
            return Err(AppError("PRESET_NOT_FOUND", "未指定预设名"))
        path = Path(raw)
        if not path.is_file():
            cand = self.preset_dir() / raw
            if not cand.suffix:
                cand = self.preset_dir() / f"{_safe_preset_name(raw)}.json"
            path = cand
        if not path.is_file():
            return Err(AppError("PRESET_NOT_FOUND", f"找不到预设: {raw}"))
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception as e:  # noqa: BLE001
            return Err(AppError("PRESET_LOAD_FAIL", f"读取失败: {path}", str(e)))
        params = MatrixPathParams.from_dict(data.get("params") or {})
        seed = data.get("seed_xyzrpy")
        if not isinstance(seed, list) or len(seed) < 6:
            return Err(AppError("PRESET_LOAD_FAIL", "预设缺少 seed_xyzrpy"))
        return Ok(
            {
                "ok": True,
                "path": str(path),
                "file": path.name,
                "preset": data,
                "params": params.to_dict(),
                "spans": data.get("spans") or spans_from_params(params),
                "seed_xyzrpy": [float(x) for x in seed[:6]],
            }
        )

    def _should_cancel(self, extra_cancel: Optional[Callable[[], bool]]) -> bool:
        if self._cancel:
            return True
        if extra_cancel is not None:
            try:
                if extra_cancel():
                    self.request_cancel("converge")
                    return True
            except Exception:  # noqa: BLE001
                pass
        return False

    def execute(
        self,
        waypoints: Sequence[MatrixWaypoint],
        *,
        frame: str = "flange",
        dwell_s: float = 0.8,
        progress_cb: Optional[Callable[[int, int, MatrixWaypoint, str], None]] = None,
        extra_cancel: Optional[Callable[[], bool]] = None,
    ) -> Result[dict]:
        """Run waypoints sequentially. Safe to call from a worker thread."""
        if self._running:
            return Err(AppError("PATH_BUSY", "矩阵走位正在进行"))
        if not waypoints:
            return Err(AppError("PATH_EMPTY", "点位列表为空"))
        self._running = True
        self._cancel = False
        self._cancel_reason = None
        reached = 0
        try:
            for i, wp in enumerate(waypoints):
                if self._should_cancel(extra_cancel):
                    code = (
                        "PATH_CONVERGED"
                        if self._cancel_reason == "converge"
                        else "PATH_CANCELLED"
                    )
                    tip = "收敛停止" if code == "PATH_CONVERGED" else "走位已取消"
                    return Err(
                        AppError(code, f"{tip}（完成 {reached}/{len(waypoints)}）")
                    )
                if progress_cb:
                    progress_cb(i + 1, len(waypoints), wp, "moving")
                move = self.arm.move_to_xyzrpy(
                    wp.xyzrpy,
                    frame=frame,
                    cancel_cb=lambda: self._should_cancel(extra_cancel),
                )
                if not isinstance(move, Ok):
                    err = move.error
                    return Err(
                        AppError(
                            err.code,
                            f"点位 {wp.label} 失败: {err.message}",
                            err.detail,
                        )
                    )
                reached += 1
                if progress_cb:
                    progress_cb(i + 1, len(waypoints), wp, "dwell")
                t_end = time.perf_counter() + max(0.0, float(dwell_s))
                while time.perf_counter() < t_end:
                    if self._should_cancel(extra_cancel):
                        code = (
                            "PATH_CONVERGED"
                            if self._cancel_reason == "converge"
                            else "PATH_CANCELLED"
                        )
                        tip = "收敛停止" if code == "PATH_CONVERGED" else "走位已取消"
                        return Err(
                            AppError(code, f"{tip}（完成 {reached}/{len(waypoints)}）")
                        )
                    time.sleep(0.05)
            return Ok({"reached": reached, "total": len(waypoints)})
        finally:
            self._running = False
