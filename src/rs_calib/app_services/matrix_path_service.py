from __future__ import annotations

import time
from typing import Callable, Optional, Sequence

from rs_calib.app_services.arm_service import ArmService
from rs_calib.config.schema import AppConfig
from rs_calib.domain.matrix_path import (
    MatrixPathParams,
    MatrixWaypoint,
    generate_matrix_waypoints,
    summarize_waypoints,
)
from rs_calib.errors import AppError, Err, Ok, Result


class MatrixPathService:
    """Plan and execute matrix (grid) paths for eye-to-hand calibration coverage."""

    def __init__(self, cfg: AppConfig, arm: ArmService) -> None:
        self.cfg = cfg
        self.arm = arm
        self._cancel = False
        self._running = False

    @property
    def running(self) -> bool:
        return self._running

    def request_cancel(self) -> None:
        self._cancel = True

    def reset_cancel(self) -> None:
        self._cancel = False

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
            }
        )

    def execute(
        self,
        waypoints: Sequence[MatrixWaypoint],
        *,
        frame: str = "flange",
        dwell_s: float = 0.8,
        progress_cb: Optional[Callable[[int, int, MatrixWaypoint, str], None]] = None,
    ) -> Result[dict]:
        """Run waypoints sequentially. Safe to call from a worker thread."""
        if self._running:
            return Err(AppError("PATH_BUSY", "矩阵走位正在进行"))
        if not waypoints:
            return Err(AppError("PATH_EMPTY", "点位列表为空"))
        self._running = True
        self._cancel = False
        reached = 0
        try:
            for i, wp in enumerate(waypoints):
                if self._cancel:
                    return Err(
                        AppError(
                            "PATH_CANCELLED",
                            f"走位已取消（完成 {reached}/{len(waypoints)}）",
                        )
                    )
                if progress_cb:
                    progress_cb(i + 1, len(waypoints), wp, "moving")
                move = self.arm.move_to_xyzrpy(
                    wp.xyzrpy,
                    frame=frame,
                    cancel_cb=lambda: self._cancel,
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
                # dwell with cancel poll
                t_end = time.perf_counter() + max(0.0, float(dwell_s))
                while time.perf_counter() < t_end:
                    if self._cancel:
                        return Err(
                            AppError(
                                "PATH_CANCELLED",
                                f"走位已取消（完成 {reached}/{len(waypoints)}）",
                            )
                        )
                    time.sleep(0.05)
            return Ok({"reached": reached, "total": len(waypoints)})
        finally:
            self._running = False
