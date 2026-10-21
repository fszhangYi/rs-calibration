from __future__ import annotations

import math
import time
from typing import Any, Optional, Sequence

from rs_calib.domain.types import ArmState
from rs_calib.errors import AppError


class EliteArmAdapter:
    """Wraps sensors EliteArmWriteSensor (connect / arm / jog / read)."""

    def __init__(self) -> None:
        self._sensor: Any = None
        self._ip = ""
        self._port = 54321

    @property
    def is_connected(self) -> bool:
        return self._sensor is not None and bool(getattr(self._sensor, "_opened", False))

    @property
    def armed(self) -> bool:
        if self._sensor is None:
            return False
        return bool(getattr(self._sensor, "armed", False))

    @property
    def endpoint(self) -> str:
        return f"{self._ip}:{self._port}"

    def connect(
        self,
        robot_ip: str,
        *,
        robot_tcp_port: int = 54321,
        num_joints: int = 6,
        max_delta_deg: float = 2.0,
        dry_run: bool = False,
    ) -> ArmState:
        self.disconnect()
        try:
            from sensors.core.base import SensorContext
            from sensors.drivers.arm.elite_write import EliteArmWriteSensor
        except ImportError as e:
            raise AppError("ARM_CONNECT_FAIL", "无法导入 sensors Elite 驱动", str(e)) from e

        cfg = {
            "robot_ip": robot_ip,
            "robot_tcp_port": robot_tcp_port,
            "num_joints": num_joints,
            "max_delta_deg": max_delta_deg,
        }
        ctx = SensorContext(dry_run=dry_run)
        sensor = EliteArmWriteSensor("arm-ec616", cfg, ctx)
        try:
            sensor.open()
        except Exception as e:  # noqa: BLE001
            raise AppError("ARM_CONNECT_FAIL", f"连接机械臂失败: {robot_ip}", str(e)) from e

        self._sensor = sensor
        self._ip = robot_ip
        self._port = robot_tcp_port
        return self.read_state()

    def disconnect(self) -> None:
        if self._sensor is not None:
            try:
                if self.armed:
                    self._sensor.disarm(stop=True)
            except Exception:  # noqa: BLE001
                pass
            try:
                self._sensor.close()
            except Exception:  # noqa: BLE001
                pass
        self._sensor = None

    def arm(self) -> dict[str, Any]:
        self._require()
        return dict(self._sensor.write({"arm": True}))

    def disarm(self, stop: bool = True) -> dict[str, Any]:
        self._require()
        return dict(self._sensor.write({"disarm": True, "stop": stop}))

    def stop(self) -> dict[str, Any]:
        self._require()
        return dict(self._sensor.write({"stop": True}))

    def read_state(self) -> ArmState:
        if self._sensor is None:
            return ArmState(ok=False, joints_rad=None, armed=False, endpoint=self.endpoint, error="未连接")
        try:
            data = dict(self._sensor.read())
        except Exception as e:  # noqa: BLE001
            return ArmState(
                ok=False,
                joints_rad=None,
                armed=self.armed,
                endpoint=self.endpoint,
                error=str(e),
                ts=time.time(),
            )
        joints = data.get("joints_rad")
        err = data.get("error")
        return ArmState(
            ok=joints is not None and not err,
            joints_rad=list(joints) if joints is not None else None,
            armed=bool(data.get("armed", False)),
            endpoint=str(data.get("endpoint", self.endpoint)),
            error=str(err) if err else None,
            ts=float(data.get("ts", time.time())),
        )

    def jog_joint(self, idx: int, delta_rad: float) -> dict[str, Any]:
        self._require_armed()
        return dict(self._sensor.write({"jog_joint": int(idx), "delta_rad": float(delta_rad)}))

    def move_joints_rad(
        self,
        joints_rad: list[float],
        *,
        relative_to_last: bool = False,
        reference_joints_rad: Optional[Sequence[float]] = None,
    ) -> dict[str, Any]:
        """Absolute joint move. Prefer ``relative_to_last`` during segmented paths so
        max_delta is checked vs last command (TT may still be settling on feedback).
        """
        self._require_armed()
        cmd: dict[str, Any] = {"joints_rad": list(joints_rad)}
        if reference_joints_rad is not None:
            cmd["reference_joints_rad"] = [float(x) for x in list(reference_joints_rad)[:6]]
        elif relative_to_last:
            cmd["relative_to_last"] = True
        return dict(self._sensor.write(cmd))

    def get_max_delta_deg(self) -> float:
        if self._sensor is None:
            return float("nan")
        rad = float(getattr(self._sensor, "max_delta_rad", float("nan")))
        return math.degrees(rad) if rad == rad else float("nan")

    def set_max_delta_deg(self, deg: float) -> float:
        """Update live driver single-step |Δq| cap (degrees)."""
        d = abs(float(deg))
        if not (0.01 <= d <= 45.0):
            raise AppError("JOG_REJECTED", "max_delta_deg 范围应为 0.01~45°")
        if self._sensor is not None and hasattr(self._sensor, "max_delta_rad"):
            self._sensor.max_delta_rad = math.radians(d)
        return d

    def sync_last_cmd_from_joints(self, joints_rad: Sequence[float] | list[float]) -> None:
        """Align driver last-command baseline to a known joint pose (avoids false Δ rejects)."""
        if self._sensor is None:
            return
        if hasattr(self._sensor, "_last_cmd_rad"):
            self._sensor._last_cmd_rad = [float(x) for x in list(joints_rad)[:6]]

    def _require(self) -> None:
        if self._sensor is None:
            raise AppError("ARM_CONNECT_FAIL", "机械臂未连接")

    def _require_armed(self) -> None:
        self._require()
        if not self.armed:
            raise AppError("ARM_NOT_ARMED", "请先点击 Arm 上使能")
