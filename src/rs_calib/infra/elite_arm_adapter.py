from __future__ import annotations

import math
import time
from typing import Any, Optional

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

    def move_joints_rad(self, joints_rad: list[float]) -> dict[str, Any]:
        self._require_armed()
        return dict(self._sensor.write({"joints_rad": list(joints_rad)}))

    def _require(self) -> None:
        if self._sensor is None:
            raise AppError("ARM_CONNECT_FAIL", "机械臂未连接")

    def _require_armed(self) -> None:
        self._require()
        if not self.armed:
            raise AppError("ARM_NOT_ARMED", "请先点击 Arm 上使能")
