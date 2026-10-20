from __future__ import annotations

import math
import time
from typing import Optional, Sequence

from rs_calib.config.schema import AppConfig
from rs_calib.domain.math3d import xyzrpy_from_T
from rs_calib.domain.types import ArmState
from rs_calib.errors import AppError, Err, Ok, Result
from rs_calib.infra.elite_arm_adapter import EliteArmAdapter
from rs_calib.infra.kinematics_bridge import KinematicsBridge


class ArmService:
    def __init__(self, cfg: AppConfig) -> None:
        self.cfg = cfg
        self.adapter = EliteArmAdapter()
        self.kin = KinematicsBridge()
        self._last: Optional[ArmState] = None

    def connect(self, ip: Optional[str] = None, *, dry_run: bool = False) -> Result[ArmState]:
        ip = ip or self.cfg.arm.robot_ip
        try:
            st = self.adapter.connect(
                ip,
                robot_tcp_port=self.cfg.arm.robot_tcp_port,
                num_joints=self.cfg.arm.num_joints,
                max_delta_deg=self.cfg.arm.max_delta_deg,
                dry_run=dry_run,
            )
            self._last = st
            if not st.ok and not dry_run:
                return Err(AppError("ARM_CONNECT_FAIL", st.error or "连接后无法读关节"))
            return Ok(st)
        except AppError as e:
            return Err(e)

    def disconnect(self) -> None:
        self.adapter.disconnect()
        self._last = None

    def arm(self) -> Result[dict]:
        try:
            return Ok(self.adapter.arm())
        except AppError as e:
            return Err(e)

    def disarm(self) -> Result[dict]:
        try:
            return Ok(self.adapter.disarm(stop=True))
        except AppError as e:
            return Err(e)

    def stop(self) -> Result[dict]:
        try:
            return Ok(self.adapter.stop())
        except AppError as e:
            return Err(e)

    def poll(self) -> ArmState:
        st = self.adapter.read_state()
        self._last = st
        return st

    @property
    def last_state(self) -> Optional[ArmState]:
        return self._last

    def flange_from_joints(self, joints_rad: Sequence[float]):
        T = self.kin.fk_flange(joints_rad)
        return T, xyzrpy_from_T(T)

    def jog_joint(self, idx: int, sign: int) -> Result[dict]:
        step = math.radians(self.cfg.arm.jog_joint_step_deg) * (1 if sign >= 0 else -1)
        try:
            return Ok(self.adapter.jog_joint(idx, step))
        except AppError as e:
            return Err(e)

    def jog_cartesian(self, axis: str, sign: int) -> Result[dict]:
        st = self.poll()
        if not st.ok or st.joints_rad is None:
            return Err(AppError("ARM_CONNECT_FAIL", st.error or "无关节状态"))
        delta = [0.0] * 6
        axis_map = {"x": 0, "y": 1, "z": 2, "rx": 3, "ry": 4, "rz": 5}
        if axis not in axis_map:
            return Err(AppError("JOG_REJECTED", f"未知轴 {axis}"))
        i = axis_map[axis]
        if i < 3:
            delta[i] = self.cfg.arm.jog_cart_step_m * (1 if sign >= 0 else -1)
        else:
            delta[i] = math.radians(self.cfg.arm.jog_cart_step_deg) * (1 if sign >= 0 else -1)
        ok, q_new, msg = self.kin.cart_jog(
            st.joints_rad,
            self.cfg.tcp_xyz,
            frame=self.cfg.arm.cart_frame,
            delta_xyzrpy=delta,
        )
        if not ok:
            return Err(AppError("JOG_REJECTED", "笛卡尔 IK 失败", msg))
        # split into max_delta steps
        max_d = math.radians(self.cfg.arm.max_delta_deg)
        q_cur = list(st.joints_rad)
        diffs = [q_new[j] - q_cur[j] for j in range(6)]
        n_steps = max(1, int(math.ceil(max(abs(d) / max_d for d in diffs))))
        last = {}
        try:
            for s in range(1, n_steps + 1):
                alpha = s / n_steps
                q_cmd = [q_cur[j] + diffs[j] * alpha for j in range(6)]
                last = self.adapter.move_joints_rad(q_cmd)
                if not last.get("ok", True):
                    return Err(AppError("JOG_REJECTED", last.get("error", "点动被拒绝")))
                time.sleep(0.03)
            return Ok(last)
        except AppError as e:
            return Err(e)
