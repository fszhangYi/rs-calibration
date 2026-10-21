from __future__ import annotations

import math
import time
from typing import Optional, Sequence

from rs_calib.config.schema import AppConfig
from rs_calib.domain.math3d import T_flange_tcp, T_from_xyzrpy, inv_T, xyzrpy_from_T
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
            return Err(AppError("JOG_REJECTED", f"笛卡尔 IK 失败: {msg}", msg))
        return self.move_joints_segmented(list(st.joints_rad), q_new)

    def set_max_delta_deg(self, deg: float) -> Result[float]:
        """Persist and push single-step joint |Δq| limit to the live driver."""
        try:
            applied = self.adapter.set_max_delta_deg(deg)
        except AppError as e:
            return Err(e)
        self.cfg.arm.max_delta_deg = float(applied)
        return Ok(float(applied))

    def set_v_norm_rad_s(self, v: float) -> Result[float]:
        """Set joint-space speed (rad/s), same semantics as sensors-arm abs ramp."""
        vv = float(v)
        if not math.isfinite(vv) or not (1e-4 <= vv <= 5.0):
            return Err(AppError("JOG_REJECTED", "v_norm_rad_s 范围应为 0.0001~5 rad/s"))
        self.cfg.arm.v_norm_rad_s = vv
        return Ok(vv)

    def move_joints_segmented(
        self,
        q_from: Sequence[float],
        q_to: Sequence[float],
        *,
        cancel_cb=None,
    ) -> Result[dict]:
        """Move joints in max_delta chunks. cancel_cb() -> bool means abort.

        Pace like sensors-arm: T = clamp(peak_Δq / v_norm, t_min, t_max), then
        sleep T/n_steps between TT chunks.
        """
        # Stay slightly under driver cap to absorb float / last-cmd baseline drift.
        live_cap = self.adapter.get_max_delta_deg()
        cap_deg = float(self.cfg.arm.max_delta_deg)
        if live_cap == live_cap:  # not NaN
            cap_deg = min(cap_deg, float(live_cap))
        max_d = math.radians(max(0.01, cap_deg)) * 0.98
        q_cur = [float(x) for x in list(q_from)[:6]]
        q_new = [float(x) for x in list(q_to)[:6]]
        self.adapter.sync_last_cmd_from_joints(q_cur)
        diffs = [q_new[j] - q_cur[j] for j in range(6)]
        peak = max(abs(d) for d in diffs)
        n_steps = max(1, int(math.ceil(peak / max_d))) if peak > 1e-12 else 1

        v = float(getattr(self.cfg.arm, "v_norm_rad_s", 0.02) or 0.02)
        if not math.isfinite(v) or v <= 0:
            v = 0.02
        t_min = float(getattr(self.cfg.arm, "t_min_s", 0.1) or 0.1)
        t_max = float(getattr(self.cfg.arm, "t_max_s", 30.0) or 30.0)
        t_min = max(0.02, min(30.0, t_min))
        t_max = max(t_min, min(30.0, t_max))
        if peak < 1e-9:
            duration = t_min
        else:
            duration = max(t_min, min(t_max, peak / v))
        sleep_s = duration / float(n_steps)

        last: dict = {}
        try:
            for s in range(1, n_steps + 1):
                if cancel_cb is not None and cancel_cb():
                    return Err(AppError("PATH_CANCELLED", "走位已取消"))
                alpha = s / n_steps
                q_cmd = [q_cur[j] + diffs[j] * alpha for j in range(6)]
                # Guard vs last command, not lagging feedback — required for continuous TT path.
                last = self.adapter.move_joints_rad(q_cmd, relative_to_last=True)
                if not last.get("ok", True):
                    return Err(AppError("JOG_REJECTED", last.get("error", "运动被拒绝")))
                if sleep_s > 1e-4:
                    time.sleep(sleep_s)
            return Ok(last)
        except AppError as e:
            return Err(e)

    def current_xyzrpy(self, frame: str = "flange") -> Result[list[float]]:
        st = self.poll()
        if not st.ok or st.joints_rad is None:
            return Err(AppError("ARM_CONNECT_FAIL", st.error or "无关节状态"))
        T_bf = self.kin.fk_flange(st.joints_rad)
        if frame == "tcp":
            T = T_bf @ T_flange_tcp(self.cfg.tcp_xyz)
        else:
            T = T_bf
        return Ok(xyzrpy_from_T(T))

    def move_to_xyzrpy(
        self,
        xyzrpy: Sequence[float],
        *,
        frame: str = "flange",
        cancel_cb=None,
    ) -> Result[dict]:
        """IK + segmented joint move to a base-frame pose (flange or TCP)."""
        st = self.poll()
        if not st.ok or st.joints_rad is None:
            return Err(AppError("ARM_CONNECT_FAIL", st.error or "无关节状态"))
        if not self.adapter.armed:
            return Err(AppError("ARM_NOT_ARMED", "请先点击 Arm 上使能"))
        T_target = T_from_xyzrpy(xyzrpy)
        if frame == "tcp":
            T_flange = T_target @ inv_T(T_flange_tcp(self.cfg.tcp_xyz))
        else:
            T_flange = T_target
        ok, q_new, msg = self.kin.ik_flange(
            T_flange,
            st.joints_rad,
            enforce_teach_soft_limits=False,
            position_tolerance_m=5e-4,
            orientation_tolerance_rad=1e-2,
        )
        if not ok:
            return Err(AppError("JOG_REJECTED", f"走位 IK 失败: {msg}", msg))
        return self.move_joints_segmented(st.joints_rad, q_new, cancel_cb=cancel_cb)
