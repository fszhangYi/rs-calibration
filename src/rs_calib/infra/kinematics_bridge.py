from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np

from rs_calib.domain.math3d import T_flange_tcp, T_from_xyzrpy, inv_T, xyzrpy_from_T
from rs_calib.errors import AppError


class KinematicsBridge:
    """EC616 FK/IK via sensors.kinematics (flange frame)."""

    def fk_flange(self, joints_rad: Sequence[float]) -> np.ndarray:
        try:
            from sensors.kinematics import fk_flange_from_joints_rad
        except ImportError as e:
            raise AppError("ARM_CONNECT_FAIL", "无法导入 sensors.kinematics", str(e)) from e
        return np.asarray(fk_flange_from_joints_rad(list(joints_rad)[:6]), dtype=np.float64)

    def fk_tcp(self, joints_rad: Sequence[float], tcp_xyz: Sequence[float]) -> np.ndarray:
        return self.fk_flange(joints_rad) @ T_flange_tcp(tcp_xyz)

    def ik_flange(
        self,
        T_base_flange: np.ndarray,
        seed_rad: Sequence[float],
    ) -> tuple[bool, list[float], str]:
        try:
            from sensors.kinematics import ik_flange
        except ImportError as e:
            return False, list(seed_rad), f"import ik failed: {e}"
        seed_deg = np.rad2deg(np.asarray(list(seed_rad)[:6], dtype=float))
        try:
            result = ik_flange(
                np.asarray(T_base_flange, dtype=float),
                seed_deg,
                return_details=True,
                enforce_teach_soft_limits=True,
            )
        except Exception as e:  # noqa: BLE001
            return False, list(seed_rad), str(e)
        ok = bool(getattr(result, "success", False))
        joint_deg = getattr(result, "joint_deg", None)
        msg = str(getattr(result, "message", ""))
        if not ok or joint_deg is None:
            return False, list(seed_rad), msg or "ik failed"
        joints_rad = [math.radians(float(x)) for x in np.asarray(joint_deg).reshape(-1)[:6]]
        return True, joints_rad, msg

    def cart_jog(
        self,
        joints_rad: Sequence[float],
        tcp_xyz: Sequence[float],
        *,
        frame: str,
        delta_xyzrpy: Sequence[float],
    ) -> tuple[bool, list[float], str]:
        """One cartesian jog step → target joints (rad).

        Base-frame translation + Euler add on current pose of flange or TCP.
        """
        q = [float(x) for x in list(joints_rad)[:6]]
        dx = [float(x) for x in list(delta_xyzrpy)[:6]]
        T_bf = self.fk_flange(q)
        if frame == "tcp":
            T_cur = T_bf @ T_flange_tcp(tcp_xyz)
            xyzrpy = xyzrpy_from_T(T_cur)
            nxt = [xyzrpy[i] + dx[i] for i in range(6)]
            T_tcp_next = T_from_xyzrpy(nxt)
            T_flange_next = T_tcp_next @ inv_T(T_flange_tcp(tcp_xyz))
        else:
            xyzrpy = xyzrpy_from_T(T_bf)
            nxt = [xyzrpy[i] + dx[i] for i in range(6)]
            T_flange_next = T_from_xyzrpy(nxt)
        return self.ik_flange(T_flange_next, q)
