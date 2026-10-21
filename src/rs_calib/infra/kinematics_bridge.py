from __future__ import annotations

import math
from typing import Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as R

from rs_calib.domain.math3d import T_flange_tcp, inv_T
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
        *,
        enforce_teach_soft_limits: bool = False,
        position_tolerance_m: float = 1e-4,
        orientation_tolerance_rad: float = 2e-3,
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
                enforce_teach_soft_limits=enforce_teach_soft_limits,
                position_tolerance_m=position_tolerance_m,
                orientation_tolerance_rad=orientation_tolerance_rad,
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

        Translation is applied in the **base** frame on the current flange/TCP pose
        (no Euler round-trip). Orientation deltas are incremental XYZ-euler in base.
        Soft limits are not enforced for jog (real arms often sit near teach limits).
        """
        q = [float(x) for x in list(joints_rad)[:6]]
        dx = [float(x) for x in list(delta_xyzrpy)[:6]] + [0.0] * 6
        dx = dx[:6]
        T_bf = self.fk_flange(q)
        if frame == "tcp":
            T_cur = T_bf @ T_flange_tcp(tcp_xyz)
        else:
            T_cur = T_bf.copy()

        T_next = T_cur.copy()
        T_next[0, 3] += dx[0]
        T_next[1, 3] += dx[1]
        T_next[2, 3] += dx[2]
        if any(abs(a) > 1e-12 for a in dx[3:6]):
            dR = R.from_euler("xyz", dx[3:6]).as_matrix()
            T_next[:3, :3] = dR @ T_cur[:3, :3]

        if frame == "tcp":
            T_flange_next = T_next @ inv_T(T_flange_tcp(tcp_xyz))
        else:
            T_flange_next = T_next

        # Jog: prefer solvable over soft-limit gate; retry with looser ori if needed.
        ok, q_new, msg = self.ik_flange(
            T_flange_next,
            q,
            enforce_teach_soft_limits=False,
            position_tolerance_m=5e-4,
            orientation_tolerance_rad=1e-2,
        )
        if ok:
            return True, q_new, msg
        ok2, q_new2, msg2 = self.ik_flange(
            T_flange_next,
            q,
            enforce_teach_soft_limits=False,
            position_tolerance_m=1e-3,
            orientation_tolerance_rad=5e-2,
        )
        if ok2:
            return True, q_new2, msg2 or msg
        return False, q, msg2 or msg or "ik failed"
