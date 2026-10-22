from __future__ import annotations

from typing import Callable, Literal, Optional, Sequence

import numpy as np
from scipy.spatial.transform import Rotation as Rsci

from rs_calib.domain.math3d import inv_T, se3_delta
from rs_calib.domain.types import CalibSample, ExtrinsicsResult

SetupName = Literal["eye_to_hand", "eye_in_hand"]
BackendName = Literal["auto", "opencv", "native"]

_METHOD_CONSTANTS = {
    "TSAI": "CALIB_HAND_EYE_TSAI",
    "PARK": "CALIB_HAND_EYE_PARK",
    "HORAUD": "CALIB_HAND_EYE_HORAUD",
    "ANDREFF": "CALIB_HAND_EYE_ANDREFF",
    "DANIILIDIS": "CALIB_HAND_EYE_DANIILIDIS",
}

_SETUPS = ("eye_to_hand", "eye_in_hand")


def available_methods() -> list[str]:
    return list(_METHOD_CONSTANTS.keys())


def available_setups() -> list[str]:
    return list(_SETUPS)


def opencv_handeye_available() -> bool:
    """True when ``cv2.calibrateHandEye`` is importable (typical OpenCV 4.x)."""
    return _resolve_calibrate_handeye() is not None


def opencv_handeye_status() -> dict[str, object]:
    """Runtime probe for UI tooltips / dual-backend gating."""
    try:
        import cv2

        ver = str(getattr(cv2, "__version__", "?"))
    except Exception as e:  # noqa: BLE001
        return {"available": False, "cv_version": None, "detail": f"import cv2 failed: {e}"}
    ok = opencv_handeye_available()
    return {
        "available": ok,
        "cv_version": ver,
        "detail": (
            "cv2.calibrateHandEye OK (OpenCV 4.x style)"
            if ok
            else "cv2.calibrateHandEye missing (common on OpenCV 5.0 wheels; use native or install opencv-python<5)"
        ),
    }


def normalize_setup(setup: str) -> SetupName:
    key = (setup or "eye_to_hand").strip().lower().replace("-", "_")
    aliases = {
        "eth": "eye_to_hand",
        "eih": "eye_in_hand",
        "eye_to_hand": "eye_to_hand",
        "eye_in_hand": "eye_in_hand",
        "eyetohand": "eye_to_hand",
        "eyeinhand": "eye_in_hand",
    }
    out = aliases.get(key, key)
    if out not in _SETUPS:
        raise ValueError(f"unknown handeye setup: {setup}")
    return out  # type: ignore[return-value]


def _pack_T(Rm: np.ndarray, t: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rm
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


def _orthonormalize_R(Rm: np.ndarray) -> np.ndarray:
    U, _, Vt = np.linalg.svd(np.asarray(Rm, dtype=np.float64))
    Rout = U @ Vt
    if np.linalg.det(Rout) < 0:
        U = U.copy()
        U[:, -1] *= -1
        Rout = U @ Vt
    return Rout


def _rotvec(Rm: np.ndarray) -> np.ndarray:
    return Rsci.from_matrix(Rm).as_rotvec()


def _tsai_p(Rm: np.ndarray) -> np.ndarray:
    """OpenCV/Tsai modified Rodrigues vector ``P = 2 q_xyz`` (``q_w >= 0``)."""
    q = Rsci.from_matrix(Rm).as_quat()  # x, y, z, w
    if q[3] < 0.0:
        q = -q
    return 2.0 * np.asarray(q[:3], dtype=np.float64)


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=np.float64).reshape(3)
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=np.float64)


def _motion_pairs_axxb(
    samples: Sequence[CalibSample],
    *,
    setup: SetupName,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Build relative motion pairs (A_i, B_i) for AX=XB.

    Uses **all unordered pose pairs** ``(i, j)`` with ``j > i``, matching OpenCV
    ``calibrateHandEye`` (consecutive-only pairs under-constrain Tsai/Andreff
    on slow/small ETH trajectories).

    eye_to_hand (X=T_base_cam), equivalent to OpenCV ETH with ``G'=inv(G)``:
      A = G_j inv(G_i),  B = C_j inv(C_i)
    eye_in_hand (X=T_flange_cam):
      A = inv(G_i) G_j,  B = C_i inv(C_j)
    """
    G = [s.T_base_flange.astype(np.float64) for s in samples]
    C = [s.T_cam_marker.astype(np.float64) for s in samples]
    n = len(G)
    # Cap poses for O(n²) pairs so live recompute stays interactive.
    max_poses = 64
    if n > max_poses:
        pick = np.linspace(0, n - 1, max_poses).astype(int)
        # unique while preserving order
        seen: set[int] = set()
        idxs: list[int] = []
        for k in pick.tolist():
            if k not in seen:
                seen.add(k)
                idxs.append(k)
        G = [G[k] for k in idxs]
        C = [C[k] for k in idxs]
        n = len(G)
    A: list[np.ndarray] = []
    B: list[np.ndarray] = []
    if setup == "eye_to_hand":
        for i in range(n):
            for j in range(i + 1, n):
                A.append(G[j] @ inv_T(G[i]))
                B.append(C[j] @ inv_T(C[i]))
        return A, B
    for i in range(n):
        for j in range(i + 1, n):
            A.append(inv_T(G[i]) @ G[j])
            B.append(C[i] @ inv_T(C[j]))
    return A, B


def _filter_pairs(
    A: list[np.ndarray],
    B: list[np.ndarray],
    *,
    min_rot_deg: float = 5.0,
) -> tuple[list[np.ndarray], list[np.ndarray]]:
    """Drop near-static relative motions that make HandEye ill-conditioned.

    Default 5° (was 1.5–2°): OpenCV Tsai also rejects tiny rotations; keeping
    them biases native Tsai/Andreff on dense live streams.
    """
    Ao, Bo = [], []
    for Ta, Tb in zip(A, B):
        ang_a = float(np.linalg.norm(_rotvec(Ta[:3, :3])) * 180.0 / np.pi)
        ang_b = float(np.linalg.norm(_rotvec(Tb[:3, :3])) * 180.0 / np.pi)
        if max(ang_a, ang_b) >= min_rot_deg:
            Ao.append(Ta)
            Bo.append(Tb)
    return Ao, Bo


def _solve_t_given_R(
    A: list[np.ndarray],
    B: list[np.ndarray],
    Rx: np.ndarray,
) -> np.ndarray:
    At, bt = [], []
    for Ta, Tb in zip(A, B):
        Ra, ta = Ta[:3, :3], Ta[:3, 3]
        tb = Tb[:3, 3]
        At.append(np.eye(3) - Ra)
        bt.append(ta - Rx @ tb)
    tx, *_ = np.linalg.lstsq(np.vstack(At), np.concatenate(bt), rcond=None)
    return tx.reshape(3)


def _solve_ax_xb_andreff(A: list[np.ndarray], B: list[np.ndarray]) -> np.ndarray:
    """Andreff-style Kronecker SVD (column-major vec)."""
    mats = []
    for Ta, Tb in zip(A, B):
        Ra, Rb = Ta[:3, :3], Tb[:3, :3]
        mats.append(np.kron(np.eye(3), Ra) - np.kron(Rb.T, np.eye(3)))
    M = np.vstack(mats)
    _, _, Vt = np.linalg.svd(M)
    rx = Vt[-1].reshape(3, 3, order="F")
    Rx = _orthonormalize_R(rx)
    return _pack_T(Rx, _solve_t_given_R(A, B, Rx))


def _solve_ax_xb_tsai(A: list[np.ndarray], B: list[np.ndarray]) -> np.ndarray:
    """Tsai–Lenz rotation then LS translation (OpenCV-compatible ``P`` vectors)."""
    # OpenCV keeps motions with 0.3 < |P| < 1.7 (roughly ~17° … ~120°).
    kept_a: list[np.ndarray] = []
    kept_b: list[np.ndarray] = []
    rows: list[np.ndarray] = []
    rhs: list[np.ndarray] = []
    for Ta, Tb in zip(A, B):
        pa = _tsai_p(Ta[:3, :3])
        pb = _tsai_p(Tb[:3, :3])
        na = float(np.linalg.norm(pa))
        nb = float(np.linalg.norm(pb))
        if na < 0.3 or nb < 0.3 or na > 1.7 or nb > 1.7:
            continue
        kept_a.append(Ta)
        kept_b.append(Tb)
        rows.append(_skew(pa + pb))
        rhs.append(pb - pa)
    if len(kept_a) < 2:
        # Fall back to unfiltered pairs if trajectory is gentle.
        kept_a, kept_b = list(A), list(B)
        rows, rhs = [], []
        for Ta, Tb in zip(A, B):
            pa = _tsai_p(Ta[:3, :3])
            pb = _tsai_p(Tb[:3, :3])
            rows.append(_skew(pa + pb))
            rhs.append(pb - pa)
    if len(rows) < 2:
        raise ValueError("Tsai needs >=2 relative motions")
    Cmat = np.vstack(rows)
    dvec = np.concatenate(rhs)
    nx1, *_ = np.linalg.lstsq(Cmat, dvec, rcond=None)
    alpha = float(np.linalg.norm(nx1))
    if alpha < 1e-12:
        Rx = np.eye(3)
    else:
        # OpenCV: Pcg = 2*Pcg_ / sqrt(1+|Pcg_|^2), R from quat with xyz=Pcg/2.
        pcg = nx1.reshape(3) * (2.0 / np.sqrt(1.0 + alpha * alpha))
        qxyz = pcg * 0.5
        qw2 = 1.0 - float(np.dot(qxyz, qxyz))
        qw = float(np.sqrt(max(0.0, qw2)))
        Rx = Rsci.from_quat([qxyz[0], qxyz[1], qxyz[2], qw]).as_matrix()
    Rx = _orthonormalize_R(Rx)
    return _pack_T(Rx, _solve_t_given_R(kept_a, kept_b, Rx))


def _solve_ax_xb_park(A: list[np.ndarray], B: list[np.ndarray]) -> np.ndarray:
    """Park–Martin axis-alignment LS for R, then LS for t.

    From ``Ra Rx = Rx Rb`` the rotation axes satisfy ``Rx @ b = a``.
    Build ``M = Σ a bᵀ`` and recover ``Rx`` by SVD (Procrustes).
    """
    M = np.zeros((3, 3), dtype=np.float64)
    for Ta, Tb in zip(A, B):
        a = _rotvec(Ta[:3, :3])
        b = _rotvec(Tb[:3, :3])
        na = np.linalg.norm(a)
        nb = np.linalg.norm(b)
        if na < 1e-9 or nb < 1e-9:
            continue
        a = a / na
        b = b / nb
        # Must be outer(a, b): Rx maps b→a. outer(b, a) recovers Rxᵀ (bug).
        M += np.outer(a, b)
    U, _, Vt = np.linalg.svd(M)
    Rx = _orthonormalize_R(U @ Vt)
    return _pack_T(Rx, _solve_t_given_R(A, B, Rx))


def _axxb_rotation_residual(A: list[np.ndarray], B: list[np.ndarray], Rx: np.ndarray) -> float:
    err = 0.0
    for Ta, Tb in zip(A, B):
        E = Ta[:3, :3] @ Rx - Rx @ Tb[:3, :3]
        err += float(np.linalg.norm(E, ord="fro") ** 2)
    return err / max(1, len(A))


def _axxb_pose_residual(A: list[np.ndarray], B: list[np.ndarray], X: np.ndarray) -> float:
    err = 0.0
    for Ta, Tb in zip(A, B):
        E = Ta @ X - X @ Tb
        err += float(np.linalg.norm(E, ord="fro") ** 2)
    return err / max(1, len(A))


def _resolve_calibrate_handeye() -> Optional[Callable]:
    """OpenCV 4.x exposes cv2.calibrateHandEye; some OpenCV 5 wheels omit it."""
    try:
        import cv2
    except Exception:  # noqa: BLE001
        return None
    fn = getattr(cv2, "calibrateHandEye", None)
    if callable(fn):
        return fn
    for mod_name in ("cv2.calib3d", "cv2.cv2"):
        try:
            import importlib

            mod = importlib.import_module(mod_name)
            fn = getattr(mod, "calibrateHandEye", None)
            if callable(fn):
                return fn
        except Exception:  # noqa: BLE001
            continue
    return None


def _opencv_calibrate(
    samples: Sequence[CalibSample],
    *,
    setup: SetupName,
    method_key: str,
) -> np.ndarray | None:
    fn = _resolve_calibrate_handeye()
    if fn is None:
        return None
    try:
        import cv2
    except Exception:  # noqa: BLE001
        return None

    R_gripper2base: list[np.ndarray] = []
    t_gripper2base: list[np.ndarray] = []
    R_target2cam: list[np.ndarray] = []
    t_target2cam: list[np.ndarray] = []
    for s in samples:
        if setup == "eye_to_hand":
            # OpenCV ETH: feed inv(T_base_flange) as gripper2base → returns cam2base
            T_g2b = inv_T(s.T_base_flange)
        else:
            # OpenCV EIH: feed T_base_flange as gripper2base → returns cam2gripper (=T_flange_cam)
            T_g2b = s.T_base_flange
        R_gripper2base.append(T_g2b[:3, :3].astype(np.float64))
        t_gripper2base.append(T_g2b[:3, 3].reshape(3, 1).astype(np.float64))
        R_target2cam.append(s.T_cam_marker[:3, :3].astype(np.float64))
        t_target2cam.append(s.T_cam_marker[:3, 3].reshape(3, 1).astype(np.float64))

    flag = getattr(cv2, _METHOD_CONSTANTS[method_key], None)
    if flag is None:
        return None
    try:
        R_out, t_out = fn(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=flag,
        )
    except Exception:  # noqa: BLE001
        return None
    return _pack_T(_orthonormalize_R(R_out), t_out)


def _native_calibrate(
    samples: Sequence[CalibSample],
    *,
    setup: SetupName,
    method_key: str,
) -> tuple[np.ndarray, str, float]:
    """Return ``(X, winner_name, axxb_residual)`` using lowest AX=XB residual."""
    A, B = _motion_pairs_axxb(samples, setup=setup)
    Af, Bf = _filter_pairs(A, B, min_rot_deg=5.0)
    if len(Af) < 2:
        # fall back if filtering removed too much (early live / tiny motion)
        Af, Bf = _filter_pairs(A, B, min_rot_deg=1.5)
    if len(Af) < 2:
        Af, Bf = A, B
    if len(Af) < 1:
        raise ValueError("no relative motions for HandEye")

    solvers = {
        "TSAI": _solve_ax_xb_tsai,
        "PARK": _solve_ax_xb_park,
        "ANDREFF": _solve_ax_xb_andreff,
        "HORAUD": _solve_ax_xb_park,  # close cousin; use Park as stand-in
        "DANIILIDIS": _solve_ax_xb_andreff,  # dual-quat deferred; Andreff is stronger than old SVD alone
    }
    primary = solvers.get(method_key, _solve_ax_xb_park)

    # Deduplicate by solver identity. Andreff/Daniilidis are only used when
    # explicitly requested — on noisy ETH data they can look OK by AX=XB
    # residual yet be far off in absolute pose.
    tried: dict[Callable, str] = {primary: method_key}
    for name, fn_solve in (
        ("PARK", _solve_ax_xb_park),
        ("TSAI", _solve_ax_xb_tsai),
    ):
        tried.setdefault(fn_solve, name)
    if method_key in ("ANDREFF", "DANIILIDIS"):
        tried.setdefault(_solve_ax_xb_andreff, method_key)

    candidates: list[tuple[float, str, np.ndarray]] = []
    for fn_solve, name in tried.items():
        try:
            X = fn_solve(Af, Bf)
            res = _axxb_pose_residual(Af, Bf, X)
            candidates.append((res, name, X))
        except Exception:  # noqa: BLE001
            continue
    if not candidates:
        raise RuntimeError("all native HandEye solvers failed")
    candidates.sort(key=lambda x: x[0])
    best_res, best_name, best_X = candidates[0]
    return best_X, best_name, float(best_res)


def align_extrinsic_to_ref(T_new: np.ndarray, T_ref: Optional[np.ndarray]) -> np.ndarray:
    """If HandEye flipped to a far SO(3) branch, keep the representative closer to ``T_ref``.

    Tries identity and 180° flips about object axes (common spurious HandEye modes).
    """
    if T_ref is None:
        return T_new
    T_new = np.asarray(T_new, dtype=np.float64)
    T_ref = np.asarray(T_ref, dtype=np.float64)
    best = T_new
    best_dr = se3_delta(T_ref, T_new)[1]
    if best_dr < 90.0:
        return T_new
    R0 = T_new[:3, :3]
    t0 = T_new[:3, 3]
    for axis in range(3):
        # 180° about axis in the flange/cam frame (right-multiply) and base (left-multiply)
        aa = np.zeros(3)
        aa[axis] = np.pi
        R180 = Rsci.from_rotvec(aa).as_matrix()
        for Rm in (R0 @ R180, R180 @ R0):
            Rm = _orthonormalize_R(Rm)
            T = _pack_T(Rm, t0)
            dr = se3_delta(T_ref, T)[1]
            if dr < best_dr:
                best_dr = dr
                best = T
    return best


def _pack_result(
    T_solved: np.ndarray,
    *,
    setup_n: SetupName,
    method_label: str,
    n_samples: int,
    indices: list[int],
    metrics: dict,
) -> ExtrinsicsResult:
    if setup_n == "eye_to_hand":
        T_base_cam = T_solved
        return ExtrinsicsResult(
            T_base_cam=T_base_cam,
            T_cam_base=inv_T(T_base_cam),
            method=method_label,
            n_samples=n_samples,
            used_indices=indices,
            metrics=metrics,
            setup=setup_n,
            T_flange_cam=None,
            T_cam_flange=None,
        )
    T_flange_cam = T_solved
    eye = np.eye(4, dtype=np.float64)
    return ExtrinsicsResult(
        T_base_cam=eye.copy(),
        T_cam_base=eye.copy(),
        method=method_label,
        n_samples=n_samples,
        used_indices=indices,
        metrics={"note": "eye_in_hand: primary is T_flange_cam", **metrics},
        setup=setup_n,
        T_flange_cam=T_flange_cam,
        T_cam_flange=inv_T(T_flange_cam),
    )


def _solve_once(
    samples: Sequence[CalibSample],
    *,
    method_key: str,
    setup_n: SetupName,
    backend: BackendName,
    align_to: Optional[np.ndarray],
) -> ExtrinsicsResult:
    """Solve with a single backend (``opencv`` / ``native`` / ``auto``)."""
    indices = [int(s.frame_idx) for s in samples]
    metrics: dict = {"backend_request": backend}
    T_solved: Optional[np.ndarray] = None
    used = method_key

    if backend in ("auto", "opencv"):
        T_cv = _opencv_calibrate(samples, setup=setup_n, method_key=method_key)
        if T_cv is not None:
            T_solved = T_cv
            used = f"{method_key}_OPENCV"
            metrics["backend"] = "opencv"
            metrics["cv_version"] = opencv_handeye_status().get("cv_version")
        elif backend == "opencv":
            st = opencv_handeye_status()
            raise RuntimeError(
                "OpenCV HandEye backend unavailable: "
                f"{st.get('detail')} (cv={st.get('cv_version')})"
            )

    if T_solved is None:
        T_solved, winner, axxb_res = _native_calibrate(
            samples, setup=setup_n, method_key=method_key
        )
        used = f"{method_key}_NATIVE"
        if not opencv_handeye_available():
            used = f"{method_key}_NATIVE_NO_CV_HANDEYE"
        if winner != method_key:
            used = f"{used}({winner})"
        metrics.update(
            {
                "backend": "native",
                "native_winner": winner,
                "axxb_residual": axxb_res,
            }
        )

    assert T_solved is not None
    if align_to is not None:
        T_solved = align_extrinsic_to_ref(T_solved, align_to)
    return _pack_result(
        T_solved,
        setup_n=setup_n,
        method_label=used,
        n_samples=len(samples),
        indices=indices,
        metrics=metrics,
    )


def solve_handeye_dual(
    samples: Sequence[CalibSample],
    method: str = "TSAI",
    *,
    setup: str = "eye_to_hand",
    align_to: Optional[np.ndarray] = None,
) -> ExtrinsicsResult:
    """Run **OpenCV calibrateHandEye** and **native** solvers, keep both.

    Primary result prefers OpenCV when available (official Tsai/Park/…).
    Native is always attached under ``metrics["dual"]`` with Δt/ΔR vs primary.
    If OpenCV HandEye is missing, falls back to native-only and records a warning.
    """
    if len(samples) < 3:
        raise ValueError(f"need at least 3 samples, got {len(samples)}")
    key = method.upper()
    if key not in _METHOD_CONSTANTS:
        raise ValueError(f"unknown handeye method: {method}")
    setup_n = normalize_setup(setup)

    native = _solve_once(
        samples,
        method_key=key,
        setup_n=setup_n,
        backend="native",
        align_to=align_to,
    )

    opencv_res: Optional[ExtrinsicsResult] = None
    opencv_err: Optional[str] = None
    if opencv_handeye_available():
        try:
            opencv_res = _solve_once(
                samples,
                method_key=key,
                setup_n=setup_n,
                backend="opencv",
                align_to=align_to,
            )
        except Exception as e:  # noqa: BLE001
            opencv_err = str(e)
    else:
        st = opencv_handeye_status()
        opencv_err = str(st.get("detail") or "calibrateHandEye unavailable")

    if opencv_res is None:
        dual = {
            "enabled": True,
            "primary": "native",
            "opencv": None,
            "native_method": native.method,
            "opencv_error": opencv_err,
            "warning": "双算法：OpenCV HandEye 不可用，仅保留原生解",
        }
        native.metrics = {**native.metrics, "dual": dual}
        native.method = f"DUAL_NO_CV→{native.method}"
        return native

    dt, dR = se3_delta(opencv_res.primary_T(), native.primary_T())
    dual = {
        "enabled": True,
        "primary": "opencv",
        "opencv_method": opencv_res.method,
        "native_method": native.method,
        "delta_t_mm": float(dt),
        "delta_R_deg": float(dR),
        "opencv_T": np.asarray(opencv_res.primary_T(), dtype=float).tolist(),
        "native_T": np.asarray(native.primary_T(), dtype=float).tolist(),
        "native_metrics": dict(native.metrics),
        "opencv_metrics": dict(opencv_res.metrics),
    }
    # Prefer OpenCV as the live/offline accepted extrinsic; native kept for compare.
    primary = opencv_res
    primary.method = (
        f"DUAL(OPENCV={opencv_res.method}|NATIVE={native.method}"
        f"|Δt={dt:.2f}mm|ΔR={dR:.2f}°)"
    )
    primary.metrics = {**primary.metrics, "dual": dual}
    return primary


def solve_handeye(
    samples: Sequence[CalibSample],
    method: str = "TSAI",
    *,
    setup: str = "eye_to_hand",
    align_to: Optional[np.ndarray] = None,
    backend: BackendName = "auto",
    dual_backends: bool = False,
) -> ExtrinsicsResult:
    """Solve hand-eye extrinsic for the selected setup.

    - ``eye_to_hand`` → ``T_base_cam`` (scene camera fixed in base)
    - ``eye_in_hand`` → ``T_flange_cam`` (wrist camera fixed on flange)

    Backends:
    - ``auto``: OpenCV ``calibrateHandEye`` if present (OpenCV 4.x), else native
    - ``opencv``: require ``cv2.calibrateHandEye``
    - ``native``: Tsai/Park/(Andreff) AX=XB implementation in this repo

    When ``dual_backends=True``, both OpenCV and native are computed (see
    ``solve_handeye_dual``); primary prefers OpenCV.
    """
    if dual_backends:
        return solve_handeye_dual(
            samples, method=method, setup=setup, align_to=align_to
        )
    if len(samples) < 3:
        raise ValueError(f"need at least 3 samples, got {len(samples)}")
    key = method.upper()
    if key not in _METHOD_CONSTANTS:
        raise ValueError(f"unknown handeye method: {method}")
    setup_n = normalize_setup(setup)
    be = (backend or "auto").strip().lower()  # type: ignore[assignment]
    if be not in ("auto", "opencv", "native"):
        raise ValueError(f"unknown handeye backend: {backend}")
    return _solve_once(
        samples,
        method_key=key,
        setup_n=setup_n,
        backend=be,  # type: ignore[arg-type]
        align_to=align_to,
    )
