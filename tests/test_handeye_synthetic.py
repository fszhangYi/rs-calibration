"""Synthetic eye-to-hand / eye-in-hand recovery tests (OpenCV input convention)."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as R

from rs_calib.domain.handeye import solve_handeye
from rs_calib.domain.math3d import inv_T, se3_delta
from rs_calib.domain.types import CalibSample


def _rand_T(rng: np.random.Generator) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = R.from_rotvec(rng.normal(size=3) * 0.5).as_matrix()
    T[:3, 3] = rng.uniform(-0.3, 0.3, size=3)
    return T


def _eth_samples(rng: np.random.Generator, n: int = 20) -> tuple[list[CalibSample], np.ndarray]:
    T_base_cam_gt = _rand_T(rng)
    T_flange_marker = _rand_T(rng)  # unknown fixed; should not matter
    T_cam_base = inv_T(T_base_cam_gt)
    samples: list[CalibSample] = []
    for i in range(n):
        T_base_flange = _rand_T(rng)
        T_cam_marker = T_cam_base @ T_base_flange @ T_flange_marker
        samples.append(
            CalibSample(
                frame_idx=i,
                T_base_flange=T_base_flange,
                T_cam_marker=T_cam_marker,
            )
        )
    return samples, T_base_cam_gt


def test_handeye_recovers_T_base_cam():
    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(42))
    result = solve_handeye(samples, method="TSAI", setup="eye_to_hand")
    assert result.setup == "eye_to_hand"
    dt, dR = se3_delta(result.T_base_cam, T_base_cam_gt)
    assert dt < 1.0, f"translation error {dt} mm"
    assert dR < 0.1, f"rotation error {dR} deg"


def test_handeye_park_recovers_T_base_cam():
    """Park had a swapped outer(a,b) bug; must recover GT on its own."""
    from rs_calib.domain.handeye import (
        _filter_pairs,
        _motion_pairs_axxb,
        _solve_ax_xb_park,
    )

    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(99), n=24)
    A, B = _motion_pairs_axxb(samples, setup="eye_to_hand")
    Af, Bf = _filter_pairs(A, B, min_rot_deg=0.5)
    X = _solve_ax_xb_park(Af, Bf)
    dt, dR = se3_delta(X, T_base_cam_gt)
    assert dt < 1.0, f"Park translation error {dt} mm"
    assert dR < 0.1, f"Park rotation error {dR} deg"


def test_handeye_tsai_recovers_T_base_cam():
    """Tsai must use OpenCV-style P=2q_xyz, not raw rotvec."""
    from rs_calib.domain.handeye import (
        _filter_pairs,
        _motion_pairs_axxb,
        _solve_ax_xb_tsai,
    )

    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(123), n=24)
    A, B = _motion_pairs_axxb(samples, setup="eye_to_hand")
    Af, Bf = _filter_pairs(A, B, min_rot_deg=0.5)
    X = _solve_ax_xb_tsai(Af, Bf)
    dt, dR = se3_delta(X, T_base_cam_gt)
    assert dt < 2.0, f"Tsai translation error {dt} mm"
    assert dR < 0.5, f"Tsai rotation error {dR} deg"


def test_handeye_native_backend_and_dual():
    from rs_calib.domain.handeye import opencv_handeye_available, solve_handeye

    samples, T_base_cam_gt = _eth_samples(np.random.default_rng(3), n=20)
    native = solve_handeye(samples, method="PARK", setup="eye_to_hand", backend="native")
    assert native.metrics.get("backend") == "native"
    dt, dR = se3_delta(native.T_base_cam, T_base_cam_gt)
    assert dt < 1.0 and dR < 0.1

    dual = solve_handeye(samples, method="PARK", setup="eye_to_hand", dual_backends=True)
    assert "dual" in dual.metrics
    if opencv_handeye_available():
        assert dual.metrics["dual"].get("primary") == "opencv"
        assert "OPENCV" in dual.method or "DUAL" in dual.method
        dt2, dR2 = se3_delta(dual.T_base_cam, T_base_cam_gt)
        assert dt2 < 1.0 and dR2 < 0.1
    else:
        assert dual.metrics["dual"].get("primary") == "native"
        assert "DUAL_NO_CV" in dual.method


def test_align_skips_degenerate_identity_ref():
    """A prior OpenCV failure (I) must not 180°-flip a later good extrinsic."""
    from rs_calib.domain.handeye import align_extrinsic_to_ref

    samples, T_gt = _eth_samples(np.random.default_rng(11), n=16)
    result = solve_handeye(samples, method="TSAI", setup="eye_to_hand", backend="native")
    bad_ref = np.eye(4)
    aligned = align_extrinsic_to_ref(result.T_base_cam, bad_ref)
    dt, dR = se3_delta(aligned, result.T_base_cam)
    assert dt < 1e-9 and dR < 1e-6


def test_degenerate_rejects_kilometre_translation():
    from rs_calib.domain.handeye import _is_degenerate_handeye_T

    T = np.eye(4)
    T[:3, 3] = [12724.0, -11174.0, 73.0]  # seen on session_20261026_212137 solve#17
    assert _is_degenerate_handeye_T(T)
    T2 = np.eye(4)
    T2[:3, 3] = [-0.8, 0.3, 0.76]
    assert not _is_degenerate_handeye_T(T2)


def test_dual_arbitrates_large_translation_disagreement():
    """When OpenCV/native Δt is large but ΔR small, must not blindly keep OpenCV."""
    from rs_calib.domain.handeye import solve_handeye_dual

    # Near-planar ETH (tiny Rz/Z) — mirrors the bad 20261026 session conditioning.
    rng = np.random.default_rng(20261026)
    T_base_cam_gt = np.eye(4)
    T_base_cam_gt[:3, :3] = R.from_euler("xyz", [0.2, -0.5, 0.1]).as_matrix()
    T_base_cam_gt[:3, 3] = [-0.8, 0.3, 0.75]
    T_fm = np.eye(4)
    T_fm[:3, 3] = [0.02, -0.02, 0.16]
    T_cam_base = inv_T(T_base_cam_gt)
    samples: list[CalibSample] = []
    for i in range(30):
        # Mostly Yaw/Pitch-ish small; translation in XY only (near planar).
        T_bf = np.eye(4)
        T_bf[:3, :3] = R.from_euler(
            "xyz",
            [rng.uniform(-0.02, 0.02), rng.uniform(-0.2, 0.2), rng.uniform(-0.005, 0.005)],
        ).as_matrix()
        T_bf[:3, 3] = [
            -0.35 + rng.uniform(0, 0.1),
            rng.uniform(-0.05, 0.05),
            0.66 + rng.uniform(0, 0.003),
        ]
        T_cm = T_cam_base @ T_bf @ T_fm
        # Add tiny noise so backends can disagree.
        T_cm = T_cm.copy()
        T_cm[:3, 3] += rng.normal(scale=0.0005, size=3)
        samples.append(CalibSample(frame_idx=i, T_base_flange=T_bf, T_cam_marker=T_cm))

    dual = solve_handeye_dual(samples, method="TSAI", setup="eye_to_hand")
    tnorm = float(np.linalg.norm(dual.T_base_cam[:3, 3]))
    assert tnorm < 3.0, f"implausible dual primary ||t||={tnorm}"
    # Should surface a motion-span or disagreement warning when conditioning is poor.
    dual_meta = dual.metrics.get("dual") or {}
    assert dual_meta.get("enabled") is True


def test_session_20261023_dual_recovers_unflipped():
    """Regression: dual must not lock the 180° branch that broke verify on this session."""
    import json
    from pathlib import Path

    from rs_calib.domain.handeye import solve_handeye
    from rs_calib.domain.math3d import inv_T
    from rs_calib.domain.verify_geom import marker_pred_T_eth
    from scipy.spatial.transform import Rotation as Rot

    sess = Path("sessions/session_20261023_161932")
    if not sess.is_dir():
        return  # optional local session fixture
    lines = (sess / "live_solves.jsonl").read_text(encoding="utf-8").strip().splitlines()
    last = json.loads(lines[-1])
    samples = [
        CalibSample(
            frame_idx=int(s["frame_idx"]),
            T_base_flange=np.asarray(s["T_base_flange"], dtype=float),
            T_cam_marker=np.asarray(s["T_cam_marker"], dtype=float),
        )
        for s in last["basis"]["samples"]
    ]
    saved = np.asarray(last["result"]["T_base_cam"], dtype=float)
    dual = solve_handeye(samples, method="TSAI", setup="eye_to_hand", dual_backends=True)
    # Must disagree with the historically flipped saved pose.
    _, dR_saved = se3_delta(dual.T_base_cam, saved)
    assert dR_saved > 90.0, f"expected escape from flipped branch, got ΔR={dR_saved}"

    # Absolute board residual with estimated T_fm should be ~cm, not meters.
    est = np.stack(
        [inv_T(s.T_base_flange) @ dual.T_base_cam @ s.T_cam_marker for s in samples]
    )
    T_fm = np.eye(4)
    T_fm[:3, :3] = Rot.from_matrix(est[:, :3, :3]).mean().as_matrix()
    T_fm[:3, 3] = np.median(est[:, :3, 3], axis=0)
    errs = [
        se3_delta(
            marker_pred_T_eth(dual.T_cam_base, s.T_base_flange, T_fm),
            s.T_cam_marker,
        )[0]
        for s in samples
    ]
    assert float(np.median(errs)) < 30.0, f"verify median {np.median(errs)} mm"


def test_handeye_recovers_T_flange_cam():
    rng = np.random.default_rng(7)
    T_flange_cam_gt = _rand_T(rng)
    T_base_marker = _rand_T(rng)  # board fixed in base
    T_cam_flange = inv_T(T_flange_cam_gt)

    samples: list[CalibSample] = []
    for i in range(24):
        T_base_flange = _rand_T(rng)
        # cam_marker = cam_flange · inv(base_flange) · base_marker
        T_cam_marker = T_cam_flange @ inv_T(T_base_flange) @ T_base_marker
        samples.append(
            CalibSample(
                frame_idx=i,
                T_base_flange=T_base_flange,
                T_cam_marker=T_cam_marker,
            )
        )

    result = solve_handeye(samples, method="TSAI", setup="eye_in_hand")
    assert result.setup == "eye_in_hand"
    assert result.T_flange_cam is not None
    dt, dR = se3_delta(result.T_flange_cam, T_flange_cam_gt)
    assert dt < 1.0, f"EIH translation error {dt} mm"
    assert dR < 0.1, f"EIH rotation error {dR} deg"
    dt2, dR2 = se3_delta(result.primary_T(), T_flange_cam_gt)
    assert dt2 < 1.0 and dR2 < 0.1


if __name__ == "__main__":
    test_handeye_recovers_T_base_cam()
    test_handeye_recovers_T_flange_cam()
    print("ok")
