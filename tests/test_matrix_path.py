"""Matrix path generation tests (no hardware)."""

from __future__ import annotations

import math

from rs_calib.domain.matrix_path import MatrixPathParams, generate_matrix_waypoints, summarize_waypoints


def test_xyz_only_when_ori_is_one():
    params = MatrixPathParams(nx=3, ny=3, nz=1, n_rx=1, n_ry=1, n_rz=1).clamp()
    wps = generate_matrix_waypoints([0.4, 0.0, 0.3, 0.0, 1.57, 0.0], params)
    assert len(wps) == 9
    s = summarize_waypoints(wps)
    assert abs(s["x_span_m"] - 0.10) < 1e-9
    assert abs(s["ry_span_deg"]) < 1e-6


def test_full_6dof_product():
    params = MatrixPathParams(
        nx=2, ny=2, nz=1, n_rx=1, n_ry=3, n_rz=1, step_ry_deg=10.0
    ).clamp()
    wps = generate_matrix_waypoints([0, 0, 0, 0, 0, 0], params)
    assert len(wps) == 2 * 2 * 1 * 1 * 3 * 1
    s = summarize_waypoints(wps)
    assert abs(s["ry_span_deg"] - 20.0) < 1e-6


def test_snake_changes_order():
    seed = [0, 0, 0, 0, 0, 0]
    p = MatrixPathParams(
        nx=3, ny=2, nz=1, n_rx=1, n_ry=1, n_rz=1, snake=True, step_x_m=0.1, step_y_m=0.1
    )
    wps = generate_matrix_waypoints(seed, p)
    assert wps[0].iy == 0
    assert wps[1].iy == 1
    assert wps[2].iy == 1
    assert wps[3].iy == 0


def test_orientation_offsets_applied():
    p = MatrixPathParams(nx=1, ny=1, nz=1, n_rx=1, n_ry=3, n_rz=1, step_ry_deg=12).clamp()
    seed = [0.1, 0.2, 0.3, 0.0, 1.0, 0.0]
    wps = generate_matrix_waypoints(seed, p)
    rys = sorted(w.xyzrpy[4] for w in wps)
    assert abs(rys[1] - 1.0) < 1e-9
    assert abs(rys[2] - rys[0] - 2 * math.radians(12)) < 1e-9


if __name__ == "__main__":
    test_xyz_only_when_ori_is_one()
    test_full_6dof_product()
    test_snake_changes_order()
    test_orientation_offsets_applied()
    print("ok")
