import numpy as np

from rs_calib.domain.math3d import T_from_xyzrpy, inv_T, se3_delta, xyzrpy_from_T


def test_xyzrpy_roundtrip():
    xyzrpy = [0.1, -0.2, 0.3, 0.4, -0.5, 0.6]
    T = T_from_xyzrpy(xyzrpy)
    back = xyzrpy_from_T(T)
    assert np.allclose(xyzrpy[:3], back[:3], atol=1e-9)
    assert np.allclose(xyzrpy[3:], back[3:], atol=1e-9)


def test_inv_T():
    T = T_from_xyzrpy([0.2, 0.0, 0.5, 0.1, 0.2, 0.3])
    I = inv_T(T) @ T
    assert np.allclose(I, np.eye(4), atol=1e-9)


def test_se3_delta_identity():
    T = np.eye(4)
    dt, dR = se3_delta(T, T)
    assert dt == 0.0 and dR == 0.0
