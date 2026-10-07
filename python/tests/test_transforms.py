"""Chapter 1 cross-checks of the bindings against Pinocchio's Lie-group functions."""

import numpy as np
import pytest

import manipulator_control as mc

pin = pytest.importorskip("pinocchio")

rng = np.random.default_rng(0)


def random_twist(max_angle=np.pi - 1e-3):
    xi = rng.uniform(-1.0, 1.0, 6)
    xi[3:] *= rng.uniform(0.0, max_angle) / np.linalg.norm(xi[3:])
    return xi


def test_exp_log_so3_match_pinocchio():
    for _ in range(50):
        phi = random_twist()[3:]
        np.testing.assert_allclose(mc.exp_so3(phi), pin.exp3(phi), atol=1e-12)
        R = pin.exp3(phi)
        np.testing.assert_allclose(mc.log_so3(R), pin.log3(R), atol=1e-9)


def test_exp_log_se3_match_pinocchio():
    # Pinocchio's Motion stores (linear, angular), the same order as this module.
    for _ in range(50):
        xi = random_twist()
        T_ref = pin.exp6(pin.Motion(xi[:3], xi[3:])).homogeneous
        np.testing.assert_allclose(mc.exp_se3(xi), T_ref, atol=1e-12)
        np.testing.assert_allclose(mc.log_se3(T_ref), pin.log6(pin.SE3(T_ref)).vector, atol=1e-9)


def test_adjoint_matches_pinocchio_action():
    for _ in range(20):
        T = mc.exp_se3(random_twist())
        np.testing.assert_allclose(mc.adjoint(T), pin.SE3(T).action, atol=1e-12)


def test_rpy_matches_pinocchio():
    for _ in range(20):
        rpy = rng.uniform([-3, -1.5, -3], [3, 1.5, 3])
        R = pin.rpy.rpyToMatrix(rpy)
        np.testing.assert_allclose(mc.rotation_from_rpy(rpy), R, atol=1e-12)
        np.testing.assert_allclose(mc.rpy_from_rotation(R), pin.rpy.matrixToRpy(R), atol=1e-9)


def test_inverse_transform():
    T = mc.exp_se3(random_twist())
    np.testing.assert_allclose(mc.inverse_transform(T) @ T, np.eye(4), atol=1e-14)
