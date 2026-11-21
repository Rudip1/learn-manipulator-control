"""Chapter 6: null-space projector and weighted pseudoinverse against NumPy, gradient projection end to end."""

import numpy as np

import manipulator_control as mc

rng = np.random.default_rng(6)


def test_projector_matches_numpy():
    J = rng.normal(size=(3, 7))
    N = mc.null_space_projector(J)
    np.testing.assert_allclose(N, np.eye(7) - np.linalg.pinv(J) @ J, atol=1e-12)
    np.testing.assert_allclose(J @ N, 0, atol=1e-12)


def test_weighted_pseudoinverse_matches_formula():
    J = rng.normal(size=(2, 5))
    A = rng.normal(size=(5, 5))
    W = A @ A.T + np.eye(5)
    Wi = np.linalg.inv(W)
    np.testing.assert_allclose(mc.weighted_pseudoinverse(J, W), Wi @ J.T @ np.linalg.inv(J @ Wi @ J.T), atol=1e-10)


def test_gradient_projection_lowers_joint_centering_cost():
    arm = mc.robots.planar([0.5, 0.4, 0.3])
    q_min, q_max = np.array([-np.pi / 2, -2.0, -2.0]), np.array([np.pi / 2, 2.0, 2.0])
    T_d = np.eye(4)
    T_d[:2, 3] = [0.6, 0.5]
    cfg = mc.ResolvedRateConfig(space=mc.TaskSpace.PlanarPosition, gain=2.0, dt=0.01, steps=1000)
    q0 = np.array([1.2, -1.5, 1.6])
    plain = mc.simulate_redundancy(arm, q0, T_d, cfg, mc.RedundancyConfig())
    centred = mc.simulate_redundancy(
        arm, q0, T_d, cfg, mc.RedundancyConfig(mc.SecondaryObjective.JointCentering, 20.0, q_min, q_max))
    H_plain = mc.joint_centering_cost(plain.trajectory.q[-1], q_min, q_max)
    assert centred.trajectory.error_norm[-1] < 1e-4
    assert centred.objective[-1] < 0.3 * H_plain
