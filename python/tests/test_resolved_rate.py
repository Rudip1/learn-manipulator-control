"""Chapter 4: generalised inverses against NumPy, and the control step against its formula."""

import numpy as np

import manipulator_control as mc

rng = np.random.default_rng(4)


def test_pseudoinverse_matches_numpy():
    for shape in [(2, 3), (3, 6), (6, 6), (6, 4)]:
        A = rng.normal(size=shape)
        np.testing.assert_allclose(mc.pseudoinverse(A), np.linalg.pinv(A), atol=1e-12)
    A = rng.normal(size=(4, 6))
    A[3] = A[0] - A[1]  # rank deficient
    np.testing.assert_allclose(mc.pseudoinverse(A), np.linalg.pinv(A, rcond=1e-10), atol=1e-10)


def test_dls_inverse_matches_formula():
    J = rng.normal(size=(3, 5))
    lam = 0.2
    np.testing.assert_allclose(mc.dls_inverse(J, lam), J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3)),
                               atol=1e-12)


def test_resolved_rate_step_is_the_control_law():
    arm = mc.robots.puma560()
    q = rng.uniform(-1, 1, 6)
    T_d = arm.end_effector(q + 0.1)
    ff = rng.normal(size=6)
    cfg = mc.ResolvedRateConfig(space=mc.TaskSpace.Pose, gain=1.7)
    e = mc.task_error(mc.TaskSpace.Pose, T_d, arm.end_effector(q))
    expected = np.linalg.pinv(mc.geometric_jacobian(arm, q)) @ (1.7 * e + ff)
    np.testing.assert_allclose(mc.resolved_rate_step(arm, q, T_d, ff, cfg), expected, atol=1e-10)
