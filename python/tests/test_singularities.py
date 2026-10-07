"""Chapter 5: manipulability and damping against NumPy."""

import numpy as np

import manipulator_control as mc

rng = np.random.default_rng(5)


def test_singular_values_and_manipulability_match_numpy():
    for shape in [(2, 3), (3, 6), (6, 6)]:
        J = rng.normal(size=shape)
        np.testing.assert_allclose(mc.singular_values(J), np.linalg.svd(J, compute_uv=False), atol=1e-12)
        np.testing.assert_allclose(mc.manipulability(J), np.sqrt(np.linalg.det(J @ J.T)), rtol=1e-10)


def test_min_singular_value_schedule_formula():
    J = np.diag([1.0, 0.03])
    lam = mc.damping_factor(J, mc.DampingSchedule.MinSingularValue, 0.2, 0.1)
    np.testing.assert_allclose(lam, 0.2 * np.sqrt(1 - 0.3**2), rtol=1e-12)
