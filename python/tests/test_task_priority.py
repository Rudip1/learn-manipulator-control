"""Chapter 7: task-priority solver against NumPy, and the task objects through the Python interface."""

import numpy as np

import manipulator_control as mc

rng = np.random.default_rng(7)


def test_compatible_levels_equal_stacked_pseudoinverse():
    J1, J2 = rng.normal(size=(2, 6)), rng.normal(size=(3, 6))
    v1, v2 = rng.normal(size=2), rng.normal(size=3)
    u = mc.task_priority_velocity([mc.PriorityLevel(J1, v1), mc.PriorityLevel(J2, v2)], 6)
    np.testing.assert_allclose(u, np.linalg.pinv(np.vstack([J1, J2])) @ np.concatenate([v1, v2]), atol=1e-10)


def test_two_task_formula():
    # Eq. (7.3) written out with NumPy.
    J1, J2 = rng.normal(size=(2, 4)), rng.normal(size=(3, 4))
    v1, v2 = rng.normal(size=2), rng.normal(size=3)
    J1p = np.linalg.pinv(J1)
    N1 = np.eye(4) - J1p @ J1
    ref = J1p @ v1 + np.linalg.pinv(J2 @ N1) @ (v2 - J2 @ J1p @ v1)
    u = mc.task_priority_velocity([mc.PriorityLevel(J1, v1), mc.PriorityLevel(J2, v2)], 4)
    np.testing.assert_allclose(u, ref, atol=1e-10)


def test_tasks_and_simulation():
    arm = mc.robots.planar([0.4, 0.35, 0.3, 0.25])
    model = mc.SerialArmModel(arm)
    target = np.eye(4)
    target[:2, 3] = [0.6, 0.5]
    tip = mc.FrameTask("tip", -1, [0, 1], target)
    joint = mc.JointTask("q1", 0, 0.0)
    tip.set_gain(2.0)
    joint.set_gain(2.0)
    r = mc.simulate_task_priority(model, np.array([0.2, 0.5, 0.4, 0.3]), [tip, joint], mc.PriorityConfig(), 0.01, 800)
    assert r.error_norm[-1].max() < 1e-6
    np.testing.assert_allclose(r.ee_position[-1, :2], [0.6, 0.5], atol=1e-6)
