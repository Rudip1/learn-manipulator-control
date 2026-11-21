"""Chapter 8: activation state machine and set-based tasks through the Python interface."""

import numpy as np

import manipulator_control as mc


def test_activation_sequence():
    s, seq = 0, []
    for sigma in [0.85, 0.91, 0.80, 0.69, -0.95, -0.75, -0.65]:
        s = mc.next_activation(s, sigma, -1.0, 1.0, 0.1, 0.3)
        seq.append(s)
    assert seq == [0, -1, -1, 0, 1, 1, 0]


def test_joint_limit_holds():
    arm = mc.robots.planar([0.5, 0.4, 0.3])
    model = mc.SerialArmModel(arm)
    target = np.eye(4)
    target[:2, 3] = [0.6, 0.5]
    limit = mc.JointLimitTask("q3", 2, -1.7, 1.7, 0.05, 0.15)
    tip = mc.FrameTask("tip", -1, [0, 1], target)
    limit.set_gain(0.5)
    tip.set_gain(2.0)
    r = mc.simulate_task_priority(model, np.array([1.2, -1.5, 1.6]), [limit, tip], mc.PriorityConfig(), 0.01, 1000)
    assert r.q[:, 2].max() < 1.7
    assert r.error_norm[-1, 1] < 1e-6
    assert limit.activation == 0  # released at the end


def test_obstacle_distance_and_jacobian():
    arm = mc.robots.planar([0.5, 0.4, 0.3])
    model = mc.SerialArmModel(arm)
    q = np.array([0.3, 0.7, 0.6])
    c = np.array([0.4, 0.4, 0.0])
    task = mc.ObstacleTask("obs", 2, c, 0.1, 0.02, 0.05, True)
    task.update(model, q)
    p = arm.frames(q)[2][:3, 3]
    np.testing.assert_allclose(task.value, np.linalg.norm((p - c)[:2]), atol=1e-14)
    d = (p - c) / np.linalg.norm(p - c)
    np.testing.assert_allclose(task.jacobian[0], d @ mc.link_jacobian(arm, q, 2)[:3], atol=1e-12)
