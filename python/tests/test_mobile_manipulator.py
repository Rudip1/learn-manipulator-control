"""Chapter 9: mobile-manipulator kinematics against an independent NumPy implementation of eqs. (9.3)-(9.5)."""

import numpy as np

import manipulator_control as mc

rng = np.random.default_rng(9)


def robot():
    mount = np.eye(4)
    mount[:3, 3] = [0.1, 0.0, 0.2]
    return mc.MobileManipulator(mc.robots.anthropomorphic(), mount)


def test_end_effector_pose_is_base_mount_arm():
    r = robot()
    for _ in range(5):
        q = rng.uniform(-np.pi, np.pi, 6)
        expected = mc.base_transform(q[:3]) @ r.mount @ r.arm.end_effector(q[3:])
        np.testing.assert_allclose(r.frame_pose(q, -1), expected, atol=1e-12)


def test_jacobian_columns():
    r = robot()
    q = rng.uniform(-np.pi, np.pi, 6)
    J = r.frame_jacobian(q, -1)
    p = r.frame_pose(q, -1)[:3, 3]
    np.testing.assert_allclose(J[:3, 0], np.cross([0, 0, 1], p - np.array([q[0], q[1], 0.0])), atol=1e-12)
    np.testing.assert_allclose(J[:3, 1], [np.cos(q[2]), np.sin(q[2]), 0.0], atol=1e-12)
    R = (mc.base_transform(q[:3]) @ r.mount)[:3, :3]
    Ja = mc.geometric_jacobian(r.arm, q[3:])
    np.testing.assert_allclose(J[:3, 2:], R @ Ja[:3], atol=1e-12)
    np.testing.assert_allclose(J[3:, 2:], R @ Ja[3:], atol=1e-12)


def test_base_position_task_has_rank_one():
    r = robot()
    q = rng.uniform(-np.pi, np.pi, 6)
    task = mc.FrameTask("base xy", 0, [0, 1], np.eye(4))
    task.update(r, q)
    assert np.linalg.matrix_rank(task.jacobian) == 1
