"""Chapter 3 cross-checks: Jacobians of the C++ library against Pinocchio."""

import numpy as np
import pytest

import manipulator_control as mc

pin = pytest.importorskip("pinocchio")
from manipulator_control.pinocchio_ref import frame_jacobian, model_from_chain  # noqa: E402

rng = np.random.default_rng(3)
ARMS = [lambda: mc.robots.planar([0.75, 0.5, 0.5]), mc.robots.puma560, mc.robots.stanford]


@pytest.mark.parametrize("make_arm", ARMS)
def test_link_and_end_effector_jacobians_match_pinocchio(make_arm):
    arm = make_arm()
    arm.base = mc.make_transform(mc.rot_y(0.2), [0.3, 0.0, 0.1])
    arm.tool = mc.translation([0.0, 0.05, 0.1])
    model, data, ids = model_from_chain(arm)
    for _ in range(10):
        q = rng.uniform(-np.pi, np.pi, arm.dof)
        for k in range(1, arm.dof + 2):  # DH frames 1..n and the end effector
            np.testing.assert_allclose(mc.link_jacobian(arm, q, k), frame_jacobian(model, data, ids[k], q),
                                       atol=1e-12)


def test_space_jacobian_columns_are_current_joint_screws():
    arm = mc.robots.puma560()
    poe = mc.to_poe(arm)
    q = rng.uniform(-np.pi, np.pi, 6)
    frames = arm.frames(q)
    Js = mc.space_jacobian(poe, q)
    for i in range(6):  # column i is the screw of joint i+1 at the current configuration
        z, o = frames[i][:3, 2], frames[i][:3, 3]
        np.testing.assert_allclose(Js[:, i], np.concatenate([-np.cross(z, o), z]), atol=1e-12)
