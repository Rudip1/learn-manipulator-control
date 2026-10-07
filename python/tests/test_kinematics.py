"""Chapter 2 cross-checks: forward kinematics of the C++ library against Pinocchio."""

import numpy as np
import pytest

import manipulator_control as mc

pin = pytest.importorskip("pinocchio")
from manipulator_control.pinocchio_ref import frame_poses, model_from_chain  # noqa: E402

rng = np.random.default_rng(1)

ARMS = {
    "planar3": lambda: mc.robots.planar([0.75, 0.5, 0.5]),
    "puma560": mc.robots.puma560,
    "stanford": mc.robots.stanford,
}


@pytest.mark.parametrize("name", ARMS)
def test_every_dh_frame_matches_pinocchio(name):
    arm = ARMS[name]()
    arm.base = mc.make_transform(mc.rot_z(0.3), [0.1, -0.2, 0.05])
    arm.tool = mc.make_transform(mc.rot_x(0.5), [0.0, 0.0, 0.08])
    model, data, ids = model_from_chain(arm)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, arm.dof)
        ours = arm.frames(q)[1:]  # T_1 .. T_n, T_e
        ref = frame_poses(model, data, ids[1:], q)
        for T, T_ref in zip(ours, ref):
            np.testing.assert_allclose(T, T_ref, atol=1e-12)


@pytest.mark.parametrize("name", ARMS)
def test_product_of_exponentials_matches_pinocchio(name):
    arm = ARMS[name]()
    model, data, ids = model_from_chain(arm)
    poe = mc.to_poe(arm)
    for _ in range(20):
        q = rng.uniform(-np.pi, np.pi, arm.dof)
        T_ref = frame_poses(model, data, [ids[-1]], q)[0]
        np.testing.assert_allclose(mc.poe_space(poe, q), T_ref, atol=1e-12)
        np.testing.assert_allclose(mc.poe_body(poe, q), T_ref, atol=1e-12)


def test_wrong_size_raises():
    with pytest.raises(ValueError):
        mc.robots.puma560().end_effector(np.zeros(4))
