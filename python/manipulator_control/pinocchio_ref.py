"""Build a Pinocchio model of a :class:`SerialChain` so tests and notebooks can use Pinocchio as an independent
reference. The model is assembled from Pinocchio's own SE3 primitives, not from this library's transforms.

Joint ``i`` of a DH chain moves about (or along) the z axis of frame ``i-1``, and the DH link transform splits as
``A_i(q) = Rz(q) * [Rz(theta_i) Tz(d_i) Tx(a_i) Rx(alpha_i)]`` (revolute) or ``Tz(q) * [...]`` (prismatic). So each
Pinocchio joint is an RZ or PZ joint whose placement is the constant bracket of the previous link.
"""

from __future__ import annotations

import numpy as np


def _fixed_part(pin, link):
    rot = pin.utils.rotate
    zero = np.zeros(3)
    return (pin.SE3(rot("z", link.theta), zero) * pin.SE3(np.eye(3), np.array([0.0, 0.0, link.d]))
            * pin.SE3(np.eye(3), np.array([link.a, 0.0, 0.0])) * pin.SE3(rot("x", link.alpha), zero))


def model_from_chain(chain):
    """Return ``(model, data, frame_ids)``; ``frame_ids[i]`` is the Pinocchio frame of DH frame ``i`` (i = 1..n)
    and ``frame_ids[-1]`` the end-effector frame (with the tool transform)."""
    import pinocchio as pin
    from manipulator_control import JointType

    model = pin.Model()
    parent = 0
    placement = pin.SE3(np.asarray(chain.base))
    frame_ids = [None]
    for i, link in enumerate(chain.links, start=1):
        joint = pin.JointModelRZ() if link.type == JointType.Revolute else pin.JointModelPZ()
        parent = model.addJoint(parent, joint, placement, f"joint{i}")
        model.appendBodyToJoint(parent, pin.Inertia.Identity(), pin.SE3.Identity())
        placement = _fixed_part(pin, link)
        frame_ids.append(model.addFrame(pin.Frame(f"frame{i}", parent, placement, pin.FrameType.OP_FRAME)))
    frame_ids.append(model.addFrame(
        pin.Frame("end_effector", parent, placement * pin.SE3(np.asarray(chain.tool)), pin.FrameType.OP_FRAME)))
    return model, model.createData(), frame_ids


def frame_poses(model, data, frame_ids, q):
    """Poses (4x4) of the given frames at configuration ``q`` according to Pinocchio."""
    import pinocchio as pin

    pin.framesForwardKinematics(model, data, np.asarray(q, dtype=float))
    return [data.oMf[f].homogeneous for f in frame_ids]


def frame_jacobian(model, data, frame_id, q):
    """6 x n Jacobian of a frame origin in world-aligned axes, rows (v, w): the geometric Jacobian."""
    import pinocchio as pin

    q = np.asarray(q, dtype=float)
    pin.computeJointJacobians(model, data, q)
    pin.framesForwardKinematics(model, data, q)
    return pin.getFrameJacobian(model, data, frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
