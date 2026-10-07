#!/usr/bin/env python3
"""Regenerate the figures in 1_theory/figures from the library. Never edit those PNGs by hand.

    python tools/make_figures.py          # all figures
    python tools/make_figures.py 1 4      # only chapters 1 and 4
"""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import manipulator_control as mc  # noqa: E402
from manipulator_control import plotting  # noqa: E402

OUT = Path(__file__).resolve().parents[1] / "1_theory" / "figures"
DPI = 120


def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / name, dpi=DPI, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote 1_theory/figures/{name}")


def chapter_1():
    """Figure 1.1: frames along a screw motion."""
    q, w, h = np.array([0.5, 0.0, 0.0]), np.array([0.0, 0.0, 1.0]), 0.1
    S = mc.screw_axis(q, w, h)
    fig = plt.figure(figsize=(5, 5))
    ax = fig.add_subplot(projection="3d")
    thetas = np.linspace(0.0, 2 * np.pi, 400)
    path = np.array([mc.exp_se3(S * t)[:3, 3] for t in thetas])
    ax.plot(*path.T, color="0.4", linewidth=1)
    for t in np.linspace(0.0, 2 * np.pi, 9):
        plotting.draw_frame(ax, mc.exp_se3(S * t), length=0.15)
    ax.plot([q[0], q[0]], [q[1], q[1]], [-0.05, 0.75], "k--", linewidth=1)
    ax.text(q[0], q[1], 0.78, "screw axis")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    plotting.set_axes_equal(ax)
    save(fig, "01_screw_motion.png")


def chapter_2():
    """Figure 2.1: DH frames of a planar three-link arm and of the PUMA 560."""
    fig = plt.figure(figsize=(10, 4.5))
    ax = fig.add_subplot(1, 2, 1)
    planar = mc.robots.planar([0.75, 0.5, 0.5])
    frames = planar.frames(np.array([0.4, 0.6, -0.5]))
    plotting.draw_planar_arm(ax, frames, color="0.3")
    for T in frames[:-1]:
        for k in range(2):
            tip = T[:2, 3] + 0.18 * T[:2, k]
            ax.annotate("", xy=tip, xytext=T[:2, 3],
                        arrowprops=dict(arrowstyle="->", color=plotting.AXIS_COLORS[k], lw=1.8))
    plotting.planar_axes(ax, limit=1.7, title="planar arm, $q = (0.4, 0.6, -0.5)$")
    ax.set_xlim(-0.3, 1.8)
    ax.set_ylim(-0.3, 1.8)

    ax = fig.add_subplot(1, 2, 2, projection="3d")
    puma = mc.robots.puma560()
    plotting.draw_arm_3d(ax, puma.frames(np.array([0.5, np.pi / 4, -np.pi / 4, 0.0, np.pi / 3, 0.0])),
                         frame_length=0.12)
    ax.set_title("PUMA 560")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    plotting.set_axes_equal(ax)
    ax.view_init(elev=20, azim=-60)
    save(fig, "02_dh_frames.png")


def chapter_3():
    """Figure 3.1: columns of the position Jacobian of a planar arm, drawn at the end-effector."""
    arm = mc.robots.planar([0.75, 0.5, 0.5])
    q = np.array([0.3, 0.9, 0.6])
    frames = arm.frames(q)
    J = mc.geometric_jacobian(arm, q)
    p = frames[-1][:2, 3]
    fig, ax = plt.subplots(figsize=(5.5, 5))
    plotting.draw_planar_arm(ax, frames, color="0.3")
    for i in range(3):
        o = frames[i][:2, 3]
        ax.plot([o[0], p[0]], [o[1], p[1]], ":", color=f"C{i}", linewidth=1)
        ax.annotate("", xy=p + 0.5 * J[:2, i], xytext=p,
                    arrowprops=dict(arrowstyle="->", color=f"C{i}", lw=2))
        tip = p + 0.5 * J[:2, i]
        ax.text(*(tip + 0.12 * J[:2, i] / np.linalg.norm(J[:2, i]) + np.array([-0.08, 0.03 * (1 - i)])),
                f"$J_{{P,{i + 1}}}$", color=f"C{i}", fontsize=12)
    plotting.planar_axes(ax)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-0.2, 1.9)
    ax.set_title("columns of $J_P$ (scaled by 0.5)")
    save(fig, "03_jacobian_columns.png")


def chapter_4():
    """Figure 4.1: transpose, pseudoinverse and DLS resolved-rate control of a planar two-link arm."""
    arm = mc.robots.planar([0.75, 0.5])
    q0, goal = np.array([0.2, 0.5]), mc.translation([0.0, 1.0, 0.0])
    methods = [("transpose", mc.InverseMethod.Transpose), ("pseudoinverse", mc.InverseMethod.Pseudoinverse),
               ("DLS, $\\lambda = 0.1$", mc.InverseMethod.DampedLeastSquares)]
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.2))
    plotting.draw_planar_arm(ax[0], arm.frames(q0), color="0.6")
    for k, (name, method) in enumerate(methods):
        cfg = mc.ResolvedRateConfig(method=method, space=mc.TaskSpace.PlanarPosition, gain=1.0, damping=0.1,
                                    dt=1 / 60, steps=600)
        traj = mc.simulate_resolved_rate(arm, q0, goal, cfg)
        ax[0].plot(traj.ee_position[:, 0], traj.ee_position[:, 1], color=f"C{k}", label=name)
        ax[1].semilogy(traj.t, traj.error_norm, color=f"C{k}", label=name)
    ax[1].semilogy(traj.t, traj.error_norm[0] * np.exp(-traj.t), "k:", label="$e^{-kt}$")
    ax[0].plot(0.0, 1.0, "kx", markersize=10)
    plotting.planar_axes(ax[0], limit=1.4, title="end-effector paths")
    ax[0].legend(fontsize=9)
    ax[1].set_xlabel("t [s]")
    ax[1].set_ylabel(r"$\|\tilde\sigma\|$ [m]")
    ax[1].set_title("task error, $k = 1$, $\\Delta t = 1/60$ s")
    ax[1].legend(fontsize=9)
    save(fig, "04_resolved_rate_methods.png")


def wrist_singularity_scenario():
    """PUMA 560 straight-line path that drives q5 through zero (shared with the chapter 5 notebook)."""
    puma = mc.robots.puma560()
    qa = np.array([0.0, 0.6, -0.3, 0.5, 0.15, 0.0])
    Ta = puma.end_effector(qa)
    g = np.linalg.inv(mc.geometric_jacobian(puma, qa))[4, :3]  # how q5 responds to a pure translation
    dp = -0.15 * g / np.linalg.norm(g)
    T, dt = 4.0, 0.005
    k = np.arange(int(T / dt) + 1)
    s = 0.5 * (1 - np.cos(np.pi * k / k[-1]))
    sd = 0.5 * np.pi / T * np.sin(np.pi * k / k[-1])
    poses, twists = [], np.zeros((k.size, 6))
    for i in k:
        Td = Ta.copy()
        Td[:3, 3] += s[i] * dp
        poses.append(Td)
        twists[i, :3] = sd[i] * dp
    return puma, qa, poses, twists, dt


def chapter_5():
    """Figure 5.1: velocity ellipses and manipulability. Figure 5.2: wrist singularity."""
    arm = mc.robots.planar([0.75, 0.5])
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.2))
    angle = np.linspace(0, 2 * np.pi, 100)
    for k, q2 in enumerate([1.6, 1.0, 0.5, 0.2, 0.02]):
        q = np.array([1.0 - 0.25 * k, q2])
        J = mc.geometric_jacobian(arm, q)[:2]
        e = mc.manipulability_ellipsoid(J)
        p = arm.end_effector(q)[:2, 3]
        pts = p[:, None] + 0.3 * e.axes @ (e.radii[:, None] * np.vstack([np.cos(angle), np.sin(angle)]))
        plotting.draw_planar_arm(ax[0], arm.frames(q), color=f"C{k}", alpha=0.5, linewidth=1.5)
        ax[0].plot(pts[0], pts[1], color=f"C{k}", label=f"$q_2$ = {q2}")
    plotting.planar_axes(ax[0], limit=1.4, title="velocity ellipses (scaled by 0.3)")
    ax[0].set_xlim(-0.3, 1.6)
    ax[0].set_ylim(-0.4, 1.5)
    ax[0].legend(fontsize=8)
    q2s = np.linspace(-np.pi, np.pi, 400)
    w = [mc.manipulability(mc.geometric_jacobian(arm, np.array([0.0, q2]))[:2]) for q2 in q2s]
    ax[1].plot(q2s, w)
    ax[1].set_xlabel("$q_2$ [rad]")
    ax[1].set_ylabel("$w$ [m$^2$]")
    ax[1].set_title(r"manipulability $l_1 l_2 |\sin q_2|$")
    save(fig, "05_manipulability.png")

    puma, qa, poses, twists, dt = wrist_singularity_scenario()
    runs = {
        "pseudoinverse": mc.ResolvedRateConfig(gain=5.0, dt=dt),
        "DLS, $\\lambda = 0.05$": mc.ResolvedRateConfig(method=mc.InverseMethod.DampedLeastSquares, damping=0.05,
                                                         gain=5.0, dt=dt),
        "variable DLS (5.9)": mc.ResolvedRateConfig(method=mc.InverseMethod.DampedLeastSquares, damping=0.05,
                                                    damping_schedule=mc.DampingSchedule.MinSingularValue,
                                                    damping_threshold=0.05, gain=5.0, dt=dt),
    }
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.6))
    for k, (name, cfg) in enumerate(runs.items()):
        tr = mc.simulate_pose_tracking(puma, qa, poses, twists, cfg)
        ax[0].plot(tr.t, tr.q[:, 3], color=f"C{k}", label=name)
        ax[0].plot(tr.t, tr.q[:, 4], color=f"C{k}", linestyle="--")
        ax[1].semilogy(tr.t[:-1], np.abs(tr.qdot[:-1]).max(axis=1), color=f"C{k}", label=name)
        ax[2].semilogy(tr.t, np.maximum(tr.error_norm, 1e-7), color=f"C{k}", label=name)
    ax[2].set_ylim(1e-6, 2e-2)
    ax[0].set_title("$q_4$ (solid) and $q_5$ (dashed) [rad]")
    ax[1].set_title(r"max $|\dot q_i|$ [rad/s]")
    ax[1].set_ylim(1e-3, 1e2)
    ax[2].set_title("pose error")
    for a in ax:
        a.set_xlabel("t [s]")
    ax[0].legend(fontsize=8)
    save(fig, "05_wrist_singularity.png")


CHAPTERS = {1: chapter_1, 2: chapter_2, 3: chapter_3, 4: chapter_4, 5: chapter_5}


def main(argv):
    selected = [int(a) for a in argv] or sorted(CHAPTERS)
    for c in selected:
        CHAPTERS[c]()


if __name__ == "__main__":
    main(sys.argv[1:])
