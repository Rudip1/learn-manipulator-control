"""Matplotlib helpers shared by the notebooks and the figure scripts. No algorithms here."""

from __future__ import annotations

import numpy as np

AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")  # x, y, z


def draw_frame(ax, T, length=0.2, label=None, alpha=1.0, linewidth=2.0):
    """Draw the axes of the homogeneous transform ``T`` on a 3D axes object (x red, y green, z blue)."""
    T = np.asarray(T)
    origin = T[:3, 3]
    for k in range(3):
        tip = origin + length * T[:3, k]
        ax.plot(*zip(origin, tip), color=AXIS_COLORS[k], alpha=alpha, linewidth=linewidth)
    if label is not None:
        ax.text(*(origin + 0.03), label)


def set_axes_equal(ax):
    """Give a 3D axes object equal scale on x, y and z."""
    limits = np.array([ax.get_xlim3d(), ax.get_ylim3d(), ax.get_zlim3d()])
    centre = limits.mean(axis=1)
    radius = 0.5 * np.max(limits[:, 1] - limits[:, 0])
    ax.set_xlim3d(centre[0] - radius, centre[0] + radius)
    ax.set_ylim3d(centre[1] - radius, centre[1] + radius)
    ax.set_zlim3d(centre[2] - radius, centre[2] + radius)


def draw_planar_arm(ax, frames, color="C0", alpha=1.0, label=None, linewidth=2.5):
    """Draw the x-y projection of a chain from its list of frames (the output of ``SerialChain.frames``)."""
    pts = np.array([np.asarray(T)[:2, 3] for T in frames])
    ax.plot(pts[:, 0], pts[:, 1], "o-", color=color, alpha=alpha, label=label, linewidth=linewidth,
            markersize=5)
    ax.plot(pts[0, 0], pts[0, 1], "ks", markersize=7, alpha=alpha)


def draw_arm_3d(ax, frames, color="0.3", frame_length=0.0, linewidth=2.5):
    """Draw a chain in 3D as segments between frame origins; optionally draw each frame's axes."""
    pts = np.array([np.asarray(T)[:3, 3] for T in frames])
    ax.plot(*pts.T, "o-", color=color, linewidth=linewidth, markersize=4)
    if frame_length > 0.0:
        for T in frames:
            draw_frame(ax, T, length=frame_length, linewidth=1.5)


def planar_axes(ax, limit=2.0, title=None):
    """Square x-y axes with equal scale, for planar arms."""
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    if title:
        ax.set_title(title)
