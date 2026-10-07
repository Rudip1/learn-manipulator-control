"""Kinematic control of manipulators: Python interface to the C++ library.

Every algorithm lives in C++ (``manipulator_control._core``); this package re-exports it and adds plotting
helpers in :mod:`manipulator_control.plotting`.
"""

from ._core import *  # noqa: F401,F403
from ._core import robots  # noqa: F401

__version__ = "0.1.0"
