"""Shared setup for the learning notebooks.

Every notebook starts with ``from nbsetup import *``. This module

* checks that the ``quadrotor_mpc`` package from ``2.Code`` is importable,
* exposes the important paths (``CODE``, ``CONFIG``, ``OUTPUT``),
* applies one plotting style and the controller colours used across the repo,
* provides ``FAST`` to shorten long runs (set ``QMPC_NOTEBOOK_FAST=1``; CI does).

Notebooks never duplicate package code: everything they plot comes from
``quadrotor_mpc`` so it cannot drift from what the simulator actually runs.
"""

from __future__ import annotations

import os
import sys
import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

NOTEBOOKS = Path(__file__).resolve().parent
REPOSITORY = NOTEBOOKS.parent
CODE = REPOSITORY / "2.Code"
CONFIG = CODE / "config"
OUTPUT = NOTEBOOKS / "outputs"  # ignored by Git
OUTPUT.mkdir(exist_ok=True)

# do-mpc announces optional features it cannot load; they are not used here.
warnings.filterwarnings("ignore", message=r".*(ONNX|opcua|approximateMPC).*")
# do-mpc calls np.any on CasADi bounds; casadi>=3.8 warns about it once per process.
warnings.filterwarnings(
    "ignore",
    message=r"\s*casadi: a numpy function was called on a casadi value",
    category=FutureWarning,
)

FAST = os.environ.get("QMPC_NOTEBOOK_FAST", "0") == "1"

try:
    import quadrotor_mpc  # noqa: F401
except ModuleNotFoundError as error:  # pragma: no cover - guidance for users
    raise ModuleNotFoundError(
        "quadrotor_mpc is not installed in this kernel. Start Jupyter from the "
        "2.Code environment:\n"
        "    cd 2.Code\n"
        "    uv sync --all-extras --locked\n"
        "    uv run jupyter lab ../3.Notebooks"
    ) from error

os.environ.setdefault("MUJOCO_GL", "egl" if sys.platform.startswith("linux") else "")
if not os.environ["MUJOCO_GL"]:
    del os.environ["MUJOCO_GL"]

# Colour follows the controller everywhere in the repository (workbench, README).
CONTROLLER_COLORS = {
    "deterministic": "#3987e5",
    "individual": "#d95926",
    "joint_uniform": "#199e70",
    "ccmpc": "#199e70",
}
CONTROLLER_LABELS = {
    "deterministic": "Deterministic",
    "individual": "Individual risk",
    "joint_uniform": "Joint risk",
    "ccmpc": "CC-MPC",
}
NEUTRAL = "#6e6d68"
CRITICAL = "#d03b3b"

plt.rcParams.update(
    {
        "figure.figsize": (10, 4.2),
        "figure.dpi": 110,
        "axes.grid": True,
        "grid.alpha": 0.3,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.titlelocation": "left",
        "axes.titleweight": "bold",
        "legend.frameon": False,
    }
)
np.set_printoptions(precision=4, suppress=True)

__all__ = [
    "CODE",
    "CONFIG",
    "CONTROLLER_COLORS",
    "CONTROLLER_LABELS",
    "CRITICAL",
    "FAST",
    "NEUTRAL",
    "OUTPUT",
    "REPOSITORY",
    "np",
    "plt",
]


def ellipse_outline(omega, center, points: int = 240):
    """x-y outline of the ellipsoid ``(p-c)' omega (p-c) <= 1`` at the centre height."""
    angles = np.linspace(0.0, 2.0 * np.pi, points)
    directions = np.stack([np.cos(angles), np.sin(angles), np.zeros_like(angles)], axis=1)
    radius = 1.0 / np.sqrt(np.einsum("ij,jk,ik->i", directions, np.asarray(omega), directions))
    center = np.asarray(center, dtype=float)
    return center[0] + radius * directions[:, 0], center[1] + radius * directions[:, 1]


__all__.append("ellipse_outline")
