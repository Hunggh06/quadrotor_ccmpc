"""Native spherical chance-constrained NMPC controller."""

from __future__ import annotations

from simulation.control.ccmpc_13state.chance_constraints import ChanceConstraintOptions
from simulation.control.ccmpc_13state.deterministic import DeterministicNMPCController


class SphericalChanceConstrainedNMPCController(DeterministicNMPCController):
    """Native spherical CC-NMPC with externally allocated Stage 5 risk."""

    def __init__(
        self,
        *,
        chance_options: ChanceConstraintOptions,
        **kwargs,
    ):
        if not chance_options.enabled:
            raise ValueError(
                "SphericalChanceConstrainedNMPCController requires enabled chance constraints"
            )
        super().__init__(chance_options=chance_options, **kwargs)
