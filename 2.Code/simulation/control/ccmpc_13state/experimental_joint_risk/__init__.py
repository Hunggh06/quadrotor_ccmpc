"""Explicit opt-in joint trajectory/risk optimization research controller."""

from dataclasses import asdict, dataclass
import math
import time

import numpy as np

from simulation.control.ccmpc_13state.chance_constrained import (
    SphericalChanceConstrainedNMPCController,
)


@dataclass(frozen=True, slots=True)
class JointRiskOptions:
    enabled: bool = True
    formulation: str = "required"
    total_epsilon: float = 0.1
    delta_min: float = 1e-4
    delta_max: float = 0.2
    feasibility_tolerance: float = 1e-8

    def __post_init__(self):
        if not all(math.isfinite(value) for value in (
            self.total_epsilon, self.delta_min, self.delta_max, self.feasibility_tolerance,
        )):
            raise ValueError("joint risk options must be finite")
        formulation = str(self.formulation).strip().lower()
        object.__setattr__(self, "formulation", formulation)
        if formulation != "required":
            raise ValueError("joint risk formulation must be required")
        if not 0 < self.delta_min <= self.delta_max < 0.5:
            raise ValueError("joint risk requires 0 < delta_min <= delta_max < 0.5")
        if not 0 < self.total_epsilon < 0.5 or self.feasibility_tolerance < 0:
            raise ValueError("invalid joint budget or feasibility tolerance")


class JointRiskNMPCController(SphericalChanceConstrainedNMPCController):
    """Optimize delta and trajectory together, with frozen seed-based sigma.

    Hard obstacle constraints reproduce the proposed mathematical formulation.
    Soft obstacle constraints are a separately labeled relaxation, not a safety
    guarantee. The existing safety supervisor still decides command acceptance.
    """

    def __init__(self, *, joint_options: JointRiskOptions, **kwargs):
        self.options = joint_options
        self.records = []
        super().__init__(joint_risk_options=joint_options, **kwargs)

    def reset(self, belief):
        self.records.clear()
        super().reset(belief)

    def solve(self, belief, obstacles, goal, time_s, **kwargs):
        started = time.perf_counter()
        solution = super().solve(belief, obstacles, goal, time_s, **kwargs)
        delta = np.asarray(solution.risk_allocations)
        self.records.append({
            "control_tick": len(self.records) + 1,
            "formulation": self.options.formulation,
            "time_s": float(time_s), "delta": delta.tolist(),
            "delta_sum": float(delta.sum()),
            "elapsed_ms": (time.perf_counter() - started) * 1000,
            "objective_value": self.last_objective_value,
            "nlp_slack_max_m": self.last_nlp_slack_max,
            "diagnostic_slack_max_m": float(np.max(solution.slacks)),
            "solver_success": solution.primary_solver_success,
            "solver_status": solution.primary_solver_status,
            "iterations": solution.primary_solver_iterations,
            "budget_status": solution.risk_budget_status,
            "risk_certificate_status": solution.risk_certificate_status,
        })
        return solution

    def report(self):
        return {
            "controller": "experimental_joint_risk", "options": asdict(self.options),
            "sigma_source": "frozen_seed_projection",
            "soft_constraints": self._chance_options.soft_constraint,
            "control_ticks": len(self.records), "records": self.records,
        }
