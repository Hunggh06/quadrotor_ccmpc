"""Applied-command provenance must match the command that reaches the plant.

Regression for ``application/native/runtime.py``: under the realtime
qualification protocol the runtime used to record
``FALLBACK_APPLIED_DEADLINE_MISS`` on an end-to-end deadline miss while still
applying the primary command (the supervisor had accepted it on its own
``solve_deadline_s`` clock).  The fix enforces the fallback through
``SafeFallbackController.apply_deadline_fallback`` and reconciles the labels
with what was actually applied when no supervisor is available.
"""

from __future__ import annotations

import importlib.util
import unittest
from pathlib import Path

import numpy as np

from quadrotor_mpc.application.validation.protocol import (
    FALLBACK_APPLIED_DEADLINE_MISS,
    FALLBACK_APPLIED_PRIMARY_INVALID,
    FALLBACK_SOURCE,
    PRIMARY_APPLIED_LATE,
    PRIMARY_APPLIED_ON_TIME,
    PRIMARY_SOURCE,
    MonteCarloProtocol,
    reconcile_applied_command,
    select_applied_command,
)
from quadrotor_mpc.control.nmpc.safety import (
    SafeFallbackController,
    SafetyFallbackOptions,
)
from quadrotor_mpc.core.contracts import ControlGoal, ControlSolution, VehicleBelief

CODE_ROOT = Path(__file__).resolve().parents[1]
MUJOCO_AVAILABLE = importlib.util.find_spec("mujoco") is not None
CASADI_AVAILABLE = importlib.util.find_spec("casadi") is not None
DO_MPC_AVAILABLE = importlib.util.find_spec("do_mpc") is not None

BOUNDS = {"thrust": 0.085, "torque_rp": 0.002, "torque_yaw": 0.0004}
GOAL = ControlGoal(
    position=np.array([1.0, 0.0, 1.0]),
    quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
)
RT = MonteCarloProtocol.REALTIME_QUALIFICATION
ALGO = MonteCarloProtocol.ALGORITHMIC_COMPARISON


def _belief() -> VehicleBelief:
    return VehicleBelief.exact(np.array([0.0, 0.0, 1.0, 0, 0, 0, 1.0, 0, 0, 0, 0, 0, 0]))


def _solution(command=(0.01, 0.0, 0.0, 0.0)) -> ControlSolution:
    steps = 3
    nominal = np.repeat(_belief().mean_state_13.reshape(1, 13), steps, axis=0)
    zeros = np.zeros((steps, 1))
    return ControlSolution(
        command=np.asarray(command, dtype=float),
        nominal_states=nominal,
        predicted_covariances=np.repeat((np.eye(12) * 1e-3)[None], steps, axis=0),
        predicted_obstacle_covariances=np.repeat((np.eye(6) * 1e-3)[None, None], steps, axis=0),
        chance_margins=zeros,
        risk_allocations=np.full((steps, 1), 0.1 / steps),
        slacks=zeros.copy(),
        projected_uncertainties=np.full((steps, 1), 0.1),
        tightened_safety_radii=np.full((steps, 1), 0.8),
        solver_status="SOLVED_SAFE",
        risk_semantics="joint",
        risk_allocation_method="uniform",
        risk_budget_total=0.1,
        risk_budget_allocated=0.1,
        risk_budget_remaining=0.0,
        risk_constraint_count=steps,
        risk_budget_status="BUDGET_OK",
        primary_solver_status="Solve_Succeeded",
        primary_solver_success=True,
        primary_solver_primal_residual=1e-7,
        primary_solver_dual_residual=1e-7,
        primary_solver_primal_residual_status="AVAILABLE",
        primary_solver_dual_residual_status="AVAILABLE",
        residual_status="AVAILABLE",
    )


class _Scripted:
    horizon_steps = 2

    def __init__(self, script):
        self.script = list(script)

    def reset(self, _belief):
        pass

    def solve(self, _belief, _obstacles, _goal, _time_s):
        return self.script.pop(0)


def _supervisor(script, **overrides) -> SafeFallbackController:
    options = SafetyFallbackOptions(
        enabled=True,
        solve_deadline_s=overrides.pop("solve_deadline_s", 1.0),
        **overrides,
    )
    return SafeFallbackController(_Scripted(script), options=options, bounds=BOUNDS)


class ReconcileAppliedCommandTests(unittest.TestCase):
    def test_consistent_decision_is_unchanged(self):
        decision = select_applied_command(protocol=RT, primary_valid=True, deadline_missed=True)
        self.assertIs(
            reconcile_applied_command(
                decision, fallback_applied=True, fallback_reason="DEADLINE_MISSED"
            ),
            decision,
        )

    def test_claimed_fallback_that_was_not_applied_is_relabelled_primary(self):
        decision = select_applied_command(protocol=RT, primary_valid=True, deadline_missed=True)
        self.assertEqual(decision.primary_disposition, FALLBACK_APPLIED_DEADLINE_MISS)
        reconciled = reconcile_applied_command(decision, fallback_applied=False)
        self.assertEqual(reconciled.applied_command_source, PRIMARY_SOURCE)
        self.assertEqual(reconciled.primary_disposition, PRIMARY_APPLIED_LATE)
        self.assertTrue(reconciled.deadline_missed)

    def test_hidden_supervisor_fallback_is_relabelled_fallback(self):
        decision = select_applied_command(protocol=RT, primary_valid=True, deadline_missed=False)
        self.assertEqual(decision.primary_disposition, PRIMARY_APPLIED_ON_TIME)
        deadline = reconcile_applied_command(
            decision, fallback_applied=True, fallback_reason="DEADLINE_MISSED"
        )
        self.assertEqual(deadline.applied_command_source, FALLBACK_SOURCE)
        self.assertEqual(deadline.primary_disposition, FALLBACK_APPLIED_DEADLINE_MISS)
        other = reconcile_applied_command(
            decision, fallback_applied=True, fallback_reason="SLACK_LIMIT_EXCEEDED"
        )
        self.assertEqual(other.primary_disposition, FALLBACK_APPLIED_PRIMARY_INVALID)


class ApplyDeadlineFallbackTests(unittest.TestCase):
    def test_accepted_primary_is_replaced_and_acceptance_rolled_back(self):
        on_time = (0.02, 0.0, 0.0, 0.0)
        late = (0.03, 0.0, 0.0, 0.0)
        controller = _supervisor(
            [_solution(on_time), _solution(late)],
            hold_last_command_steps=5,
        )
        belief = _belief()
        first = controller.solve(belief, [], GOAL, 0.0)
        self.assertFalse(first.fallback_active)
        second = controller.solve(belief, [], GOAL, 0.05)
        self.assertFalse(second.fallback_active)  # supervisor clock: on time

        enforced = controller.apply_deadline_fallback(second, belief)

        self.assertTrue(enforced.fallback_active)
        self.assertFalse(enforced.solution_accepted)
        self.assertTrue(enforced.deadline_missed)
        self.assertEqual(enforced.fallback_reason, "DEADLINE_MISSED")
        self.assertEqual(enforced.command_source, "HOLD_LAST_ACCEPTED")
        self.assertEqual(enforced.consecutive_rejections, 1)
        # Holds the last *on-time* command, never the late one being rejected.
        np.testing.assert_allclose(enforced.command, on_time)
        self.assertFalse(enforced.horizon_assurance_eligible)

    def test_existing_fallback_is_returned_unchanged(self):
        controller = _supervisor([_solution()])
        belief = _belief()
        accepted = controller.solve(belief, [], GOAL, 0.0)
        enforced = controller.apply_deadline_fallback(accepted, belief)
        self.assertIs(controller.apply_deadline_fallback(enforced, belief), enforced)


@unittest.skipUnless(
    MUJOCO_AVAILABLE and CASADI_AVAILABLE and DO_MPC_AVAILABLE,
    "optional NMPC/MuJoCo dependencies are not installed",
)
class RuntimeAppliedCommandTests(unittest.TestCase):
    """End-to-end: the logged disposition must describe the plant input."""

    PRIMARY = np.array([0.01, 0.0005, 0.0, 0.0])

    def _run(self, *, protocol: str, fallback_enabled: bool):
        from quadrotor_mpc.application.native.runtime import run_coupled_simulation
        from quadrotor_mpc.interfaces.desktop.viewer import load_native_mujoco_config

        config = load_native_mujoco_config(CODE_ROOT / "config" / "mujoco_native_ccmpc.yaml")
        return run_coupled_simulation(
            x0_vals={"x": 0.0, "y": 0.0, "z": 1.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
            goal_pos=config.goal_position,
            goal_euler=config.goal_euler,
            bounds=config.bounds,
            obstacles=[],
            margin=config.safety_margin,
            sim_seconds=0.10,
            mpc_dt=config.mpc_timestep_s,
            n_horizon=2,
            max_iter=10,
            mj_dt=config.mujoco_timestep_s,
            controller=_Scripted([_solution(self.PRIMARY), _solution(self.PRIMARY)]),
            safety_fallback_options=SafetyFallbackOptions(
                enabled=fallback_enabled,
                # Supervisor clock never misses; only the end-to-end clock does.
                solve_deadline_s=10.0,
                reject_on_deadline_miss=True,
                hold_last_command_steps=0,
            ),
            protocol_type=protocol,
            control_period_ms=1e-6,
        )

    def test_realtime_deadline_miss_applies_fallback_to_plant(self):
        result = self._run(protocol=RT.value, fallback_enabled=True)
        self.assertTrue(np.all(result["applied_command_source"] == FALLBACK_SOURCE))
        self.assertTrue(np.all(result["primary_disposition"] == FALLBACK_APPLIED_DEADLINE_MISS))
        for applied in result["u"]:
            self.assertFalse(np.allclose(applied, self.PRIMARY))

    def test_realtime_without_supervisor_reports_late_primary(self):
        result = self._run(protocol=RT.value, fallback_enabled=False)
        self.assertTrue(np.all(result["applied_command_source"] == PRIMARY_SOURCE))
        self.assertTrue(np.all(result["primary_disposition"] == PRIMARY_APPLIED_LATE))
        for applied in result["u"]:
            np.testing.assert_allclose(applied, self.PRIMARY)

    def test_algorithmic_comparison_still_applies_late_primary(self):
        result = self._run(protocol=ALGO.value, fallback_enabled=True)
        self.assertTrue(np.all(result["primary_disposition"] == PRIMARY_APPLIED_LATE))
        for applied in result["u"]:
            np.testing.assert_allclose(applied, self.PRIMARY)


if __name__ == "__main__":
    unittest.main()
