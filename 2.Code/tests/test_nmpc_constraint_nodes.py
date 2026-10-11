"""The native NMPC must constrain the controllable nodes 1..N, not 0..N-1.

do-mpc evaluates ``set_nl_cons`` at stages k = 0..N-1 on the stage state x_k.
Writing the obstacle constraint on x_k therefore constrained the fixed measured
state x_0 (no control can change it, so any violation became slack) and left
the terminal state x_N unconstrained.  The constraint is now written on the
successor x_{k+1} = f(x_k, u_k) and stage-k obstacle TVPs carry node k+1.
"""

from __future__ import annotations

import importlib.util
import unittest

import numpy as np

CASADI_AVAILABLE = importlib.util.find_spec("casadi") is not None
DO_MPC_AVAILABLE = importlib.util.find_spec("do_mpc") is not None

BOUNDS = {"thrust": 0.085, "torque_rp": 0.002, "torque_yaw": 0.0004}
OBSTACLE = {"type": "static", "x": 1.0, "y": 0.0, "z": 1.0, "radius": 0.3}


@unittest.skipUnless(CASADI_AVAILABLE and DO_MPC_AVAILABLE, "requires casadi and do-mpc")
class ConstraintNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from quadrotor_mpc.control.nmpc.core import build_cached_mpc

        cls.dt = 0.05
        cls.horizon = 3
        cls.model, cls.mpc, cls.obstacle_idx, cls.goal_state = build_cached_mpc(
            BOUNDS, [OBSTACLE], 0.1, n_horizon=cls.horizon, dt=cls.dt, max_iter=5
        )

    def _next_state(self, state: np.ndarray, control: np.ndarray) -> np.ndarray:
        import casadi as ca

        step = ca.Function("step", [self.model.x, self.model.u], [self.model._rhs])
        return np.asarray(step(state, control), dtype=float).reshape(-1)

    def test_obstacle_constraint_acts_on_successor_state(self):
        template = self.mpc.get_tvp_template()
        center = np.array([1.0, 0.0, 1.0])
        safe_distance = 0.6
        for key, value in zip(("obsx_0", "obsy_0", "obsz_0"), center):
            template["_tvp", 0, key] = value
        template["_tvp", 0, "obs_safe_dist_0"] = safe_distance
        tvp = template["_tvp", 0]

        state = np.zeros(13)
        state[2] = 1.0
        state[3] = 2.0  # moving toward the obstacle: successor differs from x_k
        state[6] = 1.0
        control = np.array([0.01, 0.0, 0.0, 0.0])
        value = float(self.mpc._nl_cons_fun(state, control, [], tvp, [], 0.0))

        successor = self._next_state(state, control)
        expected = safe_distance - np.sqrt(np.sum((successor[:3] - center) ** 2) + 1e-6)
        current = safe_distance - np.sqrt(np.sum((state[:3] - center) ** 2) + 1e-6)
        self.assertAlmostEqual(value, expected, places=9)
        self.assertNotAlmostEqual(value, current, places=3)

    def test_stage_tvp_carries_next_prediction_node(self):
        from quadrotor_mpc.control.nmpc.core import make_mpc_tvp_fun

        nodes = self.horizon + 1
        goal_state = {
            "pos": {"x": 2.0, "y": 0.0, "z": 1.0},
            "quaternion_wxyz": np.array([1.0, 0.0, 0.0, 0.0]),
            "obstacle_predictions": np.stack(
                [np.column_stack([np.arange(nodes, dtype=float), np.zeros(nodes), np.ones(nodes)])]
            ),
            "obstacle_safe_distances": np.array([[0.5, 0.6, 0.7, 0.8]]),
        }
        template = make_mpc_tvp_fun(
            self.mpc.get_tvp_template(),
            goal_state,
            [OBSTACLE],
            self.obstacle_idx,
            self.horizon,
            self.dt,
        )(0.0)
        for stage in range(self.horizon):
            node = stage + 1
            self.assertAlmostEqual(float(template["_tvp", stage, "obsx_0"]), float(node))
            self.assertAlmostEqual(
                float(template["_tvp", stage, "obs_safe_dist_0"]), 0.5 + 0.1 * node
            )
        # The unused final slot repeats node N.
        self.assertAlmostEqual(float(template["_tvp", self.horizon, "obsx_0"]), float(self.horizon))


class RiskBudgetNodeTests(unittest.TestCase):
    def test_joint_budget_is_spread_over_constrained_nodes_only(self):
        from quadrotor_mpc.control.nmpc.risk_budget import RiskBudgetOptions, allocate_risk_budget

        allocation = allocate_risk_budget(
            steps=5,
            obstacle_count=2,
            enabled=True,
            individual_epsilon=0.05,
            options=RiskBudgetOptions(semantics="joint", total_epsilon=0.1),
            first_constrained_step=1,
        )
        self.assertEqual(allocation.active_constraint_count, 8)
        np.testing.assert_array_equal(allocation.epsilons[0], 0.0)
        np.testing.assert_array_equal(allocation.gaussian_quantiles[0], 0.0)
        np.testing.assert_allclose(allocation.epsilons[1:], 0.1 / 8)
        self.assertAlmostEqual(allocation.allocated_epsilon, 0.1)
        self.assertEqual(allocation.budget_status, "BUDGET_OK")

    def test_default_keeps_legacy_full_grid(self):
        from quadrotor_mpc.control.nmpc.risk_budget import RiskBudgetOptions, allocate_risk_budget

        allocation = allocate_risk_budget(
            steps=5,
            obstacle_count=2,
            enabled=True,
            individual_epsilon=0.05,
            options=RiskBudgetOptions(semantics="joint", total_epsilon=0.1),
        )
        self.assertEqual(allocation.active_constraint_count, 10)

    def test_invalid_first_step_is_rejected(self):
        from quadrotor_mpc.control.nmpc.risk_budget import RiskBudgetOptions, allocate_risk_budget

        with self.assertRaisesRegex(ValueError, "first_constrained_step"):
            allocate_risk_budget(
                steps=3,
                obstacle_count=1,
                enabled=True,
                individual_epsilon=0.05,
                options=RiskBudgetOptions(),
                first_constrained_step=3,
            )


if __name__ == "__main__":
    unittest.main()


@unittest.skipUnless(CASADI_AVAILABLE and DO_MPC_AVAILABLE, "requires casadi and do-mpc")
class PostSolveReprojectionTests(unittest.TestCase):
    """Margins are checked against covariance projected on the SOLVED normals."""

    def _solve(self):
        from unittest import mock

        from quadrotor_mpc.control.nmpc.chance_constraints import ChanceConstraintOptions
        from quadrotor_mpc.control.nmpc.core import STATE_NAMES
        from quadrotor_mpc.control.nmpc.covariance import CovariancePropagationOptions
        from quadrotor_mpc.control.nmpc.deterministic import DeterministicNMPCController
        from quadrotor_mpc.control.nmpc.risk_budget import RiskBudgetOptions
        from quadrotor_mpc.core.contracts import (
            ControlGoal,
            ObstacleBelief,
            SphericalObstacle,
            VehicleBelief,
        )

        horizon = 2
        controller = object.__new__(DeterministicNMPCController)
        controller._obstacle_specs = (
            {"type": "static", "x": 2.0, "y": 0.0, "z": 1.0, "radius": 0.2},
        )
        controller._margin = 0.1
        controller._horizon_steps = horizon
        controller._timestep_s = 0.05
        controller._covariance_options = CovariancePropagationOptions(
            enabled=True, mode="open_loop"
        )
        controller._chance_options = ChanceConstraintOptions(
            enabled=True, risk_budget=RiskBudgetOptions(semantics="joint", total_epsilon=0.1)
        )
        # Vehicle position uncertainty is large along y only.
        vehicle_covariance = np.eye(12) * 1e-6
        vehicle_covariance[:3, :3] = np.diag([1e-4, 4e-2, 1e-4])
        controller._covariance_propagator = mock.MagicMock()
        controller._covariance_propagator.propagate.side_effect = lambda b, o, states, c: (
            np.repeat(vehicle_covariance[None], states.shape[0], axis=0),
            np.zeros((states.shape[0], len(o), 6, 6)),
        )
        controller._last_nominal_states = None
        controller._last_nominal_controls = None
        controller._solve_attempt_counter = 0

        # Seed runs along +x through the obstacle's x position (normals ~ x);
        # the "solved" trajectory sits beside it at y = -0.5 (normals ~ y).
        solved = {"x": 2.0, "y": -0.5, "z": 1.0, "qw": 1.0}

        class FakeMpc:
            solver_stats = {"success": True, "iterations": {}}

            def __init__(self):
                self.data = mock.MagicMock()
                self.data.prediction.side_effect = lambda key: np.full(
                    (horizon + 1, 1), solved.get(key[1], 0.0)
                )

            def make_step(self, _state):
                return np.zeros((4, 1))

        controller.mpc = FakeMpc()
        controller.model = mock.MagicMock()
        controller._goal_state = {}
        controller._obstacle_tvp_idx = (0,)
        belief = VehicleBelief(
            mean_state_13=np.array([0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0], dtype=float),
            error_covariance_12=np.eye(12) * 1e-3,
        )
        obstacle = ObstacleBelief(
            mean_state_6=np.array([2.0, 0.0, 1.0, 0, 0, 0]),
            covariance_6=np.zeros((6, 6)),
            shape=SphericalObstacle(0.2),
        )
        goal = ControlGoal(
            position=np.array([3.0, 0.0, 1.0]),
            quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
        )
        self.assertIn("x", STATE_NAMES)
        return controller.solve(belief, [obstacle], goal, 0.0)

    def test_margins_use_solved_normal_projection(self):
        solution = self._solve()
        diagnostic = solution.post_solve_diagnostic_profile
        self.assertIsNotNone(diagnostic)
        enforced = solution.tightened_safety_radii
        # sigma along y (0.2 m) >> sigma along x (0.01 m) on the constrained nodes.
        self.assertTrue(np.all(diagnostic.tightened_radius[1:] > enforced[1:] + 0.1))
        distance = np.sqrt(0.5**2 + 1e-6)
        np.testing.assert_allclose(
            solution.chance_margins[:, 0], distance - diagnostic.tightened_radius[:, 0]
        )
        # The solution clears the enforced (seed-normal) radius but not the
        # radius required at the solution, so the shortfall is reported as slack.
        self.assertTrue(np.all(distance > enforced[1:]))
        self.assertGreater(float(np.max(solution.slacks)), 0.0)
        self.assertEqual(solution.solver_status, "SOLVED_WITH_SLACK")

    def test_certificate_reports_achieved_risk_of_solution(self):
        from scipy.stats import norm

        solution = self._solve()
        diagnostic = solution.post_solve_diagnostic_profile
        base = solution.tightened_safety_radii[0, 0]
        distance = np.sqrt(0.5**2 + 1e-6)
        sigmas = diagnostic.projected_sigma[1:, 0]
        expected = float(np.sum(norm.sf((distance - base) / sigmas)))
        self.assertAlmostEqual(solution.achieved_risk_total, expected, places=9)
        # Each cell exceeds its uniform share 0.05 (positive slack) but the
        # horizon total is compared against the 0.1 joint budget.
        self.assertEqual(
            solution.risk_certificate_status, "CERTIFIED" if expected <= 0.1 else "EXCEEDED"
        )


class CertifyAchievedRiskTests(unittest.TestCase):
    def _options(self, semantics="joint"):
        from quadrotor_mpc.control.nmpc.chance_constraints import ChanceConstraintOptions
        from quadrotor_mpc.control.nmpc.risk_budget import RiskBudgetOptions

        return ChanceConstraintOptions(
            enabled=True,
            individual_epsilon=0.05,
            risk_budget=RiskBudgetOptions(semantics=semantics, total_epsilon=0.1),
        )

    def _certify(self, distances, sigmas, semantics="joint", first=1):
        from quadrotor_mpc.control.nmpc.chance_constraints import certify_achieved_risk

        distances = np.asarray(distances, dtype=float)
        steps = distances.shape[0]
        vehicle = np.column_stack([distances, np.zeros(steps), np.zeros(steps)])
        return certify_achieved_risk(
            vehicle_positions=vehicle,
            obstacle_positions=np.zeros((1, steps, 3)),
            base_safety_radii_m=np.array([0.5]),
            projected_sigmas_m=np.asarray(sigmas, dtype=float).reshape(steps, 1),
            options=self._options(semantics),
            first_constrained_step=first,
            distance_smoothing_m2=0.0,
        )

    def test_joint_sum_against_budget(self):
        from scipy.stats import norm

        # beta*sigma clearance per cell: z = 2.0 -> 0.02275 each, 4 cells -> 0.091
        result = self._certify([0.0, 0.7, 0.7, 0.7, 0.7], [0.1] * 5)
        self.assertAlmostEqual(result.total, 4 * norm.sf(2.0), places=12)
        self.assertEqual(result.status, "CERTIFIED")
        self.assertEqual(float(result.achieved_risks[0, 0]), 0.0)  # node 0 excluded
        tight = self._certify([0.0, 0.65, 0.65, 0.65, 0.65], [0.1] * 5)
        self.assertEqual(tight.status, "EXCEEDED")

    def test_individual_semantics_checks_every_cell(self):
        result = self._certify([0.0, 0.8, 0.62], [0.1] * 3, semantics="individual")
        self.assertGreater(result.maximum, 0.05)
        self.assertEqual(result.status, "EXCEEDED")

    def test_zero_sigma_is_geometric(self):
        safe = self._certify([0.0, 0.6], [0.0, 0.0])
        self.assertEqual(safe.total, 0.0)
        inside = self._certify([0.0, 0.4], [0.0, 0.0])
        self.assertEqual(inside.total, 1.0)
        self.assertEqual(inside.status, "EXCEEDED")
