"""Run with .venv/Scripts/python.exe -m unittest discover -s tests -p test_joint_risk.py."""

import unittest

import casadi as ca
import numpy as np
from scipy.special import ndtri
from scipy.stats import norm

from simulation.control.ccmpc_13state.chance_constraints import (
    ChanceConstraintOptions, build_spherical_chance_profile,
)
from simulation.control.ccmpc_13state.core import (
    build_controller, build_model,
    required_risk_probability,
)
from simulation.control.ccmpc_13state.covariance import CovariancePropagationOptions
from simulation.control.ccmpc_13state.experimental_joint_risk import (
    JointRiskNMPCController, JointRiskOptions,
)
from simulation.control.ccmpc_13state.experimental_joint_risk.profile import allocation_profile
from simulation.control.ccmpc_13state.risk_budget import RiskBudgetOptions
from simulation.core.contracts import (
    ControlGoal, ObstacleBelief, SphericalObstacle, VehicleBelief,
)


class JointRiskTests(unittest.TestCase):
    def test_real_solve_budget_and_profile_alignment(self):
        state = np.zeros(13)
        state[2], state[6] = 1.2, 1.0
        belief = VehicleBelief(state, np.eye(12) * 1e-4)
        obstacle = ObstacleBelief(
            np.array([2., 1., 1.2, 0., 0., 0.]), np.eye(6) * 1e-4,
            SphericalObstacle(0.1), "test_obstacle",
        )
        goal = ControlGoal(np.array([0.5, 0., 1.2]), np.array([1., 0., 0., 0.]))
        controller = JointRiskNMPCController(
            joint_options=JointRiskOptions(),
            bounds={"thrust": 0.085, "torque_rp": 0.002, "torque_yaw": 0.0004},
            obstacle_specs=[{"radius": 0.1, "type": "static", "x": 2., "y": 1., "z": 1.2}],
            margin=0.05, horizon_steps=3, timestep_s=0.05,
            chance_options=ChanceConstraintOptions(
                enabled=True, soft_constraint=False,
                risk_budget=RiskBudgetOptions(semantics="joint"),
            ),
            covariance_options=CovariancePropagationOptions(enabled=True),
        )
        controller.reset(belief)
        for tick in range(2):
            solution = controller.solve(belief, [obstacle], goal, tick * 0.05)
            self.assertTrue(solution.primary_solver_success)
            self.assertEqual(solution.command.shape, (4,))
            self.assertEqual(solution.nominal_states.shape, (4, 13))
            self.assertEqual(solution.risk_allocations[0, 0], 0)
            self.assertLessEqual(solution.risk_allocations.sum(), 0.1 + 1e-8)
            np.testing.assert_array_equal(
                solution.enforced_chance_profile.epsilon,
                solution.post_solve_diagnostic_profile.epsilon,
            )
            beta = -ndtri(solution.risk_allocations[1:])
            np.testing.assert_allclose(
                solution.tightened_safety_radii[1:],
                0.22 + beta * solution.projected_uncertainties[1:], atol=1e-12,
            )
            terminal_budget = float(
                controller.mpc.opt_x_num_unscaled["_x", 3, 0, -1, "risk_budget_used"]
            )
            self.assertAlmostEqual(terminal_budget, solution.risk_allocations.sum(), places=9)
            self.assertAlmostEqual(
                float(controller.mpc.opt_p_num["_x0", "risk_budget_used"]), 0., places=12,
            )

    def test_default_formulation_is_required(self):
        self.assertEqual(JointRiskOptions().formulation, "required")

    def test_invalid_options(self):
        for values in (
            {"delta_min": 0}, {"delta_max": 0.5}, {"delta_min": 0.3},
            {"total_epsilon": float("nan")}, {"feasibility_tolerance": -1},
        ):
            with self.subTest(values=values), self.assertRaises(ValueError):
                JointRiskOptions(**values)

    def test_removed_formulations_are_rejected(self):
        obstacles = [{"radius": 0.1}]
        model, indices = build_model(0.05, obstacles, joint_risk=True)
        for formulation in ("quantile", "aggregate"):
            with self.subTest(formulation=formulation):
                with self.assertRaisesRegex(ValueError, "required"):
                    JointRiskOptions(formulation=formulation)
                with self.assertRaisesRegex(ValueError, "required"):
                    build_model(
                        0.05, obstacles, joint_risk=True,
                        joint_formulation=formulation,
                    )
                with self.assertRaisesRegex(ValueError, "required"):
                    build_controller(
                        model, obstacles, indices, {}, 0.05, 3, 0.05,
                        joint_risk=True, joint_formulation=formulation,
                    )

    def test_required_probability_derivative(self):
        distance = ca.SX.sym("distance")
        risk = required_risk_probability(distance, 0.2, 0.05)
        function = ca.Function(
            "required_derivative", [distance], [ca.jacobian(risk, distance)],
        )
        for value in np.linspace(0.2, 0.4, 20):
            expected = -norm.pdf((value - 0.2) / 0.05) / 0.05
            self.assertAlmostEqual(float(function(value)), expected, places=9)

    def test_budget_includes_last_successor(self):
        obstacles = [{"radius": 0.1}, {"radius": 0.1}]
        model, indices = build_model(0.05, obstacles, joint_risk=True)
        mpc = build_controller(
            model, obstacles, indices,
            {"thrust": 0.085, "torque_rp": 0.002, "torque_yaw": 0.0004},
            0.05, 3, 0.05, joint_risk=True,
        )
        budget = next(c for c in mpc.nl_cons_list if c["expr_name"] == "joint_risk_budget")
        function = ca.Function("budget_test", [model.x, model.u], [budget["expr"]])
        x = model.x(0)
        u = model.u(0)
        x["risk_budget_used"] = 0.09
        u["delta_0"] = 0.006
        u["delta_1"] = 0.006
        self.assertGreater(float(function(x, u)), budget["ub"])
        self.assertNotIn("joint_risk_budget", [s["slack_name"] for s in mpc.slack_vars_list])

    def test_default_model_has_no_experimental_variables(self):
        model, _ = build_model(0.05, [{"radius": 0.1}])
        self.assertEqual(model.n_x, 13)
        self.assertEqual(model.n_u, 4)

    def test_infeasible_minimum_budget_rejected_before_build(self):
        with self.assertRaisesRegex(ValueError, "infeasible"):
            JointRiskNMPCController(
                joint_options=JointRiskOptions(delta_min=0.02),
                bounds={}, obstacle_specs=[{"radius": 0.1}], margin=0.05,
                horizon_steps=20, timestep_s=0.05,
                chance_options=ChanceConstraintOptions(
                    enabled=True, risk_budget=RiskBudgetOptions(semantics="joint"),
                ),
                covariance_options=CovariancePropagationOptions(enabled=True),
            )

    def test_profile_preserves_sigma_and_does_not_repair_overspending(self):
        options = ChanceConstraintOptions(
            enabled=True, risk_budget=RiskBudgetOptions(semantics="joint"),
        )
        profile = build_spherical_chance_profile(
            vehicle_positions=np.zeros((3, 3)),
            obstacle_positions=np.ones((1, 3, 3)),
            vehicle_covariances=np.repeat(np.eye(12)[None] * 0.01, 3, axis=0),
            obstacle_covariances=np.zeros((3, 1, 6, 6)),
            base_safety_radii_m=np.array([0.2]), options=options,
            first_constrained_step=1,
        )
        delta = np.array([[0], [0.02], [0.09]])
        actual, valid = allocation_profile(profile, delta, JointRiskOptions())
        self.assertFalse(valid)
        np.testing.assert_array_equal(actual.risk_allocations, delta)
        np.testing.assert_array_equal(actual.projected_sigmas_m, profile.projected_sigmas_m)
        self.assertEqual(actual.budget_status, "JOINT_ALLOCATION_INVALID")
        self.assertAlmostEqual(actual.allocated_epsilon, 0.11)
        valid_profile, valid = allocation_profile(profile, delta * 0.5, JointRiskOptions())
        self.assertTrue(valid)
        np.testing.assert_allclose(
            valid_profile.safety_radii_m[1:],
            0.2 - ndtri(delta[1:] * 0.5) * profile.projected_sigmas_m[1:],
        )

    def test_required_probability_is_the_chance_constraint(self):
        distance = ca.SX.sym("distance")
        base = ca.SX.sym("base")
        sigma = ca.SX.sym("sigma")
        risk = required_risk_probability(distance, base, sigma)
        function = ca.Function("required_test", [distance, base, sigma], [risk])
        for d, r, s in (
            (0.30, 0.20, 0.03),
            (0.25, 0.20, 0.05),
            (0.40, 0.20, 0.01),
        ):
            for delta in (1e-4, 0.001, 0.02, 0.1):
                actual = float(function(d, r, s))
                self.assertAlmostEqual(actual, float(norm.sf((d - r) / s)), places=9)
                lhs = actual <= delta
                rhs = d >= r + ndtri(1.0 - delta) * s
                self.assertEqual(lhs, rhs)

    def test_required_model_keeps_delta_controls(self):
        obstacles = [{"radius": 0.1}, {"radius": 0.1}]
        model, _ = build_model(
            0.05, obstacles, joint_risk=True, joint_formulation="required",
        )
        self.assertEqual(model.n_u, 6)
        self.assertEqual(model.n_x, 14)
        self.assertIn("risk_budget_used", model.x.keys())
        self.assertIn("delta_0", model.u.keys())
        self.assertIn("delta_1", model.u.keys())

    def test_required_real_solve(self):
        state = np.zeros(13)
        state[2], state[6] = 1.2, 1.0
        belief = VehicleBelief(state, np.eye(12) * 1e-4)
        obstacle = ObstacleBelief(
            np.array([2., 1., 1.2, 0., 0., 0.]), np.eye(6) * 1e-4,
            SphericalObstacle(0.1), "test_obstacle",
        )
        goal = ControlGoal(np.array([0.5, 0., 1.2]), np.array([1., 0., 0., 0.]))
        for formulation in ("required",):
            with self.subTest(formulation=formulation):
                controller = JointRiskNMPCController(
                    joint_options=JointRiskOptions(formulation=formulation),
                    bounds={"thrust": 0.085, "torque_rp": 0.002, "torque_yaw": 0.0004},
                    obstacle_specs=[{"radius": 0.1, "type": "static", "x": 2., "y": 1., "z": 1.2}],
                    margin=0.05, horizon_steps=3, timestep_s=0.05,
                    chance_options=ChanceConstraintOptions(
                        enabled=True, soft_constraint=False,
                        risk_budget=RiskBudgetOptions(semantics="joint"),
                    ),
                    covariance_options=CovariancePropagationOptions(enabled=True),
                )
                controller.reset(belief)
                solution = controller.solve(belief, [obstacle], goal, 0.0)
                self.assertTrue(solution.primary_solver_success)
                self.assertEqual(solution.command.shape, (4,))
                self.assertEqual(solution.risk_allocations.shape, (4, 1))
                self.assertLessEqual(solution.risk_allocations.sum(), 0.1 + 1e-8)
                self.assertEqual(solution.risk_allocations[0, 0], 0.)
                terminal_budget = float(
                    controller.mpc.opt_x_num_unscaled["_x", 3, 0, -1, "risk_budget_used"]
                )
                self.assertAlmostEqual(terminal_budget, solution.risk_allocations.sum(), places=8)


if __name__ == "__main__":
    unittest.main()
