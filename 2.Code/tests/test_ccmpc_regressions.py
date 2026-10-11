"""Regression tests for the 9-state CVXPY CC-MPC (``control/ccmpc/ccmpc.py``).

Locks three fixes:

1. Obstacle predictions are time-aligned with the constrained vehicle state:
   horizon step ``k`` constrains ``x[:3, k+1]`` (time ``(k+1)*dt``), so the
   obstacle mean/covariance used there must also be at ``(k+1)*dt``.
2. A QP failure restarts the iMPC loop from iteration 0 (with the covariance
   re-propagated along the new guess) and never returns an unsolved seed
   trajectory; exhausting restarts yields the emergency-hover rollout.
3. Covariance propagation uses the controller's configured timestep.
"""

from __future__ import annotations

import unittest
from unittest import mock

import numpy as np

from quadrotor_mpc.control.ccmpc.ccmpc import CCMPC
from quadrotor_mpc.control.ccmpc.obstacle import EllipsoidalObstacle, ObstacleManager

CONFIG = "config/controller.yaml"
GOAL = np.array([6.0, 0.0, 1.5])


def _initial_state() -> np.ndarray:
    state = np.zeros(9)
    state[2] = 1.5
    return state


def _moving_obstacle() -> ObstacleManager:
    return ObstacleManager(
        [EllipsoidalObstacle([3.0, -1.0, 1.5], [0.4, 0.4, 0.8], velocity=[0.0, 1.0, 0.0])]
    )


class ObstacleTimeAlignmentTests(unittest.TestCase):
    def test_prediction_for_step_k_is_at_time_k_plus_one(self):
        controller = CCMPC(CONFIG)
        manager = _moving_obstacle()
        obstacle = manager.obstacles[0]
        N = controller.control_horizon
        dt = controller.dt

        x_guess = np.tile(_initial_state(), (N + 1, 1)).T
        predictions = controller._predict_obstacles(manager, N, x_guess)

        self.assertEqual(len(predictions), N)
        for k in (0, 1, N // 2, N - 1):
            elapsed = (k + 1) * dt
            np.testing.assert_allclose(
                predictions[k][0]["pos"], obstacle.p_hat + obstacle.v_hat * elapsed
            )
            np.testing.assert_allclose(
                predictions[k][0]["Sigma"],
                obstacle.Sigma + (k + 1) * obstacle.Sigma_v * dt**2,
            )

    def test_chance_constraint_receives_aligned_obstacle(self):
        controller = CCMPC(CONFIG)
        manager = _moving_obstacle()
        obstacle = manager.obstacles[0]
        captured: dict[int, np.ndarray] = {}
        original = controller._set_chance_constraint_params

        def spy(k, i_obs, p_mav, p_obs, *args):
            if i_obs == 0:
                captured[k] = np.array(p_obs, dtype=float)
            return original(k, i_obs, p_mav, p_obs, *args)

        with mock.patch.object(controller, "_set_chance_constraint_params", side_effect=spy):
            controller.solve(_initial_state(), GOAL, obstacle_manager=manager)

        self.assertIn(0, captured)
        np.testing.assert_allclose(captured[0], obstacle.p_hat + obstacle.v_hat * controller.dt)


class ObstacleSelectionTests(unittest.TestCase):
    def test_nearest_obstacles_are_constrained_not_list_order(self):
        controller = CCMPC(CONFIG)
        self.assertEqual(controller._max_obs, 2)
        N = controller.control_horizon
        x_guess = np.tile(_initial_state(), (N + 1, 1)).T
        far_a = EllipsoidalObstacle([20.0, 0.0, 1.5], [0.4, 0.4, 0.8])
        far_b = EllipsoidalObstacle([0.0, 20.0, 1.5], [0.4, 0.4, 0.8])
        near = EllipsoidalObstacle([1.0, 0.0, 1.5], [0.4, 0.4, 0.8])
        manager = ObstacleManager([far_a, far_b, near])

        selected = controller._select_obstacles(manager, x_guess)

        self.assertEqual(len(selected), 2)
        self.assertIs(selected[0], near)
        predictions = controller._predict_obstacles(manager, N, x_guess)
        np.testing.assert_allclose(predictions[0][0]["pos"], near.p_hat)

    def test_ranking_uses_time_aligned_closest_approach(self):
        controller = CCMPC(CONFIG)
        N = controller.control_horizon
        horizon = N * controller.dt
        x_guess = np.tile(_initial_state(), (N + 1, 1)).T
        # Starts far but crosses the vehicle position mid-horizon.
        crossing = EllipsoidalObstacle(
            [0.0, 10.0, 1.5], [0.4, 0.4, 0.8], velocity=[0.0, -20.0 / horizon, 0.0]
        )
        static_a = EllipsoidalObstacle([4.0, 0.0, 1.5], [0.4, 0.4, 0.8])
        static_b = EllipsoidalObstacle([5.0, 0.0, 1.5], [0.4, 0.4, 0.8])
        manager = ObstacleManager([static_a, static_b, crossing])

        selected = controller._select_obstacles(manager, x_guess)

        self.assertIn(crossing, selected)
        self.assertIn(static_a, selected)
        self.assertNotIn(static_b, selected)


class SolverFailureRestartTests(unittest.TestCase):
    def _solve_with_failures(self, controller: CCMPC, fail_calls: set[int]):
        """Run ``solve`` with the QP forced to fail on the given call indices."""
        real_solve = controller._problem.solve
        calls = {"n": 0}

        def flaky_solve(*args, **kwargs):
            index = calls["n"]
            calls["n"] += 1
            result = real_solve(*args, **kwargs)
            if index in fail_calls:
                controller._states.value = None
                controller._controls.value = None
            return result

        with (
            mock.patch.object(controller._problem, "solve", side_effect=flaky_solve),
            mock.patch.object(controller, "_tol", -1.0),  # never converge early
        ):
            result = controller.solve(_initial_state(), GOAL)
        return result, calls["n"]

    def test_failure_on_last_iteration_restarts_instead_of_returning_seed(self):
        controller = CCMPC(CONFIG)
        max_iter = controller._max_iter
        (x_traj, u_traj), n_calls = self._solve_with_failures(controller, {max_iter - 1})

        # One full restart: max_iter calls, then max_iter more after reset.
        self.assertEqual(n_calls, 2 * max_iter)
        # The returned controls come from a solved QP, not the all-zero seed.
        self.assertGreater(float(np.max(np.abs(u_traj))), 0.0)
        np.testing.assert_allclose(x_traj[:, 0], _initial_state(), atol=1e-6)

    def test_restart_repropagates_covariance_along_new_guess(self):
        controller = CCMPC(CONFIG)
        with mock.patch.object(
            controller.uncertainty,
            "propagate",
            wraps=controller.uncertainty.propagate,
        ) as propagate:
            self._solve_with_failures(controller, {0})
        self.assertEqual(propagate.call_count, 2)

    def test_persistent_failure_returns_emergency_hover(self):
        controller = CCMPC(CONFIG)
        expected = controller._emergency_hover(_initial_state(), GOAL)
        (x_traj, u_traj), _ = self._solve_with_failures(controller, set(range(1000)))
        np.testing.assert_allclose(x_traj, expected[0])
        np.testing.assert_allclose(u_traj, expected[1])

    def test_solver_exception_is_treated_as_failure(self):
        controller = CCMPC(CONFIG)
        expected = controller._emergency_hover(_initial_state(), GOAL)
        with mock.patch.object(controller._problem, "solve", side_effect=RuntimeError("boom")):
            x_traj, u_traj = controller.solve(_initial_state(), GOAL)
        np.testing.assert_allclose(x_traj, expected[0])
        np.testing.assert_allclose(u_traj, expected[1])


class CovarianceTimestepTests(unittest.TestCase):
    def test_propagation_uses_configured_timestep(self):
        controller = CCMPC(CONFIG, timestep=0.1)
        self.assertNotAlmostEqual(controller.dt, 0.06)
        with mock.patch.object(
            controller.uncertainty,
            "propagate",
            wraps=controller.uncertainty.propagate,
        ) as propagate:
            controller.solve(_initial_state(), GOAL)
        self.assertEqual(propagate.call_args.kwargs.get("dt"), controller.dt)


if __name__ == "__main__":
    unittest.main()
