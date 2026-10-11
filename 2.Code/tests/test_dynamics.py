from __future__ import annotations

import unittest

import numpy as np

from quadrotor_mpc.control.ccmpc.dynamics import QuadrotorDynamics, continuous_dynamics


class DynamicsTests(unittest.TestCase):
    def test_hover_is_equilibrium(self) -> None:
        derivative = continuous_dynamics(np.zeros(9), np.zeros(4))
        np.testing.assert_allclose(derivative, np.zeros(9), atol=1e-12)

    def test_positive_pitch_accelerates_forward(self) -> None:
        state = np.zeros(9)
        state[7] = 0.10
        derivative = continuous_dynamics(state, np.zeros(4))
        self.assertGreater(derivative[3], 0.0)
        self.assertAlmostEqual(derivative[4], 0.0, places=12)

    def test_affine_linearization_matches_expansion_point(self) -> None:
        dynamics = QuadrotorDynamics(k_vz=3.0)
        state = np.array([1.0, -0.2, 1.5, 0.3, -0.1, 0.2, 0.04, -0.08, 0.3])
        control = np.array([0.05, -0.06, 0.4, 0.2])
        A, B, C = dynamics.linearize(state, control, 0.06)
        predicted = A @ state + B @ control + C
        actual = dynamics.discrete(state, control, 0.06)
        np.testing.assert_allclose(predicted, actual, atol=1e-10)

    def test_state_jacobian_matches_finite_differences(self) -> None:
        dynamics = QuadrotorDynamics(k_vz=3.0)
        control = np.array([0.05, -0.06, 0.4, 0.2])
        for state in (
            np.zeros(9),
            np.array([1.0, -0.2, 1.5, 0.3, -0.1, 0.2, 0.2, 0.1, 0.7]),
        ):
            analytic = dynamics.jacobian_state(state, control)
            numeric = np.empty((9, 9))
            step = 1e-6
            for column in range(9):
                delta = np.zeros(9)
                delta[column] = step
                numeric[:, column] = (
                    continuous_dynamics(state + delta, control, k_vz=3.0)
                    - continuous_dynamics(state - delta, control, k_vz=3.0)
                ) / (2.0 * step)
            np.testing.assert_allclose(analytic, numeric, atol=1e-5)

    def test_level_attitude_coupling_signs_match_theory_chapter_14(self) -> None:
        dynamics = QuadrotorDynamics()
        for psi in (0.0, 0.7):
            state = np.zeros(9)
            state[8] = psi
            jacobian = dynamics.jacobian_state(state, np.zeros(4))
            g = 9.81
            self.assertAlmostEqual(jacobian[3, 7], g * np.cos(psi), places=6)
            self.assertAlmostEqual(jacobian[3, 6], -g * np.sin(psi), places=6)
            self.assertAlmostEqual(jacobian[4, 7], g * np.sin(psi), places=6)
            self.assertAlmostEqual(jacobian[4, 6], g * np.cos(psi), places=6)


if __name__ == "__main__":
    unittest.main()
