"""Numerical display interpolation tests; no renderer or source data required."""

import unittest
import numpy as np

from experiments.joint_risk.export_joint_comparison import chance_envelopes, interpolate_state


class JointExportTests(unittest.TestCase):
    def test_envelopes_use_matching_recorded_prediction_node(self):
        radii = np.arange(2*3*2, dtype=float).reshape(2, 3, 2)
        centers = np.arange(2*2*3*3, dtype=float).reshape(2, 2, 3, 3)
        recording = {"tightened_safety_radius_horizons": radii,
                     "estimated_obstacle_predictions": centers}
        actual_centers, actual_radii = chance_envelopes(recording, 1, 2)
        np.testing.assert_array_equal(actual_radii, radii[1, 2])
        np.testing.assert_array_equal(actual_centers, centers[1, :, 2])
        with self.assertRaises(ValueError):
            chance_envelopes(recording, 1, 0)

    def test_positions_and_normalized_quaternions(self):
        states = np.zeros((2, 13))
        states[0, 6] = 1
        states[1, 7] = 1
        states[1, 0] = 2
        actual = interpolate_state(0.5, np.array([0., 1.]), states)
        self.assertAlmostEqual(actual[0], 1.)
        self.assertAlmostEqual(np.linalg.norm(actual[6:10]), 1.)

    def test_antipodal_quaternions_do_not_collapse(self):
        states = np.zeros((2, 13))
        states[:, 6] = [1, -1]
        actual = interpolate_state(0.5, np.array([0., 1.]), states)
        self.assertAlmostEqual(abs(actual[6]), 1.)

    def test_exact_endpoints(self):
        states = np.zeros((2, 13))
        states[:, 6] = 1
        states[1, 0] = 2
        np.testing.assert_array_equal(interpolate_state(0, np.array([0., 1.]), states), states[0])
        np.testing.assert_array_equal(interpolate_state(1, np.array([0., 1.]), states), states[1])


if __name__ == "__main__":
    unittest.main()
