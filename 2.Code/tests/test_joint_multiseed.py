"""Tests of seed-level analysis, using synthetic observations only."""

import unittest

import numpy as np

from experiments.joint_risk.analyze_joint_multiseed import (
    analyze, describe, holm, paired_estimate, wilson,
)


class JointMultiseedAnalysisTests(unittest.TestCase):
    def test_zero_events_does_not_imply_zero_upper_bound(self):
        low, high = wilson(0, 20)
        self.assertAlmostEqual(low, 0.)
        self.assertGreater(high, 0.1)
        self.assertLess(high, 0.2)

    def test_holm_uses_entire_family(self):
        np.testing.assert_allclose(holm([0.01, 0.04, 0.03, 0.001]), [0.03, 0.06, 0.06, 0.004])

    def test_bootstrap_flags_degenerate_data(self):
        result = paired_estimate([0, 0, 0], np.random.default_rng(123))
        self.assertTrue(result["bootstrap_degenerate"])
        self.assertEqual(result["mean_difference_ci95"], [0, 0])
        self.assertIsNone(paired_estimate([2], np.random.default_rng(123))["mean_difference_ci95"])

    def test_nonfinite_values_not_silently_omitted(self):
        with self.assertRaises(ValueError):
            describe([0, np.nan])

    def test_analysis_is_paired_and_excludes_incomplete_paths(self):
        rows = []
        for scene in ("scene_a", "scene_b"):
            for seed in (1, 2, 3):
                for variant in ("uniform", "hard_uniform", "joint_hard"):
                    success = seed == 1 or (seed == 2 and variant == "joint_hard")
                    rows.append({
                        "scenario": scene, "seed": seed, "variant": variant,
                        "status": "completed", "success": success, "collision": False,
                        "fresh_seed": True, "samples": 130, "fallback": 0,
                        "goal_distance_m": 0.1 if success else 1.5,
                        "min_clearance_m": 0.15 if variant == "joint_hard" else 0.2,
                        "solver_p95_ms": 100 if variant == "joint_hard" else 50,
                        "total_controller_p95_ms": 105 if variant == "joint_hard" else 55,
                        "max_slack_m": 0, "path_length_m": 3 if success else 1,
                        "time_to_tolerance_s": 4 if success else None,
                    })
        plan = {"scene_seeds": {"scene_a": [1, 2, 3], "scene_b": [1, 2, 3]}}
        result = analyze(plan, rows)
        for comparison in result["comparisons"]:
            self.assertEqual(comparison["paired_n"], 3)  # not 390 ticks
            self.assertEqual(comparison["joint_only_success"], 1)
            self.assertEqual(comparison["comparator_only_success"], 0)
            self.assertEqual(comparison["both_success"], 1)
            self.assertEqual(comparison["path_length_m_both_success_only"]["n"], 1)
            self.assertAlmostEqual(comparison["min_clearance_m"]["mean"], -0.05)
            self.assertEqual(comparison["mcnemar_holm_p"], 1.)


if __name__ == "__main__":
    unittest.main()
