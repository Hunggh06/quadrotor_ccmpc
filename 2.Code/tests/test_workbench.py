"""Workbench: catalog helpers, headless runs, job parsing and the Qt window."""

from __future__ import annotations

import csv
import importlib.util
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np
import yaml

CODE_ROOT = Path(__file__).resolve().parents[1]
QT_AVAILABLE = (
    importlib.util.find_spec("PySide6") is not None
    and importlib.util.find_spec("pyqtgraph") is not None
)
NATIVE_AVAILABLE = all(
    importlib.util.find_spec(name) is not None for name in ("mujoco", "casadi", "do_mpc")
)


def write_native_fixture(root: Path, name: str = "20261011T120000Z_fixture-joint") -> Path:
    run = root / "outputs" / "native" / name
    run.mkdir(parents=True)
    scenario = {
        "name": "fixture",
        "start": {"x": 0.0, "y": 0.0, "z": 1.0},
        "goal": {"position": {"x": 3.0, "y": 2.0, "z": 1.0}},
        "simulation": {"mpc_timestep_s": 0.05},
        "controller": {
            "chance_constraints": {
                "enabled": True,
                "risk_budget": {"semantics": "joint", "total_epsilon": 0.1},
            }
        },
        "obstacles": [
            {"name": "gate", "type": "static", "x": 1.5, "y": 1.0, "z": 1.0, "radius": 0.3}
        ],
    }
    (run / "scenario.yaml").write_text(yaml.safe_dump(scenario), encoding="utf-8")
    summary = {
        "scenario": "fixture",
        "created_utc": "2026-10-11T12:00:00+00:00",
        "risk_semantics": "joint",
        "collision": False,
        "termination_reason": "duration_reached",
        "min_clearance_m": 0.4,
        "maximum_slack_m": 0.002,
        "fallback_activations": 1,
        "samples": 3,
    }
    (run / "summary.yaml").write_text(yaml.safe_dump(summary), encoding="utf-8")
    fields = [
        "time_s",
        "px",
        "py",
        "pz",
        "min_clearance_m",
        "goal_distance_m",
        "solver_time_ms",
        "maximum_slack_m",
        "achieved_risk_total",
        "fallback_active",
    ]
    with (run / "telemetry.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index in range(3):
            writer.writerow(
                {
                    "time_s": 0.05 * (index + 1),
                    "px": 0.1 * index,
                    "py": 0.05 * index,
                    "pz": 1.0,
                    "min_clearance_m": 0.6 - 0.1 * index,
                    "goal_distance_m": 3.0 - index,
                    "solver_time_ms": 40.0 + index,
                    "maximum_slack_m": 0.0,
                    "achieved_risk_total": "" if index == 0 else 0.01 * index,
                    "fallback_active": "True" if index == 2 else "False",
                }
            )
    paths = np.zeros((3, 1, 21, 3))
    paths[:, 0, 0, :] = [1.5, 1.0, 1.0]
    np.savez_compressed(run / "trajectory.npz", obstacle_predictions=paths)
    return run


def write_campaign_fixture(root: Path) -> Path:
    from test_native_monte_carlo import protocol, trial  # type: ignore[import-not-found]

    from quadrotor_mpc.application.validation.monte_carlo import aggregate_native_trials

    campaign = root / "outputs" / "native_monte_carlo" / "20261011T120000Z-fixture"
    campaign.mkdir(parents=True)
    items = [trial("deterministic", seed) for seed in range(10, 40)] + [
        trial("joint_uniform", seed) for seed in range(10, 40)
    ]
    aggregate = aggregate_native_trials(items, protocol(), controller_period_ms=50.0)
    (campaign / "aggregate.json").write_text(json.dumps(aggregate), encoding="utf-8")
    (campaign / "manifest.yaml").write_text(
        yaml.safe_dump(
            {"status": "COMPLETED", "expected_trials": 60, "created_at_utc": "2026-10-11T12:00:00"}
        ),
        encoding="utf-8",
    )
    (campaign / "protocol.yaml").write_text(
        yaml.safe_dump(protocol().to_mapping()), encoding="utf-8"
    )
    with (campaign / "trials.jsonl").open("w", encoding="utf-8") as stream:
        for item in items:
            stream.write(json.dumps(item.to_mapping()) + "\n")
    return campaign


class CatalogTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        sys.path.insert(0, str(CODE_ROOT / "tests"))

    def tearDown(self):
        sys.path.remove(str(CODE_ROOT / "tests"))
        self._tmp.cleanup()

    def test_native_run_listing_and_timeseries(self):
        from quadrotor_mpc.application.catalog import list_native_runs, load_native_timeseries

        run = write_native_fixture(self.root)
        runs = list_native_runs([self.root])
        self.assertEqual([item.path for item in runs], [run])
        self.assertEqual(runs[0].fallback_activations, 1)
        series = load_native_timeseries(run)
        np.testing.assert_allclose(series.time_s, [0.05, 0.10, 0.15])
        self.assertTrue(np.isnan(series.achieved_risk_total[0]))
        np.testing.assert_allclose(series.fallback_active, [0.0, 0.0, 1.0])
        self.assertEqual(series.risk_budget_total, 0.1)
        np.testing.assert_allclose(series.goal, [3.0, 2.0, 1.0])
        self.assertEqual(series.obstacle_paths.shape, (3, 1, 3))
        self.assertAlmostEqual(series.control_period_ms, 50.0)

    def test_campaign_listing_cells_and_resumability(self):
        from quadrotor_mpc.application.catalog import list_campaigns, load_campaign_cells

        campaign = write_campaign_fixture(self.root)
        campaigns = list_campaigns([self.root])
        self.assertEqual(len(campaigns), 1)
        self.assertEqual(campaigns[0].completed_trials, 60)
        self.assertFalse(campaigns[0].resumable)
        _aggregate, cells = load_campaign_cells(campaign)
        self.assertEqual({cell.controller for cell in cells}, {"deterministic", "joint_uniform"})
        joint = next(cell for cell in cells if cell.controller == "joint_uniform")
        self.assertAlmostEqual(joint.collision_upper_bound, 0.0950, places=4)

        manifest = yaml.safe_load((campaign / "manifest.yaml").read_text(encoding="utf-8"))
        manifest.update(status="RUNNING", expected_trials=90)
        (campaign / "manifest.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")
        self.assertTrue(list_campaigns([self.root])[0].resumable)


class HeadlessRuntimeTests(unittest.TestCase):
    def test_progress_and_result_lines_are_json(self):
        from quadrotor_mpc.application.native.headless import (
            PROGRESS_PREFIX,
            RESULT_PREFIX,
            progress_line,
            result_line,
        )

        line = progress_line(
            {"time_s": 0.1, "position": np.array([1.0, 2.0, 3.0]), "achieved_risk_total": None},
            200,
            2,
        )
        payload = json.loads(line[len(PROGRESS_PREFIX) :])
        self.assertEqual(payload["position"], [1.0, 2.0, 3.0])
        self.assertEqual(payload["total_steps"], 200)
        result = json.loads(
            result_line({"termination_reason": "goal_reached"}, None)[len(RESULT_PREFIX) :]
        )
        self.assertEqual(result["termination_reason"], "goal_reached")

    def test_mode_override_matches_monte_carlo_trial_config(self):
        from quadrotor_mpc.application.native.headless import apply_run_overrides
        from quadrotor_mpc.interfaces.desktop.viewer import load_native_mujoco_config

        base = load_native_mujoco_config(CODE_ROOT / "config" / "mujoco_native_ccmpc.yaml")
        deterministic = apply_run_overrides(base, mode="deterministic", seed=3, duration_s=2.0)
        self.assertFalse(deterministic.chance_constraints.enabled)
        self.assertEqual(deterministic.estimation.seed, 3)
        self.assertEqual(deterministic.duration_s, 2.0)
        individual = apply_run_overrides(base, mode="individual", noise_scale=4.0)
        self.assertEqual(individual.chance_constraints.risk_budget.semantics, "individual")
        self.assertEqual(individual.name, f"{base.name}-individual")

    @unittest.skipUnless(NATIVE_AVAILABLE, "native MuJoCo/NMPC dependencies are not installed")
    def test_headless_cli_records_and_reports_progress(self):
        import contextlib
        import io

        from quadrotor_mpc.interfaces.cli.native import main

        with tempfile.TemporaryDirectory() as directory:
            previous = Path.cwd()
            os.chdir(directory)
            stream = io.StringIO()
            try:
                with contextlib.redirect_stdout(stream):
                    code = main(
                        [
                            "--config",
                            str(CODE_ROOT / "config" / "mujoco_native.yaml"),
                            "--headless",
                            "--progress-json",
                            "--duration",
                            "0.15",
                        ]
                    )
            finally:
                os.chdir(previous)
            self.assertEqual(code, 0)
            lines = stream.getvalue().splitlines()
            progress = [line for line in lines if line.startswith("@@progress ")]
            result = [line for line in lines if line.startswith("@@result ")]
            self.assertGreaterEqual(len(progress), 2)
            payload = json.loads(result[-1][len("@@result ") :])
            self.assertEqual(payload["termination_reason"], "duration_reached")
            self.assertTrue((Path(payload["recording"]) / "telemetry.csv").is_file())


@unittest.skipUnless(QT_AVAILABLE, "PySide6/pyqtgraph are not installed")
class WorkbenchWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
        from quadrotor_mpc.interfaces.desktop.workbench.app import create_application

        cls.app = create_application(["test"])

    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.root = Path(self._tmp.name)
        sys.path.insert(0, str(CODE_ROOT / "tests"))

    def tearDown(self):
        sys.path.remove(str(CODE_ROOT / "tests"))
        self._tmp.cleanup()

    def _window(self):
        from quadrotor_mpc.interfaces.desktop.workbench.app import WorkbenchWindow

        window = WorkbenchWindow(
            CODE_ROOT, data_roots=[self.root], work_dir=self.root / "workbench"
        )
        window.resize(1400, 900)
        window.show()
        self.app.processEvents()
        return window

    def test_pages_navigation_and_simulation_arguments(self):
        window = self._window()
        self.assertEqual(window.stack.count(), 3)
        page = window.simulation
        self.assertGreaterEqual(page.scenario.count(), 4)
        self.assertEqual(page.current_choice().path.name, "mujoco_native_ccmpc.yaml")
        page.controller.set_value("individual")
        page.seed.setValue(42)
        arguments = page.build_arguments(headless=True)
        self.assertIn("--headless", arguments)
        self.assertEqual(arguments[arguments.index("--mode") + 1], "individual")
        self.assertEqual(arguments[arguments.index("--seed") + 1], "42")
        self.assertIn("--noise-scale", arguments)
        window.nav.setCurrentRow(2)
        self.assertIs(window.stack.currentWidget(), window.results)
        window.close()

    def test_monte_carlo_protocol_keeps_shipped_statistics(self):
        window = self._window()
        page = window.monte_carlo
        page.trials.setValue(7)
        page.mode_boxes["individual"].setChecked(False)
        page.noise_boxes["four_sigma"].setChecked(False)
        self.assertEqual(page.planned_trials(), 7 * 2 * 2)
        path = page.write_protocol()
        self.assertTrue(path.is_relative_to(self.root))
        mapping = yaml.safe_load(path.read_text(encoding="utf-8"))
        self.assertEqual(mapping["trials"], 7)
        self.assertEqual(mapping["modes"], ["deterministic", "joint_uniform"])
        self.assertEqual(
            mapping["statistics"]["collision_bound_method"], "clopper_pearson_one_sided"
        )
        self.assertEqual(mapping["validation"]["chance_claim_rule"], "risk_certificate")
        from quadrotor_mpc.application.validation.monte_carlo import (
            load_native_monte_carlo_protocol,
        )

        protocol = load_native_monte_carlo_protocol(path)
        self.assertEqual(protocol.trials, 7)
        window.close()

    def test_results_page_shows_native_runs_and_campaigns(self):
        write_native_fixture(self.root)
        write_native_fixture(self.root, "20261011T130000Z_fixture-det")
        campaign = write_campaign_fixture(self.root)
        window = self._window()
        results = window.results
        results.refresh()
        self.assertEqual(results.native_table.rowCount(), 2)
        self.assertEqual(results.campaign_table.rowCount(), 1)
        results.native_table.selectRow(0)
        self.app.processEvents()
        self.assertIs(results.detail.currentWidget(), results.native_view)
        self.assertEqual(results.native_view.tiles["fallback"].value_text, "1")
        results.native_table.selectAll()
        self.app.processEvents()
        self.assertIn("Comparing 2", results.native_view.title.text())
        results.show_path(str(campaign))
        self.app.processEvents()
        self.assertIs(results.detail.currentWidget(), results.campaign_view)
        self.assertEqual(results.campaign_view.cells.rowCount(), 2)
        window.close()

    def test_process_job_parses_progress_trials_and_result(self):
        from PySide6.QtCore import QEventLoop, QTimer

        from quadrotor_mpc.interfaces.desktop.workbench.jobs import ProcessJob

        script = (
            "print('output:         /tmp/x');"
            'print(\'@@progress {"step": 1, "total_steps": 2}\');'
            "print('[001/002] nominal_sigma joint_uniform seed=1 collision=0 p99=12.0ms');"
            "print('@@result {\"recording\": null}')"
        )
        job = ProcessJob([sys.executable, "-c", script], self.root)
        seen: dict[str, list] = {"progress": [], "trial": [], "result": [], "output": []}
        job.progress.connect(seen["progress"].append)
        job.trial.connect(lambda *args: seen["trial"].append(args))
        job.result.connect(seen["result"].append)
        job.output_dir.connect(seen["output"].append)
        loop = QEventLoop()
        job.finished.connect(lambda *_: loop.quit())
        QTimer.singleShot(15000, loop.quit)
        job.start()
        loop.exec()
        self.assertEqual(seen["progress"], [{"step": 1, "total_steps": 2}])
        self.assertEqual(seen["trial"][0][:2], (1, 2))
        self.assertEqual(seen["result"], [{"recording": None}])
        self.assertEqual(seen["output"], ["/tmp/x"])


if __name__ == "__main__":
    unittest.main()
