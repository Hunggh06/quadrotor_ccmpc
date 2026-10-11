"""Discover and load recorded native runs and Monte Carlo campaigns.

Pure data helpers (no Qt) shared by the desktop workbench and its tests.
"""

from __future__ import annotations

import csv
import json
import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np
import yaml

NATIVE_SUBDIR = Path("outputs") / "native"
MONTE_CARLO_SUBDIR = Path("outputs") / "native_monte_carlo"


@dataclass(frozen=True, slots=True)
class NativeRunSummary:
    path: Path
    name: str
    created: str
    risk_semantics: str
    collision: bool
    termination_reason: str
    min_clearance_m: float | None
    maximum_slack_m: float | None
    fallback_activations: int
    samples: int


@dataclass(frozen=True, slots=True)
class CampaignSummary:
    path: Path
    name: str
    created: str
    status: str
    stage_status: str
    planned_trials: int
    completed_trials: int
    protocol: str
    modes: tuple[str, ...] = ()
    noise_levels: tuple[str, ...] = ()

    @property
    def resumable(self) -> bool:
        return self.status != "COMPLETED" and self.completed_trials < self.planned_trials


@dataclass(frozen=True, slots=True)
class NativeTimeseries:
    path: Path
    name: str
    time_s: np.ndarray
    position: np.ndarray
    min_clearance_m: np.ndarray
    goal_distance_m: np.ndarray
    solver_time_ms: np.ndarray
    maximum_slack_m: np.ndarray
    achieved_risk_total: np.ndarray
    fallback_active: np.ndarray
    obstacles: tuple[dict[str, Any], ...]
    start: np.ndarray
    goal: np.ndarray
    risk_budget_total: float | None
    control_period_ms: float
    obstacle_paths: np.ndarray = field(default_factory=lambda: np.empty((0, 0, 3)))
    summary: dict[str, Any] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class CampaignCell:
    noise_label: str
    covariance_scale: float
    controller: str
    trials: int
    success_rate: float
    collision_events: int
    collision_upper_bound: float | None
    min_clearance_mean_m: float | None
    max_slack_mean_m: float | None
    solver_p99_median_ms: float | None
    fallback_episodes: int
    claim_status: str
    claim_blockers: tuple[str, ...]
    gates: dict[str, Any]


def output_roots(code_root: Path) -> list[Path]:
    """Roots that may hold ``outputs/`` (2.Code and the repository root)."""
    roots = [Path(code_root)]
    parent = Path(code_root).parent
    if (parent / "outputs").is_dir():
        roots.append(parent)
    return roots


def _read_yaml(path: Path) -> dict[str, Any]:
    try:
        loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError):
        return {}
    return loaded if isinstance(loaded, dict) else {}


def _float_or_none(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def list_native_runs(roots: Iterable[Path]) -> list[NativeRunSummary]:
    runs: list[NativeRunSummary] = []
    seen: set[Path] = set()
    for root in roots:
        base = Path(root) / NATIVE_SUBDIR
        if not base.is_dir():
            continue
        for directory in base.iterdir():
            summary_path = directory / "summary.yaml"
            if not summary_path.is_file() or directory.resolve() in seen:
                continue
            seen.add(directory.resolve())
            summary = _read_yaml(summary_path)
            runs.append(
                NativeRunSummary(
                    path=directory,
                    name=str(summary.get("scenario", directory.name)),
                    created=str(summary.get("created_utc", directory.name[:16])),
                    risk_semantics=str(summary.get("risk_semantics", "")),
                    collision=bool(summary.get("collision", False)),
                    termination_reason=str(summary.get("termination_reason", "")),
                    min_clearance_m=_float_or_none(summary.get("min_clearance_m")),
                    maximum_slack_m=_float_or_none(summary.get("maximum_slack_m")),
                    fallback_activations=int(summary.get("fallback_activations", 0) or 0),
                    samples=int(summary.get("samples", 0) or 0),
                )
            )
    return sorted(runs, key=lambda run: run.path.name, reverse=True)


def list_campaigns(roots: Iterable[Path]) -> list[CampaignSummary]:
    campaigns: list[CampaignSummary] = []
    seen: set[Path] = set()
    for root in roots:
        for base in sorted((Path(root) / "outputs").glob("native_monte_carlo*")):
            if not base.is_dir():
                continue
            for directory in base.iterdir():
                manifest_path = directory / "manifest.yaml"
                if not manifest_path.is_file() or directory.resolve() in seen:
                    continue
                seen.add(directory.resolve())
                campaigns.append(load_campaign_summary(directory))
    return sorted(campaigns, key=lambda item: item.path.name, reverse=True)


def load_campaign_summary(directory: Path) -> CampaignSummary:
    manifest = _read_yaml(directory / "manifest.yaml")
    protocol = _read_yaml(directory / "protocol.yaml")
    completed = 0
    trials_path = directory / "trials.jsonl"
    if trials_path.is_file():
        with trials_path.open("r", encoding="utf-8") as stream:
            completed = sum(1 for line in stream if line.strip())
    stage_status = ""
    aggregate_path = directory / "aggregate.json"
    if aggregate_path.is_file():
        try:
            aggregate = json.loads(aggregate_path.read_text(encoding="utf-8"))
            stage_status = str(aggregate.get("overall", {}).get("stage_status", ""))
        except (OSError, json.JSONDecodeError):
            stage_status = ""
    levels = tuple(str(level.get("label", "")) for level in protocol.get("noise_levels", []) or [])
    return CampaignSummary(
        path=directory,
        name=str(protocol.get("name", manifest.get("run_id", directory.name))),
        created=str(manifest.get("created_at_utc", "")),
        status=str(manifest.get("status", "UNKNOWN")),
        stage_status=stage_status,
        planned_trials=int(manifest.get("expected_trials", 0) or 0),
        completed_trials=completed,
        protocol=str((protocol.get("monte_carlo") or {}).get("protocol", "")),
        modes=tuple(str(mode) for mode in protocol.get("canonical_modes", []) or []),
        noise_levels=levels,
    )


def _column(rows: Sequence[dict[str, str]], key: str, default: float = math.nan) -> np.ndarray:
    values = []
    for row in rows:
        raw = row.get(key, "")
        try:
            values.append(float(raw) if raw not in ("", "None") else default)
        except ValueError:
            values.append(1.0 if raw == "True" else 0.0 if raw == "False" else default)
    return np.asarray(values, dtype=float)


def load_native_timeseries(directory: Path) -> NativeTimeseries:
    directory = Path(directory)
    scenario = _read_yaml(directory / "scenario.yaml")
    summary = _read_yaml(directory / "summary.yaml")
    with (directory / "telemetry.csv").open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    position = np.column_stack(
        [_column(rows, "px"), _column(rows, "py"), _column(rows, "pz")]
    ).reshape(-1, 3)
    simulation = scenario.get("simulation", {}) or {}
    start = scenario.get("start", {}) or {}
    goal = scenario.get("goal", {}) or {}
    controller = scenario.get("controller", {}) or {}
    chance = controller.get("chance_constraints", {}) or {}
    budget = chance.get("risk_budget", {}) or {}
    period_ms = float(simulation.get("mpc_timestep_s", 0.05)) * 1000.0
    obstacle_paths = np.empty((len(rows), 0, 3), dtype=float)
    trajectory_path = directory / "trajectory.npz"
    if trajectory_path.is_file():
        with np.load(trajectory_path, allow_pickle=False) as arrays:
            predictions = (
                arrays["obstacle_predictions"] if "obstacle_predictions" in arrays else None
            )
            if predictions is not None and predictions.ndim == 4 and predictions.shape[2] > 0:
                obstacle_paths = predictions[:, :, 0, :].copy()
    return NativeTimeseries(
        path=directory,
        name=str(scenario.get("name", directory.name)),
        time_s=_column(rows, "time_s"),
        position=position,
        min_clearance_m=_column(rows, "min_clearance_m"),
        goal_distance_m=_column(rows, "goal_distance_m"),
        solver_time_ms=_column(rows, "solver_time_ms"),
        maximum_slack_m=_column(rows, "maximum_slack_m", 0.0),
        achieved_risk_total=_column(rows, "achieved_risk_total"),
        fallback_active=_column(rows, "fallback_active", 0.0),
        obstacles=tuple(dict(item) for item in scenario.get("obstacles", []) or []),
        start=np.asarray([float(start.get(axis, 0.0)) for axis in "xyz"], dtype=float),
        goal=np.asarray(
            [float((goal.get("position") or goal).get(axis, 0.0)) for axis in "xyz"],
            dtype=float,
        ),
        risk_budget_total=(
            _float_or_none(budget.get("total_epsilon"))
            if chance.get("enabled") and budget.get("semantics") == "joint"
            else None
        ),
        control_period_ms=period_ms,
        obstacle_paths=obstacle_paths,
        summary=summary,
    )


def load_campaign_cells(directory: Path) -> tuple[dict[str, Any], list[CampaignCell]]:
    aggregate = json.loads((Path(directory) / "aggregate.json").read_text(encoding="utf-8"))
    cells: list[CampaignCell] = []
    for label, level in aggregate.get("noise_levels", {}).items():
        for controller, summary in level.get("controllers", {}).items():
            gates = summary.get("gates", {})
            details = gates.get("details", {})

            def median(key: str, summary: dict = summary) -> float | None:
                block = summary.get(key)
                return None if not block else _float_or_none(block.get("median"))

            def mean(key: str, summary: dict = summary) -> float | None:
                block = summary.get(key)
                return None if not block else _float_or_none(block.get("mean"))

            cells.append(
                CampaignCell(
                    noise_label=str(label),
                    covariance_scale=float(level.get("covariance_scale", 1.0)),
                    controller=str(controller),
                    trials=int(summary.get("trials", summary["success_rate"]["trials"])),
                    success_rate=float(summary["success_rate"]["rate"]),
                    collision_events=int(summary["collision_rate"]["events"]),
                    collision_upper_bound=_float_or_none(
                        details.get("collision_upper_bound", details.get("collision_ci_upper"))
                    ),
                    min_clearance_mean_m=mean("min_clearance_m"),
                    max_slack_mean_m=mean("max_slack_m"),
                    solver_p99_median_ms=median("p99_solver_ms"),
                    fallback_episodes=int(details.get("fallback_episodes", 0)),
                    claim_status=str(gates.get("claim_status", "")),
                    claim_blockers=tuple(str(item) for item in gates.get("claim_blockers", [])),
                    gates=dict(gates),
                )
            )
    return aggregate, cells


def read_trial_lines(directory: Path, start: int = 0) -> list[dict[str, Any]]:
    """Read ``trials.jsonl`` from line ``start`` (used for live campaign tables)."""
    path = Path(directory) / "trials.jsonl"
    if not path.is_file():
        return []
    with path.open("r", encoding="utf-8") as stream:
        lines = [line for line in stream if line.strip()]
    return [json.loads(line) for line in lines[start:]]
