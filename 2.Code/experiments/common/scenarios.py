"""Frozen scenario paths and compact telemetry metrics."""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]

# Active experiment scenes. Historical scenes are kept under
# ``config/experiments/legacy/`` but are not exposed by the current CLIs.
SCENARIOS: dict[str, Path] = {
    "five_lowu_narrow": ROOT / "config" / "experiments" / "five_lowu_narrow.yaml",
    "five_lowu": ROOT / "config" / "experiments" / "five_lowu.yaml",
    "five_through": ROOT / "config" / "experiments" / "five_through.yaml",
    "cc_ablation": ROOT / "config" / "experiments" / "cc_ablation.yaml",
}


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _path_length(rows: list[dict[str, str]]) -> float:
    total = 0.0
    previous: tuple[float, float, float] | None = None
    for row in rows:
        point = (float(row["px"]), float(row["py"]), float(row["pz"]))
        if previous is not None:
            total += math.dist(previous, point)
        previous = point
    return total


def _time_to_tolerance(rows: list[dict[str, str]], tolerance: float) -> float | None:
    for row in rows:
        if float(row["goal_distance_m"]) <= tolerance:
            return float(row["time_s"])
    return None


def metrics(payload: dict[str, Any], scenario: str) -> dict[str, Any]:
    recording = Path(payload["recording_path"])
    rows = list(csv.DictReader(open(recording / "telemetry.csv", encoding="utf-8-sig")))
    solver = [float(row["solver_time_ms"]) for row in rows if row.get("solver_time_ms")]
    summary = payload.get("run_summary") or {}
    return {
        "scenario": scenario,
        "variant": payload["variant"],
        "seed": payload["seed"],
        "collision": bool(summary.get("collision")),
        "fallback": summary.get("fallback_activations"),
        "rejected": summary.get("rejected_primary_solutions"),
        "deadline_misses": summary.get("deadline_misses"),
        "goal_distance_m": (
            float(rows[-1]["goal_distance_m"]) if rows and "goal_distance_m" in rows[-1] else None
        ),
        "min_clearance_m": summary.get("minimum_obstacle_clearance_m"),
        "max_slack_m": summary.get("maximum_slack_m"),
        "path_length_m": _path_length(rows),
        "time_to_tolerance_s": _time_to_tolerance(rows, 0.25),
        "solver_p50_ms": percentile(solver, 0.50),
        "solver_p95_ms": percentile(solver, 0.95),
    }
