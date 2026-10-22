#!/usr/bin/env python3
"""Aggregate the CC ablation over seeds 7, 17 and 27."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "outputs" / "cc_ablation"
SUMMARY = ROOT / "experiments" / "cc_ablation" / "cc_ablation_multiseed.json"

CORE = [
    "cc_soft_m030",
    "nocc_soft_m030",
    "cc_hard_m030",
    "nocc_hard_m030",
    "cc_soft_m000",
    "nocc_soft_m000",
    "cc_hard_m000",
    "nocc_hard_m000",
]
METRICS = [
    "min_clearance_m",
    "final_goal_distance_m",
    "p95_total_ms",
    "max_slack_m",
    "deadline_misses",
    "fallback_activations",
]


def load(path: Path) -> dict[str, dict]:
    return {item["variant"]: item for item in json.loads(path.read_text(encoding="utf-8"))}


def main() -> int:
    seed_runs = {
        7: load(OUTPUT / "runs.json"),
        17: load(OUTPUT / "runs_s17.json"),
        27: load(OUTPUT / "runs_s27.json"),
    }
    aggregate: dict[str, dict] = {}
    for variant in CORE:
        rows = [seed_runs[seed][variant] for seed in seed_runs]
        metrics: dict[str, dict] = {}
        for key in METRICS:
            values = np.asarray([row["metrics"][key] for row in rows], dtype=float)
            metrics[key] = {
                "mean": float(values.mean()),
                "std": float(values.std(ddof=1)) if values.size > 1 else 0.0,
                "values": values.tolist(),
            }
        metrics["collision_count"] = int(
            sum(bool(row["metrics"]["collision"]) for row in rows)
        )
        metrics["negative_clearance_count"] = int(
            sum(float(row["metrics"]["min_clearance_m"]) < 0.0 for row in rows)
        )
        aggregate[variant] = {
            "seeds": sorted(seed_runs),
            "margin_m": rows[0]["margin_m"],
            "chance_enabled": rows[0]["chance_enabled"],
            "soft_constraint": rows[0]["soft_constraint"],
            "metrics": metrics,
        }

    SUMMARY.write_text(
        json.dumps(aggregate, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )

    print(
        "variant | clearance mean±std | goal mean±std | p95 ms mean±std | "
        "misses mean | fallback mean | collisions/3 | neg-clearance/3"
    )
    for variant in CORE:
        item = aggregate[variant]["metrics"]
        print(
            f"{variant} | "
            f"{item['min_clearance_m']['mean']:.4f}±{item['min_clearance_m']['std']:.4f} | "
            f"{item['final_goal_distance_m']['mean']:.4f}±"
            f"{item['final_goal_distance_m']['std']:.4f} | "
            f"{item['p95_total_ms']['mean']:.1f}±{item['p95_total_ms']['std']:.1f} | "
            f"{item['deadline_misses']['mean']:.1f} | "
            f"{item['fallback_activations']['mean']:.1f} | "
            f"{item['collision_count']}/3 | "
            f"{item['negative_clearance_count']}/3"
        )
    print(f"wrote {SUMMARY}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
