#!/usr/bin/env python3
"""Audit completed joint-risk runs; emit JSON and a compact report table."""

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiments.common.runner import ROOT
from experiments.common.scenarios import metrics


def summarize(index):
    result = []
    for entry in json.loads(index.read_text(encoding="utf-8")):
        recording = Path(entry["recording_path"])
        payload = json.loads((recording / "experiment.json").read_text(encoding="utf-8"))
        payload["recording_path"] = str(recording)
        row = metrics(payload, entry["scenario"])
        with (recording / "telemetry.csv").open(encoding="utf-8-sig", newline="") as stream:
            telemetry = list(csv.DictReader(stream))
        row["samples"] = len(telemetry)
        row["solver_failures"] = sum(t["primary_solver_success"] != "1" for t in telemetry)
        row["iteration_p95"] = float(np.percentile(
            [int(t["primary_solver_iterations"]) for t in telemetry], 95,
        ))
        row["budget_invalid_telemetry"] = sum(t["risk_budget_status"] != "BUDGET_OK" for t in telemetry)
        records = payload.get("controller_report", {}).get("records", [])
        if records:
            delta = np.asarray([r["delta"] for r in records])[:, 1:, :]
            row.update({
                "delta_min": float(delta.min()), "delta_max": float(delta.max()),
                "max_delta_sum": float(delta.sum(axis=(1, 2)).max()),
                "delta_sum_mean": float(delta.sum(axis=(1, 2)).mean()),
                "mean_risk_by_obstacle": delta.sum(axis=1).mean(axis=0).tolist(),
                "max_nlp_slack_m": max(r["nlp_slack_max_m"] or 0 for r in records),
                "invalid_allocations": sum(r["budget_status"] != "BUDGET_OK" for r in records),
                "raw_solver_failures": sum(not r["solver_success"] for r in records),
            })
        row["recording_path"] = str(recording)
        result.append(row)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", default="refined_narrow")
    args = parser.parse_args()
    output = ROOT / "outputs" / "joint_risk"
    rows = summarize(output / f"metrics_{args.tag}.json")
    (output / f"audit_{args.tag}.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False), encoding="utf-8",
    )
    print("| Variant | Seed | Goal | Path | t_goal | Clearance | p95 ms | Fail | Fallback | Miss |")
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for r in rows:
        arrival = "—" if r["time_to_tolerance_s"] is None else f'{r["time_to_tolerance_s"]:.2f}'
        print(
            f'| {r["variant"].removesuffix("_" + r["scenario"])} | {r["seed"]} | '
            f'{r["goal_distance_m"]:.3f} | {r["path_length_m"]:.3f} | {arrival} | '
            f'{r["min_clearance_m"]:.3f} | {r["solver_p95_ms"]:.1f} | '
            f'{r["solver_failures"]} | {r["fallback"]} | {r["deadline_misses"]}/{r["samples"]} |'
        )


if __name__ == "__main__":
    main()
