#!/usr/bin/env python3
"""Compare joint-risk NLP on existing, frozen geometry (explicit opt-in only)."""

import argparse
import csv
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.common.runner import ROOT, run_one
from experiments.common.scenarios import SCENARIOS, metrics


VARIANTS = {
    "uniform": {"description": "Original uniform CC-MPC"},
    "joint_soft": {
        "description": "Joint required-risk NLP, soft chance relaxation",
        "joint_risk": {"formulation": "required"},
    },
    "hard_uniform": {
        "description": "Uniform CC-MPC with hard chance constraints",
        "chance": {"soft_constraint": False},
    },
    "joint_hard": {
        "description": "Joint required-risk NLP, hard chance constraints (main)",
        "joint_risk": {"formulation": "required"},
        "chance": {"soft_constraint": False},
    },
    "joint_required": {
        "description": "Alias of joint_hard: required-risk constraint form",
        "joint_risk": {"formulation": "required"},
        "chance": {"soft_constraint": False},
    },
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="five_lowu_narrow")
    parser.add_argument("--variant", action="append", choices=sorted(VARIANTS))
    parser.add_argument("--seed", action="append", type=int)
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--tag", default="joint")
    args = parser.parse_args()
    output = ROOT / "outputs" / "joint_risk"
    output.mkdir(parents=True, exist_ok=True)
    results = []
    for variant in args.variant or ["uniform", "joint_soft", "hard_uniform", "joint_hard"]:
        for seed in args.seed or [1001, 17, 27]:
            print(f"{variant}/{args.scenario}/seed={seed}", flush=True)
            payload = run_one(
                name=f"{variant}_{args.scenario}", spec=VARIANTS[variant],
                config_path=SCENARIOS[args.scenario], output_root=output,
                duration=args.duration, seed=seed, tag=args.tag,
            )
            row = metrics(payload, args.scenario)
            row["recording_path"] = payload["recording_path"]
            records = payload["controller_report"].get("records", [])
            with (Path(payload["recording_path"]) / "telemetry.csv").open(
                encoding="utf-8-sig", newline="",
            ) as stream:
                telemetry = list(csv.DictReader(stream))
            row["solver_failures"] = sum(t["primary_solver_success"] != "1" for t in telemetry)
            row["invalid_allocations"] = sum(r["budget_status"] != "BUDGET_OK" for r in records)
            row["max_delta_sum"] = max((r["delta_sum"] for r in records), default=None)
            results.append(row)
            print(json.dumps(row, ensure_ascii=False), flush=True)
            (output / f"metrics_{args.tag}.json").write_text(
                json.dumps(results, indent=2, ensure_ascii=False), encoding="utf-8",
            )


if __name__ == "__main__":
    main()
