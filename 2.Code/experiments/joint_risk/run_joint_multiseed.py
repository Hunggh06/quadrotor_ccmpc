#!/usr/bin/env python3
"""Bounded, resumable paired-seed benchmark on two existing frozen scenes."""

import argparse
import gc
import hashlib
import json
from pathlib import Path
import random
import sys
import traceback

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.common.runner import ROOT, run_one
from experiments.common.scenarios import SCENARIOS
from experiments.joint_risk.run_joint_risk import VARIANTS


STUDY = "joint_risk_multiseed_v2_required"
OUTPUT = ROOT / "outputs" / "joint_risk_multiseed_v2_required"
SEEDS = [1001, 17, 27, *range(101, 118)]
SCENE_SEEDS = {"five_lowu_narrow": SEEDS, "five_lowu": SEEDS[:10]}
CONTROLLERS = ["uniform", "hard_uniform", "joint_hard"]


def save_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, ensure_ascii=False), encoding="utf-8")
    temporary.replace(path)


def make_plan():
    # Balance execution order without concurrently loading the CPU. Timing
    # comparisons are at run level, not falsely treating ticks as replicates.
    rng = random.Random(20261020)
    blocks = [(scene, seed) for scene, seeds in SCENE_SEEDS.items() for seed in seeds]
    rng.shuffle(blocks)
    schedule = []
    for scene, seed in blocks:
        variants = CONTROLLERS.copy()
        rng.shuffle(variants)
        schedule.extend({"scenario": scene, "seed": seed, "variant": v} for v in variants)
    sources = list((ROOT / "simulation/control/ccmpc_13state").rglob("*.py"))
    sources += [SCENARIOS[s] for s in SCENE_SEEDS]
    return {
        "study": STUDY, "duration_s": 6.5,
        "joint_formulation": "required",
        "scene_seeds": SCENE_SEEDS, "controllers": CONTROLLERS,
        "schedule_rng_seed": 20261020, "schedule": schedule,
        "source_sha256": {
            str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(sources)
        },
        "primary_endpoint": "reach distance <= 0.25 m by 6.5 s AND no collision in run",
        "sampling_unit": "one seed within one fixed scenario; never one control tick",
        "comparisons": "joint_hard vs uniform, joint_hard vs hard_uniform, within each scene",
        "analysis": {
            "primary": "paired success difference; exact two-sided McNemar via binomial",
            "multiplicity": "Holm across four primary contrasts; exploratory, scene previously selected",
            "uncertainty": "Wilson 95% per-arm success CI; paired percentile bootstrap 95% difference CI",
            "secondary": "paired clearance, final goal and runtime; path/arrival conditional on both successful",
            "missing": "run errors count as failures for primary endpoint, continuous missing explicit",
            "bootstrap_seed": 20261020, "bootstrap_resamples": 20000,
            "no_pooling": "Report scenes separately: ten seed values reused across scenes",
        },
        "constraints": "No geometry, algorithm, margin, solver-limit or risk-budget tuning during batch",
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print the frozen schedule and exit without running simulations",
    )
    args = parser.parse_args()
    plan = make_plan()
    if args.dry_run:
        print(json.dumps(plan, indent=2, ensure_ascii=False))
        return
    OUTPUT.mkdir(parents=True, exist_ok=True)
    plan_path = OUTPUT / "plan.json"
    if plan_path.exists():
        if json.loads(plan_path.read_text(encoding="utf-8")) != plan:
            raise RuntimeError("Study sources or schedule changed; do not mix incompatible runs")
    else:
        save_json(plan_path, plan)
    index_path = OUTPUT / "index.json"
    results = json.loads(index_path.read_text(encoding="utf-8")) if index_path.exists() else []
    finished = {(r["scenario"], r["seed"], r["variant"]) for r in results}
    for number, case in enumerate(plan["schedule"], 1):
        key = (case["scenario"], case["seed"], case["variant"])
        if key in finished:
            continue
        print(f'[{number}/90] {key}', flush=True)
        record = dict(case)
        try:
            payload = run_one(
                name=f'{case["variant"]}_{case["scenario"]}',
                spec=VARIANTS[case["variant"]], config_path=SCENARIOS[case["scenario"]],
                output_root=OUTPUT, duration=plan["duration_s"], seed=case["seed"], tag="paired",
            )
            record.update({
                "status": "completed", "recording_path": payload["recording_path"],
                "wall_time_s": payload["wall_time_s"],
                "collision": payload["run_summary"].get("collision"),
            })
        except Exception as exc:
            record.update({"status": "run_error", "error": repr(exc), "traceback": traceback.format_exc()})
            print(record["traceback"], flush=True)
        results.append(record)
        save_json(index_path, results)
        print(json.dumps(record, ensure_ascii=False), flush=True)
        gc.collect()
    print(f'Finished {len(results)}/{len(plan["schedule"])} cases: {index_path}', flush=True)


if __name__ == "__main__":
    main()
