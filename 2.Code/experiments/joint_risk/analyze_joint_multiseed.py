#!/usr/bin/env python3
"""Seed-level paired analysis; do not count time steps as independent trials."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
from scipy.stats import binomtest, norm

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
OUTPUT = ROOT / "outputs" / "joint_risk_multiseed_v2_required"
VARIANTS = ["uniform", "hard_uniform", "joint_hard"]


def wilson(successes, count):
    if count < 1:
        return None
    z = float(norm.ppf(0.975))
    p = successes / count
    center = (p + z*z / (2*count)) / (1 + z*z / count)
    half = z * np.sqrt(p*(1-p)/count + z*z/(4*count*count)) / (1 + z*z/count)
    return [float(max(0, center-half)), float(min(1, center+half))]


def describe(values):
    array = np.asarray(values, dtype=float)
    if not array.size:
        return {"n": 0}
    if not np.all(np.isfinite(array)):
        raise ValueError("Non-finite observation in statistical analysis")
    q1, median, q3 = np.percentile(array, [25, 50, 75])
    return {
        "n": len(array), "mean": float(array.mean()),
        "sd": float(array.std(ddof=1)) if len(array) > 1 else None,
        "median": float(median), "q1": float(q1), "q3": float(q3),
        "min": float(array.min()), "max": float(array.max()),
    }


def paired_estimate(differences, rng):
    array = np.asarray(differences, dtype=float)
    result = describe(array)
    result["mean_difference_ci95"] = None
    if len(array) >= 2:
        samples = rng.integers(0, len(array), size=(20000, len(array)))
        result["mean_difference_ci95"] = np.percentile(array[samples].mean(axis=1), [2.5, 97.5]).tolist()
        result["bootstrap_degenerate"] = bool(np.ptp(array) == 0)
    return result


def holm(pvalues):
    order = np.argsort(pvalues)
    adjusted = np.empty(len(pvalues))
    running = 0.
    for rank, index in enumerate(order):
        running = max(running, (len(pvalues)-rank)*pvalues[index])
        adjusted[index] = min(1., running)
    return adjusted.tolist()


def collect(output):
    from experiments.common.scenarios import metrics

    plan = json.loads((output / "plan.json").read_text(encoding="utf-8"))
    for name, expected_hash in plan["source_sha256"].items():
        if hashlib.sha256((ROOT / name).read_bytes()).hexdigest() != expected_hash:
            raise RuntimeError(f"Study source changed during benchmark: {name}")
    index = json.loads((output / "index.json").read_text(encoding="utf-8"))
    expected = {(c["scenario"], c["seed"], c["variant"]) for c in plan["schedule"]}
    actual = [(c["scenario"], c["seed"], c["variant"]) for c in index]
    if set(actual) != expected or len(set(actual)) != len(actual):
        raise RuntimeError(f"Incomplete or duplicate study: {len(actual)}/{len(expected)} cases")
    rows = []
    for entry in index:
        row = {k: entry[k] for k in ("scenario", "seed", "variant", "status")}
        row["fresh_seed"] = row["seed"] not in (1001, 17, 27)
        if entry["status"] != "completed":
            row.update({"success": False, "error": entry.get("error"), "collision": None})
            rows.append(row)
            continue
        recording = Path(entry["recording_path"])
        payload = json.loads((recording / "experiment.json").read_text(encoding="utf-8"))
        payload["recording_path"] = str(recording)
        measured = metrics(payload, entry["scenario"])
        measured.pop("variant")
        row.update(measured)
        with (recording / "telemetry.csv").open(encoding="utf-8-sig", newline="") as stream:
            telemetry = list(csv.DictReader(stream))
        row["samples"] = len(telemetry)
        row["solver_failures"] = sum(t["primary_solver_success"] != "1" for t in telemetry)
        row["budget_invalid_ticks"] = sum(t["risk_budget_status"] != "BUDGET_OK" for t in telemetry)
        row["total_controller_p95_ms"] = float(np.percentile(
            [float(t["total_controller_time_ms"]) for t in telemetry], 95,
        ))
        row["termination_reason"] = payload["run_summary"].get("termination_reason")
        if row["samples"] != 130 or row["termination_reason"] != "duration_reached":
            row["status"] = "incomplete_run"
        row["success"] = bool(
            row["status"] == "completed" and not row["collision"]
            and row["time_to_tolerance_s"] is not None
        )
        row["recording_path"] = str(recording)
        records = payload.get("controller_report", {}).get("records", [])
        if records:
            row["invalid_raw_allocations"] = sum(r["budget_status"] != "BUDGET_OK" for r in records)
            row["max_delta_sum"] = max(r["delta_sum"] for r in records)
        rows.append(row)
    return plan, rows


def analyze(plan, rows):
    result = {"scenarios": {}, "comparisons": [], "contrast_direction": "joint_hard minus comparator"}
    rng = np.random.default_rng(20261020)
    for scene, seeds in plan["scene_seeds"].items():
        groups = {}
        for variant in VARIANTS:
            group = [r for r in rows if r["scenario"] == scene and r["variant"] == variant]
            complete = [r for r in group if r["status"] == "completed"]
            successes = sum(r["success"] for r in group)
            fresh = [r for r in group if r["fresh_seed"]]
            groups[variant] = {
                "n": len(group), "successes": successes, "success_rate": successes/len(group),
                "success_wilson_ci95": wilson(successes, len(group)),
                "collisions": sum(bool(r["collision"]) for r in group),
                "unknown_collision_runs": sum(r["collision"] is None for r in group),
                "run_errors": len(group)-len(complete),
                "fresh_seed_successes": sum(r["success"] for r in fresh), "fresh_seed_n": len(fresh),
                "fallback_ticks": sum(r.get("fallback", 0) or 0 for r in group),
                "solver_failures": sum(r.get("solver_failures", 0) for r in group),
                "deadline_misses": sum(r.get("deadline_misses", 0) or 0 for r in group),
                "ticks_observed": sum(r.get("samples", 0) for r in group),
                "budget_invalid_ticks": sum(r.get("budget_invalid_ticks", 0) for r in group),
            }
            for field in (
                "goal_distance_m", "min_clearance_m", "solver_p95_ms", "total_controller_p95_ms",
                "max_slack_m", "path_length_m",
            ):
                groups[variant][field] = describe([r[field] for r in complete])
            groups[variant]["arrival_successful_only"] = describe([
                r["time_to_tolerance_s"] for r in complete if r["success"]
            ])
        result["scenarios"][scene] = groups
        lookup = {(r["seed"], r["variant"]): r for r in rows if r["scenario"] == scene}
        for baseline in ("uniform", "hard_uniform"):
            pairs = [(lookup[s, "joint_hard"], lookup[s, baseline]) for s in seeds]
            gain = sum(j["success"] and not b["success"] for j, b in pairs)
            loss = sum(b["success"] and not j["success"] for j, b in pairs)
            discordant = gain + loss
            comparison = {
                "scenario": scene, "comparator": baseline, "paired_n": len(pairs),
                "joint_only_success": gain, "comparator_only_success": loss,
                "both_success": sum(j["success"] and b["success"] for j, b in pairs),
                "neither_success": sum(not j["success"] and not b["success"] for j, b in pairs),
                "success_difference": paired_estimate([
                    int(j["success"])-int(b["success"]) for j, b in pairs
                ], rng),
                "mcnemar_exact_p": float(binomtest(gain, discordant, 0.5).pvalue) if discordant else 1.,
            }
            for field in ("min_clearance_m", "goal_distance_m", "solver_p95_ms", "total_controller_p95_ms"):
                comparison[field] = paired_estimate([
                    j[field]-b[field] for j, b in pairs
                    if j["status"] == b["status"] == "completed"
                ], rng)
            for field in ("path_length_m", "time_to_tolerance_s"):
                comparison[field + "_both_success_only"] = paired_estimate([
                    j[field]-b[field] for j, b in pairs if j["success"] and b["success"]
                ], rng)
            result["comparisons"].append(comparison)
    corrected = holm([c["mcnemar_exact_p"] for c in result["comparisons"]])
    for contrast, pvalue in zip(result["comparisons"], corrected):
        contrast["mcnemar_holm_p"] = pvalue
    return result


def plot_raw(plan, rows, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    colors = ["#0072B2", "#009E73", "#D55E00"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 9), layout="constrained")
    for i, (scene, seeds) in enumerate(plan["scene_seeds"].items()):
        lookup = {(r["seed"], r["variant"]): r for r in rows if r["scenario"] == scene}
        matrix = np.array([[int(lookup[s, v]["success"]) for v in VARIANTS] for s in seeds])
        ax = axes[i, 0]
        ax.imshow(matrix, aspect="auto", cmap=ListedColormap(["#eeeeee", "#0072B2"]), vmin=0, vmax=1)
        ax.set_xticks(range(3), VARIANTS, rotation=15)
        ax.set_yticks(range(len(seeds)), [str(s) for s in seeds])
        ax.set_title(f"{scene}: reached + no collision\nblue=success, gray=failure")
        ax.set_ylabel("Seed (one independent run per controller)")
        for column, (field, title) in enumerate((
            ("min_clearance_m", "Minimum clearance (m)"),
            ("solver_p95_ms", "Per-run solver p95 (ms)"),
        ), 1):
            ax = axes[i, column]
            for seed in seeds:
                values = [lookup[seed, v].get(field, np.nan) for v in VARIANTS]
                ax.plot(range(3), values, color="#aaaaaa", alpha=0.5, linewidth=0.7)
                for j, value in enumerate(values):
                    ax.scatter(j, value, color=colors[j], marker=["o", "s", "^"][j], s=28)
            if field.endswith("ms"):
                ax.axhline(50, color="black", linestyle="--", linewidth=1, label="50 ms reference")
                ax.legend(fontsize=8)
            ax.set_xticks(range(3), VARIANTS, rotation=15)
            ax.set_title(f"{scene}: {title}")
            ax.grid(axis="y", alpha=0.25)
    fig.savefig(output / "paired_raw.png", dpi=160, facecolor="white")
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(11, 7), layout="constrained")
    for i, scene in enumerate(plan["scene_seeds"]):
        lookup = {(r["seed"], r["variant"]): r for r in rows if r["scenario"] == scene}
        for j, field in enumerate(("min_clearance_m", "solver_p95_ms")):
            ax = axes[i, j]
            for baseline, color in zip(("uniform", "hard_uniform"), colors):
                differences = [
                    lookup[s, "joint_hard"][field]-lookup[s, baseline][field]
                    for s in plan["scene_seeds"][scene]
                    if lookup[s, "joint_hard"]["status"] == lookup[s, baseline]["status"] == "completed"
                ]
                if differences:
                    ax.hist(differences, bins="auto", histtype="step", linewidth=1.8, color=color, label=baseline)
            ax.axvline(0, color="black", linestyle="--", linewidth=1)
            ax.set_title(scene)
            ax.set_xlabel("Joint minus comparator: " + field)
            ax.set_ylabel("Paired seeds")
            ax.legend(fontsize=8)
    fig.savefig(output / "paired_differences.png", dpi=160, facecolor="white")
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    args = parser.parse_args()
    plan, rows = collect(args.output)
    # Raw plots expose skew/outliers before any inferential summaries. No
    # normality-driven test selection and no deletion of inconvenient seeds.
    plot_raw(plan, rows, args.output)
    result = analyze(plan, rows)
    for filename, data in (("run_metrics.json", rows), ("statistics.json", result)):
        (args.output / filename).write_text(json.dumps(data, indent=2, ensure_ascii=False), encoding="utf-8")
    fields = sorted(set().union(*(r.keys() for r in rows)))
    with (args.output / "run_metrics.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    for scene, groups in result["scenarios"].items():
        print(scene)
        for variant, summary in groups.items():
            print(
                f'  {variant}: success={summary["successes"]}/{summary["n"]} '
                f'collision={summary["collisions"]} error={summary["run_errors"]} '
                f'clearance median={summary["min_clearance_m"].get("median")} '
                f'p95 median={summary["solver_p95_ms"].get("median")}', flush=True,
            )
    print(json.dumps(result["comparisons"], indent=2), flush=True)


if __name__ == "__main__":
    main()
