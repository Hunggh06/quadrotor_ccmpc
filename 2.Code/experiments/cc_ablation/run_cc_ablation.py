#!/usr/bin/env python3
"""Isolate the chance-constraint (CC) contribution from the fixed safety margin.

Matrix: 3 margins x 2 controller modes (CC on/off) x 2 constraint modes
(soft/hard).  Every run uses the same scenario, seed and duration.  The
baseline configuration and the validated baseline outputs are never modified.

The CC-on controller adds ``z(epsilon) * sigma`` to the deterministic radius
``obstacle + margin + drone``.  The CC-off controller uses only that
deterministic radius.  Comparing the two at the same margin therefore answers
"does the CC component change the trajectory?"; comparing margins answers
"how much of the behaviour is just the fixed margin?".
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

from simulation.application.native.headless import HeadlessRecordingRuntime
from simulation.application.native.runtime import run_coupled_simulation
from simulation.control.ccmpc_13state.chance_constrained import (
    SphericalChanceConstrainedNMPCController,
)
from simulation.control.ccmpc_13state.deterministic import DeterministicNMPCController
from simulation.infrastructure.resources import resolve_input_path
from simulation.interfaces.desktop.viewer import load_native_mujoco_config

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_CONFIG = ROOT / "config" / "experiments" / "cc_ablation.yaml"
DEFAULT_OUTPUT = ROOT / "outputs" / "cc_ablation"

VARIANTS: dict[str, dict[str, Any]] = {}


def _add(name: str, *, chance: bool, margin: float, soft: bool) -> None:
    VARIANTS[name] = {
        "chance": bool(chance),
        "margin": float(margin),
        "soft": bool(soft),
        "description": (
            f"{'CC' if chance else 'MPC'} "
            f"{'soft' if soft else 'hard'} constraints, "
            f"fixed margin {margin:.2f} m"
        ),
    }


for _margin in (0.30, 0.10, 0.00):
    _tag = f"m{int(round(_margin * 100)):03d}"
    _add(f"cc_soft_{_tag}", chance=True, margin=_margin, soft=True)
    _add(f"nocc_soft_{_tag}", chance=False, margin=_margin, soft=True)
    _add(f"cc_hard_{_tag}", chance=True, margin=_margin, soft=False)
    _add(f"nocc_hard_{_tag}", chance=False, margin=_margin, soft=False)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", action="append", choices=sorted(VARIANTS))
    parser.add_argument("--all", action="store_true", help="run every variant")
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT))
    parser.add_argument("--duration", type=float, default=6.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--tag", default="", help="optional suffix for the output folder")
    return parser


def _metrics(run_dir: Path) -> dict[str, Any]:
    summary = {}
    summary_path = run_dir / "summary.yaml"
    if summary_path.exists():
        summary = yaml.safe_load(summary_path.read_text(encoding="utf-8")) or {}

    telemetry_path = run_dir / "telemetry.csv"
    telemetry = pd.read_csv(telemetry_path)
    position = telemetry[["px", "py", "pz"]].to_numpy(dtype=float)
    velocity = telemetry[["vx", "vy", "vz"]].to_numpy(dtype=float)
    path_length = float(np.sum(np.linalg.norm(np.diff(position, axis=0), axis=1)))
    speed = np.linalg.norm(velocity, axis=1)

    def q(series_name: str, quantile: float) -> float | None:
        if series_name not in telemetry:
            return None
        values = telemetry[series_name].to_numpy(dtype=float)
        values = values[np.isfinite(values)]
        return float(np.quantile(values, quantile)) if values.size else None

    return {
        "run_dir": str(run_dir),
        "termination_reason": summary.get("termination_reason"),
        "collision": summary.get("collision"),
        "collision_type": summary.get("collision_type"),
        "min_clearance_m": summary.get("min_clearance_m"),
        "min_ground_clearance_m": summary.get("minimum_ground_clearance_m"),
        "max_slack_m": summary.get("maximum_slack_m"),
        "fallback_activations": summary.get("fallback_activations"),
        "deadline_misses": summary.get("deadline_misses"),
        "risk_semantics": summary.get("risk_semantics"),
        "risk_allocation_method": summary.get("risk_allocation_method"),
        "risk_budget_status": summary.get("risk_budget_status"),
        "final_goal_distance_m": float(telemetry["goal_distance_m"].iloc[-1]),
        "min_goal_distance_m": float(telemetry["goal_distance_m"].min()),
        "final_position": position[-1].tolist(),
        "path_length_m": path_length,
        "max_speed_mps": float(speed.max()),
        "mean_speed_mps": float(speed.mean()),
        "p50_total_ms": q("total_controller_time_ms", 0.50),
        "p95_total_ms": q("total_controller_time_ms", 0.95),
        "p50_nlp_ms": q("nlp_solve_time_ms", 0.50),
        "p95_nlp_ms": q("nlp_solve_time_ms", 0.95),
        "samples": int(len(telemetry)),
    }


def _run_one(
    *,
    name: str,
    spec: dict[str, Any],
    config_path: Path,
    output_root: Path,
    duration: float,
    seed: int,
    tag: str,
) -> dict[str, Any]:
    base_config = load_native_mujoco_config(config_path)
    chance = replace(
        base_config.chance_constraints,
        enabled=bool(spec["chance"]),
        soft_constraint=bool(spec["soft"]),
    )
    covariance = replace(
        base_config.covariance_propagation,
        enabled=bool(spec["chance"]),
    )
    variant_dir = output_root / (name + (f"_{tag}" if tag else ""))
    recording = replace(
        base_config.recording,
        output_dir=str(variant_dir),
        enabled=True,
    )
    config = replace(
        base_config,
        safety_margin=float(spec["margin"]),
        chance_constraints=chance,
        covariance_propagation=covariance,
        estimation=replace(base_config.estimation, seed=int(seed)),
        recording=recording,
        duration_s=float(duration),
    )

    controller_kwargs = dict(
        bounds=config.bounds,
        obstacle_specs=config.obstacles,
        margin=config.safety_margin,
        horizon_steps=config.horizon_steps,
        timestep_s=config.mpc_timestep_s,
        max_iter=config.max_solver_iterations,
        covariance_options=config.covariance_propagation,
        chance_options=chance,
    )
    if chance.enabled:
        controller = SphericalChanceConstrainedNMPCController(**controller_kwargs)
    else:
        controller = DeterministicNMPCController(**controller_kwargs)

    runtime = HeadlessRecordingRuntime(config, base_dir=ROOT, enable_recording=True)
    started = time.perf_counter()
    result = run_coupled_simulation(
        x0_vals=config.start,
        goal_pos=config.goal_position,
        goal_euler=config.goal_euler,
        bounds=config.bounds,
        obstacles=[dict(item) for item in config.obstacles],
        margin=config.safety_margin,
        sim_seconds=config.duration_s,
        mpc_dt=config.mpc_timestep_s,
        n_horizon=config.horizon_steps,
        max_iter=config.max_solver_iterations,
        mj_dt=config.mujoco_timestep_s,
        runtime=runtime,
        controller=controller,
        stop_on_goal=config.stop_on_goal,
        goal_tolerance=config.goal_tolerance_m,
        stop_on_collision=config.stop_on_collision,
        estimation_options=config.estimation,
        covariance_options=config.covariance_propagation,
        chance_options=chance,
        safety_fallback_options=config.safety_fallback,
    )
    wall_time_s = time.perf_counter() - started
    recording_path = runtime.finalize(result)
    if recording_path is None:
        raise RuntimeError(f"variant {name} did not produce a recording")
    run_dir = Path(recording_path)
    metrics = _metrics(run_dir)
    payload = {
        "variant": name,
        "description": spec["description"],
        "seed": int(seed),
        "duration_s": float(duration),
        "wall_time_s": wall_time_s,
        "margin_m": float(spec["margin"]),
        "chance_enabled": bool(spec["chance"]),
        "soft_constraint": bool(spec["soft"]),
        "run_dir": str(run_dir),
        "metrics": metrics,
    }
    (run_dir / "cc_ablation.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    return payload


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    selected = list(VARIANTS) if args.all else (args.variant or ["cc_soft_m030"])
    config_path = resolve_input_path(args.config)
    output_root = Path(args.output_root)
    output_root.mkdir(parents=True, exist_ok=True)
    results = []
    for name in selected:
        print(f"[{name}] running ...", flush=True)
        payload = _run_one(
            name=name,
            spec=VARIANTS[name],
            config_path=config_path,
            output_root=output_root,
            duration=args.duration,
            seed=args.seed,
            tag=args.tag,
        )
        results.append(payload)
        m = payload["metrics"]
        print(
            f"[{name}] collision={m['collision']} "
            f"min_clearance={m['min_clearance_m']} "
            f"final_goal={m['final_goal_distance_m']:.4f} "
            f"max_slack={m['max_slack_m']} "
            f"fallback={m['fallback_activations']} "
            f"p95_total={m['p95_total_ms']} "
            f"wall={payload['wall_time_s']:.1f}s",
            flush=True,
        )
    index = output_root / ("runs" + (f"_{args.tag}" if args.tag else "") + ".json")
    index.write_text(
        json.dumps(results, indent=2, ensure_ascii=False),
        encoding="utf-8",
    )
    print(f"summary index: {index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
