"""Shared headless runner for opt-in experiments on frozen MuJoCo scenes.

The baseline controller and its configuration are not modified here.  A run
receives an explicit controller specification (uniform chance constraints or
the opt-in joint-risk controller), records normal native telemetry, and writes
one ``experiment.json`` sidecar next to the recording for later analysis.
"""

from __future__ import annotations

import json
import time
from dataclasses import replace
from pathlib import Path
from typing import Any

import yaml

from simulation.application.native.headless import HeadlessRecordingRuntime
from simulation.application.native.runtime import run_coupled_simulation
from simulation.control.ccmpc_13state.chance_constrained import (
    SphericalChanceConstrainedNMPCController,
)
from simulation.interfaces.desktop.viewer import load_native_mujoco_config

ROOT = Path(__file__).resolve().parents[2]


def _load_config(path: Path, duration: float, seed: int, output_dir: Path, chance):
    config = load_native_mujoco_config(path)
    estimation = replace(config.estimation, seed=int(seed))
    recording = replace(config.recording, output_dir=str(output_dir), enabled=True)
    return replace(
        config,
        estimation=estimation,
        recording=recording,
        duration_s=float(duration),
        chance_constraints=chance,
    )


def _build_chance(config, overrides: dict[str, Any]):
    if not overrides:
        return config.chance_constraints
    return replace(config.chance_constraints, **overrides)


def run_one(
    *,
    name: str,
    spec: dict[str, Any],
    config_path: Path,
    output_root: Path,
    duration: float,
    seed: int,
    tag: str = "",
) -> dict[str, Any]:
    """Run one controller/scenario pair and return its metadata payload."""
    base_config = load_native_mujoco_config(config_path)
    chance = _build_chance(base_config, spec.get("chance", {}))
    variant_dir = output_root / (name + (f"_{tag}" if tag else ""))
    config = _load_config(config_path, duration, seed, variant_dir, chance)

    controller_type = SphericalChanceConstrainedNMPCController
    extra_options: dict[str, Any] = {}
    if spec.get("joint_risk") is not None:
        from simulation.control.ccmpc_13state.experimental_joint_risk import (
            JointRiskNMPCController,
            JointRiskOptions,
        )

        controller_type = JointRiskNMPCController
        extra_options["joint_options"] = JointRiskOptions(**spec["joint_risk"])

    controller = controller_type(
        bounds=config.bounds,
        obstacle_specs=config.obstacles,
        margin=config.safety_margin,
        horizon_steps=config.horizon_steps,
        timestep_s=config.mpc_timestep_s,
        max_iter=config.max_solver_iterations,
        covariance_options=config.covariance_propagation,
        chance_options=chance,
        **extra_options,
    )

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

    summary: dict[str, Any] = {}
    summary_path = Path(recording_path) / "summary.yaml"
    if summary_path.exists():
        summary = yaml.safe_load(summary_path.read_text(encoding="utf-8")) or {}

    controller_report = controller.report() if hasattr(controller, "report") else {}
    payload = {
        "variant": name,
        "description": spec["description"],
        "seed": int(seed),
        "duration_s": float(duration),
        "wall_time_s": wall_time_s,
        "config": str(config_path),
        "chance_overrides": spec.get("chance", {}),
        "joint_risk_options": spec.get("joint_risk"),
        "run_summary": summary,
        "controller_report": controller_report,
    }
    sidecar = Path(recording_path) / "experiment.json"
    sidecar.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    payload["recording_path"] = str(recording_path)
    payload["sidecar"] = str(sidecar)
    return payload
