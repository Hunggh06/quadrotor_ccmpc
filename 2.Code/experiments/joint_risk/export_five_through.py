#!/usr/bin/env python3
"""Export the level five-obstacle S-slalom demo for joint-risk and CC-MPC.

Reads two recorded runs and writes images, MP4 replays, a side-by-side video
and a compact metrics table.  No controller is re-solved here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from simulation.application.native.telemetry import load_native_recording
from simulation.control.ccmpc_13state.core import DRONE_RADIUS, quat_from_euler
from simulation.core.obstacle_motion import obstacle_position
from simulation.infrastructure.mujoco.plant import MuJoCoPlant
from simulation.interfaces.desktop.viewer import NativeMuJoCoConfig
from scripts.export_native_video import SceneDrawer


VARIANTS = ("joint_hard", "uniform")
LABELS = {
    "joint_hard": "Joint-risk (required form)",
    "uniform": "Original CC-MPC (soft-uniform)",
}
SHORT = {"joint_hard": "Joint-risk", "uniform": "CC-MPC original"}
COLORS = {"joint_hard": "#D55E00", "uniform": "#0072B2"}
OBSTACLE_NAMES = ["Gate left", "Gate right", "Near crosser", "Far crosser", "Oscillator"]
HALO_COLOR = "#c97b6e"


def _halo_radius(obstacle: dict) -> float:
    """Faint safety envelope shown around every obstacle."""
    return float(obstacle["radius"]) + DRONE_RADIUS + 0.08


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = (len(ordered) - 1) * fraction
    lower, upper = math.floor(position), math.ceil(position)
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _path_length(rows: list[dict[str, str]]) -> float:
    points = np.asarray([[float(r["px"]), float(r["py"]), float(r["pz"])] for r in rows])
    return float(np.linalg.norm(np.diff(points, axis=0), axis=1).sum()) if len(points) > 1 else 0.0


def _time_to_tolerance(rows: list[dict[str, str]], tolerance: float = 0.25) -> float | None:
    for row in rows:
        if float(row["goal_distance_m"]) <= tolerance:
            return float(row["time_s"])
    return None


def find_run(root: Path, variant: str, scenario: str, tag: str) -> Path:
    directories = sorted(root.glob(f"{variant}_{scenario}_{tag}"))
    runs = [
        run for directory in directories for run in directory.iterdir()
        if (run / "telemetry.csv").exists()
    ]
    if not runs:
        raise SystemExit(f"no recording for {variant}_{scenario}_{tag} under {root}")
    return sorted(runs)[-1]


def load_variant(root: Path, variant: str, scenario: str, tag: str) -> dict:
    path = find_run(root, variant, scenario, tag)
    recording = load_native_recording(path)
    config = NativeMuJoCoConfig.from_mapping(recording["scenario"])
    payload = json.loads((path / "experiment.json").read_text(encoding="utf-8"))
    rows = recording["rows"]
    solver = [float(row["solver_time_ms"]) for row in rows if row.get("solver_time_ms")]
    if variant == "joint_hard":
        delta = np.asarray([record["delta"] for record in payload["controller_report"]["records"]])
    else:
        delta = np.asarray(recording["risk_allocation_horizons"])
    summary = payload.get("run_summary") or {}
    metrics = {
        "variant": variant,
        "label": LABELS[variant],
        "seed": int(payload["seed"]),
        "duration_s": float(payload["duration_s"]),
        "collision": bool(summary.get("collision", False)),
        "fallback": summary.get("fallback_activations", 0),
        "rejected": summary.get("rejected_primary_solutions", 0),
        "deadline_misses": summary.get("deadline_misses", 0),
        "final_goal_distance_m": float(rows[-1]["goal_distance_m"]),
        "min_clearance_m": float(summary.get("minimum_obstacle_clearance_m", math.nan)),
        "max_slack_m": float(summary.get("maximum_slack_m", math.nan)),
        "path_length_m": _path_length(rows),
        "time_to_tolerance_s": _time_to_tolerance(rows),
        "solver_p50_ms": _percentile(solver, 0.50),
        "solver_p95_ms": _percentile(solver, 0.95),
        "solver_failures": sum(row.get("primary_solver_success") != "1" for row in rows),
        "max_delta_sum": float(np.max(delta[:, 1:, :].sum(axis=(1, 2)))) if delta.size else None,
    }
    return {
        "path": path,
        "recording": recording,
        "config": config,
        "payload": payload,
        "rows": rows,
        "states": recording["states"],
        "time": np.asarray([float(row["time_s"]) for row in rows]),
        "delta": delta,
        "metrics": metrics,
        "duration_s": float(payload["duration_s"]),
    }


def _source_state(value: dict, time_s: float) -> np.ndarray:
    config = value["config"]
    initial = np.zeros(13)
    initial[:3] = [config.start[axis] for axis in "xyz"]
    initial[6:10] = quat_from_euler(*(config.start.get(axis, 0.0) for axis in ("roll", "pitch", "yaw")))
    times = np.r_[0.0, value["time"]]
    states = np.vstack([initial, value["states"]])
    right = min(int(np.searchsorted(times, time_s, side="right")), len(times) - 1)
    left = max(0, right - 1)
    weight = float(np.clip((time_s - times[left]) / max(times[right] - times[left], 1e-12), 0.0, 1.0))
    state = (1.0 - weight) * states[left] + weight * states[right]
    a, b = states[left, 6:10], states[right, 6:10]
    if np.dot(a, b) < 0.0:
        b = -b
    quaternion = (1.0 - weight) * a + weight * b
    state[6:10] = quaternion / np.linalg.norm(quaternion)
    return state


def _tick_for(value: dict, time_s: float) -> int:
    return min(max(int(np.searchsorted(value["time"], time_s, side="right")) - 1, 0), len(value["time"]) - 1)


def _make_plot_axes():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    return plt


def plot_trajectory(value: dict, output: Path) -> None:
    plt = _make_plot_axes()
    from matplotlib.patches import Circle

    config = value["config"]
    goal = np.array([config.goal_position[axis] for axis in "xyz"])
    duration = float(value["duration_s"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout="constrained")

    ax = axes[0]
    for obstacle in config.obstacles:
        track = np.array([obstacle_position(obstacle, t) for t in np.linspace(0, duration, 161)])
        if obstacle["type"] == "static":
            point = track[0]
            ax.add_patch(Circle((point[0], point[1]), obstacle["radius"],
                                color="#9a4b3f", alpha=0.8))
        else:
            ax.plot(track[:, 0], track[:, 1], color=HALO_COLOR, linewidth=9,
                    alpha=0.12, solid_capstyle="round", zorder=0)
            ax.plot(track[:, 0], track[:, 1], color="#b08a84", linestyle="--", alpha=0.7)
    positions = np.vstack([[config.start[axis] for axis in "xyz"], value["states"][:, :3]])
    ax.plot(positions[:, 0], positions[:, 1], color=COLORS[value["metrics"]["variant"]], linewidth=2.2)
    ax.scatter(*positions[-1, [0, 1]], color=COLORS[value["metrics"]["variant"]], marker="s", s=45)
    ax.scatter(goal[0], goal[1], marker="*", s=170, color="#D6A000", label="Goal")
    ax.scatter(config.start["x"], config.start["y"], marker="o", color="black", s=28, label="Start")
    ax.set(xlabel="x (m)", ylabel="y (m)", aspect="equal")
    ax.grid(alpha=0.2)
    ax.set_title("XY: S-slalom through the five obstacles")
    ax.legend(fontsize=8, loc="upper left")

    states = np.asarray(value["states"])
    time = np.asarray(value["time"])
    count = min(len(states), len(time))
    ax = axes[1]
    ax.plot(time[:count], states[:count, 2], color=COLORS[value["metrics"]["variant"]], linewidth=2.2)
    ax.axhline(goal[2], color="#D6A000", linestyle="--", linewidth=1.2, label="Goal altitude")
    ax.set(xlabel="Simulation time (s)", ylabel="z (m)", ylim=(0.95, 1.45))
    ax.grid(alpha=0.2)
    ax.set_title("Altitude: stays at z ~ 1.2 m")
    ax.legend(fontsize=8, loc="upper right")
    fig.suptitle(f"{LABELS[value['metrics']['variant']]} / five-obstacle S-slalom / seed {value['metrics']['seed']}")
    fig.savefig(output / "01_trajectory_xy_altitude.png", dpi=180)
    plt.close(fig)


def plot_telemetry(value: dict, output: Path) -> None:
    plt = _make_plot_axes()
    rows, time = value["rows"], value["time"]
    fields = [
        ("goal_distance_m", "Distance to goal (m)", 0.25),
        ("minimum_obstacle_clearance_m", "Physical obstacle clearance (m)", 0.0),
        ("solver_time_ms", "Solver time (ms)", 50.0),
        ("total_controller_time_ms", "End-to-end controller time (ms)", 50.0),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), layout="constrained")
    for ax, (field, label, reference) in zip(axes.flat, fields):
        series = [float(row[field]) for row in rows]
        ax.plot(time, series, color=COLORS[value["metrics"]["variant"]], linewidth=1.7)
        ax.axhline(reference, color="#555555", linestyle="--", linewidth=1)
        ax.set(xlabel="Simulation time (s)", ylabel=label)
        ax.grid(alpha=0.2)
    fig.suptitle(f"Recorded telemetry / {LABELS[value['metrics']['variant']]}")
    fig.savefig(output / "02_telemetry.png", dpi=180)
    plt.close(fig)


def plot_risk(value: dict, output: Path) -> None:
    plt = _make_plot_axes()
    delta, time = value["delta"], value["time"]
    if delta.ndim != 3 or delta.shape[1] < 2:
        return
    total = delta[:, 1:, :].sum(axis=(1, 2))
    mean = delta[:, 1:, :].sum(axis=1).mean(axis=0)
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.6), layout="constrained")
    axes[0].plot(time, total, color=COLORS[value["metrics"]["variant"]], linewidth=1.7)
    axes[0].axhline(0.10, color="black", linestyle="--", linewidth=1, label="Budget 0.10")
    axes[0].set(xlabel="Simulation time (s)", ylabel="Total risk over horizon", ylim=(0, 0.115))
    axes[0].legend(fontsize=8)
    axes[0].grid(alpha=0.2)
    axes[1].bar(np.arange(len(OBSTACLE_NAMES)), mean, color=COLORS[value["metrics"]["variant"]], width=0.65)
    axes[1].set_xticks(range(len(OBSTACLE_NAMES)), OBSTACLE_NAMES, rotation=18)
    axes[1].set(ylabel="Mean horizon risk per obstacle")
    axes[1].grid(alpha=0.2, axis="y")
    fig.suptitle(f"Risk allocation / {LABELS[value['metrics']['variant']]}")
    fig.savefig(output / "04_risk_allocation.png", dpi=180)
    plt.close(fig)


def _renderer_setup(value: dict):
    import mujoco

    config = value["config"]
    plant = MuJoCoPlant(config.start, config.goal_position, config.obstacles,
                        mj_dt=config.mujoco_timestep_s)
    renderer = mujoco.Renderer(plant.model, height=600, width=960)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [1.50, 0.0, 1.25]
    camera.distance = 4.2
    camera.azimuth = 120.0
    camera.elevation = -18.0
    return mujoco, plant, renderer, camera


def _render_raw(mujoco, plant, renderer, camera, value: dict, time_s: float, trail: list[np.ndarray]):
    config, state = value["config"], _source_state(value, time_s)
    positions = np.array([obstacle_position(obstacle, time_s) for obstacle in config.obstacles])
    plant.set_state_13(state, obstacle_positions=positions)
    renderer.update_scene(plant.data, camera=camera)
    # Shadows of the replay trail can look like a second ground trajectory.
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
    drawer = SceneDrawer(mujoco, renderer.scene)
    start = np.array([config.start[axis] for axis in "xyz"])
    # Exactly one translucent sphere: the controller's first prediction node.
    tick = _tick_for(value, time_s)
    recording = value["recording"]
    means = recording["estimated_obstacle_predictions"][tick, :, 1, :]
    radii = recording["tightened_safety_radius_horizons"][tick, 1, :]
    for point, radius in zip(means, radii):
        drawer.sphere(point, float(radius), (0.15, 0.55, 1.0, 0.14))
    drawer.sphere(state[:3], DRONE_RADIUS, (0.95, 0.43, 0.08, 0.16) if value["metrics"]["variant"] == "joint_hard"
                  else (0.00, 0.55, 0.85, 0.16))
    drawer.sphere(start, 0.035, (0.2, 0.8, 1.0, 0.8))
    drawer.polyline(trail, (0.95, 0.43, 0.08, 0.9) if value["metrics"]["variant"] == "joint_hard"
                    else (0.0, 0.45, 0.85, 0.9), 0.012, limit=180)
    return renderer.render().copy(), state, positions


def _frame_panel(value: dict, time_s: float, state: np.ndarray, positions: np.ndarray,
                 frame: np.ndarray):
    from PIL import Image, ImageDraw, ImageFont

    regular = Path("C:/Windows/Fonts/segoeui.ttf")
    bold = Path("C:/Windows/Fonts/segoeuib.ttf")
    font_title = ImageFont.truetype(str(bold if bold.exists() else regular), 25)
    font_body = ImageFont.truetype(str(regular), 16)
    font_small = ImageFont.truetype(str(regular), 13)
    accent = COLORS[value["metrics"]["variant"]]
    tick = _tick_for(value, time_s)
    row = value["rows"][tick]
    goal = np.array([value["config"].goal_position[axis] for axis in "xyz"])
    distance = float(np.linalg.norm(state[:3] - goal))
    clearance = min(float(np.linalg.norm(state[:3] - p) - obstacle["radius"] - DRONE_RADIUS)
                    for p, obstacle in zip(positions, value["config"].obstacles))
    image = Image.fromarray(frame).resize((960, 600))
    canvas = Image.new("RGB", (1280, 720), "#0e1625")
    canvas.paste(image, (0, 70))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle((0, 0, 1280, 5), fill=accent)
    draw.text((22, 16), LABELS[value["metrics"]["variant"]].upper(), font=font_title, fill="white")
    draw.text((22, 50), "Five obstacles / S-slalom / seed 27 / 0.5x replay",
              font=font_body, fill="#c6d3e5")
    draw.rectangle((960, 70, 1279, 670), fill="#152135")
    x = 982
    draw.text((x, 90), f"SIMULATION  {time_s:4.2f} s", font=font_title, fill="white")
    draw.text((x, 124), "Not real-time 20 Hz", font=font_small, fill="#afc0d6")
    items = [
        ("Goal distance", f"{distance:.3f} m"),
        ("Clearance", f"{clearance:.3f} m"),
        ("End-to-end", f"{float(row['total_controller_time_ms']):.1f} ms"),
        ("Command", "FALLBACK" if row["fallback_active"] == "1" else "PRIMARY"),
    ]
    for index, (label, text) in enumerate(items):
        draw.text((x, 166 + index * 38), label, font=font_body, fill="#afc0d6")
        draw.text((x + 175, 166 + index * 38), text, font=font_body, fill="white")
    draw.text((x, 342), "Horizon risk by obstacle", font=font_body, fill="white")
    risk = value["delta"][tick, 1:, :].sum(axis=0) if value["delta"].ndim == 3 else np.zeros(5)
    for index, (name, allocated) in enumerate(zip(OBSTACLE_NAMES, risk)):
        y = 376 + index * 34
        draw.text((x, y), name, font=font_small, fill="#afc0d6")
        draw.rectangle((x + 112, y + 4, x + 220, y + 14), fill="#293b55")
        draw.rectangle((x + 112, y + 4, x + 112 + min(float(allocated) / 0.1, 1.0) * 108, y + 14), fill=accent)
        draw.text((x + 232, y), f"{float(allocated):.3f}", font=font_small, fill="white")
    draw.text((x, 562), f"Total delta: {risk.sum():.5f} / 0.10000", font=font_body, fill="white")
    draw.text((18, 690), "Recorded replay. Red: obstacles. Blue sphere: estimated k=1 chance-constraint boundary. "
                         "Gold: goal. Not a safety proof or a 20 Hz real-time claim.",
              font=font_small, fill="#c6d3e5")
    return canvas


def render_videos(pair: dict[str, dict], video_paths: dict[str, Path],
                  comparison_path: Path) -> list[str]:
    import imageio.v2 as imageio
    from PIL import Image, ImageDraw, ImageFont

    duration = min(value["duration_s"] for value in pair.values())
    fps = 30
    times = np.linspace(0.0, duration, int(round(duration / 0.5 * fps)) + 1)
    setups = {name: _renderer_setup(value) for name, value in pair.items()}
    trails = {name: [] for name in pair}
    writers = {
        name: imageio.get_writer(video_paths[name], fps=fps, codec="libx264", quality=8,
                                 macro_block_size=1, pixelformat="yuv420p")
        for name in VARIANTS
    }
    side = imageio.get_writer(comparison_path, fps=fps, codec="libx264", quality=8,
                              macro_block_size=1, pixelformat="yuv420p")
    font_title = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 24)
    font_body = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 15)
    try:
        for time_s in times:
            panels = {}
            raw_frames = {}
            for name in VARIANTS:
                mujoco, plant, renderer, camera = setups[name]
                frame, state, positions = _render_raw(mujoco, plant, renderer, camera, pair[name],
                                                      time_s, trails[name])
                trails[name].append(state[:3].copy())
                raw_frames[name] = frame
                panels[name] = _frame_panel(pair[name], time_s, state, positions, frame)
            for name in VARIANTS:
                writers[name].append_data(np.asarray(panels[name]))
            left = Image.fromarray(raw_frames["joint_hard"]).resize((640, 400))
            right = Image.fromarray(raw_frames["uniform"]).resize((640, 400))
            comparison = Image.new("RGB", (1280, 720), "#0e1625")
            comparison.paste(left, (0, 80))
            comparison.paste(right, (640, 80))
            draw = ImageDraw.Draw(comparison)
            draw.rectangle((0, 0, 1280, 5), fill="#D55E00")
            draw.text((20, 16), "JOINT-RISK (REQUIRED)  vs  ORIGINAL CC-MPC (SOFT-UNIFORM)",
                      font=font_title, fill="white")
            draw.text((20, 50), "Same scene, same seed 27, same replay time, 0.5x speed",
                      font=font_body, fill="#c6d3e5")
            draw.rectangle((0, 80, 640, 108), fill="#152135")
            draw.rectangle((640, 80, 1280, 108), fill="#152135")
            draw.text((16, 84), "JOINT-RISK", font=font_title, fill="#D55E00")
            draw.text((656, 84), "ORIGINAL CC-MPC", font=font_title, fill="#0072B2")
            draw.text((18, 505), f"t = {time_s:4.2f} s", font=font_title, fill="white")
            for row_index, name in enumerate(VARIANTS):
                metrics = pair[name]["metrics"]
                text = (f"{SHORT[name]} (run):  final goal {metrics['final_goal_distance_m']:.3f} m   |   "
                        f"min clearance {metrics['min_clearance_m']:.3f} m   |   "
                        f"time-to-tolerance {metrics['time_to_tolerance_s']:.2f} s   |   "
                        f"solver p95 {metrics['solver_p95_ms']:.1f} ms")
                draw.text((18, 545 + row_index * 34), text, font=font_body,
                          fill=COLORS[name])
            draw.text((18, 650), "Both panels show the same wall-clock replay time. "
                                 "Trajectories and metrics are recorded, not re-solved.",
                      font=font_body, fill="#afc0d6")
            draw.text((18, 690), "Not a 20 Hz real-time claim and not a safety proof.",
                      font=font_body, fill="#c6d3e5")
            side.append_data(np.asarray(comparison))
        for _ in range(30):
            for name in VARIANTS:
                writers[name].append_data(np.asarray(panels[name]))
            side.append_data(np.asarray(comparison))
    finally:
        for writer in writers.values():
            writer.close()
        side.close()
        for _, _, renderer, _ in setups.values():
            renderer.close()
    return [f"{name}.mp4" for name in VARIANTS] + ["side_by_side.mp4"]


def render_snapshots(name: str, value: dict, output: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont

    font_title = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 22)
    font_body = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 15)
    mujoco, plant, renderer, camera = _renderer_setup(value)
    times = [0.0, value["duration_s"] * 0.5, value["duration_s"]]
    panels = []
    trail: list[np.ndarray] = []
    try:
        for time_s in times:
            frame, state, _ = _render_raw(mujoco, plant, renderer, camera, value, time_s, trail)
            trail.append(state[:3].copy())
            image = Image.fromarray(frame).resize((640, 400))
            canvas = Image.new("RGB", (640, 460), "#0e1625")
            canvas.paste(image, (0, 38))
            draw = ImageDraw.Draw(canvas)
            draw.text((16, 8), f"{SHORT[name]} / t={time_s:4.2f} s", font=font_title, fill="white")
            draw.text((16, 442), f"goal {float(value['rows'][_tick_for(value, time_s)]['goal_distance_m']):.3f} m",
                      font=font_body, fill="#afc0d6")
            panels.append(canvas)
    finally:
        renderer.close()
    sheet = Image.new("RGB", (1920, 460), "#0e1625")
    for index, panel in enumerate(panels):
        sheet.paste(panel, (640 * index, 0))
    sheet.save(output / "03_snapshots.jpg", quality=90)
    sheet.save(output / "03_snapshots.png")


def compose_snapshot_comparison(output: Path) -> None:
    from PIL import Image, ImageDraw, ImageFont

    joint = Image.open(output / "joint_risk" / "images" / "03_snapshots.jpg").convert("RGB")
    uniform = Image.open(output / "ccmpc_original" / "images" / "03_snapshots.jpg").convert("RGB")
    canvas = Image.new("RGB", (1920, 980), "#0e1625")
    canvas.paste(joint, (0, 60))
    canvas.paste(uniform, (0, 520))
    draw = ImageDraw.Draw(canvas)
    title = ImageFont.truetype("C:/Windows/Fonts/segoeuib.ttf", 26)
    body = ImageFont.truetype("C:/Windows/Fonts/segoeui.ttf", 15)
    draw.text((20, 16), "FIVE-OBSTACLE S-SLALOM: JOINT-RISK (TOP) vs ORIGINAL CC-MPC (BOTTOM)",
              font=title, fill="white")
    draw.text((20, 46), "Same scene, seed 27, fixed camera, three replay times: 0 / 4 / 8 s",
              font=body, fill="#afc0d6")
    canvas.save(output / "comparison" / "images" / "04_snapshots.jpg", quality=90)


def plot_comparison(pair: dict[str, dict], output: Path) -> None:
    plt = _make_plot_axes()
    from matplotlib.patches import Circle

    config = pair["uniform"]["config"]
    goal = np.array([config.goal_position[axis] for axis in "xyz"])
    duration = float(pair["uniform"]["duration_s"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2), layout="constrained")

    ax = axes[0]
    for obstacle in config.obstacles:
        track = np.array([obstacle_position(obstacle, t) for t in np.linspace(0, duration, 161)])
        if obstacle["type"] == "static":
            point = track[0]
            ax.add_patch(Circle((point[0], point[1]), obstacle["radius"],
                                color="#9a4b3f", alpha=0.8))
        else:
            ax.plot(track[:, 0], track[:, 1], color=HALO_COLOR, linewidth=9,
                    alpha=0.12, solid_capstyle="round", zorder=0)
            ax.plot(track[:, 0], track[:, 1], color="#b08a84", linestyle="--", alpha=0.55)
    for name in VARIANTS:
        value = pair[name]
        positions = np.vstack([[config.start[axis] for axis in "xyz"], value["states"][:, :3]])
        ax.plot(positions[:, 0], positions[:, 1], color=COLORS[name], linewidth=2.2,
                label=f"{SHORT[name]} / {value['metrics']['time_to_tolerance_s']:.2f} s")
        ax.scatter(*positions[-1, [0, 1]], color=COLORS[name], marker="s", s=40)
    ax.annotate("CC-MPC hesitates here\n(~3 s before gate 2)", xy=(1.31, -0.30),
                xytext=(0.35, -1.18), fontsize=8, color=COLORS["uniform"],
                arrowprops={"arrowstyle": "->", "color": COLORS["uniform"], "lw": 1.0})
    ax.scatter(goal[0], goal[1], marker="*", s=170, color="#D6A000", label="Goal")
    ax.scatter(config.start["x"], config.start["y"], marker="o", color="black", s=28, label="Start")
    ax.set(xlabel="x (m)", ylabel="y (m)", aspect="equal")
    ax.grid(alpha=0.2)
    ax.set_title("XY: two controllers, one S-slalom")
    ax.legend(fontsize=8, loc="upper left")

    ax = axes[1]
    for name in VARIANTS:
        value = pair[name]
        states = np.asarray(value["states"])
        time = np.asarray(value["time"])
        count = min(len(states), len(time))
        ax.plot(time[:count], states[:count, 2], color=COLORS[name], linewidth=2.2,
                label=SHORT[name])
    ax.axhline(goal[2], color="#D6A000", linestyle="--", linewidth=1.2, label="Goal altitude")
    ax.set(xlabel="Simulation time (s)", ylabel="z (m)", ylim=(0.95, 1.45))
    ax.grid(alpha=0.2)
    ax.set_title("Altitude: both stay level")
    ax.legend(fontsize=8, loc="upper right")
    fig.suptitle("Five-obstacle S-slalom / seed 27 / margin 0.05 m / joint budget 0.10")
    fig.savefig(output / "01_trajectories_compare.png", dpi=180)
    plt.close(fig)

    metrics = {name: pair[name]["metrics"] for name in VARIANTS}
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    panels = [
        ("time_to_tolerance_s", "Time to 0.25 m (s)", None),
        ("final_goal_distance_m", "Final goal error (m)", None),
        ("min_clearance_m", "Minimum physical clearance (m)", None),
        ("solver_p95_ms", "Solver p95 (ms)", 50.0),
    ]
    for ax, (field, label, reference) in zip(axes.flat, panels):
        values = [metrics[name][field] for name in VARIANTS]
        bars = ax.bar([SHORT[name] for name in VARIANTS], values,
                      color=[COLORS[name] for name in VARIANTS], width=0.55)
        if reference is not None:
            ax.axhline(reference, color="#555555", linestyle="--", linewidth=1)
        for bar, value in zip(bars, values):
            ax.text(bar.get_x() + bar.get_width() / 2, value, f"{value:.2f}",
                    ha="center", va="bottom", fontsize=9)
        ax.set(ylabel=label)
        ax.grid(alpha=0.2, axis="y")
    fig.suptitle("Same seed, same scene, no re-solve: recorded metrics")
    fig.savefig(output / "02_metrics_compare.png", dpi=180)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8.5, 4.5), layout="constrained")
    for name in VARIANTS:
        total = pair[name]["delta"][:, 1:, :].sum(axis=(1, 2))
        ax.plot(pair[name]["time"], total, color=COLORS[name], linewidth=1.8, label=SHORT[name])
    ax.axhline(0.10, color="black", linestyle="--", linewidth=1, label="Budget 0.10")
    ax.set(xlabel="Simulation time (s)", ylabel="Total risk over horizon", ylim=(0, 0.115))
    ax.grid(alpha=0.2)
    ax.legend(fontsize=8)
    fig.suptitle("Risk budget used over time")
    fig.savefig(output / "03_risk_budget_compare.png", dpi=180)
    plt.close(fig)


def write_metrics(pair: dict[str, dict], output: Path) -> dict:
    payload = {
        "scenario": "five_through",
        "seed": 27,
        "description": "Five obstacles at z=1.2 m form an S-slalom; goal is at flight height x=3.0 m.",
        "not_real_time_20hz": True,
        "not_safety_proof": True,
        "variants": {name: pair[name]["metrics"] for name in VARIANTS},
    }
    (output / "metrics.json").write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    return payload


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-root", type=Path, default=ROOT / "outputs" / "joint_risk")
    parser.add_argument("--scenario", default="five_through")
    parser.add_argument("--tag", default="through_demo")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "five_obstacle_through")
    args = parser.parse_args()

    pair = {name: load_variant(args.run_root, name, args.scenario, args.tag) for name in VARIANTS}
    output = args.output
    for name in VARIANTS:
        folder = "joint_risk" if name == "joint_hard" else "ccmpc_original"
        (output / folder / "images").mkdir(parents=True, exist_ok=True)
        (output / folder / "video").mkdir(parents=True, exist_ok=True)
    (output / "comparison" / "images").mkdir(parents=True, exist_ok=True)
    (output / "comparison" / "videos").mkdir(parents=True, exist_ok=True)

    for name in VARIANTS:
        folder = "joint_risk" if name == "joint_hard" else "ccmpc_original"
        value = pair[name]
        plot_trajectory(value, output / folder / "images")
        plot_telemetry(value, output / folder / "images")
        plot_risk(value, output / folder / "images")
        (output / folder / "metrics.json").write_text(
            json.dumps(value["metrics"], indent=2, ensure_ascii=False), encoding="utf-8"
        )
        render_snapshots(name, value, output / folder / "images")

    compose_snapshot_comparison(output)
    plot_comparison(pair, output / "comparison" / "images")
    write_metrics(pair, output)

    video_paths = {
        name: output / ("joint_risk" if name == "joint_hard" else "ccmpc_original") / "video" / f"{name}.mp4"
        for name in VARIANTS
    }
    comparison_path = output / "comparison" / "videos" / "side_by_side.mp4"
    video_names = render_videos(pair, video_paths, comparison_path)

    provenance = {
        "scenario": args.scenario,
        "seed": 27,
        "recording_paths": {name: str(pair[name]["path"]) for name in VARIANTS},
        "sha256": {
            name: {
                file_name: hashlib.sha256((pair[name]["path"] / file_name).read_bytes()).hexdigest()
                for file_name in ("scenario.yaml", "telemetry.csv", "trajectory.npz", "experiment.json")
            }
            for name in VARIANTS
        },
        "videos": video_names,
        "replay_speed": 0.5,
        "camera": "fixed free camera, lookat [1.50, 0, 1.25], distance 4.2",
    }
    (output / "provenance.json").write_text(json.dumps(provenance, indent=2, ensure_ascii=False), encoding="utf-8")
    print(output.resolve())


if __name__ == "__main__":
    main()
