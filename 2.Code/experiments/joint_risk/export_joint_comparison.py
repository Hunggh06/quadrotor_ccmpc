#!/usr/bin/env python3
"""Export paired, recorded joint-hard/original soft-uniform videos and figures.

No controller solve and no changes to the source recordings. The selected case
is explicitly illustrative; the package also includes the full multi-seed data.
"""

import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
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


COLORS = {"uniform": "#0072B2", "joint_hard": "#D55E00"}
LABELS = {"uniform": "Original CC-MPC (soft-uniform)", "joint_hard": "Joint-hard"}
NAMES = ["Gate left", "Gate right", "Near crosser", "Far crosser", "Oscillator"]


def load_pair(seed):
    database = ROOT / "outputs/joint_risk_multiseed_v2_required"
    metrics = json.loads((database / "run_metrics.json").read_text(encoding="utf-8"))
    pair = {}
    for variant in ("uniform", "joint_hard"):
        row = next(r for r in metrics if r["scenario"] == "five_lowu_narrow"
                   and r["seed"] == seed and r["variant"] == variant)
        path = Path(row["recording_path"])
        recording = load_native_recording(path)
        payload = json.loads((path / "experiment.json").read_text(encoding="utf-8"))
        config = NativeMuJoCoConfig.from_mapping(recording["scenario"])
        time = np.array([float(r["time_s"]) for r in recording["rows"]])
        if variant == "joint_hard":
            delta = np.asarray([r["delta"] for r in payload["controller_report"]["records"]])
        else:
            delta = recording["risk_allocation_horizons"]
        pair[variant] = dict(
            metrics=row, path=path, recording=recording, config=config,
            time=time, states=recording["states"], delta=delta,
        )
    # Only the intended soft-vs-hard option and output paths may differ in the
    # serialized experiment config. Joint decision variables live in sidecar.
    normalized = []
    for value in pair.values():
        scenario = copy.deepcopy(value["recording"]["scenario"])
        scenario["controller"]["chance_constraints"].pop("soft_constraint")
        scenario["recording"].pop("output_dir", None)
        normalized.append(scenario)
    if normalized[0] != normalized[1]:
        raise RuntimeError("Selected recordings have other configuration differences")
    np.testing.assert_array_equal(pair["uniform"]["time"], pair["joint_hard"]["time"])
    return pair


def make_plots(pair, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import LogNorm
    from matplotlib.patches import Circle

    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    config = pair["uniform"]["config"]
    goal = np.array([config.goal_position[k] for k in "xyz"])
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), layout="constrained")
    for ax, vertical in zip(axes, (1, 2)):
        for obstacle in config.obstacles:
            track = np.array([obstacle_position(obstacle, t) for t in np.linspace(0, 6.5, 131)])
            if obstacle["type"] == "static":
                p = track[0]
                ax.add_patch(Circle((p[0], p[vertical]), obstacle["radius"], color="#777777", alpha=0.8))
                ax.add_patch(Circle((p[0], p[vertical]), obstacle["radius"] + DRONE_RADIUS,
                                    fill=False, linestyle=":", color="#777777"))
            else:
                ax.plot(track[:, 0], track[:, vertical], color="#999999", linestyle="--", alpha=0.5)
        for variant, value in pair.items():
            state = value["states"]
            start = [config.start[axis] for axis in "xyz"]
            positions = np.vstack([start, state[:, :3]])
            ax.plot(positions[:, 0], positions[:, vertical], label=LABELS[variant],
                    color=COLORS[variant], linewidth=2)
            ax.scatter(*positions[-1, [0, vertical]], color=COLORS[variant], marker="s", s=45)
        ax.scatter(goal[0], goal[vertical], marker="*", s=150, color="#D6A000", label="Goal")
        ax.scatter(config.start["x"], config.start["xyz"[vertical]], marker="o", color="black", s=25)
        ax.set(xlabel="x (m)", ylabel="yz"[vertical-1] + " (m)", aspect="equal")
        ax.grid(alpha=0.2)
    axes[0].set_title("XY: physical gate and moving-obstacle tracks")
    axes[1].set_title("XZ: actual altitude, not a 2D-only planner")
    axes[0].legend(fontsize=8, loc="upper left")
    fig.suptitle("Paired illustration: seed 27, same scene / margin 0.05 m / budget 0.10")
    fig.savefig(output / "01_trajectories.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    fields = [
        ("goal_distance_m", "Distance to goal (m)", 0.25),
        ("minimum_obstacle_clearance_m", "Physical obstacle clearance (m)", 0.),
        ("total_controller_time_ms", "End-to-end controller time (ms)", 50.),
        ("maximum_slack_m", "Post-solve diagnostic slack (m; not NLP slack)", 0.),
    ]
    for ax, (field, label, reference) in zip(axes.flat, fields):
        for variant, value in pair.items():
            series = [float(row[field]) for row in value["recording"]["rows"]]
            ax.plot(value["time"], series, color=COLORS[variant], label=LABELS[variant], linewidth=1.5)
        ax.axhline(reference, color="#555555", linestyle="--", linewidth=1)
        ax.set(xlabel="Simulation time (s)", ylabel=label)
        ax.grid(alpha=0.2)
    axes[0, 0].legend(fontsize=8)
    fig.suptitle("Recorded telemetry: same seed, no re-solve or smoothed metrics")
    fig.savefig(output / "02_goal_clearance_runtime_slack.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained")
    norm = LogNorm(vmin=1e-4, vmax=0.05)
    for column, (variant, value) in enumerate(pair.items()):
        for row, obstacle in enumerate((0, 1)):
            image = axes[row, column].imshow(
                value["delta"][:, 1:, obstacle], origin="lower", aspect="auto",
                extent=[0.5, 20.5, value["time"][0], value["time"][-1]],
                cmap="viridis", norm=norm,
            )
            axes[row, column].set(title=LABELS[variant] + " / " + NAMES[obstacle],
                                  xlabel="Future node k (1..20)", ylabel="Simulation time (s)")
    fig.colorbar(image, ax=axes, label="Per-cell risk allocation delta (log scale)", shrink=0.9)
    fig.suptitle("Raw optimizer allocation before supervisor; rejected ticks are not hidden")
    fig.savefig(output / "03_gate_risk_allocation.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5), layout="constrained")
    for variant, value in pair.items():
        axes[0].plot(value["time"], value["delta"][:, 1:, :].sum(axis=(1, 2)),
                     label=LABELS[variant], color=COLORS[variant])
    axes[0].axhline(0.1, color="black", linestyle="--", label="Budget = 0.10")
    axes[0].set(xlabel="Simulation time (s)", ylabel="Total risk allocated over horizon", ylim=(0, 0.115))
    axes[0].legend(fontsize=8)
    for j, (variant, value) in enumerate(pair.items()):
        mean = value["delta"][:, 1:, :].sum(axis=1).mean(axis=0)
        axes[1].bar(np.arange(5) + (j-0.5)*0.35, mean, width=0.35,
                    label=LABELS[variant], color=COLORS[variant])
    axes[1].set_xticks(range(5), NAMES, rotation=15)
    axes[1].set_ylabel("Mean horizon risk per obstacle")
    axes[1].legend(fontsize=8)
    fig.savefig(output / "04_risk_budget_and_distribution.png", dpi=180)
    plt.close(fig)
    shutil.copy2(ROOT / "outputs/joint_risk_multiseed/paired_raw.png", output / "05_all_seeds_context.png")


def interpolate_state(time, times, states):
    right = min(int(np.searchsorted(times, time, side="right")), len(times)-1)
    left = max(0, right-1)
    weight = float(np.clip((time-times[left]) / max(times[right]-times[left], 1e-12), 0, 1))
    result = (1-weight)*states[left] + weight*states[right]
    a, b = states[left, 6:10], states[right, 6:10]
    if np.dot(a, b) < 0:
        b = -b
    q = (1-weight)*a + weight*b
    result[6:10] = q / np.linalg.norm(q)
    return result


def chance_envelopes(recording, tick, node):
    """Return recorded tightened radii and their predicted belief centers.

    Do not attach a future radius to the current true obstacle center: moving
    obstacles and state-estimation error can shift the constraint center.
    """
    radii = np.asarray(recording["tightened_safety_radius_horizons"], dtype=float)
    predictions = np.asarray(recording["estimated_obstacle_predictions"], dtype=float)
    if radii.ndim != 3 or predictions.ndim != 4:
        raise ValueError("Recording lacks horizon radii or predicted obstacle centers")
    if not 1 <= node < radii.shape[1]:
        raise ValueError("Envelope node must be a constrained future node in the recording")
    centers = predictions[tick, :, node, :]
    radius = radii[tick, node, :]
    if centers.shape != (len(radius), 3) or not np.all(np.isfinite(centers)):
        raise ValueError("Envelope center/radius shape mismatch or nonfinite center")
    if not np.all(np.isfinite(radius)) or np.any(radius < 0):
        raise ValueError("Invalid recorded chance radius")
    return centers, radius


def draw_chance_envelopes(drawer, centers, radii):
    angles = np.linspace(0, 2*np.pi, 49)
    for center, radius in zip(centers, radii):
        if radius <= 0:
            continue
        drawer.sphere(center, float(radius), (0.18, 0.65, 1.0, 0.10))
        # Two thin great circles keep the shell legible without making the
        # physical obstacle look larger or opaque.
        for second_axis in (1, 2):
            points = np.repeat(center[None, :], len(angles), axis=0)
            points[:, 0] += radius * np.cos(angles)
            points[:, second_axis] += radius * np.sin(angles)
            drawer.polyline(points, (0.30, 0.78, 1.0, 0.32), 0.0018, limit=48)


def render_video(variant, value, output, *, risk_envelopes=False, risk_node=20):
    import imageio.v2 as imageio
    import mujoco
    from PIL import Image, ImageDraw, ImageFont

    config, states = value["config"], value["states"]
    goal = np.array([config.goal_position[k] for k in "xyz"])
    initial = np.zeros(13)
    initial[:3] = [config.start[k] for k in "xyz"]
    initial[6:10] = quat_from_euler(*(config.start.get(k, 0) for k in ("roll", "pitch", "yaw")))
    source_times, source_states = np.r_[0., value["time"]], np.vstack([initial, states])
    plant = MuJoCoPlant(config.start, config.goal_position, config.obstacles, mj_dt=config.mujoco_timestep_s)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = [1.25, 0.0, 1.0]
    camera.distance, camera.azimuth, camera.elevation = 3.8, 120, -30
    renderer = mujoco.Renderer(plant.model, height=600, width=960)
    regular = str(Path("C:/Windows/Fonts/segoeui.ttf"))
    bold = str(Path("C:/Windows/Fonts/segoeuib.ttf"))
    fonts = {"title": ImageFont.truetype(bold, 27), "large": ImageFont.truetype(bold, 25),
             "body": ImageFont.truetype(regular, 17), "small": ImageFont.truetype(regular, 14)}
    filename = "01_joint_hard_seed27.mp4" if variant == "joint_hard" else "02_original_soft_uniform_seed27.mp4"
    if risk_envelopes:
        filename = filename.replace(".mp4", "_risk_envelopes.mp4")
    accent = COLORS[variant]
    metrics = value["metrics"]
    trail = []
    fps = 30
    simulation_times = np.linspace(0, 6.5, 391)  # 0.5x playback, same duration for both runs
    saved = []
    try:
        with imageio.get_writer(output / filename, fps=fps, codec="libx264", quality=8,
                                macro_block_size=1, pixelformat="yuv420p") as writer:
            for frame_index, time in enumerate(simulation_times):
                tick = min(max(int(np.searchsorted(value["time"], time, side="right"))-1, 0), len(states)-1)
                state = interpolate_state(time, source_times, source_states)
                positions = np.array([obstacle_position(o, time) for o in config.obstacles])
                plant.set_state_13(state, obstacle_positions=positions)
                renderer.update_scene(plant.data, camera=camera)
                drawer = SceneDrawer(mujoco, renderer.scene)
                trail.append(state[:3].copy())
                color = (0.0, 0.55, 0.85, 0.9) if variant == "uniform" else (0.95, 0.43, 0.08, 0.9)
                drawer.polyline(trail, color, 0.012, limit=160)
                drawer.sphere(state[:3], DRONE_RADIUS, (*color[:3], 0.16))
                drawer.sphere(initial[:3], 0.035, (0.2, 0.8, 1.0, 0.8))
                if risk_envelopes:
                    centers, radii = chance_envelopes(value["recording"], tick, risk_node)
                    draw_chance_envelopes(drawer, centers, radii)
                    # Display-only volumes must not look like solid hazards
                    # by casting large dark shadows onto the physical floor.
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                canvas = Image.new("RGB", (1280, 720), "#0e1625")
                canvas.paste(Image.fromarray(renderer.render().copy()), (0, 90))
                draw = ImageDraw.Draw(canvas)
                draw.rectangle((0, 0, 1280, 5), fill=accent)
                draw.text((22, 16), LABELS[variant].upper(), font=fonts["title"], fill="white")
                draw.text((22, 54), "Same scene / seed 27 / margin 0.05 m / N=20 / risk budget 0.10",
                          font=fonts["body"], fill="#c6d3e5")
                draw.rectangle((960, 90, 1279, 689), fill="#152135")
                x = 982
                draw.text((x, 108), f"SIMULATION  {time:4.2f} s", font=fonts["large"], fill="white")
                draw.text((x, 144), "Replay 0.5x | interpolated display", font=fonts["small"], fill="#afc0d6")
                row = value["recording"]["rows"][tick]
                reached = metrics["time_to_tolerance_s"] is not None and time >= metrics["time_to_tolerance_s"]
                status = "GOAL REACHED" if reached else ("NOT REACHED IN 6.5 s" if time >= 6.5 else "IN FLIGHT")
                draw.text((x, 180), status, font=fonts["body"], fill="#64ddb3" if reached else "#f2c46d")
                distance = float(np.linalg.norm(state[:3]-goal))
                clearance = min(float(np.linalg.norm(state[:3]-p))-o["radius"]-DRONE_RADIUS
                                for p, o in zip(positions, config.obstacles))
                items = [("Goal distance", f"{distance:.3f} m"),
                         ("Physical clearance", f"{clearance:.3f} m"),
                         ("Controller time*", f"{float(row['total_controller_time_ms']):.1f} ms"),
                         ("Applied command*", "FALLBACK" if row["fallback_active"] == "1" else "PRIMARY")]
                for j, (label, text) in enumerate(items):
                    draw.text((x, 226+j*38), label, font=fonts["body"], fill="#afc0d6")
                    draw.text((x+185, 226+j*38), text, font=fonts["body"], fill="white")
                draw.text((x, 389), "Raw horizon risk by obstacle*", font=fonts["body"], fill="white")
                risk = value["delta"][tick, 1:, :].sum(axis=0)
                for j, (name, allocated) in enumerate(zip(NAMES, risk)):
                    y = 423+j*37
                    draw.text((x, y), name, font=fonts["small"], fill="#afc0d6")
                    draw.rectangle((x+118, y+5, x+230, y+15), fill="#293b55")
                    draw.rectangle((x+118, y+5, x+118+min(allocated/0.1, 1)*112, y+15), fill=accent)
                    draw.text((x+242, y), f"{allocated:.3f}", font=fonts["small"], fill="white")
                draw.text((x, 614), f"Total delta: {risk.sum():.5f} / 0.10000", font=fonts["body"], fill="white")
                note = (f"Blue shells: k={risk_node} / +{risk_node*config.mpc_timestep_s:.2f} s"
                        if risk_envelopes else "*Latest available logged control tick")
                draw.text((x, 650), note, font=fonts["small"], fill="#afc0d6")
                footer = ("Blue shells: recorded chance constraints at predicted centers, NOT physical obstacles | NOT real-time 20 Hz"
                          if risk_envelopes else
                          "Recorded simulation, NOT 20 Hz real-time | Red spheres: physical obstacles | Gold: goal")
                draw.text((18, 698), footer,
                          font=fonts["small"], fill="#c6d3e5")
                if frame_index == 0:
                    for _ in range(30):
                        writer.append_data(np.asarray(canvas))
                writer.append_data(np.asarray(canvas))
                if frame_index in (0, 195, 390):
                    saved.append(canvas.copy())
            for _ in range(60):
                writer.append_data(np.asarray(canvas))
    finally:
        renderer.close()
    sheet = Image.new("RGB", (1280, 720*3))
    for i, frame in enumerate(saved):
        sheet.paste(frame, (0, 720*i))
    sheet.save(output / (filename.removesuffix(".mp4") + "_frames.jpg"), quality=90)
    return filename


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seed", type=int, default=27)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs/joint_vs_original")
    parser.add_argument("--plots-only", action="store_true")
    parser.add_argument("--videos-only", action="store_true")
    parser.add_argument("--risk-envelopes", action="store_true")
    parser.add_argument("--risk-node", type=int, default=1, choices=range(1, 21),
                        help="Constrained horizon node; default 1 is the first future step")
    args = parser.parse_args()
    if args.plots_only and args.videos_only:
        parser.error("--plots-only and --videos-only are mutually exclusive")
    if args.seed != 27:
        raise ValueError("This curated demo is explicitly documented as seed 27")
    args.output.mkdir(parents=True, exist_ok=True)
    pair = load_pair(args.seed)
    if not args.videos_only:
        make_plots(pair, args.output)
    videos = []
    if args.plots_only:
        suffix = "_risk_envelopes.mp4" if args.risk_envelopes else ".mp4"
        videos = [name for name in ("01_joint_hard_seed27" + suffix, "02_original_soft_uniform_seed27" + suffix)
                  if (args.output / name).exists()]
    else:
        for variant in ("joint_hard", "uniform"):
            videos.append(render_video(variant, pair[variant], args.output,
                                       risk_envelopes=args.risk_envelopes, risk_node=args.risk_node))
            print("Rendered", videos[-1], flush=True)
    provenance = {
        "scene": "five_lowu_narrow", "seed": 27, "duration_s": 6.5,
        "illustration_only": True, "same_serialized_config_except_soft_flag_and_output": True,
        "margin_m": 0.05, "default_project_margin_m": 0.30,
        "not_full_default_config": True, "simulation_replay_speed": 0.5,
        "videos": videos, "runs": {},
        "chance_envelopes": {
            "enabled": args.risk_envelopes, "node": args.risk_node,
            "first_constrained_node": 1,
            "node_zero": "current seed state; not a chance constraint",
            "horizon_s": args.risk_node * pair["uniform"]["config"].mpc_timestep_s,
            "centers": "recorded estimated_obstacle_predictions at selected future node",
            "radii": "recorded tightened_safety_radius_horizons; geometry + margin + Gaussian tightening",
            "time_sampling": "piecewise constant at logged tick; no radius interpolation",
        },
    }
    for variant, value in pair.items():
        provenance["runs"][variant] = {
            "recording_path": str(value["path"]), "metrics": value["metrics"],
            "sha256": {name: hashlib.sha256((value["path"] / name).read_bytes()).hexdigest()
                       for name in ("scenario.yaml", "telemetry.csv", "trajectory.npz", "experiment.json")},
        }
    provenance_name = "provenance_risk_envelopes.json" if args.risk_envelopes else "provenance.json"
    (args.output / provenance_name).write_text(json.dumps(provenance, indent=2, ensure_ascii=False), encoding="utf-8")
    print(args.output, flush=True)


if __name__ == "__main__":
    main()
