#!/usr/bin/env python3
"""Render a recorded native MuJoCo run to an MP4/GIF without invoking the solver.

The recording written by ``quadrotor-mpc-native`` is replayed through the same
``MuJoCoPlant`` model and drawn with MuJoCo's offscreen renderer, so no window,
Qt panel or do-mpc solve is needed.

    python scripts/export_native_video.py                       # newest run
    python scripts/export_native_video.py outputs/native/<run> --camera orbit
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from quadrotor_mpc.application.native.telemetry import load_native_recording
from quadrotor_mpc.control.nmpc.core import DRONE_RADIUS
from quadrotor_mpc.infrastructure.mujoco.plant import MuJoCoPlant
from quadrotor_mpc.interfaces.desktop.viewer import NativeMuJoCoConfig

WIDTH, HEIGHT = 960, 720  # the plant MJCF declares this offscreen buffer size


def latest_run(root: Path) -> Path:
    runs = sorted(p for p in root.iterdir() if (p / "telemetry.csv").exists())
    if not runs:
        raise SystemExit(f"no native recordings found in {root}")
    return runs[-1]


class SceneDrawer:
    def __init__(self, mujoco, scene) -> None:
        self.mj = mujoco
        self.scene = scene

    def _next(self):
        if self.scene.ngeom >= self.scene.maxgeom:
            return None
        geom = self.scene.geoms[self.scene.ngeom]
        self.scene.ngeom += 1
        return geom

    def sphere(self, pos, radius, rgba) -> None:
        geom = self._next()
        if geom is None:
            return
        self.mj.mjv_initGeom(
            geom,
            type=self.mj.mjtGeom.mjGEOM_SPHERE,
            size=np.array([radius, 0, 0], dtype=np.float64),
            pos=np.asarray(pos, dtype=np.float64),
            mat=np.eye(3).ravel(),
            rgba=np.asarray(rgba, dtype=np.float32),
        )

    def polyline(self, points, rgba, radius, limit=200) -> None:
        points = np.asarray(points, dtype=np.float64)
        if len(points) < 2:
            return
        stride = max(1, math.ceil((len(points) - 1) / limit))
        for i in range(0, len(points) - 1, stride):
            a, b = points[i], points[min(i + stride, len(points) - 1)]
            if np.linalg.norm(b - a) < 1e-9:
                continue
            geom = self._next()
            if geom is None:
                return
            self.mj.mjv_initGeom(
                geom,
                type=self.mj.mjtGeom.mjGEOM_CAPSULE,
                size=np.array([radius, 0, 0], dtype=np.float64),
                pos=np.zeros(3),
                mat=np.eye(3).ravel(),
                rgba=np.asarray(rgba, dtype=np.float32),
            )
            self.mj.mjv_connector(geom, self.mj.mjtGeom.mjGEOM_CAPSULE, radius, a, b)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", nargs="?", help="recording directory (default: newest in outputs/native)")
    parser.add_argument("-o", "--output", help="output file (.mp4 or .gif); default <run_dir>/video.mp4")
    parser.add_argument("--camera", choices=("follow", "orbit", "fixed"), default="follow")
    parser.add_argument("--distance", type=float, default=5.0)
    parser.add_argument("--azimuth", type=float, default=60.0)
    parser.add_argument("--elevation", type=float, default=-25.0)
    parser.add_argument("--speed", type=float, default=1.0, help="playback speed (1 = real time)")
    parser.add_argument("--no-envelopes", action="store_true", help="hide obstacle safety spheres")
    args = parser.parse_args()

    import mujoco

    run_dir = Path(args.run_dir) if args.run_dir else latest_run(Path("outputs/native"))
    recording = load_native_recording(run_dir)
    config = NativeMuJoCoConfig.from_mapping(recording["scenario"])
    rows, states = recording["rows"], recording["states"]
    n = len(rows)
    obstacles = [dict(o) for o in config.obstacles]
    plant = MuJoCoPlant(config.start, config.goal_position, obstacles, mj_dt=config.mujoco_timestep_s)
    goal = np.array([config.goal_position[k] for k in "xyz"], dtype=float)
    radii = [o["radius"] + config.safety_margin + DRONE_RADIUS for o in obstacles]
    has_pred = len(recording["predicted_positions"]) == n
    fps = args.speed / config.mpc_timestep_s

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.azimuth, cam.elevation = args.distance, args.azimuth, args.elevation
    if args.camera == "fixed":
        cam.lookat[:] = (goal + states[0, :3]) / 2
        cam.distance = max(args.distance, 1.6 * float(np.linalg.norm(goal - states[0, :3])) + 2.0)

    out = Path(args.output) if args.output else run_dir / "video.mp4"
    try:
        import imageio.v2 as imageio
    except ImportError as exc:
        raise SystemExit("pip install imageio imageio-ffmpeg") from exc
    writer_kwargs = {"fps": fps}
    if out.suffix.lower() == ".mp4":
        writer_kwargs.update(codec="libx264", quality=8, macro_block_size=1, pixelformat="yuv420p")
    writer = imageio.get_writer(out, **writer_kwargs)

    try:
        from PIL import Image, ImageDraw
    except ImportError:
        Image = ImageDraw = None

    renderer = mujoco.Renderer(plant.model, height=HEIGHT, width=WIDTH)
    trail: list[np.ndarray] = []
    try:
        for i in range(n):
            row, state = rows[i], states[i]
            obs_pred = recording["obstacle_predictions"][i]  # (n_obs, H+1, 3)
            plant.set_state_13(state, obstacle_positions=obs_pred[:, 0, :])
            pos = state[:3]
            trail.append(pos.copy())

            if args.camera == "follow":
                cam.lookat[:] = pos
            elif args.camera == "orbit":
                cam.lookat[:] = pos
                cam.azimuth = args.azimuth + 360.0 * i / max(n - 1, 1)
            renderer.update_scene(plant.data, camera=cam)
            draw = SceneDrawer(mujoco, renderer.scene)
            draw.sphere(goal, 0.12, (1.0, 0.78, 0.05, 0.9))
            if not args.no_envelopes:
                for p, r in zip(obs_pred[:, 0, :], radii):
                    draw.sphere(p, r, (1.0, 0.45, 0.05, 0.12))
            for pred in obs_pred:
                draw.polyline(pred, (1.0, 0.4, 0.15, 0.55), 0.009, 40)
            draw.polyline(trail, (0.2, 0.55, 1.0, 0.75), 0.018)
            if has_pred and recording["predicted_positions"][i].size:
                draw.polyline(recording["predicted_positions"][i], (0.15, 1.0, 0.35, 0.8), 0.012, 80)

            frame = renderer.render().copy()
            if Image is not None:
                img = Image.fromarray(frame)
                ImageDraw.Draw(img).text(
                    (12, 10),
                    f"t={float(row['time_s']):5.2f}s  clearance={float(row['min_clearance_m']):.3f} m  "
                    f"goal dist={np.linalg.norm(pos - goal):.2f} m",
                    fill=(255, 255, 255),
                )
                frame = np.asarray(img)
            writer.append_data(frame)
    finally:
        writer.close()
        renderer.close()
    print(f"frames: {n}  fps: {fps:.1f}  ->  {out.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
