#!/usr/bin/env python3
"""Sinh bao cao cho mot ban ghi ``quadrotor-mpc-native`` (13 trang thai).

Chi lam hai viec: doc ban ghi, roi goi dung cac ham ve co san cua repo trong
``visualization.reporting.native_plots``. Khong tu ve lai tu dau.

    python scripts/report_native_run.py                    # ban ghi moi nhat
    python scripts/report_native_run.py outputs/native/<run>

Xuat ra trong thu muc ban ghi:
    report.png               loi do thi tom tat (matplotlib)
    report_3d.html           quy dao 3D co nut Play  (build_figure)
    report_timeseries.html   vi tri / goc Euler / dieu khien / clearance
                             (build_timeseries_figure)
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from simulation.application.native.telemetry import load_native_recording
from simulation.control.ccmpc_13state.core import quat_to_euler
from simulation.interfaces.desktop.viewer import NativeMuJoCoConfig
from visualization.reporting.native_plots import build_figure, build_timeseries_figure


def latest_run(root: Path) -> Path:
    runs = sorted(p for p in root.iterdir() if (p / "telemetry.csv").is_file())
    if not runs:
        raise SystemExit(f"khong tim thay ban ghi nao trong {root}")
    return runs[-1]


def legacy_obstacles(obstacles) -> list[dict]:
    """Doi schema vat can sang dang ma native_plots hieu duoc.

    ``native_plots._obstacle_position`` chi biet vat can tinh (x, y, z) hoac
    dao dong theo truc y. Vat can dong o day dao dong theo truc z nen ta dat
    chung o vi tri xuat phat; chuyen dong that da co trong video MP4.
    """
    out = []
    for obstacle in obstacles:
        if "x" in obstacle:
            x, y, z = obstacle["x"], obstacle["y"], obstacle["z"]
        else:
            motion = obstacle["motion"]
            centre = motion.get("center") or motion["initial_position"]
            x, y, z = centre
        out.append({"type": "static", "x": x, "y": y, "z": z, "radius": obstacle["radius"]})
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", nargs="?", help="thu muc ban ghi; mac dinh moi nhat")
    args = parser.parse_args()

    run_dir = Path(args.run_dir) if args.run_dir else latest_run(Path("outputs/native"))
    recording = load_native_recording(run_dir)
    config = NativeMuJoCoConfig.from_mapping(recording["scenario"])
    rows, states = recording["rows"], np.asarray(recording["states"], dtype=float)
    n = len(rows)

    def col(name: str) -> np.ndarray:
        return np.array([float(r.get(name, "nan")) for r in rows], dtype=float)

    times = col("time_s")
    goal = {k: float(config.goal_position[k]) for k in "xyz"}
    start = {k: float(config.start[k]) for k in "xyz"}
    obstacles = legacy_obstacles(config.obstacles)

    result = {
        "t": times,
        "pos": states[:, :3],
        "euler": np.array([quat_to_euler(states[i, 6:10]) for i in range(n)], dtype=float),
        "u": np.column_stack([col("thrust_deviation"), col("tau_x"), col("tau_y"), col("tau_z")]),
        "clearance": col("min_clearance_m"),
    }

    # --- do thi dung ham co san cua repo ---
    path_3d = run_dir / "report_3d.html"
    build_figure(result, start, goal, obstacles).write_html(path_3d, include_plotlyjs="cdn")
    print(f"-> {path_3d}")

    path_ts = run_dir / "report_timeseries.html"
    build_timeseries_figure(result, goal).write_html(path_ts, include_plotlyjs="cdn")
    print(f"-> {path_ts}")

    # --- anh tom tat ---
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    solver_ms = col("solver_time_ms")
    fig = plt.figure(figsize=(13, 9))
    axes = [
        fig.add_subplot(2, 2, 1, projection="3d"),
        fig.add_subplot(2, 2, 2),
        fig.add_subplot(2, 2, 3),
        fig.add_subplot(2, 2, 4),
    ]
    fig.suptitle(
        f"{recording['scenario'].get('name', 'native run')} — {n} buoc | "
        f"clearance nho nhat {result['clearance'].min():.3f} m | "
        f"giai p99 {np.percentile(solver_ms, 99):.0f} ms"
    )

    ax = axes[0]
    ax.plot(*result["pos"].T, lw=2, label="quadrotor")
    ax.scatter(*[start[k] for k in "xyz"], c="green", s=50, label="start")
    ax.scatter(*[goal[k] for k in "xyz"], c="gold", s=120, marker="*", edgecolor="k", label="goal")
    for obstacle in obstacles:
        ax.scatter(obstacle["x"], obstacle["y"], obstacle["z"], c="k", marker="x", s=45)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title("Quy dao 3D"); ax.legend(fontsize=8)

    ax = axes[1]
    ax.plot(times, result["clearance"], c="#d62728", lw=2)
    ax.set_xlabel("thoi gian [s]"); ax.set_ylabel("clearance [m]")
    ax.set_title("Khoang cach an toan"); ax.grid(alpha=0.3)

    ax = axes[2]
    ax.plot(times, col("total_controller_time_ms"), c="#9467bd", lw=2, label="tong")
    ax.plot(times, solver_ms, c="#1f77b4", lw=1.3, label="giai NLP")
    ax.axhline(50.0, c="r", ls="--", lw=1.3, label="han 50 ms")
    ax.set_xlabel("thoi gian [s]"); ax.set_ylabel("ms")
    ax.set_title("Thoi gian giai"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    ax = axes[3]
    ax.plot(times, col("maximum_projected_uncertainty_m") * 100, c="#ff7f0e", label="Σ (cm)")
    ax.plot(times, col("maximum_tightened_safety_radius_m") * 100, c="#2ca02c", label="ban kinh siet (cm)")
    ax.plot(times, col("maximum_slack_m") * 1000, c="#d62728", label="slack (mm)")
    ax.set_xlabel("thoi gian [s]"); ax.set_ylabel("cm / mm")
    ax.set_title("Co may chance-constraint"); ax.grid(alpha=0.3); ax.legend(fontsize=8)

    fig.tight_layout()
    png = run_dir / "report.png"
    fig.savefig(png, dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"-> {png}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
