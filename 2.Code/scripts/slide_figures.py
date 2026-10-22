"""Sinh bo hinh anh cho slide thuyet trinh tu mot run recording.

Cach dung
---------
    python scripts/slide_figures.py <run_dir> --tag dyn --out <thu_muc_anh>
    python scripts/slide_figures.py --compare <r1> <r2> <r3> --tag noise --out <...>

Script nay CHI doc lai recording do `quadrotor-mpc-native` ghi ra
(telemetry.csv + trajectory.npz + scenario.yaml) va ve hinh. No khong chay
lai mo phong va khong sua bat ky file recording nao.
"""

from __future__ import annotations

import argparse
import csv
import math
import pathlib
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import yaml
from matplotlib.lines import Line2D
from matplotlib.patches import Circle

# --------------------------------------------------------------------------
# Bang mau / kieu dang dung chung
# --------------------------------------------------------------------------

PURPLE = "#7C3AED"
DARK = "#3B0764"
LIGHT = "#C4B5FD"
TEAL = "#0E7490"
AMBER = "#D97706"
RED = "#DC2626"
GREEN = "#15803D"
GREY = "#6B7280"

OBS_COLOURS = ["#DC2626", "#EA580C", "#CA8A04", "#0E7490", "#BE185D"]

plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "axes.grid": True,
        "grid.alpha": 0.22,
        "grid.linestyle": "-",
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.titlelocation": "left",
        "axes.titleweight": "bold",
        "axes.titlesize": 11.5,
        "axes.labelsize": 10,
        "xtick.labelsize": 9,
        "ytick.labelsize": 9,
        "legend.fontsize": 9,
        "legend.frameon": False,
    }
)


# --------------------------------------------------------------------------
# Doc recording
# --------------------------------------------------------------------------


def load_telemetry(run: pathlib.Path) -> list[dict[str, str]]:
    with (run / "telemetry.csv").open(newline="", encoding="utf-8") as fh:
        return list(csv.DictReader(fh))


def series(rows: list[dict[str, str]], name: str) -> np.ndarray:
    out = []
    for r in rows:
        try:
            out.append(float(r.get(name, "nan")))
        except (TypeError, ValueError):
            out.append(float("nan"))
    return np.asarray(out, dtype=float)


def load_npz(run: pathlib.Path) -> dict[str, np.ndarray]:
    with np.load(run / "trajectory.npz", allow_pickle=False) as z:
        return {k: z[k] for k in z.files}


def load_scenario(run: pathlib.Path) -> dict:
    for name in ("scenario.yaml", "summary.yaml"):
        p = run / name
        if p.exists():
            data = yaml.safe_load(p.read_text(encoding="utf-8"))
            if isinstance(data, dict) and "obstacles" in data:
                return data
    return {}


def parse_obstacles(scenario: dict) -> list[dict]:
    out = []
    for raw in scenario.get("obstacles") or []:
        if "position" in raw:
            pos = np.asarray(raw["position"], dtype=float)
        else:
            pos = np.asarray(
                [raw.get("x", 0.0), raw.get("y", 0.0), raw.get("z", 0.0)], dtype=float
            )
        out.append(
            {
                "name": str(raw.get("name", "vat_can")),
                "radius": float(raw.get("radius", 0.0)),
                "type": str(raw.get("type", "static")),
                "position": pos,
                "motion": raw.get("motion") or {},
            }
        )
    return out


def obstacle_track(obstacle: dict, times: np.ndarray) -> np.ndarray:
    """Vi tri THAT cua vat can theo mo hinh chuyen dong giai tich."""
    motion = obstacle.get("motion") or {}
    kind = str(motion.get("type", "")).strip()
    if kind == "constant_velocity":
        p0 = np.asarray(motion["initial_position"], dtype=float)
        vel = np.asarray(motion["velocity"], dtype=float)
        return p0[None, :] + times[:, None] * vel[None, :]
    if kind == "sinusoidal":
        centre = np.asarray(motion["center"], dtype=float)
        amp = np.asarray(motion["amplitude"], dtype=float)
        period = float(motion.get("period_s", 1.0))
        phase = float(motion.get("phase_rad", 0.0))
        wave = np.sin(2.0 * math.pi * times / period + phase)
        return centre[None, :] + wave[:, None] * amp[None, :]
    return np.repeat(obstacle["position"][None, :], times.size, axis=0)


def start_goal(scenario: dict) -> tuple[np.ndarray, np.ndarray | None]:
    start_raw = scenario.get("start") or {}
    if isinstance(start_raw, dict):
        start = np.asarray(
            [start_raw.get("x", 0.0), start_raw.get("y", 0.0), start_raw.get("z", 1.0)],
            dtype=float,
        )
    else:
        start = np.asarray(start_raw[:3], dtype=float)
    goal_raw = scenario.get("goal") or {}
    goal = None
    if isinstance(goal_raw, dict) and "position" in goal_raw:
        pos = goal_raw["position"]
        if isinstance(pos, dict):
            goal = np.asarray(
                [pos.get("x", 0.0), pos.get("y", 0.0), pos.get("z", 0.0)], dtype=float
            )
        else:
            goal = np.asarray(pos, dtype=float)
    return start, goal


def pct(values: np.ndarray, q: float) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return float("nan")
    return float(np.percentile(values, q))


def draw_sphere(ax, centre, radius, colour, alpha=0.16, resolution=18):
    u = np.linspace(0, 2 * np.pi, resolution)
    v = np.linspace(0, np.pi, resolution)
    x = centre[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = centre[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = centre[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=colour, alpha=alpha, linewidth=0, shade=False)


def set_equal_3d(ax, arrays, pad=0.25):
    pts = np.vstack([a.reshape(-1, 3) for a in arrays if a.size])
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    centre = 0.5 * (lo + hi)
    span = float(np.max(hi - lo)) * (1.0 + pad)
    half = span / 2.0
    ax.set_xlim(centre[0] - half, centre[0] + half)
    ax.set_ylim(centre[1] - half, centre[1] + half)
    ax.set_zlim(centre[2] - half, centre[2] + half)
    ax.set_box_aspect((1, 1, 1))


# --------------------------------------------------------------------------
# Cac hinh
# --------------------------------------------------------------------------


def fig_3d(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    true_p = npz["state_13"][:, :3]
    est_p = npz["estimated_state_13"][:, :3]
    horizon = npz["predicted_positions"]
    t = series(tele, "time_s")
    n = true_p.shape[0]

    fig = plt.figure(figsize=(11.2, 5.6), dpi=170)
    ax = fig.add_subplot(111, projection="3d")

    tracks = [obstacle_track(o, t) for o in obstacles]
    for k, (o, tr) in enumerate(zip(obstacles, tracks)):
        colour = OBS_COLOURS[k % len(OBS_COLOURS)]
        draw_sphere(ax, tr[0], o["radius"], colour, alpha=0.20)
        if o["type"] != "static":
            ax.plot(tr[:, 0], tr[:, 1], tr[:, 2], color=colour, lw=1.4,
                    ls=":", label=f"vật cản {o['name']}")

    for idx in range(0, n, max(1, n // 6)):
        h = horizon[idx]
        ax.plot(h[:, 0], h[:, 1], h[:, 2], color=LIGHT, lw=1.0, alpha=0.9)

    ax.plot(est_p[:, 0], est_p[:, 1], est_p[:, 2], color=GREY, lw=1.1,
            ls="--", alpha=0.9, label="vị trí ước lượng (EKF)")
    ax.plot(true_p[:, 0], true_p[:, 1], true_p[:, 2], color=PURPLE, lw=3.0,
            label="quỹ đạo bay thực tế", zorder=6)
    ax.scatter(*true_p[0], color=GREEN, s=55, marker="o", zorder=8,
               label="xuất phát")
    if goal is not None:
        ax.scatter(*goal, color=AMBER, s=150, marker="*", zorder=8, label="đích")
    ax.scatter(*true_p[-1], color=DARK, s=45, marker="s", zorder=8,
               label="vị trí kết thúc")

    set_equal_3d(ax, [true_p, *tracks, horizon[:: max(1, n // 6)]])
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_title("Quỹ đạo bay 3D (đường mờ = horizon dự đoán của MPC)")
    ax.legend(loc="upper left", bbox_to_anchor=(-0.05, 0.98), fontsize=8.5)
    ax.view_init(elev=24, azim=-58)
    ax.grid(True, alpha=0.18)
    fig.tight_layout()
    fig.savefig(out / f"{tag}_3d.png")
    plt.close(fig)


def fig_top(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    true_p = npz["state_13"][:, :3]
    horizon = npz["predicted_positions"]
    t = series(tele, "time_s")
    n = true_p.shape[0]

    fig, ax = plt.subplots(figsize=(10.4, 6.0), dpi=170)
    ax.grid(True, alpha=0.25, zorder=0)

    for idx in range(0, n, max(1, n // 8)):
        h = horizon[idx]
        ax.plot(h[:, 0], h[:, 1], color=LIGHT, lw=1.1, alpha=0.85, zorder=2)

    tracks = [obstacle_track(o, t) for o in obstacles]
    for k, (o, tr) in enumerate(zip(obstacles, tracks)):
        colour = OBS_COLOURS[k % len(OBS_COLOURS)]
        ax.add_patch(
            Circle(tr[0, :2], o["radius"], facecolor=colour, alpha=0.25,
                   edgecolor=colour, lw=1.4, zorder=3)
        )
        if o["type"] != "static":
            ax.plot(tr[:, 0], tr[:, 1], color=colour, lw=1.6, ls=":", zorder=3)
            ax.add_patch(
                Circle(tr[-1, :2], o["radius"], facecolor=colour, alpha=0.10,
                       edgecolor=colour, lw=1.0, ls="--", zorder=3)
            )

    ax.plot(true_p[:, 0], true_p[:, 1], color=PURPLE, lw=3.0, zorder=6,
            label="quỹ đạo bay thực tế")
    ax.scatter(*true_p[0, :2], color=GREEN, s=70, zorder=8, label="xuất phát")
    ax.scatter(*true_p[-1, :2], color=DARK, s=55, marker="s", zorder=8,
               label="vị trí kết thúc")
    if goal is not None:
        ax.scatter(*goal[:2], color=AMBER, s=230, marker="*", zorder=8, label="đích")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.set_title("Nhìn từ trên xuống (mặt phẳng X–Y)")
    ax.legend(loc="upper left", fontsize=9)
    fig.tight_layout()
    fig.savefig(out / f"{tag}_top.png")
    plt.close(fig)


def fig_clearance(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    true_p = npz["state_13"][:, :3]
    n = true_p.shape[0]
    n_obs = len(obstacles)

    tracks = [obstacle_track(o, t) for o in obstacles]
    dist = np.stack(
        [np.linalg.norm(true_p - tr, axis=1) for tr in tracks], axis=1
    )  # (n, n_obs)

    radius_h = npz["tightened_safety_radius_horizon"]  # (n, 21, n_obs)
    node = 1 if radius_h.shape[1] > 1 else 0
    req = radius_h[:, node, :]
    residual = npz["chance_residual_horizon"][:, node, :]

    fig, axes = plt.subplots(2, 1, figsize=(11.0, 6.2), dpi=170, sharex=True)

    for k, o in enumerate(obstacles):
        colour = OBS_COLOURS[k % len(OBS_COLOURS)]
        axes[0].plot(t, dist[:, k], color=colour, lw=2.2,
                     label=f"khoảng cách tới {o['name']}")
        axes[0].plot(t, req[:, k], color=colour, lw=1.4, ls="--", alpha=0.85,
                     label=f"bán kính an toàn yêu cầu – {o['name']}")
    axes[0].set_ylabel("khoảng cách [m]")
    axes[0].set_title(
        "Khoảng cách thực tế (nét liền) so với bán kính an toàn đã siết (nét đứt)"
    )
    axes[0].legend(fontsize=8.5, ncol=2)

    for k, o in enumerate(obstacles):
        colour = OBS_COLOURS[k % len(OBS_COLOURS)]
        axes[1].plot(t, residual[:, k], color=colour, lw=2.0,
                     label=o["name"])
    axes[1].axhline(0.0, color=RED, lw=1.6, ls="--")
    axes[1].set_xlabel("thời gian [s]")
    axes[1].set_ylabel("biên an toàn [m]")
    axes[1].set_title(
        "Biên an toàn còn lại (âm = vi phạm ràng buộc) — luôn dương"
    )
    axes[1].legend(fontsize=8.5, ncol=3)

    fig.tight_layout()
    fig.savefig(out / f"{tag}_clearance.png")
    plt.close(fig)


def fig_states(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    p = npz["state_13"][:, :3]
    labels = ("x", "y", "z")
    if goal is None:
        goal = np.full(3, np.nan)

    fig, axes = plt.subplots(1, 3, figsize=(13.0, 3.7), dpi=170)
    for k in range(3):
        axes[k].plot(t, p[:, k], color=PURPLE, lw=2.4)
        axes[k].axhline(goal[k], color=AMBER, lw=1.6, ls="--",
                        label=f"đích {labels[k]} = {goal[k]:.2f} m")
        axes[k].scatter(t[0], p[0, k], color=GREEN, s=40, zorder=5)
        axes[k].scatter(t[-1], p[-1, k], color=DARK, s=40, marker="s", zorder=5)
        axes[k].set_xlabel("thời gian [s]")
        axes[k].set_ylabel(f"{labels[k]} [m]")
        axes[k].set_title(f"Tọa độ {labels[k]}(t)")
        axes[k].legend(fontsize=8.5, loc="best")
    fig.tight_layout()
    fig.savefig(out / f"{tag}_states.png")
    plt.close(fig)


def fig_goal_error(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    d = series(tele, "goal_distance_m")
    tol = 0.25

    fig, ax = plt.subplots(figsize=(11.0, 4.4), dpi=170)
    ax.plot(t, d, color=PURPLE, lw=2.6, marker="o", ms=3.2)
    ax.axhline(tol, color=RED, lw=1.8, ls="--",
               label=f"ngưỡng coi như tới đích = {tol:.2f} m")
    ax.fill_between(t, 0, d, color=PURPLE, alpha=0.09)
    ax.annotate(
        f"còn {d[-1] * 100:.1f} cm sau {t[-1]:.1f} s",
        xy=(t[-1], d[-1]),
        xytext=(0.42, 0.72),
        textcoords="axes fraction",
        fontsize=10.5,
        color=DARK,
        arrowprops=dict(arrowstyle="->", color=GREY, lw=1.2),
    )
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel("sai số vị trí [m]")
    ax.set_ylim(bottom=0)
    ax.set_title("Sai số vị trí so với đích theo thời gian")
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out / f"{tag}_goalerr.png")
    plt.close(fig)


def fig_control(out: pathlib.Path, tele, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    thrust = series(tele, "thrust_deviation")
    taux = series(tele, "tau_x")
    tauy = series(tele, "tau_y")
    tauz = series(tele, "tau_z")

    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.0), dpi=170)
    axes[0].plot(t, thrust, color=PURPLE, lw=2.4)
    axes[0].axhline(0.085, color=RED, lw=1.3, ls="--", label="biên trên +0,085")
    axes[0].axhline(-0.085, color=RED, lw=1.3, ls="--", label="biên dưới −0,085")
    axes[0].set_xlabel("thời gian [s]")
    axes[0].set_ylabel("độ lệch lực đẩy [N]")
    axes[0].set_title("Tín hiệu lực đẩy")
    axes[0].legend(fontsize=8.5)

    for val, colour, name in (
        (taux, TEAL, r"$\tau_x$"),
        (tauy, AMBER, r"$\tau_y$"),
        (tauz, "#BE185D", r"$\tau_z$"),
    ):
        axes[1].plot(t, val, color=colour, lw=2.0, label=name)
    axes[1].axhline(0.002, color=GREY, lw=1.2, ls="--")
    axes[1].axhline(-0.002, color=GREY, lw=1.2, ls="--",
                    label="biên ±0,002 (roll/pitch)")
    axes[1].set_xlabel("thời gian [s]")
    axes[1].set_ylabel("mô-men [N·m]")
    axes[1].set_title("Tín hiệu mô-men điều khiển")
    axes[1].legend(fontsize=8.5, ncol=2)
    fig.tight_layout()
    fig.savefig(out / f"{tag}_control.png")
    plt.close(fig)


def fig_uncertainty(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    sigma_h = npz["projected_uncertainty_horizon"]  # (n, 21, n_obs)
    radius_h = npz["tightened_safety_radius_horizon"]
    node = 1 if radius_h.shape[1] > 1 else 0

    radius_h_node = radius_h[:, node, :]
    sigma_h_node = sigma_h[:, node, :]
    # Chon dung vat can co ban kinh yeu cau lon nhat de sigma va ban kinh
    # cung mo ta mot cap rang buoc, tranh tron hai vat can khac nhau.
    worst = np.argmax(radius_h_node, axis=1)
    idx = np.arange(radius_h_node.shape[0])
    radius_req = radius_h_node[idx, worst]
    sigma_req = sigma_h_node[idx, worst]
    # tightening = Phi^-1(1 - epsilon) * sigma, epsilon = total_epsilon / 60
    quantile = 2.9351994688666982
    base = radius_req - sigma_req * quantile

    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.4), dpi=170)

    axes[0].plot(t, sigma_req, color=TEAL, lw=2.6, label=r"$\sigma$ của vật cản ngặt nhất")
    axes[0].fill_between(t, 0, sigma_req, color=TEAL, alpha=0.10)
    axes[0].set_xlabel("thời gian [s]")
    axes[0].set_ylabel(r"độ bất định $\sigma$ [m]")
    axes[0].set_title(r"Độ bất định vị trí tương đối $\sigma$ theo thời gian")
    axes[0].legend(fontsize=9)

    axes[1].plot(t, radius_req, color=PURPLE, lw=2.6,
                 label="bán kính an toàn đã siết")
    axes[1].plot(t, base, color=GREY, lw=2.0, ls="--",
                 label="bán kính hình học gốc")
    axes[1].fill_between(t, base, radius_req, color=PURPLE, alpha=0.16,
                         label="phần phình ra do bất định")
    axes[1].set_xlabel("thời gian [s]")
    axes[1].set_ylabel("bán kính [m]")
    axes[1].set_title("Bán kính an toàn: gốc so với đã siết")
    axes[1].legend(fontsize=9, loc="best")
    fig.tight_layout()
    fig.savefig(out / f"{tag}_uncertainty.png")
    plt.close(fig)


def fig_variance(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    """Plot one obstacle's current and horizon position variance."""
    if not obstacles:
        raise ValueError("variance figure requires at least one obstacle")

    t = series(tele, "time_s")
    obstacle_index = 0
    obstacle_name = obstacles[obstacle_index]["name"]

    current = np.asarray(npz["obstacle_covariance_6"], dtype=float)
    current_pos = current[:, obstacle_index, :3, :3]
    current_var = np.diagonal(current_pos, axis1=1, axis2=2)

    horizon = np.asarray(npz["predicted_obstacle_covariance_horizon"], dtype=float)
    horizon_pos = horizon[:, :, obstacle_index, :3, :3]
    horizon_var = np.diagonal(horizon_pos, axis1=2, axis2=3)
    horizon_trace = np.sum(horizon_var, axis=2)
    current_trace = np.sum(current_var, axis=1)

    fig, axes = plt.subplots(1, 2, figsize=(12.4, 4.4), dpi=170)

    for index, (colour, label) in enumerate(
        zip((TEAL, PURPLE, AMBER), ("Var(xₒ)", "Var(yₒ)", "Var(zₒ)"))
    ):
        axes[0].plot(t, current_var[:, index], color=colour, lw=2.4, label=label)
    axes[0].set_xlabel("thời gian [s]")
    axes[0].set_ylabel("phương sai [m²]")
    axes[0].set_title(f"Covariance hiện tại — {obstacle_name}")
    axes[0].legend(fontsize=8.5)

    axes[1].plot(t, current_trace, color=GREY, lw=2.2, label="hiện tại sau cập nhật")
    axes[1].plot(t, horizon_trace[:, 1], color=PURPLE, lw=2.2,
                 label="dự báo node 1")
    axes[1].plot(t, horizon_trace[:, -1], color=RED, lw=2.2, ls="--",
                 label="dự báo node 20")
    axes[1].set_xlabel("thời gian mô phỏng [s]")
    axes[1].set_ylabel("tr(Σₚₒ) [m²]")
    axes[1].set_title("Phương sai hiện tại và dọc horizon")
    axes[1].legend(fontsize=8.5)

    fig.suptitle(
        f"Phương sai vị trí của vật cản {obstacle_name}",
        fontsize=13,
        fontweight="bold",
        x=0.012,
        ha="left",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out / f"{tag}_variance.png")
    plt.close(fig)


def fig_solver(out: pathlib.Path, tele, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    solve = series(tele, "solver_time_ms")
    total = series(tele, "total_controller_time_ms")

    fig, ax = plt.subplots(figsize=(11.0, 4.5), dpi=170)
    ax.plot(t, solve, color=PURPLE, lw=2.2, marker="o", ms=3.0,
            label="thời gian giải NLP")
    ax.plot(t, total, color=LIGHT, lw=1.8, label="tổng thời gian điều khiển")
    ax.axhline(50.0, color=RED, lw=2.0, ls="--",
               label="hạn 50 ms (20 Hz)")
    ax.fill_between(t, 50.0, solve, where=(solve > 50.0), color=RED, alpha=0.10)
    ax.text(
        0.02,
        0.93,
        "p50 = {:.0f} ms | p99 = {:.0f} ms | lớn nhất = {:.0f} ms\n"
        "số bước đạt hạn: {} / {}".format(
            pct(total, 50), pct(total, 99), np.nanmax(total),
            int(np.sum(total <= 50.0)), total.size
        ),
        transform=ax.transAxes,
        fontsize=10,
        color=DARK,
        va="top",
        bbox=dict(boxstyle="round,pad=0.45", facecolor="#F5F3FF",
                  edgecolor=LIGHT, lw=1.0),
    )
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel("thời gian [ms]")
    ax.set_title("Thời gian giải bài toán tối ưu mỗi bước điều khiển")
    ax.legend(loc="upper right", fontsize=9)
    ax.set_ylim(bottom=0)
    fig.tight_layout()
    fig.savefig(out / f"{tag}_solver.png")
    plt.close(fig)


def fig_summary(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    """Bon dai luong chinh gop trong MOT hinh 2x2.

    Hinh nay dung cho slide trinh bay: khung anh nho (9,4 x 5,5 inch) nhung chu
    co lon, nen khi thu nho vao mot slide 1280x720 thi van doc duoc, khac voi
    viec ghep bon hinh rieng le (moi hinh bi thu nho qua nho).
    """
    t = series(tele, "time_s")
    p = npz["state_13"][:, :3]
    g = np.full(3, np.nan) if goal is None else np.asarray(goal, dtype=float)

    fig, axes = plt.subplots(2, 2, figsize=(9.4, 5.5), dpi=190)
    axes = axes.ravel()

    # -- (0,0) toa do x, y, z -------------------------------------------------
    ax = axes[0]
    cols = (PURPLE, TEAL, AMBER)
    for k, name in enumerate(("x", "y", "z")):
        ax.plot(t, p[:, k], color=cols[k], lw=2.2, label=f"{name} → đích {g[k]:.2f} m")
        ax.axhline(g[k], color=cols[k], lw=1.2, ls="--", alpha=0.75)
    ax.set_xlabel("thời gian [s]", fontsize=9.5)
    ax.set_ylabel("tọa độ [m]", fontsize=9.5)
    ax.set_ylim(0.0, 4.15)  # chua trong phia tren cho bang chu giai
    ax.set_title("Tọa độ x, y, z theo thời gian", fontsize=11)
    ax.tick_params(labelsize=8.5)
    ax.legend(fontsize=8, loc="upper left", frameon=True, framealpha=0.92,
              facecolor="white", edgecolor="#DDDDDD")

    # -- (0,1) sai so toi dich -------------------------------------------------
    ax = axes[1]
    d = series(tele, "goal_distance_m")
    ax.plot(t, d, color=PURPLE, lw=2.3)
    ax.axhline(0.25, color=RED, lw=1.6, ls="--", label="ngưỡng tới đích 0,25 m")
    ax.fill_between(t, 0, d, color=PURPLE, alpha=0.09)
    ax.annotate(
        f"cuối: {d[-1] * 100:.1f} cm",
        xy=(t[-1], d[-1]),
        xytext=(0.40, 0.70),
        textcoords="axes fraction",
        fontsize=9.5,
        color=DARK,
        arrowprops=dict(arrowstyle="->", color=GREY, lw=1.1),
    )
    ax.set_xlabel("thời gian [s]", fontsize=9.5)
    ax.set_ylabel("sai số [m]", fontsize=9.5)
    ax.set_ylim(bottom=0)
    ax.set_title("Sai số vị trí so với đích", fontsize=11)
    ax.tick_params(labelsize=8.5)
    ax.legend(fontsize=8, loc="upper right")

    # -- (1,0) tin hieu dieu khien ---------------------------------------------
    ax = axes[2]
    thrust = series(tele, "thrust_deviation")
    ax.plot(t, thrust, color=PURPLE, lw=2.0, label="độ lệch lực đẩy ΔT")
    ax.axhline(0.085, color=RED, lw=1.3, ls="--")
    ax.axhline(-0.085, color=RED, lw=1.3, ls="--")
    ax.set_xlabel("thời gian [s]", fontsize=9.5)
    ax.set_ylabel("ΔT [N]", fontsize=9.5)
    ax.set_ylim(-0.12, 0.12)  # chua trong phia tren cho phan ghi chu bien
    ax.set_title("Tín hiệu điều khiển", fontsize=11)
    ax.tick_params(labelsize=8.5)
    # Chi co mot duong du lieu nen khong dung bang chu giai: ghi chu bien
    # truc tiep ngay tren duong net dut, de khong che mat doan duong nao.
    ax.text(0.985, 0.089, "biên ±0,085 N", transform=ax.get_yaxis_transform(),
            ha="right", va="bottom", fontsize=8, color=RED)

    # -- (1,1) thoi gian giai --------------------------------------------------
    ax = axes[3]
    solve = series(tele, "solver_time_ms")
    total = series(tele, "total_controller_time_ms")
    ax.plot(t, total, color=LIGHT, lw=1.9, label="tổng điều khiển")
    ax.plot(t, solve, color=PURPLE, lw=1.9, label="giải NLP")
    ax.axhline(50.0, color=RED, lw=1.8, ls="--", label="hạn 50 ms")
    ax.fill_between(t, 50.0, total, where=(total > 50.0), color=RED, alpha=0.10)
    ax.text(
        0.03,
        0.95,
        "p50 {:.0f} · p99 {:.0f} · max {:.0f} ms\nđạt hạn: {} / {} bước".format(
            pct(total, 50), pct(total, 99), np.nanmax(total),
            int(np.sum(total <= 50.0)), total.size,
        ),
        transform=ax.transAxes,
        fontsize=8.6,
        color=DARK,
        va="top",
        bbox=dict(boxstyle="round,pad=0.4", facecolor="#F5F3FF",
                  edgecolor=LIGHT, lw=1.0),
    )
    ax.set_xlabel("thời gian [s]", fontsize=9.5)
    ax.set_ylabel("thời gian [ms]", fontsize=9.5)
    ax.set_ylim(48.0, 138.0)  # chua trong phia tren cho o so lieu
    ax.set_title("Thời gian giải bài toán tối ưu", fontsize=11)
    ax.tick_params(labelsize=8.5)
    ax.legend(fontsize=7.6, loc="upper right", frameon=True, framealpha=0.92,
              facecolor="white", edgecolor="#DDDDDD", borderpad=0.5)

    fig.tight_layout(pad=0.9)
    fig.savefig(out / f"{tag}_summary.png")
    plt.close(fig)


def fig_overview(out: pathlib.Path, tele, npz, obstacles, start, goal, tag):
    t = series(tele, "time_s")
    true_p = npz["state_13"][:, :3]
    n = true_p.shape[0]
    tracks = [obstacle_track(o, t) for o in obstacles]
    dist = np.stack([np.linalg.norm(true_p - tr, axis=1) for tr in tracks], axis=1)
    radius_h = npz["tightened_safety_radius_horizon"]
    node = 1 if radius_h.shape[1] > 1 else 0
    req = radius_h[:, node, :]
    sigma = npz["projected_uncertainty_horizon"][:, node, :].max(axis=1)

    fig, axes = plt.subplots(2, 3, figsize=(13.4, 7.2), dpi=170)

    ax = axes[0, 0]
    for k, o in enumerate(obstacles):
        colour = OBS_COLOURS[k % len(OBS_COLOURS)]
        ax.add_patch(Circle(o["position"][:2], o["radius"], facecolor=colour,
                            alpha=0.25, edgecolor=colour, lw=1.2))
    ax.plot(true_p[:, 0], true_p[:, 1], color=PURPLE, lw=2.6)
    ax.scatter(*true_p[0, :2], color=GREEN, s=45, zorder=5)
    if goal is not None:
        ax.scatter(*goal[:2], color=AMBER, s=150, marker="*", zorder=5)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Quỹ đạo nhìn từ trên xuống")

    ax = axes[0, 1]
    for k, o in enumerate(obstacles):
        colour = OBS_COLOURS[k % len(OBS_COLOURS)]
        ax.plot(t, dist[:, k], color=colour, lw=1.8)
        ax.plot(t, req[:, k], color=colour, lw=1.1, ls="--", alpha=0.85)
    ax.plot([], [], color=GREY, lw=1.8, label="khoảng cách")
    ax.plot([], [], color=GREY, lw=1.1, ls="--", label="bán kính yêu cầu")
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel("[m]")
    ax.set_title("Khoảng cách tới vật cản")
    ax.legend(fontsize=8)

    ax = axes[0, 2]
    ax.plot(t, series(tele, "goal_distance_m"), color=PURPLE, lw=2.2)
    ax.axhline(0.25, color=RED, lw=1.4, ls="--")
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel("[m]")
    ax.set_title("Sai số tới đích")
    ax.set_ylim(bottom=0)

    ax = axes[1, 0]
    ax.plot(t, series(tele, "thrust_deviation"), color=PURPLE, lw=1.8)
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel("[N]")
    ax.set_title("Độ lệch lực đẩy")

    ax = axes[1, 1]
    ax.plot(t, sigma, color=TEAL, lw=2.2)
    ax.fill_between(t, 0, sigma, color=TEAL, alpha=0.12)
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel(r"$\sigma$ [m]")
    ax.set_title("Độ bất định vị trí")

    ax = axes[1, 2]
    solve = series(tele, "solver_time_ms")
    ax.plot(t, solve, color=PURPLE, lw=2.0)
    ax.axhline(50.0, color=RED, lw=1.6, ls="--")
    ax.set_xlabel("thời gian [s]")
    ax.set_ylabel("[ms]")
    ax.set_title("Thời gian giải tối ưu")
    ax.set_ylim(bottom=0)

    fig.tight_layout()
    fig.savefig(out / f"{tag}_overview.png")
    plt.close(fig)


# --------------------------------------------------------------------------
# So sanh nhieu muc nhieu
# --------------------------------------------------------------------------


def fig_noise_compare(out: pathlib.Path, runs, labels, tag):
    # Khung hinh thap (ti le ~2.2) de khi dat vao slide thi chu khong bi thu nho
    # qua muc: do net hien thi tren slide = co chu * chieu_cao_slide / chieu_cao_hinh.
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 5.0), dpi=190)

    for run, label, colour in zip(runs, labels, (TEAL, PURPLE, RED)):
        tele = load_telemetry(run)
        npz = load_npz(run)
        scenario = load_scenario(run)
        obstacles = parse_obstacles(scenario)
        t = series(tele, "time_s")
        true_p = npz["state_13"][:, :3]
        tracks = [obstacle_track(o, t) for o in obstacles]
        clearance = np.min(
            np.stack(
                [np.linalg.norm(true_p - tr, axis=1) - o["radius"]
                 for o, tr in zip(obstacles, tracks)],
                axis=1,
            ),
            axis=1,
        )
        radius_h = npz["tightened_safety_radius_horizon"]
        radius0 = radius_h[0].max(axis=1)
        sigma0 = npz["projected_uncertainty_horizon"][0].max(axis=1)
        sigma_t = npz["projected_uncertainty_horizon"].max(axis=(1, 2))
        nodes = np.arange(radius0.size)

        axes[0, 0].plot(t, clearance, color=colour, lw=2.2,
                        label=f"{label} — nhỏ nhất {clearance.min():.2f} m")
        axes[0, 1].plot(nodes, radius0, color=colour, lw=2.2, marker="o",
                        ms=3.0, label=label)
        axes[1, 0].plot(t, sigma_t, color=colour, lw=2.2, label=label)
        axes[1, 1].plot(nodes, sigma0, color=colour, lw=2.2, marker="o",
                        ms=3.0, label=label)

    axes[0, 0].axhline(0.0, color=RED, lw=1.5, ls="--")
    axes[0, 0].set_xlabel("thời gian [s]")
    axes[0, 0].set_ylabel("khoảng cách [m]")
    axes[0, 0].set_title("Khoảng cách nhỏ nhất tới vật cản")
    axes[0, 0].legend(fontsize=8.5)

    axes[0, 1].set_xlabel("nút trong horizon")
    axes[0, 1].set_ylabel("bán kính an toàn [m]")
    axes[0, 1].set_title("Bán kính an toàn đã siết dọc horizon (bước đầu)")
    axes[0, 1].legend(fontsize=8.5)

    axes[1, 0].set_xlabel("thời gian [s]")
    axes[1, 0].set_ylabel(r"$\sigma$ lớn nhất [m]")
    axes[1, 0].set_title(r"Độ bất định $\sigma$ theo thời gian")
    axes[1, 0].legend(fontsize=8.5)

    axes[1, 1].set_xlabel("nút trong horizon")
    axes[1, 1].set_ylabel(r"$\sigma$ [m]")
    axes[1, 1].set_title(r"Độ bất định dọc horizon (bước đầu)")
    axes[1, 1].legend(fontsize=8.5)

    fig.suptitle(
        "Ảnh hưởng của mức nhiễu tới mức độ thận trọng của bộ điều khiển",
        fontsize=13,
        fontweight="bold",
        x=0.012,
        ha="left",
    )
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(out / f"{tag}_compare.png")
    plt.close(fig)


# --------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="*", help="thu muc recording")
    parser.add_argument("--compare", action="store_true",
                        help="ve bieu do so sanh nhieu run")
    parser.add_argument("--labels", nargs="*", default=None)
    parser.add_argument("--tag", required=True, help="tien to ten file anh")
    parser.add_argument("--out", required=True, help="thu muc ghi anh")
    parser.add_argument("--only", nargs="*", default=None,
                         help="chi ve mot so hinh: 3d top clearance states "
                              "goalerr control uncertainty variance solver overview")
    args = parser.parse_args(argv)

    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = [pathlib.Path(p) for p in args.run]

    if not runs:
        parser.error("can it nhat mot thu muc recording")
    for r in runs:
        if not (r / "telemetry.csv").exists():
            parser.error(f"khong thay telemetry.csv trong {r}")

    if args.compare:
        labels = args.labels or [r.name[:28] for r in runs]
        fig_noise_compare(out, runs, labels, args.tag)
        print(f"da ve {args.tag}_compare.png")
        return 0

    run = runs[0]
    tele = load_telemetry(run)
    npz = load_npz(run)
    scenario = load_scenario(run)
    obstacles = parse_obstacles(scenario)
    start, goal = start_goal(scenario)

    print(f"run       : {run.name}")
    print(f"so buoc   : {len(tele)}")
    print(f"vat can   : {[o['name'] for o in obstacles]}")
    print(f"start     : {start}")
    print(f"goal      : {goal}")

    jobs = {
        "3d": lambda: fig_3d(out, tele, npz, obstacles, start, goal, args.tag),
        "top": lambda: fig_top(out, tele, npz, obstacles, start, goal, args.tag),
        "clearance": lambda: fig_clearance(out, tele, npz, obstacles, start, goal, args.tag),
        "states": lambda: fig_states(out, tele, npz, obstacles, start, goal, args.tag),
        "goalerr": lambda: fig_goal_error(out, tele, npz, obstacles, start, goal, args.tag),
        "control": lambda: fig_control(out, tele, obstacles, start, goal, args.tag),
        "uncertainty": lambda: fig_uncertainty(out, tele, npz, obstacles, start, goal, args.tag),
        "variance": lambda: fig_variance(out, tele, npz, obstacles, start, goal, args.tag),
        "solver": lambda: fig_solver(out, tele, obstacles, start, goal, args.tag),
        "summary": lambda: fig_summary(out, tele, npz, obstacles, start, goal, args.tag),
        "overview": lambda: fig_overview(out, tele, npz, obstacles, start, goal, args.tag),
    }
    selected = args.only or list(jobs)
    for name in selected:
        if name not in jobs:
            parser.error(f"khong biet hinh '{name}'")
        jobs[name]()
        print(f"  da ve {args.tag}_{name}.png")

    return 0


if __name__ == "__main__":
    sys.exit(main())
