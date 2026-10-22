#!/usr/bin/env python3
"""Plot the CC-on/off and safety-margin ablation.

Reads ``outputs/cc_ablation/runs.json`` plus each run's ``telemetry.csv`` and
writes compact comparison figures.  It never touches the baseline outputs.
"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

ROOT = Path(__file__).resolve().parents[2]
RUNS_PATH = ROOT / "outputs" / "cc_ablation" / "runs.json"
FIG_DIR = ROOT / "outputs" / "cc_ablation" / "figures"


def load_runs() -> dict[str, dict]:
    payloads = json.loads(RUNS_PATH.read_text(encoding="utf-8"))
    return {item["variant"]: item for item in payloads}


def telemetry(payload: dict) -> pd.DataFrame:
    return pd.read_csv(Path(payload["run_dir"]) / "telemetry.csv")


def style() -> None:
    plt.rcParams.update(
        {
            "figure.dpi": 130,
            "savefig.dpi": 160,
            "font.size": 9,
            "axes.grid": True,
            "grid.alpha": 0.25,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
        }
    )


COLORS = {
    "cc_soft": "#1f4e79",
    "nocc_soft": "#c55a11",
    "cc_hard": "#2e75b6",
    "nocc_hard": "#f4b183",
}


def variant_style(name: str) -> tuple[str, str, str]:
    if name.startswith("cc_soft"):
        return COLORS["cc_soft"], "-", "CC soft"
    if name.startswith("nocc_soft"):
        return COLORS["nocc_soft"], "-", "no-CC soft"
    if name.startswith("cc_hard"):
        return COLORS["cc_hard"], "--", "CC hard"
    return COLORS["nocc_hard"], "--", "no-CC hard"


def plot_clearance_time(runs: dict[str, dict], margin: float, filename: str) -> None:
    fig, ax = plt.subplots(figsize=(8.0, 4.2))
    tag = f"m{int(round(margin * 100)):03d}"
    for name in (
        f"cc_soft_{tag}",
        f"nocc_soft_{tag}",
        f"cc_hard_{tag}",
        f"nocc_hard_{tag}",
    ):
        payload = runs[name]
        df = telemetry(payload)
        color, linestyle, label = variant_style(name)
        ax.plot(
            df["time_s"],
            df["min_clearance_m"],
            color=color,
            linestyle=linestyle,
            linewidth=1.6,
            label=label,
        )
    ax.axhline(0.0, color="#8b0000", linewidth=0.9, linestyle=":")
    ax.axhline(margin, color="#555555", linewidth=0.9, linestyle=":")
    ax.text(
        0.02,
        margin + 0.008,
        f"fixed margin = {margin:.2f} m",
        color="#555555",
        fontsize=8,
    )
    ax.set_xlabel("time [s]")
    ax.set_ylabel("min obstacle clearance [m]")
    ax.set_title(f"Clearance over time, fixed margin {margin:.2f} m")
    ax.set_ylim(bottom=min(-0.08, margin - 0.05))
    ax.legend(ncol=2)
    fig.tight_layout()
    fig.savefig(FIG_DIR / filename)
    plt.close(fig)


def plot_paths(runs: dict[str, dict], margin: float, filename: str) -> None:
    tag = f"m{int(round(margin * 100)):03d}"
    fig, (ax_xy, ax_z) = plt.subplots(
        1,
        2,
        figsize=(11.0, 4.4),
        gridspec_kw={"width_ratios": [1.15, 1.0]},
    )
    for name in (
        f"cc_soft_{tag}",
        f"nocc_soft_{tag}",
        f"cc_hard_{tag}",
        f"nocc_hard_{tag}",
    ):
        payload = runs[name]
        df = telemetry(payload)
        color, linestyle, label = variant_style(name)
        ax_xy.plot(
            df["px"],
            df["py"],
            color=color,
            linestyle=linestyle,
            linewidth=1.5,
            label=label,
        )
        ax_z.plot(
            df["time_s"],
            df["pz"],
            color=color,
            linestyle=linestyle,
            linewidth=1.5,
            label=label,
        )
    # Obstacles at t=0 (static gate, crossing, vertical sinusoid centres).
    obstacles = [
        (1.45, 1.05, 0.40, "gate"),
        (1.85, -0.80, 0.24, "crossing"),
        (2.35, 1.55, 0.20, "vertical"),
    ]
    for x, y, radius, label in obstacles:
        circle = plt.Circle(
            (x, y),
            radius,
            facecolor="none",
            edgecolor="#7f7f7f",
            linewidth=1.0,
            linestyle="--",
        )
        ax_xy.add_patch(circle)
        ax_xy.plot(x, y, "x", color="#7f7f7f", markersize=4)
        ax_xy.text(x + 0.03, y + 0.03, label, fontsize=7, color="#555555")
    ax_xy.plot(0.0, 0.0, "o", color="black", markersize=4)
    ax_xy.plot(3.0, 2.0, "*", color="black", markersize=8)
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.set_title("Top view")
    ax_xy.axis("equal")
    ax_xy.legend(fontsize=7, loc="lower right")

    ax_z.set_xlabel("time [s]")
    ax_z.set_ylabel("z [m]")
    ax_z.set_title("Altitude")
    ax_z.legend(fontsize=7)
    fig.suptitle(f"Trajectories, fixed margin {margin:.2f} m", y=1.01)
    fig.tight_layout()
    fig.savefig(FIG_DIR / filename, bbox_inches="tight")
    plt.close(fig)


def plot_metric_bars(runs: dict[str, dict], filename: str) -> None:
    labels = [
        ("m030", "cc_soft_m030", "CC soft"),
        ("m030", "nocc_soft_m030", "no-CC soft"),
        ("m030", "cc_hard_m030", "CC hard"),
        ("m030", "nocc_hard_m030", "no-CC hard"),
        ("m000", "cc_soft_m000", "CC soft"),
        ("m000", "nocc_soft_m000", "no-CC soft"),
        ("m000", "cc_hard_m000", "CC hard"),
        ("m000", "nocc_hard_m000", "no-CC hard"),
    ]
    metrics = [
        ("min_clearance_m", "minimum obstacle clearance [m]", None),
        ("final_goal_distance_m", "final goal error [m]", None),
        ("p95_total_ms", "p95 controller time [ms]", 50.0),
        ("fallback_activations", "fallback activations", None),
    ]
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 6.4))
    x = np.arange(len(labels))
    for ax, (key, title, reference) in zip(axes.flat, metrics):
        values = [runs[item[1]]["metrics"][key] for item in labels]
        colors = []
        hatches = []
        for _, name, _ in labels:
            color, _, _ = variant_style(name)
            colors.append(color)
            hatches.append("//" if "hard" in name else "")
        bars = ax.bar(x, values, color=colors, edgecolor="white", linewidth=0.5)
        for bar, hatch in zip(bars, hatches):
            bar.set_hatch(hatch)
        if reference is not None:
            ax.axhline(reference, color="#8b0000", linewidth=0.9, linestyle=":")
            ax.text(
                len(labels) - 0.4,
                reference,
                " 50 ms",
                color="#8b0000",
                fontsize=7,
                va="bottom",
                ha="right",
            )
        ax.set_xticks(x)
        ax.set_xticklabels([item[2] for item in labels], rotation=30, ha="right")
        ax.set_title(title)
        ax.axvline(3.5, color="#999999", linewidth=0.8)
        ax.text(1.5, ax.get_ylim()[1] * 0.92, "margin 0.30 m", ha="center", fontsize=8)
        ax.text(5.5, ax.get_ylim()[1] * 0.92, "margin 0.00 m", ha="center", fontsize=8)
    fig.suptitle("CC on/off and safety-margin ablation (seed 7, 6 s)", y=1.01)
    fig.tight_layout()
    fig.savefig(FIG_DIR / filename, bbox_inches="tight")
    plt.close(fig)


def plot_margin_effect(runs: dict[str, dict], filename: str) -> None:
    margins = [0.30, 0.10, 0.00]
    series = [
        ("cc_soft", "CC soft", "o-", COLORS["cc_soft"]),
        ("nocc_soft", "no-CC soft", "s-", COLORS["nocc_soft"]),
        ("cc_hard", "CC hard", "o--", COLORS["cc_hard"]),
        ("nocc_hard", "no-CC hard", "s--", COLORS["nocc_hard"]),
    ]
    fig, ax = plt.subplots(figsize=(6.6, 4.2))
    for prefix, label, marker, color in series:
        values = []
        for margin in margins:
            tag = f"m{int(round(margin * 100)):03d}"
            values.append(runs[f"{prefix}_{tag}"]["metrics"]["min_clearance_m"])
        ax.plot(margins, values, marker, color=color, label=label, linewidth=1.6)
    ax.axhline(0.0, color="#8b0000", linewidth=0.9, linestyle=":")
    ax.set_xlabel("fixed safety margin [m]")
    ax.set_ylabel("minimum obstacle clearance [m]")
    ax.set_title("Minimum clearance vs fixed margin")
    ax.invert_xaxis()
    ax.legend()
    fig.tight_layout()
    fig.savefig(FIG_DIR / filename, bbox_inches="tight")
    plt.close(fig)


def main() -> int:
    style()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    runs = load_runs()
    plot_clearance_time(runs, 0.30, "fig_cc_clearance_time_m030.png")
    plot_clearance_time(runs, 0.00, "fig_cc_clearance_time_m000.png")
    plot_paths(runs, 0.30, "fig_cc_trajectories_m030.png")
    plot_paths(runs, 0.00, "fig_cc_trajectories_m000.png")
    plot_metric_bars(runs, "fig_cc_metrics_bars.png")
    plot_margin_effect(runs, "fig_cc_margin_effect.png")
    print(f"wrote figures to {FIG_DIR}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
