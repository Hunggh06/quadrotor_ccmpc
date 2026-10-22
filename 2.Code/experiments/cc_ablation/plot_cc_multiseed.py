#!/usr/bin/env python3
"""Plot the 3-seed CC ablation aggregate."""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
SUMMARY = ROOT / "experiments" / "cc_ablation" / "cc_ablation_multiseed.json"
FIG_DIR = ROOT / "outputs" / "cc_ablation" / "figures"

VARIANTS = [
    "cc_soft_m030",
    "nocc_soft_m030",
    "cc_hard_m030",
    "nocc_hard_m030",
    "cc_soft_m000",
    "nocc_soft_m000",
    "cc_hard_m000",
    "nocc_hard_m000",
]
LABELS = [
    "CC soft\nm=0.30",
    "no-CC soft\nm=0.30",
    "CC hard\nm=0.30",
    "no-CC hard\nm=0.30",
    "CC soft\nm=0.00",
    "no-CC soft\nm=0.00",
    "CC hard\nm=0.00",
    "no-CC hard\nm=0.00",
]
COLORS = ["#1f4e79", "#c55a11", "#2e75b6", "#f4b183"] * 2


def main() -> int:
    data = json.loads(SUMMARY.read_text(encoding="utf-8"))
    fig, axes = plt.subplots(2, 2, figsize=(11.0, 6.6))
    x = np.arange(len(VARIANTS))

    panels = [
        ("min_clearance_m", "minimum obstacle clearance [m]", 0.0),
        ("final_goal_distance_m", "final goal error [m]", None),
        ("p95_total_ms", "p95 controller time [ms]", 50.0),
        ("deadline_misses", "deadline misses per 120 ticks", None),
    ]
    for ax, (key, title, reference) in zip(axes.flat, panels):
        means = [data[v]["metrics"][key]["mean"] for v in VARIANTS]
        stds = [data[v]["metrics"][key]["std"] for v in VARIANTS]
        bars = ax.bar(
            x,
            means,
            yerr=stds,
            capsize=3,
            color=COLORS,
            edgecolor="white",
            linewidth=0.5,
            error_kw={"elinewidth": 0.8, "ecolor": "#333333"},
        )
        for bar, variant in zip(bars, VARIANTS):
            if "hard" in variant:
                bar.set_hatch("//")
        if reference is not None:
            ax.axhline(reference, color="#8b0000", linewidth=0.9, linestyle=":")
            ax.text(
                len(VARIANTS) - 0.5,
                reference,
                f" {reference:g}",
                color="#8b0000",
                fontsize=7,
                va="bottom",
                ha="right",
            )
        ax.set_xticks(x)
        ax.set_xticklabels(LABELS, fontsize=7)
        ax.set_title(title)
        ax.axvline(3.5, color="#999999", linewidth=0.8)
        ymax = ax.get_ylim()[1]
        ax.text(1.5, ymax * 0.95, "fixed margin 0.30 m", ha="center", fontsize=8)
        ax.text(5.5, ymax * 0.95, "fixed margin 0.00 m", ha="center", fontsize=8)

    fig.suptitle(
        "CC on/off and fixed-margin ablation, mean ± std over seeds 7/17/27",
        y=1.01,
    )
    fig.tight_layout()
    FIG_DIR.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG_DIR / "fig_cc_multiseed_summary.png", bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {FIG_DIR / 'fig_cc_multiseed_summary.png'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
