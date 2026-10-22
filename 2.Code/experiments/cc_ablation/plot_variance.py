#!/usr/bin/env python3
"""Plot obstacle position uncertainty for one recording.

The current implementation propagates one shared obstacle covariance, so the
current-covariance panel is intentionally a single line.  The predicted
projected uncertainty still differs per obstacle because the geometry differs.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--out", type=Path, default=None)
    args = parser.parse_args()
    run = args.run_dir
    out = args.out or (run / "visuals_report" / "obstacle_variance.png")
    data = np.load(run / "trajectory.npz")
    covariance = np.asarray(data["obstacle_covariance_6"], dtype=float)
    projected = np.asarray(data["projected_uncertainty_horizon"], dtype=float)
    labels = ["static_gate", "crossing", "vertical_sinusoid"]
    time = np.arange(covariance.shape[0]) * 0.05

    current_std = np.sqrt(
        np.maximum(np.trace(covariance[:, :, :3, :3], axis1=2, axis2=3) / 3.0, 0.0)
    )
    shared_std = np.mean(current_std, axis=1)
    max_projected = np.max(projected, axis=1)

    fig, axes = plt.subplots(2, 1, figsize=(7.6, 6.2), sharex=True)
    axes[0].plot(time, shared_std, color="#1f77b4", linewidth=1.8, label="shared obstacle covariance")
    axes[0].set_ylabel("Std vị trí hiện tại [m]")
    axes[0].set_title("Bất định vị trí vật cản hiện tại (covariance dùng chung)")
    axes[0].legend(loc="best", fontsize=8)
    for index, label in enumerate(labels[: max_projected.shape[1]]):
        axes[1].plot(time, max_projected[:, index], label=label)
    axes[1].set_xlabel("Thời gian [s]")
    axes[1].set_ylabel("Độ bất định max [m]")
    axes[1].set_title("Bất định hình học dự báo (max trên 20 node, theo vật cản)")
    axes[1].legend(loc="best", fontsize=8)
    fig.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
