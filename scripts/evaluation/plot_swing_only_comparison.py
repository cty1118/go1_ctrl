
#!/usr/bin/env python3
"""Member 7: Four-gait swing-only foot-tracking comparison."""

import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Input and output folders
experiments = Path.home() / "go1_experiments"
repo = Path(__file__).resolve().parents[2]
results = repo / "docs" / "results"
results.mkdir(parents=True, exist_ok=True)

gaits = ["Walk", "Trot", "Canter", "Gallop"]
legs = ["FR", "FL", "RR", "RL"]

rmse_per_leg = []
overall_rmse = []

for gait in gaits:
    filename = experiments / (
        f"{gait.lower()}_fk_trial01_swing_only_summary.csv"
    )

    with open(filename, newline="") as f:
        rows = list(csv.DictReader(f))

    # Index each leg by its name, not CSV row position.
    by_leg = {row["leg"]: row for row in rows}

    if set(by_leg) != set(legs):
        raise RuntimeError(
            f"{gait}: expected FR, FL, RR, RL"
        )

    errors = np.array([
        float(by_leg[leg]["rmse_3d_mm"])
        for leg in legs
    ])

    counts = np.array([
        int(by_leg[leg]["swing_samples"])
        for leg in legs
    ])

    if np.any(counts <= 0):
        raise RuntimeError(
            f"{gait}: invalid swing sample counts"
        )

    # Pool squared errors across all swing samples.
    pooled_rmse = np.sqrt(
        np.sum(counts * errors ** 2) / np.sum(counts)
    )

    rmse_per_leg.append(errors)
    overall_rmse.append(pooled_rmse)

rmse_per_leg = np.array(rmse_per_leg)
overall_rmse = np.array(overall_rmse)

# Create comparison figure
fig, axes = plt.subplots(
    2, 1, figsize=(12, 9)
)

x = np.arange(len(gaits))
width = 0.19

# Graph 1: Per-leg swing-only 3D RMSE
ax = axes[0]

for i, leg in enumerate(legs):
    bars = ax.bar(
        x + (i - 1.5) * width,
        rmse_per_leg[:, i],
        width,
        label=leg
    )

    for bar in bars:
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.04,
            f"{bar.get_height():.2f}",
            ha="center",
            fontsize=8
        )

ax.set_title("Per-Leg Swing-Only 3D Tracking RMSE")
ax.set_ylabel("3D Position RMSE (mm)")
ax.set_xticks(x)
ax.set_xticklabels(gaits)
ax.set_ylim(0, max(rmse_per_leg.flatten()) * 1.22)
ax.grid(axis="y", alpha=0.3)
ax.set_axisbelow(True)
ax.legend(title="Leg", ncol=4)

# Graph 2: Overall swing-only 3D RMSE
ax = axes[1]

bars = ax.bar(
    x,
    overall_rmse,
    width=0.55
)

for bar, value in zip(bars, overall_rmse):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        value + 0.04,
        f"{value:.3f} mm",
        ha="center",
        fontsize=10,
        fontweight="bold"
    )

ax.set_title("Overall Swing-Only 3D Tracking RMSE")
ax.set_ylabel("3D Position RMSE (mm)")
ax.set_xticks(x)
ax.set_xticklabels(gaits)
ax.set_ylim(0, max(overall_rmse) * 1.22)
ax.grid(axis="y", alpha=0.3)
ax.set_axisbelow(True)

# Title and experimental limitations
fig.suptitle(
    "Unitree Go1 - Four-Gait Swing Tracking Evaluation",
    fontsize=16,
    fontweight="bold"
)

fig.text(
    0.5, 0.015,
    "Suspended Gazebo simulation | FK-estimated foot positions | "
    "Single trial per gait | Preliminary sequence-aligned data",
    ha="center",
    fontsize=9
)

fig.tight_layout(rect=[0, 0.055, 1, 0.95])

output = results / "four_gait_swing_tracking_comparison.png"

fig.savefig(
    output,
    dpi=250,
    bbox_inches="tight"
)

plt.close(fig)

print("\nFOUR-GAIT SWING-ONLY TRACKING COMPARISON")
for gait, error in zip(gaits, overall_rmse):
    print(f"{gait:7s}: {error:.3f} mm")

print("\nSaved:", output)
print("Comparison completed successfully!")
