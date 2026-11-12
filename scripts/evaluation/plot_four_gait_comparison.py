
# Member 7 - Four-Gait Quantitative Comparison
# EEL5669 Unitree Go1 Project
# Evaluation: Suspended-robot simulation

import csv
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# Locate the repository's results folder.
repo = Path(__file__).resolve().parents[2]
results = repo / "docs" / "results"

csv_path = results / "four_gait_comparison.csv"
output_path = results / "four_gait_comparison.png"

# Read measured results generated from the ROS bags.
with open(csv_path, newline="") as file:
    rows = list(csv.DictReader(file))

if len(rows) != 4:
    raise RuntimeError("Expected exactly four gait results.")

gaits = [row["gait"] for row in rows]

expected_period = np.array([
    float(row["expected_period_s"]) for row in rows
])
measured_period = np.array([
    float(row["measured_period_s"]) for row in rows
])
period_std = np.array([
    float(row["period_std_s"]) for row in rows
])

expected_duty = np.array([
    float(row["expected_stance_percent"]) for row in rows
])
measured_duty = np.array([
    float(row["measured_stance_percent"]) for row in rows
])

x = np.arange(len(gaits))
width = 0.36

fig, axes = plt.subplots(2, 1, figsize=(11, 9))

# --------------------------------------------------
# Plot 1: Gait cycle period
# --------------------------------------------------
ax = axes[0]

ax.bar(
    x - width / 2,
    expected_period,
    width,
    label="Configured"
)

bars_period = ax.bar(
    x + width / 2,
    measured_period,
    width,
    label="Measured",
    yerr=period_std,
    capsize=3
)

ax.set_title("Gait Cycle Period Comparison")
ax.set_ylabel("Cycle Period (s)")
ax.set_xticks(x)
ax.set_xticklabels(gaits)
ax.set_ylim(0, 1.9)
ax.grid(axis="y", alpha=0.3)
ax.set_axisbelow(True)
ax.legend()

for bar, value in zip(bars_period, measured_period):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 0.05,
        f"{value:.4f} s",
        ha="center",
        va="bottom",
        fontsize=9
    )

# --------------------------------------------------
# Plot 2: Scheduled stance duty factor
# --------------------------------------------------
ax = axes[1]

ax.bar(
    x - width / 2,
    expected_duty,
    width,
    label="Configured"
)

bars_duty = ax.bar(
    x + width / 2,
    measured_duty,
    width,
    label="Measured"
)

ax.set_title("Scheduled Stance Duty Factor Comparison")
ax.set_ylabel("Scheduled Stance (%)")
ax.set_xticks(x)
ax.set_xticklabels(gaits)
ax.set_ylim(0, 95)
ax.grid(axis="y", alpha=0.3)
ax.set_axisbelow(True)
ax.legend()

for bar, value in zip(bars_duty, measured_duty):
    ax.text(
        bar.get_x() + bar.get_width() / 2,
        bar.get_height() + 2,
        f"{value:.2f}%",
        ha="center",
        va="bottom",
        fontsize=9
    )

# --------------------------------------------------
# Figure title and experimental limitation
# --------------------------------------------------
fig.suptitle(
    "Unitree Go1 - Four-Gait Scheduler Evaluation",
    fontsize=16,
    fontweight="bold"
)

fig.text(
    0.5, 0.025,
    "Suspended Gazebo simulation | "
    "Scheduler timing validation only; "
    "ground locomotion not evaluated",
    ha="center",
    fontsize=10
)

fig.tight_layout(rect=[0, 0.055, 1, 0.95])

fig.savefig(output_path, dpi=250, bbox_inches="tight")
plt.close(fig)

print("Comparison figure saved:", output_path)
print("Gaits analyzed:", ", ".join(gaits))
print("Analysis complete.")
