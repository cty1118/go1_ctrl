import sys
from pathlib import Path

import rosbag
import numpy as np
import matplotlib.pyplot as plt

# Read ROS bag path from the command line.
if len(sys.argv) != 2:
    raise SystemExit("Usage: python3 plot_gait_debug.py PATH_TO_BAG")

bag_path = sys.argv[1]
leg_names = ["FR", "FL", "RR", "RL"]

times = []
samples = []

# Each message contains 40 values: 10 per leg.
with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(
        topics=["/go1_ctrl/gait_debug"]
    ):
        if len(msg.data) != 40:
            continue

        times.append(t.to_sec())
        samples.append(np.array(msg.data).reshape(4, 10))

if not samples:
    raise RuntimeError("No valid gait-debug messages found.")

times = np.array(times)
times -= times[0]
data = np.stack(samples)

# Per-leg data layout:
# 0 = gait phase
# 1 = scheduled contact
# 2 = detected contact
# 3 = foot force (N)
# 4:7 = commanded feedforward torque (Nm)
# 7:10 = estimated joint torque (Nm)

output_base = str(Path(bag_path).with_suffix(""))

plot_titles = {
    "phase": "Gait Phase",
    "contact": "Scheduled vs Detected Contact",
    "force": "Foot Contact Force",
    "torque": "Commanded FF vs Estimated Joint Torque"
}

for kind, title in plot_titles.items():

    fig, axes = plt.subplots(2, 2, figsize=(12, 8),
                             sharex=True)

    for leg, ax in enumerate(axes.flat):

        if kind == "phase":
            ax.plot(times, data[:, leg, 0],
                    label="Gait Phase")
            ax.set_ylabel("Phase (0-1)")
            ax.set_ylim(-0.05, 1.05)

        elif kind == "contact":
            ax.step(times, data[:, leg, 1],
                    where="post", label="Scheduled")
            ax.step(times, data[:, leg, 2],
                    where="post", linestyle="--",
                    label="Detected")
            ax.set_ylabel("Contact (0/1)")
            ax.set_ylim(-0.1, 1.1)

        elif kind == "force":
            ax.plot(times, data[:, leg, 3],
                    label="Foot Force")
            ax.set_ylabel("Force (N)")

        elif kind == "torque":
            joint_names = ["Hip", "Thigh", "Calf"]

            for j, joint in enumerate(joint_names):
                ax.plot(times, data[:, leg, 4 + j],
                        label=f"{joint} FF cmd")
                ax.plot(times, data[:, leg, 7 + j],
                        linestyle="--",
                        label=f"{joint} estimated")

            ax.set_ylabel("Torque (Nm)")

        ax.set_title(f"{leg_names[leg]} Leg")
        ax.set_xlabel("Simulation Time (s)")
        ax.grid(True)
        ax.legend(fontsize=7)

    fig.suptitle(f"Unitree Go1 - {title}")
    fig.tight_layout()

    output = f"{output_base}_{kind}.png"
    fig.savefig(output, dpi=200)
    plt.close(fig)

    print("Saved:", output)

print("Total valid samples:", len(times))
print("Analysis completed successfully!")
