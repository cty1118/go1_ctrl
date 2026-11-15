#!/usr/bin/env python3
"""Analyze ground-supported Trot failure from a ROS bag."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import rosbag
from tf.transformations import euler_from_quaternion
from math import degrees

parser = argparse.ArgumentParser()
parser.add_argument(
    "--bag", type=Path,
    default=Path.home() / "go1_experiments/trot_ground_failure_trial01.bag"
)
args = parser.parse_args()

out = Path(__file__).resolve().parents[2] / "docs/results"
out.mkdir(parents=True, exist_ok=True)

legs = ["FR", "FL", "RR", "RL"]
modes = []
body = []
gait = []

with rosbag.Bag(str(args.bag)) as bag:
    for _, msg, stamp in bag.read_messages(topics=["/go1_ctrl/mode"]):
        modes.append((stamp.to_sec(), msg.data))

    starts = [t for t, mode in modes if mode == "GAIT trot"]
    if not starts:
        raise RuntimeError("No GAIT trot transition found")

    start = starts[0]
    stops = [t for t, mode in modes if t > start and mode == "PASSIVE"]
    if not stops:
        raise RuntimeError("No subsequent PASSIVE transition found")
    end = stops[0]

    for topic, msg, stamp in bag.read_messages(
        topics=["/gazebo/model_states", "/go1_ctrl/gait_debug"]
    ):
        t = stamp.to_sec()
        if not start <= t <= end:
            continue

        elapsed = t - start

        if topic == "/gazebo/model_states":
            if "go1_gazebo" not in msg.name:
                continue
            i = msg.name.index("go1_gazebo")
            q = msg.pose[i].orientation
            roll, pitch, _ = euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )
            body.append([
                elapsed,
                msg.pose[i].position.z,
                degrees(roll),
                degrees(pitch)
            ])

        elif len(msg.data) == 40:
            row = [elapsed]
            for i in range(4):
                d = msg.data[i*10:(i+1)*10]
                row.extend([int(d[1] > 0.5),
                            int(d[2] > 0.5), d[3]])
            gait.append(row)

if not body or not gait:
    raise RuntimeError("Missing body or gait diagnostic samples")

with open(out / "ground_trot_body_trial01.csv", "w") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "height_m", "roll_deg", "pitch_deg"])
    writer.writerows(body)

with open(out / "ground_trot_contacts_trial01.csv", "w") as f:
    writer = csv.writer(f)
    header = ["time_s"]
    for leg in legs:
        header.extend([
            f"{leg}_scheduled",
            f"{leg}_detected",
            f"{leg}_force_N"
        ])
    writer.writerow(header)
    writer.writerows(gait)

fig, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

ax[0].plot([r[0] for r in body],
           [r[2] for r in body], label="Roll")
ax[0].plot([r[0] for r in body],
           [r[3] for r in body], label="Pitch")
ax[0].set_ylabel("Angle (degrees)")
ax[0].legend()
ax[0].grid(True)

ax[1].plot([r[0] for r in body], [r[1] for r in body])
ax[1].set_ylabel("Body height (m)")
ax[1].grid(True)

for i, leg in enumerate(legs):
    ax[2].plot([r[0] for r in gait],
               [r[1 + 3*i + 2] for r in gait],
               label=leg, linewidth=1)

ax[2].axhline(20, linestyle="--", color="black",
              label="Contact threshold")
ax[2].set_ylabel("Foot force (N)")
ax[2].set_xlabel("Time after Trot activation (s)")
ax[2].legend(ncol=5)
ax[2].grid(True)

fig.suptitle("Go1 Ground-Supported Trot Failure — Trial 01")
fig.tight_layout()
fig.savefig(out / "ground_trot_failure_trial01.png", dpi=180)
plt.close(fig)

print(f"Trot started: {start:.4f} s")
print(f"PASSIVE transition: {end:.4f} s")
print(f"Duration: {end-start:.4f} s")
print(f"Body samples: {len(body)}")
print(f"Gait samples: {len(gait)}")
print(f"Evidence saved in: {out}")
