
#!/usr/bin/env python3
"""Member 7: Ground-supported Walk before/after evaluation.

Baseline: controller 030c250
Improved: controller c472c21

Requires ROS Noetic, rosbag, numpy, matplotlib and tf.
"""

import csv
from pathlib import Path

import numpy as np
import rosbag
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from tf.transformations import euler_from_quaternion

REPO = Path(__file__).resolve().parents[2]
RESULTS = REPO / "docs/results"
BAG = Path.home() / "go1_experiments/walk_ground_improved_clean_trial02.bag"

BASE_BODY = RESULTS / "ground_walk_body_trial01.csv"
BASE_CONTACTS = RESULTS / "ground_walk_contacts_trial01.csv"

LEGS = ["FR", "FL", "RR", "RL"]


def read_csv(path):
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path, header, rows):
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)


def contact_stats(rows):
    scheduled = np.zeros(4, dtype=int)
    missed = np.zeros(4, dtype=int)

    for row in rows:
        for i, leg in enumerate(LEGS):
            s = int(row[f"{leg}_scheduled"])
            c = int(row[f"{leg}_detected"])
            scheduled[i] += s
            missed[i] += (s == 1 and c == 0)

    rates = 100 * np.divide(
        missed, scheduled,
        out=np.zeros(4, dtype=float),
        where=scheduled > 0
    )

    overall = 100 * missed.sum() / scheduled.sum()
    return scheduled, missed, rates, overall


# --------------------------------------------------
# 1. Read original controller results
# --------------------------------------------------

old_body = read_csv(BASE_BODY)
old_contacts = read_csv(BASE_CONTACTS)

old = np.array([
    [
        float(r["time_s"]),
        float(r["roll_deg"]),
        float(r["pitch_deg"]),
        float(r["height_m"])
    ]
    for r in old_body
])

# --------------------------------------------------
# 2. Extract improved controller data from ROS bag
# --------------------------------------------------

body_rows = []
contact_rows = []
velocity_rows = []
modes = []

with rosbag.Bag(str(BAG)) as bag:

    for _, msg, stamp in bag.read_messages(
        topics=["/go1_ctrl/mode"]
    ):
        modes.append((stamp.to_sec(), msg.data))

    starts = [
        t for t, mode in modes
        if mode == "GAIT walk"
    ]
    if not starts:
        raise RuntimeError("No GAIT walk activation found")

    start = starts[0]

    stops = [
        (t, mode) for t, mode in modes
        if t > start and mode in ("STAND", "PASSIVE")
    ]
    if not stops:
        raise RuntimeError("No Walk termination found")

    end, end_mode = stops[0]

    for topic, msg, stamp in bag.read_messages(
        topics=[
            "/gazebo/model_states",
            "/go1_ctrl/gait_debug",
            "/go1_ctrl/cmd_vel"
        ]
    ):
        t = stamp.to_sec()

        if not start <= t <= end:
            continue

        elapsed = t - start

        if topic == "/gazebo/model_states":
            if "go1_gazebo" not in msg.name:
                continue

            i = msg.name.index("go1_gazebo")
            pose = msg.pose[i]
            q = pose.orientation

            roll, pitch, _ = euler_from_quaternion(
                [q.x, q.y, q.z, q.w]
            )

            body_rows.append([
                elapsed,
                pose.position.z,
                np.degrees(roll),
                np.degrees(pitch)
            ])

        elif topic == "/go1_ctrl/gait_debug":
            if len(msg.data) != 40:
                continue

            row = [elapsed]

            for i in range(4):
                d = msg.data[i * 10:(i + 1) * 10]

                row.extend([
                    int(d[1] > 0.5),
                    int(d[2] > 0.5),
                    d[3]
                ])

            contact_rows.append(row)

        elif topic == "/go1_ctrl/cmd_vel":
            velocity_rows.append([
                elapsed,
                msg.linear.x,
                msg.linear.y,
                msg.angular.z
            ])

if not body_rows or not contact_rows or not velocity_rows:
    raise RuntimeError("Required improved-trial data missing")

# --------------------------------------------------
# 3. Export improved controller CSV datasets
# --------------------------------------------------

RESULTS.mkdir(parents=True, exist_ok=True)

write_csv(
    RESULTS / "ground_walk_improved_body_trial02.csv",
    ["time_s", "height_m", "roll_deg", "pitch_deg"],
    body_rows
)

contact_header = ["time_s"]
for leg in LEGS:
    contact_header.extend([
        f"{leg}_scheduled",
        f"{leg}_detected",
        f"{leg}_force_N"
    ])

write_csv(
    RESULTS / "ground_walk_improved_contacts_trial02.csv",
    contact_header,
    contact_rows
)

write_csv(
    RESULTS / "ground_walk_improved_velocity_trial02.csv",
    ["time_s", "vx_mps", "vy_mps", "yaw_rate_radps"],
    velocity_rows
)

new = np.array([
    [r[0], r[2], r[3], r[1]]
    for r in body_rows
])

# --------------------------------------------------
# 4. Before/after body stability plot
# --------------------------------------------------

fig, ax = plt.subplots(
    3, 1, figsize=(11, 9), sharex=True
)

labels = [
    "Body Roll (deg)",
    "Body Pitch (deg)",
    "Body Height (m)"
]

for i in range(3):
    ax[i].plot(
        old[:, 0], old[:, i + 1],
        label="Original controller (failure)",
        linewidth=1.5
    )

    ax[i].plot(
        new[:, 0], new[:, i + 1],
        label="Updated controller (stable)",
        linewidth=1.5
    )

    ax[i].axvline(
        5.1972, color="red",
        linestyle="--", alpha=0.6
    )

    ax[i].set_ylabel(labels[i])
    ax[i].grid(alpha=0.3)
    ax[i].legend()

ax[2].set_xlabel("Time after Walk activation (s)")
fig.suptitle(
    "Ground-Supported Walk: Original vs Updated Controller"
)
fig.tight_layout()

fig.savefig(
    RESULTS / "walk_before_after_stability.png",
    dpi=200
)
plt.close(fig)

# --------------------------------------------------
# 5. Before/after contact reliability plot
# --------------------------------------------------

_, old_missed, old_rates, old_overall = contact_stats(
    old_contacts
)

new_contacts = []
for row in contact_rows:
    r = {}
    for i, leg in enumerate(LEGS):
        j = 1 + 3 * i
        r[f"{leg}_scheduled"] = row[j]
        r[f"{leg}_detected"] = row[j + 1]
    new_contacts.append(r)

_, new_missed, new_rates, new_overall = contact_stats(
    new_contacts
)

categories = LEGS + ["Overall"]
old_values = list(old_rates) + [old_overall]
new_values = list(new_rates) + [new_overall]

x = np.arange(len(categories))
width = 0.36

fig, ax = plt.subplots(figsize=(10, 5.8))

bars1 = ax.bar(
    x - width / 2, old_values, width,
    label="Original controller"
)

bars2 = ax.bar(
    x + width / 2, new_values, width,
    label="Updated controller"
)

for bars in (bars1, bars2):
    for bar in bars:
        height = bar.get_height()
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            height + 0.3,
            f"{height:.1f}%",
            ha="center",
            fontsize=9
        )

ax.set_xticks(x)
ax.set_xticklabels(categories)
ax.set_ylabel("Missed scheduled contacts (%)")
ax.set_title(
    "Ground-Supported Walk: Contact Reliability Comparison"
)
ax.grid(axis="y", alpha=0.3)
ax.legend()

fig.tight_layout()
fig.savefig(
    RESULTS / "walk_before_after_contact_comparison.png",
    dpi=200
)
plt.close(fig)

# --------------------------------------------------
# 6. Report verified results
# --------------------------------------------------

print("\nGROUND-SUPPORTED WALK COMPARISON")
print("--------------------------------------")
print(f"Updated Walk duration: {end-start:.4f} s")
print(f"Updated end mode: {end_mode}")
print(f"Updated body samples: {len(body_rows)}")
print(f"Updated gait samples: {len(contact_rows)}")
print(f"Updated velocity samples: {len(velocity_rows)}")

print(
    "Maximum absolute roll: %.3f deg"
    % np.max(np.abs(new[:, 1]))
)
print(
    "Maximum absolute pitch: %.3f deg"
    % np.max(np.abs(new[:, 2]))
)
print(
    "Mean body height: %.4f m"
    % np.mean(new[:, 3])
)

print(f"Original missed-contact rate: {old_overall:.2f}%")
print(f"Updated missed-contact rate: {new_overall:.2f}%")

relative_reduction = (
    100 * (old_overall - new_overall) / old_overall
)
print(
    f"Relative contact-mismatch reduction: "
    f"{relative_reduction:.2f}%"
)

print(f"\nEvidence saved in: {RESULTS}")
