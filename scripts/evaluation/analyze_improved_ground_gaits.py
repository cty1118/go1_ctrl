
#!/usr/bin/env python3
"""
EEL 5669 — Member 7: Ground-supported gait evaluation
Extract improved Trot, Canter, and Gallop results from ROS bags.

Controller: c472c21 (ground-gaits-fix)
Leg order in gait_debug: FR, FL, RR, RL
Each leg has 10 values:
    index 1 = scheduled stance
    index 2 = detected contact
    index 3 = contact force (N)
"""

import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rosbag
from tf.transformations import euler_from_quaternion


ROOT = Path.home() / "go1_ground_ws/src/go1_ctrl"
RESULTS = ROOT / "docs/results"
BAGS = Path.home() / "go1_experiments"

GAITS = ("trot", "canter", "gallop")
LEGS = ("FR", "FL", "RR", "RL")

# Previously measured failed-controller results (030c250).
BASELINE = {
    "walk":   {"duration": 5.1972, "roll": 58.47,
               "pitch": 24.87, "missed": 20.18},
    "trot":   {"duration": 4.0538, "roll": 58.62,
               "pitch": 18.39, "missed": 31.60},
    "canter": {"duration": 1.9142, "roll": 59.95,
               "pitch": 22.32, "missed": 40.30},
    "gallop": {"duration": 3.6164, "roll": 58.80,
               "pitch": 20.15, "missed": 41.00},
}

# Independently verified improved Walk trial.
IMPROVED_WALK = {
    "duration": 22.4464,
    "roll": 1.765,
    "pitch": 2.739,
    "missed": 7.63,
}


def save_csv(path, headers, rows):
    with open(path, "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(rows)


def analyze_gait(gait):
    path = BAGS / f"{gait}_ground_improved_trial02.bag"
    if not path.is_file():
        raise FileNotFoundError(path)

    print(f"\nAnalyzing {gait.upper()}: {path.name}", flush=True)

    body = []
    contacts = []
    velocities = []
    mode_events = []

    with rosbag.Bag(str(path)) as bag:
        # Read mode transitions first to obtain the precise gait window.
        for _, msg, stamp in bag.read_messages(
            topics=["/go1_ctrl/mode"]
        ):
            mode_events.append((stamp.to_sec(), msg.data))

        start = None
        end = None

        for index, (time_s, mode) in enumerate(mode_events):
            if mode == f"GAIT {gait}":
                start = time_s
                if index + 1 < len(mode_events):
                    end = mode_events[index + 1][0]
                break

        if start is None or end is None:
            raise ValueError(
                f"Missing complete GAIT-to-STAND interval for {gait}: "
                f"{mode_events}"
            )

        print(f"Gait interval: {start:.4f} to {end:.4f} s")
        print(f"Duration: {end - start:.4f} s")
        print(f"Ending mode: {mode_events[index + 1][1]}")

        topics = [
            "/gazebo/model_states",
            "/go1_ctrl/gait_debug",
            "/go1_ctrl/cmd_vel",
        ]

        for topic, msg, stamp in bag.read_messages(topics=topics):
            t = stamp.to_sec()
            if not (start <= t < end):
                continue

            elapsed = t - start

            if topic == "/gazebo/model_states":
                if "go1_gazebo" not in msg.name:
                    continue

                pose = msg.pose[msg.name.index("go1_gazebo")]
                q = pose.orientation

                roll, pitch, _ = euler_from_quaternion(
                    [q.x, q.y, q.z, q.w]
                )

                body.append([
                    elapsed,
                    pose.position.z,
                    np.degrees(roll),
                    np.degrees(pitch),
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
                        d[3],
                    ])
                contacts.append(row)

            elif topic == "/go1_ctrl/cmd_vel":
                velocities.append([
                    elapsed,
                    msg.linear.x,
                    msg.linear.y,
                    msg.angular.z,
                ])

    if not body or not contacts or not velocities:
        raise ValueError(f"Incomplete data for {gait}")

    save_csv(
        RESULTS / f"ground_{gait}_improved_body_trial02.csv",
        ["time_s", "height_m", "roll_deg", "pitch_deg"],
        body,
    )

    contact_headers = ["time_s"]
    for leg in LEGS:
        contact_headers.extend([
            f"{leg}_scheduled",
            f"{leg}_detected",
            f"{leg}_force_N",
        ])

    save_csv(
        RESULTS / f"ground_{gait}_improved_contacts_trial02.csv",
        contact_headers,
        contacts,
    )

    save_csv(
        RESULTS / f"ground_{gait}_improved_velocity_trial02.csv",
        ["time_s", "vx_mps", "vy_mps", "yaw_rate_radps"],
        velocities,
    )

    b = np.asarray(body, dtype=float)
    c = np.asarray(contacts, dtype=float)
    v = np.asarray(velocities, dtype=float)

    scheduled = np.array([
        np.count_nonzero(c[:, 1 + 3 * i] > 0.5)
        for i in range(4)
    ])

    missed = np.array([
        np.count_nonzero(
            (c[:, 1 + 3 * i] > 0.5)
            & (c[:, 2 + 3 * i] <= 0.5)
        )
        for i in range(4)
    ])

    rates = 100 * missed / scheduled
    overall = 100 * missed.sum() / scheduled.sum()

    roll_max = np.max(np.abs(b[:, 2]))
    pitch_max = np.max(np.abs(b[:, 3]))

    # Individual improved-gait stability plot.
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)

    axes[0].plot(b[:, 0], b[:, 2], linewidth=1)
    axes[0].set_ylabel("Roll (deg)")
    axes[0].grid(alpha=0.3)

    axes[1].plot(b[:, 0], b[:, 3], linewidth=1)
    axes[1].set_ylabel("Pitch (deg)")
    axes[1].grid(alpha=0.3)

    axes[2].plot(b[:, 0], b[:, 1], linewidth=1)
    axes[2].set_ylabel("Body height (m)")
    axes[2].set_xlabel("Elapsed gait time (s)")
    axes[2].grid(alpha=0.3)

    fig.suptitle(
        f"Improved {gait.title()} — Ground-Supported Stability"
    )
    fig.tight_layout()
    fig.savefig(
        RESULTS / f"{gait}_improved_stability.png",
        dpi=180,
    )
    plt.close(fig)

    # Foot-contact mismatch plot.
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.bar(LEGS, rates)
    ax.set_ylabel("Missed scheduled stance contacts (%)")
    ax.set_ylim(0, max(30, float(np.max(rates)) * 1.2))
    ax.set_title(f"Improved {gait.title()} — Contact Consistency")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    fig.savefig(
        RESULTS / f"{gait}_improved_contacts.png",
        dpi=180,
    )
    plt.close(fig)

    print(f"Body samples: {len(body)}")
    print(f"Gait samples: {len(contacts)}")
    print(f"Velocity samples: {len(velocities)}")
    print(f"Maximum absolute roll: {roll_max:.3f} deg")
    print(f"Maximum absolute pitch: {pitch_max:.3f} deg")
    print(f"Mean body height: {np.mean(b[:, 1]):.4f} m")
    print(f"Minimum body height: {np.min(b[:, 1]):.4f} m")
    print(f"Maximum body height: {np.max(b[:, 1]):.4f} m")

    for i, leg in enumerate(LEGS):
        print(
            f"{leg}: {missed[i]}/{scheduled[i]} missed "
            f"({rates[i]:.2f}%)"
        )

    print(f"Overall missed-contact rate: {overall:.2f}%")
    print(
        "Recorded forward speeds:",
        sorted(set(np.round(v[:, 1], 3))),
    )

    return {
        "duration": end - start,
        "roll": float(roll_max),
        "pitch": float(pitch_max),
        "missed": float(overall),
    }


def comparison_chart(improved, metric, ylabel, filename):
    labels = ["Walk", "Trot", "Canter", "Gallop"]
    keys = [label.lower() for label in labels]

    before = [BASELINE[g][metric] for g in keys]
    after = [improved[g][metric] for g in keys]

    x = np.arange(len(keys))
    width = 0.35

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.bar(x - width / 2, before, width, label="Original controller")
    ax.bar(x + width / 2, after, width, label="Improved controller")

    ax.set_xticks(x, labels)
    ax.set_ylabel(ylabel)
    ax.set_title("Ground-Supported Gait Evaluation")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(RESULTS / filename, dpi=180)
    plt.close(fig)


def main():
    RESULTS.mkdir(parents=True, exist_ok=True)

    improved = {"walk": IMPROVED_WALK}

    for gait in GAITS:
        improved[gait] = analyze_gait(gait)

    comparison_chart(
        improved, "roll",
        "Maximum absolute roll (deg)",
        "four_gaits_before_after_roll.png",
    )

    comparison_chart(
        improved, "pitch",
        "Maximum absolute pitch (deg)",
        "four_gaits_before_after_pitch.png",
    )

    comparison_chart(
        improved, "missed",
        "Missed scheduled stance contacts (%)",
        "four_gaits_before_after_contact.png",
    )

    print("\nFour-gait summary:")
    print("Gait     Duration(s)   Max roll   Max pitch   Missed(%)")

    for gait in ("walk",) + GAITS:
        r = improved[gait]
        print(
            f"{gait:<9}"
            f"{r['duration']:>10.4f}"
            f"{r['roll']:>11.3f}"
            f"{r['pitch']:>12.3f}"
            f"{r['missed']:>12.2f}"
        )

    print("\nCSV files and PNG plots saved to:", RESULTS)


if __name__ == "__main__":
    main()
