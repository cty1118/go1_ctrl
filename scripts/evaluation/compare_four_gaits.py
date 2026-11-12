
import csv
from pathlib import Path

import numpy as np
import rosbag

# Member 7: Four-gait quantitative scheduler evaluation.
# Experiments use a suspended Unitree Go1 in Gazebo.

folder = Path.home() / "go1_experiments"

gaits = {
    "Walk":   ("walk_scheduler_trial01.bag",   1.6, 0.80),
    "Trot":   ("trot_scheduler_trial01.bag",   0.6, 0.60),
    "Canter": ("canter_scheduler_trial01.bag", 0.8, 0.50),
    "Gallop": ("gallop_scheduler_trial01.bag", 0.7, 0.45),
}

legs = ["FR", "FL", "RR", "RL"]
summary_rows = []
detail_rows = []


def load_gait_data(path):
    times = []
    samples = []

    with rosbag.Bag(str(path)) as bag:
        for _, msg, stamp in bag.read_messages(
            topics=["/go1_ctrl/gait_debug"]
        ):
            if len(msg.data) != 40:
                continue

            times.append(stamp.to_sec())
            samples.append(
                np.asarray(msg.data, dtype=float).reshape(4, 10)
            )

    if not samples:
        raise RuntimeError(f"No valid gait data in {path}")

    return np.asarray(times), np.stack(samples)


def circular_mean(values):
    """Mean of phase differences, accounting for wraparound."""
    angles = 2.0 * np.pi * values
    vector = np.mean(np.exp(1j * angles))

    result = (np.angle(vector) / (2.0 * np.pi)) % 1.0

    if result > 0.999:
        result = 0.0

    return result


for gait, (filename, expected_period, expected_duty) in gaits.items():

    times, data = load_gait_data(folder / filename)

    # Index 0 = gait phase, index 1 = scheduled contact.
    phases = data[:, :, 0]
    contacts = data[:, :, 1]

    # Detect FR phase resets from nearly 1 back to nearly 0.
    resets = np.where(np.diff(phases[:, 0]) < -0.5)[0] + 1

    if len(resets) < 3:
        raise RuntimeError(
            f"{gait}: insufficient complete cycles"
        )

    # Measured gait period from consecutive FR phase resets.
    cycle_periods = np.diff(times[resets])
    period_mean = float(np.mean(cycle_periods))
    period_std = float(np.std(cycle_periods))

    # Analyze an integer number of complete FR gait cycles.
    start = int(resets[0])
    end = int(resets[-1])

    dt = np.diff(times[start:end + 1])

    if np.any(dt < 0):
        raise RuntimeError(f"{gait}: invalid time intervals")

    duty_values = []

    for leg, name in enumerate(legs):

        # Time-weighted scheduled stance fraction.
        duty = float(
            np.average(contacts[start:end, leg], weights=dt)
        )
        duty_values.append(duty)

        # Phase offset relative to FR:
        # (phase_FR - phase_leg) mod 1.
        relative_phase = (
            phases[start:end, 0] - phases[start:end, leg]
        ) % 1.0

        phase_offset = circular_mean(relative_phase)

        detail_rows.append({
            "gait": gait,
            "leg": name,
            "measured_stance_percent": round(duty * 100, 2),
            "phase_offset_relative_FR": round(phase_offset, 4),
        })

    mean_duty = float(np.mean(duty_values))

    summary_rows.append({
        "gait": gait,
        "expected_period_s": expected_period,
        "measured_period_s": round(period_mean, 4),
        "period_std_s": round(period_std, 4),
        "expected_stance_percent": expected_duty * 100,
        "measured_stance_percent": round(mean_duty * 100, 2),
        "complete_cycles": len(cycle_periods),
        "valid_samples": len(times),
    })

    print(f"\n{gait.upper()}")
    print(f"  Samples: {len(times)}")
    print(f"  Complete cycles: {len(cycle_periods)}")
    print(f"  Expected period: {expected_period:.3f} s")
    print(
        f"  Measured period: {period_mean:.4f} "
        f"+/- {period_std:.4f} s"
    )
    print(f"  Expected stance: {expected_duty * 100:.1f}%")
    print(f"  Measured stance: {mean_duty * 100:.2f}%")

    for leg, name in enumerate(legs):
        row = detail_rows[-4 + leg]
        print(
            f"  {name}: stance={duty_values[leg]*100:.2f}%, "
            f"relative phase={row['phase_offset_relative_FR']:.4f}"
        )


def save_csv(filename, rows):
    path = folder / filename

    with open(path, "w", newline="") as file:
        writer = csv.DictWriter(
            file, fieldnames=list(rows[0].keys())
        )
        writer.writeheader()
        writer.writerows(rows)

    print("Saved:", path)


save_csv("four_gait_comparison.csv", summary_rows)
save_csv("four_gait_leg_details.csv", detail_rows)

print("\nFour-gait quantitative analysis completed!")
