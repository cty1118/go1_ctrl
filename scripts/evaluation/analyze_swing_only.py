
#!/usr/bin/env python3
"""Member 7: Swing-only FK foot-tracking evaluation."""

import csv
import sys
from pathlib import Path

import numpy as np
import rosbag

if len(sys.argv) != 2:
    raise SystemExit(
        "Usage: python3 analyze_swing_only.py PATH_TO_BAG"
    )

bag_path = Path(sys.argv[1]).expanduser()
leg_names = ["FR", "FL", "RR", "RL"]

swing_data = []
gait_data = []
swing_times = []
gait_times = []

with rosbag.Bag(str(bag_path)) as bag:
    for topic, msg, stamp in bag.read_messages(
        topics=[
            "/go1_ctrl/swing_debug",
            "/go1_ctrl/gait_debug",
        ]
    ):
        if topic == "/go1_ctrl/swing_debug":
            if len(msg.data) != 24:
                raise RuntimeError("Unexpected swing data format")
            swing_data.append(np.array(msg.data).reshape(4, 6))
            swing_times.append(stamp.to_sec())

        elif topic == "/go1_ctrl/gait_debug":
            if len(msg.data) != 40:
                raise RuntimeError("Unexpected gait data format")
            gait_data.append(np.array(msg.data).reshape(4, 10))
            gait_times.append(stamp.to_sec())

if not swing_data or len(swing_data) != len(gait_data):
    raise RuntimeError(
        "Swing and gait message counts do not match. "
        "Cannot safely pair samples."
    )

# Both topics are published once per GAIT controller cycle.
# Pair messages by their per-topic sequence order.
swing = np.stack(swing_data)
gait = np.stack(gait_data)

# Check that paired messages were recorded close together.
time_difference = np.abs(
    np.array(swing_times) - np.array(gait_times)
)

max_difference = float(np.max(time_difference))

if max_difference > 0.02:
    raise RuntimeError(
        f"Possible topic misalignment: "
        f"maximum timestamp difference = {max_difference:.4f} s"
    )

# swing_debug: 4 legs x 6 values
# [desired_x,y,z, FK_estimated_x,y,z]
#
# gait_debug: 4 legs x 10 values
# Index 1 = scheduled contact
# 0 = swing, 1 = stance

rows = []
all_squared_errors = []

print("\nSWING-ONLY FOOT TRACKING EVALUATION")
print("Bag:", bag_path.name)
print("Paired samples:", len(swing))
print(f"Max timestamp separation: {max_difference:.5f} s")
print()

for leg, name in enumerate(leg_names):

    swing_mask = gait[:, leg, 1] < 0.5

    desired = swing[swing_mask, leg, 0:3]
    estimated = swing[swing_mask, leg, 3:6]

    if len(desired) == 0:
        print(f"{name}: No swing samples found")
        continue

    error = desired - estimated

    if not np.all(np.isfinite(error)):
        raise RuntimeError(f"{name}: Non-finite foot position")

    error_3d = np.linalg.norm(error, axis=1)

    rmse_3d = np.sqrt(np.mean(error_3d ** 2)) * 1000
    mean_3d = np.mean(error_3d) * 1000
    max_3d = np.max(error_3d) * 1000
    rmse_z = np.sqrt(np.mean(error[:, 2] ** 2)) * 1000

    all_squared_errors.extend((error_3d ** 2).tolist())

    rows.append({
        "leg": name,
        "swing_samples": len(desired),
        "rmse_3d_mm": round(rmse_3d, 3),
        "mean_error_3d_mm": round(mean_3d, 3),
        "max_error_3d_mm": round(max_3d, 3),
        "rmse_z_mm": round(rmse_z, 3),
    })

    print(
        f"{name}: samples={len(desired)}, "
        f"3D_RMSE={rmse_3d:.3f} mm, "
        f"Max={max_3d:.3f} mm, "
        f"Z_RMSE={rmse_z:.3f} mm"
    )

if not all_squared_errors:
    raise RuntimeError("No swing samples in this recording")

overall_rmse = (
    np.sqrt(np.mean(all_squared_errors)) * 1000
)

print(f"\nOverall swing-only 3D RMSE: {overall_rmse:.3f} mm")

output = bag_path.with_name(
    bag_path.stem + "_swing_only_summary.csv"
)

with open(output, "w", newline="") as file:
    writer = csv.DictWriter(
        file, fieldnames=list(rows[0].keys())
    )
    writer.writeheader()
    writer.writerows(rows)

print("Saved:", output)
print("Analysis completed successfully!")
