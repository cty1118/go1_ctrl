import rosbag
import numpy as np
import sys

if len(sys.argv) != 2:
    raise SystemExit("Usage: python3 analyze_swing_3d.py PATH_TO_BAG")

bag_path = sys.argv[1]
leg_names = ["FR", "FL", "RR", "RL"]

samples = []

with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(
        topics=["/go1_ctrl/swing_debug"]
    ):
        if len(msg.data) == 24:
            samples.append(msg.data)

data = np.array(samples).reshape(-1, 4, 6)

print("Leg | X RMSE | Y RMSE | Z RMSE | 3D RMSE | Max Error")
print("All results in millimeters")

for leg in range(4):
    desired = data[:, leg, 0:3]
    measured = data[:, leg, 3:6]

    error = desired - measured

    axis_rmse = np.sqrt(np.mean(error**2, axis=0)) * 1000

    error_magnitude = np.linalg.norm(error, axis=1) * 1000

    rmse_3d = np.sqrt(np.mean(error_magnitude**2))
    max_error = np.max(error_magnitude)

    print(
        f"{leg_names[leg]} | "
        f"{axis_rmse[0]:.3f} | "
        f"{axis_rmse[1]:.3f} | "
        f"{axis_rmse[2]:.3f} | "
        f"{rmse_3d:.3f} | "
        f"{max_error:.3f}"
    )
