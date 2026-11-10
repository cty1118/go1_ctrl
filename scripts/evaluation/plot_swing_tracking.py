import rosbag
import numpy as np
import matplotlib.pyplot as plt
import sys

if len(sys.argv) != 2:
    raise SystemExit("Usage: python3 plot_swing_tracking.py PATH_TO_BAG")

bag_path = sys.argv[1]

import rosbag
import numpy as np
import matplotlib.pyplot as plt

bag_path = "/home/md/go1_experiments/swing_mode3_trial01.bag"

times = []
desired = {leg: [] for leg in range(4)}
measured = {leg: [] for leg in range(4)}

leg_names = ["FR", "FL", "RR", "RL"]

with rosbag.Bag(bag_path) as bag:
    for topic, msg, t in bag.read_messages(
        topics=["/go1_ctrl/swing_debug"]
    ):
        if len(msg.data) != 24:
            continue

        times.append(t.to_sec())

        for leg in range(4):
            start = leg * 6

            desired[leg].append(msg.data[start + 2])
            measured[leg].append(msg.data[start + 5])

times = np.array(times)
if len(times) == 0:
    raise RuntimeError("No valid swing-debug data found.")

times = times - times[0]

fig, axes = plt.subplots(2, 2, figsize=(12, 8))

for leg, ax in enumerate(axes.flat):
    p_des = np.array(desired[leg])
    p = np.array(measured[leg])

    error = p_des - p
    rmse = np.sqrt(np.mean(error ** 2)) * 1000

    ax.plot(times, p_des, label="Desired Z")
    ax.plot(times, p, "--", label="Estimated Z")

    ax.set_title(f"{leg_names[leg]} Leg | Z RMSE = {rmse:.2f} mm")
    ax.set_xlabel("Simulation Time (s)")
    ax.set_ylabel("Foot Z Position (m)")
    ax.grid(True)
    ax.legend()

    print(f"{leg_names[leg]} Z RMSE: {rmse:.3f} mm")

plt.suptitle("Unitree Go1 - Desired vs Estimated Foot Z Position")
plt.tight_layout()

output = bag_path.rsplit(".bag", 1)[0] + "_swing_tracking.png"
plt.savefig(output, dpi=300)

print("Graph saved:", output)
plt.show()
