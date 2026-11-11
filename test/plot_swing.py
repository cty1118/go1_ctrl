#!/usr/bin/env python3
# Owner: Member 4. Plots the CSV written by swing_offline (swing trajectory and foothold).
# Usage: python3 test/plot_swing.py /tmp/swing.csv [out.png]
import sys

import matplotlib.pyplot as plt
import numpy as np

path = sys.argv[1] if len(sys.argv) > 1 else "/tmp/swing.csv"
out = sys.argv[2] if len(sys.argv) > 2 else "swing_trajectory.png"
d = np.genfromtxt(path, delimiter=",", names=True)

fig, ax = plt.subplots(2, 2, figsize=(11, 7))

ax[0, 0].plot(d["px"], d["pz"], label="foot path")
ax[0, 0].plot(d["land_x"][-1], d["land_z"][-1], "x", ms=10, label="final foothold")
ax[0, 0].set(xlabel="x body [m]", ylabel="z body [m]", title="Swing path (side view)")
ax[0, 0].axis("equal")
ax[0, 0].legend()

for k in ("px", "py", "pz"):
    ax[0, 1].plot(d["s"], d[k], label=k)
ax[0, 1].set(xlabel="swing progress s", ylabel="position [m]", title="p_des(s)")
ax[0, 1].legend()

# Finite-difference velocity from the logged positions, to check v_des.
t = d["t"]
for k, c in (("x", "C0"), ("z", "C2")):
    ax[1, 0].plot(d["s"], d["v" + k], c, label="v%s analytic" % k)
    ax[1, 0].plot(d["s"], np.gradient(d["p" + k], t), c + "--", label="v%s from dp/dt" % k)
ax[1, 0].set(xlabel="swing progress s", ylabel="velocity [m/s]", title="v_des(s)")
ax[1, 0].legend()

ax[1, 1].plot(d["s"], d["land_x"], label="foothold x")
ax[1, 1].set(xlabel="swing progress s", ylabel="foothold x [m]",
             title="Raibert foothold (frozen after s = 0.8)")
ax2 = ax[1, 1].twinx()
ax2.plot(d["s"], d["v_body_x"], "C1:", label="body v_x measured")
ax2.plot(d["s"], d["v_filt_x"], "C1--", label="body v_x filtered")
ax2.set_ylabel("body v_x [m/s]")
ax[1, 1].legend(loc="upper left")
ax2.legend(loc="lower right")

fig.tight_layout()
fig.savefig(out, dpi=150)
print("saved", out)
plt.show()
