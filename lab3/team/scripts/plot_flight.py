#!/usr/bin/env python3
"""Plot recorded ROS closed-loop data; explicitly identify the plant source."""
from pathlib import Path
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
data = np.genfromtxt(root / "report/ros_flight.csv", delimiter=",", names=True)
steady = data["trajectory_time"] >= 8*np.pi
errors = np.sqrt(sum((data[a]-data[b])**2 for a, b in
                     [("x", "xd"), ("y", "yd"), ("z", "zd")]))
metrics = {
    "plant": "independent rigid-body model, NOT TESSE",
    "controller": "compiled ROS Noetic controller_node via real ROS topics",
    "steady_window_trajectory_time_s": [float(data["trajectory_time"][steady][0]),
                                         float(data["trajectory_time"][-1])],
    "steady_position_rms_m": float(np.sqrt(np.mean(errors[steady]**2))),
    "steady_position_max_m": float(np.max(errors[steady])),
    "sample_count": len(data),
}
(root / "report/ros_metrics.json").write_text(json.dumps(metrics, indent=2)+"\n")
fig, axes = plt.subplots(1, 2, figsize=(10, 4))
axes[0].plot(data["xd"][steady], data["yd"][steady], "--", label="Desired", color="#c17b00")
axes[0].plot(data["x"][steady], data["y"][steady], label="Actual", color="#0075ad")
axes[0].set(xlabel="World x (m)", ylabel="World y (m)", title="Steady circular flight")
axes[0].axis("equal")
axes[0].legend()
axes[1].plot(data["trajectory_time"], errors, color="#0075ad")
axes[1].axvline(8*np.pi, color="0.4", linestyle="--", label="Steady window starts")
axes[1].set(xlabel="Trajectory time (s)", ylabel="Position error (m)", title="ROS closed-loop tracking")
axes[1].legend()
for ax in axes:
    ax.grid(alpha=0.25)
fig.suptitle("Independent rigid-body model + actual ROS controller (not TESSE)")
fig.tight_layout(rect=[0, 0, 1, 0.92])
fig.savefig(root / "report/ros_tracking.pdf")
fig.savefig(root / "report/ros_tracking.png", dpi=160)
print(json.dumps(metrics, indent=2))
