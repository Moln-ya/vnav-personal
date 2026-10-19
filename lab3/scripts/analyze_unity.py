#!/usr/bin/env python3
"""Analyze official TESSE telemetry; independent-model data is never read."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

root = Path(__file__).resolve().parents[1]
data = np.genfromtxt(root/"report/unity_flight.csv", delimiter=",", names=True)
t = data["trajectory_time"]
p = np.column_stack([data[n] for n in ("x","y","z")])
pd = np.column_stack([data[n] for n in ("xd","yd","zd")])
error = np.linalg.norm(p-pd, axis=1)
steady = t >= 8*np.pi  # exclude the first two nominal laps
phase = np.unwrap(np.arctan2(p[steady,0], p[steady,1]))
laps = float((phase[-1]-phase[0])/(2*np.pi))
summary = {
    "source": "Unmodified official MIT Lab3 Unity/TESSE; Box64 0.3.6 on ARM64",
    "trajectory": "x=5*sin(t/2), y=5*cos(t/2), z=2, yaw=-t/2",
    "gains": {"kx":2.0,"kv":2.4,"kr":1.8,"komega":0.7,"hz":200},
    "samples":len(t), "trajectory_duration_s":float(t[-1]),
    "steady_start_s":float(8*np.pi), "observed_steady_laps":laps,
    "steady_position_rms_m":float(np.sqrt(np.mean(error[steady]**2))),
    "steady_position_max_m":float(error[steady].max()),
    "steady_altitude_min_m":float(p[steady,2].min()),
    "steady_altitude_max_m":float(p[steady,2].max()),
    "steady_radial_rms_m":float(np.sqrt(np.mean((np.linalg.norm(p[steady,:2],axis=1)-5)**2))),
    "all_values_finite":bool(np.isfinite(p).all() and np.isfinite(error).all()),
}
summary["flight_completed"] = bool(laps >= 1 and t[-1] >= 55 and summary["all_values_finite"])
(root/"report/unity_metrics.json").write_text(json.dumps(summary,indent=2)+"\n")
fig, ax = plt.subplots(2,2,figsize=(11,8))
ax[0,0].plot(pd[:,0],pd[:,1],"--",label="Desired")
ax[0,0].plot(p[:,0],p[:,1],label="Official TESSE")
ax[0,0].set(xlabel="x [m]",ylabel="y [m]",aspect="equal",title="Circular trajectory")
ax[0,0].legend()
ax[0,1].plot(t,p[:,2],label="Actual z")
ax[0,1].plot(t,pd[:,2],"--",label="Desired z")
ax[0,1].set(xlabel="Trajectory time [s]",ylabel="Height [m]")
ax[0,1].legend()
ax[1,0].plot(t,error)
ax[1,0].set(xlabel="Trajectory time [s]",ylabel="Position error [m]")
ax[1,0].axvline(8*np.pi,color="gray",ls="--",label="Steady window starts")
ax[1,0].legend()
ax[1,1].plot(t[steady],(phase-phase[0])/(2*np.pi))
ax[1,1].set(xlabel="Trajectory time [s]",ylabel="Observed turns",title="Actual position phase")
for a in ax.flat:
    a.grid(alpha=.25)
fig.suptitle("MIT VNAV Lab3 — official Unity/TESSE flight")
fig.tight_layout(rect=(0,0,1,.95))
fig.savefig(root/"report/unity_tracking.pdf")
fig.savefig(root/"report/unity_tracking.png",dpi=150)
print(json.dumps(summary,indent=2))
if not summary["flight_completed"]:
    raise SystemExit("Official flight did not include a complete steady circle")
