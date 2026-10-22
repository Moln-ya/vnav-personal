#!/usr/bin/env python3
"""Rebuild unity_flight.csv from a run's rosbag, pairing each state with the
reference sampled at the state's own time. Usage: rebuild_flight_csv.py RUN_DIR

The trajectory node publishes each reference right after the state it was
sampled for, so stamp - time_from_start of the latest earlier state is the
trajectory start; the most common value is taken. The monitor's own CSV is kept
as unity_flight_monitor.csv.
"""
import collections
import csv
import os
import sys
import numpy as np
import rosbag

run = sys.argv[1]
states, reference, origins = [], [], []
last = None
with rosbag.Bag(os.path.join(run, 'unity_race.bag')) as bag:
    for topic, m, _ in bag.read_messages(topics=['/current_state', '/desired_state']):
        if topic == '/current_state':
            last = m.header.stamp.to_sec()
            p, q = m.pose.pose.position, m.pose.pose.orientation
            states.append((last, p.x, p.y, p.z, q.x, q.y, q.z, q.w))
        elif m.transforms:
            t = m.time_from_start.to_sec()
            p = m.transforms[0].translation
            reference.append((t, p.x, p.y, p.z))
            if last is not None and t > 0.1:
                origins.append(round(last - t, 3))
origin = collections.Counter(origins).most_common(1)[0][0]
ref = np.array(sorted(set(reference)))
states = np.array(states)
t = np.clip(states[:, 0] - origin, 0.0, ref[-1, 0])
keep = states[:, 0] - origin >= 0  # from the trajectory start, as the monitor logs
xd = np.column_stack([np.interp(t, ref[:, 0], ref[:, k]) for k in (1, 2, 3)])
target = os.path.join(run, 'unity_flight.csv')
monitor_copy = os.path.join(run, 'unity_flight_monitor.csv')
if os.path.exists(target) and not os.path.exists(monitor_copy):
    os.rename(target, monitor_copy)
with open(target, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(['t', 'x', 'y', 'z', 'xd', 'yd', 'zd', 'trajectory_time', 'qx', 'qy', 'qz', 'qw'])
    start = states[keep, 0][0]
    for s, r, tt in zip(states[keep], xd[keep], t[keep]):
        w.writerow([s[0] - start, s[1], s[2], s[3], r[0], r[1], r[2], tt, s[4], s[5], s[6], s[7]])
print('origin', origin, 'rows', int(keep.sum()), '->', target)
