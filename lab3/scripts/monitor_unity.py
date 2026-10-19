#!/usr/bin/env python3
"""Record live official TESSE telemetry and publish paths for the RViz video."""
import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path as FilePath

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry, Path
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker

parser = argparse.ArgumentParser()
parser.add_argument("--output", required=True)
parser.add_argument("--duration", type=float, default=50.0)
parser.add_argument("--wall-timeout", type=float, default=300.0)
args = parser.parse_args()
rospy.init_node("lab3_unity_monitor", anonymous=True)
lock = threading.Lock()
rows = []
reference = None
rotors = [0.0]*4
actual_path, desired_path = Path(), Path()
actual_path.header.frame_id = desired_path.header.frame_id = "world"
actual_pub = rospy.Publisher("/flight/actual", Path, queue_size=1)
desired_pub = rospy.Publisher("/flight/desired", Path, queue_size=1)
source_pub = rospy.Publisher("/flight/source", Marker, queue_size=1, latch=True)
last_path_t = -1.0

def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

def on_reference(msg):
    global reference
    if not msg.transforms:
        return
    with lock:
        reference = msg

def on_rotors(msg):
    global rotors
    if len(msg.angular_velocities) == 4:
        with lock:
            rotors = list(msg.angular_velocities)

def on_state(msg):
    global last_path_t
    with lock:
        if reference is None:
            return
        ref = reference
        speed = list(rotors)
        target = ref.transforms[0]
        p = msg.pose.pose.position
        pd = target.translation
        w = msg.twist.twist.angular
        stamp = msg.header.stamp.to_sec()
        rows.append([stamp, ref.time_from_start.to_sec(), p.x,p.y,p.z,
                     pd.x,pd.y,pd.z,yaw(msg.pose.pose.orientation),yaw(target.rotation),
                     w.x,w.y,w.z,*speed])
        if stamp-last_path_t < 0.1:
            return
        last_path_t = stamp
        actual_path.header.stamp = desired_path.header.stamp = msg.header.stamp
        actual_path.poses.append(PoseStamped(header=msg.header,pose=msg.pose.pose))
        desired = PoseStamped()
        desired.header = msg.header
        desired.pose.position.x,desired.pose.position.y,desired.pose.position.z = pd.x,pd.y,pd.z
        desired.pose.orientation = target.rotation
        desired_path.poses.append(desired)
        actual_path.poses = actual_path.poses[-1200:]
        desired_path.poses = desired_path.poses[-1200:]
        actual_pub.publish(actual_path)
        desired_pub.publish(desired_path)

rospy.Subscriber("/desired_state",MultiDOFJointTrajectoryPoint,on_reference,queue_size=1)
rospy.Subscriber("/rotor_speed_cmds",Actuators,on_rotors,queue_size=1)
rospy.Subscriber("/current_state",Odometry,on_state,queue_size=100)
marker = Marker()
marker.header.frame_id = "world"
marker.type = Marker.TEXT_VIEW_FACING
marker.action = Marker.ADD
marker.pose.position.z = 5
marker.pose.orientation.w = 1
marker.scale.z = 0.35
marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1
marker.text = "Official MIT Lab3 TESSE / Box64"
source_pub.publish(marker)
started = time.monotonic()
next_progress = 0
complete = False
try:
    while not rospy.is_shutdown() and time.monotonic()-started < args.wall_timeout:
        with lock:
            latest = rows[-1] if rows else None
        if latest and latest[1] >= next_progress:
            error = np.linalg.norm(np.array(latest[2:5])-latest[5:8])
            print("trajectory t=%.2f s, position=%s, error=%.3f m" %
                  (latest[1],latest[2:5],error),flush=True)
            next_progress += 5
        if latest and latest[1] >= args.duration:
            complete = True
            break
        time.sleep(0.1)
finally:
    with lock:
        data = np.asarray(rows)
    path = FilePath(args.output)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open("w",newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t","trajectory_time","x","y","z","xd","yd","zd",
                         "yaw","yawd","wx_world","wy_world","wz_world",
                         "rotor1","rotor2","rotor3","rotor4"])
        writer.writerows(data)
    if len(data):
        error = np.linalg.norm(data[:,2:5]-data[:,5:8],axis=1)
        steady = data[:,1] >= min(2*4*math.pi,args.duration/2)
        summary = dict(source="Official MIT Lab3 Unity/TESSE via Box64 0.3.6",
                       complete_duration=complete, samples=len(data),
                       trajectory_duration_s=float(data[-1,1]),
                       steady_position_rms_m=float(np.sqrt(np.mean(error[steady]**2))) if steady.any() else None,
                       steady_position_max_m=float(error[steady].max()) if steady.any() else None,
                       final_position_error_m=float(error[-1]))
        path.with_suffix(".json").write_text(json.dumps(summary,indent=2)+"\n")
        print(json.dumps(summary,indent=2),flush=True)
if not complete:
    raise SystemExit("Did not record the requested trajectory duration")
