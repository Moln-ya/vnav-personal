#!/usr/bin/env python3
"""Record live telemetry from the official Unity flight and detect final hold."""
import csv
import os
import time
import threading
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint

rospy.init_node('unity_race_monitor')
lock = threading.Lock()
target = [None]
rows = []
reference = []  # (time_from_start, x, y, z) of every desired sample
last_progress = [0.0, 0.0]
started = [None]
last_log = [0.0]
# Flown path for the RViz video, from the moment the reference starts.
actual_path = Path()
actual_path.header.frame_id = 'world'
actual_pub = rospy.Publisher('/race/actual', Path, queue_size=1)
source_pub = rospy.Publisher('/race/source', Marker, queue_size=1, latch=True)
source = Marker()
source.header.frame_id = 'world'
source.ns, source.type, source.action = 'source', Marker.TEXT_VIEW_FACING, Marker.ADD
source.pose.position.x, source.pose.position.y, source.pose.position.z = -24.0, 19.0, 3.0
source.pose.orientation.w = 1.0
source.scale.z = 2.0
source.color.r = source.color.g = source.color.b = source.color.a = 1.0
source.text = 'Official MIT Lab4 TESSE / GPU'
source_pub.publish(source)

def desired(m):
    if m.transforms:
        p = m.transforms[0].translation
        with lock:
            target[0] = m
            reference.append((m.time_from_start.to_sec(), p.x, p.y, p.z))

def current(m):
    with lock: d = target[0]
    if d is None: return
    stamp = m.header.stamp.to_sec()
    t = d.time_from_start.to_sec()
    if started[0] is None: started[0] = stamp
    p = m.pose.pose.position
    pd = d.transforms[0].translation
    q = m.pose.pose.orientation
    rows.append([stamp-started[0],p.x,p.y,p.z,pd.x,pd.y,pd.z,t,q.x,q.y,q.z,q.w])
    if not actual_path.poses or stamp - actual_path.poses[-1].header.stamp.to_sec() >= 0.05:
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = m.header.stamp
        pose.pose = m.pose.pose
        actual_path.poses.append(pose)
        actual_path.header.stamp = m.header.stamp
        actual_pub.publish(actual_path)
    if t > last_progress[0] + 1e-5: last_progress[:] = [t,stamp]
    if stamp-last_log[0] > 5:
        error = np.linalg.norm([p.x-pd.x,p.y-pd.y,p.z-pd.z])
        print('trajectory_time={:.2f} position=({:.2f},{:.2f},{:.2f}) error={:.3f}m'.format(t,p.x,p.y,p.z,error),flush=True)
        last_log[0] = stamp

rospy.Subscriber('/desired_state',MultiDOFJointTrajectoryPoint,desired,queue_size=1)
rospy.Subscriber('/current_state',Odometry,current,queue_size=100)
finish = time.monotonic() + 600
complete = False
while not rospy.is_shutdown() and time.monotonic() < finish:
    time.sleep(.2)
    if rows and last_progress[0] > 5:
        row = rows[-1]
        held = row[0]+started[0]-last_progress[1]
        if held > 10:
            error = np.linalg.norm(np.array(row[1:4])-row[4:7])
            if error < .2 or held > 30:
                complete = error < .2
                print('Final hold: {:.1f}s error={:.4f}m complete={}'.format(held,error,complete),flush=True)
                break
# The live pairing above can be off by a sample or more either way: a state may
# arrive before the reference sampled at its stamp, or wait in the queue while a
# newer reference arrives. Re-pair by time: stamp - time_from_start equals the
# trajectory start for matched samples, which are the most common value.
with lock: ref = np.array(sorted(set(reference)))
if rows and len(ref) > 1:
    data = np.array(rows)
    stamps = data[:,0] + started[0]
    moving = (data[:,7] > 0) & (data[:,7] < ref[-1,0])
    if moving.any():
        offsets, counts = np.unique(np.round(stamps[moving] - data[moving,7], 3), return_counts=True)
        origin = offsets[counts.argmax()]
        aligned = np.clip(stamps - origin, ref[0,0], ref[-1,0])
        for k in range(3): data[:,4+k] = np.interp(aligned, ref[:,0], ref[:,1+k])
        data[:,7] = aligned
        rows = data.tolist()
output_dir = os.environ.get('LAB4_OUTPUT_DIR', '/workspace/lab4/report')
os.makedirs(output_dir,exist_ok=True)
with open(os.path.join(output_dir,'unity_flight.csv'),'w',newline='') as f:
    w=csv.writer(f);w.writerow(['t','x','y','z','xd','yd','zd','trajectory_time','qx','qy','qz','qw']);w.writerows(rows)
if not complete: raise RuntimeError('Unity run did not reach a settled final pose')
