#!/usr/bin/env python3
"""ARM-compatible ROS Lab4 rigid-body validation. This is NOT the official TESSE plant.

The real controller_node and trajectory_generation_node run unchanged. A separate plant
consumes /rotor_speed_cmds and publishes world-frame /current_state and /clock.
Use this for local debugging when the x86_64 Unity player cannot run.
"""
import csv
import math
import os
import threading

import numpy as np
import rospy
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker


def main():
    rospy.init_node("rigid_body_demo")
    output = rospy.get_param("~output", "/workspace/lab4/report/rigid_body_flight.csv")
    duration = float(rospy.get_param("~duration", 140.0))
    clock = rospy.Publisher("/clock", Clock, queue_size=1)
    odom_pub = rospy.Publisher("/current_state", Odometry, queue_size=1)
    actual_pub = rospy.Publisher("/flight/actual", Path, queue_size=1)
    desired_pub = rospy.Publisher("/flight/desired", Path, queue_size=1)
    label_pub = rospy.Publisher("/flight/source", Marker, queue_size=1, latch=True)
    lock = threading.Lock()
    speeds = np.zeros(4)
    target = np.zeros(3)
    target_time = [0.0]
    have_command = threading.Event()

    def on_command(msg):
        if len(msg.angular_velocities) != 4:
            return
        candidate = np.asarray(msg.angular_velocities, dtype=float)
        if not np.all(np.isfinite(candidate)):
            return
        with lock:
            speeds[:] = candidate
        have_command.set()

    rospy.Subscriber("/rotor_speed_cmds", Actuators, on_command, queue_size=1)
    def on_desired(msg):
        if not msg.transforms:
            return
        p = msg.transforms[0].translation
        with lock:
            target[:] = [p.x, p.y, p.z]
            target_time[0] = msg.time_from_start.to_sec()
    rospy.Subscriber("/desired_state", MultiDOFJointTrajectoryPoint,
                     on_desired, queue_size=1)
    x, v, omega = np.array([0.4, 8.0, 2.0]), np.zeros(3), np.zeros(3)
    q = np.array([0.0, 0.0, -0.7749114, -0.6320699]); q /= np.linalg.norm(q)
    arm = 0.3 / math.sqrt(2)
    positions = np.array([[arm, arm, 0], [-arm, arm, 0],
                          [-arm, -arm, 0], [arm, -arm, 0]])
    actual, desired = Path(), Path()
    actual.header.frame_id = desired.header.frame_id = "world"
    label = Marker()
    label.header.frame_id = "world"
    label.ns = "source"
    label.id = 0
    label.type = Marker.TEXT_VIEW_FACING
    label.action = Marker.ADD
    label.pose.position.z = 5
    label.pose.orientation.w = 1
    label.scale.z = 0.35
    label.color.r = label.color.g = label.color.b = label.color.a = 1
    label.text = "Lab4 ROS rigid-body validation (not Unity/TESSE)"
    label_pub.publish(label)
    t, tick = 0.001, 0
    last_target_time, last_advance = -1.0, 0.0
    rows = []
    import time  # Wall time remains usable while this node supplies /clock.
    while not rospy.is_shutdown() and t < duration:
        wall_start = time.monotonic()
        stamp = rospy.Time.from_sec(t)
        clock.publish(Clock(clock=stamp))
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = "world"
        msg.child_frame_id = "base_link_gt"
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = x
        (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
         msg.pose.pose.orientation.z, msg.pose.pose.orientation.w) = q
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = v
        R = tr.quaternion_matrix(q)[:3, :3]
        world_omega = R.dot(omega)
        (msg.twist.twist.angular.x, msg.twist.twist.angular.y,
         msg.twist.twist.angular.z) = world_omega
        odom_pub.publish(msg)
        if tick % 4 == 0:
            pose = PoseStamped(header=msg.header, pose=msg.pose.pose)
            actual.poses.append(pose)
            actual.header.stamp = stamp
            actual_pub.publish(actual)
        with lock:
            current_speeds = speeds.copy()
            current_target = target.copy()
            current_target_time = target_time[0]
        if current_target_time > last_target_time + 1e-6:
            last_target_time, last_advance = current_target_time, t
        if current_target_time > 1.0 and t - last_advance > 5.0:
            break
        if tick % 4 == 0:
            desired_pose = PoseStamped()
            desired_pose.header = msg.header
            (desired_pose.pose.position.x, desired_pose.pose.position.y,
             desired_pose.pose.position.z) = current_target
            desired_pose.pose.orientation.w = 1
            desired.poses.append(desired_pose)
            desired.header.stamp = stamp
            desired_pub.publish(desired)
            rows.append([t, *x, *current_target, current_target_time, *current_speeds])
        # Rotor force and moment are computed independently of the controller's map.
        if have_command.is_set():
            squared = current_speeds * np.abs(current_speeds)
            lift = 1e-3 * squared
            forces = np.column_stack((np.zeros(4), np.zeros(4), lift))
            moment = np.cross(positions, forces).sum(axis=0)
            moment[2] += np.dot([1, -1, 1, -1], 1e-5 * squared)
            for _ in range(5):
                R = tr.quaternion_matrix(q)[:3, :3]
                v += 0.001 * (lift.sum()*R[:, 2] - np.array([0, 0, 9.81]))
                x += 0.001*v
                omega += 0.001*moment  # J=I, omega x J*omega=0.
                speed = np.linalg.norm(omega)
                if speed > 1e-12:
                    dq = tr.quaternion_about_axis(0.001*speed, omega/speed)
                    q = tr.quaternion_multiply(q, dq)
                    q /= np.linalg.norm(q)
        if not np.all(np.isfinite(x)) or np.linalg.norm(x) > 1e3:
            raise RuntimeError("Rigid-body demo diverged")
        tick += 1
        t += 0.005
        time.sleep(max(0.001, 0.003-(time.monotonic()-wall_start)))
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    with open(output, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["t", "x", "y", "z", "xd", "yd", "zd", "trajectory_time",
                         "rotor1", "rotor2", "rotor3", "rotor4"])
        writer.writerows(rows)
    rospy.loginfo("Saved %s; source: independent rigid-body demo, not TESSE", output)


if __name__ == "__main__":
    main()
