#!/usr/bin/env python3
"""Integration checks against the actual ROS trajectory and hover executables.
Run with an isolated roscore and /use_sim_time=true. No Unity required.
"""
import math
import os
import subprocess
import threading
import time
import numpy as np
import rospy
import tf.transformations as tf
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint


def main():
    rospy.init_node('lab4_integration_checks')
    clock = rospy.Publisher('/clock', Clock, queue_size=1, latch=True)
    now = [1.0]
    def step(dt=0.01):
        now[0] += dt
        clock.publish(Clock(clock=rospy.Time.from_sec(now[0])))
        time.sleep(0.006)
    def wait_for(test, seconds=5):
        end = time.monotonic() + seconds
        while not test() and time.monotonic() < end:
            step()
        assert test(), 'Timeout waiting for ROS connection or state'
    clock.publish(Clock(clock=rospy.Time.from_sec(now[0])))
    children = []
    log = open('/tmp/lab4-integration-nodes.log', 'w')
    def launch(pkg, executable, ns):
        p = subprocess.Popen(['rosrun', pkg, executable, '__ns:='+ns], stdout=log, stderr=log)
        children.append(p)
        return p
    def pose(x, y, z, yaw):
        p = Pose()
        p.position.x, p.position.y, p.position.z = x, y, z
        q = tf.quaternion_from_euler(0, 0, yaw)
        p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = q
        return p
    try:
        launch('trajectory_generation', 'trajectory_generation_node', '/check')
        states, targets = [], []
        rospy.Subscriber('/check/desired_state', MultiDOFJointTrajectoryPoint,
                         lambda m: targets.append(m), queue_size=100)
        gates = rospy.Publisher('/check/desired_traj_vertices', PoseArray, queue_size=1, latch=True)
        odom = rospy.Publisher('/check/current_state', Odometry, queue_size=1)
        wait_for(lambda: gates.get_num_connections() and odom.get_num_connections())
        gates.publish(PoseArray())
        for _ in range(10): step()
        assert not targets, 'Empty input must not create a trajectory'
        msg = PoseArray()
        msg.poses = [pose(1, 0, 2, math.radians(-179)), pose(2, 1, 2, math.radians(-175))]
        msg.header.frame_id = 'world'
        gates.publish(msg)
        for _ in range(20): step()
        assert not targets, 'Waypoints must wait for a valid current state'
        current = Odometry()
        current.pose.pose = pose(0, 0, 2, math.radians(179))
        odom.publish(current)
        wait_for(lambda: len(targets) > 4)
        previous = -1
        while now[0] < 24:
            if int(now[0]*100) % 20 == 0: gates.publish(msg)
            step()
        assert len(targets) > 500
        samples = []
        yaw_values = []
        for point in targets:
            assert len(point.transforms) == len(point.velocities) == len(point.accelerations) == 1
            t = point.time_from_start.to_sec()
            assert t + 1e-9 >= previous, 'Repeated gates must not restart the trajectory'
            previous = t
            p, v, a = point.transforms[0], point.velocities[0], point.accelerations[0]
            q = [p.rotation.x, p.rotation.y, p.rotation.z, p.rotation.w]
            assert abs(np.linalg.norm(q)-1) < 1e-9
            row = [t, p.translation.x, p.translation.y, p.translation.z,
                   v.linear.x, v.linear.y, v.linear.z, a.linear.x, a.linear.y, a.linear.z]
            assert np.isfinite(row).all()
            yaw_values.append(tf.euler_from_quaternion(q)[2])
            if not samples or t > samples[-1][0] + 1e-8: samples.append(row)
        samples = np.array(samples)
        final = targets[-1]
        assert np.linalg.norm(samples[-1,1:4] - [2,1,2]) < 1e-7
        assert np.linalg.norm(samples[-1,4:]) < 1e-7
        assert targets[-100].time_from_start == final.time_from_start, 'Must continue holding the end'
        # Independent differential consistency of published velocity and acceleration.
        velocity_from_position = np.gradient(samples[:,1:4], samples[:,0], axis=0)
        acceleration_from_velocity = np.gradient(samples[:,4:7], samples[:,0], axis=0)
        assert np.max(np.abs(velocity_from_position[2:-2] - samples[2:-2,4:7])) < 0.01
        assert np.max(np.abs(acceleration_from_velocity[2:-2] - samples[2:-2,7:10])) < 0.02
        unwrapped = np.unwrap(yaw_values)
        assert np.ptp(unwrapped) < math.radians(12), 'Yaw must take the short path across pi'
        assert abs(math.atan2(math.sin(yaw_values[-1]-math.radians(-175)),
                              math.cos(yaw_values[-1]-math.radians(-175)))) < 1e-7
        print('PASS: empty input, wait for state, repeated gates, finite states, yaw wrapping, derivatives, endpoint hold')

        launch('trajectory_generation', 'trajectory_generation_node', '/invalid')
        invalid_targets = []
        rospy.Subscriber('/invalid/desired_state', MultiDOFJointTrajectoryPoint,
                         lambda m: invalid_targets.append(m), queue_size=10)
        bad_gates = rospy.Publisher('/invalid/desired_traj_vertices', PoseArray, queue_size=1)
        bad_odom = rospy.Publisher('/invalid/current_state', Odometry, queue_size=1)
        wait_for(lambda: bad_gates.get_num_connections() and bad_odom.get_num_connections())
        bad_odom.publish(current)
        invalid = PoseArray(); invalid.poses = [pose(float('nan'),0,2,0)]
        bad_gates.publish(invalid)
        for _ in range(20): step()
        assert not invalid_targets, 'NaN waypoint must not reach optimizer'
        single = PoseArray(); single.poses = [pose(0,0,2,math.radians(179))]
        bad_odom.publish(current)
        bad_gates.publish(single)
        wait_for(lambda: len(invalid_targets) > 10)
        for _ in range(100): step()
        p = invalid_targets[-1]
        assert np.isfinite([p.transforms[0].translation.x, p.velocities[0].linear.x]).all()
        assert abs(p.transforms[0].translation.z-2) < 1e-8
        print('PASS: invalid waypoint rejection, one coincident gate, minimum segment duration')

        launch('planner_pkg', 'simple_traj_planner', '/hover')
        hover_targets = []
        rospy.Subscriber('/hover/desired_state', MultiDOFJointTrajectoryPoint,
                         lambda m: hover_targets.append(m), queue_size=100)
        hover_gates = rospy.Publisher('/hover/desired_traj_vertices', PoseArray, queue_size=1)
        wait_for(lambda: hover_gates.get_num_connections())
        hover_gates.publish(msg)
        wait_for(lambda: len(hover_targets) > 30)
        h = hover_targets[-1]
        assert h.transforms[0].translation.x == 1
        assert h.transforms[0].translation.z == 2
        assert h.velocities[0].linear.x == h.accelerations[0].linear.x == 0
        hover_gates.publish(PoseArray())
        for _ in range(50): step()
        assert len(hover_targets) > 50, 'Hover publication must continue without new waypoints'
        print('PASS: first-gate hover and continuous publication')
        assert all(p.poll() is None for p in children), 'All nodes must remain alive'
    finally:
        for p in children: p.terminate()
        for p in children:
            try: p.wait(timeout=5)
            except subprocess.TimeoutExpired: p.kill()
        log.close()

if __name__ == '__main__': main()
