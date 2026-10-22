#!/usr/bin/env bash
# Run INSIDE the prepared ROS container, on a private ROS master.
set -eo pipefail
source /opt/ros/noetic/setup.bash
source /tmp/lab4_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11315
export ROS_HOSTNAME=localhost
LAB4_DIR=/workspace/lab4
mkdir -p "$LAB4_DIR/report"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
roscore -p 11315 > /tmp/lab4-validation-master.log 2>&1 & pids+=("$!")
sleep 2
rosparam set /use_sim_time true
roslaunch trajectory_generation traj_following.launch gains_file:=/workspace/lab3/controller_pkg/config/gains.yaml v_max:=3.0 a_max:=2.0 > /tmp/lab4-validation-follower.log 2>&1 & pids+=("$!")
roslaunch planner_pkg traj_gen.launch simulator_data_directory:="$LAB4_DIR/simulator_data" > /tmp/lab4-validation-gates.log 2>&1 & pids+=("$!")
rosbag record -O "$LAB4_DIR/report/rigid_body_validation.bag" /current_state /desired_state > /tmp/lab4-validation-bag.log 2>&1 & pids+=("$!")
sleep 2
python3 "$LAB4_DIR/scripts/rigid_body_validation.py" _duration:=180 > /tmp/lab4-validation-plant.log 2>&1
