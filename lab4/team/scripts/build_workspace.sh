#!/usr/bin/env bash
# Inside Docker; keep outputs outside the source repository.
set -eo pipefail
source /opt/ros/noetic/setup.bash
LAB4_DIR=${LAB4_DIR:-/workspace/lab4}
LAB3_DIR=${LAB3_DIR:-/workspace/lab3}
LAB4_WS=${LAB4_WS:-/tmp/lab4_ws}
mkdir -p "$LAB4_WS/src"
for pkg in planner_pkg trajectory_generation_pkg; do
  ln -sfn "$LAB4_DIR/$pkg" "$LAB4_WS/src/$pkg"
done
for pkg in mav_trajectory_generation mav_msgs catkin_simple tesse_ros_bridge; do
  ln -sfn "$LAB4_DIR/dependencies/$pkg" "$LAB4_WS/src/$pkg"
done
ln -sfn "$LAB3_DIR/controller_pkg" "$LAB4_WS/src/controller_pkg"
cd "$LAB4_WS"
catkin build -j2 --no-status --cmake-args -DCMAKE_BUILD_TYPE=Release
