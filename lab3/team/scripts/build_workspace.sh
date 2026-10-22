#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/noetic/setup.bash
LAB3_DIR=${LAB3_DIR:-/workspace/lab3}
LAB3_WS=${LAB3_WS:-/tmp/lab3_unity_ws}
mkdir -p "$LAB3_WS/src"
ln -sfn "$LAB3_DIR/controller_pkg" "$LAB3_WS/src/controller_pkg"
for pkg in catkin_simple mav_msgs tesse_ros_bridge; do
  ln -sfn "$LAB3_DIR/dependencies/$pkg" "$LAB3_WS/src/$pkg"
done
python3 -m pip install --no-deps "$LAB3_DIR/dependencies/tesse_interface"
cd "$LAB3_WS"
catkin build -j2 --no-status --cmake-args -DCMAKE_BUILD_TYPE=Release
