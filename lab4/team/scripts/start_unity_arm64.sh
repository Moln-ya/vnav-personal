#!/usr/bin/env bash
# Foreground session; keep it running while invoking run_unity_race.sh elsewhere.
set -eo pipefail
source /tmp/lab4_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11316 ROS_HOSTNAME=localhost
LAB4_RUNTIME=${LAB4_RUNTIME:-/tmp/lab4-runtime}
export DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1 BOX64_LOG=0
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
Xvfb :98 -screen 0 960x600x24 > /tmp/lab4-xvfb.log 2>&1 & pids+=("$!")
roscore -p 11316 > /tmp/lab4-unity-master.log 2>&1 & pids+=("$!")
sleep 2
cd "$LAB4_RUNTIME/lab4"
"$LAB4_RUNTIME/box64-0.3.6/build/box64" ./lab4.x86_64 -screen-width 960 \
  -screen-height 600 -screen-fullscreen 0 -force-glcore \
  -logFile /tmp/lab4-unity.log > /tmp/lab4-box64.log 2>&1 & pids+=("$!")
sleep 15
LAB4_WINDOW_ID=$(xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1)
xdotool windowfocus "$LAB4_WINDOW_ID" mousemove 512 571 click 1
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_imu:=false \
  publish_collisions:=true > /tmp/lab4-unity-bridge.log 2>&1 & pids+=("$!")
wait "${pids[-1]}"
