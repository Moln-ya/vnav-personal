#!/usr/bin/env bash
# Foreground runtime session. Run run_unity_flight.sh in another terminal.
set -eo pipefail
LAB3_WS=${LAB3_WS:-/tmp/lab3_unity_ws}
LAB3_RUNTIME=${LAB3_RUNTIME:-/tmp/lab3-runtime}
source "$LAB3_WS/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11317 ROS_HOSTNAME=localhost
export DISPLAY=:97 LIBGL_ALWAYS_SOFTWARE=1 BOX64_LOG=0
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
Xvfb :97 -screen 0 960x600x24 >/tmp/lab3-unity-xvfb.log 2>&1 & pids+=("$!")
roscore -p 11317 >/tmp/lab3-unity-master.log 2>&1 & pids+=("$!")
sleep 2
cd "$LAB3_RUNTIME/lab3"
"$LAB3_RUNTIME/box64-0.3.6/build/box64" ./lab3.x86_64 \
  -screen-width 960 -screen-height 600 -screen-fullscreen 0 -force-glcore \
  -logFile /tmp/lab3-unity-box64.log >/tmp/lab3-box64.log 2>&1 & pids+=("$!")
sleep 15
# Close Unity's startup audio-error console on the headless X server.
LAB3_WINDOW_ID=$(xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1)
xdotool windowfocus "$LAB3_WINDOW_ID" mousemove 512 571 click 1
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_imu:=false \
  publish_collisions:=true >/tmp/lab3-unity-bridge.log 2>&1 & pids+=("$!")
wait "${pids[-1]}"
