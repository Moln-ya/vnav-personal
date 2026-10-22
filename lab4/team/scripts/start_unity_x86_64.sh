#!/usr/bin/env bash
# Legacy software-rendering reproduction. Use start_unity_gpu.sh for RTX rendering.
# Run inside scripts/noetic.sh; leave this running while recording the race.
set -eo pipefail
source /tmp/lab4_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11316 ROS_HOSTNAME=localhost
export DISPLAY=:98 LIBGL_ALWAYS_SOFTWARE=1
LAB4_RUNTIME=${LAB4_RUNTIME:-/tmp/lab4-runtime}
LAB4_SESSION=${LAB4_SESSION:-/tmp/lab4-unity-native}
[[ $(uname -m) == x86_64 ]] || { printf 'Requires x86_64.\n' >&2; exit 1; }
mkdir -p "$LAB4_RUNTIME" "$LAB4_SESSION"
if [[ ! -f "$LAB4_RUNTIME/lab4/lab4.x86_64" ]]; then
  if ! unzip -tq "$LAB4_RUNTIME/lab4.zip" >/dev/null 2>&1; then
    curl -fL --retry 3 https://vnav.mit.edu/material/lab4.zip -o "$LAB4_RUNTIME/lab4.zip"
  fi
  unzip -nq "$LAB4_RUNTIME/lab4.zip" -d "$LAB4_RUNTIME"
fi
chmod +x "$LAB4_RUNTIME/lab4/lab4.x86_64"
if [[ -e /tmp/.X98-lock ]]; then
  printf 'Display :98 is already in use.\n' >&2
  exit 1
fi
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
Xvfb :98 -screen 0 960x600x24 > "$LAB4_SESSION/xvfb.log" 2>&1 & pids+=("$!")
roscore -p 11316 > "$LAB4_SESSION/master.log" 2>&1 & pids+=("$!")
ready=false
for attempt in {1..60}; do
  if rosparam list >/dev/null 2>&1 && xdotool getdisplaygeometry >/dev/null 2>&1; then
    ready=true; break
  fi
  sleep 0.5
done
[[ $ready == true ]] || { printf 'ROS master/display startup failed.\n' >&2; exit 1; }
cd "$LAB4_RUNTIME/lab4"
./lab4.x86_64 -screen-width 960 -screen-height 600 -screen-fullscreen 0 \
  -force-glcore -logFile "$LAB4_SESSION/unity.log" > "$LAB4_SESSION/player.log" 2>&1 &
pids+=("$!")
LAB4_UNITY_PID=$!
LAB4_WINDOW_ID=
for attempt in {1..120}; do
  LAB4_WINDOW_ID=$(xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1) || true
  if [[ -n "$LAB4_WINDOW_ID" ]]; then break; fi
  kill -0 "$LAB4_UNITY_PID"
  sleep 0.5
done
[[ -n "$LAB4_WINDOW_ID" ]] || { printf 'Unity window did not appear.\n' >&2; exit 1; }
sleep 10
xdotool windowfocus "$LAB4_WINDOW_ID" mousemove 512 571 click 1
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_imu:=false \
  publish_collisions:=true > "$LAB4_SESSION/bridge.log" 2>&1 & pids+=("$!")
timeout 60 rostopic echo -n 1 /current_state > "$LAB4_SESSION/initial_state.txt"
printf 'Official Unity x86_64 and ROS bridge ready; logs: %s\n' "$LAB4_SESSION"
wait "${pids[-1]}"
