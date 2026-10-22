#!/usr/bin/env bash
# Foreground runtime session on x86_64 Ubuntu 20.04 / Noetic with WSL2 GPU.
# Unity renders through Mesa D3D12 on the NVIDIA GPU. Run run_unity_flight.sh
# in another terminal after "GPU rendering verified" is printed.
set -eo pipefail
LAB3_WS=${LAB3_WS:-/tmp/lab3_unity_ws}
LAB3_RUNTIME=${LAB3_RUNTIME:-/tmp/lab3-runtime}
LAB3_SESSION=${LAB3_SESSION:-/tmp/lab3-unity-gpu}
GPU_LIB=
for dir in /usr/lib/wsl/lib /opt/vnav-gpu/lib; do
  if [[ -f "$dir/libd3d12.so" ]]; then GPU_LIB=$dir; break; fi
done
[[ -n $GPU_LIB ]] || { printf 'No libd3d12.so found.\n' >&2; exit 1; }
if [[ ! -f "$LAB3_RUNTIME/lab3/lab3.x86_64" ]]; then
  mkdir -p "$LAB3_RUNTIME"
  if ! 7z t "$LAB3_RUNTIME/lab3.zip" >/dev/null 2>&1; then
    curl -fL --retry 3 https://vnav.mit.edu/material/lab3.zip -o "$LAB3_RUNTIME/lab3.zip"
  fi
  7z x -aos "-o$LAB3_RUNTIME" "$LAB3_RUNTIME/lab3.zip"
fi
chmod +x "$LAB3_RUNTIME/lab3/lab3.x86_64"
[[ ! -e /tmp/.X97-lock ]] || { printf 'Display :97 is already in use.\n' >&2; exit 1; }
source "$LAB3_WS/devel/setup.bash"
LAB3_BRIDGE_NODE=$(readlink -f "$LAB3_WS/src/tesse_ros_bridge")/src/tesse_ros_bridge/tesse_ros_node.py
mkdir -p "$LAB3_SESSION"
export DISPLAY=:97 ROS_MASTER_URI=http://localhost:11317 ROS_HOSTNAME=localhost
unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
pids=()
cleanup() {
  # Reverse start order: the bridge hangs if roscore exits before it.
  local i pid
  for ((i=${#pids[@]}-1; i>=0; i--)); do
    pid=${pids[i]}
    kill -INT "$pid" 2>/dev/null || continue
    timeout 10 tail --pid="$pid" -f /dev/null || kill -KILL "$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  # roslaunch runs the bridge node in its own session and it can outlive
  # roslaunch; match this workspace's copy so another lab's bridge is untouched.
  pkill -KILL -f "^python3 $LAB3_BRIDGE_NODE " 2>/dev/null || true
}
trap cleanup EXIT
Xvfb :97 -screen 0 960x600x24 > "$LAB3_SESSION/xvfb.log" 2>&1 & pids+=("$!")
roscore -p 11317 > "$LAB3_SESSION/master.log" 2>&1 & pids+=("$!")
ready=false
for attempt in {1..60}; do
  if LD_LIBRARY_PATH="$GPU_LIB" glxinfo -B > "$LAB3_SESSION/glxinfo.txt" 2>&1; then
    ready=true; break
  fi
  sleep 0.5
done
[[ $ready == true ]] || { printf 'OpenGL initialization failed; see %s/glxinfo.txt.\n' "$LAB3_SESSION" >&2; exit 1; }
grep -q 'OpenGL renderer string: D3D12 (NVIDIA' "$LAB3_SESSION/glxinfo.txt" || {
  cat "$LAB3_SESSION/glxinfo.txt"
  printf 'Refusing to run Unity without the NVIDIA hardware renderer.\n' >&2
  exit 1
}
cd "$LAB3_RUNTIME/lab3"
env -u PYTHONPATH LD_LIBRARY_PATH="$GPU_LIB" ./lab3.x86_64 \
  -screen-width 960 -screen-height 600 -screen-fullscreen 0 -force-glcore \
  -logFile "$LAB3_SESSION/unity.log" > "$LAB3_SESSION/player.log" 2>&1 & pids+=("$!")
LAB3_PLAYER_PID=$!
LAB3_WINDOW_ID=
for attempt in {1..120}; do
  LAB3_WINDOW_ID=$(xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1) || true
  if [[ -n $LAB3_WINDOW_ID ]]; then break; fi
  kill -0 "$LAB3_PLAYER_PID"
  sleep 0.5
done
[[ -n $LAB3_WINDOW_ID ]] || { printf 'Unity window did not appear.\n' >&2; exit 1; }
sleep 5
grep -q 'Renderer: D3D12 (NVIDIA' "$LAB3_SESSION/unity.log" || {
  printf 'Unity did not confirm NVIDIA hardware rendering.\n' >&2; exit 1;
}
roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch publish_imu:=false \
  publish_collisions:=true > "$LAB3_SESSION/bridge.log" 2>&1 & pids+=("$!")
timeout 60 rostopic echo -n 1 /current_state > "$LAB3_SESSION/initial_state.txt"
# Close Unity's startup audio-error console on the headless X server.
xdotool windowfocus "$LAB3_WINDOW_ID" mousemove 512 570 sleep 0.5 \
  mousedown 1 sleep 1 mouseup 1 sleep 0.5 mousemove 900 30
printf 'Unity GPU rendering verified: NVIDIA via D3D12. Logs: %s\n' "$LAB3_SESSION"
wait "${pids[-1]}"
