#!/usr/bin/env bash
# Unity renders through hardware D3D12. ROS1 runs natively on an Ubuntu 20.04
# system with Noetic installed, otherwise in the PRoot Focal environment.
set -eo pipefail
LAB4_SOURCE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
LAB4_RUNTIME=${LAB4_RUNTIME:-/tmp/lab4-runtime}
LAB4_SESSION=${LAB4_SESSION:-/tmp/lab4-unity-gpu}
# Recent WSL exposes the D3D12 libraries itself; older images need setup_unity_gpu.sh.
GPU_LIB=
for dir in ${LAB4_GPU_DIR:+"$LAB4_GPU_DIR/lib"} /usr/lib/wsl/lib /opt/vnav-gpu/lib; do
  if [[ -f "$dir/libd3d12.so" ]]; then GPU_LIB=$dir; break; fi
done
[[ -n $GPU_LIB ]] || { printf 'No libd3d12.so found; run scripts/setup_unity_gpu.sh first.\n' >&2; exit 1; }
if [[ -f /opt/ros/noetic/setup.bash ]] && grep -q '^VERSION_CODENAME=focal' /etc/os-release; then
  ros() {
    bash -c 'source /opt/ros/noetic/setup.bash
      if [[ -f /tmp/lab4_ws/devel/setup.bash ]]; then source /tmp/lab4_ws/devel/setup.bash; fi
      exec "$@"' ros "$@"
  }
else
  ros() { bash "$LAB4_SOURCE/scripts/noetic.sh" "$@"; }
fi
if [[ ! -f "$LAB4_RUNTIME/lab4/lab4.x86_64" ]]; then
  mkdir -p "$LAB4_RUNTIME"
  if ! 7z t "$LAB4_RUNTIME/lab4.zip" >/dev/null 2>&1; then
    curl -fL --retry 3 https://vnav.mit.edu/material/lab4.zip -o "$LAB4_RUNTIME/lab4.zip"
  fi
  7z x -aos "-o$LAB4_RUNTIME" "$LAB4_RUNTIME/lab4.zip"
fi
chmod +x "$LAB4_RUNTIME/lab4/lab4.x86_64"
[[ ! -e /tmp/.X98-lock ]] || { printf 'Display :98 is already in use.\n' >&2; exit 1; }
mkdir -p "$LAB4_SESSION"
LAB4_PID_DIR=$(mktemp -d "$LAB4_SESSION/pids.XXXXXX")
export DISPLAY=:98 ROS_MASTER_URI=http://localhost:11316 ROS_HOSTNAME=localhost
unset LIBGL_ALWAYS_SOFTWARE LIBGL_ALWAYS_INDIRECT
export GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export LD_LIBRARY_PATH="$GPU_LIB"
pids=()
pidfiles=()
cleanup() {
  for path in "${pidfiles[@]}"; do
    if [[ -f "$path" ]]; then
      read -r pid < "$path"
      if [[ "$pid" =~ ^[0-9]+$ ]]; then kill -TERM "$pid" 2>/dev/null || true; fi
    fi
  done
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
guest_service() {
  local name=$1; shift
  local pidfile="$LAB4_PID_DIR/$name.pid"
  pidfiles+=("$pidfile")
  ros env DISPLAY=:98 \
    ROS_MASTER_URI=http://localhost:11316 ROS_HOSTNAME=localhost \
    bash -c 'printf "%s\n" "$$" > "$1"; shift; exec "$@"' service "$pidfile" "$@" \
    > "$LAB4_SESSION/$name.log" 2>&1 &
  pids+=("$!")
}
guest_service xvfb Xvfb :98 -screen 0 960x600x24
guest_service master roscore -p 11316
ready=false
for attempt in {1..60}; do
  if glxinfo -B > "$LAB4_SESSION/glxinfo.txt" 2>&1; then ready=true; break; fi
  sleep 0.5
done
[[ "$ready" == true ]] || { printf 'Hardware OpenGL initialization failed; see %s/glxinfo.txt.\n' "$LAB4_SESSION" >&2; exit 1; }
if ! grep -q 'OpenGL renderer string: D3D12 (NVIDIA' "$LAB4_SESSION/glxinfo.txt"; then
  cat "$LAB4_SESSION/glxinfo.txt"
  printf 'Refusing to run Unity without the NVIDIA hardware renderer.\n' >&2
  exit 1
fi
cd "$LAB4_RUNTIME/lab4"
env -u PYTHONPATH ./lab4.x86_64 -screen-width 960 -screen-height 600 \
  -screen-fullscreen 0 -force-glcore -logFile "$LAB4_SESSION/unity.log" \
  > "$LAB4_SESSION/player.log" 2>&1 & pids+=("$!")
LAB4_PLAYER_PID=$!
LAB4_WINDOW_ID=
for attempt in {1..120}; do
  LAB4_WINDOW_ID=$(ros env DISPLAY=:98 \
    xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1) || true
  if [[ -n "$LAB4_WINDOW_ID" ]]; then break; fi
  kill -0 "$LAB4_PLAYER_PID"
  sleep 0.5
done
[[ -n "$LAB4_WINDOW_ID" ]] || { printf 'Unity window did not appear.\n' >&2; exit 1; }
sleep 5
grep -q 'Renderer: D3D12 (NVIDIA' "$LAB4_SESSION/unity.log" || {
  printf 'Unity did not confirm NVIDIA hardware rendering.\n' >&2; exit 1;
}
guest_service bridge roslaunch tesse_ros_bridge tesse_quadrotor_bridge.launch \
  publish_imu:=false publish_collisions:=true
ros env ROS_MASTER_URI=http://localhost:11316 \
  ROS_HOSTNAME=localhost timeout 60 rostopic echo -n 1 /current_state \
  > "$LAB4_SESSION/initial_state.txt"
ros env DISPLAY=:98 xdotool \
  windowfocus "$LAB4_WINDOW_ID" mousemove 512 570 sleep 0.5 \
  mousedown 1 sleep 1 mouseup 1 sleep 0.5 mousemove 900 30
printf 'Unity GPU rendering verified: NVIDIA RTX 4060 via D3D12. Logs: %s\n' "$LAB4_SESSION"
wait "${pids[-1]}"
