#!/usr/bin/env bash
# Requires Unity on :98 and the bridge on ROS master 11316.
set -eo pipefail
source /tmp/lab4_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11316 ROS_HOSTNAME=localhost
LAB4_DIR=/workspace/lab4
export LAB4_OUTPUT_DIR=${LAB4_OUTPUT_DIR:-$LAB4_DIR/report/runs/unity-$(date -u +%Y%m%dT%H%M%SZ)}
mkdir -p "$LAB4_OUTPUT_DIR"
for artifact in unity_race.bag unity_race.bag.active unity_race.mp4 rviz_race.mp4 unity_flight.csv; do
  if [[ -e "$LAB4_OUTPUT_DIR/$artifact" ]]; then
    printf 'Refusing to overwrite %s/%s\n' "$LAB4_OUTPUT_DIR" "$artifact" >&2
    exit 1
  fi
done
if [[ -n ${LAB4_GPU_SESSION:-} ]]; then
  cp "$LAB4_GPU_SESSION/glxinfo.txt" "$LAB4_OUTPUT_DIR/gpu_glxinfo.txt"
  cp "$LAB4_GPU_SESSION/unity.log" "$LAB4_OUTPUT_DIR/unity_startup.log"
fi
# Unity turns a gate green when the drone flies through it and never turns it
# back: respawning keeps the colours, and reloading the scene spawns a second
# drone. Only a freshly started player shows red gates until each is passed.
LAB4_UNITY_PID=$(pgrep -f 'lab4\.x86_64' | head -n1)
[[ -n $LAB4_UNITY_PID ]] || { printf 'Unity player is not running.\n' >&2; exit 1; }
LAB4_GATES_USED=/tmp/lab4-gates-used.$LAB4_UNITY_PID
if [[ -e $LAB4_GATES_USED && ${LAB4_ALLOW_GREEN_GATES:-0} != 1 ]]; then
  printf 'This Unity session already flew the course, so its gates are green.\n' >&2
  printf 'Restart Unity for red gates, or set LAB4_ALLOW_GREEN_GATES=1 for a trial.\n' >&2
  exit 1
fi
rosnode kill /controller_node /simple_traj_planner /traj_vertices_publisher 2>/dev/null || true
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
rosbag record -O "$LAB4_OUTPUT_DIR/unity_race.bag" /current_state /desired_state /desired_traj_vertices /clock /tesse/collision > "$LAB4_OUTPUT_DIR/bag.log" 2>&1 & pids+=("$!")
roslaunch trajectory_generation traj_following.launch \
  gains_file:="${LAB4_GAINS_FILE:-$LAB4_DIR/trajectory_generation_pkg/config/gains_gpu.yaml}" \
  v_max:="${LAB4_V_MAX:-8.0}" a_max:="${LAB4_A_MAX:-6.0}" a_z_max:="${LAB4_A_Z_MAX:-5.0}" \
  corridor:="${LAB4_CORRIDOR:-0.5}" > "$LAB4_OUTPUT_DIR/follower.log" 2>&1 & pids+=("$!")
ready=false
for attempt in {1..40}; do
  if rosnode ping -c 1 /controller_node >/dev/null 2>&1; then ready=true; break; fi
  sleep 0.5
done
[[ $ready == true ]] || { printf 'Controller did not start.\n' >&2; exit 1; }
LAB4_WINDOW_ID=$(DISPLAY=:98 xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1)
DISPLAY=:98 xdotool windowfocus "$LAB4_WINDOW_ID" key r
sleep 1
# Dismiss the development console if the headless machine has no audio device.
DISPLAY=:98 xdotool windowfocus "$LAB4_WINDOW_ID" mousemove 512 570 sleep 0.5 \
  mousedown 1 sleep 1 mouseup 1 sleep 0.5 mousemove 900 30
python3 -u "$LAB4_DIR/scripts/monitor_unity.py" > "$LAB4_OUTPUT_DIR/monitor.log" 2>&1 & pids+=("$!")
LAB4_MONITOR_PID=$!
ready=false
for attempt in {1..40}; do
  if rosnode ping -c 1 /unity_race_monitor >/dev/null 2>&1; then ready=true; break; fi
  sleep 0.5
done
[[ $ready == true ]] || { printf 'Telemetry monitor did not start.\n' >&2; exit 1; }
# RViz on its own display, recorded alongside the Unity window (set LAB4_RVIZ=0 to skip).
if [[ ${LAB4_RVIZ:-1} == 1 ]]; then
  Xvfb :96 -screen 0 1280x800x24 > "$LAB4_OUTPUT_DIR/rviz_xvfb.log" 2>&1 & pids+=("$!")
  sleep 1
  DISPLAY=:96 DISABLE_ROS1_EOL_WARNINGS=1 rosrun rviz rviz -d "$LAB4_DIR/rviz/race_gpu.rviz" \
    __name:=lab4_race_rviz > "$LAB4_OUTPUT_DIR/rviz.log" 2>&1 & pids+=("$!")
  sleep 5
  ffmpeg -n -loglevel error -f x11grab -draw_mouse 0 -framerate "${LAB4_RECORD_FPS:-30}" -video_size 1280x800 -i :96 \
    -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -movflags +faststart "$LAB4_OUTPUT_DIR/rviz_race.mp4" > "$LAB4_OUTPUT_DIR/rviz_video.log" 2>&1 & pids+=("$!")
fi
ffmpeg -n -loglevel error -f x11grab -framerate "${LAB4_RECORD_FPS:-30}" -video_size 960x600 -i :98 \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -movflags +faststart "$LAB4_OUTPUT_DIR/unity_race.mp4" > "$LAB4_OUTPUT_DIR/video.log" 2>&1 & pids+=("$!")
sleep 1
touch "$LAB4_GATES_USED"
roslaunch planner_pkg traj_gen.launch simulator_data_directory:="$LAB4_DIR/simulator_data" > "$LAB4_OUTPUT_DIR/gates.log" 2>&1 & pids+=("$!")
timeout 45 rostopic echo -n 1 /desired_state > "$LAB4_OUTPUT_DIR/initial_desired_state.txt"
printf 'Recording new Unity run in %s\n' "$LAB4_OUTPUT_DIR"
wait "$LAB4_MONITOR_PID"
