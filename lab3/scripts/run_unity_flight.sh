#!/usr/bin/env bash
# Uses the live official simulator started by start_unity_gpu.sh (or
# start_unity_arm64.sh). Each run writes a new directory under report/runs/:
# the Unity window video, the RViz video, telemetry CSV/JSON, bag and plots.
set -eo pipefail
LAB3_DIR=${LAB3_DIR:-/workspace/lab3}
LAB3_WS=${LAB3_WS:-/tmp/lab3_unity_ws}
source "$LAB3_WS/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11317 ROS_HOSTNAME=localhost
export DISABLE_ROS1_EOL_WARNINGS=1 LIBGL_ALWAYS_SOFTWARE=1
OUT=${LAB3_OUTPUT_DIR:-$LAB3_DIR/report/runs/unity-$(date -u +%Y%m%dT%H%M%SZ)}
GAINS=${LAB3_GAINS_FILE:-$LAB3_DIR/controller_pkg/config/gains_gpu.yaml}
DURATION=${LAB3_DURATION:-55}
FPS=${LAB3_RECORD_FPS:-30}
SOURCE=${LAB3_SOURCE_LABEL:-Official MIT Lab3 TESSE / GPU}
mkdir -p "$OUT"
for artifact in unity_flight.csv unity_flight.bag unity_sim.mp4 rviz.mp4; do
  [[ ! -e "$OUT/$artifact" ]] || { printf 'Refusing to overwrite %s/%s\n' "$OUT" "$artifact" >&2; exit 1; }
done
cp "$GAINS" "$OUT/gains_used.yaml"
pids=()
videos=()
cleanup() {
  # Finish the MP4s while their X displays are still alive.
  for pid in "${videos[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${videos[@]}"; do wait "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
# Only these two Lab3 nodes are restarted. The runtime and other masters stay up.
rosnode kill /controller_node /traj_publisher 2>/dev/null || true
if [[ ${LAB3_RESPAWN:-1} == 1 ]]; then
  # Start every run from Unity's spawn pose, hovering on the ground.
  LAB3_WINDOW_ID=$(DISPLAY=:97 xdotool search --onlyvisible --name tesse_vnav_scenes | head -n1)
  DISPLAY=:97 xdotool windowfocus "$LAB3_WINDOW_ID" key r
  sleep 2
fi
python3 "$LAB3_DIR/scripts/monitor_unity.py" --output "$OUT/unity_flight.csv" \
  --duration "$DURATION" --wall-timeout 600 --source "$SOURCE" \
  --trail-seconds "${LAB3_TRAIL_SECONDS:-12.6}" \
  > "$OUT/monitor.log" 2>&1 & monitor_pid=$!; pids+=("$monitor_pid")
rosbag record -O "$OUT/unity_flight.bag" \
  /clock /current_state /desired_state /rotor_speed_cmds /flight/actual \
  /flight/desired /flight/source /tesse/collisions \
  > "$OUT/bag.log" 2>&1 & pids+=("$!")
if [[ ${LAB3_RVIZ:-1} == 1 ]]; then
  Xvfb :96 -screen 0 1280x800x24 > "$OUT/rviz_xvfb.log" 2>&1 & pids+=("$!")
  sleep 1
  DISPLAY=:96 rosrun rviz rviz -d "$LAB3_DIR/controller_pkg/rviz/circle.rviz" \
    __name:=lab3_official_rviz > "$OUT/rviz.log" 2>&1 & pids+=("$!")
  sleep 5
  ffmpeg -n -loglevel error -f x11grab -draw_mouse 0 -video_size 1280x800 -framerate "$FPS" -i :96 \
    -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -movflags +faststart \
    "$OUT/rviz.mp4" > "$OUT/rviz_ffmpeg.log" 2>&1 & videos+=("$!")
fi
ffmpeg -n -loglevel error -f x11grab -draw_mouse 0 -video_size 960x600 -framerate "$FPS" -i :97 \
  -c:v libx264 -preset ultrafast -crf 23 -pix_fmt yuv420p -movflags +faststart \
  "$OUT/unity_sim.mp4" > "$OUT/unity_ffmpeg.log" 2>&1 & videos+=("$!")
roslaunch controller_pkg traj_tracking.launch gains_file:="$GAINS" \
  > "$OUT/controller.log" 2>&1 & pids+=("$!")
printf 'Recording Lab3 Unity run in %s\n' "$OUT"
wait "$monitor_pid"
# Close MP4s and rosbag before analysis.
cleanup
pids=()
videos=()
python3 "$LAB3_DIR/scripts/analyze_unity.py" "$OUT"
