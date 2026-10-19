#!/usr/bin/env bash
# Uses the live official simulator started by start_unity_arm64.sh.
set -eo pipefail
LAB3_DIR=${LAB3_DIR:-/workspace/lab3}
LAB3_WS=${LAB3_WS:-/tmp/lab3_unity_ws}
source "$LAB3_WS/devel/setup.bash"
export ROS_MASTER_URI=http://localhost:11317 ROS_HOSTNAME=localhost
export DISABLE_ROS1_EOL_WARNINGS=1 LIBGL_ALWAYS_SOFTWARE=1
mkdir -p "$LAB3_DIR/report/video"
pids=()
cleanup() {
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT
# Only these two Lab3 nodes are restarted. The runtime and other masters stay up.
rosnode kill /controller_node /traj_publisher 2>/dev/null || true
python3 "$LAB3_DIR/scripts/monitor_unity.py" \
  --output "$LAB3_DIR/report/unity_flight.csv" --duration 55 --wall-timeout 600 \
  > /tmp/lab3-final-monitor.log 2>&1 & monitor_pid=$!; pids+=("$monitor_pid")
rosbag record -O "$LAB3_DIR/report/unity_flight.bag" \
  /clock /current_state /desired_state /rotor_speed_cmds /flight/actual \
  /flight/desired /flight/source /tesse/collisions \
  >/tmp/lab3-final-bag.log 2>&1 & pids+=("$!")
if [ "${LAB3_REUSE_RVIZ:-0}" != 1 ]; then
  Xvfb :96 -screen 0 1280x800x24 >/tmp/lab3-official-rviz-xvfb.log 2>&1 & pids+=("$!")
  sleep 1
  DISPLAY=:96 rosrun rviz rviz -d "$LAB3_DIR/controller_pkg/rviz/circle.rviz" \
    __name:=lab3_official_rviz >/tmp/lab3-official-rviz.log 2>&1 & pids+=("$!")
  sleep 5
fi
ffmpeg -y -f x11grab -video_size 1280x800 -framerate 8 -i :96 \
  -c:v libx264 -threads 1 -preset ultrafast -crf 23 -pix_fmt yuv420p \
  -movflags +faststart "$LAB3_DIR/report/video/rviz_official_tesse_full.mp4" \
  >/tmp/lab3-final-ffmpeg.log 2>&1 & pids+=("$!")
roslaunch controller_pkg traj_tracking.launch \
  >/tmp/lab3-final-controller.log 2>&1 & pids+=("$!")
wait "$monitor_pid"
# Close MP4 and rosbag before extracting the steady flight segment.
cleanup
pids=()
ffmpeg -y -loglevel error -ss 30 \
  -i "$LAB3_DIR/report/video/rviz_official_tesse_full.mp4" \
  -c:v libx264 -threads 1 -preset fast -crf 20 -pix_fmt yuv420p \
  -movflags +faststart "$LAB3_DIR/report/video/rviz_official_tesse.mp4"
python3 "$LAB3_DIR/scripts/analyze_unity.py"
