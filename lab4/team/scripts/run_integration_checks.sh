#!/usr/bin/env bash
set -eo pipefail
source /tmp/lab4_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11314 ROS_HOSTNAME=localhost
roscore -p 11314 > /tmp/lab4-integration-master.log 2>&1 & LAB4_MASTER_PID=$!
trap 'kill -INT "$LAB4_MASTER_PID" 2>/dev/null || true; wait "$LAB4_MASTER_PID" 2>/dev/null || true' EXIT
LAB4_MASTER_READY=false
for attempt in {1..30}; do
  if rosparam list >/dev/null 2>&1; then
    LAB4_MASTER_READY=true
    break
  fi
  if ! kill -0 "$LAB4_MASTER_PID" 2>/dev/null; then break; fi
  sleep 0.5
done
if [[ "$LAB4_MASTER_READY" != true ]]; then
  printf 'ROS master did not become ready; see /tmp/lab4-integration-master.log\n' >&2
  exit 1
fi
rosparam set /use_sim_time true
python3 /workspace/lab4/scripts/test_trajectory.py
