# Sourced inside the Noetic filesystem by both command and interactive shells.
_lab4_load_noetic() {
  # Do not pass the launcher's command arguments to catkin's setup parser.
  source /opt/ros/noetic/setup.bash
  if [[ -f /tmp/lab4_ws/devel/setup.bash ]]; then
    source /tmp/lab4_ws/devel/setup.bash
  fi
}
_lab4_load_noetic
unset -f _lab4_load_noetic
