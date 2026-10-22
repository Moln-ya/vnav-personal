#!/usr/bin/env bash
# Run ROS1 in its Focal filesystem without inheriting the host's ROS2 setup.
set -euo pipefail
LAB4_SOURCE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
NOETIC_ROOT=${NOETIC_ROOT:-/opt/vnav-noetic/bundle/rootfs}
if ! command -v proot >/dev/null || [[ ! -f "$NOETIC_ROOT/opt/ros/noetic/setup.bash" ]]; then
  printf 'Noetic is not prepared. Run: sudo bash %s/scripts/setup_noetic.sh\n' "$LAB4_SOURCE" >&2
  exit 1
fi
if [[ ! -d "$LAB4_SOURCE/../lab3/controller_pkg" ]]; then
  printf 'Missing Lab3 controller: %s/../lab3/controller_pkg\n' "$LAB4_SOURCE" >&2
  exit 1
fi
guest_env=(PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 HOME=/root)
for name in http_proxy https_proxy no_proxy HTTP_PROXY HTTPS_PROXY NO_PROXY DISPLAY TERM; do
  if [[ -v "$name" ]]; then guest_env+=("$name=${!name}"); fi
done
if [[ $# == 0 ]]; then
  set -- bash --noprofile --rcfile /workspace/lab4/scripts/noetic_env.bash -i
fi
exec proot -0 -r "$NOETIC_ROOT" \
  -b /dev -b /proc -b /sys -b /tmp -b /etc/resolv.conf -b /etc/hosts \
  -b "$LAB4_SOURCE:/workspace/lab4" \
  -b "$LAB4_SOURCE/../lab3:/workspace/lab3" -w /workspace/lab4 \
  /usr/bin/env -i "${guest_env[@]}" /bin/bash --noprofile --norc -c '
    source /workspace/lab4/scripts/noetic_env.bash
    exec "$@"
  ' noetic "$@"
