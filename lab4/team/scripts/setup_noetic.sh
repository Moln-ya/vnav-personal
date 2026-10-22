#!/usr/bin/env bash
# Ubuntu 22.04 host: install an isolated official Ubuntu 20.04/ROS1 Noetic.
set -euo pipefail
LAB4_SOURCE=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
NOETIC_DIR=${NOETIC_DIR:-/opt/vnav-noetic}
export NOETIC_ROOT="$NOETIC_DIR/bundle/rootfs"
if [[ $EUID != 0 ]]; then
  printf 'Run this installer with sudo (it installs tools and writes %s).\n' "$NOETIC_DIR" >&2
  exit 1
fi
if [[ $(uname -m) != x86_64 ]]; then
  printf 'This installer currently targets x86_64 hosts.\n' >&2
  exit 1
fi
if ! command -v proot >/dev/null || ! command -v skopeo >/dev/null || ! command -v umoci >/dev/null; then
  apt-get update -o Dir::Etc::sourceparts=-
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    -o Dir::Etc::sourceparts=- proot skopeo umoci
fi
mkdir -p "$NOETIC_DIR"
if [[ ! -f "$NOETIC_ROOT/opt/ros/noetic/setup.bash" ]]; then
  skopeo copy --override-arch amd64 \
    docker://docker.io/library/ros@sha256:72b8bc59035dc0a5b8e07aae28c16caa84192971d72d207c72ed734fb1d5e97d \
    "oci:$NOETIC_DIR/image:noetic"
  umoci unpack --rootless --image "$NOETIC_DIR/image:noetic" "$NOETIC_DIR/bundle"
fi
mkdir -p "$NOETIC_ROOT/workspace/lab4" "$NOETIC_ROOT/workspace/lab3"
bash "$LAB4_SOURCE/scripts/noetic.sh" bash -c '
  set -eo pipefail
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config python3-catkin-tools python3-numpy \
    python3-matplotlib python3-scipy python3-defusedxml python3-pil python3-pip \
    libeigen3-dev libgoogle-glog-dev libyaml-cpp-dev libnlopt-cxx-dev \
    ros-noetic-tf ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs \
    ros-noetic-rosgraph-msgs ros-noetic-visualization-msgs ros-noetic-cmake-modules \
    ros-noetic-cv-bridge ros-noetic-ackermann-msgs ros-noetic-rviz ros-noetic-rviz-plugin-tutorials \
    xvfb xdotool curl unzip ffmpeg libgl1-mesa-dri libxcursor1 libxrandr2 \
    libxi6 libxinerama1 libasound2 libpulse0 libudev1 libgtk2.0-0 libsdl2-2.0-0 libnss3
  python3 -m pip install --no-deps /workspace/lab4/dependencies/tesse_interface
  bash /workspace/lab4/scripts/build_workspace.sh
'
printf '\nNoetic ready. Enter it with: bash %s/scripts/noetic.sh\n' "$LAB4_SOURCE"
