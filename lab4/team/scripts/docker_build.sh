#!/usr/bin/env bash
# Uses the user's existing container by default; copies only Lab4 sources.
set -euo pipefail
LAB4_CONTAINER=${LAB4_CONTAINER:-ros_noetic_lab3}
LAB4_SOURCE=$(cd "$(dirname "$0")/.." && pwd)
docker cp "$LAB4_SOURCE/." "$LAB4_CONTAINER:/workspace/lab4"
docker exec "$LAB4_CONTAINER" bash /workspace/lab4/scripts/build_workspace.sh
