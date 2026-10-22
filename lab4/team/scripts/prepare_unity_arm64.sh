#!/usr/bin/env bash
# Run inside the Docker container after installing Dockerfile's dependencies.
set -eo pipefail
LAB4_DIR=${LAB4_DIR:-/workspace/lab4}
LAB4_RUNTIME=${LAB4_RUNTIME:-/tmp/lab4-runtime}
mkdir -p "$LAB4_RUNTIME"
if [ ! -x "$LAB4_RUNTIME/box64-0.3.6/build/box64" ]; then
  curl -fL --retry 3 https://codeload.github.com/ptitSeb/box64/tar.gz/refs/tags/v0.3.6 -o "$LAB4_RUNTIME/box64.tar.gz"
  tar -xzf "$LAB4_RUNTIME/box64.tar.gz" -C "$LAB4_RUNTIME"
  cmake -S "$LAB4_RUNTIME/box64-0.3.6" -B "$LAB4_RUNTIME/box64-0.3.6/build" -DARM_DYNAREC=ON -DCMAKE_BUILD_TYPE=Release
  cmake --build "$LAB4_RUNTIME/box64-0.3.6/build" -j1
fi
if [ ! -f "$LAB4_RUNTIME/lab4/lab4.x86_64" ]; then
  curl -fL --retry 3 https://vnav.mit.edu/material/lab4.zip -o "$LAB4_RUNTIME/lab4.zip"
  unzip -q "$LAB4_RUNTIME/lab4.zip" -d "$LAB4_RUNTIME"
fi
chmod +x "$LAB4_RUNTIME/lab4/lab4.x86_64"
python3 -m pip install --no-deps "$LAB4_DIR/dependencies/tesse_interface"
printf 'Runtime ready: %s\n' "$LAB4_RUNTIME"
