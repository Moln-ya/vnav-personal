#!/usr/bin/env bash
# Inside the ARM64 Noetic container. The official Unity files stay unmodified.
set -eo pipefail
LAB3_RUNTIME=${LAB3_RUNTIME:-/tmp/lab3-runtime}
mkdir -p "$LAB3_RUNTIME"
if [ ! -x "$LAB3_RUNTIME/box64-0.3.6/build/box64" ]; then
  curl -fL --retry 3 https://codeload.github.com/ptitSeb/box64/tar.gz/refs/tags/v0.3.6 -o "$LAB3_RUNTIME/box64.tar.gz"
  tar -xzf "$LAB3_RUNTIME/box64.tar.gz" -C "$LAB3_RUNTIME"
  cmake -S "$LAB3_RUNTIME/box64-0.3.6" -B "$LAB3_RUNTIME/box64-0.3.6/build" -DARM_DYNAREC=ON -DCMAKE_BUILD_TYPE=Release
  cmake --build "$LAB3_RUNTIME/box64-0.3.6/build" -j1
fi
if [ ! -f "$LAB3_RUNTIME/lab3/lab3.x86_64" ]; then
  curl -fL --retry 3 https://vnav.mit.edu/material/lab3.zip -o "$LAB3_RUNTIME/lab3.zip"
  unzip -q "$LAB3_RUNTIME/lab3.zip" -d "$LAB3_RUNTIME"
fi
chmod +x "$LAB3_RUNTIME/lab3/lab3.x86_64"
printf 'Runtime ready: %s\n' "$LAB3_RUNTIME"
