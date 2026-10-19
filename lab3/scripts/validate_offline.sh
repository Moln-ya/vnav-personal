#!/usr/bin/env bash
set -euo pipefail
lab3_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
validation_dir=$(mktemp -d /tmp/lab3-validation.XXXXXX)
${CXX:-g++} -std=c++11 -O2 -Wall -Wextra \
  -I/usr/include/eigen3 -I"$lab3_root/controller_pkg/include" \
  "$lab3_root/scripts/offline_flight.cpp" -o "$validation_dir/offline_flight"
cd "$validation_dir"
./offline_flight hover
./offline_flight circle
cp flight.csv "$lab3_root/report/offline_circle.csv"
./offline_flight circle 0.0005
./offline_flight circle 0.001 drag
./offline_flight circle 0.001 drag compensate
cp flight.csv "$lab3_root/report/offline_circle_drag_compensated.csv"
printf 'CSV files saved under %s/report; numerical plant is not TESSE.\n' "$lab3_root"
