#!/usr/bin/env bash
# Restore missing WSL OpenGL/D3D12 user libraries, without installing a kernel driver.
set -euo pipefail
GPU_DIR=${LAB4_GPU_DIR:-/opt/vnav-gpu}
GPU_CACHE=${LAB4_GPU_CACHE:-/tmp/lab4-gpu}
if [[ $EUID != 0 ]]; then printf 'Run this installer with sudo.\n' >&2; exit 1; fi
[[ -c /dev/dxg ]] || { printf 'WSL GPU device /dev/dxg is not exposed.\n' >&2; exit 1; }
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
  -o Dir::Etc::sourceparts=- mesa-utils libegl1 libegl-mesa0 p7zip-full
mkdir -p "$GPU_DIR/lib" "$GPU_CACHE"
if [[ ! -f "$GPU_DIR/lib/libd3d12.so" || ! -f "$GPU_DIR/lib/libd3d12core.so" ]]; then
  curl -fL --retry 3 \
    'https://pkgs.dev.azure.com/shine-oss/13eb32df-d33f-470f-b930-499535a958b4/_packaging/7925a3a1-b93c-4977-8a97-5b877bf2068b/nuget/v3/flat2/microsoft.direct3d.linux/1.611.1-81528511/microsoft.direct3d.linux.1.611.1-81528511.nupkg' \
    -o "$GPU_CACHE/microsoft.direct3d.linux.nupkg"
  printf '%s  %s\n' 2a0e23e4b1755f1a9c639a5115063db123ad172120d36520ff2aa962c50973f4 \
    "$GPU_CACHE/microsoft.direct3d.linux.nupkg" | sha256sum -c -
  7z e -y "-o$GPU_DIR/lib" "$GPU_CACHE/microsoft.direct3d.linux.nupkg" \
    build/native/lib/x64/libd3d12.so build/native/lib/x64/libd3d12core.so
fi
shopt -s nullglob
driver_dirs=(/usr/lib/wsl/drivers/nvmdi.inf_amd64_*)
if [[ ${#driver_dirs[@]} != 1 ]]; then
  printf 'Expected one WSL NVIDIA driver directory; found %s.\n' "${#driver_dirs[@]}" >&2
  exit 1
fi
driver_dir=${driver_dirs[0]}
if [[ ! -f "$driver_dir/libnvwgf2umx.so" ]]; then
  driver_version=$(nvidia-smi --query-gpu=driver_version --format=csv,noheader | head -n1)
  [[ "$driver_version" == 591.74 ]] || {
    printf 'This recovery package is for driver 591.74; active driver is %s.\n' "$driver_version" >&2
    exit 1
  }
  if [[ ! -f "$GPU_CACHE/nvidia-package-list.txt" ]]; then
    curl -fL --retry 3 \
      https://us.download.nvidia.com/Windows/591.74/591.74-desktop-win10-win11-64bit-international-dch-whql.exe \
      -o "$GPU_CACHE/nvidia-591.74.exe"
    7z l "$GPU_CACHE/nvidia-591.74.exe" > "$GPU_CACHE/nvidia-package-list.txt"
  fi
  mkdir -p "$GPU_DIR/nvidia-extracted"
  7z e -y -r "-o$GPU_DIR/nvidia-extracted" "$GPU_CACHE/nvidia-591.74.exe" libnvwgf2umx.so
  test -s "$GPU_DIR/nvidia-extracted/libnvwgf2umx.so"
  printf '%s  %s\n' bef6c824f550ff561f3866e1b053b1f86dba893a3cce072a3a0c4a5dbec3983f \
    "$GPU_DIR/nvidia-extracted/libnvwgf2umx.so" | sha256sum -c -
  install -m 755 "$GPU_DIR/nvidia-extracted/libnvwgf2umx.so" "$driver_dir/libnvwgf2umx.so"
fi
sha256sum "$GPU_DIR/lib/libd3d12.so" "$GPU_DIR/lib/libd3d12core.so" \
  "$driver_dir/libnvwgf2umx.so" > "$GPU_DIR/library-sha256.txt"
printf 'WSL GPU libraries ready. Verify glxinfo reports D3D12 / NVIDIA before running Unity.\n'
