# Lab3 ARM64 Compatibility Environment

## System

- Host: Apple Silicon Mac
- Virtualisation: VMware Fusion
- Guest OS: Ubuntu 20.04.5 LTS
- Architecture: ARM64 / aarch64
- ROS: ROS Noetic
- ROS workspace: ~/vnav_ws

## Graphics

- Renderer: Mesa llvmpipe
- Hardware acceleration: No
- OpenGL core profile: 4.5
- OpenGL renderer: LLVMpipe

## Box64

- Version: v0.3.8
- Architecture: ARM64
- Dynamic recompiler: Enabled
- Source: Official Box64 GitHub release archive
- Source location: ~/lab3_tools/box64-v0.3.8-source

## Build Configuration

CMake options:

- ARM_DYNAREC=ON
- NOGIT=ON
- CMAKE_BUILD_TYPE=RelWithDebInfo

Build parallelism: 2 jobs

## Validation

- Source extraction: Successful
- CMake configuration: Successful
- Compilation: Successful
- ARM64 executable verification: Successful
- Box64 version command: Successful
- TESSE simulator: Not tested
- UAV trajectory following: Not tested

## Notes

Box64 was compiled locally on ARM64 Ubuntu.

The objective is to evaluate whether the original
x86_64 TESSE simulator can run without changing
the UAV control algorithms.

Successful Box64 compilation does not establish
TESSE simulator compatibility.
