FROM ros:noetic-ros-base-focal
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-catkin-tools python3-numpy python3-matplotlib \
    python3-scipy python3-defusedxml python3-pil python3-pip libeigen3-dev \
    ros-noetic-tf ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs \
    ros-noetic-rosgraph-msgs ros-noetic-visualization-msgs ros-noetic-cmake-modules \
    ros-noetic-cv-bridge ros-noetic-ackermann-msgs ros-noetic-rviz \
    xvfb xdotool curl unzip ffmpeg libgl1-mesa-dri libxcursor1 libxrandr2 \
    libxi6 libxinerama1 libasound2 libpulse0 libudev1 libgtk2.0-0 \
    libsdl2-2.0-0 libnss3 texlive-latex-base texlive-latex-recommended \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace/lab3
CMD ["bash"]
