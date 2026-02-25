#!/bin/bash
# Docker entrypoint: source ROS2 Humble + workspace overlay
set -e

# Source ROS2 Humble base
source /opt/ros/humble/setup.bash

# Source the workspace if it has been built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# GUI / rendering defaults
export LIBGL_ALWAYS_SOFTWARE=1
export XDG_RUNTIME_DIR=/tmp/runtime-root
mkdir -p "$XDG_RUNTIME_DIR"
chmod 0700 "$XDG_RUNTIME_DIR"

# Ensure DISPLAY points to the TCP X11 bridge (Docker Desktop)
export DISPLAY="${DISPLAY:-host.docker.internal:0}"

exec "$@"
