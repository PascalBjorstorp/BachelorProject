#!/bin/bash
# F1Tenth Gym ROS2 Bridge - Build Script
# Rebuilds the package after code changes

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

# Check if setup has been run
if [ ! -d "$VENV_DIR" ]; then
    echo "ERROR: Virtual environment not found!"
    echo "Please run setup.sh first:"
    echo "  ./setup.sh"
    exit 1
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Build the workspace
echo "Building the ROS2 package..."
cd "$SCRIPT_DIR"
colcon build --symlink-install

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Build successful!"
    echo ""
    echo "To run the simulation:"
    echo "  ./run.sh"
else
    echo ""
    echo "✗ Build failed!"
    exit 1
fi
