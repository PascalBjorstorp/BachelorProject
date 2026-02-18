#!/bin/bash
# Quick Setup: Source ROS2 + workspace + ready for commands

set -e

echo "Setting up ROS2 Jazz environment..."

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source workspace
WORKSPACE_DIR="${HOME}/sim_ws"
if [ -d "$WORKSPACE_DIR/install" ]; then
    source "$WORKSPACE_DIR/install/local_setup.bash"
    echo "✓ Environment ready!"
    echo ""
    echo "Available commands:"
    echo "  ros2 topic list          # See all topics"
    echo "  ros2 topic echo /ego_racecar/odom  # Watch state"
    echo "  ros2 launch mpc_f1_10th mpc_launch.py  # Start MPC"
    echo ""
else
    echo "⚠ Workspace not found at $WORKSPACE_DIR"
    echo "Run setup first: ./setup_ros2.sh"
fi
