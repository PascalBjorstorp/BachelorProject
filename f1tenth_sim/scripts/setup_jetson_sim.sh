#!/bin/bash
# Setup script for running F1TENTH simulation on Jetson (ROS 2 Humble)
#
# This script:
# 1. Installs Python dependencies for f1tenth_gym
# 2. Builds the workspace
#
# Prerequisites:
# - ROS 2 Humble installed
# - Python 3.10 (default on Ubuntu 22.04)
# - colcon build tools

set -e

echo "=== F1TENTH Simulation Setup for Jetson ==="

# Check ROS 2 Humble
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

if [ "$ROS_DISTRO" != "humble" ]; then
    echo "Warning: Expected ROS 2 Humble, found $ROS_DISTRO"
fi

# Install system dependencies
echo ""
echo "=== Installing system dependencies ==="
sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-venv \
    python3-numpy \
    python3-yaml \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-nav2-map-server

# Install Python dependencies
echo ""
echo "=== Installing Python dependencies ==="

# Use pip with --break-system-packages for ROS integration
# Or create a venv if preferred
pip3 install --user \
    transforms3d \
    gymnasium>=0.29.0 \
    numba \
    bottleneck>=1.3.6 \
    "numpy>=1.18.0,<2.0"

# Install f1tenth_gym from source
echo ""
echo "=== Installing f1tenth_gym ==="
pip3 install --user git+https://github.com/f1tenth/f1tenth_gym.git@dev-dynamics

# Build the workspace
echo ""
echo "=== Building workspace ==="
cd ~/f1tenth_ws
colcon build --packages-select f1tenth_sim --symlink-install

echo ""
echo "=== Setup complete! ==="
echo ""
echo "To run the simulation:"
echo "  source ~/f1tenth_ws/install/setup.bash"
echo "  ros2 launch f1tenth_sim gym_bridge_launch.py ground_truth:=true"
echo ""
echo "To record a bag for AMCL benchmark:"
echo "  # In another terminal:"
echo "  ros2 bag record -o ~/f1tenth_bags/amcl_benchmark /scan /map /tf /tf_static /ego_racecar/odom"
