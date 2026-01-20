#!/bin/bash
# F1Tenth Gym ROS2 Bridge - Run Script
# This script activates the environment and launches the simulation

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

# Check if setup has been run
if [ ! -d "$VENV_DIR" ]; then
    echo "ERROR: Virtual environment not found!"
    echo "Please run setup.sh first:"
    echo "  ./setup.sh"
    exit 1
fi

if [ ! -d "$SCRIPT_DIR/install" ]; then
    echo "ERROR: Package not built!"
    echo "Please run setup.sh first:"
    echo "  ./setup.sh"
    exit 1
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Ensure venv packages take priority over system packages
# This is needed because ROS2 spawns subprocesses that may not inherit venv properly
export PYTHONPATH="$VENV_DIR/lib/python3.12/site-packages:$PYTHONPATH"

# Source ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Source the local workspace
source "$SCRIPT_DIR/install/setup.bash"

echo "Environment ready!"
echo ""
echo "To launch the simulation:"
echo "  ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
echo ""
echo "To run keyboard teleop (in another terminal after sourcing):"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard"
echo ""

# If script is being sourced, don't launch
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
    # Script is being executed, launch the simulation
    echo "Launching simulation..."
    ros2 launch f1tenth_gym_ros gym_bridge_launch.py
fi
