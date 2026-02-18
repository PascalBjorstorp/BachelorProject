#!/bin/bash
# Run F1/10th Simulator

source /opt/ros/jazzy/setup.bash

SIMULATOR_DIR=~/BachelorProject/f1tenth_sim

if [ ! -d "$SIMULATOR_DIR" ]; then
    echo "Error: Simulator not found at $SIMULATOR_DIR"
    echo ""
    echo "Install it first:"
    echo "  git clone https://github.com/PascalBjorstorp/BachelorProject.git"
    echo "  cd BachelorProject/f1tenth_sim"
    echo "  ./setup.sh"
    exit 1
fi

echo "Starting F1/10th Simulator..."
echo ""
echo "Published Topics:"
echo "  - /ego_racecar/odom (nav_msgs/Odometry)"
echo "  - /scan (sensor_msgs/LaserScan)"
echo ""
echo "Subscribed Topics:"
echo "  - /drive (ackermann_msgs/AckermannDriveStamped)"
echo ""

cd "$SIMULATOR_DIR"
source ~/.venv/bin/activate
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
