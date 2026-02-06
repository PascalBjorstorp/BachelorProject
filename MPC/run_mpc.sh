#!/bin/bash
# Run MPC Node

source /opt/ros/jazzy/setup.bash
source ~/sim_ws/install/local_setup.bash

echo "Starting MPC Node..."
echo ""
echo "Make sure the simulator is running in another terminal:"
echo "  ./run_simulator.sh"
echo ""
echo "Control Loop:"
echo "  - Subscribes to: /ego_racecar/odom"
echo "  - Publishes to: /drive"
echo "  - Frequency: 20 Hz"
echo ""

ros2 launch mpc_f1_10th mpc_launch.py
