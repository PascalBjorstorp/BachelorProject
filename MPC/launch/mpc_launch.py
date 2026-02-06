"""
Launch file for MPC with F1/10th simulator

Usage:
  ros2 launch mpc_f1_10th mpc_launch.py

Prerequisites:
  1. F1/10th simulator must be running:
     - In one terminal: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  2. Build this package:
     - colcon build --symlink-install
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for MPC node with F1/10th simulator."""
    
    # Get package directory
    pkg_dir = get_package_share_directory('mpc_f1_10th')
    
    # MPC Node
    mpc_node = Node(
        package='mpc_f1_10th',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        mpc_node,
    ])
