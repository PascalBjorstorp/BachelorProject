"""
Scan Splitter Launch File

Starts the scan splitter node that classifies LiDAR beams as wall or obstacle
using a precomputed distance field from the static map.

Configuration is compile-time via:
  config/scan_splitter_config.hpp

Requires:
  - /map topic (from nav2_map_server or equivalent)
  - /scan topic (from Hokuyo driver or simulator)
  - TF: map → laser frame

Usage:
  ros2 launch f1tenth_lidar scan_splitter.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_lidar',
            executable='scan_splitter_node',
            name='scan_splitter_node',
            output='screen',
        ),
    ])
