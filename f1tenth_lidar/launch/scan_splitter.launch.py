"""
Scan Splitter Launch File

Starts the scan splitter node that classifies LiDAR beams as wall or obstacle
using a precomputed distance field from the static map.

Requires:
  - /map topic (from nav2_map_server or equivalent)
  - /scan topic (from Hokuyo driver or simulator)
  - TF: map → laser frame

Usage:
  ros2 launch f1tenth_lidar scan_splitter.launch.py
  ros2 launch f1tenth_lidar scan_splitter.launch.py obstacle_threshold:=0.4
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_lidar')
    config_path = os.path.join(pkg_dir, 'config', 'scan_splitter.yaml')

    declare_threshold = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.1',
        description='Distance (m) from nearest wall to classify beam as obstacle'
    )

    declare_enable = DeclareLaunchArgument(
        'enable_splitting',
        default_value='true',
        description='Enable wall/obstacle splitting. When false, /scan is passed through as /scan_walls.'
    )

    splitter_node = Node(
        package='f1tenth_lidar',
        executable='scan_splitter_node',
        name='scan_splitter_node',
        output='screen',
        parameters=[
            config_path,
            {
                'obstacle_threshold_m': LaunchConfiguration('obstacle_threshold'),
                'enable_splitting': LaunchConfiguration('enable_splitting'),
            },
        ],
    )

    return LaunchDescription([
        declare_threshold,
        declare_enable,
        splitter_node,
    ])
