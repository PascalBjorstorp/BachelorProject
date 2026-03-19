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
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(text: str) -> bool:
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def _create_splitter_node(context, config_path):
    # Only apply launch overrides when they are explicitly set.
    # Otherwise, values from scan_splitter.yaml are used as-is.
    sentinel = '__from_yaml__'
    overrides = {}

    obstacle_threshold = LaunchConfiguration('obstacle_threshold').perform(context)
    if obstacle_threshold != sentinel:
        overrides['obstacle_threshold_m'] = float(obstacle_threshold)

    enable_splitting = LaunchConfiguration('enable_splitting').perform(context)
    if enable_splitting != sentinel:
        overrides['enable_splitting'] = _parse_bool(enable_splitting)

    min_cluster_size = LaunchConfiguration('min_cluster_size').perform(context)
    if min_cluster_size != sentinel:
        overrides['min_cluster_size'] = int(min_cluster_size)

    parameters = [config_path]
    if overrides:
        parameters.append(overrides)

    return [Node(
        package='f1tenth_lidar',
        executable='scan_splitter_node',
        name='scan_splitter_node',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_lidar')
    config_path = os.path.join(pkg_dir, 'config', 'scan_splitter.yaml')

    declare_threshold = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='__from_yaml__',
        description='Optional override for obstacle_threshold_m. Default: value from scan_splitter.yaml'
    )

    declare_enable = DeclareLaunchArgument(
        'enable_splitting',
        default_value='__from_yaml__',
        description='Optional override for enable_splitting. Default: value from scan_splitter.yaml'
    )

    declare_min_cluster_size = DeclareLaunchArgument(
        'min_cluster_size',
        default_value='__from_yaml__',
        description='Optional override for min_cluster_size. Default: value from scan_splitter.yaml'
    )

    return LaunchDescription([
        declare_threshold,
        declare_enable,
        declare_min_cluster_size,
        OpaqueFunction(function=lambda context: _create_splitter_node(context, config_path)),
    ])
