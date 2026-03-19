"""
Lateral Planner Launch File

Starts the C++ lateral planner node for opponent avoidance.

Usage:
  ros2 launch f1tenth_lateral_planner lateral_planner.launch.py
  ros2 launch f1tenth_lateral_planner lateral_planner.launch.py \
      trajectory_file:=/path/to/raceline.csv
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _parse_bool(text: str) -> bool:
    return str(text).strip().lower() in ('1', 'true', 'yes', 'on')


def _create_planner_node(context, config_path):
    # Use YAML values by default. Apply launch overrides only when explicit.
    sentinel = '__from_yaml__'
    overrides = {}

    trajectory_file = LaunchConfiguration('trajectory_file').perform(context)
    if trajectory_file != sentinel:
        overrides['trajectory_file'] = trajectory_file

    enabled = LaunchConfiguration('enabled').perform(context)
    if enabled != sentinel:
        overrides['enabled'] = _parse_bool(enabled)

    parameters = [config_path]
    if overrides:
        parameters.append(overrides)

    return [Node(
        package='f1tenth_lateral_planner',
        executable='lateral_planner_node',
        name='lateral_planner_node',
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_lateral_planner')
    config_path = os.path.join(pkg_dir, 'config', 'lateral_planner.yaml')

    declare_trajectory = DeclareLaunchArgument(
        'trajectory_file',
        default_value='__from_yaml__',
        description='Optional override for trajectory_file. Default: value from lateral_planner.yaml'
    )

    declare_enabled = DeclareLaunchArgument(
        'enabled',
        default_value='__from_yaml__',
        description='Optional override for enabled. Default: value from lateral_planner.yaml'
    )

    return LaunchDescription([
        declare_trajectory,
        declare_enabled,
        OpaqueFunction(function=lambda context: _create_planner_node(context, config_path)),
    ])
