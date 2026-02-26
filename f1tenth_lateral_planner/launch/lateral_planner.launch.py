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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_lateral_planner')
    config_path = os.path.join(pkg_dir, 'config', 'lateral_planner.yaml')

    # Resolve default trajectory from f1tenth_planning package
    try:
        planning_share = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            planning_share, 'trajectories', 'Spielberg_raceline.csv'
        )
    except Exception:
        default_trajectory = ''

    declare_trajectory = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to global raceline CSV'
    )

    planner_node = Node(
        package='f1tenth_lateral_planner',
        executable='lateral_planner_node',
        name='lateral_planner_node',
        output='screen',
        parameters=[
            config_path,
            {'trajectory_file': LaunchConfiguration('trajectory_file')},
        ],
    )

    return LaunchDescription([
        declare_trajectory,
        planner_node,
    ])
