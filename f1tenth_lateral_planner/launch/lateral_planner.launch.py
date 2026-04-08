"""
Lateral Planner Launch File

Starts the C++ lateral planner node for opponent avoidance.

Configuration is compile-time via:
  f1tenth_lateral_planner/config/lateral_planner_config.hpp
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f1tenth_lateral_planner',
            executable='lateral_planner_node',
            name='lateral_planner_node',
            output='screen',
        ),
    ])
