"""
Lateral Planner Launch File

Starts the C++ lateral planner node for opponent avoidance.

Configuration defaults are compile-time via:
  f1tenth_lateral_planner/config/lateral_planner_config.hpp

Runtime override:
  avoidance_enabled:=true|false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'avoidance_enabled',
            default_value='true',
            description='Enable obstacle avoidance (false publishes baseline raceline only)'
        ),
        Node(
            package='f1tenth_lateral_planner',
            executable='lateral_planner_node',
            name='lateral_planner_node',
            output='screen',
            parameters=[{
                'avoidance_enabled': LaunchConfiguration('avoidance_enabled')
            }],
        ),
    ])
