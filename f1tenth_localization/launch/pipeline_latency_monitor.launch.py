"""
Pipeline Latency Monitor Launch File

Runs a lightweight C++ node that measures per-stage latency through the
localization pipeline:
  /scan → /scan_walls → /amcl_pose → /ekf_pose

Prints a summary to the terminal at ~1 Hz (configurable via print_every).

Usage:
  ros2 launch f1tenth_localization pipeline_latency_monitor.launch.py
  ros2 launch f1tenth_localization pipeline_latency_monitor.launch.py print_every:=20
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'print_every',
            default_value='40',
            description='Print latency summary every N scan cycles (~1 Hz at 40 Hz scan rate)'),

        Node(
            package='f1tenth_localization',
            executable='pipeline_latency_monitor',
            name='pipeline_latency_monitor',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'walls_topic': '/scan_walls',
                'amcl_topic': '/amcl_pose',
                'ekf_topic': '/ekf_pose',
                'print_every': LaunchConfiguration('print_every'),
            }],
        ),
    ])
