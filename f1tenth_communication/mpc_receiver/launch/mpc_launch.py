"""
Launch file for MPC Receiver (software Frenet controller — test/fallback).

Usage:
    ros2 launch mpc_receiver mpc_launch.py trajectory_file:=/path/to/raceline.csv
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('mpc_receiver')
    default_config = os.path.join(pkg_share, 'config', 'mpc_params.yaml')

    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        description='Path to raceline CSV file'
    )

    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value='/drive',
        description='Topic to publish drive commands'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/mpc_state',
        description='Topic to receive MPC state from'
    )

    mpc_receiver_node = Node(
        package='mpc_receiver',
        executable='mpc_receiver_mpc_node',
        name='mpc_receiver_mpc',
        output='screen',
        parameters=[
            default_config,
            {
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'input_topic': LaunchConfiguration('input_topic'),
            }
        ],
    )

    return LaunchDescription([
        trajectory_file_arg,
        drive_topic_arg,
        input_topic_arg,
        mpc_receiver_node,
    ])
