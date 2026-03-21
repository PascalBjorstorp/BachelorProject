"""
Launch file for FPGA-only MPC receiver.

Usage:
    ros2 launch state_receiver mpc_launch.py

Arguments:
    drive_topic: Topic to publish drive commands (default: /drive)
    input_topic: Topic to receive MPC state from (default: /mpc_state)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('state_receiver')
    default_config = os.path.join(pkg_share, 'config', 'mpc_params.yaml')

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

    mpc_fpga_node = Node(
        package='state_receiver',
        executable='mpc_receiver_node',
        name='mpc_receiver',
        output='screen',
        parameters=[
            default_config,
            {
                'drive_topic': LaunchConfiguration('drive_topic'),
                'input_topic': LaunchConfiguration('input_topic'),
            }
        ],
    )

    return LaunchDescription([
        drive_topic_arg,
        input_topic_arg,
        mpc_fpga_node,
    ])
