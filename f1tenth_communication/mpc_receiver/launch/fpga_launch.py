"""
Launch file for MPC Receiver with FPGA Pure Pursuit controller.

Usage:
    ros2 launch mpc_receiver fpga_launch.py trajectory_file:=/path/to/raceline.csv

Arguments:
    trajectory_file: Path to raceline CSV file (required)
    use_fpga: Whether to use FPGA or software fallback (default: true)
    drive_topic: Topic to publish drive commands (default: /drive)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package paths
    pkg_share = get_package_share_directory('mpc_receiver')
    default_config = os.path.join(pkg_share, 'config', 'fpga_params.yaml')
    
    # Declare launch arguments
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        description='Path to raceline CSV file'
    )
    
    use_fpga_arg = DeclareLaunchArgument(
        'use_fpga',
        default_value='true',
        description='Use FPGA hardware (false for software fallback)'
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
    
    # MPC Receiver FPGA Node
    mpc_receiver_fpga_node = Node(
        package='mpc_receiver',
        executable='mpc_receiver_fpga_node',
        name='mpc_receiver_fpga',
        output='screen',
        parameters=[
            default_config,
            {
                'trajectory_file': LaunchConfiguration('trajectory_file'),
                'use_fpga': LaunchConfiguration('use_fpga'),
                'drive_topic': LaunchConfiguration('drive_topic'),
                'input_topic': LaunchConfiguration('input_topic'),
            }
        ],
        remappings=[
            ('/drive', LaunchConfiguration('drive_topic')),
            ('/mpc_state', LaunchConfiguration('input_topic')),
        ]
    )
    
    return LaunchDescription([
        trajectory_file_arg,
        use_fpga_arg,
        drive_topic_arg,
        input_topic_arg,
        mpc_receiver_fpga_node,
    ])
