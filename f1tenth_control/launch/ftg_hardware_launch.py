"""Follow The Gap (FTG) Hardware Launch File.

Launch the FTG algorithm for deployment on real F1Tenth hardware.
Uses hardware topic names and conservative default parameters.

Usage:
  ros2 launch f1tenth_control ftg_hardware_launch.py
  ros2 launch f1tenth_control ftg_hardware_launch.py max_speed:=3.0

Prerequisites:
  1. Launch f1tenth_system first:
     ros2 launch f1tenth_stack bringup_launch.py
  
  2. Hold R1 on controller to enable autonomous mode

"""

import os
import warnings

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Build and return the FTG hardware launch description."""
    # Get package share directory
    pkg_share = get_package_share_directory('f1tenth_control')

    # Default config path
    default_config_path = os.path.join(pkg_share, 'config', 'ftg_params.yaml')

    # Verify config file exists at launch-time path resolution.
    if not os.path.exists(default_config_path):
        warnings.warn(
            'FTG configuration file not found at "%s". '
            'The node may fail at runtime if config_file argument is not provided.'
            % default_config_path,
            RuntimeWarning
        )

    # Declare launch arguments - conservative defaults for real hardware
    declare_max_speed = DeclareLaunchArgument(
        'max_speed',
        default_value='3.0',  # Hardware-focused default limit
        description='Maximum speed in m/s used by FTG command limiting'
    )

    declare_min_speed = DeclareLaunchArgument(
        'min_speed',
        default_value='1.0',
        description='Minimum speed in m/s'
    )

    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_path,
        description='Path to the FTG configuration file'
    )

    # Log startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '=' * 60, '\n',
            '  F1Tenth FTG Hardware Mode\n',
            '=' * 60, '\n',
            '  Max Speed: ', LaunchConfiguration('max_speed'), ' m/s\n',
            '  Min Speed: ', LaunchConfiguration('min_speed'), ' m/s\n',
            '\n',
            '  Topics:\n',
            '    Subscribing: /scan, /ego_racecar/odom\n',
            '    Publishing:  /drive\n',
            '\n',
            '  Controls:\n',
            '    Hold R1 on controller to enable autonomous\n',
            '    Hold L1 for manual override\n',
            '    Circle for emergency stop\n',
            '=' * 60, '\n',
        ]
    )

    # FTG Node in composable container (zero-copy intra-process comms)
    ftg_container = ComposableNodeContainer(
        name='ftg_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='f1tenth_control',
                plugin='f1tenth_control::FTGNode',
                name='ftg_node',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {
                        'max_speed': LaunchConfiguration('max_speed'),
                        'min_speed': LaunchConfiguration('min_speed'),
                    }
                ],
                remappings=[
                    # Hardware topic names (no namespace prefix)
                    ('scan', '/scan'),
                    ('odom', '/ego_racecar/odom'),
                    ('drive', '/drive'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_max_speed,
        declare_min_speed,
        declare_config_file,
        startup_info,
        ftg_container,
    ])
