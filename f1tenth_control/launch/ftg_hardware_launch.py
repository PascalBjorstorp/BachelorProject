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
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('f1tenth_control')

    # Default config path
    default_config_path = os.path.join(pkg_share, 'config', 'ftg_params.yaml')

    # Verify config file exists at build time
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
        default_value='3.0',  # Conservative for real hardware
        description='Maximum speed in m/s (recommend starting low: 2-3 m/s)'
    )

    declare_min_speed = DeclareLaunchArgument(
        'min_speed',
        default_value='1.0',
        description='Minimum speed in m/s'
    )

    declare_mapping_mode = DeclareLaunchArgument(
        'mapping_mode',
        default_value='false',
        description='Enable mapping mode for track boundary extraction'
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
            '    Subscribing: /scan, /odom\n',
            '    Publishing:  /drive\n',
            '\n',
            '  Controls:\n',
            '    Hold R1 on controller to enable autonomous\n',
            '    Hold L1 for manual override\n',
            '    Circle for emergency stop\n',
            '=' * 60, '\n',
        ]
    )

    # FTG Node configured for real hardware
    ftg_node = Node(
        package='f1tenth_control',
        executable='ftg_node',
        name='ftg_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'max_speed': LaunchConfiguration('max_speed'),
                'min_speed': LaunchConfiguration('min_speed'),
                'mapping_mode': LaunchConfiguration('mapping_mode'),
            }
        ],
        remappings=[
            # Hardware topic names (no namespace prefix)
            ('scan', '/scan'),
            ('odom', '/ego_racecar/odom'),
            ('drive', '/drive'),
        ]
    )

    return LaunchDescription([
        declare_max_speed,
        declare_min_speed,
        declare_mapping_mode,
        declare_config_file,
        startup_info,
        ftg_node,
    ])
