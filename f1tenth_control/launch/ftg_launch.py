"""
Follow The Gap (FTG) Launch File
================================
Launches the FTG algorithm node with configurable parameters.

Usage:
  ros2 launch f1tenth_control ftg_launch.py
  ros2 launch f1tenth_control ftg_launch.py max_speed:=3.0
  ros2 launch f1tenth_control ftg_launch.py mapping_mode:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('f1tenth_control')
    
    # Default config path
    default_config_path = os.path.join(pkg_share, 'config', 'ftg_params.yaml')
    
    # Verify config file exists at build time (helps catch packaging errors)
    if not os.path.exists(default_config_path):
        import warnings
        warnings.warn(
            f"FTG configuration file not found at '{default_config_path}'. "
            "The node may fail at runtime if config_file argument is not provided.",
            RuntimeWarning
        )
    
    # Declare launch arguments for commonly tuned parameters
    declare_max_speed = DeclareLaunchArgument(
        'max_speed',
        default_value='4.0',
        description='Maximum speed in m/s'
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
    
    # FTG Node
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
            ('scan', '/scan'),
            ('odom', '/odom'),
            ('drive', '/drive'),
        ]
    )
    
    return LaunchDescription([
        declare_max_speed,
        declare_min_speed,
        declare_mapping_mode,
        declare_config_file,
        ftg_node,
    ])
