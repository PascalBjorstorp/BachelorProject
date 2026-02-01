"""
Launch file for Performance Monitor.

Monitors CPU, GPU (on Jetson), memory usage, and localization latency.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Parameters
    output_dir_arg = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/f1tenth_performance',
        description='Directory to save performance CSV files'
    )
    
    sample_rate_arg = DeclareLaunchArgument(
        'sample_rate_hz',
        default_value='10.0',
        description='How often to sample performance metrics'
    )
    
    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='LiDAR scan topic'
    )
    
    amcl_pose_topic_arg = DeclareLaunchArgument(
        'amcl_pose_topic',
        default_value='/amcl_pose',
        description='AMCL pose output topic'
    )
    
    # Performance Monitor Node
    monitor_node = Node(
        package='f1tenth_localization',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('output_dir'),
            'sample_rate_hz': LaunchConfiguration('sample_rate_hz'),
            'scan_topic': LaunchConfiguration('scan_topic'),
            'amcl_pose_topic': LaunchConfiguration('amcl_pose_topic'),
        }]
    )
    
    return LaunchDescription([
        output_dir_arg,
        sample_rate_arg,
        scan_topic_arg,
        amcl_pose_topic_arg,
        monitor_node,
    ])
