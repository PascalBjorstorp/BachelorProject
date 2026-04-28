"""
Pipeline Latency Monitor Launch File

Runs a lightweight C++ node that measures per-stage latency through the
localization pipeline:
    /scan → /amcl_pose → /ekf_pose → /drive → /ackermann_cmd

Prints a summary to the terminal at ~1 Hz (configurable via print_every).
Also logs per-cycle latency samples to CSV (configurable via launch args).
Also launches performance_monitor_cpp for CPU logging.

Usage:
  ros2 launch f1tenth_localization pipeline_latency_monitor.launch.py
  ros2 launch f1tenth_localization pipeline_latency_monitor.launch.py print_every:=20
    ros2 launch f1tenth_localization pipeline_latency_monitor.launch.py log_to_csv:=true csv_output_dir:=/tmp/f1tenth_latency
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # -------------------- Pipeline Latency Monitor --------------------
        DeclareLaunchArgument(
            'print_every',
            default_value='40',
            description='Print latency summary every N scan cycles (~1 Hz at 40 Hz scan rate)'),

        DeclareLaunchArgument(
            'drive_topic',
            default_value='/drive',
            description='Drive command topic used for ekf_pose -> drive latency measurement'),

        DeclareLaunchArgument(
            'ackermann_topic',
            default_value='/ackermann_cmd',
            description='Ackermann mux output topic used for drive -> ackermann latency measurement'),

        DeclareLaunchArgument(
            'amcl_timing_topic',
            default_value='/amcl_timing',
            description='AMCL timing topic recorded into the pipeline latency CSV'),

        DeclareLaunchArgument(
            'amcl_particle_count_topic',
            default_value='/amcl_particle_count',
            description='AMCL particle count topic recorded into the pipeline latency CSV'),

        DeclareLaunchArgument(
            'stage_match_max_ms',
            default_value='20.0',
            description='Maximum stage matching window in ms to reject startup/stale pairs'),

        DeclareLaunchArgument(
            'strict_mode',
            default_value='true',
            description='Enable strict mismatch counters and warnings'),

        DeclareLaunchArgument(
            'log_to_csv',
            default_value='true',
            description='Enable per-cycle latency CSV logging'),

        DeclareLaunchArgument(
            'csv_output_dir',
            default_value='f1tenth_localization/Benchmark/Matlab/csv',
            description='Directory where latency CSV file is written'),

        Node(
            package='f1tenth_localization',
            executable='pipeline_latency_monitor',
            name='pipeline_latency_monitor',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'amcl_topic': '/amcl_pose',
                'amcl_timing_topic': LaunchConfiguration('amcl_timing_topic'),
                'amcl_particle_count_topic': LaunchConfiguration('amcl_particle_count_topic'),
                'ekf_topic': '/ekf_pose',
                'drive_topic': LaunchConfiguration('drive_topic'),
                'ackermann_topic': LaunchConfiguration('ackermann_topic'),
                'stage_match_max_ms': LaunchConfiguration('stage_match_max_ms'),
                'strict_mode': LaunchConfiguration('strict_mode'),
                'print_every': LaunchConfiguration('print_every'),
                'log_to_csv': LaunchConfiguration('log_to_csv'),
                'csv_output_dir': LaunchConfiguration('csv_output_dir'),
            }],
        ),

        Node(
            package='f1tenth_localization',
            executable='performance_monitor_cpp',
            name='performance_monitor',
            output='screen',
        ),
    ])
