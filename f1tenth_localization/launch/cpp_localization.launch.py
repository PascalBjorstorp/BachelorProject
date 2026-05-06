"""
Launch file for the C++ GPU AMCL localization stack.

Launches three nodes:
    1. gpu_amcl_cpp      — GPU particle-filter AMCL (40 Hz, subscribes to /scan)
  2. odom_fused        — IMU + wheel odom fusion (200 Hz)
  3. ekf_localization  — EKF sensor fusion + TF broadcast

The map_server is expected to be running from bringup_launch.py.
The LiDAR driver publishing /scan is expected to be running from bringup_launch.py.

All parameters are loaded from config/gpu_amcl_cpp_params.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')

    return LaunchDescription([
        # ── Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the unified YAML parameter file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        DeclareLaunchArgument(
            'amcl_global_initialization',
            default_value='true',
            description='Seed GPU AMCL particles globally along raceline with heading cone'),

        # NOTE: map_server is now launched from bringup_launch.py
        # so the map is available before localization starts.

        # ── Odom fusion node (must start first) ───────────────────
        Node(
            package='f1tenth_localization',
            executable='odom_fused_node',
            name='odom_fused',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ── AMCL node ─────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='gpu_amcl_cpp_node',
            name='gpu_amcl_cpp',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'global_initialization': ParameterValue(
                    LaunchConfiguration('amcl_global_initialization'),
                    value_type=bool)},
            ],
        ),

        # ── EKF node ──────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='ekf_localization_node',
            name='ekf_localization',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
