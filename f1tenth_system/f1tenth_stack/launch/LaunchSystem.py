"""

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    params_file_localization = os.path.join(pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')

    return LaunchDescription([
        # ── Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'params_file_localization',
            default_value=params_file_localization,
            description='Path to the unified YAML parameter file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        # NOTE: map_server is now launched from bringup_launch.py
        # so the map is available before localization starts.

        # ── Odom fusion node (must start first) ───────────────────
        Node(
            package='f1tenth_localization',
            executable='odom_fused_node',
            name='odom_fused',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file_localization'),
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
                LaunchConfiguration('params_file_localization'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ── EKF node ──────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='ekf_localization_node',
            name='ekf_localization',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file_localization'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
