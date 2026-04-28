"""
Launch the odometry-only localization path.

This is useful for pure odometry tests when the base hardware stack is already
running and downstream control still expects /ekf_pose. It launches the odom
relay and EKF, but no AMCL correction source.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the unified localization YAML parameter file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

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
