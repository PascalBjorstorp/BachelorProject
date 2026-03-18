"""
MPCC ROS2 Launch File

Launches the MPCC node with a reference trajectory.

Usage:
    ros2 launch mpcc_f1_10th mpcc_launch.py

Override trajectory:
    ros2 launch mpcc_f1_10th mpcc_launch.py trajectory_file:=/path/to/traj.csv
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve default trajectory path from f1tenth_planning if available
    try:
        from ament_index_python.packages import get_package_share_directory
        planning_dir = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            planning_dir, 'trajectories', 'Spielberg_raceline_optimized_wide.csv')
    except Exception:
        default_trajectory = '/ros2_ws/src/f1tenth_planning/trajectories/Spielberg_raceline_optimized_wide.csv'

    trajectory_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file (TUM format)')

    return LaunchDescription([
        trajectory_arg,

        # MPCC controller node
        Node(
            package='mpcc_f1_10th',
            executable='mpcc_node',
            name='mpcc_node',
            output='screen',
            emulate_tty=True,
            arguments=[LaunchConfiguration('trajectory_file')],
        ),
    ])
