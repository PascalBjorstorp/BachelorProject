"""
Launch file for MPC with F1/10th simulator

Usage:
  ros2 launch mpc_f1_10th mpc_launch.py

Prerequisites:
  1. F1/10th simulator must be running:
     - In one terminal: ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  2. Build this package:
     - colcon build --symlink-install
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for MPC node with F1/10th simulator."""

    # Try to resolve default trajectory path from f1tenth_planning package
    # Fall back to a local path if the package is not found
    try:
        from ament_index_python.packages import get_package_share_directory
        planning_pkg_dir = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            planning_pkg_dir, 'trajectories', 'Spielberg_raceline.csv')
    except Exception:
        # Fallback: use path relative to workspace
        default_trajectory = '/home/jonathan/Documents/GitHub/BachelorProject/f1tenth_planning/trajectories/Spielberg_raceline.csv'

    # Launch argument for trajectory file override
    trajectory_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file (TUM format)')

    # MPC Node — pass trajectory path as first argument
    mpc_node = Node(
        package='mpc_f1_10th',
        executable='mpc_node',
        name='mpc_node',
        output='screen',
        emulate_tty=True,
        arguments=[LaunchConfiguration('trajectory_file')],
    )

    return LaunchDescription([
        trajectory_arg,
        mpc_node,
    ])

