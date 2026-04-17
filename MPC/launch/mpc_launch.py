"""
Launch file for MPC Riccati-ADMM with F1/10th simulator.

Usage:
  ros2 launch mpc_riccati mpc_launch.py

Prerequisites:
  1. F1/10th simulator must be running:
     - ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  2. Build this package:
     - colcon build --symlink-install --packages-select mpc_riccati
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """
    Build the launch graph for the MPC simulator node.

    Resolves the default trajectory from the f1tenth_planning package share,
    falls back to a hardcoded workspace path when the package is unavailable,
    and launches the Riccati-ADMM MPC node against the F1/10th gym simulator.

    Returns:
        LaunchDescription containing the trajectory argument and
        mpc_node action.
    """

    # Resolve default trajectory path from mpc_riccati if available
    try:
        from ament_index_python.packages import get_package_share_directory
        mpc_dir = get_package_share_directory('mpc_riccati')
        default_trajectory = os.path.join(
            mpc_dir, 'trajectories', 'my_track_raceline.csv')
    except Exception:
        # DEPLOYMENT NOTE: This hardcoded fallback path assumes a specific workspace
        # layout. Override via the trajectory_file launch argument.
        default_trajectory = '/ros2_ws/src/MPC/trajectories/my_track_raceline.csv'

    trajectory_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file (TUM format)')

    mpc_node = Node(
        package='mpc_riccati',
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
