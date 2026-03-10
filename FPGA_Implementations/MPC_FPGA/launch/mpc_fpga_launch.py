"""
Launch file for FPGA MPC (HLS C-simulation) with F1/10th simulator.

Usage:
  ros2 launch mpc_fpga mpc_fpga_launch.py

Prerequisites:
  1. F1/10th simulator must be running:
     - ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  2. Build this package:
     - colcon build --packages-select mpc_fpga
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Generate launch description for FPGA MPC node."""

    # Resolve default trajectory path from f1tenth_planning if available
    try:
        from ament_index_python.packages import get_package_share_directory
        planning_dir = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            planning_dir, 'trajectories', 'Spielberg_raceline.csv')
    except Exception:
        default_trajectory = '/ros2_ws/src/f1tenth_planning/trajectories/Spielberg_raceline.csv'

    trajectory_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file (TUM format)')

    mpc_fpga_node = Node(
        package='mpc_fpga',
        executable='mpc_fpga_node',
        name='mpc_fpga_node',
        output='screen',
        emulate_tty=True,
        arguments=[LaunchConfiguration('trajectory_file')],
    )

    return LaunchDescription([
        trajectory_arg,
        mpc_fpga_node,
    ])
