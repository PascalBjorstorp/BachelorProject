"""
@file state_publisher_launch.py
@brief Launch description for Jetson-side MPC state publisher.
@details Starts state_publisher with parameter file defaults and a configurable
trajectory CSV input path.
@dependencies launch, launch_ros, ament_index_python, state_publisher package
"""

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    """
    @brief Build launch description for the Jetson state publisher node.

    @return LaunchDescription with trajectory argument and configured node action.
    """
    # Use params file from config directory
    params_file = PathJoinSubstitution([
        FindPackageShare('state_publisher'),
        'config',
        'params.yaml'
    ])

    # Resolve trajectory defaults from installed and source layouts.
    state_publisher_share = get_package_share_directory('state_publisher')
    install_prefix = os.path.dirname(os.path.dirname(os.path.dirname(state_publisher_share)))
    workspace_root = os.path.dirname(install_prefix)

    trajectory_candidates = []
    try:
        planning_share = get_package_share_directory('f1tenth_planning')
        trajectory_candidates.append(
            os.path.join(planning_share, 'trajectories', 'my_track_raceline.csv')
        )
    except Exception:
        pass

    trajectory_candidates.append(
        os.path.join(
            install_prefix,
            'f1tenth_planning',
            'share',
            'f1tenth_planning',
            'trajectories',
            'my_track_raceline.csv',
        )
    )
    trajectory_candidates.append(
        os.path.join(
            workspace_root,
            'src',
            'f1tenth_planning',
            'trajectories',
            'my_track_raceline.csv',
        )
    )

    default_trajectory = next(
        (candidate for candidate in trajectory_candidates if os.path.exists(candidate)),
        trajectory_candidates[0],
    )

    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file'
    )
    
    return LaunchDescription([
        trajectory_file_arg,
        Node(
            package='state_publisher',
            executable='state_publisher_node',
            name='state_publisher',
            output='screen',
            parameters=[
                params_file,
                {
                    'trajectory_file': LaunchConfiguration('trajectory_file')
                }
            ],
        ),
    ])
