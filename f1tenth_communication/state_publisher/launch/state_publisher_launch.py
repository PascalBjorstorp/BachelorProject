"""
Launch state publisher node for MPC communication.
Runs on Jetson to publish vehicle state + waypoint index.
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
    # Use params file from config directory
    params_file = PathJoinSubstitution([
        FindPackageShare('state_publisher'),
        'config',
        'params.yaml'
    ])

    try:
        planning_share = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            planning_share,
            'trajectories',
            'my_track_raceline.csv'
        )
    except Exception:
        default_trajectory = '/ros2_ws/src/f1tenth_planning/trajectories/my_track_raceline.csv'

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
