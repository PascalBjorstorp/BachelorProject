"""
Launch file for Pure Pursuit path follower.

This launches the Pure Pursuit node which follows a pre-computed racing line.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    f1tenth_control_share = get_package_share_directory('f1tenth_control')
    
    # Default trajectory path - go up from install dir to find planning package
    # This works both from install and source directories
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(f1tenth_control_share)))
    default_trajectory = os.path.join(
        workspace_root, 'f1tenth_planning', 'trajectories', 'Spielberg_raceline.csv'
    )
    
    # Fallback: check if it exists in the expected location
    if not os.path.exists(default_trajectory):
        # Try relative to home
        default_trajectory = os.path.expanduser(
            '~/Documents/GitHub/BachelorProject/f1tenth_planning/trajectories/Spielberg_raceline.csv'
        )
    
    # Launch arguments
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file'
    )
    
    min_lookahead_arg = DeclareLaunchArgument(
        'min_lookahead',
        default_value='0.2',
        description='Minimum lookahead distance [m]'
    )
    
    max_lookahead_arg = DeclareLaunchArgument(
        'max_lookahead',
        default_value='1.5',
        description='Maximum lookahead distance [m]'
    )
    
    lookahead_gain_arg = DeclareLaunchArgument(
        'lookahead_gain',
        default_value='0.10',
        description='Velocity-proportional lookahead gain'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='12.0',
        description='Maximum speed [m/s]'
    )
    
    speed_gain_arg = DeclareLaunchArgument(
        'speed_gain',
        default_value='0.8',
        description='Multiplier for trajectory target speeds (0-1)'
    )
    
    # Pure Pursuit Node
    pure_pursuit_node = Node(
        package='f1tenth_control',
        executable='pure_pursuit_node_exe',
        name='pure_pursuit_node',
        output='screen',
        parameters=[{
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'min_lookahead': LaunchConfiguration('min_lookahead'),
            'max_lookahead': LaunchConfiguration('max_lookahead'),
            'lookahead_gain': LaunchConfiguration('lookahead_gain'),
            'max_speed': LaunchConfiguration('max_speed'),
            'min_speed': 1.0,
            'speed_gain': LaunchConfiguration('speed_gain'),
            'max_steering': 0.4189,
            'max_steering_rate': 3.0,  # rad/s - limits steering rate for stability
            'wheelbase': 0.3302,
            'curvature_speed_factor': 0.3,  # Speed reduction based on path curvature
            'publish_visualization': True,
            'control_rate': 200.0,
        }],
        remappings=[
            ('/odom', '/ego_racecar/odom'),
            ('/drive', '/drive'),
        ]
    )
    
    return LaunchDescription([
        trajectory_file_arg,
        min_lookahead_arg,
        max_lookahead_arg,
        lookahead_gain_arg,
        max_speed_arg,
        speed_gain_arg,
        pure_pursuit_node,
    ])
