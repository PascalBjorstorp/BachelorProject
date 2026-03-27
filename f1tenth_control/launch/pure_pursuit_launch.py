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
    
    # Default trajectory: look in f1tenth_planning's installed share directory
    try:
        f1tenth_planning_share = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            f1tenth_planning_share, 'trajectories', 'my_track_raceline.csv'
        )
    except Exception:
        # Fallback: try workspace source directory
        workspace_root = os.path.dirname(os.path.dirname(f1tenth_control_share))
        default_trajectory = os.path.join(
            workspace_root, 'f1tenth_planning', 'trajectories', 'my_track_raceline.csv'
        )
    
    # Launch arguments
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file'
    )
    
    min_lookahead_arg = DeclareLaunchArgument(
        'min_lookahead',
        default_value='0.30',
        description='Minimum lookahead distance [m]'
    )
    
    max_lookahead_arg = DeclareLaunchArgument(
        'max_lookahead',
        default_value='0.60',
        description='Maximum lookahead distance [m]'
    )
    
    lookahead_gain_arg = DeclareLaunchArgument(
        'lookahead_gain',
        default_value='0.08',
        description='Velocity-proportional lookahead gain'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='5.0',
        description='Maximum commanded speed cap [m/s]'
    )

    cte_lookahead_weight_arg = DeclareLaunchArgument(
        'cte_lookahead_weight',
        default_value='1.50',
        description='Weight on cross-track error in dynamic lookahead'
    )

    cte_lookahead_gain_arg = DeclareLaunchArgument(
        'cte_lookahead_gain',
        default_value='0.05',
        description='Lookahead reduction gain based on cross-track error [m/m]'
    )

    curvature_lookahead_gain_arg = DeclareLaunchArgument(
        'curvature_lookahead_gain',
        default_value='0.05',
        description='Lookahead reduction gain based on path curvature [m/(1/m)]'
    )

    curvature_speed_factor_arg = DeclareLaunchArgument(
        'curvature_speed_factor',
        default_value='1.20',
        description='Curvature-based speed slowdown aggressiveness'
    )

    curvature_speed_floor_ratio_arg = DeclareLaunchArgument(
        'curvature_speed_floor_ratio',
        default_value='0.12',
        description='Minimum speed ratio after curvature slowdown [0..1]'
    )

    cte_speed_factor_arg = DeclareLaunchArgument(
        'cte_speed_factor',
        default_value='2.50',
        description='CTE-based speed slowdown aggressiveness'
    )

    cte_speed_floor_ratio_arg = DeclareLaunchArgument(
        'cte_speed_floor_ratio',
        default_value='0.25',
        description='Minimum speed ratio after CTE slowdown [0..1]'
    )
    
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/ekf_pose',
        description='Pose topic in map frame'
    )

    pose_timeout_arg = DeclareLaunchArgument(
        'pose_timeout_s',
        default_value='0.10',
        description='Fail-safe timeout for stale pose [s]'
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
            'cte_lookahead_weight': LaunchConfiguration('cte_lookahead_weight'),
            'cte_lookahead_gain': LaunchConfiguration('cte_lookahead_gain'),
            'curvature_lookahead_gain': LaunchConfiguration('curvature_lookahead_gain'),
            'curvature_speed_factor': LaunchConfiguration('curvature_speed_factor'),
            'curvature_speed_floor_ratio': LaunchConfiguration('curvature_speed_floor_ratio'),
            'cte_speed_factor': LaunchConfiguration('cte_speed_factor'),
            'cte_speed_floor_ratio': LaunchConfiguration('cte_speed_floor_ratio'),
            'max_steering': 0.4189,
            'wheelbase': 0.3302,
            'publish_visualization': True,
            'pose_topic': LaunchConfiguration('pose_topic'),
            'pose_timeout_s': LaunchConfiguration('pose_timeout_s'),
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
        cte_lookahead_weight_arg,
        cte_lookahead_gain_arg,
        curvature_lookahead_gain_arg,
        curvature_speed_factor_arg,
        curvature_speed_floor_ratio_arg,
        cte_speed_factor_arg,
        cte_speed_floor_ratio_arg,
        pose_topic_arg,
        pose_timeout_arg,
        pure_pursuit_node,
    ])
