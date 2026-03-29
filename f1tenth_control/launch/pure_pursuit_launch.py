"""
Launch file for Pure Pursuit path follower.

Loads a pre-computed racing line and starts the Pure Pursuit controller
as a composable ROS2 node. Trajectory path defaults to f1tenth_planning share.

Topics:
  Sub: /ego_racecar/odom, <pose_topic>, /pp_enable
  Pub: /drive

Prerequisites:
  - f1tenth_planning package installed with trajectory CSV.
  - Localization stack publishing <pose_topic>.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Build and return the Pure Pursuit launch description."""
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
        default_value='0.48',
        description='Minimum lookahead distance [m]'
    )
    
    max_lookahead_arg = DeclareLaunchArgument(
        'max_lookahead',
        default_value='1.20',
        description='Maximum lookahead distance [m]'
    )
    
    lookahead_gain_arg = DeclareLaunchArgument(
        'lookahead_gain',
        default_value='0.15',
        description='Velocity-proportional lookahead gain. Lookahead = gain * speed + min, capped at max [m/(m/s)]'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='5.5',
        description='Maximum commanded speed cap [m/s]'
    )

    cte_lookahead_weight_arg = DeclareLaunchArgument(
        'cte_lookahead_weight',
        default_value='1.0',
        description='Weight on cross-track error in dynamic lookahead. Higher values increase lookahead in when CTE is large [unitless]'
    )

    cte_lookahead_gain_arg = DeclareLaunchArgument(
        'cte_lookahead_gain',
        default_value='0.05',
        description='Lookahead reduction gain based on cross-track error. Higher values reduce lookahead when CTE is large [m/m]'
    )

    curvature_lookahead_gain_arg = DeclareLaunchArgument(
        'curvature_lookahead_gain',
        default_value='1.34',
        description='Turn-radius-based lookahead limit factor (L_max = gain/kappa). Higher values reduce lookahead in tight curves [m]'
    )

    curvature_speed_factor_arg = DeclareLaunchArgument(
        'curvature_speed_factor',
        default_value='0.10',
        description='Curvature-based speed slowdown aggressiveness. Higher values result in more aggressive speed reduction in tight curves [unitless]'
    )

    curvature_speed_floor_ratio_arg = DeclareLaunchArgument(
        'curvature_speed_floor_ratio',
        default_value='0.43',
        description='Minimum speed ratio after curvature slowdown [0..1]'
    )

    cte_speed_factor_arg = DeclareLaunchArgument(
        'cte_speed_factor',
        default_value='0.10',
        description='CTE-based speed slowdown aggressiveness. Higher values result in more aggressive speed reduction when CTE is large [unitless]'
    )

    cte_speed_floor_ratio_arg = DeclareLaunchArgument(
        'cte_speed_floor_ratio',
        default_value='0.37',
        description='Minimum speed ratio after CTE slowdown [0..1]'
    )

    max_lateral_accel_arg = DeclareLaunchArgument(
        'max_lateral_accel',
        default_value='7.27',
        description='Physics-aware lateral acceleration limit for speed regulation [m/s^2]'
    )

    min_regulated_speed_arg = DeclareLaunchArgument(
        'min_regulated_speed',
        default_value='0.30',
        description='Minimum speed allowed after regulation [m/s]'
    )

    curvature_preview_factor_arg = DeclareLaunchArgument(
        'curvature_preview_factor',
        default_value='1.2',
        description='Preview distance multiplier for curvature-based braking'
    )

    vehicle_half_width_arg = DeclareLaunchArgument(
        'vehicle_half_width',
        default_value='0.1365',
        description='Half of physical car width for corridor clearance [m]'
    )

    wall_safety_margin_arg = DeclareLaunchArgument(
        'wall_safety_margin',
        default_value='0.03',
        description='Additional static wall clearance margin [m]'
    )

    corridor_half_width_ref_arg = DeclareLaunchArgument(
        'corridor_half_width_ref',
        default_value='0.25',
        description='Reference usable half-width for full speed [m]'
    )

    corridor_speed_floor_ratio_arg = DeclareLaunchArgument(
        'corridor_speed_floor_ratio',
        default_value='0.20',
        description='Floor for corridor-based speed scaling [0..1]'
    )

    corridor_lookahead_factor_arg = DeclareLaunchArgument(
        'corridor_lookahead_factor',
        default_value='2.0',
        description='Additional lookahead allowed per usable half-width [m/m]'
    )

    max_steering_rate_arg = DeclareLaunchArgument(
        'max_steering_rate',
        default_value='2.80',
        description='Steering rate limit applied in callback-driven command shaping [rad/s]'
    )

    max_accel_cmd_arg = DeclareLaunchArgument(
        'max_accel_cmd',
        default_value='3.00',
        description='Command-side acceleration limit [m/s^2]'
    )

    max_decel_cmd_arg = DeclareLaunchArgument(
        'max_decel_cmd',
        default_value='5.00',
        description='Command-side deceleration limit [m/s^2]'
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

    odom_timeout_arg = DeclareLaunchArgument(
        'odom_timeout_s',
        default_value='0.20',
        description='Fail-safe timeout for stale odometry [s]'
    )
    
    # Pure Pursuit component container
    pure_pursuit_container = ComposableNodeContainer(
        name='pure_pursuit_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='f1tenth_control',
                plugin='f1tenth_control::PurePursuitNode',
                name='pure_pursuit_node',
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
                    'max_lateral_accel': LaunchConfiguration('max_lateral_accel'),
                    'min_regulated_speed': LaunchConfiguration('min_regulated_speed'),
                    'curvature_preview_factor': LaunchConfiguration('curvature_preview_factor'),
                    'vehicle_half_width': LaunchConfiguration('vehicle_half_width'),
                    'wall_safety_margin': LaunchConfiguration('wall_safety_margin'),
                    'corridor_half_width_ref': LaunchConfiguration('corridor_half_width_ref'),
                    'corridor_speed_floor_ratio': LaunchConfiguration('corridor_speed_floor_ratio'),
                    'corridor_lookahead_factor': LaunchConfiguration('corridor_lookahead_factor'),
                    'max_steering': 0.4189,
                    'wheelbase': 0.324,
                    'max_steering_rate': LaunchConfiguration('max_steering_rate'),
                    'max_accel_cmd': LaunchConfiguration('max_accel_cmd'),
                    'max_decel_cmd': LaunchConfiguration('max_decel_cmd'),
                    'pose_topic': LaunchConfiguration('pose_topic'),
                    'pose_timeout_s': LaunchConfiguration('pose_timeout_s'),
                    'odom_timeout_s': LaunchConfiguration('odom_timeout_s'),
                }],
                remappings=[
                    ('/odom', '/ego_racecar/odom'),
                    ('/drive', '/drive'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
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
        max_lateral_accel_arg,
        min_regulated_speed_arg,
        curvature_preview_factor_arg,
        vehicle_half_width_arg,
        wall_safety_margin_arg,
        corridor_half_width_ref_arg,
        corridor_speed_floor_ratio_arg,
        corridor_lookahead_factor_arg,
        max_steering_rate_arg,
        max_accel_cmd_arg,
        max_decel_cmd_arg,
        pose_topic_arg,
        pose_timeout_arg,
        odom_timeout_arg,
        pure_pursuit_container,
    ])
