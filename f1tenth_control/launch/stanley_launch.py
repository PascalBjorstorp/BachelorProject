"""
Launch file for Stanley path-following controller.

Stanley is more robust than Pure Pursuit at high speeds due to:
- Heading error correction
- Velocity-dependent cross-track error gain
- Feedforward steering from path curvature
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    f1tenth_control_share = get_package_share_directory('f1tenth_control')
    
    # Default trajectory path
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(f1tenth_control_share)))
    default_trajectory = os.path.join(
        workspace_root, 'f1tenth_planning', 'trajectories', 'Spielberg_raceline.csv'
    )
    
    if not os.path.exists(default_trajectory):
        default_trajectory = os.path.expanduser(
            '/ros2_ws/src/f1tenth_planning/trajectories/Spielberg_raceline.csv'
        )
    
    # Launch arguments
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to trajectory CSV file'
    )
    
    # Stanley gains - analytically motivated defaults
    # The controller is: δ = k_h * θ_e + atan(k_e * e / (k_s + v)) + k_ff * κ * L - k_d * ω
    # 
    # k_e: Cross-track gain. Higher = tighter path tracking. 
    #      The atan() naturally damps at high speed via (k_s + v) denominator.
    # k_h: Heading gain. CRITICAL for stability - too high causes oscillation.
    #      Velocity-adaptive: effective k_h reduces at high speed automatically.
    # k_s: Softening constant. Higher = more high-speed damping.
    # k_d: Damping gain using angular velocity. Directly suppresses oscillation.
    
    k_e_arg = DeclareLaunchArgument(
        'k_e',
        default_value='6.0',  # Increased for tighter tracking
        description='Cross-track error gain (CTE term)'
    )
    
    k_h_arg = DeclareLaunchArgument(
        'k_h',
        default_value='0.5',  # Increase for
        description='Heading error gain (auto-reduces at high speed)'
    )
    
    k_s_arg = DeclareLaunchArgument(
        'k_s',
        default_value='0.2',  # Lower = more responsive at speed
        description='Softening constant for CTE term denominator'
    )
    
    k_d_arg = DeclareLaunchArgument(
        'k_d',
        default_value='0.04',  # Reduced damping - just enough to prevent sharp oscillation
        description='Damping gain (suppresses oscillation)'
    )
    
    # Speed settings
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='20.0',
        description='Maximum speed [m/s]'
    )
    
    speed_gain_arg = DeclareLaunchArgument(
        'speed_gain',
        default_value='1.0',
        description='Multiplier for trajectory target speeds'
    )
    
    # Stanley component container
    stanley_container = ComposableNodeContainer(
        name='stanley_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='f1tenth_control',
                plugin='f1tenth_control::StanleyNode',
                name='stanley_node',
                parameters=[{
                    'trajectory_file': LaunchConfiguration('trajectory_file'),
                    'k_e': LaunchConfiguration('k_e'),
                    'k_h': LaunchConfiguration('k_h'),
                    'k_s': LaunchConfiguration('k_s'),
                    'k_d': LaunchConfiguration('k_d'),
                    'use_feedforward': True,
                    'feedforward_gain': 1.0,
                    'max_speed': LaunchConfiguration('max_speed'),
                    'min_speed': 1.0,
                    'speed_gain': LaunchConfiguration('speed_gain'),
                    'max_steering': 0.4189,
                    'max_steering_rate': 3.0,  # rad/s - allows responsive steering
                    'wheelbase': 0.3302,
                    'curvature_speed_factor': 0.5,
                    'publish_visualization': True,
                    'control_rate': 200.0,
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
        k_e_arg,
        k_h_arg,
        k_s_arg,
        k_d_arg,
        max_speed_arg,
        speed_gain_arg,
        stanley_container,
    ])
