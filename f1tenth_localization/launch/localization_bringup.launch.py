"""
F1TENTH Localization Bringup Launch File

Supports multiple localization modes:
- sim:        Ground truth only (simulation default, no localization)
- amcl:       AMCL particle filter localization
- ekf:        EKF odometry fusion only
- ekf_amcl:   Full stack with EKF + AMCL
- scan_match: Scan matching (ICP/NDT) - future

Usage:
  # Simulation with ground truth (no localization)
  ros2 launch f1tenth_localization localization_bringup.launch.py mode:=sim
  
  # AMCL only (for benchmarking on Jetson)
  ros2 launch f1tenth_localization localization_bringup.launch.py mode:=amcl
  
  # Real robot with EKF + AMCL
  ros2 launch f1tenth_localization localization_bringup.launch.py mode:=ekf_amcl is_sim:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Get package directories
    pkg_dir = get_package_share_directory('f1tenth_localization')
    
    # Declare arguments
    declare_mode = DeclareLaunchArgument(
        'mode',
        default_value='sim',
        description='Localization mode: sim, amcl, ekf, ekf_amcl, scan_match',
        choices=['sim', 'amcl', 'ekf', 'ekf_amcl', 'scan_match']
    )
    
    declare_is_sim = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Whether running in simulation'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='spielberg',
        description='Map name for localization'
    )
    
    # AMCL parameters
    declare_min_particles = DeclareLaunchArgument(
        'min_particles',
        default_value='500',
        description='AMCL minimum particles'
    )
    
    declare_max_particles = DeclareLaunchArgument(
        'max_particles',
        default_value='2000',
        description='AMCL maximum particles'
    )
    
    declare_max_beams = DeclareLaunchArgument(
        'max_beams',
        default_value='60',
        description='AMCL max laser beams to use'
    )
    
    # Get configurations
    mode = LaunchConfiguration('mode')
    is_sim = LaunchConfiguration('is_sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ==========================================================================
    # Mode: sim - Ground truth only, no localization needed
    # ==========================================================================
    sim_mode_info = LogInfo(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
        msg='Running in SIM mode: Using ground truth from simulation, no localization active'
    )
    
    # ==========================================================================
    # Mode: amcl - AMCL particle filter localization
    # Needs a fake odom frame when running in simulation
    # ==========================================================================
    
    # Static transform: map -> odom (identity, for simulation)
    # This creates the middle "odom" frame that AMCL expects
    odom_static_tf = Node(
        condition=IfCondition(
            PythonExpression(["'", mode, "' in ['amcl', 'ekf_amcl'] and '", is_sim, "' == 'true'"])
        ),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    # Another static transform: odom -> ego_racecar/odom (if needed)
    odom_ego_static_tf = Node(
        condition=IfCondition(
            PythonExpression(["'", mode, "' in ['amcl', 'ekf_amcl'] and '", is_sim, "' == 'true'"])
        ),
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_ego_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'ego_racecar/odom'],
        output='screen'
    )
    
    # AMCL Node
    amcl_node = LifecycleNode(
        condition=IfCondition(PythonExpression(["'", mode, "' in ['amcl', 'ekf_amcl']"])),
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='/',
        output='screen',
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Frame IDs
                'base_frame_id': 'ego_racecar/base_link',
                'odom_frame_id': 'odom',  # Now we have a proper odom frame
                'global_frame_id': 'map',
                # Topics
                'scan_topic': 'scan',
                # Update thresholds
                'update_min_d': 0.1,
                'update_min_a': 0.2,
                'transform_tolerance': 1.0,
                # Particles
                'min_particles': LaunchConfiguration('min_particles'),
                'max_particles': LaunchConfiguration('max_particles'),
                # Laser model
                'laser_model_type': 'likelihood_field',
                'laser_likelihood_max_dist': 2.0,
                'laser_max_range': 10.0,
                'laser_min_range': 0.1,
                'max_beams': LaunchConfiguration('max_beams'),
                # Motion model
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'alpha5': 0.1,
                # Initial pose (will be overridden by /initialpose topic)
                'set_initial_pose': True,
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_a': 0.0,
            }
        ]
    )
    
    # AMCL Lifecycle Manager
    amcl_lifecycle_manager = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' in ['amcl', 'ekf_amcl']"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }]
    )
    
    amcl_mode_info = LogInfo(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'amcl'"])),
        msg='Running in AMCL mode: Particle filter localization active'
    )
    
    # ==========================================================================
    # Mode: ekf - EKF odometry fusion only (robot_localization)
    # ==========================================================================
    
    ekf_mode_info = LogInfo(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'ekf'"])),
        msg='Running in EKF mode: Extended Kalman Filter odometry fusion'
    )
    
    # EKF node (robot_localization package)
    # TODO: Configure when IMU/wheel encoders are available
    ekf_node = Node(
        condition=IfCondition(PythonExpression(["'", mode, "' in ['ekf', 'ekf_amcl']"])),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_dir, 'config', 'ekf_params.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # ==========================================================================
    # Mode: ekf_amcl - Full localization stack
    # ==========================================================================
    
    ekf_amcl_mode_info = LogInfo(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'ekf_amcl'"])),
        msg='Running in EKF+AMCL mode: Full localization stack'
    )
    
    # ==========================================================================
    # Mode: scan_match - Scan matching (future)
    # ==========================================================================
    
    scan_match_mode_info = LogInfo(
        condition=IfCondition(PythonExpression(["'", mode, "' == 'scan_match'"])),
        msg='Running in SCAN_MATCH mode: ICP/NDT scan matching (not yet implemented)'
    )
    
    return LaunchDescription([
        # Arguments
        declare_mode,
        declare_is_sim,
        declare_use_sim_time,
        declare_map_name,
        declare_min_particles,
        declare_max_particles,
        declare_max_beams,
        
        # Info messages
        sim_mode_info,
        amcl_mode_info,
        ekf_mode_info,
        ekf_amcl_mode_info,
        scan_match_mode_info,
        
        # Static transforms for simulation
        odom_static_tf,
        odom_ego_static_tf,
        
        # AMCL
        amcl_node,
        amcl_lifecycle_manager,
        
        # EKF (disabled for now until config is ready)
        # ekf_node,
    ])
