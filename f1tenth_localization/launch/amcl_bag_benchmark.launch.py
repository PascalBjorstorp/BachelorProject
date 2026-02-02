"""
AMCL Bag Playback Benchmark Launch File

This launch file runs AMCL with bag playback for benchmarking on Jetson.
It eliminates network overhead by playing recorded data locally.

Usage:
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py bag_path:=/path/to/bag

  # With custom particle count
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    bag_path:=/path/to/bag min_particles:=500 max_particles:=2000

  # With custom scan rate filtering (default 40Hz)
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    bag_path:=/path/to/bag scan_rate_hz:=40.0

Requirements:
  - ROS 2 bag recorded with: /scan, /map, /tf, /tf_static, /ego_racecar/odom
  - Bag must be recorded with sim.yaml tf_frame_id='odom'
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # ==================== Arguments ====================
    
    # Bag path (required)
    declare_bag_path = DeclareLaunchArgument(
        'bag_path',
        description='Path to the ROS 2 bag directory to play'
    )
    
    # Bag playback options
    declare_loop = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop bag playback'
    )
    
    declare_rate = DeclareLaunchArgument(
        'playback_rate',
        default_value='1.0',
        description='Bag playback rate multiplier'
    )
    
    # AMCL particle parameters
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
        default_value='120',
        description='AMCL max laser beams to use'
    )
    
    # AMCL update thresholds
    declare_update_min_d = DeclareLaunchArgument(
        'update_min_d',
        default_value='0.001',
        description='Min translation (m) before filter update'
    )
    
    declare_update_min_a = DeclareLaunchArgument(
        'update_min_a',
        default_value='0.001',
        description='Min rotation (rad) before filter update'
    )
    
    # Scan rate filtering (to simulate real hardware)
    declare_scan_rate = DeclareLaunchArgument(
        'scan_rate_hz',
        default_value='40.0',
        description='Target scan rate in Hz (use 0 for no filtering)'
    )
    
    # Performance monitor
    declare_enable_monitor = DeclareLaunchArgument(
        'enable_monitor',
        default_value='true',
        description='Enable performance monitoring'
    )
    
    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='/tmp/f1tenth_benchmark',
        description='Output directory for performance logs'
    )
    
    # Get launch configurations
    bag_path = LaunchConfiguration('bag_path')
    loop = LaunchConfiguration('loop')
    playback_rate = LaunchConfiguration('playback_rate')
    scan_rate_hz = LaunchConfiguration('scan_rate_hz')
    enable_monitor = LaunchConfiguration('enable_monitor')
    output_dir = LaunchConfiguration('output_dir')
    
    # ==================== Info Messages ====================
    
    info_msg = LogInfo(
        msg=['Starting AMCL Bag Benchmark with bag: ', bag_path]
    )
    
    particle_info = LogInfo(
        msg=['Particles: ', LaunchConfiguration('min_particles'), 
             ' - ', LaunchConfiguration('max_particles'),
             ', Beams: ', LaunchConfiguration('max_beams')]
    )
    
    # ==================== Bag Playback ====================
    
    # Build bag play command with optional loop
    # Using --clock to publish /clock for use_sim_time
    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', playback_rate,
            '--loop',  # Always loop, control externally if needed
        ],
        output='screen',
        name='bag_player'
    )
    
    # ==================== Scan Rate Throttler ====================
    
    # Throttle /scan to simulate real 40Hz LiDAR
    # This republishes /scan to /scan_throttled at the target rate
    scan_throttler = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttler',
        arguments=[
            'messages', '/scan', scan_rate_hz, '/scan_throttled'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=PythonExpression(['"', scan_rate_hz, '" != "0"'])
    )
    
    # ==================== AMCL Node ====================
    
    # Determine which scan topic to use based on throttling
    # When scan_rate > 0, use throttled topic
    amcl_scan_topic = PythonExpression([
        '"/scan_throttled" if float("', scan_rate_hz, '") > 0 else "/scan"'
    ])
    
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='/',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Must be True for bag playback
            # Frame IDs
            'base_frame_id': 'ego_racecar/base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            # Topics - use throttled scan if rate limiting enabled
            'scan_topic': '/scan_throttled',  # Will be remapped if needed
            # Update thresholds
            'update_min_d': LaunchConfiguration('update_min_d'),
            'update_min_a': LaunchConfiguration('update_min_a'),
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
            # Initial pose
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_a': 0.0,
        }],
        remappings=[
            ('/scan', '/scan_throttled'),  # Use throttled scan
        ]
    )
    
    # AMCL Lifecycle Manager
    amcl_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }]
    )
    
    # ==================== Performance Monitor ====================
    
    performance_monitor = Node(
        package='f1tenth_localization',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'output_dir': output_dir,
            'sample_rate_hz': 100.0,
            'scan_topic': '/scan_throttled',
            'amcl_pose_topic': '/amcl_pose',
        }],
    )
    
    # ==================== Launch Description ====================
    
    return LaunchDescription([
        # Arguments
        declare_bag_path,
        declare_loop,
        declare_rate,
        declare_min_particles,
        declare_max_particles,
        declare_max_beams,
        declare_update_min_d,
        declare_update_min_a,
        declare_scan_rate,
        declare_enable_monitor,
        declare_output_dir,
        
        # Info
        info_msg,
        particle_info,
        
        # Bag playback first
        bag_play_cmd,
        
        # Scan throttler (wait for bag to start)
        TimerAction(
            period=2.0,
            actions=[scan_throttler]
        ),
        
        # AMCL (wait for throttler)
        TimerAction(
            period=3.0,
            actions=[amcl_node, amcl_lifecycle]
        ),
        
        # Performance monitor (wait for AMCL)
        TimerAction(
            period=5.0,
            actions=[performance_monitor]
        ),
    ])
