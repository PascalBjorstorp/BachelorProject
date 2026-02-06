"""
AMCL Bag Playback Benchmark Launch File

This launch file runs AMCL with bag playback for benchmarking on Jetson.
It eliminates network overhead by playing recorded data locally.

Usage:
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py bag_path:=/path/to/bag

  # With custom particle count
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    bag_path:=/path/to/bag min_particles:=500 max_particles:=2000

Requirements:
  - ROS 2 bag recorded with: /scan, /map, /tf, /tf_static, /ego_racecar/odom
  - Bag must be recorded with sim.yaml tf_frame_id='odom'
  - Use ABSOLUTE paths (not ~/)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, 
    OpaqueFunction, RegisterEventHandler, Shutdown
)
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

# ==================== Default Configuration ====================
# Particle filter settings
DEFAULT_MIN_PARTICLES = 500
DEFAULT_MAX_PARTICLES = 2000
DEFAULT_MAX_BEAMS = 120

# AMCL type identifier (for comparison in plots)
AMCL_TYPE = 'nav2_amcl'  # e.g., 'nav2_amcl', 'gpu_amcl', 'custom_amcl'

# Performance monitor
SAMPLE_RATE_HZ = 20.0  # How often to sample CPU/memory/GPU metrics

# Frame IDs
BASE_FRAME_ID = 'ego_racecar/base_link'
ODOM_FRAME_ID = 'odom'
GLOBAL_FRAME_ID = 'map'

# Topics
SCAN_TOPIC = '/scan'
ODOM_TOPIC = '/ego_racecar/odom'
AMCL_POSE_TOPIC = '/amcl_pose'


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    # Get resolved arguments
    bag_path = LaunchConfiguration('bag_path').perform(context)
    playback_rate = LaunchConfiguration('playback_rate').perform(context)
    min_particles = LaunchConfiguration('min_particles').perform(context)
    max_particles = LaunchConfiguration('max_particles').perform(context)
    max_beams = LaunchConfiguration('max_beams').perform(context)
    update_min_d = LaunchConfiguration('update_min_d').perform(context)
    update_min_a = LaunchConfiguration('update_min_a').perform(context)
    output_dir = LaunchConfiguration('output_dir').perform(context)
    
    # Expand ~ in bag path
    bag_path = os.path.expanduser(bag_path)
    output_dir = os.path.expanduser(output_dir)
    
    # ==================== Bag Playback ====================
    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', playback_rate,
        ],
        output='screen',
        name='bag_player'
    )
    
    # ==================== Odom TF Publisher ====================
    # Publishes odom -> base_link TF from odometry messages
    # This allows AMCL to work with bags recorded in ground truth mode
    odom_tf_publisher = Node(
        package='f1tenth_localization',
        executable='odom_tf_publisher.py',
        name='odom_tf_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'odom_topic': ODOM_TOPIC,
            'odom_frame': ODOM_FRAME_ID,
            'base_frame': BASE_FRAME_ID,
        }],
    )
    
    # ==================== AMCL Node ====================
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='/',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            # Frame IDs
            'base_frame_id': BASE_FRAME_ID,
            'odom_frame_id': ODOM_FRAME_ID,
            'global_frame_id': GLOBAL_FRAME_ID,
            # Topics
            'scan_topic': SCAN_TOPIC,
            # Update thresholds
            'update_min_d': float(update_min_d),
            'update_min_a': float(update_min_a),
            'transform_tolerance': 1.0,
            # Particles
            'min_particles': int(min_particles),
            'max_particles': int(max_particles),
            # Laser model
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 10.0,
            'laser_min_range': 0.1,
            'max_beams': int(max_beams),
            # Motion model (Differential - closest to Ackermann steering)
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.1,  # rotation from rotation
            'alpha2': 0.1,  # rotation from translation
            'alpha3': 0.2,  # translation from translation
            'alpha4': 0.2,  # translation from rotation
            # Initial pose
            'set_initial_pose': True,
            'initial_pose_x': 0.0,
            'initial_pose_y': 0.0,
            'initial_pose_a': 0.0,
            # Stability improvements (reduce KD-tree crash on convergence)
            'force_update_after_initialpose': True,
            'resample_interval': 2,     # Resample every 2nd update (reduces clustering)
            'pf_err': 0.02,             # Tighter KLD sampling
            'pf_z': 0.999,              # Tighter KLD (99.9% confidence)
            'first_map_only': True,     # Don't reload map
        }]
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
            'sample_rate_hz': SAMPLE_RATE_HZ,
            'scan_topic': SCAN_TOPIC,
            'amcl_pose_topic': AMCL_POSE_TOPIC,
            # Benchmark config for CSV metadata
            'amcl_type': AMCL_TYPE,
            'min_particles': int(min_particles),
            'max_particles': int(max_particles),
            'max_beams': int(max_beams),
        }],
    )
    
    # ==================== Event-Based Startup Sequence ====================
    # Order: performance_monitor -> odom_tf_publisher -> AMCL -> bag playback
    # This ensures everything is ready BEFORE data starts flowing
    
    # Start odom_tf_publisher when performance monitor starts
    start_odom_tf_on_monitor = RegisterEventHandler(
        OnProcessStart(
            target_action=performance_monitor,
            on_start=[
                LogInfo(msg='Performance monitor started, launching odom TF publisher...'),
                odom_tf_publisher,
            ]
        )
    )
    
    # Start AMCL and lifecycle manager when odom_tf_publisher starts
    start_amcl_on_odom_tf = RegisterEventHandler(
        OnProcessStart(
            target_action=odom_tf_publisher,
            on_start=[
                LogInfo(msg='Odom TF publisher started, launching AMCL...'),
                amcl_node,
                amcl_lifecycle,
            ]
        )
    )
    
    # Start bag playback LAST when AMCL starts (all nodes ready before data flows)
    start_bag_on_amcl = RegisterEventHandler(
        OnProcessStart(
            target_action=amcl_node,
            on_start=[
                LogInfo(msg='AMCL started, beginning bag playback...'),
                bag_play_cmd,
            ]
        )
    )
    
    # Shutdown everything when bag playback finishes
    shutdown_on_bag_end = RegisterEventHandler(
        OnProcessExit(
            target_action=bag_play_cmd,
            on_exit=[
                LogInfo(msg='Bag playback finished, shutting down...'),
                Shutdown(reason='Bag playback completed'),
            ]
        )
    )
    
    return [
        # Start performance monitor FIRST
        performance_monitor,
        # Event handlers for sequenced startup
        start_odom_tf_on_monitor,
        start_amcl_on_odom_tf,
        start_bag_on_amcl,
        # Shutdown handler
        shutdown_on_bag_end,
    ]


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription([
        # ==================== Arguments ====================
        DeclareLaunchArgument(
            'bag_path',
            description='Path to the ROS 2 bag directory to play (use absolute path)'
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Bag playback rate multiplier'
        ),
        DeclareLaunchArgument(
            'min_particles',
            default_value=str(DEFAULT_MIN_PARTICLES),
            description='AMCL minimum particles'
        ),
        DeclareLaunchArgument(
            'max_particles',
            default_value=str(DEFAULT_MAX_PARTICLES),
            description='AMCL maximum particles'
        ),
        DeclareLaunchArgument(
            'max_beams',
            default_value=str(DEFAULT_MAX_BEAMS),
            description='AMCL max laser beams to use'
        ),
        DeclareLaunchArgument(
            'update_min_d',
            default_value='0.001',
            description='Min translation (m) before filter update'
        ),
        DeclareLaunchArgument(
            'update_min_a',
            default_value='0.001',
            description='Min rotation (rad) before filter update'
        ),
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/f1tenth_benchmark',
            description='Output directory for performance logs'
        ),
        
        # ==================== Info Messages ====================
        LogInfo(msg=['Starting AMCL Bag Benchmark with bag: ', LaunchConfiguration('bag_path')]),
        LogInfo(msg=['Particles: ', LaunchConfiguration('min_particles'), 
                     ' - ', LaunchConfiguration('max_particles'),
                     ', Beams: ', LaunchConfiguration('max_beams')]),
        
        # ==================== Launch Setup ====================
        OpaqueFunction(function=launch_setup),
    ])
