"""
AMCL Bag Playback Benchmark Launch File

This launch file runs AMCL with bag playback for benchmarking on Jetson.
It eliminates network overhead by playing recorded data locally.
Supports both nav2_amcl and gpu_amcl for comparison.

Usage:
  # Run with nav2_amcl (default)
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py bag_path:=/path/to/bag

  # Run with GPU AMCL
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    bag_path:=/path/to/bag amcl_type:=gpu_amcl

  # With custom particle count
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    bag_path:=/path/to/bag min_particles:=500 max_particles:=2000

  # Record output bag for Foxglove visualization
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    bag_path:=/path/to/bag record_output:=true

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
DEFAULT_MIN_PARTICLES = 100
DEFAULT_MAX_PARTICLES = 400
DEFAULT_MAX_BEAMS = 270

# AMCL type: 'nav2_amcl' or 'gpu_amcl'
DEFAULT_AMCL_TYPE = 'nav2_amcl'

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

# Topics to record for Foxglove visualization
RECORD_TOPICS = [
    '/scan',
    '/map',
    '/amcl_pose',
    '/particlecloud',
    '/tf',
    '/tf_static',
    '/ego_racecar/odom',
]


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
    amcl_type = LaunchConfiguration('amcl_type').perform(context)
    record_output = LaunchConfiguration('record_output').perform(context).lower() == 'true'
    
    # Expand ~ in bag path
    bag_path = os.path.expanduser(bag_path)
    output_dir = os.path.expanduser(output_dir)
    
    # Generate output bag path with timestamp
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_bag_path = os.path.join(
        output_dir, 
        f'amcl_output_{amcl_type}_p{min_particles}-{max_particles}_b{max_beams}_{timestamp}'
    )
    
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
    
    # ==================== AMCL Node (nav2_amcl or gpu_amcl) ====================
    if amcl_type == 'gpu_amcl':
        # GPU AMCL - Custom implementation with CuPy acceleration
        amcl_node = Node(
            package='gpu_amcl',
            executable='gpu_amcl_node.py',
            name='gpu_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                # GPU settings
                'use_gpu': True,
                'num_particles': int(max_particles),
                # Frame IDs
                'base_frame_id': BASE_FRAME_ID,
                'odom_frame_id': ODOM_FRAME_ID,
                'global_frame_id': GLOBAL_FRAME_ID,
                # Topics
                'scan_topic': SCAN_TOPIC,
                'odom_topic': ODOM_TOPIC,
                # Update thresholds
                'update_min_d': float(update_min_d),
                'update_min_a': float(update_min_a),
                'transform_tolerance': 1.0,
                # Sensor model
                'max_beams': int(max_beams),
                'laser_max_range': 10.0,
                'z_hit': 0.95,
                'z_rand': 0.05,
                'sigma_hit': 0.2,
                # Motion model
                'alpha1': 0.1,
                'alpha2': 0.1,
                'alpha3': 0.2,
                'alpha4': 0.2,
                # Initial pose
                'initial_pose_x': 0.0,
                'initial_pose_y': 0.0,
                'initial_pose_a': 0.0,
                # Publishing (40Hz for racing)
                'publish_rate': 40.0,
                # IMU fusion (disabled for bag playback without IMU)
                'use_imu_rotation': False,
            }],
        )
        # GPU AMCL doesn't need lifecycle manager
        amcl_lifecycle = None
    else:
        # nav2_amcl - Standard ROS2 AMCL
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
                # Global localization recovery ("kidnapped robot" detection)
                'recovery_alpha_slow': 0.001,   # Long-term average decay (slow)
                'recovery_alpha_fast': 0.1,     # Short-term average decay (fast)
                # ^ When fast drops below slow, particles are randomly injected
            }]
        )
        # nav2_amcl needs lifecycle manager
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
            'amcl_type': amcl_type,  # Use the selected AMCL type
            'min_particles': int(min_particles),
            'max_particles': int(max_particles),
            'max_beams': int(max_beams),
        }],
    )
    
    # ==================== Output Bag Recording (Optional) ====================
    # Records AMCL output for Foxglove visualization
    if record_output:
        bag_record_cmd = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-o', output_bag_path,
                '--use-sim-time',
            ] + RECORD_TOPICS,
            output='screen',
            name='bag_recorder'
        )
    else:
        bag_record_cmd = None
    
    # ==================== Event-Based Startup Sequence ====================
    # Order: performance_monitor -> odom_tf_publisher -> AMCL -> bag recording -> bag playback
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
    
    # Start AMCL (and lifecycle manager for nav2_amcl) when odom_tf_publisher starts
    amcl_start_actions = [
        LogInfo(msg=f'{amcl_type} starting...'),
        amcl_node,
    ]
    if amcl_lifecycle is not None:
        amcl_start_actions.append(amcl_lifecycle)
    
    start_amcl_on_odom_tf = RegisterEventHandler(
        OnProcessStart(
            target_action=odom_tf_publisher,
            on_start=amcl_start_actions
        )
    )
    
    # Start bag recording and playback when AMCL starts
    bag_start_actions = [
        LogInfo(msg='AMCL started, beginning bag playback...'),
    ]
    if bag_record_cmd is not None:
        bag_start_actions.append(LogInfo(msg=f'Recording output to: {output_bag_path}'))
        bag_start_actions.append(bag_record_cmd)
    bag_start_actions.append(bag_play_cmd)
    
    start_bag_on_amcl = RegisterEventHandler(
        OnProcessStart(
            target_action=amcl_node,
            on_start=bag_start_actions
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
            description='Output directory for performance logs and recorded bags'
        ),
        DeclareLaunchArgument(
            'amcl_type',
            default_value=DEFAULT_AMCL_TYPE,
            description="AMCL implementation: 'nav2_amcl' or 'gpu_amcl'"
        ),
        DeclareLaunchArgument(
            'record_output',
            default_value='false',
            description='Record output bag with AMCL data for Foxglove visualization'
        ),
        
        # ==================== Info Messages ====================
        LogInfo(msg=['Starting AMCL Bag Benchmark with bag: ', LaunchConfiguration('bag_path')]),
        LogInfo(msg=['AMCL Type: ', LaunchConfiguration('amcl_type')]),
        LogInfo(msg=['Particles: ', LaunchConfiguration('min_particles'), 
                     ' - ', LaunchConfiguration('max_particles'),
                     ', Beams: ', LaunchConfiguration('max_beams')]),
        LogInfo(msg=['Record output: ', LaunchConfiguration('record_output')]),
        
        # ==================== Launch Setup ====================
        OpaqueFunction(function=launch_setup),
    ])
