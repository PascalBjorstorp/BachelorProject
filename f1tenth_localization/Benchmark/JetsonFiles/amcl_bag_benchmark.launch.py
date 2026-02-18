"""
AMCL Bag Playback Benchmark Launch File

This launch file runs AMCL with bag playback for benchmarking on Jetson.
It eliminates network overhead by playing recorded data locally.
Supports both nav2_amcl and gpu_amcl for comparison.

Usage:
  # Run with nav2_amcl (default)
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py

  # Run with GPU AMCL
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py amcl_type:=gpu_amcl

  # With custom particle count
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
    min_particles:=500 max_particles:=2000

  # Record output bag for Foxglove visualization
  ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py record_output:=true

Requirements:
  - ROS 2 bag recorded with: /scan, /map, /tf, /tf_static, /ego_racecar/odom
  - Bag must be recorded with sim.yaml tf_frame_id='odom'
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
from ament_index_python.packages import get_package_share_directory

# Derive colcon workspace root from the installed package location
# install/<pkg>/share/<pkg>/ → 4 levels up → workspace root
_pkg_share = get_package_share_directory('f1tenth_localization')
_workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_pkg_share))))

# ==================== Default Configuration ====================
# Particle filter settings
DEFAULT_MIN_PARTICLES = 100 
DEFAULT_MAX_PARTICLES = 3000
DEFAULT_MAX_BEAMS = 270

# AMCL type: 'nav2_amcl' or 'gpu_amcl'
DEFAULT_AMCL_TYPE = 'gpu_amcl'

# Performance monitor
SAMPLE_RATE_HZ = 5.0  # How often to sample CPU/memory/GPU metrics (sim Hz; ~7.5 Hz wall at 1.5x sim speed)

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
    min_particles = LaunchConfiguration('min_particles').perform(context)
    max_particles = LaunchConfiguration('max_particles').perform(context)
    max_beams = LaunchConfiguration('max_beams').perform(context)
    update_min_d = LaunchConfiguration('update_min_d').perform(context)
    update_min_a = LaunchConfiguration('update_min_a').perform(context)
    csv_output_dir = LaunchConfiguration('csv_output_dir').perform(context)
    bag_output_dir = LaunchConfiguration('bag_output_dir').perform(context)
    amcl_type = LaunchConfiguration('amcl_type').perform(context)
    record_output = LaunchConfiguration('record_output').perform(context).lower() == 'true'
    
    # Expand ~ in paths
    bag_path = os.path.expanduser(bag_path)
    csv_output_dir = os.path.expanduser(csv_output_dir)
    bag_output_dir = os.path.expanduser(bag_output_dir)
    
    # AMCL params YAML
    pkg_dir = get_package_share_directory('f1tenth_localization')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')
    gpu_amcl_params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_params.yaml')
    
    # Generate output bag path with timestamp
    from datetime import datetime
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_bag_path = os.path.join(
        bag_output_dir, 
        f'amcl_output_{amcl_type}_p{min_particles}-{max_particles}_b{max_beams}_{timestamp}'
    )
    
    # ==================== Bag Playback ====================
    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
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
        # GPU AMCL - params from YAML, overrides from launch args
        amcl_node = Node(
            package='f1tenth_localization',
            executable='gpu_amcl_node.py',
            name='gpu_amcl',
            output='screen',
            parameters=[
                gpu_amcl_params_file,
                {
                    'use_sim_time': True,
                    'num_particles': int(max_particles),
                    'max_beams': int(max_beams),
                    'update_min_d': float(update_min_d),
                    'update_min_a': float(update_min_a),
                    'use_kld_sampling': LaunchConfiguration('use_kld').perform(context).lower() == 'true',
                    # Disable IMU for bag playback (no IMU data in bags)
                    'use_imu_rotation': False,
                },
            ],
        )
        # GPU AMCL doesn't need lifecycle manager
        amcl_lifecycle = None
    else:
        # nav2_amcl - Standard ROS2 AMCL (params from YAML, overrides from launch args)
        amcl_node = LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='/',
            output='screen',
            parameters=[
                amcl_params_file,
                {
                    'use_sim_time': True,
                    # Override from launch args
                    'min_particles': int(min_particles),
                    'max_particles': int(max_particles),
                    'max_beams': int(max_beams),
                    'update_min_d': float(update_min_d),
                    'update_min_a': float(update_min_a),
                },
            ]
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
            'output_dir': csv_output_dir,
            'sample_rate_hz': SAMPLE_RATE_HZ,
            'scan_topic': SCAN_TOPIC,
            'amcl_pose_topic': AMCL_POSE_TOPIC,
            # Benchmark config for CSV metadata
            'amcl_type': amcl_type,
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
            default_value=os.path.join(_workspace_root, 'f1tenth_localization', 'Benchmark', 'bags', 'lapBags'),
            description='Path to the ROS 2 bag directory to play'
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
            'csv_output_dir',
            default_value=os.path.join(_workspace_root, 'f1tenth_localization', 'Benchmark', 'Matlab', 'csv'),
            description='Output directory for performance CSV logs'
        ),
        DeclareLaunchArgument(
            'bag_output_dir',
            default_value=os.path.join(_workspace_root, 'f1tenth_localization', 'Benchmark', 'bags', 'benchmarkBags'),
            description='Output directory for recorded output bags'
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
        DeclareLaunchArgument(
            'use_kld',
            default_value='false',
            description='Enable KLD sampling for adaptive particle count (GPU AMCL only)'
        ),
        
        # ==================== Info Messages ====================
        LogInfo(msg=['Starting AMCL Bag Benchmark with bag: ', LaunchConfiguration('bag_path')]),
        LogInfo(msg=['AMCL Type: ', LaunchConfiguration('amcl_type')]),
        LogInfo(msg=['Particles: ', LaunchConfiguration('min_particles'), 
                     ' - ', LaunchConfiguration('max_particles'),
                     ', Beams: ', LaunchConfiguration('max_beams')]),
        LogInfo(msg=['Record output: ', LaunchConfiguration('record_output')]),
        LogInfo(msg=['KLD sampling: ', LaunchConfiguration('use_kld')]),
        
        # ==================== Launch Setup ====================
        OpaqueFunction(function=launch_setup),
    ])
