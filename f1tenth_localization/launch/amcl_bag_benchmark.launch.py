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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


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
    
    nodes = []
    
    # ==================== Bag Playback ====================
    bag_play_cmd = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', bag_path,
            '--clock',
            '--rate', playback_rate,
            '--loop',
        ],
        output='screen',
        name='bag_player'
    )
    nodes.append(bag_play_cmd)
    
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
            'base_frame_id': 'ego_racecar/base_link',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            # Topics
            'scan_topic': '/scan',
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
            'sample_rate_hz': 100.0,
            'scan_topic': '/scan',
            'amcl_pose_topic': '/amcl_pose',
        }],
    )
    
    # Use TimerAction to sequence startup
    return [
        # Start bag playback first
        bag_play_cmd,
        # Start AMCL after bag has time to start
        TimerAction(
            period=3.0,
            actions=[amcl_node, amcl_lifecycle]
        ),
        # Start performance monitor after AMCL
        TimerAction(
            period=5.0,
            actions=[performance_monitor]
        ),
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
            default_value='500',
            description='AMCL minimum particles'
        ),
        DeclareLaunchArgument(
            'max_particles',
            default_value='2000',
            description='AMCL maximum particles'
        ),
        DeclareLaunchArgument(
            'max_beams',
            default_value='120',
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
