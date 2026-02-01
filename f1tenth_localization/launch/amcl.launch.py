"""
F1TENTH AMCL Launch File

This launch file runs ONLY AMCL particle filter localization.
Use this on Jetson while simulation runs on PC.

IMPORTANT: The simulation must be configured to publish odom->base_link!
In f1tenth_sim/config/sim.yaml, set:
  tf_frame_id: 'odom'
  odom_frame_id: 'odom'

Usage:
  ros2 launch f1tenth_localization amcl.launch.py
  ros2 launch f1tenth_localization amcl.launch.py min_particles:=200 max_particles:=1000

Parameters:
  min_particles:  AMCL minimum particles (default: 500)
  max_particles:  AMCL maximum particles (default: 2000)
  max_beams:      AMCL max laser beams (default: 60)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    # Arguments
    declare_min_particles = DeclareLaunchArgument(
        'min_particles',
        default_value='1000',
        description='AMCL minimum particles'
    )
    
    declare_max_particles = DeclareLaunchArgument(
        'max_particles',
        default_value='5000',
        description='AMCL maximum particles'
    )
    
    declare_max_beams = DeclareLaunchArgument(
        'max_beams',
        default_value='600',
        description='AMCL max laser beams to use'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Info message
    info_msg = LogInfo(
        msg='Starting AMCL localization. Ensure simulation is running with tf_frame_id=odom!'
    )
    
    # AMCL Node
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='/',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Frame IDs
            'base_frame_id': 'ego_racecar/base_link',
            'odom_frame_id': 'odom',
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
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }]
    )
    
    return LaunchDescription([
        # Arguments
        declare_min_particles,
        declare_max_particles,
        declare_max_beams,
        declare_use_sim_time,
        
        # Info
        info_msg,
        
        # AMCL
        amcl_node,
        amcl_lifecycle,
    ])
