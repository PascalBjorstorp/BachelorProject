"""
F1TENTH Jetson Full Stack Launch File

Launches the complete localization stack for the real F1Tenth car on Jetson:
  1. Hokuyo UST-10LX LiDAR driver (publishes to /scan_raw)
  2. Scan filter node (clips range, downsamples 1080→270 pts, publishes to /scan)
  3. Nav2 AMCL localization
  4. Performance monitor (optional)

Network Setup (run once before launching):
  sudo ip addr add 192.168.0.15/24 dev eth0
  
Usage:
  # Full stack with default settings
  ros2 launch f1tenth_localization jetson_localization.launch.py
  
  # With custom AMCL particles
  ros2 launch f1tenth_localization jetson_localization.launch.py min_particles:=200 max_particles:=1000
  
  # With performance monitoring
  ros2 launch f1tenth_localization jetson_localization.launch.py enable_monitor:=true
  
  # Custom LiDAR IP
  ros2 launch f1tenth_localization jetson_localization.launch.py lidar_ip:=192.168.1.10
  
  # Disable scan filtering (raw 1080 points to AMCL)
  ros2 launch f1tenth_localization jetson_localization.launch.py enable_filter:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    # LiDAR arguments
    declare_lidar_ip = DeclareLaunchArgument(
        'lidar_ip',
        default_value='192.168.0.10',
        description='IP address of the Hokuyo UST-10LX'
    )
    
    declare_laser_frame = DeclareLaunchArgument(
        'laser_frame_id',
        default_value='laser',
        description='TF frame for laser data'
    )
    
    # Scan filter arguments
    declare_enable_filter = DeclareLaunchArgument(
        'enable_filter',
        default_value='true',
        description='Enable scan filtering (downsampling + range clipping)'
    )
    
    declare_range_min = DeclareLaunchArgument(
        'range_min',
        default_value='0.1',
        description='Minimum valid range in meters'
    )
    
    declare_range_max = DeclareLaunchArgument(
        'range_max',
        default_value='10.0',
        description='Maximum valid range in meters'
    )
    
    declare_downsample = DeclareLaunchArgument(
        'downsample_factor',
        default_value='4',
        description='Downsample factor (4 = 1080->270 points for 1° resolution)'
    )
    
    # AMCL arguments
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
    
    # Monitoring
    declare_enable_monitor = DeclareLaunchArgument(
        'enable_monitor',
        default_value='false',
        description='Enable performance monitoring'
    )
    
    # =========================================================================
    # Info Messages
    # =========================================================================
    info_msg = LogInfo(
        msg='Starting F1Tenth Jetson Localization Stack'
    )
    
    # =========================================================================
    # LiDAR Driver (with filter enabled: publishes to /scan_raw)
    # =========================================================================
    lidar_launch_with_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'hokuyo_lidar.launch.py')
        ),
        launch_arguments={
            'ip_address': LaunchConfiguration('lidar_ip'),
            'laser_frame_id': LaunchConfiguration('laser_frame_id'),
            'scan_topic': '/scan_raw',  # Filter will republish to /scan
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_filter')),
    )
    
    # LiDAR Driver (with filter disabled: publishes directly to /scan)
    lidar_launch_no_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'hokuyo_lidar.launch.py')
        ),
        launch_arguments={
            'ip_address': LaunchConfiguration('lidar_ip'),
            'laser_frame_id': LaunchConfiguration('laser_frame_id'),
            'scan_topic': '/scan',  # Direct to AMCL
        }.items(),
        condition=UnlessCondition(LaunchConfiguration('enable_filter')),
    )
    
    # =========================================================================
    # Scan Filter (downsamples 1080 → 270 points, clips range, removes shadows)
    # =========================================================================
    scan_filter_node = Node(
        package='f1tenth_localization',
        executable='scan_filter.py',
        name='scan_filter',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_filter')),
        parameters=[{
            'input_topic': '/scan_raw',
            'output_topic': '/scan',
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'downsample_factor': LaunchConfiguration('downsample_factor'),
            'shadow_filter_enabled': True,
            'shadow_filter_threshold': 0.3,  # meters - min range jump to detect edge
            'shadow_filter_window': 1,       # points to invalidate around edges
        }],
    )
    
    # =========================================================================
    # AMCL Localization
    # =========================================================================
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'amcl.launch.py')
        ),
        launch_arguments={
            'min_particles': LaunchConfiguration('min_particles'),
            'max_particles': LaunchConfiguration('max_particles'),
            'max_beams': LaunchConfiguration('max_beams'),
            'use_sim_time': 'false',
        }.items(),
    )
    
    # =========================================================================
    # Performance Monitor (Optional)
    # =========================================================================
    monitor_node = Node(
        package='f1tenth_localization',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_monitor')),
        parameters=[{
            'amcl_type': 'nav2_amcl',
            'min_particles': LaunchConfiguration('min_particles'),
            'max_particles': LaunchConfiguration('max_particles'),
            'max_beams': LaunchConfiguration('max_beams'),
            'output_dir': '/tmp/f1tenth_performance',
        }],
    )
    
    return LaunchDescription([
        # Arguments
        declare_lidar_ip,
        declare_laser_frame,
        declare_enable_filter,
        declare_range_min,
        declare_range_max,
        declare_downsample,
        declare_min_particles,
        declare_max_particles,
        declare_max_beams,
        declare_enable_monitor,
        
        # Info
        info_msg,
        
        # LiDAR driver (conditional based on filter setting)
        lidar_launch_with_filter,
        lidar_launch_no_filter,
        
        # Scan filter (only when enabled)
        scan_filter_node,
        
        # AMCL
        amcl_launch,
        
        # Optional performance monitor
        monitor_node,
    ])
