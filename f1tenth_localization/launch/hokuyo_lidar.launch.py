"""
Hokuyo UST-10LX Ethernet LiDAR Launch File

Launches the urg_node2 driver for the UST-10LX with proper configuration.

Requirements:
  - urg_node2 package: sudo apt install ros-jazzy-urg-node
  - Network configured: Jetson must be on same subnet as LiDAR
  
Network Setup (run once):
  sudo ip addr add 192.168.0.15/24 dev eth0
  ping 192.168.0.10  # Test connectivity

Usage:
  # With default settings
  ros2 launch f1tenth_localization hokuyo_lidar.launch.py
  
  # With custom IP
  ros2 launch f1tenth_localization hokuyo_lidar.launch.py ip_address:=192.168.1.10
  
  # Publish to different frame
  ros2 launch f1tenth_localization hokuyo_lidar.launch.py laser_frame_id:=ego_racecar/laser
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('f1tenth_localization')
    
    # Declare arguments
    declare_ip_address = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.10',
        description='IP address of the Hokuyo UST-10LX'
    )
    
    declare_ip_port = DeclareLaunchArgument(
        'ip_port',
        default_value='10940',
        description='Port number for the Hokuyo UST-10LX'
    )
    
    declare_laser_frame_id = DeclareLaunchArgument(
        'laser_frame_id',
        default_value='laser',
        description='TF frame ID for laser data'
    )
    
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic to publish laser scans'
    )
    
    declare_angle_min = DeclareLaunchArgument(
        'angle_min',
        default_value='-2.356194',
        description='Minimum scan angle in radians (-135 deg)'
    )
    
    declare_angle_max = DeclareLaunchArgument(
        'angle_max',
        default_value='2.356194',
        description='Maximum scan angle in radians (+135 deg)'
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
    
    # Info message
    info_msg = LogInfo(
        msg=['Starting Hokuyo UST-10LX driver at IP: ', LaunchConfiguration('ip_address')]
    )
    
    # URG Node for Hokuyo LiDAR
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[{
            'ip_address': LaunchConfiguration('ip_address'),
            'ip_port': LaunchConfiguration('ip_port'),
            'laser_frame_id': LaunchConfiguration('laser_frame_id'),
            'angle_min': LaunchConfiguration('angle_min'),
            'angle_max': LaunchConfiguration('angle_max'),
            'range_min': LaunchConfiguration('range_min'),
            'range_max': LaunchConfiguration('range_max'),
            'publish_intensity': False,
            'publish_multiecho': False,
            'cluster': 1,
            'skip': 1,
            'calibrate_time': False,
        }],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    )
    
    # Static transform from base_link to laser (adjust x, y, z, yaw for your setup)
    # This should match your physical mounting position
    # Default: laser mounted 0.275m forward of base_link, facing forward
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=[
            '--x', '0.275',      # Forward offset (meters)
            '--y', '0.0',        # Lateral offset (meters)  
            '--z', '0.05',       # Height offset (meters)
            '--roll', '0.0',     # Roll (radians)
            '--pitch', '0.0',    # Pitch (radians)
            '--yaw', '0.0',      # Yaw (radians)
            '--frame-id', 'ego_racecar/base_link',
            '--child-frame-id', LaunchConfiguration('laser_frame_id'),
        ],
    )
    
    return LaunchDescription([
        # Arguments
        declare_ip_address,
        declare_ip_port,
        declare_laser_frame_id,
        declare_scan_topic,
        declare_angle_min,
        declare_angle_max,
        declare_range_min,
        declare_range_max,
        
        # Info
        info_msg,
        
        # Nodes
        urg_node,
        laser_tf,
    ])
