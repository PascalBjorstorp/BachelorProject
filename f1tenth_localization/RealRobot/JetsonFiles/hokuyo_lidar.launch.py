"""
Hokuyo UST-10LX Ethernet LiDAR Launch File

Launches the urg_node2 driver for the UST-10LX.
All hardware defaults are in config/hokuyo_ust10lx.yaml.

Requirements:
  - urg_node2 package: sudo apt install ros-jazzy-urg-node
  - Network configured: Jetson must be on same subnet as LiDAR
  
Network Setup (run once):
  sudo ip addr add 192.168.0.15/24 dev eth0
  ping 192.168.0.10  # Test connectivity

Usage:
  ros2 launch f1tenth_localization hokuyo_lidar.launch.py
  ros2 launch f1tenth_localization hokuyo_lidar.launch.py ip_address:=192.168.1.10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    config_path = os.path.join(pkg_dir, 'config', 'hokuyo_ust10lx.yaml')
    
    # Only expose arguments that change between deployments
    declare_ip_address = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.0.10',
        description='IP address of the Hokuyo UST-10LX'
    )
    
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic to publish laser scans'
    )
    
    info_msg = LogInfo(
        msg=['Starting Hokuyo UST-10LX driver at IP: ', LaunchConfiguration('ip_address')]
    )
    
    # URG Node — loads defaults from YAML, overrides IP from launch arg
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        output='screen',
        parameters=[
            config_path,
            {'ip_address': LaunchConfiguration('ip_address')},
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    )
    
    # Static TF: base_link → laser (physical mounting position)
    # laser_frame_id comes from the YAML (ego_racecar/laser)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=[
            '--x', '0.275',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'ego_racecar/base_link',
            '--child-frame-id', 'ego_racecar/laser',
        ],
    )
    
    return LaunchDescription([
        declare_ip_address,
        declare_scan_topic,
        info_msg,
        urg_node,
        laser_tf,
    ])
