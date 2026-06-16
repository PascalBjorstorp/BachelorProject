"""
Hokuyo UST-10LX Ethernet LiDAR Launch File

Uses a custom SCIP 2.0 driver for continuous streaming at the full 40 Hz
sensor rate (urg_node is limited to ~20 Hz due to synchronous protocol).

All hardware defaults are in config/hokuyo_ust10lx.yaml.

Requirements:
  - Network configured: Jetson must be on same subnet as LiDAR
  
Network Setup (run once):
  sudo ip addr add 192.168.10.15/24 dev eth0
  ping 192.168.10.10  # Test connectivity

Usage:
  ros2 launch f1tenth_lidar hokuyo_lidar.launch.py
  ros2 launch f1tenth_lidar hokuyo_lidar.launch.py ip_address:=192.168.10.10
  ros2 launch f1tenth_lidar hokuyo_lidar.launch.py driver:=urg_node  # fallback to urg_node
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_lidar')
    config_path = os.path.join(pkg_dir, 'config', 'hokuyo_ust10lx.yaml')
    
    declare_ip_address = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.10.10',
        description='IP address of the Hokuyo UST-10LX'
    )
    
    declare_scan_topic = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan',
        description='Topic to publish laser scans'
    )

    declare_driver = DeclareLaunchArgument(
        'driver',
        default_value='scip',
        description='Driver to use: "scip" for 40 Hz SCIP 2.0 driver, "urg_node" for standard driver (~20 Hz)'
    )

    use_scip = PythonExpression(["'", LaunchConfiguration('driver'), "' == 'scip'"])
    
    info_msg = LogInfo(
        msg=['Starting Hokuyo UST-10LX driver at IP: ', LaunchConfiguration('ip_address'),
             ' (driver: ', LaunchConfiguration('driver'), ')']
    )
    
    # Custom SCIP 2.0 driver — full 40 Hz continuous streaming (C++)
    scip_driver = Node(
        package='f1tenth_lidar',
        executable='hokuyo_scip_driver_node',
        name='hokuyo_scip_driver',
        output='screen',
        parameters=[
            config_path,
            {'ip_address': LaunchConfiguration('ip_address'),
             'scan_topic': LaunchConfiguration('scan_topic')},
        ],
        condition=IfCondition(use_scip),
    )

    # Fallback: standard urg_node driver (~20 Hz)
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
        condition=UnlessCondition(use_scip),
    )
    
    # Static TF: base_link → laser (physical mounting position)
    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='laser_tf_publisher',
        arguments=[
            '--x', '0.265',
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
        declare_driver,
        info_msg,
        scip_driver,
        urg_node,
        laser_tf,
    ])
