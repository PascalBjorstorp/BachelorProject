"""
F1TENTH Real Robot Localization Launch File

Launches the full localization stack for the real F1Tenth car:
  1. Hokuyo UST-10LX LiDAR driver (270 pts via cluster=4)
  2. Map server (loads static map from YAML)
  3. Nav2 AMCL localization (params from nav2_amcl_params.yaml)
  4. Performance monitor (optional)

Network Setup (run once before launching):
  sudo ip addr add 192.168.0.15/24 dev eth0

Usage:
  ros2 launch f1tenth_localization real_localization.launch.py
  ros2 launch f1tenth_localization real_localization.launch.py min_particles:=200 max_particles:=1000
  ros2 launch f1tenth_localization real_localization.launch.py enable_monitor:=true
  ros2 launch f1tenth_localization real_localization.launch.py lidar_ip:=192.168.1.10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

# Derive workspace root for map path default
_pkg_share = get_package_share_directory('f1tenth_localization')
_workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_pkg_share))))


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')

    # Default map: Spielberg from f1tenth_sim/maps
    default_map = os.path.join(_workspace_root, 'f1tenth_sim', 'maps', 'Spielberg_map.yaml')

    # =========================================================================
    # Launch Arguments
    # =========================================================================

    declare_lidar_ip = DeclareLaunchArgument(
        'lidar_ip', default_value='192.168.0.10',
        description='IP address of the Hokuyo UST-10LX'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file', default_value=default_map,
        description='Path to the map YAML file for map_server'
    )

    declare_min_particles = DeclareLaunchArgument(
        'min_particles', default_value='500',
        description='AMCL minimum particles'
    )

    declare_max_particles = DeclareLaunchArgument(
        'max_particles', default_value='2000',
        description='AMCL maximum particles'
    )

    declare_max_beams = DeclareLaunchArgument(
        'max_beams', default_value='120',
        description='AMCL max laser beams to use'
    )

    declare_enable_monitor = DeclareLaunchArgument(
        'enable_monitor', default_value='false',
        description='Enable performance monitoring'
    )

    # =========================================================================
    # Nodes
    # =========================================================================

    info_msg = LogInfo(msg='Starting F1Tenth Real Robot Localization Stack')

    # 1) LiDAR driver — cluster=4 and range filtering handled in YAML config
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'hokuyo_lidar.launch.py')
        ),
        launch_arguments={
            'ip_address': LaunchConfiguration('lidar_ip'),
        }.items(),
    )

    # 2) Map server — serves the static map to AMCL
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='/',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map_file'),
        }],
    )

    # 3) AMCL localization — params from YAML, particles/beams overridden by launch args
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='/',
        output='screen',
        parameters=[
            amcl_params_file,
            {
                'use_sim_time': False,
                'min_particles': LaunchConfiguration('min_particles'),
                'max_particles': LaunchConfiguration('max_particles'),
                'max_beams': LaunchConfiguration('max_beams'),
            },
        ],
    )

    # Lifecycle manager — auto-activates map_server and AMCL
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': 0.0,
        }],
    )

    # 4) Performance monitor (optional)
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
        }],
    )

    return LaunchDescription([
        declare_lidar_ip,
        declare_map_file,
        declare_min_particles,
        declare_max_particles,
        declare_max_beams,
        declare_enable_monitor,
        info_msg,
        lidar_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager,
        monitor_node,
    ])
