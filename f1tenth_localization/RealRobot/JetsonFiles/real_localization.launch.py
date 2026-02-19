"""
F1TENTH Real Robot Localization Launch File

Launches the full localization stack for the real F1Tenth car:
  1. Hokuyo UST-10LX LiDAR driver (270 pts via cluster=4)
  2. Map server (loads static map from YAML)
  3. Localization: nav2_amcl (default) or gpu_amcl
  4. Performance monitor (optional)

Network Setup (run once before launching):
  sudo ip addr add 192.168.0.15/24 dev eth0

Usage:
  ros2 launch f1tenth_localization real_localization.launch.py
  ros2 launch f1tenth_localization real_localization.launch.py amcl_type:=gpu_amcl
  ros2 launch f1tenth_localization real_localization.launch.py min_particles:=200 max_particles:=1000
  ros2 launch f1tenth_localization real_localization.launch.py enable_monitor:=true
  ros2 launch f1tenth_localization real_localization.launch.py lidar_ip:=192.168.1.10
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode

# Derive workspace root for map path default
_pkg_share = get_package_share_directory('f1tenth_localization')
_workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(_pkg_share))))

# Frame / topic constants (must match real robot URDF and drivers)
BASE_FRAME_ID = 'ego_racecar/base_link'
ODOM_FRAME_ID = 'ego_racecar/odom'
GLOBAL_FRAME_ID = 'map'
SCAN_TOPIC = '/scan'
ODOM_TOPIC = '/ego_racecar/odom'


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    pkg_dir = get_package_share_directory('f1tenth_localization')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')
    gpu_amcl_params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_params.yaml')

    amcl_type = LaunchConfiguration('amcl_type').perform(context)
    min_particles = LaunchConfiguration('min_particles').perform(context)
    max_particles = LaunchConfiguration('max_particles').perform(context)
    max_beams = LaunchConfiguration('max_beams').perform(context)

    nodes = []

    info_msg = LogInfo(msg=f'Starting F1Tenth Real Robot Localization — {amcl_type}')
    nodes.append(info_msg)

    # 1) LiDAR driver — cluster=4 and range filtering handled in YAML config
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'hokuyo_lidar.launch.py')
        ),
        launch_arguments={
            'ip_address': LaunchConfiguration('lidar_ip'),
        }.items(),
    )
    nodes.append(lidar_launch)

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
    nodes.append(map_server_node)

    # 3) AMCL localization
    lifecycle_node_names = ['map_server']

    if amcl_type == 'gpu_amcl':
        # GPU AMCL — CuPy-accelerated, params from YAML, no lifecycle
        amcl_node = Node(
            package='f1tenth_localization',
            executable='gpu_amcl_node.py',
            name='gpu_amcl',
            output='screen',
            parameters=[
                gpu_amcl_params_file,
                {
                    'use_sim_time': False,
                    'num_particles': int(max_particles),
                    'max_beams': int(max_beams),
                },
            ],
        )
    else:
        # nav2_amcl — params from YAML, overrides from launch args
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
                    'min_particles': int(min_particles),
                    'max_particles': int(max_particles),
                    'max_beams': int(max_beams),
                },
            ],
        )
        lifecycle_node_names.append('amcl')

    nodes.append(amcl_node)

    # Lifecycle manager — auto-activates map_server (and AMCL if nav2_amcl)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': lifecycle_node_names,
            'bond_timeout': 0.0,
        }],
    )
    nodes.append(lifecycle_manager)

    # 4) Performance monitor (optional)
    enable_monitor = LaunchConfiguration('enable_monitor').perform(context).lower() == 'true'
    if enable_monitor:
        monitor_node = Node(
            package='f1tenth_localization',
            executable='performance_monitor.py',
            name='performance_monitor',
            output='screen',
            parameters=[{
                'amcl_type': amcl_type,
                'min_particles': int(min_particles),
                'max_particles': int(max_particles),
                'max_beams': int(max_beams),
            }],
        )
        nodes.append(monitor_node)

    return nodes


def generate_launch_description():
    default_map = os.path.join(_workspace_root, 'f1tenth_sim', 'maps', 'Spielberg_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_ip', default_value='192.168.0.10',
            description='IP address of the Hokuyo UST-10LX'
        ),
        DeclareLaunchArgument(
            'map_file', default_value=default_map,
            description='Path to the map YAML file for map_server'
        ),
        DeclareLaunchArgument(
            'amcl_type', default_value='gpu_amcl',
            description="AMCL implementation: 'gpu_amcl' or 'nav2_amcl'"
        ),
        DeclareLaunchArgument(
            'min_particles', default_value='500',
            description='AMCL minimum particles'
        ),
        DeclareLaunchArgument(
            'max_particles', default_value='2000',
            description='AMCL maximum particles'
        ),
        DeclareLaunchArgument(
            'max_beams', default_value='120',
            description='AMCL max laser beams to use'
        ),
        DeclareLaunchArgument(
            'enable_monitor', default_value='false',
            description='Enable performance monitoring'
        ),
        OpaqueFunction(function=launch_setup),
    ])
