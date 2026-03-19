"""
F1TENTH Simulation Localization Launch File

Launches AMCL localization for use with the F1Tenth simulator.
The simulator provides /scan, /map, and ego_racecar/odom->base_link TF.
Supports both nav2_amcl and gpu_amcl (default: gpu_amcl).

IMPORTANT: The simulation must be configured to publish ego_racecar/odom->base_link!
In f1tenth_sim/config/sim.yaml, set:
  tf_frame_id: 'ego_racecar/odom'
  odom_frame_id: 'ego_racecar/odom'

Usage:
  ros2 launch f1tenth_localization sim_localization.launch.py
  ros2 launch f1tenth_localization sim_localization.launch.py amcl_type:=nav2_amcl
  ros2 launch f1tenth_localization sim_localization.launch.py min_particles:=200 max_particles:=1000
  ros2 launch f1tenth_localization sim_localization.launch.py enable_monitor:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    pkg_dir = get_package_share_directory('f1tenth_localization')
    lidar_pkg_dir = get_package_share_directory('f1tenth_lidar')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')
    gpu_amcl_params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_params.yaml')

    amcl_type = LaunchConfiguration('amcl_type').perform(context)
    min_particles = LaunchConfiguration('min_particles').perform(context)
    max_particles = LaunchConfiguration('max_particles').perform(context)
    max_beams = LaunchConfiguration('max_beams').perform(context)

    nodes = []

    info_msg = LogInfo(
        msg=f'Starting F1Tenth Simulation Localization ({amcl_type}). '
            'Ensure simulator is running with tf_frame_id=ego_racecar/odom!'
    )
    nodes.append(info_msg)

    # Scan splitter — classifies beams as wall/obstacle so AMCL gets /scan_walls
    splitter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_dir, 'launch', 'scan_splitter.launch.py')
        ),
    )
    nodes.append(splitter_launch)

    # ---- AMCL node ----
    lifecycle_node_names = []

    if amcl_type == 'gpu_amcl':
        # GPU AMCL — params from YAML, no lifecycle
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
                },
            ],
        )
    else:
        # nav2_amcl — params from YAML, needs lifecycle
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
                    'min_particles': int(min_particles),
                    'max_particles': int(max_particles),
                    'max_beams': int(max_beams),
                },
            ],
        )
        lifecycle_node_names.append('amcl')

    nodes.append(amcl_node)

    # Lifecycle manager (only needed if nav2_amcl)
    if lifecycle_node_names:
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': lifecycle_node_names,
                'bond_timeout': 0.0,
            }],
        )
        nodes.append(lifecycle_manager)

    # Performance monitor (optional)
    enable_monitor = LaunchConfiguration('enable_monitor').perform(context).lower() == 'true'
    if enable_monitor:
        monitor_node = Node(
            package='f1tenth_localization',
            executable='performance_monitor_cpp',
            name='performance_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'amcl_type': amcl_type,
                'min_particles': int(min_particles),
                'max_particles': int(max_particles),
                'max_beams': int(max_beams),
            }],
        )
        nodes.append(monitor_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
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
