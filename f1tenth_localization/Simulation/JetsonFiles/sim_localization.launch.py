"""
F1TENTH Simulation Localization Launch File

Launches AMCL localization for use with the F1Tenth simulator.
The simulator provides /scan, /map, and odom->base_link TF.

IMPORTANT: The simulation must be configured to publish odom->base_link!
In f1tenth_sim/config/sim.yaml, set:
  tf_frame_id: 'odom'
  odom_frame_id: 'odom'

Usage:
  ros2 launch f1tenth_localization sim_localization.launch.py
  ros2 launch f1tenth_localization sim_localization.launch.py min_particles:=200 max_particles:=1000
  ros2 launch f1tenth_localization sim_localization.launch.py enable_monitor:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')

    # =========================================================================
    # Launch Arguments
    # =========================================================================

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

    info_msg = LogInfo(
        msg='Starting F1Tenth Simulation Localization. '
            'Ensure simulator is running with tf_frame_id=odom!'
    )

    # AMCL localization — params from YAML, particles/beams overridden by launch args
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
                'min_particles': LaunchConfiguration('min_particles'),
                'max_particles': LaunchConfiguration('max_particles'),
                'max_beams': LaunchConfiguration('max_beams'),
            },
        ],
    )

    # Lifecycle manager — auto-activates AMCL
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }],
    )

    # Performance monitor (optional)
    monitor_node = Node(
        package='f1tenth_localization',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_monitor')),
        parameters=[{
            'use_sim_time': True,
            'amcl_type': 'nav2_amcl',
            'min_particles': LaunchConfiguration('min_particles'),
            'max_particles': LaunchConfiguration('max_particles'),
            'max_beams': LaunchConfiguration('max_beams'),
        }],
    )

    return LaunchDescription([
        declare_min_particles,
        declare_max_particles,
        declare_max_beams,
        declare_enable_monitor,
        info_msg,
        amcl_node,
        lifecycle_manager,
        monitor_node,
    ])
