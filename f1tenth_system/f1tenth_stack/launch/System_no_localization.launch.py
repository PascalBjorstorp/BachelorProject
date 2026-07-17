"""
Launch the normal F1TENTH system stack without AMCL/EKF localization.

Use this for pure odometry runs or as the base stack before launching an
alternative localization node such as nav2_amcl in a second terminal.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stack_pkg_dir = get_package_share_directory('f1tenth_stack')
    planner_pkg_dir = get_package_share_directory('f1tenth_planning')

    system_launch = os.path.join(stack_pkg_dir, 'launch', 'System_launch.py')
    joy_teleop_config = os.path.join(stack_pkg_dir, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(stack_pkg_dir, 'config', 'vesc.yaml')
    mux_config = os.path.join(stack_pkg_dir, 'config', 'mux.yaml')
    default_map = os.path.join(planner_pkg_dir, 'maps', 'my_track_map.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config,
            description='Path to VESC configuration file'),

        DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config,
            description='Path to ackermann_mux configuration file'),

        DeclareLaunchArgument(
            'joy_config',
            default_value=joy_teleop_config,
            description='Path to joystick configuration file'),

        DeclareLaunchArgument(
            'use_teleop',
            default_value='false',
            description='Launch joystick teleop'),

        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Launch LiDAR driver'),

        DeclareLaunchArgument(
            'mapping_mode',
            default_value='false',
            description='Mapping mode: full-resolution LiDAR, no scan splitter or lateral planner'),

        DeclareLaunchArgument(
            'lidar_cluster',
            default_value='4',
            description='Racing /scan clustering; 4 publishes 270 beams'),

        DeclareLaunchArgument(
            'lateral_planner_avoidance_enabled',
            default_value='false',
            description='Enable lateral planner obstacle avoidance'),

        DeclareLaunchArgument(
            'lateral_planner_delay_sec',
            default_value='2.0',
            description='Delay before starting the lateral planner after bringup starts in seconds'),

        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to the map YAML file for map_server'),

        DeclareLaunchArgument(
            'bringup_delay_sec',
            default_value='2.0',
            description='Delay before bringup starts in seconds'),

        DeclareLaunchArgument(
            'use_dynamic_bicycle_model',
            default_value='true',
            description='Enable dynamic bicycle model inside vesc_to_odom node'),

        DeclareLaunchArgument(
            'oldOdom',
            default_value='false',
            description='Use legacy analytical vesc_to_odom implementation'),

        DeclareLaunchArgument(
            'use_localization',
            default_value='false',
            description='Keep localization disabled for this launch wrapper'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'use_localization': LaunchConfiguration('use_localization'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vesc_config': LaunchConfiguration('vesc_config'),
                'mux_config': LaunchConfiguration('mux_config'),
                'joy_config': LaunchConfiguration('joy_config'),
                'use_teleop': LaunchConfiguration('use_teleop'),
                'use_lidar': LaunchConfiguration('use_lidar'),
                'mapping_mode': LaunchConfiguration('mapping_mode'),
                'lidar_cluster': LaunchConfiguration('lidar_cluster'),
                'lateral_planner_avoidance_enabled': LaunchConfiguration(
                    'lateral_planner_avoidance_enabled'),
                'lateral_planner_delay_sec': LaunchConfiguration('lateral_planner_delay_sec'),
                'map_file': LaunchConfiguration('map_file'),
                'bringup_delay_sec': LaunchConfiguration('bringup_delay_sec'),
                'use_dynamic_bicycle_model': LaunchConfiguration('use_dynamic_bicycle_model'),
                'oldOdom': LaunchConfiguration('oldOdom'),
            }.items(),
        ),
    ])
