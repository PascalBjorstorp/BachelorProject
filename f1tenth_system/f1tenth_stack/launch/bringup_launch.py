# Copyright 2025 F1TENTH Foundation
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
#
# F1TENTH Driver Stack Launch File
# ================================
# Unified launch file for the F1TENTH car. Use launch arguments to
# select which subsystems to start:
#
#   Full stack (default — includes scan splitter + lateral planner, 270 beams):
#     ros2 launch f1tenth_stack bringup_launch.py \
#       trajectory_file:=/path/to/raceline.csv
#
#   Mapping mode (270 beams @ 20 Hz, no scan splitter or lateral planner):
#     ros2 launch f1tenth_stack bringup_launch.py mapping_mode:=true
#
#   Teleop only (no LiDAR):
#     ros2 launch f1tenth_stack bringup_launch.py use_lidar:=false
#
#   VESC only (testing motor/odom):
#     ros2 launch f1tenth_stack bringup_launch.py use_teleop:=false use_lidar:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')
    lidar_pkg_share = get_package_share_directory('f1tenth_lidar')

    # ── Config file paths ──
    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')
    hokuyo_config = os.path.join(lidar_pkg_share, 'config', 'hokuyo_ust10lx.yaml')

    # ── Package directories for included launch files ──
    lidar_pkg_dir = get_package_share_directory('f1tenth_lidar')
    lateral_planner_pkg_dir = get_package_share_directory('f1tenth_lateral_planner')

    # ── Default map path ──
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    default_map = os.path.join(workspace_root, 'f1tenth_sim', 'maps', 'my_track_map.yaml')

    # ── Launch arguments ──
    ld = LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value=joy_teleop_config,
                              description='Path to joystick configuration file'),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config,
                              description='Path to VESC configuration file'),
        DeclareLaunchArgument('sensors_config', default_value=sensors_config,
                              description='Path to sensors configuration file'),
        DeclareLaunchArgument('mux_config', default_value=mux_config,
                              description='Path to ackermann_mux configuration file'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Launch joystick teleop and mux'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='Launch LiDAR driver (Hokuyo SCIP 2.0, 40 Hz)'),
        DeclareLaunchArgument('mapping_mode', default_value='false',
                              description='Mapping mode: 270 beams @ 20 Hz, no scan splitter or lateral planner'),
        DeclareLaunchArgument('trajectory_file', default_value='/home/f1tenth/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv',
                              description='Path to global raceline CSV for lateral planner'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='Path to the map YAML file for map_server'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock for simulation time'),
    ])

    use_teleop = LaunchConfiguration('use_teleop')
    use_lidar = LaunchConfiguration('use_lidar')
    mapping_mode = LaunchConfiguration('mapping_mode')

    # ══════════════════════
    #  Map Server (always)
    # ══════════════════════
    # Serves the static occupancy-grid map on /map with transient_local QoS.
    # Started early so the map is available before localization and scan_splitter.
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='/',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_file'),
        }],
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }],
    ))

    # ══════════════════════
    #  VESC nodes (single process, zero-copy intra-process comms)
    # ══════════════════════
    ld.add_action(ComposableNodeContainer(
        name='vesc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vesc_driver',
                plugin='vesc_driver::VescDriver',
                name='vesc_driver_node',
                parameters=[LaunchConfiguration('vesc_config')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='vesc_ackermann',
                plugin='vesc_ackermann::VescToOdom',
                name='vesc_to_odom_node',
                parameters=[LaunchConfiguration('vesc_config')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='vesc_ackermann',
                plugin='vesc_ackermann::AckermannToVesc',
                name='ackermann_to_vesc_node',
                parameters=[LaunchConfiguration('vesc_config')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    ))

    # ══════════════════════
    #  Joystick + Mux
    # ══════════════════════
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')],
        condition=IfCondition(use_teleop),
    ))

    ld.add_action(Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')],
        condition=IfCondition(use_teleop),
    ))

    ld.add_action(Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        condition=IfCondition(use_teleop),
    ))

    # ══════════════════════
    #  LiDAR — Custom SCIP 2.0 driver
    # ══════════════════════
    # Normal mode: 270 beams @ 40 Hz (cluster=4, skip=0)
    ld.add_action(Node(
        package='f1tenth_lidar',
        executable='hokuyo_scip_driver_node',
        name='hokuyo_scip_driver',
        output='screen',
        parameters=[hokuyo_config, {'skip': 0}],
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and '", mapping_mode, "' != 'true'"
        ])),
    ))

    # Mapping mode: 1080 beams @ 20 Hz (cluster=1, skip=1)
    ld.add_action(Node(
        package='f1tenth_lidar',
        executable='hokuyo_scip_driver_node',
        name='hokuyo_scip_driver',
        output='screen',
        parameters=[hokuyo_config, {'skip': 0}],
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and '", mapping_mode, "' == 'true'"
        ])),
    ))

    # ══════════════════════
    #  Scan Splitter — /scan → /scan_walls + /scan_obstacles
    # ══════════════════════
    # Only launched in racing mode (mapping_mode=false).
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_dir, 'launch', 'scan_splitter.launch.py')
        ),
        condition=UnlessCondition(mapping_mode),
    ))

    # ══════════════════════
    #  Lateral Planner — opponent avoidance → /local_raceline
    # ══════════════════════
    # Only launched in racing mode (mapping_mode=false).
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lateral_planner_pkg_dir, 'launch', 'lateral_planner.launch.py')
        ),
        launch_arguments={
            'trajectory_file': LaunchConfiguration('trajectory_file'),
        }.items(),
        condition=UnlessCondition(mapping_mode),
    ))

    # ══════════════════════
    #  Static transforms
    # ══════════════════════
    # ego_racecar/base_link → ego_racecar/laser
    # Update x, y, z to match your LiDAR mounting position.
    # Use update_vehicle_params.py in the workspace root to set all values.
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
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
    ))

    # ego_racecar/base_link → ego_racecar/imu
    # VESC firmware already compensates for upside-down mounting — identity rotation.
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'ego_racecar/base_link',
            '--child-frame-id', 'ego_racecar/imu',
        ],
    ))

    return ld
