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
#   Full stack (default):
#     ros2 launch f1tenth_stack bringup_launch.py
#
#   Teleop only (no LiDAR):
#     ros2 launch f1tenth_stack bringup_launch.py use_lidar:=false
#
#   VESC only (testing motor/odom):
#     ros2 launch f1tenth_stack bringup_launch.py use_teleop:=false use_lidar:=false

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')
    lidar_pkg_share = get_package_share_directory('f1tenth_localization')

    # ── Config file paths ──
    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')
    hokuyo_config = os.path.join(lidar_pkg_share, 'config', 'hokuyo_ust10lx.yaml')

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
    ])

    use_teleop = LaunchConfiguration('use_teleop')
    use_lidar = LaunchConfiguration('use_lidar')

    # ══════════════════════
    #  VESC nodes (always)
    # ══════════════════════
    ld.add_action(Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen',
    ))

    ld.add_action(Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
    ))

    ld.add_action(Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
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
    #  LiDAR — Custom SCIP 2.0 driver (40 Hz)
    # ══════════════════════
    ld.add_action(Node(
        package='f1tenth_localization',
        executable='hokuyo_scip_driver.py',
        name='hokuyo_scip_driver',
        output='screen',
        parameters=[hokuyo_config],
        condition=IfCondition(use_lidar),
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
            '--x', '0.275',
            '--y', '0.0',
            '--z', '0.05',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'ego_racecar/base_link',
            '--child-frame-id', 'ego_racecar/laser',
        ],
    ))

    # base_link → imu
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=[
            '0.0', '0.0', '0.0',      # x, y, z
            '0.0', '0.0', '0.0',      # roll, pitch, yaw
            'base_link', 'imu',
        ],
    ))

    return ld
