# Copyright 2025 F1TENTH Foundation
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
#
# F1TENTH Driver Stack Launch File
# ================================
# Launches all nodes needed to operate the F1TENTH car:
# - Joystick driver and teleop
# - VESC motor controller communication
# - Odometry computation
# - LiDAR sensor
# - Ackermann command multiplexer
# - Static transforms

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get config file paths
    pkg_share = get_package_share_directory('f1tenth_stack')

    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')

    # Declare launch arguments
    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Path to joystick configuration file'
    )
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Path to VESC configuration file'
    )
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Path to sensors configuration file'
    )
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Path to ackermann_mux configuration file'
    )

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la])

    # ===================
    # Joystick Nodes
    # ===================
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )

    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )

    # ===================
    # VESC Nodes
    # ===================
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    # ===================
    # LiDAR Node
    # ===================
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )

    # ===================
    # Ackermann Mux
    # ===================
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    # ===================
    # Static Transforms
    # ===================
    # Transform from base_link to laser
    # Adjust x, y, z for your specific LiDAR mounting position
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=[
            '0.27', '0.0', '0.11',   # x, y, z translation (meters)
            '0.0', '0.0', '0.0',     # roll, pitch, yaw rotation (radians)
            'base_link', 'laser'     # parent_frame, child_frame
        ]
    )

    # Transform from base_link to IMU (if needed)
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=[
            '0.0', '0.0', '0.0',     # x, y, z (IMU typically at base_link origin)
            '0.0', '0.0', '0.0',     # roll, pitch, yaw
            'base_link', 'imu'
        ]
    )

    # Add all nodes to launch description
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(static_tf_base_to_laser)
    ld.add_action(static_tf_base_to_imu)

    return ld
