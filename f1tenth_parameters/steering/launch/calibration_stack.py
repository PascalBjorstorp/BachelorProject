#!/usr/bin/env python3
"""Dedicated calibration stack with exclusive raw-servo selector.

The stack starts VESC, odometry, Ackermann motor conversion, mux, LiDAR, and
transforms. It does not start MPC, MPCC, planner, map server or teleoperation.
AckermannToVesc's servo output is remapped away from the real VESC topic; the
selector is the only publisher of /commands/servo/position.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def _selector_process(raw_min: float, raw_max: float) -> ExecuteProcess:
    root = Path(__file__).resolve().parents[1]
    selector = root / "steering_calibration" / "servo_selector.py"
    return ExecuteProcess(cmd=["python3", str(selector), "--raw-min", str(raw_min), "--raw-max", str(raw_max)],
                          output="screen")


def generate_description(lidar_ip: str, raw_min: float, raw_max: float) -> LaunchDescription:
    stack_share = get_package_share_directory("f1tenth_stack")
    lidar_share = get_package_share_directory("f1tenth_lidar")
    vesc_config = os.path.join(stack_share, "config", "vesc.yaml")
    mux_config = os.path.join(stack_share, "config", "mux.yaml")
    hokuyo_config = os.path.join(lidar_share, "config", "hokuyo_ust10lx.yaml")
    return LaunchDescription([
        ComposableNodeContainer(
            name="steering_calibration_vesc", namespace="", package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(package="vesc_driver", plugin="vesc_driver::VescDriver",
                               name="vesc_driver_node", parameters=[vesc_config],
                               extra_arguments=[{"use_intra_process_comms": True}]),
                ComposableNode(package="vesc_ackermann", plugin="vesc_ackermann::VescToOdom",
                               name="vesc_to_odom_node", parameters=[vesc_config],
                               extra_arguments=[{"use_intra_process_comms": True}]),
                ComposableNode(package="vesc_ackermann", plugin="vesc_ackermann::AckermannToVesc",
                               name="ackermann_to_vesc_node", parameters=[vesc_config],
                               remappings=[("/commands/servo/position", "/steering_calibration/servo_from_ackermann")],
                               extra_arguments=[{"use_intra_process_comms": True}]),
            ], output="screen",
        ),
        _selector_process(raw_min, raw_max),
        Node(package="ackermann_mux", executable="ackermann_mux", name="ackermann_mux",
             parameters=[mux_config], output="screen"),
        Node(package="f1tenth_lidar", executable="hokuyo_scip_driver_node", name="hokuyo_scip_driver",
             parameters=[hokuyo_config, {"ip_address": lidar_ip, "skip": 0}], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_laser",
             arguments=["--x", "0.265", "--y", "0.0", "--z", "0.05", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "ego_racecar/base_link", "--child-frame-id", "ego_racecar/laser"], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_imu",
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "ego_racecar/base_link", "--child-frame-id", "ego_racecar/imu"], output="screen"),
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--lidar-ip", default="192.168.10.10")
    parser.add_argument("--raw-min", type=float, required=True)
    parser.add_argument("--raw-max", type=float, required=True)
    args = parser.parse_args()
    service = LaunchService()
    service.include_launch_description(generate_description(args.lidar_ip, args.raw_min, args.raw_max))
    return service.run()


if __name__ == "__main__":    raise SystemExit(main())
