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

import yaml

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


def _hardware_from_config(config_path: Path) -> dict:
    config_path = config_path.expanduser().resolve()
    if not config_path.is_file():
        raise FileNotFoundError(f"calibration config does not exist: {config_path}")
    document = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    hardware = document.get("hardware")
    if not isinstance(hardware, dict):
        raise ValueError("calibration config has no hardware mapping")
    required = (
        "lidar_ip_address", "laser_to_base_x_m", "laser_to_base_y_m",
        "laser_to_base_z_m", "laser_to_base_yaw_rad", "base_frame_id",
        "laser_frame_id", "imu_frame_id",
    )
    missing = [key for key in required if key not in hardware]
    if missing:
        raise ValueError(f"calibration hardware configuration missing: {missing}")
    return hardware


def generate_description(hardware: dict, raw_min: float, raw_max: float) -> LaunchDescription:
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
             parameters=[hokuyo_config, {"ip_address": str(hardware["lidar_ip_address"]), "skip": 0}], output="screen"),
        # This transform is constructed only from the immutable session
        # configuration snapshot.  Offline ICP reads that same snapshot.
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_laser",
             arguments=["--x", str(float(hardware["laser_to_base_x_m"])),
                        "--y", str(float(hardware["laser_to_base_y_m"])),
                        "--z", str(float(hardware["laser_to_base_z_m"])),
                        "--roll", "0.0", "--pitch", "0.0",
                        "--yaw", str(float(hardware["laser_to_base_yaw_rad"])),
                        "--frame-id", str(hardware["base_frame_id"]),
                        "--child-frame-id", str(hardware["laser_frame_id"])], output="screen"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_imu",
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.0", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", str(hardware["base_frame_id"]), "--child-frame-id", str(hardware["imu_frame_id"])], output="screen"),
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True,
                        help="Immutable session calibration_config_snapshot.yaml")
    parser.add_argument("--raw-min", type=float, required=True)
    parser.add_argument("--raw-max", type=float, required=True)
    args = parser.parse_args()
    hardware = _hardware_from_config(args.config)
    service = LaunchService()
    service.include_launch_description(generate_description(hardware, args.raw_min, args.raw_max))
    return service.run()


if __name__ == "__main__":    raise SystemExit(main())
