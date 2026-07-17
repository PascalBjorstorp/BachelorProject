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
        "laser_frame_id", "imu_frame_id", "imu_to_base_x_m",
        "imu_to_base_y_m", "imu_to_base_yaw_rad",
    )
    missing = [key for key in required if key not in hardware]
    if missing:
        raise ValueError(f"calibration hardware configuration missing: {missing}")
    return hardware


def generate_description(hardware: dict, raw_min: float, raw_max: float, *, simulation: bool = False) -> LaunchDescription:
    stack_share = get_package_share_directory("f1tenth_stack")
    lidar_share = get_package_share_directory("f1tenth_lidar")
    sim_share = get_package_share_directory("f1tenth_gym_ros") if simulation else None
    vesc_config = os.path.join(stack_share, "config", "vesc.yaml")
    mux_config = os.path.join(stack_share, "config", "mux.yaml")
    hokuyo_config = os.path.join(lidar_share, "config", "hokuyo_ust10lx.yaml")
    ackermann_parameters = [vesc_config]
    if simulation:
        # The real runner applies this reversible steering-test profile before
        # launch. Mirror it here so a standalone smoke test cannot inherit the
        # normal ACCEL_TO_CURRENT racing mode and coast across the room.
        ackermann_parameters.append({
            "accel_to_current_gain": 0.0,
            "accel_to_brake_gain": 0.0,
        })
    vesc_components = [] if simulation else [
        ComposableNode(package="vesc_driver", plugin="vesc_driver::VescDriver",
                       name="vesc_driver_node", parameters=[vesc_config],
                       extra_arguments=[{"use_intra_process_comms": True}]),
    ]
    vesc_components.extend([
        ComposableNode(package="vesc_ackermann", plugin="vesc_ackermann::VescToOdom",
                       name="vesc_to_odom_node", parameters=[vesc_config],
                       extra_arguments=[{"use_intra_process_comms": True}]),
        ComposableNode(package="vesc_ackermann", plugin="vesc_ackermann::AckermannToVesc",
                       name="ackermann_to_vesc_node", parameters=ackermann_parameters,
                       remappings=[("/commands/servo/position", "/steering_calibration/servo_from_ackermann")],
                       extra_arguments=[{"use_intra_process_comms": True}]),
    ])
    if simulation:
        room_map = Path(__file__).resolve().parents[3] / "f1tenth_sim" / "maps" / "calibration_room_map"
        sensor = Node(
            package="f1tenth_gym_ros", executable="gym_bridge", name="bridge",
            parameters=[os.path.join(str(sim_share), "config", "sim.yaml"), {
                "headless": True, "num_agent": 1, "map_path": str(room_map), "map_img_ext": ".pgm",
                "sx": 0.0, "sy": 0.0, "stheta": 0.7853981633974483,
                "scan_beams": 1080, "scan_publish_rate": 40.0,
                "scan_distance_to_base_link": float(hardware["laser_to_base_x_m"]),
                "publish_odom": False, "publish_base_tf": False,
                "publish_sim_vesc_sensors": True, "drive_input_mode": "vesc",
                "sim_vesc_current_topic": "/commands/motor/current",
                "sim_vesc_brake_topic": "/commands/motor/brake",
                "sim_vesc_motor_speed_topic": "/commands/motor/speed",
                "sim_vesc_servo_command_topic": "/commands/servo/position",
            }], output="screen",
        )
    else:
        sensor = Node(
            package="f1tenth_lidar", executable="hokuyo_scip_driver_node", name="hokuyo_scip_driver",
            parameters=[hokuyo_config, {"ip_address": str(hardware["lidar_ip_address"]), "skip": 0, "cluster": 1}],
            output="screen",
        )
    transforms = [
        Node(package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_imu",
             arguments=["--x", str(float(hardware["imu_to_base_x_m"])),
                        "--y", str(float(hardware["imu_to_base_y_m"])),
                        "--z", str(float(hardware.get("imu_to_base_z_m", 0.0))),
                        "--roll", "0.0", "--pitch", "0.0",
                        "--yaw", str(float(hardware["imu_to_base_yaw_rad"])),
                        "--frame-id", str(hardware["base_frame_id"]), "--child-frame-id", str(hardware["imu_frame_id"])], output="screen"),
    ]
    if not simulation:
        transforms.insert(0, Node(
            package="tf2_ros", executable="static_transform_publisher", name="static_baselink_to_laser",
            arguments=["--x", str(float(hardware["laser_to_base_x_m"])),
                       "--y", str(float(hardware["laser_to_base_y_m"])),
                       "--z", str(float(hardware["laser_to_base_z_m"])),
                       "--roll", "0.0", "--pitch", "0.0",
                       "--yaw", str(float(hardware["laser_to_base_yaw_rad"])),
                       "--frame-id", str(hardware["base_frame_id"]),
                       "--child-frame-id", str(hardware["laser_frame_id"])], output="screen",
        ))
    return LaunchDescription([
        ComposableNodeContainer(
            name="steering_calibration_vesc", namespace="", package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=vesc_components, output="screen",
        ),
        _selector_process(raw_min, raw_max),
        Node(package="ackermann_mux", executable="ackermann_mux", name="ackermann_mux",
             parameters=[mux_config], output="screen"),
        # Calibration is isolated from AMCL/planning and intentionally records
        # every native Hokuyo sample. Never inherit the reduced racing default.
        sensor,
        *transforms,
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True,
                        help="Immutable session calibration_config_snapshot.yaml")
    parser.add_argument("--raw-min", type=float, required=True)
    parser.add_argument("--raw-max", type=float, required=True)
    parser.add_argument("--simulation", action="store_true",
                        help="Use the purpose-built open-room simulator; never starts hardware drivers.")
    args = parser.parse_args()
    hardware = _hardware_from_config(args.config)
    service = LaunchService()
    service.include_launch_description(generate_description(
        hardware, args.raw_min, args.raw_max, simulation=args.simulation,
    ))
    return service.run()


if __name__ == "__main__":    raise SystemExit(main())
