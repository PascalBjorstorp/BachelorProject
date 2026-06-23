#!/usr/bin/env python3
"""Hardware-only end-stop stack with the same raw-servo selector.

No Ackermann conversion, mux, planner, or motor command source is started.
The selector still owns the actual VESC servo topic and clamps only to the
provisional hard raw guard supplied by the calibration configuration.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_description(raw_min: float, raw_max: float) -> LaunchDescription:
    stack_share = get_package_share_directory("f1tenth_stack")
    vesc_config = os.path.join(stack_share, "config", "vesc.yaml")
    selector = Path(__file__).resolve().parents[1] / "steering_calibration" / "servo_selector.py"
    return LaunchDescription([
        ComposableNodeContainer(
            name="steering_calibration_hardware", namespace="", package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                ComposableNode(package="vesc_driver", plugin="vesc_driver::VescDriver",
                               name="vesc_driver_node", parameters=[vesc_config],
                               extra_arguments=[{"use_intra_process_comms": True}]),
                ComposableNode(package="vesc_ackermann", plugin="vesc_ackermann::VescToOdom",
                               name="vesc_to_odom_node", parameters=[vesc_config],
                               extra_arguments=[{"use_intra_process_comms": True}]),
            ], output="screen",
        ),
        ExecuteProcess(cmd=["python3", str(selector), "--raw-min", str(raw_min), "--raw-max", str(raw_max)],
                       output="screen"),
    ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True,
                        help="Immutable session calibration_config_snapshot.yaml; archived for common launcher interface.")
    parser.add_argument("--raw-min", type=float, required=True)
    parser.add_argument("--raw-max", type=float, required=True)
    args = parser.parse_args()
    service = LaunchService()
    service.include_launch_description(generate_description(args.raw_min, args.raw_max))
    return service.run()


if __name__ == "__main__":
    raise SystemExit(main())
