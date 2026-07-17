#!/usr/bin/env python3
"""Export analysis-critical scalar fields from an MCAP bag to Parquet.

The MCAP remains the primary record: every topic is recorded, including raw
LaserScan arrays, TF, parameter events and diagnostics. This exporter produces
compact scalar tables for the analyses while preserving a per-message topic
index so omitted conversions remain discoverable.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def header_ns(message) -> int | None:
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return None
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def parse_json_or_raw(value: str) -> dict:
    try:
        parsed = json.loads(value)
        return parsed if isinstance(parsed, dict) else {"value": parsed}
    except Exception:
        return {"raw": value, "parse_error": True}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="A stage bag directory containing metadata.yaml")
    args = parser.parse_args()
    bag_dir = args.bag.resolve()
    if not (bag_dir / "metadata.yaml").exists():
        raise SystemExit(f"not a rosbag directory: {bag_dir}")
    derived = bag_dir.parent / "derived"
    derived.mkdir(exist_ok=True)
    try:
        import pandas as pd
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit("Source the ROS workspace and install pandas/pyarrow before export: " + repr(exc))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    names = [
        "imu", "odom", "vesc", "drive", "ackermann", "servo_command", "servo_echo",
        "servo_raw", "servo_selected", "servo_from_ackermann", "servo_mode", "servo_selector_status",
        "motor_command", "events", "scan_index", "tf_index", "parameter_event_index", "topic_index",
    ]
    rows: dict[str, list[dict]] = {name: [] for name in names}
    msg_classes: dict[str, object] = {}
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        type_name = topic_types.get(topic)
        if not type_name:
            continue
        try:
            cls = msg_classes.setdefault(type_name, get_message(type_name))
            msg = deserialize_message(raw, cls)
        except Exception as exc:
            rows["topic_index"].append({"topic": topic, "bag_ns": int(bag_ns), "header_ns": None,
                                        "type": type_name, "decode_error": repr(exc)})
            continue
        common = {"topic": topic, "bag_ns": int(bag_ns), "header_ns": header_ns(msg), "type": type_name}
        rows["topic_index"].append({**common, "decode_error": None})
        if topic == "/sensors/imu/raw":
            rows["imu"].append({**common,
                "ax": msg.linear_acceleration.x, "ay": msg.linear_acceleration.y, "az": msg.linear_acceleration.z,
                "gx": msg.angular_velocity.x, "gy": msg.angular_velocity.y, "gz": msg.angular_velocity.z,
            })
        elif topic == "/ego_racecar/odom":
            pose, twist = msg.pose.pose, msg.twist.twist
            rows["odom"].append({**common,
                "x": pose.position.x, "y": pose.position.y, "z": pose.position.z,
                "qx": pose.orientation.x, "qy": pose.orientation.y, "qz": pose.orientation.z, "qw": pose.orientation.w,
                "vx": twist.linear.x, "vy": twist.linear.y, "vz": twist.linear.z,
                "wx": twist.angular.x, "wy": twist.angular.y, "wz": twist.angular.z,
            })
        elif topic == "/sensors/core":
            state = msg.state
            rows["vesc"].append({**common,
                "erpm": getattr(state, "speed", math.nan),
                "motor_current": getattr(state, "current_motor", math.nan),
                "input_current": getattr(state, "current_input", math.nan),
                "battery_voltage": getattr(state, "voltage_input", math.nan),
                "duty_cycle": getattr(state, "duty_cycle", math.nan),
                "temp_fet": getattr(state, "temp_fet", math.nan),
                "temp_motor": getattr(state, "temp_motor", math.nan),
                "fault_code": getattr(state, "fault_code", math.nan),
            })
        elif topic in {"/drive", "/ackermann_cmd"}:
            key = "drive" if topic == "/drive" else "ackermann"
            rows[key].append({**common,
                "speed": msg.drive.speed, "steering_angle": msg.drive.steering_angle,
                "acceleration": msg.drive.acceleration,
                "steering_angle_velocity": msg.drive.steering_angle_velocity,
            })
        elif topic == "/commands/servo/position":
            rows["servo_command"].append({**common, "value": msg.data})
        elif topic == "/steering_calibration/servo_raw":
            rows["servo_raw"].append({**common, "value": msg.data})
        elif topic == "/steering_calibration/servo_selected":
            rows["servo_selected"].append({**common, "value": msg.data})
        elif topic == "/steering_calibration/servo_from_ackermann":
            rows["servo_from_ackermann"].append({**common, "value": msg.data})
        elif topic == "/steering_calibration/servo_mode":
            rows["servo_mode"].append({**common, "value": msg.data})
        elif topic == "/steering_calibration/servo_selector_status":
            rows["servo_selector_status"].append({**common, **parse_json_or_raw(msg.data)})
        elif topic == "/sensors/servo_position_command":
            rows["servo_echo"].append({**common, "value": msg.data})
        elif topic in {"/commands/motor/speed", "/commands/motor/current", "/commands/motor/brake"}:
            rows["motor_command"].append({**common, "command": topic.rsplit("/", 1)[-1], "value": msg.data})
        elif topic == "/scan":
            finite_count = sum(1 for value in msg.ranges if math.isfinite(value))
            rows["scan_index"].append({**common,
                "frame_id": msg.header.frame_id, "range_count": len(msg.ranges), "finite_range_count": finite_count,
                "angle_min": msg.angle_min, "angle_max": msg.angle_max, "angle_increment": msg.angle_increment,
                "time_increment": msg.time_increment, "scan_time": msg.scan_time,
                "range_min": msg.range_min, "range_max": msg.range_max,
            })
        elif topic in {"/tf", "/tf_static"}:
            rows["tf_index"].append({**common, "transform_count": len(msg.transforms)})
        elif topic == "/parameter_events":
            rows["parameter_event_index"].append({**common,
                "node": getattr(msg, "node", ""), "new_parameter_count": len(getattr(msg, "new_parameters", [])),
                "changed_parameter_count": len(getattr(msg, "changed_parameters", [])),
                "deleted_parameter_count": len(getattr(msg, "deleted_parameters", [])),
            })
        elif topic == "/steering_calibration/event":
            rows["events"].append({**common, **parse_json_or_raw(msg.data)})

    import pandas as pd
    for name, data in rows.items():
        if data:
            pd.DataFrame(data).to_parquet(derived / f"{name}.parquet", index=False)
    topic_counts = {}
    for row in rows["topic_index"]:
        if row.get("decode_error") is None:
            topic_counts[row["topic"]] = topic_counts.get(row["topic"], 0) + 1
    summary = {
        "bag_dir": str(bag_dir),
        "topic_types": topic_types,
        "row_counts": {name: len(data) for name, data in rows.items()},
        "decoded_message_counts_by_topic": dict(sorted(topic_counts.items())),
        "raw_laserscan_arrays_preserved_in_mcap": True,
        "all_unexported_topics_remain_in_mcap": True,
    }
    (derived / "export_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True, default=str) + "\n")
    print(json.dumps(summary, indent=2, sort_keys=True, default=str))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
