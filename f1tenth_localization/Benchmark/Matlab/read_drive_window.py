#!/usr/bin/env python3
"""Read first and last active drive-command timestamps from a ROS 2 bag."""

import json
import math
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_seconds(msg, fallback_time):
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_time
    sec = float(getattr(stamp, "sec", 0.0))
    nanosec = float(getattr(stamp, "nanosec", getattr(stamp, "nsec", 0.0)))
    value = sec + nanosec * 1e-9
    if not math.isfinite(value) or value <= 0.0:
        return fallback_time
    return value


def main():
    if len(sys.argv) < 4:
        raise SystemExit("usage: read_drive_window.py BAG SPEED_THRESHOLD ACCEL_THRESHOLD")

    bag_path = sys.argv[1]
    speed_threshold = float(sys.argv[2])
    accel_threshold = float(sys.argv[3])
    candidate_topics = ["/drive", "/ackermann_cmd"]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )

    topics = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    drive_topic = next((topic for topic in candidate_topics if topic in topics), None)
    if drive_topic is None:
        drive_topic = next((topic for topic in topics if "drive" in topic or "ackermann" in topic), None)
    if drive_topic is None:
        print(json.dumps({"ok": False, "reason": "no drive topic"}))
        return

    msg_type = get_message(topics[drive_topic])
    first_active = None
    last_active = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic != drive_topic:
            continue

        msg = deserialize_message(data, msg_type)
        command = getattr(msg, "drive", msg)
        speed = float(getattr(command, "speed", math.nan))
        acceleration = float(getattr(command, "acceleration", math.nan))
        active = (
            (math.isfinite(speed) and abs(speed) > speed_threshold)
            or (math.isfinite(acceleration) and abs(acceleration) > accel_threshold)
        )
        if active:
            timestamp_s = stamp_to_seconds(msg, float(timestamp_ns) * 1e-9)
            if first_active is None:
                first_active = timestamp_s
            last_active = timestamp_s

    if first_active is None or last_active is None:
        print(json.dumps({"ok": False, "reason": "no active command", "topic": drive_topic}))
        return

    print(json.dumps({
        "ok": True,
        "topic": drive_topic,
        "first": first_active,
        "last": last_active,
    }))


if __name__ == "__main__":
    main()
