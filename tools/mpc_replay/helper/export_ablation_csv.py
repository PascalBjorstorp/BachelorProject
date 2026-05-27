#!/usr/bin/env python3
"""
Export per-solve ADMM iteration count joined with the nearest vehicle pose
and commanded control (steering + speed) from a rosbag2 MCAP.

Topics used:
  /mpc/timing/iteration_count   std_msgs/Float64           (the join spine)
  /ekf_pose                     PoseWithCovarianceStamped
  /drive                        ackermann_msgs/AckermannDriveStamped

For each iteration_count message we attach the nearest /ekf_pose and /drive
message by bag record timestamp. The pose topic has no exact controller-side
sync but for ~200Hz pose at race speeds the spatial error is < 1cm.

Output columns: idx, stamp_ns, iterations, pos_x, pos_y, steer, speed

Usage:
  python3 export_ablation_csv.py --bag /path/to/file.mcap --out /path/to/out.csv
"""

import argparse
import bisect
import csv
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


ITER_TOPIC = "/mpc/timing/iteration_count"
POSE_TOPIC = "/ekf_pose"
DRIVE_TOPIC = "/drive"


def find_nearest(events, stamp_list, stamp):
    i = bisect.bisect_left(stamp_list, stamp)
    candidates = []
    if i < len(events):
        candidates.append(events[i])
    if i > 0:
        candidates.append(events[i - 1])
    return min(candidates, key=lambda e: abs(e[0] - stamp))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to .mcap bag file")
    ap.add_argument("--out", required=True, help="Output CSV path")
    ap.add_argument("--iter-topic", default=ITER_TOPIC)
    ap.add_argument("--pose-topic", default=POSE_TOPIC)
    ap.add_argument("--drive-topic", default=DRIVE_TOPIC)
    args = ap.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (args.iter_topic, args.pose_topic, args.drive_topic):
        if t not in type_map:
            print(f"Topic {t} not found in bag", file=sys.stderr)
            return 3

    iter_msg_type  = get_message(type_map[args.iter_topic])
    pose_msg_type  = get_message(type_map[args.pose_topic])
    drive_msg_type = get_message(type_map[args.drive_topic])

    iter_events  = []   # (stamp_ns, iters)
    pose_events  = []   # (stamp_ns, x, y)
    drive_events = []   # (stamp_ns, steer, speed)

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == args.iter_topic:
            msg = deserialize_message(data, iter_msg_type)
            iter_events.append((int(stamp_ns), float(msg.data)))
        elif topic == args.pose_topic:
            msg = deserialize_message(data, pose_msg_type)
            x = float(msg.pose.pose.position.x)
            y = float(msg.pose.pose.position.y)
            pose_events.append((int(stamp_ns), x, y))
        elif topic == args.drive_topic:
            msg = deserialize_message(data, drive_msg_type)
            steer = float(msg.drive.steering_angle)
            speed = float(msg.drive.speed)
            drive_events.append((int(stamp_ns), steer, speed))

    if not pose_events:
        print(f"No messages on {args.pose_topic}", file=sys.stderr)
        return 4
    if not drive_events:
        print(f"No messages on {args.drive_topic}", file=sys.stderr)
        return 4

    pose_events.sort(key=lambda p: p[0])
    drive_events.sort(key=lambda d: d[0])
    pose_stamps  = [p[0] for p in pose_events]
    drive_stamps = [d[0] for d in drive_events]

    count = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "stamp_ns", "iterations", "pos_x", "pos_y", "steer", "speed"])
        for stamp, iters in iter_events:
            pose = find_nearest(pose_events, pose_stamps, stamp)
            drive = find_nearest(drive_events, drive_stamps, stamp)
            count += 1
            w.writerow([count, stamp, iters,
                        pose[1], pose[2],
                        drive[1], drive[2]])

    print(f"Exported {count} ablation rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
