#!/usr/bin/env python3
"""
Export per-solve ADMM iteration count joined with the nearest vehicle pose.

The hardware MPC node publishes the iteration count as std_msgs/Float64 on
`/mpc/timing/iteration_count` (no header timestamp), and pose comes from the
EKF on `/ekf_pose`. We pair them by bag record timestamp so the output CSV
can drive both the boxplot (Test 4 baseline) and the spatial mismatch
heatmap (Test 4 follow-up).

Output columns: idx, stamp_ns, iterations, pos_x, pos_y

Usage:
  python3 export_iteration_count_csv.py \
    --bag /path/to/file.mcap \
    --out /path/to/iters_with_pose.csv
"""

import argparse
import bisect
import csv
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


DEFAULT_ITER_TOPIC = "/mpc/timing/iteration_count"
DEFAULT_POSE_TOPIC = "/ekf_pose"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="Path to .mcap bag file")
    ap.add_argument("--out", required=True, help="Output CSV path")
    ap.add_argument("--iter-topic", default=DEFAULT_ITER_TOPIC,
                    help=f"Iteration-count topic (default: {DEFAULT_ITER_TOPIC})")
    ap.add_argument("--pose-topic", default=DEFAULT_POSE_TOPIC,
                    help=f"Pose topic (default: {DEFAULT_POSE_TOPIC})")
    args = ap.parse_args()

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )

    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (args.iter_topic, args.pose_topic):
        if t not in type_map:
            print(f"Topic {t} not found in bag", file=sys.stderr)
            return 3
    iter_msg_type = get_message(type_map[args.iter_topic])
    pose_msg_type = get_message(type_map[args.pose_topic])

    iter_events = []   # list of (stamp_ns, iterations)
    pose_events = []   # list of (stamp_ns, x, y)

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

    if not pose_events:
        print(f"No messages on pose topic {args.pose_topic}", file=sys.stderr)
        return 4

    pose_events.sort(key=lambda p: p[0])
    pose_stamps = [p[0] for p in pose_events]

    count = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "stamp_ns", "iterations", "pos_x", "pos_y"])
        for stamp, iters in iter_events:
            i = bisect.bisect_left(pose_stamps, stamp)
            candidates = []
            if i < len(pose_events):
                candidates.append(pose_events[i])
            if i > 0:
                candidates.append(pose_events[i - 1])
            nearest = min(candidates, key=lambda p: abs(p[0] - stamp))
            count += 1
            w.writerow([count, stamp, iters, nearest[1], nearest[2]])

    print(f"Exported {count} (iter+pose) rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
