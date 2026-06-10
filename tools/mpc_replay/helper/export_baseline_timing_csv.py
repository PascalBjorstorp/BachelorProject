#!/usr/bin/env python3
"""
Export the CPU MPC node's per-solve timing topics joined with the nearest
vehicle pose from a rosbag2 MCAP.

For Test 1 (baseline vs FPGA comparison) we need:
  /mpc/timing/iteration_count   ADMM iterations per solve
  /mpc/timing/solve_us          per-solve CPU compute time [us]
  /mpc/timing/ekf_to_control_ms end-to-end pipeline latency [ms] (optional)
  /ekf_pose                     vehicle pose (for downstream lap filtering)

We use the iteration_count messages as the join spine (one row per solve)
and attach the nearest other-topic message by bag record timestamp.

Output columns: idx, stamp_ns, iterations, solve_us, ekf_to_control_ms,
                pos_x, pos_y

Usage:
  python3 export_baseline_timing_csv.py --bag /path/to/file.mcap \
                                        --out /path/to/timing.csv
"""

import argparse
import bisect
import csv
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


ITER_TOPIC    = "/mpc/timing/iteration_count"
SOLVE_TOPIC   = "/mpc/timing/solve_us"
EKF2CTL_TOPIC = "/mpc/timing/ekf_to_control_ms"
POSE_TOPIC    = "/ekf_pose"


def nearest(events, stamps, stamp):
    i = bisect.bisect_left(stamps, stamp)
    cands = []
    if i < len(events): cands.append(events[i])
    if i > 0: cands.append(events[i - 1])
    return min(cands, key=lambda e: abs(e[0] - stamp))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    reader = SequentialReader()
    reader.open(StorageOptions(uri=args.bag, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for t in (ITER_TOPIC, SOLVE_TOPIC, POSE_TOPIC):
        if t not in type_map:
            print(f"Required topic missing: {t}", file=sys.stderr); return 3

    iter_type  = get_message(type_map[ITER_TOPIC])
    solve_type = get_message(type_map[SOLVE_TOPIC])
    pose_type  = get_message(type_map[POSE_TOPIC])
    ekf_type   = get_message(type_map[EKF2CTL_TOPIC]) if EKF2CTL_TOPIC in type_map else None

    iters, solves, ekfs, poses = [], [], [], []
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic == ITER_TOPIC:
            iters.append((int(stamp_ns), float(deserialize_message(data, iter_type).data)))
        elif topic == SOLVE_TOPIC:
            solves.append((int(stamp_ns), float(deserialize_message(data, solve_type).data)))
        elif ekf_type is not None and topic == EKF2CTL_TOPIC:
            ekfs.append((int(stamp_ns), float(deserialize_message(data, ekf_type).data)))
        elif topic == POSE_TOPIC:
            msg = deserialize_message(data, pose_type)
            poses.append((int(stamp_ns),
                          float(msg.pose.pose.position.x),
                          float(msg.pose.pose.position.y)))

    if not solves:
        print(f"No {SOLVE_TOPIC} messages found", file=sys.stderr); return 4
    if not poses:
        print(f"No {POSE_TOPIC} messages found", file=sys.stderr); return 4

    solves.sort(key=lambda x: x[0]); solve_stamps = [s[0] for s in solves]
    ekfs.sort(key=lambda x: x[0]);   ekf_stamps   = [e[0] for e in ekfs]
    poses.sort(key=lambda x: x[0]);  pose_stamps  = [p[0] for p in poses]

    count = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "stamp_ns", "iterations", "solve_us",
                    "ekf_to_control_ms", "pos_x", "pos_y"])
        for stamp, iters_val in iters:
            s = nearest(solves, solve_stamps, stamp)
            e = nearest(ekfs, ekf_stamps, stamp) if ekfs else (stamp, float("nan"))
            p = nearest(poses, pose_stamps, stamp)
            count += 1
            w.writerow([count, stamp, iters_val, s[1], e[1], p[1], p[2]])

    print(f"Exported {count} timing rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
