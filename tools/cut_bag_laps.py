#!/usr/bin/env python3
"""
cut_bag_laps.py
===============
Cut an mcap bag down to a contiguous range of laps, detected from the
/local_raceline topic. Skips the ramp-up first lap and keeps N laps.

All topics are copied within the kept time window. For latched / transient-local
topics (e.g. /map), the last message published before the window is carried
forward so the cut bag stays usable for replay.

Usage:
    python3 tools/cut_bag_laps.py IN_BAG OUT_BAG [--skip-laps 1] [--keep-laps 5]
"""

import argparse
import os
import numpy as np

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

START_RADIUS = 0.30


def reader_for(uri):
    if os.path.isdir(uri):
        mcaps = [f for f in os.listdir(uri) if f.endswith(".mcap")]
        if len(mcaps) != 1:
            # let rosbag2 handle multi-file bags via the directory uri
            storage_uri = uri
        else:
            storage_uri = os.path.join(uri, mcaps[0])
    else:
        storage_uri = uri
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=storage_uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    return r, storage_uri


def find_window(storage_uri, skip_laps, keep_laps):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=storage_uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
    RL = get_message(tmap["/local_raceline"])
    r.set_filter(rosbag2_py.StorageFilter(topics=["/local_raceline"]))
    T, X, Y = [], [], []
    while r.has_next():
        _, data, t = r.read_next()
        m = deserialize_message(data, RL)
        if not m.poses:
            continue
        p = m.poses[0].pose.position
        T.append(t); X.append(p.x); Y.append(p.y)
    T = np.array(T); X = np.array(X); Y = np.array(Y)

    moved = np.hypot(X - X[0], Y - Y[0]) > 1.0
    start = int(np.argmax(moved)) if moved.any() else 0
    near = np.hypot(X - X[start], Y - Y[start]) < START_RADIUS
    bounds, outside = [], False
    for i in range(start, len(X)):
        if not near[i]:
            outside = True
        elif outside:
            bounds.append(i); outside = False

    n_laps = len(bounds)
    # lap 1 = start..bounds[0], lap k = bounds[k-2]..bounds[k-1]
    # skip the first `skip_laps` laps, then keep `keep_laps` laps.
    i0 = skip_laps - 1            # boundary index that starts the kept range
    i1 = skip_laps - 1 + keep_laps
    if i0 < 0 or i1 >= len(bounds):
        raise ValueError(
            f"Not enough laps: detected {n_laps} lap boundaries, "
            f"need boundary indices {i0}..{i1}")
    t0, t1 = int(T[bounds[i0]]), int(T[bounds[i1]])
    print(f"  detected {n_laps} lap boundaries")
    print(f"  skip {skip_laps} lap(s), keep {keep_laps} laps "
          f"-> boundaries {i0}..{i1}")
    print(f"  window: {t0} .. {t1}  ({(t1 - t0) / 1e9:.1f} s)")
    return t0, t1


def scan_window_topics(storage_uri, t0, t1):
    """Topics with >=1 message inside [t0,t1], and the last pre-window message
    (data) for every topic (used to carry latched topics forward)."""
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=storage_uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    in_window = set()
    last_before = {}
    while r.has_next():
        topic, data, stamp = r.read_next()
        if stamp < t0:
            last_before[topic] = data
        elif stamp <= t1:
            in_window.add(topic)
    return in_window, last_before


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("in_bag")
    ap.add_argument("out_bag")
    ap.add_argument("--skip-laps", type=int, default=1)
    ap.add_argument("--keep-laps", type=int, default=5)
    args = ap.parse_args()

    _, storage_uri = reader_for(args.in_bag)
    print(f"Input: {storage_uri}")
    t0, t1 = find_window(storage_uri, args.skip_laps, args.keep_laps)

    # Scan pass: which topics actually appear in the window, and the last
    # pre-window message for each topic (to carry latched topics forward).
    in_window, last_before = scan_window_topics(storage_uri, t0, t1)
    carry = {t: d for t, d in last_before.items() if t not in in_window}
    if carry:
        print(f"  carrying forward latched topics: {sorted(carry)}")

    # Copy pass
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=storage_uri, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    topics = r.get_all_topics_and_types()

    if os.path.exists(args.out_bag):
        raise SystemExit(f"Output already exists: {args.out_bag}")
    w = rosbag2_py.SequentialWriter()
    w.open(rosbag2_py.StorageOptions(uri=args.out_bag, storage_id="mcap"),
           rosbag2_py.ConverterOptions("cdr", "cdr"))
    for t in topics:
        w.create_topic(t)

    # Carried latched messages first (stamped at window start) for valid order.
    n_carry = 0
    for tname, tdata in carry.items():
        w.write(tname, tdata, t0)
        n_carry += 1

    n_win = 0
    while r.has_next():
        topic, data, stamp = r.read_next()
        if stamp < t0:
            continue
        if stamp > t1:
            break
        n_win += 1
        w.write(topic, data, stamp)

    del w
    print(f"  wrote {n_win} in-window messages + {n_carry} carried latched messages")
    print(f"  Output bag: {args.out_bag}")


if __name__ == "__main__":
    main()
