#!/usr/bin/env python3
"""Analyze steering authority from an FPGA ROS2 bag.

This script reads a rosbag2 SQLite file directly and compares:
- commanded steering (/drive)
- actual steering state from /mpc_state (steering_angle_fp)
- curvature-implied steering demand from /mpc_state ref_kappa_fp[0]

Usage:
    source /opt/ros/jazzy/setup.bash
    source install/setup.bash
    python3 MPC/test/analyze_fpga_bag_steering.py \
        --bag /home/akselmo/Downloads/bags/FPGADebugging/FPGADebugging_0.db3
"""

from __future__ import annotations

import argparse
import math
import sqlite3
import statistics
from dataclasses import dataclass

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

Q16_16_SCALE = 65536.0


@dataclass
class TopicSample:
    ts_ns: int
    value: float


@dataclass
class DriveSample:
    ts_ns: int
    steer: float
    speed: float
    accel: float


def percentile(values: list[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    i = int(round((len(s) - 1) * p))
    i = max(0, min(len(s) - 1, i))
    return s[i]


def nearest_pairs(
    left: list[TopicSample], right: list[TopicSample], max_dt_s: float
) -> list[tuple[TopicSample, TopicSample, float]]:
    if not left or not right:
        return []

    max_dt_ns = int(max_dt_s * 1e9)
    out: list[tuple[TopicSample, TopicSample, float]] = []
    j = 0

    for l in left:
        while j + 1 < len(right):
            dt0 = abs(right[j].ts_ns - l.ts_ns)
            dt1 = abs(right[j + 1].ts_ns - l.ts_ns)
            if dt1 <= dt0:
                j += 1
            else:
                break

        dt_ns = abs(right[j].ts_ns - l.ts_ns)
        if dt_ns <= max_dt_ns:
            out.append((l, right[j], dt_ns / 1e9))

    return out


def main() -> int:
    parser = argparse.ArgumentParser(description="Analyze steering under-authority in FPGA bags")
    parser.add_argument("--bag", required=True, help="Path to rosbag2 .db3 file")
    parser.add_argument("--wheelbase", type=float, default=0.324, help="Vehicle wheelbase [m]")
    args = parser.parse_args()

    conn = sqlite3.connect(args.bag)
    cur = conn.cursor()

    topics = {tid: (name, ttype) for tid, name, ttype in cur.execute("SELECT id, name, type FROM topics")}
    bag_start_ns = next(cur.execute("SELECT MIN(timestamp) FROM messages"))[0]

    def load_topic(name: str):
        tid = next((k for k, v in topics.items() if v[0] == name), None)
        if tid is None:
            return []
        msg_cls = get_message(topics[tid][1])
        rows = list(cur.execute("SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)))
        out = []
        for ts, data in rows:
            msg = deserialize_message(data, msg_cls)
            out.append((ts, msg))
        return out

    drive_rows = load_topic("/drive")
    mpc_rows = load_topic("/mpc_state")
    ack_rows = load_topic("/ackermann_cmd")

    if not drive_rows:
        print("No /drive messages found.")
        return 1
    if not mpc_rows:
        print("No /mpc_state messages found.")
        return 1

    drive_samples = [
        DriveSample(ts_ns=ts, steer=float(msg.drive.steering_angle), speed=float(msg.drive.speed), accel=float(msg.drive.acceleration))
        for ts, msg in drive_rows
    ]

    mpc_actual = [
        TopicSample(ts_ns=ts, value=float(msg.steering_angle_fp) / Q16_16_SCALE)
        for ts, msg in mpc_rows
    ]

    mpc_demand = []
    for ts, msg in mpc_rows:
        if not msg.ref_kappa_fp:
            continue
        kappa0 = float(msg.ref_kappa_fp[0]) / Q16_16_SCALE
        delta_req = math.atan(args.wheelbase * kappa0)
        mpc_demand.append(TopicSample(ts_ns=ts, value=delta_req))

    drive_t0 = drive_samples[0].ts_ns
    drive_t1 = drive_samples[-1].ts_ns

    mpc_actual_window = [s for s in mpc_actual if drive_t0 <= s.ts_ns <= drive_t1]
    mpc_demand_window = [s for s in mpc_demand if drive_t0 <= s.ts_ns <= drive_t1]

    paired_cmd_actual = nearest_pairs(
        [TopicSample(ts_ns=s.ts_ns, value=s.steer) for s in drive_samples],
        mpc_actual_window,
        max_dt_s=0.01,
    )

    paired_actual_demand = nearest_pairs(
        mpc_actual_window,
        mpc_demand_window,
        max_dt_s=0.01,
    )

    print("=== FPGA Steering Bag Analysis ===")
    print(f"bag: {args.bag}")
    print(f"/drive messages: {len(drive_samples)}")
    print(
        "/drive window: "
        f"{(drive_t0 - bag_start_ns) / 1e9:.3f}s -> {(drive_t1 - bag_start_ns) / 1e9:.3f}s "
        f"(dur {(drive_t1 - drive_t0) / 1e9:.3f}s)"
    )

    cmd_abs = [abs(s.steer) for s in drive_samples]
    print(
        f"commanded steer |delta|: max={max(cmd_abs):.4f} rad "
        f"p95={percentile(cmd_abs, 0.95):.4f} rad"
    )

    if ack_rows:
        ack_vals = [abs(float(msg.drive.steering_angle)) for _, msg in ack_rows]
        print(
            f"ackermann_cmd |delta|: max={max(ack_vals):.4f} rad "
            f"p95={percentile(ack_vals, 0.95):.4f} rad"
        )

    if mpc_actual_window:
        act_abs = [abs(s.value) for s in mpc_actual_window]
        print(
            f"actual steer |delta_actual|: max={max(act_abs):.4f} rad "
            f"p95={percentile(act_abs, 0.95):.4f} rad"
        )

    if mpc_demand_window:
        dem_abs = [abs(s.value) for s in mpc_demand_window]
        print(
            f"curvature demand |atan(L*kappa0)|: max={max(dem_abs):.4f} rad "
            f"p95={percentile(dem_abs, 0.95):.4f} rad"
        )

    if paired_cmd_actual:
        errs = [left.value - right.value for left, right, _ in paired_cmd_actual]
        abs_errs = [abs(e) for e in errs]
        print(
            f"cmd-vs-actual pairs: {len(paired_cmd_actual)} "
            f"mean(cmd-actual)={statistics.mean(errs):+.4f} rad "
            f"mean|cmd-actual|={statistics.mean(abs_errs):.4f} rad "
            f"max|cmd-actual|={max(abs_errs):.4f} rad"
        )

    if paired_actual_demand:
        ratios = []
        for actual, demand, _ in paired_actual_demand:
            if abs(demand.value) > 0.03:
                ratios.append(abs(actual.value) / abs(demand.value))
        if ratios:
            print(
                "actual-vs-demand ratio for |demand|>0.03: "
                f"mean={statistics.mean(ratios):.3f} "
                f"median={statistics.median(ratios):.3f} "
                f"p95={percentile(ratios, 0.95):.3f}"
            )

    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
