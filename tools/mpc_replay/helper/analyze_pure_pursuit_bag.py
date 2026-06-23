#!/usr/bin/env python3
"""
Analyze a pure-pursuit rosbag2 MCAP recording and summarize race laps 2..N.

Outputs:
  - summary.txt      high-level run metrics for kept laps
  - lap_stats.csv    one row per kept lap
  - trajectory.png   XY trajectory plot for laps 2..N

Default topics match the pure-pursuit ground-truth bags used in this repo:
  /ekf_pose          map-frame vehicle pose
  /ego_racecar/odom  vehicle velocity
  /drive             commanded speed + steering (optional diagnostics)
"""

import argparse
import bisect
import csv
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


POSE_TOPIC = "/ekf_pose"
ODOM_TOPIC = "/ego_racecar/odom"
DRIVE_TOPIC = "/drive"


def resolve_bag_uri(path_str: str) -> str:
    path = Path(path_str).expanduser().resolve()
    if path.is_dir():
        return str(path)
    if path.is_file() and path.suffix == ".mcap":
        return str(path)
    raise SystemExit(f"bag path does not exist or is unsupported: {path}")


def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    """Match the lap detection used by the existing MPC replay helpers."""
    n = len(pos_x)
    if n < 2:
        return [(0, n)]
    sx, sy = pos_x[0], pos_y[0]
    dist = np.sqrt((pos_x - sx) ** 2 + (pos_y - sy) ** 2)
    seg = np.sqrt(np.diff(pos_x) ** 2 + np.diff(pos_y) ** 2)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    starts = [0]
    last_path = 0.0
    in_zone = True
    for i in range(1, n):
        inside = dist[i] < min_radius
        if inside and not in_zone and (cum[i] - last_path) > min_lap_length:
            starts.append(i)
            last_path = cum[i]
        in_zone = inside
    starts.append(n)
    return [(starts[i], starts[i + 1]) for i in range(len(starts) - 1)]


def nearest_value(events, stamps, stamp_ns, max_dt_ns=None):
    if not events:
        return None
    idx = bisect.bisect_left(stamps, stamp_ns)
    candidates = []
    if idx < len(events):
        candidates.append(events[idx])
    if idx > 0:
        candidates.append(events[idx - 1])
    if not candidates:
        return None
    best = min(candidates, key=lambda e: abs(e[0] - stamp_ns))
    if max_dt_ns is not None and abs(best[0] - stamp_ns) > max_dt_ns:
        return None
    return best


def load_bag(uri, pose_topic, odom_topic, drive_topic):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    for required in (pose_topic, odom_topic):
        if required not in type_map:
            raise SystemExit(f"required topic missing from bag: {required}")

    pose_type = get_message(type_map[pose_topic])
    odom_type = get_message(type_map[odom_topic])
    drive_type = get_message(type_map[drive_topic]) if drive_topic in type_map else None

    poses = []
    odoms = []
    drives = []

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        stamp_ns = int(stamp_ns)
        if topic == pose_topic:
            msg = deserialize_message(data, pose_type)
            poses.append((
                stamp_ns,
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
            ))
        elif topic == odom_topic:
            msg = deserialize_message(data, odom_type)
            vx = float(msg.twist.twist.linear.x)
            vy = float(msg.twist.twist.linear.y)
            odoms.append((
                stamp_ns,
                math.hypot(vx, vy),
                vx,
                vy,
            ))
        elif drive_type is not None and topic == drive_topic:
            msg = deserialize_message(data, drive_type)
            drives.append((
                stamp_ns,
                float(msg.drive.speed),
                float(msg.drive.steering_angle),
            ))

    if not poses:
        raise SystemExit(f"no pose messages found on {pose_topic}")
    if not odoms:
        raise SystemExit(f"no odom messages found on {odom_topic}")

    return poses, odoms, drives


def compute_path_length(x_vals, y_vals):
    if len(x_vals) < 2:
        return 0.0
    dx = np.diff(x_vals)
    dy = np.diff(y_vals)
    return float(np.sum(np.hypot(dx, dy)))


def plot_trajectory(poses, kept_laps, out_path):
    fig, ax = plt.subplots(figsize=(9, 9))
    cmap = plt.get_cmap("tab10")
    for lap_offset, (lap_num, lap_slice) in enumerate(kept_laps):
        start_idx, end_idx = lap_slice
        xs = np.asarray([poses[i][1] for i in range(start_idx, end_idx)])
        ys = np.asarray([poses[i][2] for i in range(start_idx, end_idx)])
        color = cmap((lap_offset % 10) / 9.0 if len(kept_laps) > 1 else 0.0)
        ax.plot(xs, ys, linewidth=1.6, color=color, label=f"Lap {lap_num}")
        ax.scatter(xs[0], ys[0], s=18, color=color)

    start_x, start_y = poses[0][1], poses[0][2]
    ax.scatter([start_x], [start_y], marker="x", s=80, color="black", label="Start zone")
    ax.set_title("Pure Pursuit Trajectory, Laps 2-11")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9, ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True,
                    help="Path to the rosbag2 directory or .mcap file")
    ap.add_argument("--out-dir", required=True,
                    help="Directory for summary, CSV, and trajectory plot")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Keep race laps 2..MAX_LAP (default 11)")
    ap.add_argument("--pose-topic", default=POSE_TOPIC)
    ap.add_argument("--odom-topic", default=ODOM_TOPIC)
    ap.add_argument("--drive-topic", default=DRIVE_TOPIC)
    ap.add_argument("--max-join-dt-ms", type=float, default=50.0,
                    help="Maximum pose-to-odom/drive join offset in ms")
    args = ap.parse_args()

    bag_uri = resolve_bag_uri(args.bag)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    poses, odoms, drives = load_bag(
        bag_uri,
        args.pose_topic,
        args.odom_topic,
        args.drive_topic,
    )

    pos_x = np.asarray([p[1] for p in poses], dtype=np.float64)
    pos_y = np.asarray([p[2] for p in poses], dtype=np.float64)
    laps = detect_laps(pos_x, pos_y)
    kept_laps = [(lap_num, laps[lap_num - 1])
                 for lap_num in range(2, args.max_lap + 1)
                 if lap_num <= len(laps)]
    if not kept_laps:
        raise SystemExit(
            f"no race laps found in range 2..{args.max_lap}; detected {len(laps)} laps"
        )

    max_dt_ns = int(args.max_join_dt_ms * 1e6)
    odom_stamps = [o[0] for o in odoms]
    drive_stamps = [d[0] for d in drives]

    lap_rows = []
    all_speeds = []
    all_cmd_speeds = []
    all_cmd_steers = []

    for lap_num, (start_idx, end_idx) in kept_laps:
        lap_pose_times = np.asarray([poses[i][0] for i in range(start_idx, end_idx)], dtype=np.int64)
        lap_x = np.asarray([poses[i][1] for i in range(start_idx, end_idx)], dtype=np.float64)
        lap_y = np.asarray([poses[i][2] for i in range(start_idx, end_idx)], dtype=np.float64)

        lap_speeds = []
        lap_cmd_speeds = []
        lap_cmd_steers = []

        for stamp_ns in lap_pose_times:
            odom_event = nearest_value(odoms, odom_stamps, int(stamp_ns), max_dt_ns)
            if odom_event is not None:
                lap_speeds.append(float(odom_event[1]))
            drive_event = nearest_value(drives, drive_stamps, int(stamp_ns), max_dt_ns)
            if drive_event is not None:
                lap_cmd_speeds.append(float(drive_event[1]))
                lap_cmd_steers.append(float(drive_event[2]))

        lap_duration_s = float((lap_pose_times[-1] - lap_pose_times[0]) * 1e-9)
        lap_distance_m = compute_path_length(lap_x, lap_y)
        lap_mean_speed = float(np.mean(lap_speeds)) if lap_speeds else float("nan")
        lap_avg_speed_by_distance = (lap_distance_m / lap_duration_s) if lap_duration_s > 1e-9 else float("nan")
        lap_rows.append({
            "lap": lap_num,
            "start_stamp_ns": int(lap_pose_times[0]),
            "end_stamp_ns": int(lap_pose_times[-1]),
            "duration_s": lap_duration_s,
            "path_length_m": lap_distance_m,
            "mean_speed_mps": lap_mean_speed,
            "median_speed_mps": float(np.median(lap_speeds)) if lap_speeds else float("nan"),
            "std_speed_mps": float(np.std(lap_speeds)) if lap_speeds else float("nan"),
            "min_speed_mps": float(np.min(lap_speeds)) if lap_speeds else float("nan"),
            "max_speed_mps": float(np.max(lap_speeds)) if lap_speeds else float("nan"),
            "avg_speed_from_distance_mps": lap_avg_speed_by_distance,
            "mean_command_speed_mps": float(np.mean(lap_cmd_speeds)) if lap_cmd_speeds else float("nan"),
            "mean_command_steering_rad": float(np.mean(lap_cmd_steers)) if lap_cmd_steers else float("nan"),
            "samples": int(len(lap_pose_times)),
        })
        all_speeds.extend(lap_speeds)
        all_cmd_speeds.extend(lap_cmd_speeds)
        all_cmd_steers.extend(lap_cmd_steers)

    summary = {
        "bag_uri": bag_uri,
        "detected_laps_total": len(laps),
        "kept_laps_start": kept_laps[0][0],
        "kept_laps_end": kept_laps[-1][0],
        "kept_lap_count": len(kept_laps),
        "mean_velocity_mps": float(np.mean(all_speeds)),
        "median_velocity_mps": float(np.median(all_speeds)),
        "std_velocity_mps": float(np.std(all_speeds)),
        "min_velocity_mps": float(np.min(all_speeds)),
        "max_velocity_mps": float(np.max(all_speeds)),
        "p95_velocity_mps": float(np.percentile(all_speeds, 95.0)),
        "mean_lap_time_s": float(np.mean([r["duration_s"] for r in lap_rows])),
        "std_lap_time_s": float(np.std([r["duration_s"] for r in lap_rows])),
        "min_lap_time_s": float(np.min([r["duration_s"] for r in lap_rows])),
        "max_lap_time_s": float(np.max([r["duration_s"] for r in lap_rows])),
        "total_kept_distance_m": float(sum(r["path_length_m"] for r in lap_rows)),
        "mean_path_length_m": float(np.mean([r["path_length_m"] for r in lap_rows])),
        "mean_command_speed_mps": float(np.mean(all_cmd_speeds)) if all_cmd_speeds else float("nan"),
        "mean_command_steering_rad": float(np.mean(all_cmd_steers)) if all_cmd_steers else float("nan"),
    }

    csv_path = out_dir / "lap_stats.csv"
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(lap_rows[0].keys()))
        writer.writeheader()
        writer.writerows(lap_rows)

    summary_path = out_dir / "summary.txt"
    summary_lines = [
        f"Bag: {summary['bag_uri']}",
        f"Detected laps: {summary['detected_laps_total']}",
        f"Kept race laps: {summary['kept_laps_start']}..{summary['kept_laps_end']} ({summary['kept_lap_count']} laps)",
        "",
        f"Mean velocity [m/s]: {summary['mean_velocity_mps']:.6f}",
        f"Median velocity [m/s]: {summary['median_velocity_mps']:.6f}",
        f"Velocity std [m/s]: {summary['std_velocity_mps']:.6f}",
        f"Velocity min/max [m/s]: {summary['min_velocity_mps']:.6f} / {summary['max_velocity_mps']:.6f}",
        f"Velocity p95 [m/s]: {summary['p95_velocity_mps']:.6f}",
        "",
        f"Mean lap time [s]: {summary['mean_lap_time_s']:.6f}",
        f"Lap time std [s]: {summary['std_lap_time_s']:.6f}",
        f"Lap time min/max [s]: {summary['min_lap_time_s']:.6f} / {summary['max_lap_time_s']:.6f}",
        "",
        f"Total kept distance [m]: {summary['total_kept_distance_m']:.6f}",
        f"Mean lap path length [m]: {summary['mean_path_length_m']:.6f}",
    ]
    if all_cmd_speeds:
        summary_lines.append(f"Mean command speed [m/s]: {summary['mean_command_speed_mps']:.6f}")
    if all_cmd_steers:
        summary_lines.append(f"Mean command steering [rad]: {summary['mean_command_steering_rad']:.6f}")
    summary_path.write_text("\n".join(summary_lines) + "\n")

    plot_path = out_dir / "trajectory_laps_2_11.png"
    plot_trajectory(poses, kept_laps, plot_path)

    print("\n".join(summary_lines))
    print()
    print(f"Wrote: {csv_path}")
    print(f"Wrote: {summary_path}")
    print(f"Wrote: {plot_path}")


if __name__ == "__main__":
    raise SystemExit(main())
