#!/usr/bin/env python3
"""Extract all identification-relevant signals from a ROS 2 MCAP bag.

Outputs use bag-record time as the common clock (`t_ns`, `t_s`). Header time is
preserved separately for timing diagnostics. The script also stores raw /scan
ranges and the single OccupancyGrid map found in this bag.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd

from mcap_io import McapReader, decode_message


TOPIC_ALIASES = {
    "/ekf_pose": "ekf_pose",
    "/amcl_pose": "amcl_pose",
    "/odom_pose": "odom_pose",
    "/ego_racecar/odom": "odom",
    "/drive": "drive",
    "/ackermann_cmd": "ackermann_cmd",
    "/commands/servo/position": "servo_command",
    "/sensors/servo_position_command": "servo_feedback",
    "/imu/filtered_angular_velocity": "imu_filtered_yaw_rate",
    "/sensors/imu/raw": "imu_raw",
    "/scan": "scan",
    "/map": "map",
    "/tf_static": "tf_static",
}

MPC_TIMING_TOPICS = {
    "/mpc/timing/pose_seq",
    "/mpc/timing/solve_us",
    "/mpc/timing/iteration_count",
    "/mpc/timing/solver_enter_seq",
    "/mpc/timing/control_gap_ms",
    "/mpc/timing/ekf_to_control_ms",
    "/mpc/timing/output_gap_ms",
    "/mpc/timing/drive_age_ms",
    "/mpc/timing/skipped_poses",
}


def relative_time_ns(log_time_ns: int, bag_start_ns: int) -> float:
    return (log_time_ns - bag_start_ns) * 1e-9


def common_row(message, decoded: dict[str, Any], bag_start_ns: int) -> dict[str, Any]:
    header_ns = int(decoded.get("stamp_ns", 0))
    return {
        "t_s": relative_time_ns(message.log_time_ns, bag_start_ns),
        "t_ns": message.log_time_ns,
        "header_stamp_ns": header_ns,
        "header_minus_bag_ms": (header_ns - message.log_time_ns) * 1e-6 if header_ns else np.nan,
    }


def pose_row(message, decoded: dict[str, Any], bag_start_ns: int) -> dict[str, Any]:
    row = common_row(message, decoded, bag_start_ns)
    cov = decoded.get("covariance", np.full(36, np.nan))
    row.update({
        "frame_id": decoded.get("frame_id", ""),
        "x": decoded["x"], "y": decoded["y"], "z": decoded["z"],
        "qx": decoded["qx"], "qy": decoded["qy"], "qz": decoded["qz"], "qw": decoded["qw"],
        "yaw": decoded["yaw"],
        "cov_x": float(cov[0]), "cov_y": float(cov[7]), "cov_yaw": float(cov[35]),
    })
    return row


def odom_row(message, decoded: dict[str, Any], bag_start_ns: int) -> dict[str, Any]:
    row = common_row(message, decoded, bag_start_ns)
    pose_cov = decoded["pose_covariance"]
    twist_cov = decoded["twist_covariance"]
    row.update({
        "frame_id": decoded.get("frame_id", ""), "child_frame_id": decoded.get("child_frame_id", ""),
        "x": decoded["pose_x"], "y": decoded["pose_y"], "z": decoded["pose_z"],
        "yaw": decoded["pose_yaw"],
        "vx": decoded["twist_vx"], "vy": decoded["twist_vy"], "vz": decoded["twist_vz"],
        "wx": decoded["twist_wx"], "wy": decoded["twist_wy"], "wz": decoded["twist_wz"],
        "pose_cov_x": float(pose_cov[0]), "pose_cov_y": float(pose_cov[7]), "pose_cov_yaw": float(pose_cov[35]),
        "twist_cov_vx": float(twist_cov[0]), "twist_cov_vy": float(twist_cov[7]), "twist_cov_wz": float(twist_cov[35]),
    })
    return row


def ackermann_row(message, decoded: dict[str, Any], bag_start_ns: int) -> dict[str, Any]:
    row = common_row(message, decoded, bag_start_ns)
    row.update({"frame_id": decoded.get("frame_id", ""), **{k: decoded[k] for k in [
        "steering_angle", "steering_angle_velocity", "speed", "acceleration", "jerk"
    ]}})
    return row


def float_row(message, decoded: dict[str, Any], bag_start_ns: int) -> dict[str, Any]:
    row = common_row(message, decoded, bag_start_ns)
    row["value"] = decoded["value"]
    return row


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    pd.DataFrame(rows).to_csv(path, index=False)


def write_local_raceline_snapshot(writer: csv.writer, seq: int, message, path_msg: dict[str, Any]) -> None:
    """Match the hardware-node local raceline CSV schema for later closed-loop replay."""
    poses = path_msg["poses"]
    if len(poses) < 2:
        return
    xy = np.array([[p["x"], p["y"]] for p in poses], dtype=float)
    ds = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    s = np.r_[0.0, np.cumsum(ds)]
    headings = np.empty(len(poses), dtype=float)
    for i in range(len(poses)):
        i0 = max(0, i - 1)
        i1 = min(len(poses) - 1, i + 1)
        dx, dy = xy[i1] - xy[i0]
        headings[i] = headings[i - 1] if i and dx * dx + dy * dy < 1e-12 else np.arctan2(dy, dx)
    headings = np.unwrap(headings)
    kappas = np.zeros(len(poses))
    if len(poses) >= 3:
        for i in range(1, len(poses) - 1):
            denom = max(s[i + 1] - s[i - 1], 1e-6)
            kappas[i] = (headings[i + 1] - headings[i - 1]) / denom
        kappas[0], kappas[-1] = kappas[1], kappas[-2]
    for i, p in enumerate(poses):
        # The planner encodes v_ref in z, left/right bounds in orientation x/y;
        # this matches local_raceline_callback() in mpc_hardware_node.c.
        writer.writerow([
            seq, message.log_time_ns, i, len(poses), s[i], p["x"], p["y"],
            headings[i], kappas[i], abs(p["z"]), p["qx"], p["qy"],
        ])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="Path to shortened_baseline_0.mcap")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--include-local-raceline", action="store_true",
                        help="Write all /local_raceline snapshots (large CSV).")
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    rows: dict[str, list[dict[str, Any]]] = defaultdict(list)
    scans: list[np.ndarray] = []
    scan_meta: list[dict[str, Any]] = []
    tf_rows: list[dict[str, Any]] = []
    map_msg = None
    summary = Counter()
    type_summary: dict[str, str] = {}
    bag_start_ns = None
    local_file = None
    local_writer = None
    local_seq = 0
    if args.include_local_raceline:
        local_file = (args.output / "local_raceline_snapshots.csv").open("w", newline="")
        local_writer = csv.writer(local_file)
        local_writer.writerow([
            "seq", "ros_time_ns", "waypoint_index", "waypoint_count", "s_meters", "x_meters", "y_meters",
            "heading_radians", "curvature_radians_per_meter", "velocity_meters_per_second",
            "left_bound_meters", "right_bound_meters",
        ])

    try:
        for message in McapReader(args.bag).messages():
            if bag_start_ns is None:
                bag_start_ns = message.log_time_ns
            summary[message.topic] += 1
            type_summary[message.topic] = message.type_name
            selected = message.topic in TOPIC_ALIASES or message.topic in MPC_TIMING_TOPICS or (
                args.include_local_raceline and message.topic == "/local_raceline"
            )
            if not selected:
                continue
            decoded = decode_message(message.type_name, message.data)
            if decoded is None:
                continue
            if message.topic in ("/ekf_pose", "/amcl_pose", "/odom_pose"):
                rows[TOPIC_ALIASES[message.topic]].append(pose_row(message, decoded, bag_start_ns))
            elif message.topic == "/ego_racecar/odom":
                rows["odom"].append(odom_row(message, decoded, bag_start_ns))
            elif message.topic in ("/drive", "/ackermann_cmd"):
                rows[TOPIC_ALIASES[message.topic]].append(ackermann_row(message, decoded, bag_start_ns))
            elif message.topic in ("/commands/servo/position", "/sensors/servo_position_command", "/imu/filtered_angular_velocity"):
                rows[TOPIC_ALIASES[message.topic]].append(float_row(message, decoded, bag_start_ns))
            elif message.topic in MPC_TIMING_TOPICS:
                row = float_row(message, decoded, bag_start_ns)
                row["topic"] = message.topic
                rows["mpc_timing"].append(row)
            elif message.topic == "/sensors/imu/raw":
                row = common_row(message, decoded, bag_start_ns)
                row.update({k: decoded[k] for k in ["frame_id", "yaw", "av_x", "av_y", "av_z", "la_x", "la_y", "la_z"]})
                rows["imu_raw"].append(row)
            elif message.topic == "/scan":
                ranges = decoded["ranges"]
                scans.append(ranges)
                scan_meta.append({
                    **common_row(message, decoded, bag_start_ns),
                    "frame_id": decoded["frame_id"],
                    **{k: decoded[k] for k in ["angle_min", "angle_max", "angle_increment", "time_increment", "scan_time", "range_min", "range_max"]},
                    "range_count": int(ranges.size),
                })
            elif message.topic == "/map":
                map_msg = decoded
            elif message.topic == "/tf_static":
                for transform in decoded:
                    row = common_row(message, transform, bag_start_ns)
                    row.update(transform)
                    tf_rows.append(row)
            elif args.include_local_raceline and message.topic == "/local_raceline":
                write_local_raceline_snapshot(local_writer, local_seq, message, decoded)
                local_seq += 1
    finally:
        if local_file is not None:
            local_file.close()

    if bag_start_ns is None:
        raise RuntimeError("No CDR messages found in MCAP")

    for name, values in rows.items():
        write_csv(args.output / f"{name}.csv", values)
    write_csv(args.output / "scan_index.csv", scan_meta)
    write_csv(args.output / "tf_static.csv", tf_rows)

    if scans:
        lengths = {scan.size for scan in scans}
        if len(lengths) != 1:
            raise RuntimeError(f"/scan range count changes across bag: {sorted(lengths)}")
        np.savez_compressed(args.output / "scans.npz", ranges=np.stack(scans).astype(np.float32))

    if map_msg is None:
        raise RuntimeError("No /map OccupancyGrid found in MCAP; cannot run scan-to-map ICP")
    origin = map_msg["origin"]
    np.savez_compressed(
        args.output / "map.npz",
        grid=map_msg["grid"].astype(np.int8),
        resolution=np.float64(map_msg["resolution"]),
        width=np.int32(map_msg["width"]), height=np.int32(map_msg["height"]),
        origin_x=np.float64(origin["x"]), origin_y=np.float64(origin["y"]), origin_yaw=np.float64(origin["yaw"]),
        frame_id=np.array(map_msg["frame_id"]),
    )

    manifest = {
        "bag": str(args.bag.resolve()), "bag_start_ns": bag_start_ns,
        "topics": {topic: {"count": int(summary[topic]), "type": type_summary.get(topic, "")} for topic in sorted(summary)},
        "scan_count": len(scans), "scan_shape": list(scans[0].shape) if scans else None,
        "map": {
            "frame_id": map_msg["frame_id"], "resolution_m": float(map_msg["resolution"]),
            "width": int(map_msg["width"]), "height": int(map_msg["height"]),
            "origin": {"x": float(origin["x"]), "y": float(origin["y"]), "yaw": float(origin["yaw"])},
        },
    }
    (args.output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
