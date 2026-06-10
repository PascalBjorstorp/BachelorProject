#!/usr/bin/env python3
"""Estimate a fixed OptiTrack map correction from lidar scan-to-map alignment.

The correction is a 2D SE(2) transform applied after the logged map<-world
OptiTrack transform:

    T_map_world_corrected = T_delta * T_map_world_logged

This is intended as an offline calibration check, not as an independent AMCL
ground-truth source. The lidar and map are the same sensing model used by AMCL.
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
import pandas as pd
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from scipy import optimize
from scipy.ndimage import distance_transform_edt, map_coordinates

from analyze_real_vs_sim_amcl_equivalence import (
    PARTICLES,
    filter_optitrack,
    first_available_tf,
    interp_yaw,
    qmul,
    quat_to_rot,
    read_map_world_tf,
    read_pose_series,
    real_bag_for_particles,
    roi_mask,
    yaw_from_quat,
)


@dataclass
class MapDistance:
    distance_m: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


@dataclass
class ScanMatchSamples:
    particles: int
    bag: str
    tf_source: str
    pose_x: np.ndarray
    pose_y: np.ndarray
    pose_yaw: np.ndarray
    point_x_base: np.ndarray
    point_y_base: np.ndarray
    map_distance: MapDistance
    n_scans: int


def qnorm(q: Sequence[float]) -> np.ndarray:
    q_arr = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q_arr)
    return q_arr / norm if norm > 0.0 else q_arr


def open_reader(bag_dir: Path) -> Tuple[rosbag2_py.SequentialReader, Dict[str, str]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    return reader, topic_types


def read_map_distance(bag_dir: Path) -> MapDistance:
    reader, topic_types = open_reader(bag_dir)
    if "/map" not in topic_types:
        raise RuntimeError(f"{bag_dir} has no /map topic")
    msg_type = get_message(topic_types["/map"])
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/map":
            continue
        msg = deserialize_message(data, msg_type)
        info = msg.info
        width = int(info.width)
        height = int(info.height)
        resolution = float(info.resolution)
        origin_x = float(info.origin.position.x)
        origin_y = float(info.origin.position.y)
        grid = np.asarray(msg.data, dtype=np.int16)[:width * height].reshape(height, width)
        occupied = grid >= 50
        distance_m = distance_transform_edt(~occupied) * resolution
        return MapDistance(distance_m, resolution, origin_x, origin_y, width, height)
    raise RuntimeError(f"{bag_dir} /map topic is empty")


def read_base_to_laser(bag_dir: Path) -> Tuple[np.ndarray, float]:
    reader, topic_types = open_reader(bag_dir)
    if "/tf_static" not in topic_types:
        return np.array([0.265, 0.0], dtype=float), 0.0
    msg_type = get_message(topic_types["/tf_static"])
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != "/tf_static":
            continue
        msg = deserialize_message(data, msg_type)
        for transform in msg.transforms:
            parent = transform.header.frame_id.strip("/")
            child = transform.child_frame_id.strip("/")
            if parent.endswith("base_link") and child.endswith("laser"):
                trans = transform.transform.translation
                rot = transform.transform.rotation
                return (
                    np.array([trans.x, trans.y], dtype=float),
                    yaw_from_quat([rot.x, rot.y, rot.z, rot.w]),
                )
    return np.array([0.265, 0.0], dtype=float), 0.0


def read_scan_series(
    bag_dir: Path,
    scan_topic: str,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, float, float]:
    reader, topic_types = open_reader(bag_dir)
    if scan_topic not in topic_types:
        raise RuntimeError(f"{bag_dir} has no {scan_topic} topic")
    msg_type = get_message(topic_types[scan_topic])
    stamps: List[float] = []
    ranges: List[np.ndarray] = []
    angles: np.ndarray | None = None
    range_min = 0.0
    range_max = math.inf
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic != scan_topic:
            continue
        msg = deserialize_message(data, msg_type)
        stamp = msg.header.stamp
        stamps.append(stamp.sec + stamp.nanosec * 1e-9)
        scan_ranges = np.asarray(msg.ranges, dtype=float)
        ranges.append(scan_ranges)
        if angles is None:
            angles = float(msg.angle_min) + np.arange(scan_ranges.size) * float(msg.angle_increment)
            range_min = float(msg.range_min)
            range_max = float(msg.range_max)
    if angles is None:
        raise RuntimeError(f"{bag_dir} {scan_topic} topic is empty")
    return np.asarray(stamps, dtype=float), np.vstack(ranges), angles, range_min, range_max


def correction_pose(
    x: np.ndarray,
    y: np.ndarray,
    yaw: np.ndarray,
    params: Sequence[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    dx, dy, dtheta = params
    c = math.cos(dtheta)
    s = math.sin(dtheta)
    x_out = c * x - s * y + dx
    y_out = s * x + c * y + dy
    yaw_out = yaw + dtheta
    return x_out, y_out, yaw_out


def scan_distance_cost(params: Sequence[float], datasets: Sequence[ScanMatchSamples]) -> float:
    values: List[np.ndarray] = []
    for data in datasets:
        x_pose, y_pose, yaw = correction_pose(data.pose_x, data.pose_y, data.pose_yaw, params)
        cy = np.cos(yaw)
        sy = np.sin(yaw)
        x_map = x_pose + cy * data.point_x_base - sy * data.point_y_base
        y_map = y_pose + sy * data.point_x_base + cy * data.point_y_base

        md = data.map_distance
        col = (x_map - md.origin_x) / md.resolution
        row = (y_map - md.origin_y) / md.resolution
        in_bounds = (col >= 0.0) & (col <= md.width - 1) & (row >= 0.0) & (row <= md.height - 1)
        if not np.any(in_bounds):
            return 10.0
        dist = np.full(x_map.shape, 1.0, dtype=float)
        dist[in_bounds] = map_coordinates(
            md.distance_m,
            [row[in_bounds], col[in_bounds]],
            order=1,
            mode="nearest",
        )
        values.append(np.minimum(dist, 0.5))

    all_dist = np.concatenate(values)
    if all_dist.size == 0:
        return 10.0
    # A trimmed mean keeps dynamic returns and occasional invalid map hits from
    # dominating the transform estimate.
    cutoff = np.percentile(all_dist, 80)
    kept = all_dist[all_dist <= cutoff]
    return float(np.mean(kept))


def optimize_correction(datasets: Sequence[ScanMatchSamples]) -> Tuple[np.ndarray, float, float]:
    zero = np.array([0.0, 0.0, 0.0], dtype=float)
    initial_cost = scan_distance_cost(zero, datasets)

    best = zero.copy()
    best_cost = initial_cost
    starts = [
        zero,
        np.array([0.00, 0.10, 0.0]),
        np.array([0.00, -0.10, 0.0]),
        np.array([0.10, 0.00, 0.0]),
        np.array([-0.10, 0.00, 0.0]),
        np.array([0.25, 0.05, 0.0]),
        np.array([-0.25, -0.05, 0.0]),
        np.array([0.00, 0.00, math.radians(2.0)]),
        np.array([0.00, 0.00, math.radians(-2.0)]),
    ]
    for start in starts:
        result = optimize.minimize(
            lambda p: scan_distance_cost(p, datasets),
            start,
            method="Powell",
            bounds=[(-0.5, 0.5), (-0.5, 0.5), (math.radians(-8.0), math.radians(8.0))],
            options={"xtol": 2e-4, "ftol": 2e-5, "maxiter": 80},
        )
        if result.success and float(result.fun) <= best_cost:
            best = np.asarray(result.x, dtype=float)
            best_cost = float(result.fun)
    return best, initial_cost, best_cost


def bag_samples(
    real_root: Path,
    particles: int,
    roi: Tuple[float, float, float, float],
    scan_topic: str,
    fallback_tf: Tuple[np.ndarray, np.ndarray],
    min_speed_mps: float,
    max_scans: int,
    max_beams_per_scan: int,
) -> ScanMatchSamples:
    bag_dir = real_bag_for_particles(real_root, particles)
    tf = read_map_world_tf(bag_dir)
    tf_source = "bag /tf_static"
    if tf is None:
        tf = fallback_tf
        tf_source = "fallback /tf_static"
    trans, q_tf = tf
    rot = quat_to_rot(q_tf)

    t_opti, opti_raw, q_opti_raw = read_pose_series(bag_dir, "/vrpn_mocap/Car2/pose")
    t_opti, opti_raw, q_opti_raw = filter_optitrack(t_opti, opti_raw, q_opti_raw)
    gt_pos = (rot @ opti_raw.T).T + trans
    gt_yaw = np.array([yaw_from_quat(qmul(q_tf, q)) for q in q_opti_raw])
    speed = np.r_[
        np.nan,
        np.linalg.norm(np.diff(gt_pos[:, :2], axis=0), axis=1) /
        np.maximum(np.diff(t_opti), np.finfo(float).eps),
    ]

    t_scan, ranges, angles, range_min, range_max = read_scan_series(bag_dir, scan_topic)
    t0 = max(float(np.min(t_opti)), float(np.min(t_scan)))
    t1 = min(float(np.max(t_opti)), float(np.max(t_scan)))
    valid_time = (t_scan >= t0) & (t_scan <= t1)
    t_scan = t_scan[valid_time]
    ranges = ranges[valid_time]

    pose_x = np.interp(t_scan, t_opti, gt_pos[:, 0])
    pose_y = np.interp(t_scan, t_opti, gt_pos[:, 1])
    pose_yaw = interp_yaw(t_opti, gt_yaw, t_scan)
    scan_speed = np.interp(t_scan, t_opti, np.nan_to_num(speed, nan=0.0))
    mask = roi_mask(pose_x, pose_y, roi) & (scan_speed >= min_speed_mps)
    scan_idx = np.flatnonzero(mask)
    if scan_idx.size > max_scans:
        scan_idx = scan_idx[np.linspace(0, scan_idx.size - 1, max_scans).round().astype(int)]

    laser_offset, laser_yaw = read_base_to_laser(bag_dir)
    cl = math.cos(laser_yaw)
    sl = math.sin(laser_yaw)
    pose_x_rows: List[np.ndarray] = []
    pose_y_rows: List[np.ndarray] = []
    pose_yaw_rows: List[np.ndarray] = []
    point_x_rows: List[np.ndarray] = []
    point_y_rows: List[np.ndarray] = []
    for i in scan_idx:
        r = ranges[i]
        hit = (
            np.isfinite(r)
            & (r > range_min + 0.02)
            & (r < min(range_max - 0.05, 8.0))
        )
        valid = np.flatnonzero(hit)
        if valid.size == 0:
            continue
        if valid.size > max_beams_per_scan:
            valid = valid[np.linspace(0, valid.size - 1, max_beams_per_scan).round().astype(int)]
        lx = r[valid] * np.cos(angles[valid])
        ly = r[valid] * np.sin(angles[valid])
        bx = laser_offset[0] + cl * lx - sl * ly
        by = laser_offset[1] + sl * lx + cl * ly
        n = bx.size
        pose_x_rows.append(np.full(n, pose_x[i], dtype=float))
        pose_y_rows.append(np.full(n, pose_y[i], dtype=float))
        pose_yaw_rows.append(np.full(n, pose_yaw[i], dtype=float))
        point_x_rows.append(bx)
        point_y_rows.append(by)

    if not pose_x_rows:
        raise RuntimeError(f"No usable ROI scan points for {bag_dir}")

    return ScanMatchSamples(
        particles=particles,
        bag=bag_dir.name,
        tf_source=tf_source,
        pose_x=np.concatenate(pose_x_rows),
        pose_y=np.concatenate(pose_y_rows),
        pose_yaw=np.concatenate(pose_yaw_rows),
        point_x_base=np.concatenate(point_x_rows),
        point_y_base=np.concatenate(point_y_rows),
        map_distance=read_map_distance(bag_dir),
        n_scans=int(len(scan_idx)),
    )


def summarize_result(label: str, params: np.ndarray, initial_cost: float, final_cost: float) -> Dict[str, float | str]:
    return {
        "label": label,
        "dx_m": float(params[0]),
        "dy_m": float(params[1]),
        "dyaw_deg": float(math.degrees(params[2])),
        "initial_trimmed_scan_distance_cm": float(100.0 * initial_cost),
        "corrected_trimmed_scan_distance_cm": float(100.0 * final_cost),
        "improvement_cm": float(100.0 * (initial_cost - final_cost)),
    }


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_real_root = repo_root / "f1tenth_localization" / "Benchmark" / "bags" / "OptitrackBags" / "AMCL"
    default_output = repo_root / "f1tenth_localization" / "Benchmark" / "Matlab" / "plots" / "OptiTrackScanMapCalibration"
    parser = argparse.ArgumentParser()
    parser.add_argument("--real-root", type=Path, default=default_real_root)
    parser.add_argument("--output-dir", type=Path, default=default_output)
    parser.add_argument("--scan-topic", default="/scan")
    parser.add_argument("--roi-min-x", type=float, default=0.0)
    parser.add_argument("--roi-max-x", type=float, default=8.0)
    parser.add_argument("--roi-min-y", type=float, default=-1.4)
    parser.add_argument("--roi-max-y", type=float, default=math.inf)
    parser.add_argument("--min-speed-mps", type=float, default=0.2)
    parser.add_argument("--max-scans-per-bag", type=int, default=250)
    parser.add_argument("--max-beams-per-scan", type=int, default=80)
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    roi = (args.roi_min_x, args.roi_max_x, args.roi_min_y, args.roi_max_y)
    fallback_tf = first_available_tf(args.real_root)

    datasets: List[ScanMatchSamples] = []
    rows: List[Dict[str, float | int | str]] = []
    for particles in PARTICLES:
        data = bag_samples(
            args.real_root,
            particles,
            roi,
            args.scan_topic,
            fallback_tf,
            args.min_speed_mps,
            args.max_scans_per_bag,
            args.max_beams_per_scan,
        )
        datasets.append(data)
        params, initial_cost, final_cost = optimize_correction([data])
        row = summarize_result(str(particles), params, initial_cost, final_cost)
        row.update({
            "particles": particles,
            "bag": data.bag,
            "tf_source": data.tf_source,
            "n_scans": data.n_scans,
            "n_scan_points": int(data.pose_x.size),
        })
        rows.append(row)
        print(
            f"{particles:4d}: dx={params[0] * 100:+.1f} cm, "
            f"dy={params[1] * 100:+.1f} cm, "
            f"dyaw={math.degrees(params[2]):+.2f} deg, "
            f"cost {initial_cost * 100:.2f}->{final_cost * 100:.2f} cm"
            ,
            flush=True,
        )

    pooled_params, pooled_initial, pooled_final = optimize_correction(datasets)
    pooled = summarize_result("pooled", pooled_params, pooled_initial, pooled_final)
    pooled.update({
        "particles": "all",
        "bag": "all",
        "tf_source": "mixed",
        "n_scans": int(sum(d.n_scans for d in datasets)),
        "n_scan_points": int(sum(d.pose_x.size for d in datasets)),
        "pooled_trimmed_scan_distance_cm": float(100.0 * pooled_final),
        "pooled_minus_optimal_cm": 0.0,
    })

    for row, data in zip(rows, datasets):
        pooled_cost = 100.0 * scan_distance_cost(pooled_params, [data])
        row["pooled_trimmed_scan_distance_cm"] = float(pooled_cost)
        row["pooled_minus_optimal_cm"] = float(
            pooled_cost - row["corrected_trimmed_scan_distance_cm"])
    rows.append(pooled)

    df = pd.DataFrame(rows)
    per_bag = df[df["label"] != "pooled"].copy()
    consistency = pd.DataFrame([{
        "n_bags": int(len(per_bag)),
        "mean_dx_cm": float(100.0 * per_bag["dx_m"].astype(float).mean()),
        "std_dx_cm": float(100.0 * per_bag["dx_m"].astype(float).std(ddof=1)),
        "mean_dy_cm": float(100.0 * per_bag["dy_m"].astype(float).mean()),
        "std_dy_cm": float(100.0 * per_bag["dy_m"].astype(float).std(ddof=1)),
        "mean_dyaw_deg": float(per_bag["dyaw_deg"].astype(float).mean()),
        "std_dyaw_deg": float(per_bag["dyaw_deg"].astype(float).std(ddof=1)),
        "pooled_dx_cm": float(100.0 * pooled_params[0]),
        "pooled_dy_cm": float(100.0 * pooled_params[1]),
        "pooled_dyaw_deg": float(math.degrees(pooled_params[2])),
        "pooled_initial_distance_cm": float(100.0 * pooled_initial),
        "pooled_corrected_distance_cm": float(100.0 * pooled_final),
    }])

    df.to_csv(args.output_dir / "OptiTrack_ScanMap_TF_Correction_PerBag.csv", index=False)
    consistency.to_csv(args.output_dir / "OptiTrack_ScanMap_TF_Correction_Consistency.csv", index=False)

    print(
        "pooled: "
        f"dx={pooled_params[0] * 100:+.1f} cm, "
        f"dy={pooled_params[1] * 100:+.1f} cm, "
        f"dyaw={math.degrees(pooled_params[2]):+.2f} deg, "
        f"cost {pooled_initial * 100:.2f}->{pooled_final * 100:.2f} cm"
    )
    print(f"wrote {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
