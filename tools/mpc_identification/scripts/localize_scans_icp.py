#!/usr/bin/env python3
"""Register every /scan against the occupancy-grid wall cloud using SE(2) ICP.

The EKF pose is only an initial condition. The output records ICP quality so
subsequent fitting can reject geometrically weak or bad matches instead of
silently treating all scan registrations as ground truth.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.spatial import cKDTree

from geometry import fit_se2, interpolate_pose, transform_points, wrap_angle


def scan_points_base(ranges: np.ndarray, angle_min: float, angle_increment: float,
                     range_min: float, range_max: float, laser_x: float, laser_y: float,
                     laser_yaw: float, stride: int) -> np.ndarray:
    idx = np.arange(0, len(ranges), max(1, stride))
    r = ranges[idx]
    valid = np.isfinite(r) & (r >= range_min) & (r <= range_max)
    r = r[valid]
    a = angle_min + idx[valid] * angle_increment + laser_yaw
    pts = np.column_stack([r * np.cos(a), r * np.sin(a)])
    pts[:, 0] += laser_x
    pts[:, 1] += laser_y
    return pts


def icp_se2(local_points: np.ndarray, tree: cKDTree, map_points: np.ndarray, initial_pose: np.ndarray,
            max_iterations: int, max_correspondence_m: float, min_inliers: int,
            tolerance_xy_m: float, tolerance_yaw_rad: float) -> tuple[np.ndarray, float, int, float, bool]:
    pose = initial_pose.astype(float).copy()
    last_rmse = np.inf
    condition = 0.0
    success = False
    for _ in range(max_iterations):
        global_points = transform_points(local_points, pose)
        distance, nearest = tree.query(global_points)
        mask = distance <= max_correspondence_m
        n = int(np.count_nonzero(mask))
        if n < min_inliers:
            return pose, float("inf"), n, condition, False
        src = local_points[mask]
        dst = map_points[nearest[mask]]
        centered = src - src.mean(axis=0)
        singular = np.linalg.svd(centered, compute_uv=False)
        condition = float(singular[-1] / max(singular[0], 1e-12)) if singular.size == 2 else 0.0
        new_pose = fit_se2(src, dst)
        rmse = float(np.sqrt(np.mean(distance[mask] ** 2)))
        delta_xy = float(np.linalg.norm(new_pose[:2] - pose[:2]))
        delta_yaw = abs(float(wrap_angle(new_pose[2] - pose[2])))
        pose = new_pose
        if delta_xy < tolerance_xy_m and delta_yaw < tolerance_yaw_rad:
            success = True
            break
        last_rmse = rmse
    global_points = transform_points(local_points, pose)
    distance, _nearest = tree.query(global_points)
    mask = distance <= max_correspondence_m
    n = int(np.count_nonzero(mask))
    rmse = float(np.sqrt(np.mean(distance[mask] ** 2))) if n else float("inf")
    return pose, rmse, n, condition, (success or (n >= min_inliers and np.isfinite(rmse)))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scan-index", type=Path, required=True)
    parser.add_argument("--scans", type=Path, required=True)
    parser.add_argument("--map-points", type=Path, required=True)
    parser.add_argument("--prior", type=Path, required=True, help="EKF pose CSV")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--laser-x", type=float, default=0.265)
    parser.add_argument("--laser-y", type=float, default=0.0)
    parser.add_argument("--laser-yaw", type=float, default=0.0)
    parser.add_argument("--scan-stride", type=int, default=3)
    parser.add_argument("--max-iterations", type=int, default=15)
    parser.add_argument("--max-correspondence", type=float, default=0.35)
    parser.add_argument("--min-inliers", type=int, default=60)
    parser.add_argument("--max-rmse", type=float, default=0.12)
    parser.add_argument("--min-condition", type=float, default=0.02,
                        help="Minimum minor/major local-point singular-value ratio.")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    scan_index = pd.read_csv(args.scan_index)
    scan_ranges = np.load(args.scans, allow_pickle=False)["ranges"]
    map_points = np.load(args.map_points, allow_pickle=False)["points"].astype(float)
    prior = pd.read_csv(args.prior).sort_values("t_s")
    if len(scan_index) != len(scan_ranges):
        raise RuntimeError(f"scan_index rows ({len(scan_index)}) != scan ranges ({len(scan_ranges)})")
    tree = cKDTree(map_points)
    prior_poses = interpolate_pose(
        prior["t_s"].to_numpy(), prior["x"].to_numpy(), prior["y"].to_numpy(),
        prior["yaw"].to_numpy(), scan_index["t_s"].to_numpy(),
    )

    out = []
    last_good = None
    for i, meta in scan_index.iterrows():
        local = scan_points_base(
            scan_ranges[i], meta.angle_min, meta.angle_increment, meta.range_min, meta.range_max,
            args.laser_x, args.laser_y, args.laser_yaw, args.scan_stride,
        )
        prior_pose = prior_poses[i]
        # EKF remains the initial seed to avoid local-wall ambiguity. Do not carry the
        # previous result as primary seed; that would turn one bad ICP update into drift.
        pose, rmse, inliers, condition, converged = icp_se2(
            local, tree, map_points, prior_pose, args.max_iterations, args.max_correspondence,
            args.min_inliers, 1e-4, 1e-4,
        )
        valid = bool(converged and np.isfinite(rmse) and rmse <= args.max_rmse and condition >= args.min_condition)
        if valid:
            last_good = pose.copy()
        out.append({
            "t_s": meta.t_s, "t_ns": int(meta.t_ns), "scan_index": i,
            "x": pose[0], "y": pose[1], "yaw": pose[2],
            "prior_x": prior_pose[0], "prior_y": prior_pose[1], "prior_yaw": prior_pose[2],
            "delta_x_from_prior": pose[0] - prior_pose[0],
            "delta_y_from_prior": pose[1] - prior_pose[1],
            "delta_yaw_from_prior": float(wrap_angle(pose[2] - prior_pose[2])),
            "rmse_m": rmse, "inliers": inliers, "condition": condition,
            "valid": int(valid), "converged": int(converged), "scan_points": len(local),
        })
        if (i + 1) % 100 == 0 or i + 1 == len(scan_index):
            print(f"ICP {i + 1:4d}/{len(scan_index)}: valid={sum(x['valid'] for x in out):4d}, last rmse={rmse:.3f} m")

    frame = pd.DataFrame(out)
    frame.to_csv(args.output, index=False)
    print(f"Wrote {args.output}; valid scans: {int(frame.valid.sum())}/{len(frame)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
