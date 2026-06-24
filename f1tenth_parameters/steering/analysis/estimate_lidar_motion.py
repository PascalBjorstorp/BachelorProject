#!/usr/bin/env python3
"""Estimate LiDAR body velocity from raw LaserScan pairs in an MCAP bag.

No odometry translation, velocity, or pose is used by ICP. IMU yaw rate is only
an initial rotational guess. Translation is obtained from point-to-line scan
matching against static scene geometry. The implementation reports convergence,
residual, correspondence, conditioning and covariance diagnostics for every
scan pair so later fitting can reject weak estimates.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def rot(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def stamp_ns(msg, fallback: int) -> int:
    stamp = msg.header.stamp
    value = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else int(fallback)


def scan_points(msg, downsample: int) -> tuple[np.ndarray, np.ndarray]:
    """Return (points, per-beam relative time in seconds from the scan start)."""
    ranges = np.asarray(msg.ranges, dtype=float)
    n = ranges.size
    angles = msg.angle_min + np.arange(n, dtype=float) * msg.angle_increment
    time_increment = float(getattr(msg, "time_increment", 0.0) or 0.0)
    rel_t = np.arange(n, dtype=float) * time_increment
    valid = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
    ranges, angles, rel_t = ranges[valid], angles[valid], rel_t[valid]
    points = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles)))
    step = max(1, downsample)
    return points[::step], rel_t[::step]


def deskew_scan(points: np.ndarray, rel_t: np.ndarray, v_laser: np.ndarray, omega: float) -> np.ndarray:
    """Motion-compensate a scan to its first-beam time under constant velocity.

    A spinning LiDAR samples each beam at a different instant (``rel_t`` seconds
    after the scan start) while the vehicle is moving, so a single scan is not a
    rigid snapshot. At 40 Hz and a few m/s this within-scan skew is several
    centimetres and biases scan-to-scan matching. Expressing every point in the
    sensor pose at the scan start removes it. ``v_laser`` (sensor-frame linear
    velocity) and ``omega`` (yaw rate) come from the constant-velocity
    prediction carried between pairs; when no motion estimate or per-beam time is
    available the scan is returned unchanged.
    """
    if points.size == 0 or rel_t.size != len(points) or not np.any(rel_t > 0.0):
        return points
    if not (np.all(np.isfinite(v_laser)) and math.isfinite(omega)):
        return points
    cos_t, sin_t = np.cos(omega * rel_t), np.sin(omega * rel_t)
    x = cos_t * points[:, 0] - sin_t * points[:, 1] + v_laser[0] * rel_t
    y = sin_t * points[:, 0] + cos_t * points[:, 1] + v_laser[1] * rel_t
    return np.column_stack((x, y))


def pca_normals(points: np.ndarray, neighbors: int) -> np.ndarray:
    """Surface normals from local PCA, robust to missing beams / range gaps."""
    from scipy.spatial import cKDTree

    if len(points) < max(4, neighbors):
        raise RuntimeError("too few target points for local normal estimation")
    tree = cKDTree(points)
    _, idx = tree.query(points, k=min(neighbors, len(points)), workers=-1)
    normals = np.zeros_like(points)
    for i, local_idx in enumerate(np.atleast_2d(idx)):
        local = points[local_idx]
        centred = local - local.mean(axis=0)
        covariance = centred.T @ centred
        values, vectors = np.linalg.eigh(covariance)
        normal = vectors[:, int(np.argmin(values))]
        norm = float(np.linalg.norm(normal))
        if norm < 1e-12:
            normal = np.array([0.0, 0.0])
        else:
            normal = normal / norm
        normals[i] = normal
    return normals


@dataclass
class IcpResult:
    R: np.ndarray
    t: np.ndarray
    rmse_m: float
    correspondence_count: int
    inlier_ratio: float
    iterations: int
    converged: bool
    final_cost: float
    condition_number: float
    yaw_std_rad: float
    dx_std_m: float
    dy_std_m: float


def _correspondences(
    source: np.ndarray,
    R: np.ndarray,
    t: np.ndarray,
    tree,
    target: np.ndarray,
    target_normals: np.ndarray,
    max_corr: float,
    trim_fraction: float,
    min_corr: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    transformed = source @ R.T + t
    dist, indices = tree.query(transformed, k=1, workers=-1)
    mask = np.isfinite(dist) & (dist < max_corr)
    if int(mask.sum()) < min_corr:
        raise RuntimeError("too few ICP correspondences")
    cutoff = np.quantile(dist[mask], trim_fraction)
    mask &= dist <= cutoff
    if int(mask.sum()) < min_corr:
        raise RuntimeError("too few trimmed ICP correspondences")
    return transformed[mask], target[indices[mask]], target_normals[indices[mask]], dist[mask], mask


def robust_point_to_line_icp(
    source: np.ndarray,
    target: np.ndarray,
    target_normals: np.ndarray,
    *,
    yaw_seed: float,
    max_iter: int,
    max_corr: float,
    trim_fraction: float,
    min_corr: int,
    translation_tolerance_m: float,
    rotation_tolerance_rad: float,
    relative_cost_tolerance: float,
    translation_seed: np.ndarray | None = None,
) -> IcpResult:
    from scipy.spatial import cKDTree

    if len(source) < min_corr or len(target) < min_corr:
        raise RuntimeError("too few valid scan points")
    tree = cKDTree(target)
    R = rot(yaw_seed)
    # Warm-start the translation from the constant-velocity prediction; this
    # keeps the first correspondence search near the true motion at speed, which
    # a zero start cannot do once per-frame displacement approaches max_corr.
    t = np.zeros(2, dtype=float) if translation_seed is None else np.asarray(translation_seed, dtype=float).copy()
    previous_cost = math.inf
    converged = False
    final_A = None
    final_w = None
    final_residual = None
    iterations = 0

    for iterations in range(1, max_iter + 1):
        transformed, q, n, _, _ = _correspondences(
            source, R, t, tree, target, target_normals, max_corr, trim_fraction, min_corr
        )
        residual = np.sum(n * (transformed - q), axis=1)
        dtheta = np.column_stack((-transformed[:, 1], transformed[:, 0]))
        A = np.column_stack((np.sum(n * dtheta, axis=1), n[:, 0], n[:, 1]))
        # Robust Huber weights reject static-wall endpoints, scan shadows and
        # transient dynamic objects without discarding the raw underlying scan.
        scale = max(float(np.median(np.abs(residual))) * 1.4826, 0.0015)
        weights = np.minimum(1.0, 2.5 * scale / np.maximum(np.abs(residual), 1e-12))
        step, *_ = np.linalg.lstsq(A * weights[:, None], -residual * weights, rcond=None)
        d_yaw, dx, dy = map(float, step)
        cost = float(np.mean((weights * residual) ** 2))
        relative_improvement = abs(previous_cost - cost) / max(previous_cost, 1e-15) if math.isfinite(previous_cost) else math.inf
        R = rot(d_yaw) @ R
        t += np.array([dx, dy], dtype=float)
        final_A, final_w, final_residual = A, weights, residual
        if (
            abs(d_yaw) <= rotation_tolerance_rad
            and math.hypot(dx, dy) <= translation_tolerance_m
            and relative_improvement <= relative_cost_tolerance
        ):
            converged = True
            break
        previous_cost = cost

    transformed, q, n, _, mask = _correspondences(
        source, R, t, tree, target, target_normals, max_corr, trim_fraction, min_corr
    )
    residual = np.sum(n * (transformed - q), axis=1)
    dtheta = np.column_stack((-transformed[:, 1], transformed[:, 0]))
    A = np.column_stack((np.sum(n * dtheta, axis=1), n[:, 0], n[:, 1]))
    scale = max(float(np.median(np.abs(residual))) * 1.4826, 0.0015)
    weights = np.minimum(1.0, 2.5 * scale / np.maximum(np.abs(residual), 1e-12))
    H = A.T @ (weights[:, None] * A)
    try:
        condition = float(np.linalg.cond(H))
    except np.linalg.LinAlgError:
        condition = math.inf
    dof = max(1, len(residual) - 3)
    sigma2 = float(np.sum(weights * residual * residual) / dof)
    covariance = np.linalg.pinv(H) * sigma2
    std = np.sqrt(np.maximum(np.diag(covariance), 0.0))
    return IcpResult(
        R=R,
        t=t,
        rmse_m=float(np.sqrt(np.mean(residual * residual))),
        correspondence_count=int(len(residual)),
        inlier_ratio=float(len(residual) / max(len(source), 1)),
        iterations=iterations,
        converged=converged,
        final_cost=float(np.mean((weights * residual) ** 2)),
        condition_number=condition,
        yaw_std_rad=float(std[0]),
        dx_std_m=float(std[1]),
        dy_std_m=float(std[2]),
    )


def load_imu(derived: Path):
    import pandas as pd

    path = derived / "imu.parquet"
    if not path.exists():
        raise FileNotFoundError(f"export IMU first: {path}")
    table = pd.read_parquet(path).sort_values("bag_ns")
    return table["bag_ns"].to_numpy(dtype=float), table["gz"].to_numpy(dtype=float)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "steering_calibration.yaml")
    args = parser.parse_args()
    bag_dir = args.bag.resolve()
    if not (bag_dir / "metadata.yaml").exists():
        raise SystemExit(f"not a bag: {bag_dir}")
    import yaml

    cfg = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    icp = cfg["analysis"]["icp"]
    hardware = cfg["hardware"]
    derived = bag_dir.parent / "derived"
    imu_t, imu_gz = load_imu(derived)
    try:
        import pandas as pd
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit("Source the ROS workspace and install offline dependencies: " + repr(exc))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if "/scan" not in types:
        raise SystemExit("bag contains no /scan topic")
    scan_type = get_message(types["/scan"])
    deskew_enabled = bool(icp.get("motion_deskew", True))
    seed_enabled = bool(icp.get("constant_velocity_seed", True))
    max_gap_s = float(icp.get("max_scan_period_s", 0.20))
    previous = None
    # Constant-velocity state carried between pairs (LiDAR frame) for deskew and
    # translation seeding. Both default to zero, so the cold start matches the
    # previous zero-seed behaviour.
    last_v_laser = np.zeros(2, dtype=float)
    last_omega = 0.0
    rows: list[dict] = []
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic != "/scan":
            continue
        scan = deserialize_message(raw, scan_type)
        t_ns = stamp_ns(scan, bag_ns)
        points, rel_t = scan_points(scan, int(icp["downsample"]))
        omega_now = float(np.interp(t_ns, imu_t, imu_gz, left=np.nan, right=np.nan))
        if not math.isfinite(omega_now):
            omega_now = last_omega
        if deskew_enabled:
            points = deskew_scan(points, rel_t, last_v_laser, omega_now)
        if previous is None:
            previous = (t_ns, points)
            continue
        previous_ns, target = previous
        previous = (t_ns, points)
        dt_s = (t_ns - previous_ns) * 1e-9
        if dt_s <= 0.005 or dt_s > max_gap_s:
            rows.append({"bag_ns": int(t_ns), "previous_bag_ns": int(previous_ns), "dt_s": dt_s,
                         "valid": False, "reason": "scan_period_out_of_range"})
            continue
        mid_ns = 0.5 * (t_ns + previous_ns)
        yaw_rate_seed = float(np.interp(mid_ns, imu_t, imu_gz, left=np.nan, right=np.nan))
        if not math.isfinite(yaw_rate_seed):
            rows.append({"bag_ns": int(t_ns), "previous_bag_ns": int(previous_ns), "dt_s": dt_s,
                         "valid": False, "reason": "missing_imu_yaw_seed"})
            continue
        yaw_seed = yaw_rate_seed * dt_s
        translation_seed = last_v_laser * dt_s if seed_enabled else None
        try:
            target_normals = pca_normals(target, int(icp["normal_neighbors"]))
            result = robust_point_to_line_icp(
                points,
                target,
                target_normals,
                yaw_seed=yaw_seed,
                max_iter=int(icp["max_iterations"]),
                max_corr=float(icp["max_correspondence_m"]),
                trim_fraction=float(icp["trim_fraction"]),
                min_corr=int(icp["min_correspondences"]),
                translation_tolerance_m=float(icp["translation_update_tolerance_m"]),
                rotation_tolerance_rad=float(icp["rotation_update_tolerance_rad"]),
                relative_cost_tolerance=float(icp["relative_cost_tolerance"]),
                translation_seed=translation_seed,
            )
        except Exception as exc:
            rows.append({"bag_ns": int(t_ns), "previous_bag_ns": int(previous_ns), "dt_s": dt_s,
                         "valid": False, "reason": repr(exc)})
            continue
        # Carry the LiDAR-frame velocity forward as the next pair's prediction.
        # Only from a converged solve, so one bad pair cannot poison the next
        # frame's seed/deskew; otherwise the last good velocity persists.
        if result.converged and math.isfinite(result.rmse_m):
            last_v_laser = result.t / dt_s
            last_omega = math.atan2(float(result.R[1, 0]), float(result.R[0, 0])) / dt_s
        # Convert the scan-to-scan transform from the LiDAR frame to the
        # base frame.  The previous implementation silently assumed laser yaw
        # was zero; this applies all configured planar extrinsics.
        laser_offset_base = np.array([
            float(hardware["laser_to_base_x_m"]),
            float(hardware["laser_to_base_y_m"]),
        ])
        laser_yaw = float(hardware.get("laser_to_base_yaw_rad", 0.0))
        c_yaw, s_yaw = math.cos(laser_yaw), math.sin(laser_yaw)
        R_base_laser = np.array([[c_yaw, -s_yaw], [s_yaw, c_yaw]])
        R_laser_base = R_base_laser.T
        R_base = R_base_laser @ result.R @ R_laser_base
        delta_base = R_base_laser @ result.t + (np.eye(2) - R_base) @ laser_offset_base
        delta_yaw = math.atan2(float(R_base[1, 0]), float(R_base[0, 0]))
        yaw_seed_residual = wrap_angle(delta_yaw - yaw_seed)
        reasons: list[str] = []
        if not result.converged:
            reasons.append("solver_not_converged")
        if result.correspondence_count < int(icp["min_correspondences"]):
            reasons.append("too_few_correspondences")
        if result.inlier_ratio < float(icp["min_inlier_ratio"]):
            reasons.append("low_inlier_ratio")
        if result.rmse_m > float(icp["max_pair_rmse_m"]):
            reasons.append("high_point_to_line_rmse")
        if result.condition_number > float(icp["max_hessian_condition_number"]):
            reasons.append("poor_geometry_conditioning")
        if abs(yaw_seed_residual) > float(icp["max_imu_yaw_residual_rad"]):
            reasons.append("imu_yaw_inconsistent")
        valid = not reasons
        rows.append({
            "bag_ns": int(t_ns),
            "previous_bag_ns": int(previous_ns),
            "dt_s": dt_s,
            "vx": float(delta_base[0] / dt_s),
            "vy": float(delta_base[1] / dt_s),
            "yaw_rate_icp": float(delta_yaw / dt_s),
            "yaw_rate_imu_seed": yaw_rate_seed,
            "yaw_seed_residual_rad": yaw_seed_residual,
            "icp_rmse_m": result.rmse_m,
            "correspondences": result.correspondence_count,
            "inlier_ratio": result.inlier_ratio,
            "iterations": result.iterations,
            "solver_converged": result.converged,
            "final_cost": result.final_cost,
            "hessian_condition_number": result.condition_number,
            "yaw_std_rad": result.yaw_std_rad,
            "dx_std_m": result.dx_std_m,
            "dy_std_m": result.dy_std_m,
            "valid": valid,
            "reason": ";".join(reasons),
        })
    table = pd.DataFrame(rows)
    output = derived / "lidar_velocity.parquet"
    table.to_parquet(output, index=False)
    summary = {
        "output": str(output),
        "scan_pairs": int(len(table)),
        "valid_pairs": int(table["valid"].sum()) if len(table) and "valid" in table else 0,
        "method": "IMU-yaw-seeded, odometry-unseeded robust point-to-line ICP",
        "motion_deskew_enabled": deskew_enabled,
        "constant_velocity_translation_seed_enabled": seed_enabled,
        "solver_tolerances": {
            "max_iterations": int(icp["max_iterations"]),
            "translation_update_tolerance_m": float(icp["translation_update_tolerance_m"]),
            "rotation_update_tolerance_rad": float(icp["rotation_update_tolerance_rad"]),
            "relative_cost_tolerance": float(icp["relative_cost_tolerance"]),
        },
        "laser_to_base_geometry_used": {
            "x_m": float(hardware["laser_to_base_x_m"]),
            "y_m": float(hardware["laser_to_base_y_m"]),
            "z_m": float(hardware.get("laser_to_base_z_m", 0.0)),
            "yaw_rad": float(hardware.get("laser_to_base_yaw_rad", 0.0)),
            "base_frame_id": hardware.get("base_frame_id"),
            "laser_frame_id": hardware.get("laser_frame_id"),
        },
    }
    (derived / "lidar_motion_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
