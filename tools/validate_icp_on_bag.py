#!/usr/bin/env python3
"""Validate the improved scan-matching on a real MCAP bag.

Reads the first N /scan messages (plus /sensors/imu/raw yaw rate) from a bag and
runs the production ICP twice over the same consecutive pairs:

  * improved : within-scan motion deskew + constant-velocity translation seed
  * baseline : original zero-translation seed, no deskew

and reports convergence, ICP iterations and point-to-line RMSE for each, so the
effect of the changes is measurable on real hardware data rather than only on
the synthetic geometry test.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ANALYSIS = Path("/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_parameters/steering/analysis")
sys.path.insert(0, str(ANALYSIS))
from estimate_lidar_motion import deskew_scan, pca_normals, robust_point_to_line_icp, rot, scan_points, stamp_ns  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

ICP = dict(max_iter=120, max_corr=0.3, trim_fraction=0.85, min_corr=120, normal_neighbors=9,
           translation_tolerance_m=1e-6, rotation_tolerance_rad=1e-6, relative_cost_tolerance=1e-7,
           downsample=1)


def _solve(source, target, normals, yaw_seed, translation_seed):
    return robust_point_to_line_icp(
        source, target, normals, yaw_seed=yaw_seed,
        max_iter=ICP["max_iter"], max_corr=ICP["max_corr"], trim_fraction=ICP["trim_fraction"],
        min_corr=ICP["min_corr"], translation_tolerance_m=ICP["translation_tolerance_m"],
        rotation_tolerance_rad=ICP["rotation_tolerance_rad"], relative_cost_tolerance=ICP["relative_cost_tolerance"],
        translation_seed=translation_seed,
    )


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--scans", type=int, default=300)
    args = ap.parse_args()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {e.name: e.type for e in reader.get_all_topics_and_types()}
    scan_cls = get_message(types["/scan"])
    imu_cls = get_message(types["/sensors/imu/raw"])

    scans: list[tuple[int, np.ndarray, np.ndarray]] = []
    imu_t: list[int] = []
    imu_gz: list[float] = []
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic == "/sensors/imu/raw":
            m = deserialize_message(raw, imu_cls)
            imu_t.append(stamp_ns(m, bag_ns)); imu_gz.append(float(m.angular_velocity.z))
        elif topic == "/scan":
            m = deserialize_message(raw, scan_cls)
            pts, rel = scan_points(m, ICP["downsample"])
            scans.append((stamp_ns(m, bag_ns), pts, rel))
            if len(scans) >= args.scans and imu_t and imu_t[-1] > scans[-1][0]:
                break
    imu_t = np.array(imu_t, dtype=float); imu_gz = np.array(imu_gz, dtype=float)
    print(f"bag={args.bag.name}  scans={len(scans)}  imu_samples={len(imu_t)}")

    stats = {m: {"valid": 0, "iters": [], "rmse": [], "speed": []} for m in ("improved", "baseline")}
    last_v = np.zeros(2)
    last_w = 0.0
    pairs = 0
    for (pt_ns, pts, rel), (t_ns, cur, cur_rel) in zip(scans, scans[1:]):
        dt = (t_ns - pt_ns) * 1e-9
        if not (0.005 < dt <= 0.2):
            continue
        mid = 0.5 * (t_ns + pt_ns)
        yaw_seed_val = float(np.interp(mid, imu_t, imu_gz, left=np.nan, right=np.nan))
        if not math.isfinite(yaw_seed_val):
            continue
        omega_now = float(np.interp(t_ns, imu_t, imu_gz, left=last_w, right=last_w))
        try:
            # improved: deskew both scans with carried velocity, seed translation
            tgt_i = deskew_scan(pts, rel, last_v, omega_now)
            src_i = deskew_scan(cur, cur_rel, last_v, omega_now)
            n_i = pca_normals(tgt_i, ICP["normal_neighbors"])
            r_i = _solve(src_i, tgt_i, n_i, yaw_seed_val * dt, last_v * dt)
            # baseline: raw scans, zero translation seed
            n_b = pca_normals(pts, ICP["normal_neighbors"])
            r_b = _solve(cur, pts, n_b, yaw_seed_val * dt, None)
        except Exception:
            continue
        pairs += 1
        if r_i.converged and math.isfinite(r_i.rmse_m):
            last_v = r_i.t / dt
            last_w = math.atan2(r_i.R[1, 0], r_i.R[0, 0]) / dt
        for name, r in (("improved", r_i), ("baseline", r_b)):
            if r.converged:
                stats[name]["valid"] += 1
            stats[name]["iters"].append(r.iterations)
            stats[name]["rmse"].append(r.rmse_m)
            stats[name]["speed"].append(math.hypot(*(r.t / dt)))

    print(f"processed pairs: {pairs}\n")
    print(f"{'mode':10s} {'conv%':>7s} {'med_iter':>9s} {'med_rmse_mm':>12s} {'med_speed':>10s}")
    for name in ("baseline", "improved"):
        s = stats[name]
        if not s["iters"]:
            print(f"{name:10s}  no pairs"); continue
        print(f"{name:10s} {100*s['valid']/pairs:7.1f} {np.median(s['iters']):9.1f} "
              f"{1000*np.median(s['rmse']):12.2f} {np.median(s['speed']):10.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
