#!/usr/bin/env python3
"""Baseline (displacement-targeted) LiDAR velocity from raw LaserScan.

Per-frame (consecutive-scan) ICP forward velocity is unobservable at calibration
speeds: 0.6 m/s @ 40 Hz is ~1.5 cm/frame, below LiDAR range noise. This estimator
instead matches each scan against an *earlier* scan chosen so the inter-scan
displacement is a fixed target (~0.12 m), keeping translation SNR high at every
speed. Velocity = base-frame displacement / dt over that baseline. Yaw is still
IMU-seeded; deskew and warm-start are retained. Same per-pair diagnostics/gates
and output schema as estimate_lidar_motion.py, plus baseline_frames / baseline_dt_s.
"""
from __future__ import annotations
import sys, math, json
from pathlib import Path
import numpy as np

PROJ = "/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_parameters/steering/analysis"
sys.path.insert(0, PROJ)
import estimate_lidar_motion as elm  # noqa: E402
import pandas as pd, yaml  # noqa: E402
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

D_MIN = 0.12      # target inter-scan displacement (m)
DT_MAX = 0.32     # max baseline (s) before overlap degrades
DT_MIN = 0.04     # min baseline (s); below this forward motion is in the noise
V_FLOOR = 0.30    # speed floor for predicting the baseline (m/s)


def estimate(session: Path, stage: str):
    cfg = yaml.safe_load((session / "calibration_config_snapshot.yaml").read_text())
    icp = cfg["analysis"]["icp"]; hw = cfg["hardware"]
    derived = session / stage / "derived"
    bag = session / stage / "bag"
    imu_t, imu_gz = elm.load_imu(derived)
    windows = elm.capture_windows(derived)
    stype = get_message("sensor_msgs/msg/LaserScan")
    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag), storage_id="mcap"), ConverterOptions("", ""))

    lo = np.array([float(hw["laser_to_base_x_m"]), float(hw["laser_to_base_y_m"])])
    laser_yaw = float(hw.get("laser_to_base_yaw_rad", 0.0))
    c, s = math.cos(laser_yaw), math.sin(laser_yaw)
    R_bl = np.array([[c, -s], [s, c]]); R_lb = R_bl.T

    buf: list[dict] = []          # recent scans: dict(t, pts)
    last_v_laser = np.zeros(2); last_omega = 0.0; v_est = 0.6
    win_i = 0; active_key = None
    rows: list[dict] = []

    def yaw_between(t0, t1):
        # integral of yaw rate over [t0,t1] via trapezoid on imu samples
        m = (imu_t >= t0) & (imu_t <= t1)
        if m.sum() >= 2:
            tt = imu_t[m]; gg = imu_gz[m]
            return float(np.trapz(gg, tt) * 1e-9) if hasattr(np, "trapz") else float(np.trapezoid(gg, tt) * 1e-9)
        mid = 0.5 * (t0 + t1)
        return float(np.interp(mid, imu_t, imu_gz)) * (t1 - t0) * 1e-9

    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic != "/scan":
            continue
        scan = deserialize_message(raw, stype)
        t_ns = elm.stamp_ns(scan, bag_ns)
        window, win_i = elm._window_for(t_ns, windows, win_i)
        if windows and window is None:
            buf.clear(); active_key = None; continue
        key = None if window is None else (window["trial_id"], window["phase"], window["segment_id"])
        if key != active_key:
            buf.clear(); last_v_laser = np.zeros(2); last_omega = 0.0; v_est = 0.6; active_key = key
        pts, rel = elm.scan_points(scan, int(icp["downsample"]))
        pts, rel = elm.filter_self_points(pts, rel, hw)
        pts = elm.deskew_scan(pts, rel, last_v_laser, last_omega)
        buf.append({"t": t_ns, "pts": pts})
        # prune
        while len(buf) > 2 and (t_ns - buf[0]["t"]) * 1e-9 > DT_MAX + 0.1:
            buf.pop(0)
        if len(buf) < 2:
            continue
        # choose target: smallest dt reaching D_MIN, else largest dt <= DT_MAX
        target = None
        for j in range(len(buf) - 2, -1, -1):
            dt = (t_ns - buf[j]["t"]) * 1e-9
            if dt < DT_MIN:
                continue
            if dt > DT_MAX:
                break
            target = j
            if max(v_est, V_FLOOR) * dt >= D_MIN:
                break
        if target is None:
            continue
        tgt = buf[target]; dt_s = (t_ns - tgt["t"]) * 1e-9
        src_pts = buf[-1]["pts"]; tgt_pts = tgt["pts"]
        meta = {"bag_ns": int(t_ns), "previous_bag_ns": int(tgt["t"]), "dt_s": dt_s,
                "baseline_frames": len(buf) - 1 - target, "baseline_dt_s": dt_s,
                "trial_id": None if window is None else window["trial_id"],
                "phase": None if window is None else window["phase"],
                "segment_id": None if window is None else window["segment_id"]}
        if len(src_pts) < int(icp["min_correspondences"]) or len(tgt_pts) < int(icp["min_correspondences"]):
            rows.append({**meta, "valid": False, "reason": "too_few_points"}); continue
        yaw_seed = yaw_between(tgt["t"], t_ns)
        translation_seed = last_v_laser * dt_s
        try:
            normals = elm.pca_normals(tgt_pts, int(icp["normal_neighbors"]))
            res = elm.robust_point_to_line_icp(
                src_pts, tgt_pts, normals, yaw_seed=yaw_seed,
                max_iter=int(icp["max_iterations"]), max_corr=float(icp["max_correspondence_m"]),
                trim_fraction=float(icp["trim_fraction"]), min_corr=int(icp["min_correspondences"]),
                translation_tolerance_m=float(icp["translation_update_tolerance_m"]),
                rotation_tolerance_rad=float(icp["rotation_update_tolerance_rad"]),
                relative_cost_tolerance=float(icp["relative_cost_tolerance"]),
                translation_seed=translation_seed)
        except Exception as exc:
            rows.append({**meta, "valid": False, "reason": repr(exc)}); continue
        if res.converged and math.isfinite(res.rmse_m):
            last_v_laser = res.t / dt_s
            last_omega = math.atan2(float(res.R[1, 0]), float(res.R[0, 0])) / dt_s
            v_est = float(np.hypot(*last_v_laser))
        R_base = R_bl @ res.R @ R_lb
        delta_base = R_bl @ res.t + (np.eye(2) - R_base) @ lo
        delta_yaw = math.atan2(float(R_base[1, 0]), float(R_base[0, 0]))
        yaw_seed_residual = elm.wrap_angle(delta_yaw - yaw_seed)
        reasons = []
        if not res.converged: reasons.append("solver_not_converged")
        if res.correspondence_count < int(icp["min_correspondences"]): reasons.append("too_few_correspondences")
        if res.inlier_ratio < float(icp["min_inlier_ratio"]): reasons.append("low_inlier_ratio")
        if res.rmse_m > float(icp["max_pair_rmse_m"]): reasons.append("high_point_to_line_rmse")
        if res.condition_number > float(icp["max_hessian_condition_number"]): reasons.append("poor_geometry_conditioning")
        if abs(yaw_seed_residual) > float(icp["max_imu_yaw_residual_rad"]): reasons.append("imu_yaw_inconsistent")
        rows.append({**meta,
            "vx": float(delta_base[0] / dt_s), "vy": float(delta_base[1] / dt_s),
            "yaw_rate_icp": float(delta_yaw / dt_s),
            "yaw_rate_imu_seed": yaw_seed / dt_s,
            "yaw_seed_residual_rad": yaw_seed_residual,
            "icp_rmse_m": res.rmse_m, "correspondences": res.correspondence_count,
            "inlier_ratio": res.inlier_ratio, "iterations": res.iterations,
            "solver_converged": res.converged, "final_cost": res.final_cost,
            "hessian_condition_number": res.condition_number,
            "yaw_std_rad": res.yaw_std_rad, "dx_std_m": res.dx_std_m, "dy_std_m": res.dy_std_m,
            "valid": not reasons, "reason": ";".join(reasons)})
    table = pd.DataFrame(rows)
    out = derived / "lidar_velocity_baseline.parquet"
    table.to_parquet(out, index=False)
    nvalid = int(table["valid"].sum()) if len(table) and "valid" in table else 0
    print(f"{stage}: pairs={len(table)} valid={nvalid} "
          f"({100*nvalid/max(len(table),1):.0f}%) median_baseline_dt="
          f"{table.get('baseline_dt_s', pd.Series(dtype=float)).median():.3f}s")
    return out


if __name__ == "__main__":
    session = Path("/home/akselmo/Downloads/steering/runs/20260625T124715Z_steering_calibration")
    stages = sys.argv[1:] or ["04_static_map_training"]
    for st in stages:
        estimate(session, st)
