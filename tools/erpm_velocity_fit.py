#!/usr/bin/env python3
"""Estimate a better ERPM -> ground-speed model from a real driving bag.

LiDAR scan matching (the improved deskewed, velocity-seeded ICP) is the
independent ground-speed reference. We:

  1. Fit a clean static ERPM->v map using only trustworthy samples (converged
     low-RMSE ICP, near-straight, low longitudinal acceleration). This avoids
     letting the noisy high-acceleration LiDAR pairs corrupt the gain, which is
     what a single global scalar fit on all data does.
  2. Quantify the high-acceleration mismatch: the signed residual
     (LiDAR speed - static-map speed) versus IMU longitudinal acceleration. Hard
     drive makes the wheel/ERPM over-read ground speed (slip) and the motor lag
     adds a transient; hard brake under-reads.
  3. Fit a causal correction v_hat = static_map(ERPM) + c1*a_imu (+ c2*I_motor)
     using IMU acceleration that is available in real time, and report the error
     reduction in the high-acceleration regime against the clean map alone.

Outputs a parquet of per-pair samples and a JSON/printed report.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ANALYSIS = Path("/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_parameters/steering/analysis")
sys.path.insert(0, str(ANALYSIS))
from estimate_lidar_motion import deskew_scan, pca_normals, robust_point_to_line_icp, stamp_ns, scan_points  # noqa: E402

import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

ICP = dict(max_iter=120, max_corr=0.3, trim_fraction=0.85, min_corr=120, normal_neighbors=9,
           tol_t=1e-6, tol_r=1e-6, tol_c=1e-7)


def _interp(t, ts, ys):
    return np.interp(t, ts, ys, left=np.nan, right=np.nan)


def collect(bag: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {e.name: e.type for e in reader.get_all_topics_and_types()}
    scan_cls = get_message(types["/scan"])
    imu_cls = get_message(types["/sensors/imu/raw"])
    core_cls = get_message(types["/sensors/core"])
    scans, it, igz, iax, ct, cerpm, ccur = [], [], [], [], [], [], []
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic == "/scan":
            m = deserialize_message(raw, scan_cls)
            pts, rel = scan_points(m, 1)
            scans.append((stamp_ns(m, bag_ns), pts, rel))
        elif topic == "/sensors/imu/raw":
            m = deserialize_message(raw, imu_cls)
            it.append(stamp_ns(m, bag_ns)); igz.append(float(m.angular_velocity.z)); iax.append(float(m.linear_acceleration.x))
        elif topic == "/sensors/core":
            m = deserialize_message(raw, core_cls)
            ct.append(stamp_ns(m, bag_ns)); cerpm.append(float(m.state.speed)); ccur.append(float(m.state.current_motor))
    return (scans, np.array(it, float), np.array(igz, float), np.array(iax, float),
            np.array(ct, float), np.array(cerpm, float), np.array(ccur, float))


def icp_table(bag: Path):
    scans, it, igz, iax, ct, cerpm, ccur = collect(bag)
    # Stationary IMU ax bias (first second of near-zero ERPM), removed from accel.
    ax_bias = 0.0
    if len(ct) and len(it):
        still = np.abs(_interp(it, ct, cerpm)) < 50.0
        if still.sum() > 50:
            ax_bias = float(np.nanmedian(iax[still]))
    rows = []
    last_v = np.zeros(2); last_w = 0.0
    for (pt, pts, rel), (t, cur, crel) in zip(scans, scans[1:]):
        dt = (t - pt) * 1e-9
        if not (0.005 < dt <= 0.2):
            continue
        mid = 0.5 * (t + pt)
        yaw = _interp(mid, it, igz)
        if not math.isfinite(yaw):
            continue
        omega = _interp(t, it, igz)
        omega = omega if math.isfinite(omega) else last_w
        try:
            tgt = deskew_scan(pts, rel, last_v, omega)
            src = deskew_scan(cur, crel, last_v, omega)
            normals = pca_normals(tgt, ICP["normal_neighbors"])
            r = robust_point_to_line_icp(src, tgt, normals, yaw_seed=yaw * dt,
                                         max_iter=ICP["max_iter"], max_corr=ICP["max_corr"],
                                         trim_fraction=ICP["trim_fraction"], min_corr=ICP["min_corr"],
                                         translation_tolerance_m=ICP["tol_t"], rotation_tolerance_rad=ICP["tol_r"],
                                         relative_cost_tolerance=ICP["tol_c"], translation_seed=last_v * dt)
        except Exception:
            continue
        if r.converged and math.isfinite(r.rmse_m):
            last_v = r.t / dt; last_w = math.atan2(r.R[1, 0], r.R[0, 0]) / dt
        vx = float(r.t[0] / dt)
        rows.append(dict(
            t_s=(mid - scans[0][0]) * 1e-9, lidar_vx=vx, icp_rmse_m=r.rmse_m, converged=bool(r.converged),
            yaw_rate=float(yaw), erpm=float(_interp(mid, ct, cerpm)),
            imu_ax=float(_interp(mid, it, iax) - ax_bias), motor_current=float(_interp(mid, ct, ccur)),
        ))
    import pandas as pd
    return pd.DataFrame(rows), ax_bias


def fit_origin(x, y):
    return float(np.dot(x, y) / max(np.dot(x, x), 1e-12))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=Path("/home/akselmo/Documents/GitHub/BachelorProject/tools"))
    args = ap.parse_args()
    import pandas as pd
    df, ax_bias = icp_table(args.bag)
    df.to_csv(args.out / "erpm_velocity_samples.csv", index=False)

    fin = df[np.isfinite(df.erpm) & np.isfinite(df.lidar_vx) & np.isfinite(df.imu_ax)].copy()
    # Forward motion only.
    motion = fin[(fin.erpm > 200) & (fin.lidar_vx > 0.2)].copy()
    # Trustworthy steady samples for the static map.
    clean = motion[(motion.converged) & (motion.icp_rmse_m <= 0.03) &
                   (motion.yaw_rate.abs() <= 0.16) & (motion.imu_ax.abs() <= 0.4)].copy()

    e_clean = clean.erpm.to_numpy(); v_clean = clean.lidar_vx.to_numpy()
    gain_clean = fit_origin(e_clean, v_clean)                      # v = erpm/gain_erpm_per_mps
    # quadratic-in-erpm static map: v = a*erpm + b*erpm*|erpm|
    Xq = np.column_stack([e_clean, e_clean * np.abs(e_clean)])
    aq, bq = np.linalg.lstsq(Xq, v_clean, rcond=None)[0]
    # naive scalar on ALL motion (contaminated by high-accel lidar noise + slip)
    gain_all = fit_origin(motion.erpm.to_numpy(), motion.lidar_vx.to_numpy())

    def lin(e): return gain_clean * e
    def quad(e): return aq * e + bq * e * np.abs(e)

    # Residual of the clean linear map vs lidar over all forward motion.
    motion["res_lin"] = motion.lidar_vx - lin(motion.erpm.to_numpy())
    # Causal correction: regress residual on imu_ax (+ motor_current), weighting
    # by ICP quality so noisy high-accel pairs do not dominate the slope.
    valid = motion[motion.converged & np.isfinite(motion.res_lin)].copy()
    w = 1.0 / np.maximum(valid.icp_rmse_m.to_numpy(), 0.005) ** 2
    A = np.column_stack([valid.imu_ax.to_numpy(), valid.motor_current.to_numpy(), np.ones(len(valid))])
    coef = np.linalg.lstsq(A * np.sqrt(w)[:, None], valid.res_lin.to_numpy() * np.sqrt(w), rcond=None)[0]
    c_ax, c_cur, c_b = map(float, coef)

    def fused(e, ax, cur): return lin(e) + c_ax * ax + c_cur * cur + c_b

    # Evaluate by acceleration regime against lidar (valid pairs only).
    def rmse(pred, truth): r = pred - truth; r = r[np.isfinite(r)]; return float(np.sqrt(np.mean(r**2))) if len(r) else math.nan
    def report_regime(name, mask):
        sub = valid[mask]
        if len(sub) < 10:
            return {"regime": name, "n": int(len(sub))}
        truth = sub.lidar_vx.to_numpy()
        return {
            "regime": name, "n": int(len(sub)),
            "rmse_scalar_all_mps": rmse(gain_all * sub.erpm.to_numpy(), truth),
            "rmse_clean_linear_mps": rmse(lin(sub.erpm.to_numpy()), truth),
            "rmse_clean_quadratic_mps": rmse(quad(sub.erpm.to_numpy()), truth),
            "rmse_fused_accel_mps": rmse(fused(sub.erpm.to_numpy(), sub.imu_ax.to_numpy(), sub.motor_current.to_numpy()), truth),
            "median_signed_slip_clean_linear_mps": float(np.nanmedian(lin(sub.erpm.to_numpy()) - truth)),
            "median_icp_rmse_m": float(np.nanmedian(sub.icp_rmse_m)),
        }
    ax_abs = valid.imu_ax.abs().to_numpy()
    regimes = [
        report_regime("steady |a|<0.5", ax_abs < 0.5),
        report_regime("moderate 0.5-1.5", (ax_abs >= 0.5) & (ax_abs < 1.5)),
        report_regime("high |a|>=1.5", ax_abs >= 1.5),
    ]

    report = {
        "bag": args.bag.name,
        "samples_total": int(len(df)),
        "forward_motion_samples": int(len(motion)),
        "clean_static_map_samples": int(len(clean)),
        "imu_ax_stationary_bias_removed_mps2": ax_bias,
        "static_map": {
            "linear_erpm_per_mps": 1.0 / gain_clean if gain_clean > 0 else None,
            "linear_gain_mps_per_erpm": gain_clean,
            "quadratic_mps_per_erpm": aq,
            "quadratic_mps_per_erpm2": bq,
            "naive_scalar_all_data_gain_mps_per_erpm": gain_all,
            "naive_vs_clean_gain_shift_pct": 100.0 * (gain_all - gain_clean) / gain_clean if gain_clean else None,
        },
        "causal_accel_correction": {
            "model": "v_hat = clean_linear(erpm) + c_ax*imu_ax + c_cur*motor_current + c_b",
            "c_ax_mps_per_mps2": c_ax,
            "c_cur_mps_per_amp": c_cur,
            "c_b_mps": c_b,
        },
        "by_acceleration_regime": regimes,
    }
    (args.out / "erpm_velocity_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
