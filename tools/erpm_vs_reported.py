#!/usr/bin/env python3
"""Show that reported odom speed == ERPM*gain (circular) and quantify how the
true LiDAR ground speed diverges from it, including a time-lag search.

Uses the per-pair LiDAR velocity already computed in tools/erpm_velocity_samples.csv
(independent scan-matching truth) and reads the reported /ego_racecar/odom speed,
/sensors/core ERPM and /sensors/imu/raw acceleration directly from the bag.
"""
from __future__ import annotations
import argparse, math, sys
from pathlib import Path
import numpy as np
import pandas as pd

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def origin_gain(x, y):
    return float(np.dot(x, y) / max(np.dot(x, x), 1e-12))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", type=Path)
    ap.add_argument("--csv", type=Path, default=Path("/home/akselmo/Documents/GitHub/BachelorProject/tools/erpm_velocity_samples.csv"))
    args = ap.parse_args()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(args.bag), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {e.name: e.type for e in reader.get_all_topics_and_types()}
    odom_cls = get_message(types["/ego_racecar/odom"])
    core_cls = get_message(types["/sensors/core"])
    scan_cls = get_message(types["/scan"])
    t0 = None
    ot, ovx, ct, cerpm = [], [], [], []
    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic == "/scan" and t0 is None:
            m = deserialize_message(raw, scan_cls)
            s = m.header.stamp; t0 = int(s.sec) * 1_000_000_000 + int(s.nanosec) or bag_ns
        elif topic == "/ego_racecar/odom":
            m = deserialize_message(raw, odom_cls)
            s = m.header.stamp; ot.append((int(s.sec)*1_000_000_000+int(s.nanosec)) or bag_ns); ovx.append(float(m.twist.twist.linear.x))
        elif topic == "/sensors/core":
            m = deserialize_message(raw, core_cls)
            s = m.header.stamp; ct.append((int(s.sec)*1_000_000_000+int(s.nanosec)) or bag_ns); cerpm.append(float(m.state.speed))
    ot, ovx, ct, cerpm = map(lambda a: np.array(a, float), (ot, ovx, ct, cerpm))

    # (1) Circularity: reported odom speed vs ERPM at the same instants.
    erpm_at_odom = np.interp(ot, ct, cerpm)
    move = np.abs(erpm_at_odom) > 200
    k_odom = origin_gain(erpm_at_odom[move], ovx[move])     # mps per erpm
    pred = k_odom * erpm_at_odom[move]
    rmse_circular = float(np.sqrt(np.mean((pred - ovx[move]) ** 2)))
    print(f"[circularity] reported odom vs ERPM:  gain = {1/k_odom:.2f} ERPM/(m/s)   RMSE = {rmse_circular:.4f} m/s")
    print(f"              -> odom is ERPM*gain to within {rmse_circular*1000:.1f} mm/s; it is NOT an independent speed.\n")

    # (2) LiDAR (true) vs reported. reported = ERPM * k_odom.
    df = pd.read_csv(args.csv)
    df = df[np.isfinite(df.lidar_vx) & np.isfinite(df.erpm) & (df.erpm > 200) & df.converged].copy()
    df["reported"] = k_odom * df.erpm
    df["resid"] = df.lidar_vx - df.reported           # true - reported
    # smooth to suppress per-pair LiDAR noise for the systematic/lag analysis
    df = df.sort_values("t_s").reset_index(drop=True)
    df["lidar_s"] = df.lidar_vx.rolling(7, center=True, min_periods=3).median()
    df["reported_s"] = df.reported.rolling(7, center=True, min_periods=3).median()
    s = df.dropna(subset=["lidar_s", "reported_s"]).copy()

    print(f"[true vs reported]  n={len(s)}")
    print(f"  corr(resid, signed_accel) = {np.corrcoef(s.resid, s.imu_ax)[0,1]:+.3f}  (slip/lag is signed; pure noise ~0)")
    print(f"  median resid by signed accel regime (true - reported):")
    for lo, hi, name in [(-99,-1.5,"hard brake a<-1.5"),(-1.5,-0.5,"brake"),(-0.5,0.5,"cruise"),(0.5,1.5,"drive"),(1.5,99,"hard drive a>1.5")]:
        b = s[(s.imu_ax>=lo)&(s.imu_ax<hi)]
        if len(b)>10:
            print(f"    {name:18s} n={len(b):4d}  median={b.resid.median():+.3f}  mean(smoothed)={float((b.lidar_s-b.reported_s).mean()):+.3f} m/s")

    # (3) Time-lag search: shift reported in time, find lag minimising RMSE to LiDAR.
    t = s.t_s.to_numpy(); lv = s.lidar_s.to_numpy(); rp = s.reported_s.to_numpy()
    best = (0.0, math.inf)
    grid = np.arange(-0.30, 0.301, 0.01)
    base_rmse = None
    for lag in grid:
        rp_shift = np.interp(t, t + lag, rp, left=np.nan, right=np.nan)
        d = lv - rp_shift; d = d[np.isfinite(d)]
        if len(d) < 100: continue
        r = float(np.sqrt(np.mean(d**2)))
        if abs(lag) < 1e-9: base_rmse = r
        if r < best[1]: best = (float(lag), r)
    print(f"\n[time lag]  best lag = {best[0]*1000:+.0f} ms  (RMSE {base_rmse:.3f} -> {best[1]:.3f} m/s)")
    print( "            positive lag => reported ERPM speed LAGS true ground speed by that much.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
