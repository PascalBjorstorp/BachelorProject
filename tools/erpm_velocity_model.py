#!/usr/bin/env python3
"""Test whether the turn-dependent ERPM/velocity gap is physical (fittable) or a
LiDAR-under-rotation artifact, and if physical, fit + cross-validate a causal
yaw-aware ERPM -> longitudinal-velocity model.

Stores full LiDAR motion (vx, vy, yaw) + ICP quality so we can:
  (1) check LiDAR reliability vs yaw (rmse, inlier, ICP-yaw vs IMU-yaw),
  (2) see if wheel speed tracks vx or total |v| in corners (sideslip vs slip),
  (3) fit vx_est = k*ERPM (+ yaw/lateral correction) on a train split and report
      held-out RMSE vs the plain scalar.
"""
from __future__ import annotations
import argparse, math, sys
from pathlib import Path
import numpy as np

ANALYSIS = Path("/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_parameters/steering/analysis")
sys.path.insert(0, str(ANALYSIS))
from estimate_lidar_motion import deskew_scan, pca_normals, robust_point_to_line_icp, stamp_ns, scan_points  # noqa: E402
import rosbag2_py  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

LASER_X = 0.265  # m, laser forward of base_link (from config)
ICP = dict(max_iter=120, max_corr=0.3, trim_fraction=0.85, min_corr=120, nn=9, tol=1e-6, tolc=1e-7)


def collect(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="mcap"),
           rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {e.name: e.type for e in r.get_all_topics_and_types()}
    sc = get_message(types["/scan"]); ic = get_message(types["/sensors/imu/raw"]); cc = get_message(types["/sensors/core"])
    scans, it, igz, iay, ct, ce, ccur = [], [], [], [], [], [], []
    while r.has_next():
        topic, raw, b = r.read_next()
        if topic == "/scan":
            m = deserialize_message(raw, sc); p, rel = scan_points(m, 1); scans.append((stamp_ns(m, b), p, rel))
        elif topic == "/sensors/imu/raw":
            m = deserialize_message(raw, ic); it.append(stamp_ns(m, b)); igz.append(float(m.angular_velocity.z)); iay.append(float(m.linear_acceleration.y))
        elif topic == "/sensors/core":
            m = deserialize_message(raw, cc); ct.append(stamp_ns(m, b)); ce.append(float(m.state.speed)); ccur.append(float(m.state.current_motor))
    f = lambda a: np.array(a, float)
    return scans, f(it), f(igz), f(iay), f(ct), f(ce), f(ccur)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("bag", type=Path)
    ap.add_argument("--out", type=Path, default=Path("/home/akselmo/Documents/GitHub/BachelorProject/tools"))
    a = ap.parse_args()
    import pandas as pd
    scans, it, igz, iay, ct, ce, ccur = collect(a.bag)
    rows = []; last_v = np.zeros(2); last_w = 0.0
    for (pt, p, rel), (t, cur, crel) in zip(scans, scans[1:]):
        dt = (t - pt) * 1e-9
        if not (0.005 < dt <= 0.2): continue
        mid = 0.5 * (t + pt)
        yaw = float(np.interp(mid, it, igz))
        if not math.isfinite(yaw): continue
        om = float(np.interp(t, it, igz)); om = om if math.isfinite(om) else last_w
        try:
            tgt = deskew_scan(p, rel, last_v, om); src = deskew_scan(cur, crel, last_v, om)
            nm = pca_normals(tgt, ICP["nn"])
            res = robust_point_to_line_icp(src, tgt, nm, yaw_seed=yaw*dt, max_iter=ICP["max_iter"],
                  max_corr=ICP["max_corr"], trim_fraction=ICP["trim_fraction"], min_corr=ICP["min_corr"],
                  translation_tolerance_m=ICP["tol"], rotation_tolerance_rad=ICP["tol"],
                  relative_cost_tolerance=ICP["tolc"], translation_seed=last_v*dt)
        except Exception:
            continue
        if res.converged and math.isfinite(res.rmse_m):
            last_v = res.t/dt; last_w = math.atan2(res.R[1,0], res.R[0,0])/dt
        yaw_icp = math.atan2(res.R[1,0], res.R[0,0])/dt
        # laser->base: vx unchanged, vy_base = vy_laser - yaw_rate*LASER_X
        vx = res.t[0]/dt; vy_base = res.t[1]/dt - yaw_icp*LASER_X
        rows.append(dict(t_s=(mid-scans[0][0])*1e-9, vx=vx, vy=vy_base, speed=math.hypot(vx, vy_base),
            yaw_icp=yaw_icp, yaw_imu=yaw, rmse=res.rmse_m, inlier=res.inlier_ratio, converged=bool(res.converged),
            erpm=float(np.interp(mid, ct, ce)), motor_current=float(np.interp(mid, ct, ccur)),
            imu_gz=yaw, imu_ay=float(np.interp(mid, it, iay))))
    df = pd.DataFrame(rows); df.to_csv(a.out/"erpm_velocity_model_samples.csv", index=False)
    G = 4417.48; k_cfg = 1.0/G
    d = df[df.converged & (df.erpm > 500) & np.isfinite(df.vx)].copy()
    d["wheel"] = d.erpm * k_cfg; d["yaw_abs"] = d.yaw_imu.abs()
    print(f"rows={len(df)} usable={len(d)}\n")

    print("[1] LiDAR reliability vs |yaw| (does scan-matching degrade under rotation?)")
    print(f"{'|yaw| bin':14s}{'n':>6s}{'med_rmse_mm':>12s}{'med_inlier':>11s}{'med|yawICP-IMU|':>16s}")
    for lo, hi in [(0,0.2),(0.2,0.6),(0.6,1.0),(1.0,2.0),(2.0,9)]:
        b = d[(d.yaw_abs>=lo)&(d.yaw_abs<hi)]
        if len(b)>20:
            print(f"{lo:.1f}-{hi:<9.1f}{len(b):>6d}{1000*b.rmse.median():>12.2f}{b.inlier.median():>11.2f}{np.median(np.abs(b.yaw_icp-b.yaw_imu)):>16.3f}")

    print("\n[2] wheel speed vs longitudinal vx and total |v| by |yaw| (sideslip vs slip)")
    print(f"{'|yaw| bin':14s}{'n':>6s}{'med(wheel-vx)':>15s}{'med(wheel-|v|)':>16s}{'med|beta|deg':>14s}")
    for lo, hi in [(0,0.2),(0.2,0.6),(0.6,1.0),(1.0,2.0),(2.0,9)]:
        b = d[(d.yaw_abs>=lo)&(d.yaw_abs<hi)]
        if len(b)>20:
            beta = np.degrees(np.arctan2(b.vy, np.maximum(b.vx,0.1)))
            print(f"{lo:.1f}-{hi:<9.1f}{len(b):>6d}{float((b.wheel-b.vx).median()):>15.3f}{float((b.wheel-b.speed).median()):>16.3f}{float(np.median(np.abs(beta))):>14.2f}")

    print("\n[3] fit vx from ERPM, train/test split (gated to reliable pairs), target = LiDAR vx")
    g = d[(d.rmse<=0.03)&(d.inlier>=0.45)&(np.abs(d.yaw_icp-d.yaw_imu)<=0.15)&(d.vx>0.3)].copy().sort_values("t_s")
    n = len(g); tr = g.iloc[:int(0.7*n)]; te = g.iloc[int(0.7*n):]
    def rmse(p, y): r = p-y; r=r[np.isfinite(r)]; return float(np.sqrt(np.mean(r**2)))
    yx = tr.vx.to_numpy()
    # M0 plain scalar (fit on train)
    k0 = float(np.dot(tr.erpm, yx)/np.dot(tr.erpm, tr.erpm))
    # M1 yaw-corrected: vx = k*erpm + c*(erpm*|yaw|)
    X1 = np.column_stack([tr.erpm, tr.erpm*tr.yaw_abs]); c1 = np.linalg.lstsq(X1, yx, rcond=None)[0]
    # M2 add yaw^2 term: vx = k*erpm + c*erpm*|yaw| + d*erpm*yaw^2
    X2 = np.column_stack([tr.erpm, tr.erpm*tr.yaw_abs, tr.erpm*tr.yaw_abs**2]); c2 = np.linalg.lstsq(X2, yx, rcond=None)[0]
    print(f"  train n={len(tr)} test n={len(te)}")
    print(f"  configured scalar (1/{G:.0f})        test RMSE = {rmse(k_cfg*te.erpm, te.vx):.4f} m/s")
    print(f"  M0 refit scalar (1/{1/k0:.0f})        test RMSE = {rmse(k0*te.erpm, te.vx):.4f} m/s")
    print(f"  M1 +c*erpm*|yaw|                  test RMSE = {rmse(c1[0]*te.erpm + c1[1]*te.erpm*te.yaw_abs, te.vx):.4f} m/s   (k={1/c1[0]:.0f}, c={c1[1]:.2e})")
    print(f"  M2 +erpm*|yaw|+erpm*yaw^2         test RMSE = {rmse(c2[0]*te.erpm + c2[1]*te.erpm*te.yaw_abs + c2[2]*te.erpm*te.yaw_abs**2, te.vx):.4f} m/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
