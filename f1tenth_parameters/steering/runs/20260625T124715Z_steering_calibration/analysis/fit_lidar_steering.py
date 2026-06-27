#!/usr/bin/env python3
"""Steering tuning + steering rate from baseline (viable) LiDAR ICP velocity.

Primary speed source: lidar_velocity_baseline.parquet (displacement-targeted ICP,
forward velocity now observable). Wheel odometry is carried alongside purely as an
independent cross-check. Yaw is IMU. delta_eq = arctan(L * yaw_IMU / v_lidar).
"""
from __future__ import annotations
import json, re, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

SES = Path("/home/akselmo/Downloads/steering/runs/20260625T124715Z_steering_calibration")
OUT = SES / "analysis"
L = 0.324
GZ_BIAS = float(json.loads((OUT / "imu_bias.json").read_text())["gz_bias_intercept_rad_s"])
CENTRE_SERVO = float(json.loads((OUT / "centre_trim_offline.json").read_text())["centre_servo_raw"])
TRIAL_RE = re.compile(r"(high_raw|low_raw)_([0-9.]+)_(outward|inward|shuffled)")


def parse_trial(t):
    m = TRIAL_RE.search(str(t))
    return (m.group(1), float(m.group(2)), m.group(3)) if m else (None, None, None)


def accepted_ids(ev):
    d = ev[ev.event == "trial_decision"]
    return set(d[d.decision.astype(str).str.lower().str.contains("accept")].trial_id.astype(str))


def load(stage, name):
    return pd.read_parquet(SES / stage / "derived" / f"{name}.parquet")


def mad(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    return float("nan") if a.size == 0 else float(1.4826 * np.median(np.abs(a - np.median(a))))


# ---------------- static map ----------------
def static_segments(stage, trim_s=1.5):
    ev = load(stage, "events"); im = load(stage, "imu"); od = load(stage, "odom")
    ec = load(stage, "servo_echo"); lv = load(stage, "lidar_velocity_baseline")
    acc = accepted_ids(ev)
    starts = ev[(ev.event == "phase_start") & (ev.phase == "static_map_capture")]
    ends = ev[(ev.event == "phase_end") & (ev.phase == "static_map_capture")]
    rows = []
    for _, s in starts.iterrows():
        tid = str(s.trial_id)
        if tid not in acc:
            continue
        e = ends[(ends.trial_id.astype(str) == tid) & (ends.bag_ns > s.bag_ns)]
        if not len(e):
            continue
        a = int(s.bag_ns + trim_s * 1e9); b = int(e.iloc[0].bag_ns - trim_s * 1e9)
        if b <= a:
            continue
        imw = im[(im.bag_ns >= a) & (im.bag_ns <= b)]
        odw = od[(od.bag_ns >= a) & (od.bag_ns <= b)]
        ecw = ec[(ec.bag_ns >= a) & (ec.bag_ns <= b)]
        lvw = lv[(lv.bag_ns >= a) & (lv.bag_ns <= b)]
        lvv = lvw[lvw.valid]
        if len(imw) < 20 or len(ecw) < 3 or len(lvw) < 5:
            continue
        vfrac = len(lvv) / max(len(lvw), 1)
        # Gate on absolute valid-pair count, not fraction: the low fraction is
        # geometry/overlap-driven (longer baseline => fewer correspondences), not
        # a quality failure. ~12 valid pairs already give a robust median speed.
        if len(lvv) < 12:
            continue
        side, frac, approach = parse_trial(tid)
        v_l = float(lvv.vx.median()); v_o = float(odw.vx.median())
        yaw = float(imw.gz.median()) - GZ_BIAS
        rows.append(dict(stage=stage, trial_id=tid, side=side, fraction=frac, approach=approach,
                         raw_echo=float(ecw.value.median()),
                         v_lidar=v_l, v_lidar_mad=mad(lvv.vx), valid_frac=vfrac, n_lidar=len(lvv),
                         v_odom=v_o, yaw_rate=yaw,
                         curvature=yaw / v_l, delta_eq_rad=float(np.arctan(L * yaw / v_l)),
                         delta_eq_odom_rad=float(np.arctan(L * yaw / v_o))))
    df = pd.DataFrame(rows)
    if len(df):
        df["delta_eq_deg"] = np.degrees(df.delta_eq_rad)
    return df


train = static_segments("04_static_map_training")
hold = static_segments("05_static_map_holdout")


def collapse(df):
    return (df.groupby(["side", "fraction"], dropna=False)
            .agg(n=("delta_eq_rad", "size"), raw_echo=("raw_echo", "median"),
                 delta_rad=("delta_eq_rad", "median"), delta_deg=("delta_eq_deg", "median"),
                 delta_rad_std=("delta_eq_rad", "std"),
                 delta_odom_rad=("delta_eq_odom_rad", "median"),
                 v_lidar=("v_lidar", "median"), v_odom=("v_odom", "median"))
            .reset_index().sort_values("raw_echo").reset_index(drop=True))


cond = collapse(train)
x = np.r_[CENTRE_SERVO, cond.raw_echo.to_numpy()]; y = np.r_[0.0, cond.delta_rad.to_numpy()]
o = np.argsort(x); x, y = x[o], y[o]
local_gain = np.gradient(y, x); glob = float(np.polyfit(x, y, 1)[0])
he = hold.copy(); he["pred"] = np.interp(he.raw_echo, x, y); he["err"] = he.pred - he.delta_eq_rad
hold_rmse = float(np.sqrt(np.mean(he.err ** 2))); hold_bias = float(he.err.mean())
rep_std = float(np.median(train.groupby(["side", "fraction", "approach"]).delta_eq_rad.std().dropna()))
piv = (train.groupby(["side", "fraction", "approach"]).delta_eq_rad.median().reset_index()
       .pivot_table(index=["side", "fraction"], columns="approach", values="delta_eq_rad").reset_index())
hyst = float(np.median(np.abs((piv["outward"] - piv["inward"]).dropna()))) if {"outward", "inward"}.issubset(piv.columns) else None
# lidar-vs-odom cross check
xcheck_speed = float((train.v_lidar / train.v_odom).median())
xcheck_delta_deg = float(np.degrees(np.median(np.abs(train.delta_eq_rad - train.delta_eq_odom_rad))))

static_map = dict(
    method="baseline LiDAR ICP speed + IMU yaw; odometry = independent cross-check",
    wheelbase_m=L, centre_servo_raw=CENTRE_SERVO,
    raw_servo=x.tolist(), delta_eq_rad=y.tolist(), delta_eq_deg=np.degrees(y).tolist(),
    local_gain_rad_per_servo=local_gain.tolist(),
    global_linear_gain_rad_per_servo=glob, global_linear_gain_deg_per_servo=math.degrees(glob),
    training_points=int(len(train)), training_conditions=int(len(cond)), holdout_points=int(len(hold)),
    holdout_rmse_rad=hold_rmse, holdout_rmse_deg=math.degrees(hold_rmse), holdout_bias_rad=hold_bias,
    repeatability_median_std_deg=math.degrees(rep_std),
    hysteresis_median_abs_deg=None if hyst is None else math.degrees(hyst),
    crosscheck_lidar_over_odom_speed_ratio=xcheck_speed,
    crosscheck_lidar_vs_odom_delta_median_abs_deg=xcheck_delta_deg)
(OUT / "lidar_static_map.json").write_text(json.dumps(static_map, indent=2) + "\n")
cond.to_csv(OUT / "lidar_static_map_conditions.csv", index=False)

# ---------------- steering rate (stage 6) ----------------
ev6 = load("06_command_to_curvature_response", "events")
im6 = load("06_command_to_curvature_response", "imu").sort_values("bag_ns")
lv6 = load("06_command_to_curvature_response", "lidar_velocity_baseline")
od6 = load("06_command_to_curvature_response", "odom").sort_values("bag_ns")
acc6 = accepted_ids(ev6)
steps = ev6[(ev6.event == "phase_start") & (ev6.phase == "response_step")]
ends6 = ev6[(ev6.event == "phase_end") & (ev6.phase == "response_step")]
imt = im6.bag_ns.to_numpy(float); imgz = im6.gz.to_numpy(float) - GZ_BIAS
DT = 1 / 200.0; WIN = 21
srows = []
for _, s in steps.iterrows():
    tid = str(s.trial_id)
    if tid not in acc6:
        continue
    e = ends6[(ends6.trial_id.astype(str) == tid) & (ends6.bag_ns > s.bag_ns)]
    if not len(e):
        continue
    t0 = int(s.bag_ns); t1 = int(e.iloc[0].bag_ns)
    # per-step LiDAR speed (steady during a step): median of valid baseline pairs
    lseg = lv6[(lv6.bag_ns >= t0) & (lv6.bag_ns <= t1) & (lv6.valid)]
    oseg = od6[(od6.bag_ns >= t0) & (od6.bag_ns <= t1)]
    v_l = float(lseg.vx.median()) if len(lseg) >= 3 else float("nan")
    v_o = float(oseg.vx.median())
    v_use = v_l if np.isfinite(v_l) and abs(v_l) > 0.2 else v_o
    grid = np.arange(t0 - int(0.3e9), t1, int(DT * 1e9), dtype=float)
    yaw = np.interp(grid, imt, imgz)
    delta = np.arctan(L * yaw / v_use)
    ds = savgol_filter(delta, WIN, 2); tr = (grid - t0) * 1e-9
    d0 = float(np.median(ds[tr < 0])); d_ss = float(np.median(ds[tr >= tr.max() - 1.0]))
    D = d_ss - d0
    if abs(D) < math.radians(1.0):
        continue
    norm = (ds - d0) / D; rate = np.gradient(ds, tr); post = tr >= 0
    peak = float(np.max(np.abs(rate[post & (tr <= 1.5)])))

    def tc(f):
        idx = np.where(post & (norm >= f))[0]
        return float(tr[idx[0]]) if len(idx) else float("nan")
    t10, t90 = tc(0.1), tc(0.9)
    srows.append(dict(trial_id=tid, side=s.side, speed_mps=float(s.speed_mps),
                      v_lidar=v_l, v_odom=v_o,
                      delta_ss_deg=math.degrees(d_ss), step_mag_deg=math.degrees(D),
                      delay_s=t10, rise_10_90_s=t90 - t10,
                      peak_rate_rad_s=peak, peak_rate_deg_s=math.degrees(peak)))
resp = pd.DataFrame(srows)
resp.to_csv(OUT / "lidar_response_steps.csv", index=False)


def agg(d):
    return dict(n=int(len(d)), peak_rate_rad_s_median=float(d.peak_rate_rad_s.median()),
                peak_rate_deg_s_median=float(d.peak_rate_deg_s.median()),
                peak_rate_deg_s_max=float(d.peak_rate_deg_s.max()),
                delay_s_median=float(d.delay_s.median()), rise_10_90_s_median=float(d.rise_10_90_s.median()))


rate = dict(method="delta_eq=arctan(L*yaw_IMU/v); v=baseline LiDAR (per-step median), odom cross-check",
            overall=agg(resp), by_speed={f"{k:.1f}": agg(v) for k, v in resp.groupby("speed_mps")})
(OUT / "lidar_steering_rate.json").write_text(json.dumps(rate, indent=2) + "\n")

print("=== LiDAR STATIC MAP ===")
print(json.dumps({k: static_map[k] for k in ["centre_servo_raw", "global_linear_gain_deg_per_servo",
      "training_points", "holdout_points", "holdout_rmse_deg", "repeatability_median_std_deg",
      "hysteresis_median_abs_deg", "crosscheck_lidar_over_odom_speed_ratio",
      "crosscheck_lidar_vs_odom_delta_median_abs_deg"]}, indent=2))
print(cond[["side", "fraction", "raw_echo", "delta_deg", "delta_odom_rad", "v_lidar", "v_odom", "delta_rad_std"]].to_string(index=False))
print("\n=== LiDAR STEERING RATE ===")
print(json.dumps(rate, indent=2))
