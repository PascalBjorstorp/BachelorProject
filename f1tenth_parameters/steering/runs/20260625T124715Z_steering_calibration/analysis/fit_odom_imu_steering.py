#!/usr/bin/env python3
"""Steering tuning + steering-rate identification from odometry speed + IMU yaw.

Why not LiDAR ICP forward velocity: per-frame ICP `vx` for this vehicle/scene is
unobservable (0.6 m/s at 40 Hz ~ 1.5 cm/frame, below LiDAR range noise). Measured
`vx` ranges -5..+7 m/s within a single steady run, is biased ~68% low vs wheel
odometry, and its quality metrics do not separate good from bad samples. Wheel
odometry speed (VESC VEL_TO_ERPM, encoder) is steady to <1% and IMU yaw is clean,
so curvature kappa = yaw_rate / v and delta_eq = arctan(L*kappa) are built from
those two channels.
"""
from __future__ import annotations
import json, re, math
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.signal import savgol_filter

SES = Path("/home/akselmo/Downloads/steering/runs/20260625T124715Z_steering_calibration")
OUT = SES / "analysis"
L = 0.324  # wheelbase m

bias = json.loads((OUT / "imu_bias.json").read_text())
GZ_BIAS = float(bias["gz_bias_intercept_rad_s"])
centre = json.loads((OUT / "centre_trim_offline.json").read_text())
CENTRE_SERVO = float(centre["centre_servo_raw"])

TRIAL_RE = re.compile(r"(high_raw|low_raw)_([0-9.]+)_(outward|inward|shuffled)")

def parse_trial(tid: str):
    m = TRIAL_RE.search(str(tid))
    if not m:
        return None, None, None
    return m.group(1), float(m.group(2)), m.group(3)

def accepted_ids(ev: pd.DataFrame) -> set[str]:
    dec = ev[ev.event == "trial_decision"]
    ok = dec[dec.decision.astype(str).str.lower().str.contains("accept")]
    return set(ok.trial_id.astype(str))

def load(stage, name):
    return pd.read_parquet(SES / stage / "derived" / f"{name}.parquet")

def mad(a):
    a = np.asarray(a, float); a = a[np.isfinite(a)]
    if a.size == 0: return float("nan")
    return float(1.4826 * np.median(np.abs(a - np.median(a))))

# ----------------------------------------------------------------------------
# Deliverable 1: static steering map (raw servo -> effective steering angle)
# ----------------------------------------------------------------------------
def static_segments(stage, trim_s=1.5):
    ev = load(stage, "events"); im = load(stage, "imu")
    od = load(stage, "odom"); ec = load(stage, "servo_echo")
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
        if len(imw) < 20 or len(odw) < 20 or len(ecw) < 3:
            continue
        side, frac, approach = parse_trial(tid)
        v = float(odw.vx.median()); yaw = float(imw.gz.median()) - GZ_BIAS
        rows.append(dict(
            stage=stage, trial_id=tid, side=side, fraction=frac, approach=approach,
            raw_echo=float(ecw.value.median()),
            odom_v=v, odom_v_mad=mad(odw.vx),
            yaw_rate=yaw, yaw_rate_mad=mad(imw.gz),
            curvature=yaw / v, delta_eq_rad=float(np.arctan(L * yaw / v)),
        ))
    df = pd.DataFrame(rows)
    df["delta_eq_deg"] = np.degrees(df.delta_eq_rad)
    return df

train = static_segments("04_static_map_training")
hold = static_segments("05_static_map_holdout")

# nominal-condition collapse (side, fraction) -> median
def collapse(df):
    g = df.groupby(["side", "fraction"], dropna=False).agg(
        n=("delta_eq_rad", "size"),
        raw_echo=("raw_echo", "median"),
        delta_rad=("delta_eq_rad", "median"),
        delta_deg=("delta_eq_deg", "median"),
        delta_rad_std=("delta_eq_rad", "std"),
        v=("odom_v", "median"),
    ).reset_index().sort_values("raw_echo").reset_index(drop=True)
    return g

cond = collapse(train)
# build interpolation map: anchor centre at 0 rad
x = np.r_[CENTRE_SERVO, cond.raw_echo.to_numpy()]
y = np.r_[0.0, cond.delta_rad.to_numpy()]
o = np.argsort(x); x, y = x[o], y[o]
local_gain = np.gradient(y, x)  # rad per servo unit
# global linear fit through map (servo->rad)
A = np.polyfit(x, y, 1)
glob_gain = float(A[0])  # rad per servo unit

# holdout validation against the training interpolation
hold_eval = hold.copy()
hold_eval["delta_pred_rad"] = np.interp(hold_eval.raw_echo, x, y)
hold_eval["err_rad"] = hold_eval.delta_pred_rad - hold_eval.delta_eq_rad
hold_rmse = float(np.sqrt(np.mean(hold_eval.err_rad ** 2)))
hold_bias = float(hold_eval.err_rad.mean())

# repeatability (std across repeats within nominal condition) and hysteresis
rep = train.groupby(["side", "fraction", "approach"]).agg(
    delta_rad=("delta_eq_rad", "median"), n=("delta_eq_rad", "size")).reset_index()
pivot = rep.pivot_table(index=["side", "fraction"], columns="approach",
                        values="delta_rad").reset_index()
hyst = None
if {"outward", "inward"}.issubset(pivot.columns):
    pivot["hysteresis_rad"] = pivot["outward"] - pivot["inward"]
    hyst = float(np.median(np.abs(pivot["hysteresis_rad"].dropna())))
rep_std = float(np.median(train.groupby(["side", "fraction", "approach"])
                          .delta_eq_rad.std().dropna()))

static_map = dict(
    method="odom speed + IMU yaw (LiDAR ICP vx rejected: unobservable, see header)",
    wheelbase_m=L, centre_servo_raw=CENTRE_SERVO,
    raw_servo=x.tolist(), delta_eq_rad=y.tolist(),
    delta_eq_deg=np.degrees(y).tolist(),
    local_gain_rad_per_servo=local_gain.tolist(),
    global_linear_gain_rad_per_servo=glob_gain,
    global_linear_gain_deg_per_servo=math.degrees(glob_gain),
    training_points=int(len(train)), training_conditions=int(len(cond)),
    holdout_points=int(len(hold)),
    holdout_rmse_rad=hold_rmse, holdout_rmse_deg=math.degrees(hold_rmse),
    holdout_bias_rad=hold_bias,
    repeatability_median_std_rad=rep_std, repeatability_median_std_deg=math.degrees(rep_std),
    hysteresis_median_abs_rad=hyst,
    hysteresis_median_abs_deg=None if hyst is None else math.degrees(hyst),
)
(OUT / "odom_imu_static_map.json").write_text(json.dumps(static_map, indent=2) + "\n")
cond.to_csv(OUT / "odom_imu_static_map_conditions.csv", index=False)
pd.concat([train.assign(split="train"), hold.assign(split="holdout")]
          ).to_csv(OUT / "odom_imu_static_map_segments.csv", index=False)

# ----------------------------------------------------------------------------
# Deliverable 2: steering RATE (effective) from stage 6 step responses
# ----------------------------------------------------------------------------
ev6 = load("06_command_to_curvature_response", "events")
im6 = load("06_command_to_curvature_response", "imu").sort_values("bag_ns")
od6 = load("06_command_to_curvature_response", "odom").sort_values("bag_ns")
acc6 = accepted_ids(ev6)
steps = ev6[(ev6.event == "phase_start") & (ev6.phase == "response_step")]
ends6 = ev6[(ev6.event == "phase_end") & (ev6.phase == "response_step")]

imt = im6.bag_ns.to_numpy(float); imgz = im6.gz.to_numpy(float) - GZ_BIAS
odt = od6.bag_ns.to_numpy(float); odv = od6.vx.to_numpy(float)
DT = 1.0 / 200.0
WIN = 21  # SavGol window (~0.105 s)

step_rows = []
for _, s in steps.iterrows():
    tid = str(s.trial_id)
    if tid not in acc6:
        continue
    e = ends6[(ends6.trial_id.astype(str) == tid) & (ends6.bag_ns > s.bag_ns)]
    if not len(e):
        continue
    t0 = int(s.bag_ns); t1 = int(e.iloc[0].bag_ns)
    side = s.side; speed = float(s.speed_mps)
    # resample on a uniform 200 Hz grid from t0-0.3s to step end
    g0 = t0 - int(0.3e9)
    grid = np.arange(g0, t1, int(DT * 1e9), dtype=float)
    yaw = np.interp(grid, imt, imgz)
    v = np.interp(grid, odt, odv)
    v = np.where(np.abs(v) < 0.2, np.nan, v)
    delta = np.arctan(L * yaw / v)
    if np.isnan(delta).mean() > 0.3:
        continue
    delta = pd.Series(delta).interpolate(limit_direction="both").to_numpy()
    delta_s = savgol_filter(delta, WIN, 2)
    t_rel = (grid - t0) * 1e-9
    pre = t_rel < 0.0
    d0 = float(np.median(delta_s[pre])) if pre.any() else 0.0
    post = t_rel >= 0.0
    ss_mask = t_rel >= (t_rel.max() - 1.0)
    d_ss = float(np.median(delta_s[ss_mask]))
    D = d_ss - d0
    if abs(D) < math.radians(1.0):
        continue
    norm = (delta_s - d0) / D
    rate = np.gradient(delta_s, t_rel)  # rad/s
    # restrict transient to [t0, t0+1.5s] for peak rate
    tr = post & (t_rel <= 1.5)
    peak_rate = float(np.max(np.abs(rate[tr])))
    # delay & rise from normalized response
    def t_cross(frac):
        idx = np.where(post & (norm >= frac))[0]
        return float(t_rel[idx[0]]) if len(idx) else float("nan")
    t10, t90 = t_cross(0.1), t_cross(0.9)
    # 5% settling
    band = 0.05
    settled = np.where(post & (np.abs(norm - 1.0) <= band))[0]
    t_settle = float("nan")
    for i in settled:
        if np.all(np.abs(norm[i:] - 1.0) <= band):
            t_settle = float(t_rel[i]); break
    overshoot = float(np.max(norm[tr]) - 1.0)
    step_rows.append(dict(
        trial_id=tid, side=side, speed_mps=speed,
        raw_target=float(s.raw_servo_target),
        delta_ss_rad=d_ss, delta_ss_deg=math.degrees(d_ss),
        step_mag_rad=D, step_mag_deg=math.degrees(D),
        delay_10pct_s=t10, rise_10_90_s=(t90 - t10),
        settle_5pct_s=t_settle, overshoot_frac=overshoot,
        peak_rate_rad_s=peak_rate, peak_rate_deg_s=math.degrees(peak_rate),
    ))

resp = pd.DataFrame(step_rows)
resp.to_csv(OUT / "odom_imu_response_steps.csv", index=False)

def agg(df):
    return dict(
        n=int(len(df)),
        peak_rate_rad_s_median=float(df.peak_rate_rad_s.median()),
        peak_rate_deg_s_median=float(df.peak_rate_deg_s.median()),
        peak_rate_deg_s_max=float(df.peak_rate_deg_s.max()),
        delay_s_median=float(df.delay_10pct_s.median()),
        rise_10_90_s_median=float(df.rise_10_90_s.median()),
        settle_5pct_s_median=float(df.settle_5pct_s.median()),
        overshoot_frac_median=float(df.overshoot_frac.median()),
    )

rate_report = dict(
    method="effective delta_eq = arctan(L*yaw_IMU/v_odom); peak |d delta/dt| over first 1.5 s",
    note="effective vehicle steering rate (command+servo+linkage+tyre+yaw), not isolated servo shaft",
    overall=agg(resp),
    by_speed={f"{k:.1f}": agg(v) for k, v in resp.groupby("speed_mps")},
    by_speed_side={f"{k0:.1f}_{k1}": agg(v) for (k0, k1), v in resp.groupby(["speed_mps", "side"])},
)
(OUT / "odom_imu_steering_rate.json").write_text(json.dumps(rate_report, indent=2) + "\n")

print("=== STATIC MAP (steering tuning) ===")
print(json.dumps({k: static_map[k] for k in [
    "centre_servo_raw", "global_linear_gain_rad_per_servo",
    "global_linear_gain_deg_per_servo", "training_points", "holdout_points",
    "holdout_rmse_deg", "repeatability_median_std_deg", "hysteresis_median_abs_deg"]}, indent=2))
print("\ncondition table:")
print(cond.to_string(index=False))
print("\n=== STEERING RATE ===")
print(json.dumps(rate_report, indent=2))
