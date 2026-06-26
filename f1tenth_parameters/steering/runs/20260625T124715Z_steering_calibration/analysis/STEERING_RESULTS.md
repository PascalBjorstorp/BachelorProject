# Steering calibration results

Session: `20260625T124715Z_steering_calibration`. Wheelbase L = 0.324 m.

## Making the LiDAR ICP viable (the key fix)

The pipeline derives effective steering from LiDAR forward velocity
(`κ = yaw_rate / vx`, `δ_eq = arctan(L·κ)`). As recorded it failed: `fit_static_map.py`
rejected **0/49** segments. Cause = **observability, not bad data**. The estimator
matched *consecutive* scans; at 0.6 m/s and 40 Hz that is ~1.5 cm of forward motion per
pair, below LiDAR range noise. Per-frame `vx` therefore swung −5…+7 m/s within one
steady run and was biased ~50% low.

**Fix: match each scan against an *earlier* scan chosen so the displacement is a fixed
target (~0.12 m)** instead of the previous frame. Forward motion then sits well above
the noise. Measured on a real steady window:

| matching baseline | forward displacement | vx median | scatter (MAD) |
|---|---|---|---|
| consecutive (0.025 s) | 1.5 cm | 0.294 (garbage) | 0.101 |
| ~0.125 s | 7 cm | 0.594 | 0.058 |
| ~0.25 s | 15 cm | 0.583 | 0.023 |

Implemented in `estimate_lidar_motion_baseline.py` (displacement-targeted baseline,
speed-robust at 0.6–2.1 m/s; reuses the existing point-to-line ICP, deskew, warm-start
and per-pair gates). Re-ran stages 3/4/5/6 → `lidar_velocity_baseline.parquet`. LiDAR
forward velocity is now **observable and accurate**, so it is the primary speed source
below; wheel odometry is kept as an independent cross-check.

Reproduce: `estimate_lidar_motion_baseline.py` then `fit_lidar_steering.py`.
Outputs: `lidar_static_map.json`, `lidar_static_map_conditions.csv`,
`lidar_steering_rate.json`, `lidar_response_steps.csv` (and the `_odom_imu_*`
files are the earlier odometry-only version, kept for comparison).

---

## 1. Steering tuning — static map (raw servo → effective steering angle)

LiDAR speed + IMU yaw. **Centre servo (zero steady yaw): 0.532.** Full coverage
(16/16 conditions, 43 training + 18 hold-out points). Hold-out RMSE **0.32°**,
repeatability 0.14°, hysteresis 0.16° — all far inside any usable tolerance.

| side | fraction | raw servo echo | δ_eq LiDAR (deg) | δ_eq odom (deg) |
|---|---:|---:|---:|---:|
| low_raw | 0.87 | 0.170 | +23.84 | +21.67 |
| low_raw | 0.72 | 0.233 | +19.93 | +18.51 |
| low_raw | 0.57 | 0.295 | +16.68 | +15.51 |
| low_raw | 0.42 | 0.358 | +12.58 | +11.61 |
| low_raw | 0.27 | 0.420 | +7.74 | +7.31 |
| low_raw | 0.15 | 0.470 | +4.43 | +4.19 |
| low_raw | 0.08 | 0.499 | +2.28 | +2.15 |
| low_raw | 0.04 | 0.516 | +0.90 | +0.85 |
| **centre** | — | **0.532** | **0.00** | **0.00** |
| high_raw | 0.04 | 0.549 | −0.38 | −0.37 |
| high_raw | 0.08 | 0.565 | −1.86 | −1.75 |
| high_raw | 0.15 | 0.593 | −3.44 | −3.26 |
| high_raw | 0.27 | 0.641 | −6.67 | −6.32 |
| high_raw | 0.42 | 0.701 | −11.12 | −10.38 |
| high_raw | 0.57 | 0.761 | −14.38 | −13.42 |
| high_raw | 0.72 | 0.822 | −17.57 | −16.43 |
| high_raw | 0.87 | 0.882 | −21.18 | −19.39 |

`low_raw` steers left (+δ), `high_raw` steers right (−δ). Usable envelope **+23.8° … −21.2°**
(servo 0.170 … 0.882). Approx global slope −1.13 rad/servo (−64.6°/servo); the curve is
mildly nonlinear, so prefer the table / `local_gain_rad_per_servo` for interpolation.

### LiDAR ↔ odometry cross-check (a real, deployable finding)
LiDAR and odometry agree on map **shape and centre**, differing only by a speed scale:
LiDAR ground speed is a **consistent ~7% below** odometry at every speed
(ratio 0.924 @ 0.6, 0.931 @ 1.5, 0.949 @ 2.1 m/s). That consistency means the VESC
**odometry over-reads speed by ~7%** (wheel-radius / ERPM-gain), not ICP noise. The two
maps differ by the same ~7% in angle. **Which to deploy:** if your MPC consumes VESC/odom
speed, use the odom-scaled map for internal consistency; if it uses true ground speed,
use the LiDAR map. Either way, consider correcting the odometry speed scale by ~0.93.

---

## 2. Steering rate (effective, vehicle-level)

Peak `|dδ_eq/dt|` over the first 1.5 s of each accepted step (28 steps; IMU yaw at 200 Hz,
LiDAR per-step speed, odom fallback). The peak lands mid-rise ~0.10 s after onset — a real
slew, not noise. Result is essentially identical to the odom-based estimate (rate is
yaw-dominated, so the 7% speed scale barely matters).

| metric | value |
|---|---|
| **Peak effective steering rate (median)** | **2.66 rad/s ≈ 152 °/s** |
| Peak effective steering rate (max observed) | 3.55 rad/s ≈ 203 °/s |
| Effective dead time (10%) | ~0.078 s |
| Rise 10–90% | ~0.077 s |

By speed (median peak rate): 0.6 m/s → 165 °/s (n=12); 1.5 m/s → 149 °/s (n=12);
2.1 m/s → 44 °/s (n=4*).

\* 2.1 m/s is the incomplete condition (4 accepted steps) and 0.9 m/s was never captured —
these are the trials that could not be completed; treat 2.1 m/s as indicative only.

**For MPC:** nominal effective steering-rate capability ~2.6 rad/s (150 °/s) with ~0.08 s
effective dead time; for a hard rate limit the data supports up to ~3.5 rad/s (200 °/s) at
low speed.

---

## Caveats
- These are **effective vehicle-level** dynamics (command + servo + linkage + tyre + yaw),
  the quantity the MPC plant sees — not an isolated servo-shaft angle/rate.
  `/sensors/servo_position_command` is a command echo, not a measured shaft angle.
- The LiDAR fix recovers forward *velocity*; per-pair *fraction* valid is lower than the
  consecutive-scan version (longer baseline ⇒ fewer correspondences), but each capture
  window still yields tens of valid pairs, so window-median speed is robust. Gating is on
  absolute valid-pair count, not fraction.
