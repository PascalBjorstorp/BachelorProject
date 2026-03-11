#!/usr/bin/env python3
"""
Offline re-analysis of vehicle parameter test data.

Reads existing CSV files from the data/ directory and applies
improved analysis methods (understeer gradient, EMA smoothing)
without needing to run on hardware.

Usage:
    python3 reanalyze.py
"""

import os
import sys
import glob
import numpy as np
import pandas as pd

DATA_DIR = os.path.join(os.path.dirname(__file__), 'data')
PARAMS_FILE = os.path.join(os.path.dirname(__file__), 'vehicle_params.yaml')

# Vehicle constants
MASS = 3.314          # kg
L_F  = 0.166          # m (front axle to CoG)
L_R  = 0.16           # m (rear axle to CoG)
L    = L_F + L_R      # 0.326 m wheelbase
EMA_ALPHA = 0.3       # LiDAR velocity smoothing factor
MIN_SPEED_CX = 2.0    # m/s — ignore low-speed longit. data (LiDAR SNR)
GRAVITY = 9.81


# ============================================================
# FRICTION (re-analysis using steady-state plateau)
# ============================================================

def reanalyze_friction():
    """Re-analyse friction coefficient using plateau detection (not raw peak)."""
    print("\n" + "=" * 60)
    print("FRICTION — OFFLINE RE-ANALYSIS")
    print("Method: Steady-state plateau detection")
    print("=" * 60)

    raw_files = sorted(glob.glob(os.path.join(DATA_DIR, 'friction_test_*.csv')))
    if not raw_files:
        print("  ERROR: No friction_test_*.csv found in data/")
        return None

    print(f"\n  Found {len(raw_files)} run files")

    run_mus = []
    for fpath in raw_files:
        df = pd.read_csv(fpath)
        # Extract speed setpoints from phase column (e.g. 'friction_v2.0')
        speeds = sorted(df['speed_setpoint'].unique())

        speed_ay = []
        for spd in speeds:
            seg = df[df['speed_setpoint'] == spd]
            ay_vals = np.abs(seg['imu_ay'].values)
            if len(ay_vals) > 10:
                # Use median of last 50% (steady-state) to reject transient
                n = len(ay_vals)
                steady = ay_vals[n // 2:]
                speed_ay.append((spd, np.median(steady)))

        if len(speed_ay) < 3:
            continue

        # Plateau detection: find where ay stops increasing significantly
        speed_ay.sort()
        plateau_N = 3
        plateau_threshold = 0.05  # 5% relative increase
        plateau_vals = []
        for i in range(len(speed_ay) - 1, 0, -1):
            _, ay_curr = speed_ay[i]
            _, ay_prev = speed_ay[i - 1]
            rel_increase = (ay_curr - ay_prev) / ay_prev if ay_prev > 0 else 1.0
            if rel_increase < plateau_threshold:
                plateau_vals.append(ay_curr)
                if len(plateau_vals) >= plateau_N:
                    break
            else:
                break

        if plateau_vals:
            max_ay = np.mean(plateau_vals)
        else:
            # No plateau found — use highest speed point
            max_ay = speed_ay[-1][1]

        mu = max_ay / GRAVITY
        run_mus.append(mu)
        print(f"  {os.path.basename(fpath)}: μ = {mu:.4f} (plateau ay = {max_ay:.2f} m/s²)")

    if run_mus:
        mu_mean = np.mean(run_mus)
        mu_std = np.std(run_mus)
        print(f"\n  *** BEST ESTIMATE ***")
        print(f"    μ = {mu_mean:.3f} ± {mu_std:.3f}  (n={len(run_mus)} runs)")
        return mu_mean
    return None


# ============================================================
# MOTOR TORQUE (re-analysis)
# ============================================================

def reanalyze_motor_torque():
    """Re-analyse motor torque data (Kt_eff from F vs I regression)."""
    print("\n" + "=" * 60)
    print("MOTOR TORQUE — OFFLINE RE-ANALYSIS")
    print("Filters: 0 < I_motor < 60 A, ax > 0.5 (active acceleration)")
    print("=" * 60)

    raw_files = sorted(glob.glob(os.path.join(DATA_DIR, 'motor_torque_*.csv')))
    if not raw_files:
        print("  ERROR: No motor_torque_*.csv found in data/")
        return None, None

    print(f"\n  Found {len(raw_files)} run files")

    all_I, all_F = [], []
    for fpath in raw_files:
        df = pd.read_csv(fpath)
        accel = df[df['phase'] == 'acceleration']
        ax = accel['imu_ax'].values
        I = accel['motor_current'].values
        mask = (I > 0) & (I < 60) & (ax > 0.5)
        if mask.sum() > 10:
            all_I.extend(I[mask].tolist())
            all_F.extend((MASS * ax[mask]).tolist())
            print(f"  {os.path.basename(fpath)}: {mask.sum()} pts")

    if len(all_I) < 20:
        print("  ERROR: Not enough valid data points")
        return None, None

    I_arr = np.array(all_I)
    F_arr = np.array(all_F)

    # Linear regression: F = Kt_eff * I + F_loss
    p = np.polyfit(I_arr, F_arr, 1)
    Kt_eff = p[0]
    F_loss = p[1]
    pred = Kt_eff * I_arr + F_loss
    ss_res = np.sum((F_arr - pred) ** 2)
    ss_tot = np.sum((F_arr - np.mean(F_arr)) ** 2)
    r2 = 1 - ss_res / ss_tot

    print(f"\n  Linear fit (N={len(I_arr)}):")
    print(f"    F = {Kt_eff:.4f} × I + ({F_loss:.2f})")
    print(f"    Kt_eff = {Kt_eff:.4f} N/A")
    print(f"    Rolling resistance from fit = {abs(F_loss):.2f} N")
    print(f"    R² = {r2:.4f}")
    print(f"\n  *** BEST ESTIMATE ***")
    print(f"    Kt_eff = {Kt_eff:.3f} N/A")
    print(f"    Rolling resistance = {abs(F_loss):.2f} N")

    return Kt_eff, abs(F_loss)


# ============================================================
# CORNERING STIFFNESS (re-analysis)
# ============================================================

def reanalyze_cornering():
    """Re-analyse cornering stiffness using understeer-gradient method."""
    print("\n" + "=" * 60)
    print("CORNERING STIFFNESS — OFFLINE RE-ANALYSIS")
    print("Method: Understeer-Gradient  (δ/ω = L/v + K_us·v)")
    print("=" * 60)

    summary_files = sorted(glob.glob(os.path.join(DATA_DIR, 'cornering_stiffness_*_summary.csv')))
    if not summary_files:
        print("  ERROR: No cornering_stiffness_*_summary.csv found in data/")
        return None

    # Use the most recent summary
    sfile = summary_files[-1]
    print(f"\n  Reading: {os.path.basename(sfile)}")
    df = pd.read_csv(sfile)
    print(f"  Data points: {len(df)}")
    print(f"  Columns: {list(df.columns)}")

    # ----------------------------------------------------------
    # 1. Filter quality: omega > 0.3 rad/s, speed > 1.0 m/s
    # ----------------------------------------------------------
    ok = (df['omega'] > 0.3) & (df['speed'] > 1.0)
    df = df[ok].copy()
    print(f"\n  After quality filter (omega>0.3, v>1.0): {len(df)} points")

    # ----------------------------------------------------------
    # 2. Understeer-Gradient fit:  δ/ω = L/v + K_us·v
    #    Rearranging:  δ/ω - L/v = K_us·v
    #    → linear regression of Y = K_us * X  (origin fit)
    #    where Y = δ/ω − L/v  and  X = v
    # ----------------------------------------------------------
    delta = df['steering_angle_rad'].values
    omega = df['omega'].values
    v     = df['speed'].values
    ay    = df['ay'].values  # centripetal acceleration

    ratio = delta / omega          # L/v + K_us*v
    Y = ratio - L / v              # K_us * v
    X = v

    # Weighted least-squares (weight by speed — higher speed is more reliable)
    K_us = float(np.sum(Y * X * v**2) / np.sum(X**2 * v**2))
    # Simple unweighted for comparison
    K_us_simple = float(np.dot(Y, X) / np.dot(X, X))

    residuals = Y - K_us * X
    ss_res = np.sum(residuals**2)
    ss_tot = np.sum((Y - np.mean(Y))**2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0

    print(f"\n  Understeer Gradient Fit:")
    print(f"    K_us (speed-weighted) = {K_us:.6f} rad/(m/s²)")
    print(f"    K_us (unweighted)     = {K_us_simple:.6f} rad/(m/s²)")
    print(f"    R² = {r2:.4f}")
    print(f"    Residual std = {np.std(residuals):.5f} rad·s/m")

    # ----------------------------------------------------------
    # 3. Extract C_alpha from K_us
    #    K_us = m*(l_f/(L·C_r) - l_r/(L·C_f))
    #    Assume C_f = C_r = C_alpha (single-track symmetric):
    #    K_us ≈ m*(l_f - l_r)/(L·C_alpha)
    # ----------------------------------------------------------
    num   = MASS * (L_F - L_R) / L
    if abs(K_us) > 1e-6:
        C_alpha_kus = num / K_us
        print(f"\n  Symmetric-axle assumption (C_f≈C_r):")
        print(f"    C_alpha ≈ {C_alpha_kus:.1f} N/rad  (per axle)")
    else:
        C_alpha_kus = None
        print("\n  K_us ≈ 0 → car is nearly neutral; cannot resolve C_alpha from K_us alone.")

    # ----------------------------------------------------------
    # 4. Per-point naive estimates for diagnostics only
    #    WARNING: C_alpha_f_naive = m*v²*l_r/(L*l_f) — this is a
    #    kinematic identity, NOT real tire stiffness. It scales with v²
    #    and is shown here only to confirm the degeneracy is present.
    # ----------------------------------------------------------
    hi = df['speed'] > 2.0
    if 'C_alpha_f_naive' in df.columns and hi.sum() > 0:
        cf_naive = df.loc[hi, 'C_alpha_f_naive'].values
        cr_naive = df.loc[hi, 'C_alpha_r_naive'].values
        cf_pos = cf_naive[cf_naive > 0]
        cr_pos = cr_naive[cr_naive > 0]
        print(f"\n  Per-point naive values (KINEMATIC IDENTITY, v>2 m/s, n={hi.sum()}):")
        print(f"  ⚠  These are geometry artifacts (C ~ m*v²/L), not tire stiffness.")
        if len(cf_pos):
            print(f"    C_alpha_f_naive: median={np.median(cf_pos):.1f}  "
                  f"mean={np.mean(cf_pos):.1f}  std={np.std(cf_pos):.1f}  N/rad")
        if len(cr_pos):
            print(f"    C_alpha_r_naive: median={np.median(cr_pos):.1f}  "
                  f"mean={np.mean(cr_pos):.1f}  std={np.std(cr_pos):.1f}  N/rad")

    # ----------------------------------------------------------
    # 5. Best estimate
    #    Absolute C_alpha requires a second equation (yaw-resonance
    #    from step-steer transients). Without that data here, we use
    #    the report's combined K_us + omega_n result.
    # ----------------------------------------------------------
    C_alpha_best_f = 44.8   # N/rad — combined K_us + yaw-resonance (report_outline.tex)
    C_alpha_best_r = 55.7   # N/rad — ±30-40% uncertainty
    source = "combined K_us + yaw-resonance (report_outline.tex)"

    print(f"\n  *** BEST ESTIMATE [{source}] ***")
    print(f"    C_alpha_f = {C_alpha_best_f:.1f} N/rad" if C_alpha_best_f else "    C_alpha_f = N/A")
    print(f"    C_alpha_r = {C_alpha_best_r:.1f} N/rad" if C_alpha_best_r else "    C_alpha_r = N/A")

    return C_alpha_best_f, C_alpha_best_r


# ============================================================
# LONGITUDINAL STIFFNESS (re-analysis)
# ============================================================

def ema(arr, alpha):
    """Exponential moving average."""
    out = np.empty_like(arr, dtype=float)
    out[0] = arr[0]
    for i in range(1, len(arr)):
        out[i] = alpha * arr[i] + (1 - alpha) * out[i - 1]
    return out


def reanalyze_longitudinal():
    """Re-analyse longitudinal stiffness using EMA-smoothed LiDAR velocity."""
    print("\n" + "=" * 60)
    print("LONGITUDINAL STIFFNESS — OFFLINE RE-ANALYSIS")
    print(f"EMA alpha = {EMA_ALPHA},  min v for C_x = {MIN_SPEED_CX} m/s")
    print("=" * 60)

    raw_files = sorted(glob.glob(os.path.join(DATA_DIR, 'longitudinal_stiffness_*.csv')))
    # exclude any summary files
    raw_files = [f for f in raw_files if 'summary' not in f]
    if not raw_files:
        print("  ERROR: No longitudinal_stiffness_*.csv found in data/")
        return None

    print(f"\n  Found {len(raw_files)} run files")

    all_kappa = []
    all_Fx    = []
    accel_kappa, accel_Fx = [], []
    brake_kappa, brake_Fx = [], []

    for fpath in raw_files:
        df = pd.read_csv(fpath)
        if 'v_lidar' not in df.columns:
            print(f"  SKIP {os.path.basename(fpath)} — no v_lidar column")
            continue

        # Apply EMA smoothing to LiDAR velocity per phase
        for phase in ['acceleration', 'cruise', 'braking']:
            mask = df['phase'] == phase
            if mask.sum() < 5:
                continue

            seg = df[mask].copy()
            seg_vlidar_ema = ema(seg['v_lidar'].values, EMA_ALPHA)
            seg_vwheel = seg['odom_vx'].values
            seg_ax     = seg['imu_ax'].values

            v_max_arr  = np.maximum(np.maximum(np.abs(seg_vwheel), np.abs(seg_vlidar_ema)), 0.5)
            kappa_arr  = (seg_vwheel - seg_vlidar_ema) / v_max_arr
            F_x_arr    = MASS * seg_ax

            hi_speed = np.abs(seg_vlidar_ema) > MIN_SPEED_CX
            lin      = hi_speed & (np.abs(kappa_arr) > 0.005) & (np.abs(kappa_arr) < 0.20)

            if np.sum(lin) > 0:
                all_kappa.extend(kappa_arr[lin].tolist())
                all_Fx.extend(F_x_arr[lin].tolist())
                if phase == 'acceleration':
                    accel_kappa.extend(kappa_arr[lin].tolist())
                    accel_Fx.extend(F_x_arr[lin].tolist())
                elif phase == 'braking':
                    brake_kappa.extend(kappa_arr[lin].tolist())
                    brake_Fx.extend(F_x_arr[lin].tolist())

    def fit_cx(kappa_list, Fx_list, label):
        if len(kappa_list) < 10:
            print(f"\n  {label}: only {len(kappa_list)} points — not enough for fit")
            return 0.0
        kk = np.array(kappa_list)
        ff = np.array(Fx_list)
        Cx = float(np.sum(ff * kk) / np.sum(kk ** 2))
        ss_res = np.sum((ff - Cx * kk)**2)
        ss_tot = np.sum((ff - np.mean(ff))**2)
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        p = np.polyfit(kk, ff, 1)
        print(f"\n  {label} ({len(kappa_list)} pts):")
        print(f"    C_x (origin-fit) = {abs(Cx):.1f} N/slip,  R² = {r2:.3f}")
        print(f"    C_x (polyfit)    = slope {p[0]:.1f}, intercept {p[1]:.1f} N")
        return Cx

    Cx_accel   = fit_cx(accel_kappa, accel_Fx,   "Acceleration phase")
    Cx_brake   = fit_cx(brake_kappa, brake_Fx,   "Braking phase")
    Cx_combined = fit_cx(all_kappa,   all_Fx,    "Combined (all phases)")

    best_Cx = Cx_accel if accel_kappa else (Cx_brake if brake_kappa else Cx_combined)
    # NOTE: Use acceleration-only estimate (braking phase inflated by regen)

    print(f"\n  *** BEST ESTIMATE ***")
    print(f"    C_x = {abs(best_Cx):.1f} N/unit-slip  (combined, EMA-smoothed LiDAR)")
    print(f"    C_x per tire ≈ {abs(best_Cx)/4:.1f} N/unit-slip")

    return best_Cx


# ============================================================
# UPDATE vehicle_params.yaml
# ============================================================

def update_yaml(key, value, comment=''):
    """Simple in-place update of a key in vehicle_params.yaml."""
    with open(PARAMS_FILE, 'r') as f:
        lines = f.readlines()

    pattern = f'  {key}:'
    updated = False
    for i, line in enumerate(lines):
        if line.startswith(pattern) or line.startswith(f'  {key} '):
            # replace the value part
            parts = line.split(':', 1)
            old_rest = parts[1]
            # keep existing comment or use new one
            if '#' in old_rest:
                comment_part = '  # ' + old_rest.split('#', 1)[1].strip()
            else:
                comment_part = f'  # {comment}' if comment else ''
            lines[i] = f'  {key}: {value:.2f}{comment_part}\n'
            updated = True
            break

    if updated:
        with open(PARAMS_FILE, 'w') as f:
            f.writelines(lines)
        print(f"  → Updated {key} = {value:.2f} in vehicle_params.yaml")
    else:
        print(f"  WARNING: key '{key}' not found in vehicle_params.yaml")


# ============================================================
# MAIN
# ============================================================

if __name__ == '__main__':
    print("F1/10th Vehicle Parameter Offline Re-Analysis")
    print("Using improved methods: plateau detection, understeer-gradient, EMA smoothing")

    mu = reanalyze_friction()
    Kt_eff, F_rolling = reanalyze_motor_torque() or (None, None)
    C_af, C_ar = reanalyze_cornering() or (None, None)
    Cx = reanalyze_longitudinal()

    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    if mu:   print(f"  μ           = {mu:.3f}")
    if Kt_eff: print(f"  Kt_eff      = {Kt_eff:.3f} N/A")
    if F_rolling: print(f"  F_rolling   = {F_rolling:.2f} N")
    if C_af: print(f"  C_alpha_f   = {C_af:.1f} N/rad")
    if C_ar: print(f"  C_alpha_r   = {C_ar:.1f} N/rad")
    if Cx:   print(f"  C_x         = {abs(Cx):.1f} N/unit-slip")

    # Auto-update vehicle_params.yaml
    print("\n  Updating vehicle_params.yaml...")
    if mu:   update_yaml('mu', mu, '[TESTED] friction coefficient')
    if Kt_eff: update_yaml('Kt_eff', Kt_eff, '[TESTED] effective motor-current-to-wheel-force (N/A)')
    if F_rolling: update_yaml('rolling_resistance', F_rolling, '[TESTED] rolling resistance force (N)')
    if C_af: update_yaml('C_alpha_f', C_af, '[TESTED] cornering stiffness front (N/rad)')
    if C_ar: update_yaml('C_alpha_r', C_ar, '[TESTED] cornering stiffness rear (N/rad)')
    if Cx:   update_yaml('C_x', abs(Cx), '[TESTED] combined longitudinal stiffness (N/slip)')

    print("\nDone.")
