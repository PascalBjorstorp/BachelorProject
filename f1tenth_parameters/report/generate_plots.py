#!/usr/bin/env python3
"""
Generate all report figures from test CSV data.

Usage:
    python3 generate_plots.py                      # auto-detect latest CSVs
    python3 generate_plots.py --data-dir data/     # explicit data directory
    python3 generate_plots.py --prefix 20260301    # filter by date prefix
    python3 generate_plots.py --wn-best 10         # explicit omega_n pick (rad/s)
    python3 generate_plots.py --wn-best auto       # estimate omega_n from step-response

Produces PDF figures in figures/ matching the report_outline.tex placeholders:
  1. friction_ay_vs_speed.pdf
  2. cornering_understeer_gradient.pdf
  3. cornering_solution_range.pdf
  4. rolling_resistance_coastdown.pdf
  5. torque_I_vs_F.pdf
  6. torque_current_vs_time.pdf
  7. max_dyn_speed_vs_time.pdf
  8. steering_step_response.pdf
"""

import argparse
import glob
import os
import sys

import matplotlib
matplotlib.use('Agg')  # non-interactive backend
import matplotlib.pyplot as plt
import numpy as np

# ── Helpers ──────────────────────────────────────────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(SCRIPT_DIR)
DATA_DIR = os.path.join(ROOT_DIR, 'data')
OLD_DATA_DIR = os.path.join(ROOT_DIR, 'oldData')
FIG_DIR = os.path.join(ROOT_DIR, 'figures')

GRAVITY = 9.81


def load_csv(path):
    """Load a CSV into a dict of numpy arrays (numeric) or string arrays."""
    import csv
    with open(path, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return {}
    data = {}
    for key in rows[0].keys():
        try:
            data[key] = np.array([float(r[key]) for r in rows])
        except (ValueError, TypeError):
            data[key] = np.array([r[key] for r in rows])
    return data


def find_csvs(pattern, data_dir, prefix=None):
    """Find all CSVs matching *pattern* in data_dir."""
    candidates = sorted(glob.glob(os.path.join(data_dir, f'*{pattern}*.csv')))
    if prefix:
        candidates = [c for c in candidates if prefix in os.path.basename(c)]
    candidates = [c for c in candidates if '_summary' not in c]
    return candidates


def find_latest(pattern, data_dir, prefix=None):
    """Find the most recent CSV matching *pattern* in data_dir."""
    candidates = find_csvs(pattern, data_dir, prefix)
    if not candidates:
        return None
    return candidates[-1]


def setup_fig(width=6, height=4):
    fig, ax = plt.subplots(figsize=(width, height))
    ax.grid(True, alpha=0.3)
    return fig, ax


def save_fig(fig, name):
    os.makedirs(FIG_DIR, exist_ok=True)
    path = os.path.join(FIG_DIR, name)
    fig.savefig(path, bbox_inches='tight', dpi=150)
    plt.close(fig)
    print(f'  -> {path}')


def _moving_average(x, window):
    """Simple moving average used for noise-robust transient metrics."""
    if window <= 1:
        return x
    kernel = np.ones(window, dtype=float) / float(window)
    return np.convolve(x, kernel, mode='same')


def _ordered_unique(values):
    """Return first-seen unique values while preserving order."""
    seen = set()
    out = []
    for v in values:
        if v not in seen:
            seen.add(v)
            out.append(v)
    return out


def estimate_wn_from_steering_rate(data_dir, prefix):
    """Estimate a yaw natural-frequency proxy from steering step-response runs.

    For each step segment, the 10-90% rise time of the yaw-rate response is
    measured and mapped to omega_n with:
        omega_n ≈ 1.8 / t_r
    (standard second-order approximation for zeta ≈ 0.7).

    Returns:
      dict with keys: wn, sample_count, q25, q75
      or None if no reliable estimate can be extracted.
    """
    paths = find_csvs('steering_rate', data_dir, prefix)
    if not paths:
        return None

    wn_samples = []

    for p in paths:
        d = load_csv(p)
        needed = {'timestamp_s', 'imu_gz', 'phase'}
        if not needed.issubset(set(d.keys())):
            continue

        t = d['timestamp_s'].astype(float)
        omega = d['imu_gz'].astype(float)
        phase = d['phase']

        if len(t) < 50:
            continue

        dt = float(np.median(np.diff(t)))
        if not np.isfinite(dt) or dt <= 0:
            continue

        # Light smoothing (~10 ms) to stabilize threshold crossings.
        smooth_n = max(3, int(round(0.01 / dt)))
        if smooth_n % 2 == 0:
            smooth_n += 1
        omega_f = _moving_average(omega, smooth_n)

        for label in _ordered_unique(phase):
            mask = phase == label
            if np.sum(mask) < 200:
                continue

            ts = t[mask] - t[mask][0]
            ys = omega_f[mask]
            n = len(ys)

            # Initial and steady-state levels from segment head/tail windows.
            n0 = max(8, int(0.03 * n))
            nss = max(20, int(0.25 * n))
            y0 = float(np.median(ys[:n0]))
            yss = float(np.median(ys[-nss:]))
            dy = yss - y0

            # Skip too-small responses (poor signal-to-noise for rise-time).
            if abs(dy) < 0.25:
                continue

            y10 = y0 + 0.1 * dy
            y90 = y0 + 0.9 * dy

            if dy > 0:
                idx10 = np.where(ys >= y10)[0]
                idx90 = np.where(ys >= y90)[0]
            else:
                idx10 = np.where(ys <= y10)[0]
                idx90 = np.where(ys <= y90)[0]

            if len(idx10) == 0 or len(idx90) == 0:
                continue

            tr = float(ts[idx90[0]] - ts[idx10[0]])
            if tr <= 0.01 or tr > 1.0:
                continue

            wn = 1.8 / tr
            if 2.0 <= wn <= 60.0:
                wn_samples.append(wn)

    if len(wn_samples) < 3:
        return None

    arr = np.array(wn_samples, dtype=float)
    return {
        'wn': float(np.median(arr)),
        'sample_count': int(len(arr)),
        'q25': float(np.percentile(arr, 25)),
        'q75': float(np.percentile(arr, 75)),
    }


# ── Vehicle parameters (for computing derived quantities from raw data) ─────

DEFAULT_MASS = 3.314       # kg
DEFAULT_L = 0.324          # m  wheelbase
DEFAULT_LF = 0.169         # m  CG to front axle
DEFAULT_LR = 0.155         # m  CG to rear axle
DEFAULT_IZ = 0.035         # kg*m^2  yaw inertia

# Servo nonlinearity correction for cornering stiffness tests.
# These tests were run with the OLD servo gain (-0.7940). The physical
# servo mapping (actual wheel angle vs. servo value) is:
#   actual = -0.9262 * s^2 - 0.4778 * s + 0.5365
# where the VESC command path is:
#   s = gain * cmd + offset
# (ackermann_to_vesc applies gain directly to the signed steering command).
# With gain=-0.7940, offset=0.55, the actual angle at each commanded angle
# magnitude is computed by _servo_actual_angle() below.
_SERVO_OLD_GAIN   = -0.7940
_SERVO_OFFSET     = 0.55
_SERVO_POLY       = np.array([-0.9262, -0.4778, 0.5365])  # actual = poly(s)
_SERVO_ZERO_ANGLE = np.polyval(_SERVO_POLY, _SERVO_OFFSET)  # actual at cmd=0

def _servo_actual_angle(cmd_abs):
    """Return actual wheel angle for a positive commanded angle using old gain."""
    s = _SERVO_OLD_GAIN * cmd_abs + _SERVO_OFFSET
    return np.polyval(_SERVO_POLY, s) - _SERVO_ZERO_ANGLE


def compute_cornering_summary(data_dir, prefix):
    """Compute per-(speed, steering) cornering stiffness summary from raw CSVs.

    Returns a dict matching the expected summary column layout:
      speed, alpha_f_deg, alpha_r_deg, F_yf, F_yr
    or None if no data is available.
    """
    paths = find_csvs('cornering_stiffness', data_dir, prefix)
    if not paths:
        return None

    # Aggregate all raw runs (record phase only)
    all_rows = []
    for p in paths:
        d = load_csv(p)
        if 'phase' not in d:
            continue
        mask = d['phase'] == 'record'
        if np.sum(mask) < 20:
            continue
        all_rows.append({k: v[mask] for k, v in d.items()})

    if not all_rows:
        return None

    # Merge
    merged = {}
    for key in all_rows[0]:
        merged[key] = np.concatenate([r[key] for r in all_rows])

    vx = merged['odom_vx']
    ay = merged['imu_ay']
    gz = merged['imu_gz']
    steer = merged['cmd_steering']
    cmd_speed = merged['cmd_speed']

    # Group by (cmd_speed, cmd_steering) pair
    pairs = sorted(set(zip(cmd_speed, np.abs(steer))))
    out = {'speed': [], 'cmd_steering': [], 'vx_mean': [],
           'ay_mean': [], 'gz_mean': [], 'delta': [],
           'alpha_f_deg': [], 'alpha_r_deg': [],
           'F_yf': [], 'F_yr': []}

    for sp, st in pairs:
        sm = (cmd_speed == sp) & (np.abs(steer) == st)
        v = np.mean(np.abs(vx[sm]))
        if v < 0.3:
            continue
        omega = np.mean(gz[sm])
        delta = _servo_actual_angle(st)  # corrected actual wheel angle (old gain)
        a_y = np.mean(ay[sm])

        # Slip angles (bicycle model, v_y ≈ 0 assumption)
        alpha_f = delta - np.arctan2(DEFAULT_LF * omega, v)
        alpha_r = np.arctan2(DEFAULT_LR * omega, v)

        # Lateral forces (steady-state moment balance)
        F_yf = DEFAULT_MASS * a_y * DEFAULT_LR / DEFAULT_L
        F_yr = DEFAULT_MASS * a_y * DEFAULT_LF / DEFAULT_L

        out['speed'].append(sp)
        out['cmd_steering'].append(st)
        out['vx_mean'].append(v)
        out['ay_mean'].append(a_y)
        out['gz_mean'].append(omega)
        out['delta'].append(delta)
        out['alpha_f_deg'].append(np.degrees(alpha_f))
        out['alpha_r_deg'].append(np.degrees(alpha_r))
        out['F_yf'].append(F_yf)
        out['F_yr'].append(F_yr)

    if not out['speed']:
        return None

    return {k: np.array(v) for k, v in out.items()}


# ── Plot functions ───────────────────────────────────────────────────────────

def plot_friction(data_dir, prefix):
    """Friction coefficient: a_y vs speed using all available friction runs."""
    paths = find_csvs('friction_test', data_dir, prefix)
    if not paths:
        print('  [skip] no friction_test CSV found')
        return

    # Compute per-run per-speed steady-state ay means, then aggregate across runs.
    speed_to_run_ay = {}
    for p in paths:
        d = load_csv(p)
        if 'speed_setpoint' not in d or 'imu_ay' not in d or 'imu_gz' not in d or 'odom_vx' not in d:
            continue

        run_speeds = sorted(set(d['speed_setpoint']))
        for sp in run_speeds:
            mask = d['speed_setpoint'] == sp
            if np.sum(mask) < 10:
                continue

            ay_vals = np.abs(d['imu_ay'][mask])
            vx_vals = d['odom_vx'][mask]
            gz_vals = d['imu_gz'][mask]
            consistency = np.abs(np.abs(ay_vals) - np.abs(vx_vals * gz_vals))

            steady_mask = consistency < 0.8
            if np.sum(steady_mask) >= 20:
                ay_used = ay_vals[steady_mask]
            else:
                ay_used = ay_vals

            speed_to_run_ay.setdefault(float(sp), []).append(float(np.mean(ay_used)))

    if not speed_to_run_ay:
        print('  [skip] no valid friction phases')
        return

    speeds = np.array(sorted(speed_to_run_ay.keys()))
    ay_means = np.array([np.mean(speed_to_run_ay[s]) for s in speeds])
    ay_stds = np.array([np.std(speed_to_run_ay[s]) for s in speeds])

    # Data-driven limit: use highest-speed aggregate point (5.0 m/s in current set).
    idx_peak = int(np.argmax(speeds))
    ay_limit = float(ay_means[idx_peak])
    mu_limit = ay_limit / GRAVITY

    fig, ax = setup_fig()
    ax.errorbar(speeds, ay_means, yerr=ay_stds, fmt='o-',
                capsize=4, color='tab:blue', label=r'$\bar{a}_y$ (IMU)')

    ax.axhline(ay_limit, ls='--', color='tab:red', alpha=0.8, linewidth=2,
               label=rf'Data peak (all runs): $\mu = {mu_limit:.3f}$, '
                     rf'$\mu g = {ay_limit:.2f}$ m/s$^2$')

    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel(r'Lateral acceleration $a_y$ (m/s$^2$)')
    ax.set_title(f'Friction Coefficient Identification ({len(paths)} runs)')
    ax.legend(loc='upper left')
    save_fig(fig, 'friction_ay_vs_speed.pdf')


def plot_cornering_understeer_gradient(data_dir, prefix):
    """Understeer gradient: (delta - L*omega/v) vs a_y with linear fit."""
    summary = compute_cornering_summary(data_dir, prefix)
    if not summary:
        print('  [skip] no cornering_stiffness data found')
        return

    delta = summary['delta']         # commanded steering angle (rad)
    vx = summary['vx_mean']          # mean measured speed (m/s)
    ay = summary['ay_mean']          # mean lateral acceleration (m/s^2)
    gz = summary['gz_mean']          # mean yaw rate (rad/s)
    speeds = summary['speed']        # commanded speed

    # Understeer deviation: delta - L*omega/v
    delta_kin = DEFAULT_L * gz / vx  # kinematic steering angle
    delta_dev = delta - delta_kin    # understeer deviation

    # Linear regression: delta_dev = K_us * a_y
    from numpy.polynomial import polynomial as P
    coeffs = P.polyfit(ay, delta_dev, 1)  # coeffs[0] = intercept, coeffs[1] = slope
    K_us = coeffs[1]
    pred = coeffs[0] + coeffs[1] * ay
    ss_res = np.sum((delta_dev - pred)**2)
    ss_tot = np.sum((delta_dev - np.mean(delta_dev))**2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0

    fig, ax = setup_fig(width=7, height=5)

    # Colour by speed
    unique_speeds = sorted(set(speeds))
    cmap = plt.cm.viridis(np.linspace(0.2, 0.9, len(unique_speeds)))
    speed_colors = {s: c for s, c in zip(unique_speeds, cmap)}

    for s in unique_speeds:
        mask = speeds == s
        ax.scatter(ay[mask], delta_dev[mask], c=[speed_colors[s]], s=50,
                   edgecolors='k', linewidths=0.5, zorder=5,
                   label=f'v = {s:.1f} m/s')

    # Fit line
    ay_fit = np.linspace(0, np.max(ay) * 1.1, 100)
    ax.plot(ay_fit, coeffs[0] + K_us * ay_fit, 'r-', linewidth=2,
            label=rf'$K_{{us}} = {K_us:.4f}$ rad/(m/s²), $R^2 = {r2:.2f}$')
    ax.axhline(0, color='gray', ls='-', lw=0.5)

    ax.set_xlabel(r'Lateral acceleration $a_y$ (m/s$^2$)')
    ax.set_ylabel(r'$\delta - L\omega/v_x$ (rad)')
    ax.set_title('Understeer Gradient Measurement')
    ax.legend(fontsize=8, loc='upper left')

    n_pts = len(ay)
    n_files = len(find_csvs('cornering_stiffness', data_dir, prefix))
    ax.text(0.98, 0.02, f'n = {n_pts} points, {n_files} runs',
            transform=ax.transAxes, ha='right', va='bottom', fontsize=8,
            bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

    save_fig(fig, 'cornering_understeer_gradient.pdf')
    return K_us, r2


def plot_cornering_solution_range(data_dir, prefix, K_us=None,
                                  wn_range=(8.0, 15.0), wn_best=10.0):
    """Solution range: C_af vs C_ar for measured K_us across omega_n range."""
    if K_us is None:
        summary = compute_cornering_summary(data_dir, prefix)
        if not summary:
            print('  [skip] no cornering_stiffness data found')
            return
        delta = summary['delta']
        vx = summary['vx_mean']
        ay = summary['ay_mean']
        gz = summary['gz_mean']
        delta_dev = delta - DEFAULT_L * gz / vx
        from numpy.polynomial import polynomial as P
        coeffs = P.polyfit(ay, delta_dev, 1)
        K_us = coeffs[1]

    m = DEFAULT_MASS
    L = DEFAULT_L
    lf = DEFAULT_LF
    lr = DEFAULT_LR
    Iz = DEFAULT_IZ

    # Solve K_us and omega_n equations for C_af, C_ar
    # K_us = (m/L)(lf/C_ar - lr/C_af)
    # omega_n^2 = (lf^2 * C_af + lr^2 * C_ar)/Iz + (C_af + C_ar)/m - v^2
    # We parametrize by omega_n and solve the two equations

    v_ref = 2.0  # reference speed for omega_n
    wn_grid = np.linspace(5, 45, 400)
    C_af_vals = []
    C_ar_vals = []
    wn_valid = []

    for wn in wn_grid:
        # From K_us: lf/C_ar - lr/C_af = K_us * L / m
        K_val = K_us * L / m
        # From omega_n: (lf^2*C_af + lr^2*C_ar)/Iz + (C_af + C_ar)/m = wn^2 + v^2
        rhs = wn**2 + v_ref**2

        # Let x = C_af, y = C_ar
        # Eq1: lf/y - lr/x = K_val  =>  lf*x - lr*y = K_val*x*y
        # Eq2: (lf^2/Iz + 1/m)*x + (lr^2/Iz + 1/m)*y = rhs

        # From Eq2: y = (rhs - A*x) / B  where A = lf^2/Iz + 1/m, B = lr^2/Iz + 1/m
        A = lf**2 / Iz + 1.0 / m
        B = lr**2 / Iz + 1.0 / m

        # Substitute into Eq1: lf/((rhs - A*x)/B) - lr/x = K_val
        # => lf*B/(rhs - A*x) - lr/x = K_val
        # => lf*B*x - lr*(rhs - A*x) = K_val * x * (rhs - A*x)
        # => (lf*B + lr*A)*x - lr*rhs = K_val*rhs*x - K_val*A*x^2
        # => K_val*A*x^2 + (lf*B + lr*A - K_val*rhs)*x - lr*rhs = 0

        a_coef = K_val * A
        b_coef = lf * B + lr * A - K_val * rhs
        c_coef = -lr * rhs

        disc = b_coef**2 - 4 * a_coef * c_coef
        if disc < 0 or abs(a_coef) < 1e-12:
            continue

        x1 = (-b_coef + np.sqrt(disc)) / (2 * a_coef)
        x2 = (-b_coef - np.sqrt(disc)) / (2 * a_coef)

        for x in [x1, x2]:
            if x > 0:
                y = (rhs - A * x) / B
                if y > 0:
                    C_af_vals.append(x)
                    C_ar_vals.append(y)
                    wn_valid.append(wn)
                    break

    if not C_af_vals:
        print('  [skip] no valid C_alpha solutions found')
        return

    C_af_vals = np.array(C_af_vals)
    C_ar_vals = np.array(C_ar_vals)
    wn_valid = np.array(wn_valid)

    fig, ax = setup_fig(width=7, height=5)

    # Plot C_af and C_ar vs omega_n
    ax.plot(wn_valid, C_af_vals, '-', color='tab:blue', linewidth=2,
            label=r'$C_{\alpha f}$ (front)')
    ax.plot(wn_valid, C_ar_vals, '-', color='tab:orange', linewidth=2,
            label=r'$C_{\alpha r}$ (rear)')

    # Shade selected frequency range
    wn_lo, wn_hi = sorted((float(wn_range[0]), float(wn_range[1])))
    ax.axvspan(wn_lo, wn_hi, alpha=0.15, color='gray',
               label=rf'$\omega_n \in [{wn_lo}, {wn_hi}]$ rad/s')

    # Best estimate at selected omega_n
    idx_best = np.argmin(np.abs(wn_valid - wn_best))
    wn_best_used = float(wn_valid[idx_best])
    Caf_best = C_af_vals[idx_best]
    Car_best = C_ar_vals[idx_best]
    ax.plot(wn_best_used, Caf_best, 'o', color='tab:blue', markersize=10,
            markeredgecolor='k', zorder=5)
    ax.plot(wn_best_used, Car_best, 'o', color='tab:orange', markersize=10,
            markeredgecolor='k', zorder=5)
    ax.annotate(f'{Caf_best:.1f} N/rad', (wn_best_used, Caf_best),
                textcoords='offset points', xytext=(10, 5), fontsize=9,
                color='tab:blue')
    ax.annotate(f'{Car_best:.1f} N/rad', (wn_best_used, Car_best),
                textcoords='offset points', xytext=(10, -12), fontsize=9,
                color='tab:orange')

    ax.set_xlabel(r'Yaw natural frequency $\omega_n$ (rad/s)')
    ax.set_ylabel(r'Cornering stiffness (N/rad)')
    ax.set_title(rf'Solution Range for $K_{{us}} = {K_us:.4f}$ rad/(m/s²)')
    ax.legend(fontsize=8, loc='upper left')

    save_fig(fig, 'cornering_solution_range.pdf')
    return Caf_best, Car_best, wn_best_used


def plot_rolling_resistance(data_dir, prefix):
    """Rolling resistance: coast-down velocity vs time for all runs."""
    paths = find_csvs('rolling_resistance', data_dir, prefix)
    if not paths:
        print('  [skip] no rolling_resistance CSV found')
        return

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    c_rolls_v = []
    c_rolls_imu = []

    for i, p in enumerate(paths):
        d = load_csv(p)
        phase = d.get('phase', np.array([]))
        coast_mask = phase == 'coast'
        if np.sum(coast_mask) < 10:
            continue

        t = d['timestamp_s'][coast_mask]
        vx = d['odom_vx'][coast_mask]
        ax_imu = d['imu_ax'][coast_mask]

        # Zero-reference time for coast phase
        t = t - t[0]

        # Linear fit v(t) = v0 + a*t
        coeffs = np.polyfit(t, vx, 1)
        a_coast = coeffs[0]
        c_roll_v = abs(a_coast) / GRAVITY

        # IMU method
        c_roll_imu = np.mean(np.abs(ax_imu)) / GRAVITY

        c_rolls_v.append(c_roll_v)
        c_rolls_imu.append(c_roll_imu)

        # Plot individual runs
        color = plt.cm.tab10(i % 10)
        ax1.plot(t, vx, '-', color=color, alpha=0.6, linewidth=1,
                 label=f'Run {i+1}' if i < 5 else None)

        # Fit line (only for first 3 runs)
        if i < 3:
            t_fit = np.linspace(0, t[-1], 100)
            ax1.plot(t_fit, np.polyval(coeffs, t_fit), '--', color=color,
                     alpha=0.5, linewidth=0.8)

    if not c_rolls_v:
        print('  [skip] no valid coast data')
        plt.close(fig)
        return

    c_rolls_v = np.array(c_rolls_v)
    c_rolls_imu = np.array(c_rolls_imu)

    ax1.set_xlabel('Time since coast start (s)')
    ax1.set_ylabel('Speed (m/s)')
    ax1.set_title('Coast-Down Velocity Profiles')
    ax1.legend(fontsize=7, loc='upper right', ncol=2)
    ax1.grid(True, alpha=0.3)

    # Right panel: bar chart of c_roll per run
    runs = np.arange(1, len(c_rolls_v) + 1)
    ax2.bar(runs - 0.15, c_rolls_v, 0.3, color='tab:blue', alpha=0.7,
            label='Velocity fit')
    ax2.bar(runs + 0.15, c_rolls_imu, 0.3, color='tab:orange', alpha=0.7,
            label='IMU')
    ax2.axhline(np.mean(c_rolls_v), ls='--', color='tab:blue', lw=1.5,
                label=rf'Mean = {np.mean(c_rolls_v):.3f} $\pm$ {np.std(c_rolls_v):.3f}')
    ax2.set_xlabel('Run')
    ax2.set_ylabel(r'$c_\mathrm{roll}$')
    ax2.set_title('Rolling Resistance per Run')
    ax2.set_xticks(runs)
    ax2.legend(fontsize=7)
    ax2.grid(True, alpha=0.3, axis='y')

    fig.tight_layout()
    save_fig(fig, 'rolling_resistance_coastdown.pdf')


def plot_torque_I_vs_F(data_dir, prefix):
    """Motor current vs net force – two-panel view."""
    paths = find_csvs('motor_torque', data_dir, prefix)
    if not paths:
        print('  [skip] no motor_torque CSV found')
        return

    # Use latest raw CSV
    d = load_csv(paths[-1])
    phase = d.get('phase', np.array([]))
    accel_mask = phase == 'acceleration'
    if not np.any(accel_mask):
        print('  [skip] no acceleration data in motor_torque CSV')
        return

    I_all = d['motor_current'][accel_mask]
    ax_imu = d['imu_ax'][accel_mask]
    vx = d['odom_vx'][accel_mask]
    F_all = DEFAULT_MASS * ax_imu

    Kt = 60.0 / (2.0 * np.pi * 3500.0)
    G = 11.82
    r_eff = 0.051
    SLOPE_THEORY = Kt * G / r_eff
    I_LIM = 60.0

    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(10, 4.5),
                                            gridspec_kw={'wspace': 0.35})

    # Left: raw scatter colour = velocity
    sc = ax_left.scatter(I_all, F_all, c=vx, s=2, alpha=0.25,
                         cmap='viridis', rasterized=True)
    cb = fig.colorbar(sc, ax=ax_left, pad=0.02)
    cb.set_label('Velocity (m/s)')
    ax_left.axvspan(I_LIM, I_all.max() + 2, color='red', alpha=0.08)
    ax_left.axvline(I_LIM, color='red', ls='--', lw=0.8, alpha=0.6)
    ax_left.set_xlabel('Motor current (A)')
    ax_left.set_ylabel(r'Net force $F_{net} = m \cdot a_x$ (N)')
    ax_left.set_title('Raw scatter (all acceleration data)')
    ax_left.grid(True, alpha=0.3)

    # Right: binned linear region
    linear_mask = (F_all > 1.0) & (I_all > 5.0) & (I_all < I_LIM)
    I_lin = I_all[linear_mask]
    F_lin = F_all[linear_mask]

    if len(I_lin) > 20:
        bin_edges = np.arange(5, 56, 5)
        bin_centres, bin_means, bin_stds = [], [], []
        for lo, hi in zip(bin_edges[:-1], bin_edges[1:]):
            m = (I_lin >= lo) & (I_lin < hi)
            if m.sum() >= 3:
                bin_centres.append((lo + hi) / 2)
                bin_means.append(np.mean(F_lin[m]))
                bin_stds.append(np.std(F_lin[m]))

        bin_centres = np.array(bin_centres)
        bin_means = np.array(bin_means)
        bin_stds = np.array(bin_stds)

        ax_right.errorbar(bin_centres, bin_means, yerr=bin_stds, fmt='o',
                          capsize=4, color='tab:blue', markersize=7,
                          label=r'Bin mean $\pm\, 1\sigma$')

        if len(bin_centres) > 1:
            p = np.polyfit(bin_centres, bin_means, 1)
            I_fit = np.linspace(0, 60, 80)
            r2_val = 1 - (np.sum((bin_means - np.polyval(p, bin_centres))**2)
                          / np.sum((bin_means - np.mean(bin_means))**2))
            ax_right.plot(I_fit, np.polyval(p, I_fit), '--', color='tab:blue',
                          alpha=0.7, lw=1.5,
                          label=f'Fit: {p[0]:.3f} N/A (R²={r2_val:.2f})')

        ax_right.plot(I_fit, SLOPE_THEORY * I_fit, ':', color='tab:orange',
                      lw=1.5, label=f'Theoretical: {SLOPE_THEORY:.3f} N/A')

    ax_right.set_xlabel('Motor current (A)')
    ax_right.set_ylabel(r'Net force $F_{net}$ (N)')
    ax_right.set_title(r'Linear region (5–55 A, $F>1$ N)')
    ax_right.legend(fontsize=7, loc='upper left')
    ax_right.grid(True, alpha=0.3)
    ax_right.set_xlim(0, 60)
    ax_right.set_ylim(bottom=0)

    fig.suptitle('Motor Torque Mapping — Current vs. Force', fontsize=11, y=1.01)
    fig.tight_layout()
    save_fig(fig, 'torque_I_vs_F.pdf')


def plot_torque_current_time(data_dir, prefix):
    """Motor current time series at each speed target."""
    path = find_latest('motor_torque', data_dir, prefix)
    if not path:
        print('  [skip] no motor_torque raw CSV found')
        return

    d = load_csv(path)
    t_raw = d['timestamp_s'].astype(float)
    current = d['motor_current'].astype(float)
    phase = d.get('phase', np.array([''] * len(t_raw)))
    cmd_speed = d.get('cmd_speed', np.zeros_like(t_raw))

    # Collapse time gaps
    GAP_SEP = 0.5
    dt = np.diff(t_raw)
    gap_idx = np.where(dt > 1.0)[0]

    t = t_raw.copy()
    for gi in gap_idx:
        actual_gap = t_raw[gi + 1] - t_raw[gi]
        shift = actual_gap - GAP_SEP
        t[gi + 1:] -= shift

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 5), sharex=True,
                                    gridspec_kw={'height_ratios': [2, 1]})

    # Insert NaN at gap positions to break lines
    t_plot = t.copy()
    cur_plot = current.copy()
    spd_plot = cmd_speed.copy()
    for offset, gi in enumerate(gap_idx):
        idx = gi + 1 + offset
        nan_t = (t_plot[idx - 1] + t_plot[idx]) / 2.0 if idx < len(t_plot) else t_plot[-1]
        t_plot = np.insert(t_plot, idx, nan_t)
        cur_plot = np.insert(cur_plot, idx, np.nan)
        spd_plot = np.insert(spd_plot, idx, np.nan)

    ax1.plot(t_plot, cur_plot, '-', linewidth=0.5, color='tab:blue', alpha=0.7)
    ax1.set_ylabel('Motor current (A)')
    ax1.set_title('Motor Current vs. Time')
    ax1.grid(True, alpha=0.3)

    # Shade acceleration vs braking regions
    accel_mask = phase == 'acceleration'
    brake_mask = phase == 'braking'
    for mask, colour in [(accel_mask, 'green'), (brake_mask, 'red')]:
        if np.any(mask):
            diffs = np.diff(np.where(mask)[0])
            splits = np.where(diffs > 5)[0] + 1
            segments = np.split(np.where(mask)[0], splits)
            for seg in segments:
                if len(seg) > 0:
                    ax1.axvspan(t[seg[0]], t[seg[-1]], alpha=0.08, color=colour)

    ax2.plot(t_plot, spd_plot, '-', color='tab:green', linewidth=1,
             drawstyle='steps-post')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Cmd speed (m/s)')
    ax2.grid(True, alpha=0.3)

    for gi in gap_idx:
        for axi in (ax1, ax2):
            axi.axvline(t[gi] + GAP_SEP * 0.5, color='grey', ls=':',
                        lw=0.6, alpha=0.5)

    fig.tight_layout()
    save_fig(fig, 'torque_current_vs_time.pdf')


def plot_max_dynamics(data_dir, prefix):
    """Speed vs time for max acceleration/braking test."""
    # Max dynamics data is in oldData
    path = find_latest('max_dynamics', data_dir, prefix)
    if not path:
        # Try oldData
        old_dir = os.path.join(os.path.dirname(data_dir), 'oldData')
        path = find_latest('max_dynamics', old_dir, prefix)
    if not path:
        print('  [skip] no max_dynamics CSV found')
        return

    d = load_csv(path)
    t = d['timestamp_s']
    vx = d['odom_vx']
    phase = d.get('phase', np.array([''] * len(t)))

    fig, ax = setup_fig(width=7)
    ax.plot(t, vx, '-', label=r'$v_x$ (odometry)', color='tab:blue', linewidth=1.5)

    ax_imu = d['imu_ax']
    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1] if len(dt) > 1 else 0.005

    decel_mask = phase == 'deceleration'
    accel_mask = phase == 'acceleration'

    if np.any(decel_mask):
        decel_start = np.argmax(decel_mask)
        v_imu = np.full_like(t, np.nan)
        v_imu[decel_start] = vx[decel_start]
        for i in range(decel_start + 1, len(t)):
            v_imu[i] = max(0, v_imu[i - 1] + ax_imu[i] * dt[i])
        ax.plot(t, v_imu, '-', label=r'$v_x$ (IMU integrated)',
                color='tab:orange', linewidth=1.5)

    if np.any(accel_mask):
        ta = t[accel_mask]
        ax.axvspan(ta[0], ta[-1], alpha=0.06, color='green', label='Accel')
    if np.any(decel_mask):
        td = t[decel_mask]
        ax.axvspan(td[0], td[-1], alpha=0.06, color='red', label='Braking')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (m/s)')
    ax.set_title('Maximum Dynamics — Acceleration and Braking')
    ax.legend(loc='best')
    save_fig(fig, 'max_dyn_speed_vs_time.pdf')


def plot_steering_rate(data_dir, prefix):
    """IMU yaw rate and commanded steering vs time."""
    path = find_latest('steering_rate', data_dir, prefix)
    if not path:
        print('  [skip] no steering_rate CSV found')
        return

    d = load_csv(path)
    t = d['timestamp_s'].astype(float)
    omega = d['imu_gz'].astype(float)
    cmd = d['cmd_steering'].astype(float)

    # Detect recording gaps and insert NaN to break lines
    dt = np.diff(t)
    gap_idx = np.where(dt > 0.1)[0]

    t_plot = t.copy()
    omega_plot = omega.copy()
    cmd_plot = cmd.copy()
    for offset, gi in enumerate(gap_idx):
        idx = gi + 1 + offset
        mid_t = (t_plot[idx - 1] + t_plot[idx]) / 2.0
        t_plot = np.insert(t_plot, idx, mid_t)
        omega_plot = np.insert(omega_plot, idx, np.nan)
        cmd_plot = np.insert(cmd_plot, idx, np.nan)

    fig, ax1 = setup_fig(width=7)
    color_omega = 'tab:blue'
    color_cmd = 'tab:red'

    ax1.plot(t_plot, np.degrees(omega_plot), '-', color=color_omega,
             linewidth=0.8, label=r'$\omega_z$ (IMU)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'Yaw rate $\omega_z$ (deg/s)', color=color_omega)
    ax1.tick_params(axis='y', labelcolor=color_omega)

    ax2 = ax1.twinx()
    ax2.plot(t_plot, np.degrees(cmd_plot), '--', color=color_cmd,
             linewidth=1.2, label=r'$\delta_{cmd}$ (commanded)')
    ax2.set_ylabel(r'Steering command $\delta$ (deg)', color=color_cmd)
    ax2.tick_params(axis='y', labelcolor=color_cmd)

    lines1, labels1 = ax1.get_legend_handles_labels()
    lines2, labels2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, labels1 + labels2, loc='best')

    ax1.set_title('Steering Step Response')
    fig.tight_layout()
    save_fig(fig, 'steering_step_response.pdf')


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Generate report figures from test CSV data.')
    parser.add_argument('--data-dir', type=str, default=DATA_DIR,
                        help=f'Directory with CSV files (default: {DATA_DIR})')
    parser.add_argument('--prefix', type=str, default=None,
                        help='Filter CSVs by date prefix (e.g. 20260301)')
    parser.add_argument('--wn-best', type=str, default='10.0',
                        help=("Best-estimate omega_n in rad/s for selecting one "
                              "C_alpha pair, or 'auto' to estimate from steering "
                              "step-response data (default: 10.0)."))
    parser.add_argument('--wn-range', type=float, nargs=2,
                        metavar=('WN_MIN', 'WN_MAX'),
                        default=(8.0, 15.0),
                        help='Shaded omega_n range in cornering plot (default: 8 15).')
    args = parser.parse_args()

    data_dir = args.data_dir
    prefix = args.prefix
    wn_lo, wn_hi = sorted((float(args.wn_range[0]), float(args.wn_range[1])))
    wn_range = (wn_lo, wn_hi)

    wn_best_source = 'user/default'
    if args.wn_best.lower() == 'auto':
        est = estimate_wn_from_steering_rate(data_dir, prefix)
        if est is not None:
            wn_best = float(est['wn'])
            wn_best_source = (f"auto estimate from steering_rate "
                              f"(n={est['sample_count']}, "
                              f"IQR=[{est['q25']:.2f}, {est['q75']:.2f}])")
            # Guard against using an auto-estimate that is inconsistent with the
            # intended reporting range; keep reporting within the selected band.
            if not (wn_lo <= wn_best <= wn_hi):
                wn_best_mid = 0.5 * (wn_lo + wn_hi)
                print(f"[warn] auto omega_n={wn_best:.2f} rad/s is outside "
                      f"selected range [{wn_lo:.2f}, {wn_hi:.2f}] rad/s.")
                print(f"[warn] using midpoint omega_n={wn_best_mid:.2f} rad/s "
                      f"for cornering-solution reporting.")
                wn_best = wn_best_mid
                wn_best_source += '; out-of-range fallback to range midpoint'
        else:
            wn_best = 10.0
            wn_best_source = 'fallback default (auto unavailable)'
    else:
        try:
            wn_best = float(args.wn_best)
        except ValueError:
            print(f"Invalid --wn-best value: {args.wn_best}")
            print("Use a numeric value in rad/s or 'auto'.")
            sys.exit(2)

    if not os.path.isdir(data_dir):
        print(f'Data directory not found: {data_dir}')
        print('Run the tests first to generate CSV data.')
        sys.exit(1)

    print(f'Data dir: {data_dir}')
    print(f'Output:   {FIG_DIR}/')
    if prefix:
        print(f'Prefix:   {prefix}')
    print(f'omega_n range: [{wn_lo:.2f}, {wn_hi:.2f}] rad/s')
    print(f'omega_n best:  {wn_best:.2f} rad/s ({wn_best_source})')
    print()

    # ── 1. Friction ──
    print('[1/8  Friction a_y vs speed]')
    try:
        plot_friction(data_dir, prefix)
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 2. Cornering understeer gradient ──
    print('[2/8  Cornering understeer gradient]')
    K_us = None
    try:
        result = plot_cornering_understeer_gradient(data_dir, prefix)
        if result:
            K_us = result[0]
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 3. Cornering solution range ──
    print('[3/8  Cornering solution range]')
    try:
        result = plot_cornering_solution_range(data_dir, prefix, K_us=K_us,
                                               wn_range=wn_range, wn_best=wn_best)
        if result:
            Caf_best, Car_best, wn_best_used = result
            print(f'  [info] selected omega_n on curve: {wn_best_used:.2f} rad/s')
            print(f'  [info] C_alpha_f={Caf_best:.2f} N/rad, '
                  f'C_alpha_r={Car_best:.2f} N/rad')
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 4. Rolling resistance coastdown ──
    print('[4/8  Rolling resistance coastdown]')
    try:
        plot_rolling_resistance(data_dir, prefix)
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 5. Motor torque I vs F ──
    print('[5/8  Motor torque I vs F]')
    try:
        plot_torque_I_vs_F(data_dir, prefix)
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 6. Motor torque current vs time ──
    print('[6/8  Motor torque current vs time]')
    try:
        plot_torque_current_time(data_dir, prefix)
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 7. Max dynamics speed vs time ──
    print('[7/8  Max dynamics speed vs time]')
    try:
        plot_max_dynamics(data_dir, prefix)
    except Exception as e:
        print(f'  [ERROR] {e}')

    # ── 8. Steering step response ──
    print('[8/8  Steering step response]')
    try:
        plot_steering_rate(data_dir, prefix)
    except Exception as e:
        print(f'  [ERROR] {e}')

    print(f'\nDone. Figures saved to {FIG_DIR}/')


if __name__ == '__main__':
    main()
