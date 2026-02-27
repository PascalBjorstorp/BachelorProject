#!/usr/bin/env python3
"""
Generate all report figures from test CSV data.

Usage:
    python3 generate_plots.py                      # auto-detect latest CSVs
    python3 generate_plots.py --data-dir data/     # explicit data directory
    python3 generate_plots.py --prefix 20260301    # filter by date prefix

Produces PDF figures in figures/ matching the report_outline.tex placeholders:
  1. wheelbase_circle_trajectory.pdf
  2. friction_ay_vs_speed.pdf
  3. cornering_Fy_vs_alpha.pdf
  4. cornering_alpha_vs_speed.pdf
  5. long_v_comparison.pdf
  6. long_Fx_vs_kappa.pdf
  7. torque_I_vs_F.pdf
  8. torque_current_vs_time.pdf
  9. max_dyn_speed_vs_time.pdf
  10. steering_step_response.pdf
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


def find_latest(pattern, data_dir, prefix=None):
    """Find the most recent CSV matching *pattern* in data_dir."""
    candidates = sorted(glob.glob(os.path.join(data_dir, f'*{pattern}*.csv')))
    if prefix:
        candidates = [c for c in candidates if prefix in os.path.basename(c)]
    # Exclude summary CSVs for raw data, include for summary
    if '_summary' not in pattern:
        candidates = [c for c in candidates if '_summary' not in c]
    if not candidates:
        return None
    return candidates[-1]  # latest by timestamp in filename


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


# ── Vehicle parameters (for computing derived quantities from raw data) ─────

DEFAULT_MASS = 3.314       # kg
DEFAULT_L = 0.324          # m  wheelbase
DEFAULT_LF = 0.166         # m  CG to front axle
DEFAULT_LR = 0.16          # m  CG to rear axle


def compute_cornering_summary(data_dir, prefix):
    """Compute per-(speed, steering) cornering stiffness summary from raw CSVs.

    Returns a dict matching the expected summary column layout:
      speed, alpha_f_deg, alpha_r_deg, F_yf, F_yr
    or None if no data is available.
    """
    paths = sorted(glob.glob(os.path.join(data_dir, '*cornering_stiffness*.csv')))
    if prefix:
        paths = [p for p in paths if prefix in os.path.basename(p)]
    paths = [p for p in paths if '_summary' not in p]
    if not paths:
        return None

    # Aggregate all raw runs
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
    out = {'speed': [], 'alpha_f_deg': [], 'alpha_r_deg': [],
           'F_yf': [], 'F_yr': [], 'cmd_steering': []}

    for sp, st in pairs:
        sm = (cmd_speed == sp) & (np.abs(steer) == st)
        v = np.mean(np.abs(vx[sm]))
        if v < 0.3:
            continue
        omega = np.mean(gz[sm])
        delta = st  # use absolute value of steering
        a_y = np.mean(ay[sm])

        # Slip angles (bicycle model, small v_y assumption)
        alpha_f = delta - np.arctan2(DEFAULT_LF * omega, v)
        alpha_r = np.arctan2(DEFAULT_LR * omega, v)

        # Lateral forces (steady-state: l_f*F_yf = l_r*F_yr, F_yf+F_yr = m*a_y)
        F_yf = DEFAULT_MASS * a_y * DEFAULT_LR / DEFAULT_L
        F_yr = DEFAULT_MASS * a_y * DEFAULT_LF / DEFAULT_L

        out['speed'].append(sp)
        out['cmd_steering'].append(st)
        out['alpha_f_deg'].append(np.degrees(alpha_f))
        out['alpha_r_deg'].append(np.degrees(alpha_r))
        out['F_yf'].append(F_yf)
        out['F_yr'].append(F_yr)

    if not out['speed']:
        return None

    return {k: np.array(v) for k, v in out.items()}


# ── Plot functions ───────────────────────────────────────────────────────────

def plot_wheelbase(data_dir, prefix):
    """Figure 1: Wheelbase test — calibrated servo gain results."""
    paths = sorted(glob.glob(os.path.join(data_dir, '*wheelbase_test*.csv')))
    if prefix:
        paths = [p for p in paths if prefix in os.path.basename(p)]
    if not paths:
        print('  [skip] no wheelbase_test CSV found')
        return

    # ── Servo gain constants (calibrated) ───────────────────────────
    SERVO_GAIN   = -0.7940
    SERVO_OFFSET =  0.5500
    PHYSICAL_L   =  0.324   # CAD wheelbase (m)
    CMD_STEER    =  0.3     # rad — commanded steering angle

    # The servo duty that was physically sent during the test = 0.7548.
    # Using the calibrated gain, compute the actual steering angle.
    SERVO_DUTY_DURING_TEST = 0.7548
    delta_actual = abs((SERVO_DUTY_DURING_TEST - SERVO_OFFSET) / SERVO_GAIN)
    beta = np.arctan(0.5 * np.tan(delta_actual))

    # Load all runs
    all_runs = []
    for path in paths:
        all_runs.append(load_csv(path))

    # Collect per-run, per-speed circle radii
    speed_data = {}  # speed -> list of dicts
    for run_idx, d in enumerate(all_runs):
        phase = d.get('phase', np.array([]))
        circle_phases = sorted(set(
            p for p in phase if isinstance(p, str) and p.startswith('circle_v')))
        for cp in circle_phases:
            mask = phase == cp
            if np.sum(mask) < 50:
                continue
            vx = d['odom_vx'][mask]
            gz = d['imu_gz'][mask]
            om_odom = d['odom_omega'][mask]

            # Trim 20 % from each end (transient)
            n = len(vx)
            trim = int(n * 0.2)
            if n - 2 * trim < 10:
                continue
            vx_t = vx[trim:n - trim]
            gz_t = gz[trim:n - trim]
            om_t = om_odom[trim:n - trim]

            valid_imu  = np.abs(gz_t) > 0.05
            valid_odom = np.abs(om_t) > 0.05
            if np.sum(valid_imu) < 5 or np.sum(valid_odom) < 5:
                continue

            r_imu  = np.median(np.abs(vx_t[valid_imu]  / gz_t[valid_imu]))
            r_odom = np.median(np.abs(vx_t[valid_odom] / om_t[valid_odom]))

            speed_val = float(cp.replace('circle_v', ''))
            entry = {
                'run':    run_idx + 1,
                'R_imu':  r_imu,
                'R_odom': r_odom,
                'L_imu':  r_imu  * 2 * np.sin(beta),
                'L_odom': r_odom * 2 * np.sin(beta),
                'v_mean': np.mean(vx_t),
                'v_std':  np.std(vx_t),
            }
            speed_data.setdefault(speed_val, []).append(entry)

    if not speed_data:
        print('  [skip] no valid circle data extracted')
        return

    speeds_sorted = sorted(speed_data.keys())
    n_runs = len(all_runs)
    lowest_speed = speeds_sorted[0]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    # ── Panel 1 (top-left): Wheelbase per speed ────────────────────
    ax = axes[0, 0]
    sp_means = [np.mean([e['L_imu'] for e in speed_data[s]])
                for s in speeds_sorted]
    sp_stds = [np.std([e['L_imu'] for e in speed_data[s]])
               for s in speeds_sorted]

    x_pos = np.arange(len(speeds_sorted))
    ax.bar(x_pos, sp_means, 0.5, yerr=sp_stds, capsize=5,
           color='tab:blue', alpha=0.7, label='Measured (IMU)')
    ax.axhline(PHYSICAL_L, color='k', ls=':', lw=1.5,
               label=f'CAD ({PHYSICAL_L} m)')
    ax.set_xticks(x_pos)
    ax.set_xticklabels([f'{s:.1f}' for s in speeds_sorted])
    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel('Wheelbase (m)')
    ax.set_title('Effective Wheelbase vs Speed')
    ax.legend(fontsize=8, loc='upper left')
    ax.grid(True, alpha=0.3, axis='y')
    # Annotate speed-dependent growth
    if len(speeds_sorted) >= 2:
        growth = (sp_means[-1] - sp_means[0]) / sp_means[0] * 100
        ax.text(0.02, 0.72,
                f'Growth: +{growth:.1f}%\n(understeer + tire slip)',
                transform=ax.transAxes, fontsize=8,
                bbox=dict(boxstyle='round,pad=0.3', facecolor='lightyellow',
                          alpha=0.8, edgecolor='none'))

    # ── Panel 2 (top-right): Per-run consistency at lowest speed ───
    ax = axes[0, 1]
    entries = speed_data[lowest_speed]
    runs = [e['run'] for e in entries]
    L_vals = [e['L_imu'] for e in entries]
    L_mean = np.mean(L_vals)
    L_std  = np.std(L_vals)

    ax.bar(runs, L_vals, 0.6, color='tab:blue', alpha=0.7)
    ax.axhline(PHYSICAL_L, color='k', ls=':', lw=1.5,
               label=f'CAD ({PHYSICAL_L} m)')
    ax.axhline(L_mean, color='tab:red', ls='-', lw=2,
               label=f'Mean = {L_mean:.4f} m')
    ax.axhspan(L_mean - L_std, L_mean + L_std, alpha=0.15, color='tab:red',
               label=rf'$\pm 1\sigma$ = {L_std:.4f} m')
    ax.set_xlabel('Run')
    ax.set_ylabel('Wheelbase (m)')
    ax.set_title(f'Per-Run Consistency (v = {lowest_speed} m/s)')
    ax.set_xticks(runs)
    ax.legend(fontsize=7)
    ax.grid(True, alpha=0.3, axis='y')
    y_margin = max(L_std * 5, 0.01)
    ax.set_ylim(L_mean - y_margin, L_mean + y_margin)

    # ── Panel 3 (bottom-left): Circle trajectory (latest run) ──────
    ax = axes[1, 0]
    d = all_runs[-1]
    phase = d.get('phase', np.array([]))
    x = d['odom_x']
    y = d['odom_y']
    circle_phases = sorted(set(
        p for p in phase if isinstance(p, str) and p.startswith('circle_v')))
    cmap_colors = plt.cm.viridis(
        np.linspace(0.2, 0.9, max(len(circle_phases), 1)))
    for i, cp in enumerate(circle_phases):
        mask = phase == cp
        if np.sum(mask) < 10:
            continue
        speed_val = cp.replace('circle_v', '')
        ax.plot(x[mask], y[mask], '-', color=cmap_colors[i], linewidth=1.2,
                label=f'v = {speed_val} m/s', alpha=0.85)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_aspect('equal')
    ax.set_title(f'Circle Trajectories (run {n_runs})')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3)

    # ── Panel 4 (bottom-right): Summary text ───────────────────────
    ax = axes[1, 1]
    ax.axis('off')

    lines = []
    lines.append('WHEELBASE TEST SUMMARY')
    lines.append('=' * 40)
    lines.append(f'Runs: {n_runs}')
    lines.append(f'Servo gain: {SERVO_GAIN}')
    lines.append(f'Cmd δ: {np.degrees(CMD_STEER):.1f}°')
    lines.append(f'Actual δ: {np.degrees(delta_actual):.1f}°')
    lines.append(f'δ_actual / δ_cmd = {delta_actual/CMD_STEER:.2f}')
    lines.append('')
    lines.append(f'{"Speed":>6} {"L_meas":>8} {"vs CAD":>9}')
    lines.append(f'{"-"*6:>6} {"-"*8:>8} {"-"*9:>9}')
    for i, sp in enumerate(speeds_sorted):
        diff_pct = (sp_means[i] / PHYSICAL_L - 1) * 100
        lines.append(f'{sp:6.1f} {sp_means[i]:8.4f} {diff_pct:+8.1f}%')
    lines.append('')
    lines.append(f'Best (v={lowest_speed} m/s, IMU):')
    lines.append(f'  L = {L_mean:.4f} ± {L_std:.4f} m')
    lines.append(f'  CAD = {PHYSICAL_L} m')
    diff_pct = (L_mean / PHYSICAL_L - 1) * 100
    lines.append(f'  Diff: {diff_pct:+.1f}%')
    lines.append('')
    lines.append('Understeer inflates apparent L')
    lines.append('(see report for K_us analysis)')

    text = '\n'.join(lines)
    ax.text(0.05, 0.95, text, transform=ax.transAxes,
            va='top', ha='left', fontsize=9, family='monospace',
            bbox=dict(boxstyle='round,pad=0.5', facecolor='white',
                      edgecolor='gray', alpha=0.9))

    fig.suptitle('Wheelbase Test', fontsize=13, y=1.01)
    fig.tight_layout()
    save_fig(fig, 'wheelbase_circle_trajectory.pdf')


def plot_friction(data_dir, prefix):
    """Figure 1: a_y (IMU) vs speed with mu line."""
    path = find_latest('friction', data_dir, prefix)
    summary = find_latest('friction_summary', data_dir, prefix)
    if not path and not summary:
        print('  [skip] no friction CSV found')
        return

    fig, ax = setup_fig()

    if summary and os.path.exists(summary):
        d = load_csv(summary)
        speeds = d['speed_actual']
        ay = d['ay_imu']
        ay_std = d.get('ay_std', np.zeros_like(ay))
        mu = d.get('mu', ay / GRAVITY)

        ax.errorbar(speeds, ay, yerr=ay_std, fmt='o-', capsize=4,
                     color='tab:blue', label=r'$\bar{a}_y$ (IMU)')
        ax.axhline(np.max(ay), ls='--', color='tab:red', alpha=0.7,
                    label=rf'$\mu g = {np.max(mu):.2f} \times {GRAVITY:.1f}$'
                          rf' = {np.max(ay):.2f} m/s²')
        ax.set_xlabel('Speed (m/s)')
        ax.set_ylabel(r'Lateral acceleration $a_y$ (m/s²)')
        ax.set_title('Friction Coefficient Identification')
        if 'n_samples_total' in d and 'n_samples' in d and len(d['n_samples']) > 0:
            retention = np.mean(d['n_samples'] / np.maximum(d['n_samples_total'], 1.0)) * 100.0
            ax.text(0.02, 0.98, f'Steady-state sample retention: {retention:.0f}%',
                transform=ax.transAxes, va='top', ha='left', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.2', facecolor='white', alpha=0.7, edgecolor='none'))
        ax.legend()
    else:
        # Fall back to raw CSV — compute per-speed stats
        d = load_csv(path)
        phase = d.get('phase', np.array([]))
        ay = d['imu_ay']

        # Phases are named 'friction_vX.X'
        friction_phases = sorted(
            set(p for p in phase if isinstance(p, str) and p.startswith('friction_v')))

        if friction_phases:
            speeds = []
            ay_means = []
            ay_stds = []
            for fp in friction_phases:
                mask = phase == fp
                if np.sum(mask) < 10:
                    continue
                ay_vals = np.abs(ay[mask])
                # Trim 20% from each end for transients
                n = len(ay_vals)
                trim = int(n * 0.2)
                if n - 2 * trim < 5:
                    continue
                ay_vals = ay_vals[trim:n - trim]
                speed_val = float(fp.replace('friction_v', ''))
                speeds.append(speed_val)
                ay_means.append(np.mean(ay_vals))
                ay_stds.append(np.std(ay_vals))

            if speeds:
                speeds = np.array(speeds)
                ay_means = np.array(ay_means)
                ay_stds = np.array(ay_stds)
                mu = ay_means / GRAVITY

                ax.errorbar(speeds, ay_means, yerr=ay_stds, fmt='o-',
                            capsize=4, color='tab:blue',
                            label=r'$\bar{a}_y$ (IMU)')

                # Identify mu from highest a_y
                mu_max = np.max(mu)
                ay_max = np.max(ay_means)

                ax.axhline(ay_max, ls='--', color='tab:red', alpha=0.7,
                           linewidth=2,
                           label=rf'$\mu g = {mu_max:.2f} \times'
                                 rf' {GRAVITY:.1f} = {ay_max:.2f}$'
                                 r' m/s$^2$')

                ax.set_xlabel('Speed (m/s)')
                ax.set_ylabel(r'Lateral acceleration $a_y$ (m/s$^2$)')
                ax.set_title('Friction Coefficient Identification')
                ax.legend(loc='upper left')
            else:
                ax.text(0.5, 0.5, 'No valid friction data',
                        transform=ax.transAxes, ha='center', va='center')
        else:
            # Truly raw: plot over time
            t = d['timestamp_s']
            ax.plot(t, ay, '.', markersize=1, alpha=0.3, label=r'$a_y$ (IMU)')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(r'$a_y$ (m/s²)')
            ax.set_title('Friction Test — Raw IMU Data')
            ax.legend()

    save_fig(fig, 'friction_ay_vs_speed.pdf')


def plot_cornering_Fy_alpha(data_dir, prefix):
    """Figure 2: F_y vs slip angle + effective C_alpha vs speed."""
    summary = find_latest('cornering_stiffness_summary', data_dir, prefix)
    if summary:
        d = load_csv(summary)
    else:
        d = compute_cornering_summary(data_dir, prefix)
    if not d:
        print('  [skip] no cornering_stiffness data found')
        return

    if 'consistency_rmse' in d:
        keep = d['consistency_rmse'] < 1.0
        if np.any(keep):
            d = {k: v[keep] for k, v in d.items()}
    alpha_f = d['alpha_f_deg']
    alpha_r = d['alpha_r_deg']
    Fyf = d['F_yf']
    Fyr = d['F_yr']
    speeds = d['speed']

    # Compute effective C_alpha at each speed
    C_af = np.abs(Fyf) / np.abs(np.radians(alpha_f))
    C_ar = np.abs(Fyr) / np.abs(np.radians(alpha_r))

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # ── Left panel: F_y vs slip angle (data points, speed annotated) ──
    ax1.plot(np.abs(alpha_f), np.abs(Fyf), 'o-', color='tab:blue',
             markersize=7, label='Front')
    ax1.plot(np.abs(alpha_r), np.abs(Fyr), 's-', color='tab:orange',
             markersize=7, label='Rear')

    # Annotate speed at each point
    for i, sp in enumerate(speeds):
        ax1.annotate(f'{sp:.1f}', (np.abs(alpha_f[i]), np.abs(Fyf[i])),
                     textcoords='offset points', xytext=(5, 5),
                     fontsize=7, color='tab:blue')
        ax1.annotate(f'{sp:.1f}', (np.abs(alpha_r[i]), np.abs(Fyr[i])),
                     textcoords='offset points', xytext=(5, -10),
                     fontsize=7, color='tab:orange')

    ax1.plot(0, 0, 'kx', markersize=8, markeredgewidth=2, zorder=5)
    ax1.set_xlabel(r'Slip angle $|\alpha|$ (deg)')
    ax1.set_ylabel(r'Lateral force $|F_y|$ (N)')
    ax1.set_title(r'$F_y$ vs. Slip Angle (speed annotated, m/s)')
    ax1.legend(loc='upper left')
    ax1.grid(True, alpha=0.3)

    # ── Right panel: Effective C_alpha vs speed ──
    ax2.plot(speeds, C_af, 'o-', color='tab:blue', markersize=7,
             label=rf'$C_{{\alpha f}}$ (front)')
    ax2.plot(speeds, C_ar, 's-', color='tab:orange', markersize=7,
             label=rf'$C_{{\alpha r}}$ (rear)')

    # Show mean values
    C_af_mean = np.mean(C_af)
    C_ar_mean = np.mean(C_ar)
    ax2.axhline(C_af_mean, ls='--', color='tab:blue', alpha=0.5)
    ax2.axhline(C_ar_mean, ls='--', color='tab:orange', alpha=0.5)
    ax2.text(speeds[0], C_af_mean * 1.05,
             rf'mean = {C_af_mean:.0f} N/rad', fontsize=8, color='tab:blue')
    ax2.text(speeds[0], C_ar_mean * 0.90,
             rf'mean = {C_ar_mean:.0f} N/rad', fontsize=8, color='tab:orange')

    ax2.set_xlabel('Speed (m/s)')
    ax2.set_ylabel(r'Effective $C_\alpha$ (N/rad)')
    ax2.set_title(r'Effective $C_\alpha$ vs. Speed')
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    # Note about data limitation
    fig.text(0.5, -0.02,
             r'Note: Test used single steering angle ($\delta = 0.1$ rad). '
             r'$C_\alpha$ varies with speed because slip angle range is narrow.',
             ha='center', fontsize=8, style='italic')

    fig.tight_layout()
    save_fig(fig, 'cornering_Fy_vs_alpha.pdf')


def plot_cornering_alpha_speed(data_dir, prefix):
    """Figure 3: slip angles vs speed."""
    summary = find_latest('cornering_stiffness_summary', data_dir, prefix)
    if summary:
        d = load_csv(summary)
    else:
        d = compute_cornering_summary(data_dir, prefix)
    if not d:
        print('  [skip] no cornering_stiffness data found')
        return

    if 'consistency_rmse' in d:
        keep = d['consistency_rmse'] < 1.0
        if np.any(keep):
            d = {k: v[keep] for k, v in d.items()}
    speeds = d['speed']
    alpha_f = np.abs(d['alpha_f_deg'])
    alpha_r = np.abs(d['alpha_r_deg'])

    fig, ax = setup_fig()
    ax.plot(speeds, alpha_f, 'o-', label=r'$|\alpha_f|$ (front)', color='tab:blue')
    ax.plot(speeds, alpha_r, 's-', label=r'$|\alpha_r|$ (rear)', color='tab:orange')
    ax.set_xlabel('Speed (m/s)')
    ax.set_ylabel(r'Slip angle $|\alpha|$ (deg)')
    ax.set_title('Slip Angle Growth with Speed')
    ax.legend()
    save_fig(fig, 'cornering_alpha_vs_speed.pdf')


def plot_long_v_comparison(data_dir, prefix):
    """Figure 4: v_wheel vs v_lidar (LiDAR scan-matching) over time.

    Uses the best (highest sample count) of the 10 longitudinal stiffness
    runs.  Shows v_wheel from ERPM odometry and v_body from LiDAR
    point-to-line ICP scan-matching with motion deskewing.
    """
    # Find all longitudinal stiffness CSVs
    candidates = sorted(glob.glob(os.path.join(data_dir, '*longitudinal_stiffness*.csv')))
    if prefix:
        candidates = [c for c in candidates if prefix in os.path.basename(c)]
    candidates = [c for c in candidates if '_summary' not in c]
    if not candidates:
        print('  [skip] no longitudinal_stiffness CSV found')
        return

    # Pick the file with the most rows
    best_path = None
    best_rows = 0
    for c in candidates:
        d = load_csv(c)
        n = len(d.get('odom_vx', []))
        if n > best_rows:
            best_rows = n
            best_path = c
    if not best_path:
        print('  [skip] no valid longitudinal_stiffness CSV')
        return

    d = load_csv(best_path)
    t = d['timestamp_s']
    t = t - t[0]  # zero-referenced time
    vw = d['odom_vx']
    vl = d['v_lidar']
    phase = d.get('phase', np.array([''] * len(t)))

    # Smooth the LiDAR velocity for visualization (reduce ICP noise)
    # Apply a zero-phase Butterworth low-pass filter at 3 Hz
    from scipy.signal import butter, filtfilt
    # Find effective sample rate from data
    dt_med = np.median(np.diff(t))
    fs = 1.0 / max(dt_med, 0.001)
    if fs > 10:  # only filter if sample rate is reasonable
        b, a = butter(2, 3.0, fs=fs)
        vl_smooth = filtfilt(b, a, vl)
    else:
        vl_smooth = vl

    fig, ax = setup_fig(width=7)
    ax.plot(t, vw, '-', label=r'$v_\mathrm{wheel}$ (ERPM)', color='tab:blue',
            alpha=0.8, linewidth=1.0)
    ax.plot(t, vl_smooth, '-', label=r'$v_\mathrm{body}$ (LiDAR ICP)', color='tab:orange',
            alpha=0.8, linewidth=1.0)

    # Shade phases
    for phase_name, colour in [('acceleration', 'green'), ('cruise', 'gold'),
                                ('braking', 'red')]:
        mask = phase == phase_name
        if np.any(mask):
            t_ph = t[mask]
            ax.axvspan(t_ph[0], t_ph[-1], alpha=0.08, color=colour,
                       label=phase_name.capitalize())

    # Annotate cruise ratio
    cruise_mask = phase == 'cruise'
    if np.any(cruise_mask):
        ratio = np.mean(vl[cruise_mask]) / np.mean(vw[cruise_mask])
        ax.text(0.98, 0.05, f'Cruise ratio = {ratio:.3f}',
                transform=ax.transAxes, ha='right', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Wheel Speed vs. LiDAR Body Speed — Longitudinal Slip')
    ax.legend(loc='best', fontsize=8)
    save_fig(fig, 'long_v_comparison.pdf')


def plot_long_Fx_kappa(data_dir, prefix):
    """Figure 5: F_x vs slip ratio κ — longitudinal tire stiffness identification.

    Pools all longitudinal stiffness test runs.  Uses LiDAR scan-matching
    body velocity to compute the slip ratio κ, and IMU a_x for the
    longitudinal force F_x = m·a_x.  A linear regression in the linear
    region (|κ| ∈ [0.005, 0.15], |v_lidar| > 2 m/s) yields C_x.
    """
    # Collect all longitudinal_stiffness CSVs
    candidates = sorted(glob.glob(os.path.join(data_dir, '*longitudinal_stiffness*.csv')))
    if prefix:
        candidates = [c for c in candidates if prefix in os.path.basename(c)]
    candidates = [c for c in candidates if '_summary' not in c]
    # Only use the latest batch (files from the same session, within 20 min)
    if candidates:
        # Extract timestamps, keep files within 20 min of the latest
        import re
        times = []
        for c in candidates:
            m = re.search(r'(\d{8}_\d{6})', os.path.basename(c))
            if m:
                times.append(m.group(1))
            else:
                times.append('')
        if times:
            latest = max(times)
            # parse YYYYMMDD_HHMMSS
            try:
                from datetime import datetime
                latest_dt = datetime.strptime(latest, '%Y%m%d_%H%M%S')
                batch = []
                for c, ts in zip(candidates, times):
                    if ts:
                        dt = datetime.strptime(ts, '%Y%m%d_%H%M%S')
                        if abs((latest_dt - dt).total_seconds()) < 1200:
                            batch.append(c)
                if len(batch) >= 3:
                    candidates = batch
            except ValueError:
                pass

    if not candidates:
        print('  [skip] no longitudinal_stiffness CSV found')
        return

    # Pool data from all runs (acceleration only)
    all_kappa = []
    all_Fx = []
    all_v = []
    n_runs = 0
    for path in candidates:
        d = load_csv(path)
        if 'v_lidar' not in d:
            continue
        vl = d['v_lidar']
        vw = d['odom_vx']
        ax = d['imu_ax']
        phase = d.get('phase', np.array([''] * len(vl)))

        mask = (phase == 'acceleration') & (np.abs(vl) > 2.0)
        if np.sum(mask) < 10:
            continue
        kappa = (vw[mask] - vl[mask]) / np.maximum(np.abs(vw[mask]), np.abs(vl[mask]))
        Fx = DEFAULT_MASS * ax[mask]
        all_kappa.append(kappa)
        all_Fx.append(Fx)
        all_v.append(vl[mask])
        n_runs += 1

    if not all_kappa:
        print('  [skip] no valid v_lidar data in longitudinal_stiffness CSVs')
        return

    kappa = np.concatenate(all_kappa)
    Fx = np.concatenate(all_Fx)
    v = np.concatenate(all_v)

    # Filter to linear region
    ok = (np.abs(kappa) > 0.005) & (np.abs(kappa) < 0.15)

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    # Left panel: scatter of acceleration data coloured by velocity
    ax = axes[0]
    ax.grid(True, alpha=0.3)
    sc = ax.scatter(kappa, Fx, c=v, s=2, alpha=0.15,
                    cmap='viridis', label='_nolegend_')
    cbar = fig.colorbar(sc, ax=ax, pad=0.02)
    cbar.set_label(r'$v_\mathrm{body}$ (m/s)')

    # Linear fit in linear region
    if np.sum(ok) > 10:
        from numpy.polynomial import polynomial as P
        c = P.polyfit(kappa[ok], Fx[ok], 1)
        kap_fit = np.linspace(-0.15, 0.15, 100)
        ax.plot(kap_fit, c[0] + c[1] * kap_fit, 'r-', linewidth=2,
                label=rf'$C_x = {c[1]:.1f}$ N/slip')
        # R²
        pred = c[0] + c[1] * kappa[ok]
        ss_res = np.sum((Fx[ok] - pred)**2)
        ss_tot = np.sum((Fx[ok] - np.mean(Fx[ok]))**2)
        r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0
        ax.text(0.02, 0.95,
                f'$R^2 = {r2:.3f}$\n$n = {np.sum(ok)}$ pts\n{n_runs} runs',
                transform=ax.transAxes, va='top', fontsize=9,
                bbox=dict(boxstyle='round,pad=0.3', fc='white', alpha=0.8))

    ax.set_xlabel(r'Slip ratio $\kappa$')
    ax.set_ylabel(r'Longitudinal force $F_x = m \cdot a_x$ (N)')
    ax.set_title(r'$F_x$ vs. $\kappa$ — Acceleration Data ($|v| > 2$ m/s)')
    ax.set_xlim(-0.25, 0.25)
    ax.legend(fontsize=8)

    # Right panel: bin-averaged in linear region
    ax = axes[1]
    ax.grid(True, alpha=0.3)
    if np.sum(ok) > 20:
        bin_edges = np.linspace(-0.15, 0.15, 31)
        centres, means, stds, counts = [], [], [], []
        for i in range(len(bin_edges) - 1):
            m = ok & (kappa >= bin_edges[i]) & (kappa < bin_edges[i + 1])
            if np.sum(m) > 5:
                centres.append(0.5 * (bin_edges[i] + bin_edges[i + 1]))
                means.append(np.mean(Fx[m]))
                stds.append(np.std(Fx[m]) / np.sqrt(np.sum(m)))
                counts.append(np.sum(m))
        centres = np.array(centres)
        means = np.array(means)
        stds = np.array(stds)

        ax.errorbar(centres, means, yerr=stds, fmt='ko', ms=5, capsize=3,
                    label='Bin mean ± SE')
        kap_fit = np.linspace(-0.15, 0.15, 100)
        ax.plot(kap_fit, c[0] + c[1] * kap_fit, 'r-', linewidth=2,
                label=rf'$C_x = {c[1]:.1f}$ N/slip')
        ax.axhline(0, color='gray', ls='-', lw=0.5)
        ax.axvline(0, color='gray', ls='-', lw=0.5)

    ax.set_xlabel(r'Slip ratio $\kappa$')
    ax.set_ylabel(r'Longitudinal force $F_x$ (N)')
    ax.set_title(r'$F_x$ vs. $\kappa$ — Bin-Averaged (Acceleration, $|v| > 2$ m/s)')
    ax.set_xlim(-0.18, 0.18)
    ax.legend(fontsize=8)

    fig.tight_layout()
    save_fig(fig, 'long_Fx_vs_kappa.pdf')


def plot_torque_I_vs_F(data_dir, prefix):
    """Figure 6: motor current vs net force – two-panel view.

    Left:  raw scatter coloured by velocity, showing current-limited launch
           vs. steady-state cruising.
    Right: bin-averaged I vs F in the linear region (I < 55 A, F > 1 N)
           with a linear fit and the theoretical K_t line.
    """
    summary = find_latest('motor_torque_summary', data_dir, prefix)
    raw = find_latest('motor_torque', data_dir, prefix)

    if not summary and not raw:
        print('  [skip] no motor_torque CSV found')
        return

    # --- load raw data (preferred for the improved plot) -----------------
    if raw:
        d = load_csv(raw)
    elif summary:
        # Fall back to summary if raw is missing
        d = load_csv(summary)
        fig, ax = setup_fig()
        accel = d.get('phase', np.array([])) == 'acceleration'
        if np.any(accel):
            ax.errorbar(d['avg_current'][accel], d['F_net'][accel],
                        fmt='o', capsize=4, color='tab:blue', label='Accel')
        ax.set_xlabel('Motor current (A)')
        ax.set_ylabel('Net force $F_{net}$ (N)')
        ax.set_title('Motor Torque Mapping — Current vs. Force')
        ax.legend()
        save_fig(fig, 'torque_I_vs_F.pdf')
        return

    phase = d.get('phase', np.array([]))
    accel_mask = phase == 'acceleration'
    if not np.any(accel_mask):
        print('  [skip] no acceleration data in motor_torque CSV')
        return

    I_all  = d['motor_current'][accel_mask]
    ax_imu = d['imu_ax'][accel_mask]
    vx     = d['odom_vx'][accel_mask]
    F_all  = 3.314 * ax_imu                       # F = m·a

    MASS = 3.314
    Kt   = 60.0 / (2.0 * np.pi * 3500.0)          # 0.00273 N·m/A
    G    = 11.82
    r_eff = 0.051
    SLOPE_THEORY = Kt * G / r_eff                  # ≈ 0.632 N/A
    I_LIM = 60.0                                   # VESC current limit (approx)

    # ── Figure with two panels ──────────────────────────────────────────
    fig, (ax_left, ax_right) = plt.subplots(1, 2, figsize=(10, 4.5),
                                            gridspec_kw={'wspace': 0.35})

    # ---------- LEFT: raw scatter, colour = velocity --------------------
    sc = ax_left.scatter(I_all, F_all, c=vx, s=2, alpha=0.25,
                         cmap='viridis', rasterized=True)
    cb = fig.colorbar(sc, ax=ax_left, pad=0.02)
    cb.set_label('Velocity (m/s)')

    # annotate current-limit band
    ax_left.axvspan(I_LIM, I_all.max() + 2, color='red', alpha=0.08)
    ax_left.axvline(I_LIM, color='red', ls='--', lw=0.8, alpha=0.6)
    ax_left.text(I_LIM + 1, ax_left.get_ylim()[1] * 0.85 if ax_left.get_ylim()[1] > 0 else 30,
                 'VESC\ncurrent\nlimit', fontsize=7, color='red', va='top')

    ax_left.set_xlabel('Motor current (A)')
    ax_left.set_ylabel('Net force  $F_{net} = m \\cdot a_x$  (N)')
    ax_left.set_title('Raw scatter (all acceleration data)')
    ax_left.grid(True, alpha=0.3)

    # ---------- RIGHT: binned linear region -----------------------------
    # Keep only the "active acceleration" points (F > 1 N, I between 5–55 A)
    linear_mask = (F_all > 1.0) & (I_all > 5.0) & (I_all < I_LIM)
    I_lin = I_all[linear_mask]
    F_lin = F_all[linear_mask]

    if len(I_lin) > 20:
        # Bin by current
        bin_edges = np.arange(5, 56, 5)
        bin_centres, bin_means, bin_stds, bin_ns = [], [], [], []
        for lo, hi in zip(bin_edges[:-1], bin_edges[1:]):
            m = (I_lin >= lo) & (I_lin < hi)
            if m.sum() >= 3:
                bin_centres.append((lo + hi) / 2)
                bin_means.append(np.mean(F_lin[m]))
                bin_stds.append(np.std(F_lin[m]))
                bin_ns.append(m.sum())

        bin_centres = np.array(bin_centres)
        bin_means   = np.array(bin_means)
        bin_stds    = np.array(bin_stds)

        ax_right.errorbar(bin_centres, bin_means, yerr=bin_stds, fmt='o',
                          capsize=4, color='tab:blue', markersize=7,
                          label='Bin mean ± 1 σ')

        # Linear fit through bin means
        if len(bin_centres) > 1:
            p = np.polyfit(bin_centres, bin_means, 1)
            I_fit = np.linspace(0, 60, 80)
            ax_right.plot(I_fit, np.polyval(p, I_fit), '--', color='tab:blue',
                          alpha=0.7, lw=1.5,
                          label=f'Fit: {p[0]:.3f} N/A  (R²='
                                f'{1 - np.sum((bin_means - np.polyval(p, bin_centres))**2) / np.sum((bin_means - np.mean(bin_means))**2):.2f})')

        # Theoretical K_t line
        ax_right.plot(I_fit, SLOPE_THEORY * I_fit, ':', color='tab:orange',
                      lw=1.5, label=f'Theoretical: {SLOPE_THEORY:.3f} N/A')
    else:
        ax_right.text(0.5, 0.5, 'Too few points\nin linear region',
                      transform=ax_right.transAxes, ha='center', fontsize=10)

    ax_right.set_xlabel('Motor current (A)')
    ax_right.set_ylabel('Net force  $F_{net}$  (N)')
    ax_right.set_title('Linear region (5–55 A, $F>1$ N)')
    ax_right.legend(fontsize=7, loc='upper left')
    ax_right.grid(True, alpha=0.3)
    ax_right.set_xlim(0, 60)
    ax_right.set_ylim(bottom=0)

    fig.suptitle('Motor Torque Mapping — Current vs. Force', fontsize=11, y=1.01)
    fig.tight_layout()
    save_fig(fig, 'torque_I_vs_F.pdf')


def plot_torque_current_time(data_dir, prefix):
    """Figure 7: motor current time series at each speed target.

    Time gaps between speed steps are collapsed so the segments appear
    back-to-back with a small 0.5 s visual separator.
    """
    path = find_latest('motor_torque', data_dir, prefix)
    if not path:
        print('  [skip] no motor_torque raw CSV found')
        return

    d = load_csv(path)
    t_raw   = d['timestamp_s'].astype(float)
    current = d['motor_current'].astype(float)
    phase   = d.get('phase', np.array([''] * len(t_raw)))
    cmd_speed = d.get('cmd_speed', np.zeros_like(t_raw))

    # ── Collapse time gaps ──────────────────────────────────────────────
    GAP_SEP = 0.5                                   # visual gap (seconds)
    dt = np.diff(t_raw)
    gap_idx = np.where(dt > 1.0)[0]                 # detect pauses > 1 s

    # Build a shifted time axis that removes the dead time
    t = t_raw.copy()
    for gi in gap_idx:
        actual_gap = t_raw[gi + 1] - t_raw[gi]
        shift = actual_gap - GAP_SEP               # amount to subtract
        t[gi + 1:] -= shift

    # ── Plot ────────────────────────────────────────────────────────────
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 5), sharex=True,
                                    gridspec_kw={'height_ratios': [2, 1]})

    # Insert NaN at gap positions to break lines
    t_plot   = t.copy()
    cur_plot = current.copy()
    spd_plot = cmd_speed.copy()
    for offset, gi in enumerate(gap_idx):
        idx = gi + 1 + offset
        nan_t = (t_plot[idx - 1] + t_plot[idx]) / 2.0 if idx < len(t_plot) else t_plot[-1]
        t_plot   = np.insert(t_plot,   idx, nan_t)
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

    # Add thin vertical lines at gap boundaries for clarity
    for gi in gap_idx:
        for axi in (ax1, ax2):
            axi.axvline(t[gi] + GAP_SEP * 0.5, color='grey', ls=':',
                        lw=0.6, alpha=0.5)

    fig.tight_layout()
    save_fig(fig, 'torque_current_vs_time.pdf')


def plot_max_dynamics(data_dir, prefix):
    """Figure 8: speed (odom + IMU-integrated) vs time."""
    path = find_latest('max_dynamics', data_dir, prefix)
    if not path:
        print('  [skip] no max_dynamics CSV found')
        return

    d = load_csv(path)
    t = d['timestamp_s']
    vx = d['odom_vx']
    phase = d.get('phase', np.array([''] * len(t)))

    fig, ax = setup_fig(width=7)
    ax.plot(t, vx, '-', label=r'$v_x$ (odometry)', color='tab:blue', linewidth=1.5)

    # Reconstruct IMU-integrated velocity from imu_ax
    # (simple trapezoidal integration for the plot)
    ax_imu = d['imu_ax']
    dt = np.diff(t, prepend=t[0])
    dt[0] = dt[1] if len(dt) > 1 else 0.005

    # Find deceleration start — IMU integration starts there
    decel_mask = phase == 'deceleration'
    accel_mask = phase == 'acceleration'

    if np.any(decel_mask):
        decel_start = np.argmax(decel_mask)
        # Integrate from decel_start using odom speed as initial condition
        v_imu = np.full_like(t, np.nan)
        v_imu[decel_start] = vx[decel_start]
        for i in range(decel_start + 1, len(t)):
            v_imu[i] = max(0, v_imu[i - 1] + ax_imu[i] * dt[i])
        ax.plot(t, v_imu, '-', label=r'$v_x$ (IMU integrated)',
                color='tab:orange', linewidth=1.5)

    # Shade phases
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
    """Figure 9: IMU yaw rate and commanded steering vs time.

    Breaks lines at recording gaps (≈ 2 s between steps) so that
    matplotlib does not draw a misleading diagonal through zero.
    """
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
    gap_idx = np.where(dt > 0.1)[0]   # gaps > 100 ms

    t_plot = t.copy()
    omega_plot = omega.copy()
    cmd_plot = cmd.copy()
    for offset, gi in enumerate(gap_idx):
        idx = gi + 1 + offset
        mid_t = (t_plot[idx - 1] + t_plot[idx]) / 2.0
        t_plot     = np.insert(t_plot,     idx, mid_t)
        omega_plot = np.insert(omega_plot, idx, np.nan)
        cmd_plot   = np.insert(cmd_plot,   idx, np.nan)

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

    # Combined legend
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
    args = parser.parse_args()

    data_dir = args.data_dir
    prefix = args.prefix

    if not os.path.isdir(data_dir):
        print(f'Data directory not found: {data_dir}')
        print('Run the tests first to generate CSV data.')
        sys.exit(1)

    print(f'Data dir: {data_dir}')
    print(f'Output:   {FIG_DIR}/')
    if prefix:
        print(f'Prefix:   {prefix}')
    print()

    plots = [
        ('1/10  Wheelbase circle trajectory', plot_wheelbase),
        ('2/10  Friction a_y vs speed',       plot_friction),
        ('3/10  Cornering F_y vs alpha',      plot_cornering_Fy_alpha),
        ('4/10  Cornering alpha vs speed',    plot_cornering_alpha_speed),
        ('5/10  Longitudinal v comparison',   plot_long_v_comparison),
        ('6/10  Longitudinal F_x vs kappa',   plot_long_Fx_kappa),
        ('7/10  Motor torque I vs F',         plot_torque_I_vs_F),
        ('8/10  Motor torque current vs time',plot_torque_current_time),
        ('9/10  Max dynamics speed vs time',  plot_max_dynamics),
        ('10/10 Steering step response',      plot_steering_rate),
    ]

    for label, func in plots:
        print(f'[{label}]')
        try:
            func(data_dir, prefix)
        except Exception as e:
            print(f'  [ERROR] {e}')

    print(f'\nDone. Figures saved to {FIG_DIR}/')


if __name__ == '__main__':
    main()
