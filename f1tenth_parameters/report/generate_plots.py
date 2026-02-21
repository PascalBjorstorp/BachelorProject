#!/usr/bin/env python3
"""
Generate all report figures from test CSV data.

Usage:
    python3 generate_plots.py                      # auto-detect latest CSVs
    python3 generate_plots.py --data-dir data/     # explicit data directory
    python3 generate_plots.py --prefix 20260301    # filter by date prefix

Produces PDF figures in figures/ matching the report_outline.tex placeholders:
  1. friction_ay_vs_speed.pdf
  2. cornering_Fy_vs_alpha.pdf
  3. cornering_alpha_vs_speed.pdf
  4. long_v_comparison.pdf
  5. long_Fx_vs_kappa.pdf
  6. torque_I_vs_F.pdf
  7. torque_current_vs_time.pdf
  8. max_dyn_speed_vs_time.pdf
  9. steering_step_response.pdf
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


# ── Plot functions ───────────────────────────────────────────────────────────

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
        ax.legend()
    else:
        # Fall back to raw CSV
        d = load_csv(path)
        t = d['timestamp_s']
        ay = d['imu_ay']
        ax.plot(t, ay, '.', markersize=1, alpha=0.3, label=r'$a_y$ (IMU)')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel(r'$a_y$ (m/s²)')
        ax.set_title('Friction Test — Raw IMU Data')
        ax.legend()

    save_fig(fig, 'friction_ay_vs_speed.pdf')


def plot_cornering_Fy_alpha(data_dir, prefix):
    """Figure 2: F_y vs slip angle for front and rear."""
    summary = find_latest('cornering_stiffness_summary', data_dir, prefix)
    if not summary:
        print('  [skip] no cornering_stiffness_summary CSV found')
        return

    d = load_csv(summary)
    alpha_f = d['alpha_f_deg']
    alpha_r = d['alpha_r_deg']
    Fyf = d['F_yf']
    Fyr = d['F_yr']

    fig, ax = setup_fig()

    # Front
    ax.errorbar(np.abs(alpha_f), np.abs(Fyf),
                xerr=np.degrees(d.get('sigma_alpha_f', np.zeros_like(alpha_f))),
                yerr=d.get('sigma_Fyf', np.zeros_like(Fyf)),
                fmt='o-', capsize=3, label='Front', color='tab:blue')
    # Rear
    ax.errorbar(np.abs(alpha_r), np.abs(Fyr),
                xerr=np.degrees(d.get('sigma_alpha_r', np.zeros_like(alpha_r))),
                yerr=d.get('sigma_Fyr', np.zeros_like(Fyr)),
                fmt='s-', capsize=3, label='Rear', color='tab:orange')

    # Linear fits through origin
    for alpha, Fy, color, name in [(alpha_f, Fyf, 'tab:blue', 'front'),
                                    (alpha_r, Fyr, 'tab:orange', 'rear')]:
        a_rad = np.abs(np.radians(alpha))  # convert back to rad for C_alpha
        f = np.abs(Fy)
        mask = a_rad > 0.001
        if np.sum(mask) > 1:
            C = np.sum(f[mask] * a_rad[mask]) / np.sum(a_rad[mask] ** 2)
            a_fit = np.linspace(0, np.max(np.abs(alpha)) * 1.05, 50)
            ax.plot(a_fit, C * np.radians(a_fit), '--', color=color, alpha=0.5,
                    label=rf'$C_{{\alpha,\mathrm{{{name}}}}}$ = {C:.0f} N/rad')

    ax.set_xlabel(r'Slip angle $|\alpha|$ (deg)')
    ax.set_ylabel(r'Lateral force $|F_y|$ (N)')
    ax.set_title('Cornering Stiffness — Force vs. Slip Angle')
    ax.legend()
    save_fig(fig, 'cornering_Fy_vs_alpha.pdf')


def plot_cornering_alpha_speed(data_dir, prefix):
    """Figure 3: slip angles vs speed."""
    summary = find_latest('cornering_stiffness_summary', data_dir, prefix)
    if not summary:
        print('  [skip] no cornering_stiffness_summary CSV found')
        return

    d = load_csv(summary)
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
    """Figure 4: v_wheel vs v_body (IMU) over time."""
    path = find_latest('longitudinal_stiffness', data_dir, prefix)
    if not path:
        print('  [skip] no longitudinal_stiffness CSV found')
        return

    d = load_csv(path)
    t = d['timestamp_s']
    vw = d['odom_vx']
    vi = d['v_imu']
    phase = d.get('phase', np.array([''] * len(t)))

    fig, ax = setup_fig(width=7)
    ax.plot(t, vw, '-', label=r'$v_\mathrm{wheel}$ (ERPM)', color='tab:blue', alpha=0.8)
    ax.plot(t, vi, '-', label=r'$v_\mathrm{body}$ (IMU)', color='tab:orange', alpha=0.8)

    # Shade acceleration vs braking
    if hasattr(phase, '__len__') and len(phase) > 0:
        accel_mask = phase == 'acceleration'
        brake_mask = phase == 'braking'
        if np.any(accel_mask):
            t_a = t[accel_mask]
            ax.axvspan(t_a[0], t_a[-1], alpha=0.08, color='green', label='Accel phase')
        if np.any(brake_mask):
            t_b = t[brake_mask]
            ax.axvspan(t_b[0], t_b[-1], alpha=0.08, color='red', label='Brake phase')

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Velocity (m/s)')
    ax.set_title('Wheel Speed vs. Body Speed — Longitudinal Slip')
    ax.legend(loc='best')
    save_fig(fig, 'long_v_comparison.pdf')


def plot_long_Fx_kappa(data_dir, prefix):
    """Figure 5: F_x vs slip ratio with linear fit."""
    path = find_latest('longitudinal_stiffness', data_dir, prefix)
    if not path:
        print('  [skip] no longitudinal_stiffness CSV found')
        return

    d = load_csv(path)
    kappa = d['slip_ratio']
    ax_imu = d['imu_ax']
    phase = d.get('phase', np.array([''] * len(kappa)))

    # F_x = m * a_x (mass from vehicle_params or default)
    mass = 3.314
    Fx = mass * ax_imu

    fig, ax = setup_fig()

    # Color by phase
    accel_mask = phase == 'acceleration'
    brake_mask = phase == 'braking'
    if np.any(accel_mask):
        ax.scatter(kappa[accel_mask], Fx[accel_mask], s=4, alpha=0.3,
                   color='tab:blue', label='Acceleration')
    if np.any(brake_mask):
        ax.scatter(kappa[brake_mask], Fx[brake_mask], s=4, alpha=0.3,
                   color='tab:red', label='Braking')

    # Linear fit in |kappa| < 0.15
    linear_mask = (np.abs(kappa) > 0.005) & (np.abs(kappa) < 0.15)
    if np.sum(linear_mask) > 5:
        k_lin = kappa[linear_mask]
        f_lin = Fx[linear_mask]
        C_x = np.sum(f_lin * k_lin) / np.sum(k_lin ** 2)
        k_fit = np.linspace(np.min(kappa), np.max(kappa), 100)
        ax.plot(k_fit, C_x * k_fit, 'k--', linewidth=2,
                label=rf'$C_x = {C_x:.0f}$ N (linear fit)')

    ax.set_xlabel(r'Slip ratio $\kappa$')
    ax.set_ylabel(r'Longitudinal force $F_x$ (N)')
    ax.set_title(r'Longitudinal Tire Stiffness — $F_x$ vs. $\kappa$')
    ax.legend()
    save_fig(fig, 'long_Fx_vs_kappa.pdf')


def plot_torque_I_vs_F(data_dir, prefix):
    """Figure 6: motor current vs net force (scatter + linear fit)."""
    summary = find_latest('motor_torque_summary', data_dir, prefix)
    raw = find_latest('motor_torque', data_dir, prefix)

    if not summary and not raw:
        print('  [skip] no motor_torque CSV found')
        return

    fig, ax = setup_fig()

    if summary and os.path.exists(summary):
        d = load_csv(summary)
        # Acceleration points
        accel = d.get('phase', np.array([])) == 'acceleration'
        brake = d.get('phase', np.array([])) == 'braking'

        if np.any(accel):
            I_a = d['avg_current'][accel]
            F_a = d['F_net'][accel]
            F_std = d.get('F_net_std', np.zeros_like(F_a))
            if isinstance(F_std[0], str):
                F_std = np.zeros_like(F_a)
            else:
                F_std = F_std[accel]
            ax.errorbar(I_a, F_a, yerr=F_std, fmt='o', capsize=4,
                        color='tab:blue', label='Accel', markersize=8)

            # Linear fit
            if len(I_a) > 1:
                p = np.polyfit(I_a, F_a, 1)
                I_fit = np.linspace(0, np.max(I_a) * 1.1, 50)
                ax.plot(I_fit, np.polyval(p, I_fit), '--', color='tab:blue',
                        alpha=0.6, label=rf'Fit: {p[0]:.3f} N/A + {p[1]:.1f} N')

        if np.any(brake):
            I_b = d['avg_current'][brake]
            F_b = d['F_net'][brake]
            ax.plot(I_b, F_b, 's', color='tab:red', label='Braking', markersize=8)

    elif raw:
        d = load_csv(raw)
        phase = d.get('phase', np.array([]))
        accel_mask = phase == 'acceleration'
        if np.any(accel_mask):
            I = d['motor_current'][accel_mask]
            ax_imu = d['imu_ax'][accel_mask]
            F = 3.314 * ax_imu
            ax.scatter(I, F, s=2, alpha=0.2, color='tab:blue')

    ax.set_xlabel('Motor current (A)')
    ax.set_ylabel('Net force $F_{net} = m \\cdot a_x$ (N)')
    ax.set_title('Motor Torque Mapping — Current vs. Force')
    ax.legend()
    save_fig(fig, 'torque_I_vs_F.pdf')


def plot_torque_current_time(data_dir, prefix):
    """Figure 7: motor current time series at each speed target."""
    path = find_latest('motor_torque', data_dir, prefix)
    if not path:
        print('  [skip] no motor_torque raw CSV found')
        return

    d = load_csv(path)
    t = d['timestamp_s']
    current = d['motor_current']
    phase = d.get('phase', np.array([''] * len(t)))
    cmd_speed = d.get('cmd_speed', np.zeros_like(t))

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7, 5), sharex=True,
                                    gridspec_kw={'height_ratios': [2, 1]})

    ax1.plot(t, current, '-', linewidth=0.5, color='tab:blue', alpha=0.7)
    ax1.set_ylabel('Motor current (A)')
    ax1.set_title('Motor Current vs. Time')
    ax1.grid(True, alpha=0.3)

    # Shade acceleration vs braking
    accel_mask = phase == 'acceleration'
    brake_mask = phase == 'braking'
    if np.any(accel_mask):
        ta = t[accel_mask]
        # Find contiguous acceleration segments
        diffs = np.diff(np.where(accel_mask)[0])
        splits = np.where(diffs > 5)[0] + 1
        segments = np.split(np.where(accel_mask)[0], splits)
        for seg in segments:
            if len(seg) > 0:
                ax1.axvspan(t[seg[0]], t[seg[-1]], alpha=0.08, color='green')
    if np.any(brake_mask):
        tb = t[brake_mask]
        diffs = np.diff(np.where(brake_mask)[0])
        splits = np.where(diffs > 5)[0] + 1
        segments = np.split(np.where(brake_mask)[0], splits)
        for seg in segments:
            if len(seg) > 0:
                ax1.axvspan(t[seg[0]], t[seg[-1]], alpha=0.08, color='red')

    ax2.plot(t, cmd_speed, '-', color='tab:green', linewidth=1)
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Cmd speed (m/s)')
    ax2.grid(True, alpha=0.3)

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
    """Figure 9: IMU yaw rate and commanded steering vs time."""
    path = find_latest('steering_rate', data_dir, prefix)
    if not path:
        print('  [skip] no steering_rate CSV found')
        return

    d = load_csv(path)
    t = d['timestamp_s']
    omega = d['imu_gz']
    cmd = d['cmd_steering']

    fig, ax1 = setup_fig(width=7)
    color_omega = 'tab:blue'
    color_cmd = 'tab:red'

    ax1.plot(t, np.degrees(omega), '-', color=color_omega, linewidth=0.8,
             label=r'$\omega_z$ (IMU)')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel(r'Yaw rate $\omega_z$ (deg/s)', color=color_omega)
    ax1.tick_params(axis='y', labelcolor=color_omega)

    ax2 = ax1.twinx()
    ax2.plot(t, np.degrees(cmd), '--', color=color_cmd, linewidth=1.2,
             label=r'$\delta_{cmd}$ (commanded)')
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
        ('1/9  Friction a_y vs speed',      plot_friction),
        ('2/9  Cornering F_y vs alpha',      plot_cornering_Fy_alpha),
        ('3/9  Cornering alpha vs speed',    plot_cornering_alpha_speed),
        ('4/9  Longitudinal v comparison',   plot_long_v_comparison),
        ('5/9  Longitudinal F_x vs kappa',   plot_long_Fx_kappa),
        ('6/9  Motor torque I vs F',         plot_torque_I_vs_F),
        ('7/9  Motor torque current vs time',plot_torque_current_time),
        ('8/9  Max dynamics speed vs time',  plot_max_dynamics),
        ('9/9  Steering step response',      plot_steering_rate),
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
