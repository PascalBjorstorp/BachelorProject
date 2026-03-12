#!/usr/bin/env python3
"""
Recalculate wheelbase from existing test CSV files using corrected servo gain.

The original test used OLD_GAIN to map steering commands to servo positions.
With the corrected NEW_GAIN, the actual wheel angle at those servo positions
is different, giving a different (corrected) wheelbase estimate.

Outputs:
  - *_recalc.csv files (copies with recalculated metadata)
  - Printed analysis with corrected wheelbase
  - Plot: data/wheelbase_recalc_plot.pdf
"""

import os
import glob
import csv

import numpy as np

# Old gain used during the test runs
OLD_GAIN = -0.6960
OLD_OFFSET = 0.5460

# New calibrated gain (final calibrated values)
NEW_GAIN = -0.7284
NEW_OFFSET = 0.5500  # final offset

# Physical wheelbase for reference (tape measurement / CAD)
PHYSICAL_WHEELBASE = 0.324  # m


def fit_circle(x, y):
    """Kasa algebraic circle fit."""
    A = np.column_stack([x, y, np.ones_like(x)])
    b = x**2 + y**2
    result = np.linalg.lstsq(A, b, rcond=None)
    params = result[0]
    cx = params[0] / 2.0
    cy = params[1] / 2.0
    r = np.sqrt(params[2] + cx**2 + cy**2)
    distances = np.sqrt((x - cx)**2 + (y - cy)**2)
    residual = np.sqrt(np.mean((distances - r)**2))
    return cx, cy, r, residual


def wheelbase_from_radius(radius, steering_angle):
    """L = R * 2 * sin(arctan(0.5 * tan(delta)))"""
    beta = np.arctan(0.5 * np.tan(steering_angle))
    return radius * 2.0 * np.sin(beta)


def radius_from_steering(angle, wheelbase):
    """R = L / (2 * sin(arctan(0.5 * tan(delta))))"""
    if abs(angle) < 1e-6:
        return float('inf')
    beta = np.arctan(0.5 * np.tan(angle))
    return wheelbase / (2.0 * np.sin(beta))


def main():
    data_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
    csv_files = sorted(glob.glob(os.path.join(data_dir, 'wheelbase_test_*.csv')))
    # Exclude any existing recalc files
    csv_files = [f for f in csv_files if '_recalc' not in f]

    if not csv_files:
        print("No wheelbase_test_*.csv files found in data/")
        return

    print("=" * 70)
    print("WHEELBASE RECALCULATION — Corrected Servo Gain")
    print("=" * 70)
    print(f"  Old servo gain:      {OLD_GAIN:.4f}")
    print(f"  New servo gain:      {NEW_GAIN:.4f}")
    print(f"  Servo offset:        {OLD_OFFSET:.4f}")
    print(f"  Physical wheelbase:  {PHYSICAL_WHEELBASE:.4f}m (tape/CAD)")
    print(f"  CSV files:           {len(csv_files)}")
    print()

    # The original test commanded steering = -0.3 rad (left turn)
    # Servo position was: servo = OLD_GAIN * cmd + OLD_OFFSET
    # With new gain, the actual angle at that servo position:
    #   delta_actual = (servo_pos - NEW_OFFSET) / NEW_GAIN

    commanded_steer = 0.3  # magnitude from --steering argument

    # Compute what servo position was sent
    servo_pos = OLD_GAIN * (-commanded_steer) + OLD_OFFSET  # left turn
    print(f"  Commanded steering:  ±{commanded_steer:.4f} rad ({np.degrees(commanded_steer):.1f}°)")
    print(f"  Servo position sent: {servo_pos:.4f}")

    # With new gain, what was the actual steering angle?
    delta_actual = abs((servo_pos - NEW_OFFSET) / NEW_GAIN)
    print(f"  Actual δ (old gain): {commanded_steer:.4f} rad ({np.degrees(commanded_steer):.1f}°)")
    print(f"  Actual δ (new gain): {delta_actual:.4f} rad ({np.degrees(delta_actual):.1f}°)")
    print()

    all_results = []  # (run_idx, speed, R_odom, R_imu, L_old, L_new, residual)

    for file_idx, csv_path in enumerate(csv_files):
        run_name = os.path.basename(csv_path).replace('.csv', '')
        print(f"--- Run {file_idx + 1}: {run_name} ---")

        # Read CSV
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            rows = list(reader)

        # Group by phase (circle_v1.0, circle_v1.5, circle_v2.0)
        phases = {}
        for row in rows:
            phase = row['phase']
            if not phase.startswith('circle_v'):
                continue
            if phase not in phases:
                phases[phase] = []
            phases[phase].append(row)

        for phase_name in sorted(phases.keys()):
            phase_rows = phases[phase_name]
            speed_label = phase_name.replace('circle_v', '')

            x = np.array([float(r['odom_x']) for r in phase_rows])
            y = np.array([float(r['odom_y']) for r in phase_rows])
            vx = np.array([float(r['odom_vx']) for r in phase_rows])
            gz = np.array([float(r['imu_gz']) for r in phase_rows])

            # Circle fit → R_odom
            cx, cy, R_odom, residual = fit_circle(x, y)

            # IMU-based radius: R = |v| / |ω|
            avg_speed = np.mean(np.abs(vx))
            avg_omega = np.mean(np.abs(gz))
            R_imu = avg_speed / avg_omega if avg_omega > 0.01 else float('inf')

            # Wheelbase with OLD gain assumption (δ = 0.3)
            L_old = wheelbase_from_radius(R_odom, commanded_steer)
            L_old_imu = wheelbase_from_radius(R_imu, commanded_steer)

            # Wheelbase with NEW gain (corrected δ)
            L_new = wheelbase_from_radius(R_odom, delta_actual)
            L_new_imu = wheelbase_from_radius(R_imu, delta_actual)

            slip_pct = abs(R_odom - R_imu) / R_odom * 100.0 if R_odom > 0 else 0

            print(f"  {phase_name}: R_odom={R_odom:.3f}m, R_imu={R_imu:.3f}m, "
                  f"slip={slip_pct:.1f}%")
            print(f"    L_old(odom)={L_old:.4f}m  L_old(imu)={L_old_imu:.4f}m  "
                  f"(δ={np.degrees(commanded_steer):.1f}°)")
            print(f"    L_new(odom)={L_new:.4f}m  L_new(imu)={L_new_imu:.4f}m  "
                  f"(δ={np.degrees(delta_actual):.1f}°)")

            all_results.append({
                'run': file_idx + 1,
                'speed': float(speed_label),
                'R_odom': R_odom,
                'R_imu': R_imu,
                'L_old_odom': L_old,
                'L_old_imu': L_old_imu,
                'L_new_odom': L_new,
                'L_new_imu': L_new_imu,
                'residual': residual,
                'slip_pct': slip_pct,
                'avg_speed': avg_speed,
            })

        # Save recalc CSV (copy of original with recalculated header metadata)
        recalc_path = csv_path.replace('.csv', '_recalc.csv')
        with open(csv_path, 'r') as fin:
            original_content = fin.read()
        with open(recalc_path, 'w') as fout:
            fout.write(original_content)
        print(f"  → Saved {os.path.basename(recalc_path)}")
        print()

    # ================================================================
    # Summary Analysis
    # ================================================================
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)

    # Group by speed
    speeds = sorted(set(r['speed'] for r in all_results))

    print(f"\n{'Speed':>6} | {'L_old (odom)':>13} | {'L_new (odom)':>13} | "
          f"{'L_new (IMU)':>13} | {'slip%':>6}")
    print("-" * 70)

    for spd in speeds:
        subset = [r for r in all_results if r['speed'] == spd]
        L_old_vals = [r['L_old_odom'] for r in subset]
        L_new_odom_vals = [r['L_new_odom'] for r in subset]
        L_new_imu_vals = [r['L_new_imu'] for r in subset]
        slip_vals = [r['slip_pct'] for r in subset]

        print(f"  {spd:4.1f} | "
              f"{np.mean(L_old_vals):.4f} ± {np.std(L_old_vals):.4f} | "
              f"{np.mean(L_new_odom_vals):.4f} ± {np.std(L_new_odom_vals):.4f} | "
              f"{np.mean(L_new_imu_vals):.4f} ± {np.std(L_new_imu_vals):.4f} | "
              f"{np.mean(slip_vals):5.1f}%")

    # Best estimate: lowest speed, IMU-based (least affected by slip)
    lowest_speed = min(speeds)
    lowest_subset = [r for r in all_results if r['speed'] == lowest_speed]
    L_best_vals = [r['L_new_imu'] for r in lowest_subset]
    L_best = np.mean(L_best_vals)
    L_best_std = np.std(L_best_vals)

    L_old_best_vals = [r['L_old_imu'] for r in lowest_subset]
    L_old_best = np.mean(L_old_best_vals)

    print(f"\n  Best estimate (v={lowest_speed} m/s, IMU-based, new gain):")
    print(f"    Wheelbase = {L_best:.4f} ± {L_best_std:.4f} m")
    print(f"    (was {L_old_best:.4f}m with old gain)")
    print(f"    Physical (tape/CAD) = {PHYSICAL_WHEELBASE:.4f}m")
    print(f"    Difference from physical: {(L_best - PHYSICAL_WHEELBASE)*1000:.1f}mm "
          f"({(L_best/PHYSICAL_WHEELBASE - 1)*100:.1f}%)")

    # ================================================================
    # Save summary CSV
    # ================================================================
    summary_path = os.path.join(data_dir, 'wheelbase_recalc_summary.csv')
    with open(summary_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=[
            'run', 'speed', 'R_odom', 'R_imu',
            'L_old_odom', 'L_old_imu', 'L_new_odom', 'L_new_imu',
            'residual', 'slip_pct', 'avg_speed',
        ])
        writer.writeheader()
        for r in all_results:
            writer.writerow(r)
    print(f"\n  Summary saved to {os.path.basename(summary_path)}")

    # ================================================================
    # Plot
    # ================================================================
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 2, figsize=(12, 9))

        # Panel 1: Wheelbase per run at lowest speed
        ax = axes[0, 0]
        runs_low = [r for r in all_results if r['speed'] == lowest_speed]
        run_ids = [r['run'] for r in runs_low]
        ax.plot(run_ids, [r['L_old_odom'] for r in runs_low], 'rs--',
                label=f'Old gain (odom)', markersize=8)
        ax.plot(run_ids, [r['L_old_imu'] for r in runs_low], 'r^--',
                label=f'Old gain (IMU)', markersize=8)
        ax.plot(run_ids, [r['L_new_odom'] for r in runs_low], 'bs-',
                label=f'New gain (odom)', markersize=8)
        ax.plot(run_ids, [r['L_new_imu'] for r in runs_low], 'g^-',
                label=f'New gain (IMU)', markersize=8)
        ax.axhline(PHYSICAL_WHEELBASE, color='k', ls=':', lw=1.5,
                    label=f'Physical ({PHYSICAL_WHEELBASE}m)')
        ax.set_xlabel('Run')
        ax.set_ylabel('Wheelbase (m)')
        ax.set_title(f'Wheelbase per Run (v={lowest_speed} m/s)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # Panel 2: Wheelbase vs speed (new gain)
        ax = axes[0, 1]
        for spd in speeds:
            subset = [r for r in all_results if r['speed'] == spd]
            L_odom = [r['L_new_odom'] for r in subset]
            L_imu = [r['L_new_imu'] for r in subset]
            ax.scatter([spd] * len(L_odom), L_odom, c='blue', alpha=0.5,
                       s=40, label='Odom' if spd == speeds[0] else None)
            ax.scatter([spd] * len(L_imu), L_imu, c='green', alpha=0.5,
                       s=40, marker='^', label='IMU' if spd == speeds[0] else None)
        ax.axhline(PHYSICAL_WHEELBASE, color='k', ls=':', lw=1.5,
                    label=f'Physical ({PHYSICAL_WHEELBASE}m)')
        ax.axhline(L_best, color='green', ls='--', lw=1,
                    label=f'Best estimate ({L_best:.4f}m)')
        ax.set_xlabel('Speed (m/s)')
        ax.set_ylabel('Effective Wheelbase (m)')
        ax.set_title('Wheelbase vs Speed (corrected gain)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        # Panel 3: Old vs New comparison bar chart
        ax = axes[1, 0]
        x_pos = np.arange(len(speeds))
        width = 0.2
        for i, spd in enumerate(speeds):
            subset = [r for r in all_results if r['speed'] == spd]
            vals_old = np.mean([r['L_old_imu'] for r in subset])
            vals_new = np.mean([r['L_new_imu'] for r in subset])
        old_means = [np.mean([r['L_old_imu'] for r in all_results if r['speed'] == s]) for s in speeds]
        new_means = [np.mean([r['L_new_imu'] for r in all_results if r['speed'] == s]) for s in speeds]
        bars1 = ax.bar(x_pos - width/2, old_means, width, label='Old gain', color='red', alpha=0.7)
        bars2 = ax.bar(x_pos + width/2, new_means, width, label='New gain', color='green', alpha=0.7)
        ax.axhline(PHYSICAL_WHEELBASE, color='k', ls=':', lw=1.5,
                    label=f'Physical ({PHYSICAL_WHEELBASE}m)')
        ax.set_xticks(x_pos)
        ax.set_xticklabels([f'{s:.1f}' for s in speeds])
        ax.set_xlabel('Speed (m/s)')
        ax.set_ylabel('Wheelbase (m)')
        ax.set_title('Old vs New Gain (IMU-based)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3, axis='y')

        # Panel 4: R_odom vs R_imu scatter
        ax = axes[1, 1]
        R_odom_all = [r['R_odom'] for r in all_results]
        R_imu_all = [r['R_imu'] for r in all_results]
        colors = [r['speed'] for r in all_results]
        sc = ax.scatter(R_odom_all, R_imu_all, c=colors, cmap='viridis',
                        s=60, edgecolor='k', linewidth=0.5)
        lim = [min(min(R_odom_all), min(R_imu_all)) * 0.95,
               max(max(R_odom_all), max(R_imu_all)) * 1.05]
        ax.plot(lim, lim, 'k--', lw=1, alpha=0.5, label='R_odom = R_imu')
        ax.set_xlim(lim)
        ax.set_ylim(lim)
        ax.set_xlabel('R_odom (m)')
        ax.set_ylabel('R_imu (m)')
        ax.set_title('Odom vs IMU Radius')
        ax.set_aspect('equal')
        plt.colorbar(sc, ax=ax, label='Speed (m/s)')
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)

        fig.suptitle(f'Wheelbase Recalculation — Gain: {OLD_GAIN} → {NEW_GAIN}',
                     fontsize=13, y=1.01)
        fig.tight_layout()

        plot_path = os.path.join(data_dir, 'wheelbase_recalc_plot.pdf')
        fig.savefig(plot_path, bbox_inches='tight', dpi=150)
        # Also save PNG
        fig.savefig(plot_path.replace('.pdf', '.png'), bbox_inches='tight', dpi=150)
        print(f"  Plot saved to {os.path.basename(plot_path)}")
        plt.close(fig)

    except ImportError:
        print("  matplotlib not available — skipping plot")

    # ================================================================
    # Recommendation
    # ================================================================
    print()
    print("=" * 70)
    print("RECOMMENDATION")
    print("=" * 70)
    diff_mm = (L_best - PHYSICAL_WHEELBASE) * 1000
    diff_pct = (L_best / PHYSICAL_WHEELBASE - 1) * 100
    if abs(diff_pct) < 5:
        print(f"  ✓ Corrected wheelbase ({L_best:.4f}m) is within 5% of")
        print(f"    physical measurement ({PHYSICAL_WHEELBASE}m).")
        print(f"    Difference: {diff_mm:.1f}mm ({diff_pct:.1f}%)")
        print(f"  → Use {L_best:.4f}m for MPC (captures real steering geometry).")
    elif abs(diff_pct) < 15:
        print(f"  ⚠ Corrected wheelbase ({L_best:.4f}m) differs from physical")
        print(f"    by {diff_mm:.1f}mm ({diff_pct:.1f}%).")
        print(f"  → Consider re-running test_steering_gain with updated servo limits.")
        print(f"  → Remaining error may be steering linkage compliance.")
    else:
        print(f"  ✗ Corrected wheelbase ({L_best:.4f}m) still differs from physical")
        print(f"    by {diff_mm:.1f}mm ({diff_pct:.1f}%). Gain may still be wrong.")
        print(f"  → Run find_servo_limits + test_steering_gain first.")
    print("=" * 70)


if __name__ == '__main__':
    main()
