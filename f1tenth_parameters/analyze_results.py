#!/usr/bin/env python3
"""
Combined Analysis and Parameter Export

Loads test data from the data/ directory and produces:
1. Summary of all identified parameters
2. YAML snippet for vesc.yaml
3. C header snippet for mpc_types.h

Usage:
    python3 analyze_results.py [--data-dir data/]
"""

import argparse
import glob
import os
import sys

import numpy as np

from common import (
    load_csv, fit_circle, steering_angle_from_radius,
    DEFAULT_WHEELBASE, DEFAULT_SERVO_GAIN, DEFAULT_SERVO_OFFSET,
    DEFAULT_ERPM_GAIN, DEFAULT_MAX_STEER, DATA_DIR
)

GRAVITY = 9.81


def find_latest_file(data_dir: str, prefix: str) -> str:
    """Find the most recent CSV file with the given prefix."""
    pattern = os.path.join(data_dir, f'{prefix}_*.csv')
    files = sorted(glob.glob(pattern))
    if not files:
        return None
    return files[-1]  # Most recent (sorted by timestamp in filename)


def analyze_steering_calibration(filepath: str) -> dict:
    """Analyze steering calibration data."""
    data = load_csv(filepath)
    if not data:
        return {}
    
    print(f"\n  Steering calibration: {os.path.basename(filepath)}")
    print(f"    {len(data.get('timestamp_s', []))} data points")
    
    # Group by servo position phase
    # Full analysis is done by the test script itself
    # Here we just report the file exists
    return {'steering_calibration_file': filepath}


def analyze_speed_calibration(filepath: str) -> dict:
    """Analyze speed calibration data."""
    data = load_csv(filepath)
    if not data:
        return {}
    
    print(f"\n  Speed calibration: {os.path.basename(filepath)}")
    print(f"    {len(data.get('timestamp_s', []))} data points")
    
    return {'speed_calibration_file': filepath}


def analyze_circle_test(filepath: str, steering_angle: float = 0.3) -> dict:
    """Analyze circle test data for wheelbase and cornering stiffness."""
    data = load_csv(filepath)
    if not data:
        return {}
    
    print(f"\n  Circle test: {os.path.basename(filepath)}")
    
    results = {}
    circle_data = []  # (speed, radius) pairs
    
    # Group data by phase
    phases = np.unique(data.get('phase', []))
    
    for phase in phases:
        if not str(phase).startswith('circle_v'):
            continue
        
        mask = data['phase'] == phase
        x = data['odom_x'][mask]
        y = data['odom_y'][mask]
        
        if len(x) > 20:
            cx, cy, r, res = fit_circle(x, y)
            speed = np.mean(data['odom_vx'][mask])
            circle_data.append((speed, r))
            print(f"    Phase {phase}: R={r:.3f}m, v={speed:.2f} m/s, residual={res:.4f}m")
    
    if circle_data:
        # Extract wheelbase from lowest-speed circle: L ≈ R * tan(δ)
        lowest = min(circle_data, key=lambda x: x[0])
        measured_wheelbase = lowest[1] * np.tan(steering_angle)
        results['wheelbase'] = float(measured_wheelbase)
        print(f"    Measured wheelbase: {measured_wheelbase:.4f}m (from v={lowest[0]:.2f} m/s)")
        
        # Max steer from steering calibration or use default
        results['max_steer'] = float(steering_angle_from_radius(
            min(r for _, r in circle_data), DEFAULT_WHEELBASE))
    
    return results


def analyze_max_dynamics(filepath: str) -> dict:
    """Analyze max dynamics data."""
    data = load_csv(filepath)
    if not data:
        return {}
    
    print(f"\n  Max dynamics: {os.path.basename(filepath)}")
    
    results = {}
    
    # Acceleration phase
    mask_accel = data.get('phase', np.array([])) == 'acceleration'
    if np.any(mask_accel):
        speeds = data['odom_vx'][mask_accel]
        results['max_velocity'] = float(np.max(speeds))
        
        # Compute max acceleration
        times = data['timestamp_s'][mask_accel]
        if len(speeds) > 5:
            dv = np.diff(speeds)
            dt = np.diff(times)
            valid = dt > 0
            if np.any(valid):
                accels = dv[valid] / dt[valid]
                # Smooth
                if len(accels) > 5:
                    kernel = 5
                    accels_smooth = np.convolve(accels, np.ones(kernel)/kernel, mode='valid')
                    results['max_acceleration'] = float(np.max(accels_smooth))
                else:
                    results['max_acceleration'] = float(np.max(accels))
        
        print(f"    Max velocity: {results.get('max_velocity', 0):.2f} m/s")
        print(f"    Max acceleration: {results.get('max_acceleration', 0):.2f} m/s²")
    
    # Deceleration phase
    mask_decel = data.get('phase', np.array([])) == 'deceleration'
    if np.any(mask_decel):
        speeds = data['odom_vx'][mask_decel]
        times = data['timestamp_s'][mask_decel]
        if len(speeds) > 5:
            dv = np.diff(speeds)
            dt = np.diff(times)
            valid = dt > 0
            if np.any(valid):
                decels = dv[valid] / dt[valid]
                if len(decels) > 5:
                    kernel = 5
                    decels_smooth = np.convolve(decels, np.ones(kernel)/kernel, mode='valid')
                    results['max_deceleration'] = float(abs(np.min(decels_smooth)))
                else:
                    results['max_deceleration'] = float(abs(np.min(decels)))
        
        print(f"    Max deceleration: {results.get('max_deceleration', 0):.2f} m/s²")
    
    return results


def analyze_friction(filepath: str) -> dict:
    """Analyze friction test data."""
    data = load_csv(filepath)
    if not data:
        return {}
    
    print(f"\n  Friction test: {os.path.basename(filepath)}")
    
    results = {}
    
    ay = data.get('imu_ay', np.array([]))
    if len(ay) > 0:
        max_ay = np.max(np.abs(ay))
        mu = max_ay / GRAVITY
        results['friction_coefficient'] = float(mu)
        results['max_lateral_accel'] = float(max_ay)
        print(f"    Max lateral accel: {max_ay:.3f} m/s² ({mu:.3f} g)")
        print(f"    Friction coefficient μ: {mu:.3f}")
    
    return results


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Parameter Analysis')
    parser.add_argument('--data-dir', type=str, default=DATA_DIR,
                        help=f'Directory containing test data (default: {DATA_DIR})')
    args = parser.parse_args()
    
    if not os.path.exists(args.data_dir):
        print(f"Data directory not found: {args.data_dir}")
        print("Run the test scripts first to generate data.")
        sys.exit(1)
    
    csv_files = glob.glob(os.path.join(args.data_dir, '*.csv'))
    if not csv_files:
        print(f"No CSV files found in {args.data_dir}")
        print("Run the test scripts first to generate data.")
        sys.exit(1)
    
    print("=" * 70)
    print("F1/10th PARAMETER IDENTIFICATION - COMBINED ANALYSIS")
    print("=" * 70)
    print(f"Data directory: {args.data_dir}")
    print(f"Found {len(csv_files)} data files")
    
    # Collect all results
    all_results = {}
    
    # Analyze each test type
    tests = [
        ('steering_calibration', analyze_steering_calibration),
        ('speed_calibration', analyze_speed_calibration),
        ('circle_test', analyze_circle_test),
        ('max_dynamics', analyze_max_dynamics),
        ('friction_test', analyze_friction),
    ]
    
    for test_name, analyze_fn in tests:
        filepath = find_latest_file(args.data_dir, test_name)
        if filepath:
            results = analyze_fn(filepath)
            all_results.update(results)
        else:
            print(f"\n  {test_name}: No data found (run test_{test_name}.py)")
    
    # ---- Generate parameter summary ----
    print("\n" + "=" * 70)
    print("IDENTIFIED PARAMETERS")
    print("=" * 70)
    
    # Use measured values where available, defaults otherwise
    wheelbase = all_results.get('wheelbase', DEFAULT_WHEELBASE)
    max_steer = all_results.get('max_steer', DEFAULT_MAX_STEER)
    max_velocity = all_results.get('max_velocity', 20.0)
    mu = all_results.get('friction_coefficient', 1.0)
    
    params = {
        'wheelbase': wheelbase,
        'max_steer': max_steer,
        'max_velocity': max_velocity,
        'friction_coefficient': mu,
        'max_acceleration': all_results.get('max_acceleration', 0),
        'max_deceleration': all_results.get('max_deceleration', 0),
    }
    
    for name, value in params.items():
        status = "MEASURED" if name in all_results else "DEFAULT"
        print(f"  {name:30s} = {value:10.4f}  [{status}]")
    
    # ---- Generate YAML snippet ----
    print("\n" + "-" * 70)
    print("VESC.YAML SNIPPET (copy to f1tenth_stack/config/vesc.yaml)")
    print("-" * 70)
    print(f"""
    # Calibrated parameters
    speed_to_erpm_gain: {DEFAULT_ERPM_GAIN:.1f}
    steering_angle_to_servo_gain: {DEFAULT_SERVO_GAIN:.4f}
    steering_angle_to_servo_offset: {DEFAULT_SERVO_OFFSET:.4f}
    wheelbase: {wheelbase:.4f}
""")
    
    # ---- Generate C header snippet ----
    print("-" * 70)
    print("MPC_TYPES.H SNIPPET (update in MPC/include/mpc_types.h)")
    print("-" * 70)
    
    wb_q16 = int(wheelbase * 65536)
    ms_q16 = int(max_steer * 65536)
    mv_q16 = int(max_velocity * 65536)
    
    print(f"""
    // Vehicle parameters (from parameter identification)
    #define WHEELBASE       {wb_q16}   // {wheelbase:.4f}m in Q16.16
    #define MAX_STEER       {ms_q16}   // {max_steer:.4f} rad ({np.degrees(max_steer):.1f}°) in Q16.16
    #define MAX_VELOCITY    {mv_q16}   // {max_velocity:.2f} m/s in Q16.16
    // Friction coefficient μ = {mu:.3f}
    // Max lateral accel = {mu * GRAVITY:.2f} m/s²
""")
    
    print("=" * 70)
    print("Done. Run individual test scripts for detailed analysis.")
    print("=" * 70)


if __name__ == '__main__':
    main()
