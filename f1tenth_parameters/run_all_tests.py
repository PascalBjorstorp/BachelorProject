#!/usr/bin/env python3
"""
Master Test Runner for F1/10th Vehicle Model Parameters

Walks through all vehicle model parameter tests with y/n prompts.
These tests identify the physical car parameters needed for MPC / path planning.

PREREQUISITE CALIBRATION (must be done BEFORE running these tests):
    1. prerequisites/vesc_pid_test.py            — VESC speed PID tuning
    2. prerequisites/find_servo_offset.py        — Servo center offset
    3. prerequisites/find_servo_limits.py        — Physical servo min/max limits
    4. prerequisites/test_steering_gain.py       — Steering gain calibration
    5. prerequisites/test_steering_calibration.py — Full steering calibration sweep
    6. prerequisites/test_speed_sweep.py         — Speed/ERPM calibration

Once the VESC calibration is done, run this script
to identify the vehicle model parameters:
    - Wheelbase
    - Max velocity, acceleration, deceleration
    - Steering actuator rate
    - Friction coefficient / max lateral acceleration
    - Cornering stiffness (front/rear)
    - Longitudinal tire stiffness
    - Motor torque mapping

Usage:
    python3 run_all_tests.py
    python3 run_all_tests.py --skip-confirmed   # Skip tests that already have data
"""

import argparse
import os
import subprocess
import sys
import time
from datetime import datetime

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ============================================================================
# Test definitions
# ============================================================================
# Each test: (name, script, description, default_args, data_prefix)

TESTS = [
    {
        'name': 'Wheelbase Verification',
        'script': 'tests/test_wheelbase.py',
        'description': (
            'Drives circles at low speed to extract the effective wheelbase.\n'
            '  Requires: ~3m x 3m open space\n'
            '  Duration: ~2 min per speed point\n'
            '  Uses: odom + IMU cross-check'
        ),
        'default_args': ['--steering', '0.3', '--speeds', '1.0,1.5,2.0', '--laps', '2'],
        'data_prefix': 'wheelbase_test',
        'parameters': ['wheelbase'],
    },
    {
        'name': 'Maximum Dynamics',
        'script': 'tests/test_max_dynamics.py',
        'description': (
            'Measures max velocity, acceleration, and deceleration.\n'
            '  Requires: ~5m straight clear space\n'
            '  Duration: ~30s\n'
            '  Uses: odom + IMU (IMU-based decel is more accurate)\n'
            '  CAUTION: Drives at maximum speed!'
        ),
        'default_args': ['--max-speed', '2.5', '--accel-time', '5.0'],
        'data_prefix': 'max_dynamics_test',
        'parameters': ['max_velocity', 'max_acceleration', 'max_deceleration'],
    },
    {
        'name': 'Steering Rate',
        'script': 'tests/test_steering_rate.py',
        'description': (
            'Measures the steering actuator speed (servo rate limit).\n'
            '  Requires: ~3m x 3m open space\n'
            '  Duration: ~30s\n'
            '  Uses: IMU yaw rate for accurate timing\n'
            '  Also reports: servo constant (time for 60 deg)\n'
            '  NOTE: Not yet validated on real hardware'
        ),
        'default_args': ['--speed', '1.5', '--steering', '0.3', '--repeats', '6'],
        'data_prefix': 'steering_rate_test',
        'parameters': ['max_steering_rate', 'servo_constant_60deg'],
    },
    {
        'name': 'Friction Limit',
        'script': 'tests/test_friction.py',
        'description': (
            'Measures tire grip (friction coefficient mu = a_y_max / g).\n'
            '  Requires: ~8m x 8m open space (auto-geofence ~8.3m)\n'
            '  Duration: ~70s per run, 5 runs\n'
            '  Uses: IMU lateral acceleration (not affected by odom errors)\n'
            '  CAUTION: Approaches the limits of grip!'
        ),
        'default_args': ['--steering', '0.3', '--min-speed', '2.0', '--max-speed', '5.0',
                         '--speed-step', '0.5', '--slip-abort', '0'],
        'data_prefix': 'friction_test',
        'parameters': ['friction_coefficient', 'max_lateral_accel'],
    },
    {
        'name': 'Cornering Stiffness',
        'script': 'tests/test_cornering_stiffness.py',
        'description': (
            'Identifies front/rear cornering stiffness (C_alpha_f, C_alpha_r).\n'
            '  Requires: ~10m x 10m open space (auto-geofence)\n'
            '  Duration: ~3min per run (4 steering angles × 3 speeds × 14s each)\n'
            '  Sweeps 4 steering angles [0.12-0.24 rad] at v=1.5-2.5 m/s\n'
            '  Uses LiDAR v_y for sideslip correction (breaks v² artifact)\n'
            '  Needs: vehicle mass, l_f, l_r (--mass, --l-f, --l-r)'
        ),
        'default_args': ['--steering', '0.12', '0.16', '0.20', '0.24',
                         '--min-speed', '1.5', '--max-speed', '2.5',
                         '--speed-step', '0.5',
                         '--settle-time', '4', '--record-time', '10'],
        'data_prefix': 'cornering_stiffness',
        'parameters': ['C_alpha_f', 'C_alpha_r', 'understeer_gradient'],
    },
    {
        'name': 'Longitudinal Tire Stiffness',
        'script': 'tests/test_longitudinal_stiffness.py',
        'description': (
            'Identifies longitudinal tire stiffness (C_x) from slip ratio.\n'
            '  Requires: ~20m straight clear space\n'
            '  Duration: ~45s per run\n'
            '  Uses LiDAR ICP body velocity vs wheel speed (ERPM)\n'
            '  Higher speed improves LiDAR SNR for slip measurement\n'
            '  Needs: vehicle mass (--mass)'
        ),
        'default_args': ['--max-speed', '5.0', '--cruise-time', '1.0', '--geofence', '20.0'],
        'data_prefix': 'longitudinal_stiffness',
        'parameters': ['C_x'],
    },
    {
        'name': 'Motor Torque',
        'script': 'tests/test_motor_torque.py',
        'description': (
            'Maps motor current to wheel force (effective torque).\n'
            '  Requires: ~10m straight clear space (repositioning between speeds)\n'
            '  Duration: ~2 min\n'
            '  Records motor current at different accelerations\n'
            '  Needs: vehicle mass, tire radius (--mass, --r-tire)\n'
            '  NOTE: Update --r-tire with your measured value!'
        ),
        'default_args': ['--speeds', '1.5,2.0,2.5,3.0', '--geofence', '10.0'],
        'data_prefix': 'motor_torque',
        'parameters': ['max_drive_torque', 'max_brake_torque', 'Kt_effective'],
    },
]


def prompt_yes_no(question: str) -> bool:
    """Prompt user with a y/n question. Returns True for yes."""
    while True:
        answer = input(f'{question} [y/n]: ').strip().lower()
        if answer in ('y', 'yes'):
            return True
        if answer in ('n', 'no'):
            return False
        print("  Please answer 'y' or 'n'.")


def prompt_custom_args(default_args: list) -> list:
    """Let user modify default arguments."""
    default_str = ' '.join(default_args)
    print(f'  Default arguments: {default_str}')
    custom = input('  Custom arguments (Enter to use defaults, or type new args): ').strip()
    if custom:
        return custom.split()
    return default_args


def has_existing_data(data_prefix: str) -> bool:
    """Check if there's existing data for this test."""
    if not os.path.isdir(DATA_DIR):
        return False
    for f in os.listdir(DATA_DIR):
        if f.startswith(data_prefix) and f.endswith('.csv'):
            return True
    return False


def run_test(test: dict, custom_args: list = None) -> bool:
    """
    Run a single test script as a subprocess.
    Returns True if the test completed successfully.
    """
    script_path = os.path.join(SCRIPT_DIR, test['script'])
    if not os.path.isfile(script_path):
        print(f'  ERROR: Script not found: {script_path}')
        return False

    args = custom_args if custom_args is not None else test['default_args']
    cmd = [sys.executable, script_path] + args

    print(f'  Running: {" ".join(cmd)}')
    print(f'  (Ctrl+C in the test to abort it safely)')
    print()

    try:
        result = subprocess.run(cmd, cwd=SCRIPT_DIR)
        return result.returncode == 0
    except KeyboardInterrupt:
        print('\n  Test interrupted by user.')
        return False


def print_header():
    """Print the runner header."""
    print()
    print('=' * 70)
    print('  F1/10th Vehicle Model Parameter Identification')
    print('=' * 70)
    print()
    print('  This runner walks through all vehicle model parameter tests.')
    print('  Each test will ask for confirmation before running.')
    print()
    print('  PREREQUISITES (must be completed before these tests):')
    print('    1. VESC PID tuned                (prerequisites/vesc_pid_test.py)')
    print('    2. Servo offset calibrated       (prerequisites/find_servo_offset.py)')
    print('    3. Servo limits found            (prerequisites/find_servo_limits.py)')
    print('    4. Steering gain calibrated      (prerequisites/test_steering_gain.py)')
    print('    5. Steering calibration done     (prerequisites/test_steering_calibration.py)')
    print('    6. Speed calibration done        (prerequisites/test_speed_sweep.py)')
    print()
    print('  Make sure the f1tenth_stack is running:')
    print('    ros2 launch f1tenth_stack bringup_launch.py')
    print()
    print('  Keep the joystick in hand at all times!')
    print()


def print_summary(results: dict):
    """Print summary of all test runs."""
    print()
    print('=' * 70)
    print('  TEST SUMMARY')
    print('=' * 70)
    print()

    for test in TESTS:
        name = test['name']
        status = results.get(name, 'skipped')
        if status == 'passed':
            marker = '[DONE]'
        elif status == 'failed':
            marker = '[FAIL]'
        else:
            marker = '[SKIP]'
        print(f'  {marker}  {name}')
        for param in test['parameters']:
            print(f'           -> {param}')

    print()

    # Count results
    passed = sum(1 for v in results.values() if v == 'passed')
    failed = sum(1 for v in results.values() if v == 'failed')
    skipped = len(TESTS) - passed - failed

    print(f'  Results: {passed} passed, {failed} failed, {skipped} skipped')
    print()

    if passed > 0:
        print(f'  Test data saved in: {DATA_DIR}/')
        print()

    if failed > 0:
        print('  Re-run failed tests individually for debugging.')
        print()

    # Remind about updating parameters
    if passed == len(TESTS):
        print('  All tests completed! Update your MPC parameters:')
        print('    - Wheelbase       → mpc_types.h: WHEELBASE')
        print('    - Max velocity    → mpc_types.h: MAX_VELOCITY')
        print('    - Max accel/decel → MPC constraints')
        print('    - Steering rate   → MPC rate penalty weight')
        print('    - Friction coeff  → MPC cornering speed limit')
        print()

    print('=' * 70)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Vehicle Model Parameter Test Runner',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'This runs the vehicle MODEL parameter tests (wheelbase, dynamics,\n'
            'steering rate, friction). VESC calibration tests (servo offset,\n'
            'steering gain, speed sweep) are prerequisites and should be\n'
            'completed separately before running this.\n'
        )
    )
    parser.add_argument(
        '--skip-confirmed', action='store_true',
        help='Automatically skip tests that already have data in data/')
    args = parser.parse_args()

    print_header()

    if not prompt_yes_no('Prerequisites completed? Ready to start?'):
        print('  Exiting. Complete prerequisites first.')
        return

    results = {}

    for i, test in enumerate(TESTS, 1):
        print()
        print('-' * 70)
        print(f'  TEST {i}/{len(TESTS)}: {test["name"]}')
        print('-' * 70)
        print(f'  Script: {test["script"]}')
        print(f'  {test["description"]}')
        print()

        # Check for existing data
        if args.skip_confirmed and has_existing_data(test['data_prefix']):
            print(f'  Existing data found for {test["data_prefix"]}.')
            if not prompt_yes_no('  Re-run this test anyway?'):
                print(f'  Skipping {test["name"]}.')
                results[test['name']] = 'skipped'
                continue

        if not prompt_yes_no(f'  Run {test["name"]}?'):
            print(f'  Skipping {test["name"]}.')
            results[test['name']] = 'skipped'
            continue

        # Let user customize arguments
        test_args = prompt_custom_args(test['default_args'])

        print()
        success = run_test(test, test_args)
        results[test['name']] = 'passed' if success else 'failed'

        if not success:
            print(f'\n  {test["name"]} did not complete successfully.')
            if not prompt_yes_no('  Continue with remaining tests?'):
                # Mark remaining as skipped
                for remaining in TESTS[i:]:
                    results[remaining['name']] = 'skipped'
                break

        # Pause between tests
        if i < len(TESTS):
            print()
            print('  Test complete. Prepare the car for the next test.')
            input('  Press Enter when ready to continue...')

    print_summary(results)


if __name__ == '__main__':
    main()
