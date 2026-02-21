#!/usr/bin/env python3
"""
Master Test Runner for F1/10th Vehicle Model Parameters

Walks through all vehicle model parameter tests with y/n prompts.
These tests identify the physical car parameters needed for MPC / path planning.

PREREQUISITE CALIBRATION (must be done BEFORE running these tests):
    1. vesc_pid_test.py            — VESC speed PID tuning
    2. find_servo_offset.py        — Servo center offset
    3. find_servo_limits.py        — Physical servo min/max limits
    4. test_steering_gain.py       — Steering gain calibration
    5. test_steering_calibration.py — Full steering calibration sweep
    6. test_speed_sweep.py         — Speed/ERPM calibration
    7. test_current_limits.py      — Find safe motor current limits (iterative!)

Once the VESC calibration and current limits are set, run this script
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
        'script': 'test_wheelbase.py',
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
        'script': 'test_max_dynamics.py',
        'description': (
            'Measures max velocity, acceleration, and deceleration.\n'
            '  Requires: ~5m straight clear space\n'
            '  Duration: ~30s\n'
            '  Uses: odom + IMU (IMU-based decel is more accurate)\n'
            '  CAUTION: Drives at maximum speed!'
        ),
        'default_args': ['--max-speed', '3.0', '--accel-time', '5.0'],
        'data_prefix': 'max_dynamics_test',
        'parameters': ['max_velocity', 'max_acceleration', 'max_deceleration'],
    },
    {
        'name': 'Steering Rate',
        'script': 'test_steering_rate.py',
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
        'script': 'test_friction.py',
        'description': (
            'Measures tire grip (friction coefficient mu = a_y_max / g).\n'
            '  Requires: ~3m x 3m open space\n'
            '  Duration: ~1 min\n'
            '  Uses: IMU lateral acceleration (not affected by odom errors)\n'
            '  CAUTION: Approaches the limits of grip!'
        ),
        'default_args': ['--steering', '0.3', '--max-speed', '4.0', '--speed-step', '0.5'],
        'data_prefix': 'friction_test',
        'parameters': ['friction_coefficient', 'max_lateral_accel'],
    },
    {
        'name': 'Cornering Stiffness',
        'script': 'test_cornering_stiffness.py',
        'description': (
            'Identifies front/rear cornering stiffness (C_alpha_f, C_alpha_r).\n'
            '  Requires: ~3m x 3m open space\n'
            '  Duration: ~2 min\n'
            '  Drives steady-state circles at increasing speed\n'
            '  Needs: vehicle mass, l_f, l_r (--mass, --l-f, --l-r)'
        ),
        'default_args': ['--steering', '0.3', '--max-speed', '3.5', '--speed-step', '0.5'],
        'data_prefix': 'cornering_stiffness',
        'parameters': ['C_alpha_f', 'C_alpha_r', 'understeer_gradient'],
    },
    {
        'name': 'Longitudinal Tire Stiffness',
        'script': 'test_longitudinal_stiffness.py',
        'description': (
            'Identifies longitudinal tire stiffness (C_x) from slip ratio.\n'
            '  Requires: ~5m straight clear space\n'
            '  Duration: ~30s\n'
            '  Compares wheel speed (ERPM) vs body speed (IMU)\n'
            '  Needs: vehicle mass (--mass)'
        ),
        'default_args': ['--max-speed', '3.0'],
        'data_prefix': 'longitudinal_stiffness',
        'parameters': ['C_x'],
    },
    {
        'name': 'Motor Torque',
        'script': 'test_motor_torque.py',
        'description': (
            'Maps motor current to wheel force (effective torque).\n'
            '  Requires: ~5m straight clear space\n'
            '  Duration: ~1 min\n'
            '  Records motor current at different accelerations\n'
            '  Needs: vehicle mass, tire radius (--mass, --r-tire)\n'
            '  NOTE: Update --r-tire with your measured value!'
        ),
        'default_args': ['--speeds', '1.5,2.0,3.0,4.0'],
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
    print('    1. VESC PID tuned                (vesc_pid_test.py)')
    print('    2. Servo offset calibrated       (find_servo_offset.py)')
    print('    3. Servo limits found            (find_servo_limits.py)')
    print('    4. Steering gain calibrated      (test_steering_gain.py)')
    print('    5. Steering calibration done     (test_steering_calibration.py)')
    print('    6. Speed calibration done        (test_speed_sweep.py)')
    print('    7. VESC current limits set       (test_current_limits.py)')
    print()
    print('  test_current_limits.py is iterative — run it separately until')
    print('  you find safe motor current limits, then run this script.')
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
