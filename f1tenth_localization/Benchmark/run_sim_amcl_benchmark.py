#!/usr/bin/env python3
"""Run headless sim benchmarks for GPU AMCL and Nav2 AMCL."""

import argparse
from datetime import datetime
import json
import os
import signal
import subprocess
import sys
import time
from typing import List


def run_one(args: argparse.Namespace, localizer: str, root: str, run_index: int) -> int:
    if args.runs == 1:
        run_dir = os.path.join(root, localizer)
    else:
        run_dir = os.path.join(root, localizer, f'run_{run_index + 1:02d}')
    os.makedirs(run_dir, exist_ok=True)
    status_path = os.path.join(run_dir, 'run_status.json')
    try:
        os.remove(status_path)
    except FileNotFoundError:
        pass

    cmd = [
        'ros2',
        'launch',
        'f1tenth_stack',
        'sim_amcl_benchmark.launch.py',
        f'localizer:={localizer}',
        f'output_dir:={run_dir}',
        f'max_laps:={args.laps}',
        f'max_duration_sec:={args.max_duration_sec}',
        f'map_file:={args.map_file}',
        f'trajectory_file:={args.trajectory_file}',
        f'initial_pose_x:={args.initial_pose_x}',
        f'initial_pose_y:={args.initial_pose_y}',
        f'initial_pose_yaw:={args.initial_pose_yaw}',
        f'amcl_global_initialization:={str(args.global_localization).lower()}',
        f'realistic_plant:={str(args.realistic_plant).lower()}',
        f'sim_drive_uses_acceleration_field:={str(args.sim_drive_uses_acceleration_field).lower()}',
        f'lateral_planner_avoidance_enabled:={str(args.avoidance_enabled).lower()}',
        f'monitor_strict_mode:={str(args.monitor_strict_mode).lower()}',
    ]

    if args.extra_launch_arg:
        cmd.extend(args.extra_launch_arg)

    print(f'\n=== {localizer} benchmark run {run_index + 1}/{args.runs} ===')
    print(' '.join(cmd))
    env = os.environ.copy()
    env.setdefault('PYTHONUNBUFFERED', '1')
    env.setdefault('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    env.setdefault('NUMBA_DISABLE_COVERAGE', '1')

    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    timeout = args.process_timeout_sec
    deadline = None if timeout <= 0.0 else time.monotonic() + timeout

    def read_status():
        if not os.path.exists(status_path):
            return None
        try:
            with open(status_path) as handle:
                return json.load(handle)
        except (OSError, json.JSONDecodeError):
            return None

    def stop_process(reason: str) -> int:
        print(f'{localizer}: run status is {reason}, stopping launch')
        os.killpg(proc.pid, signal.SIGINT)
        try:
            return proc.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            print(f'{localizer}: SIGINT failed, sending SIGTERM')
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                return proc.wait(timeout=10.0)
            except subprocess.TimeoutExpired:
                print(f'{localizer}: SIGTERM failed, sending SIGKILL')
                os.killpg(proc.pid, signal.SIGKILL)
                return proc.wait(timeout=10.0)

    def checked_return(code: int) -> int:
        status = read_status()
        if status is None:
            print(f'{localizer}: missing run status: {status_path}')
            return code if code != 0 else 1
        reason = str(status.get('reason', ''))
        laps = int(status.get('laps', 0) or 0)
        print(f'{localizer}: status reason={reason} laps={laps}/{args.laps}')
        if args.laps > 0 and (reason != 'laps_complete' or laps < args.laps):
            return code if code != 0 else 1
        return code

    try:
        while True:
            code = proc.poll()
            if code is not None:
                return checked_return(code)
            status = read_status()
            if status is not None and str(status.get('reason', '')):
                return checked_return(stop_process(str(status.get('reason'))))
            if deadline is not None and time.monotonic() >= deadline:
                print(f'{localizer}: timeout, sending SIGINT')
                return checked_return(stop_process('process_timeout'))
            time.sleep(1.0)
    except KeyboardInterrupt:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=20.0)
        raise


def parse_args(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
    default_root = os.path.join(
        repo_root,
        'f1tenth_localization',
        'Benchmark',
        'Matlab',
        'sim_benchmark',
        datetime.now().strftime('%Y%m%d_%H%M%S'))
    parser.add_argument('--output-root', default=default_root)
    parser.add_argument('--localizers', nargs='+', default=['gpu', 'nav2'],
                        choices=['gpu', 'nav2'])
    parser.add_argument('--runs', type=int, default=1,
                        help='Number of repeated runs per localizer')
    parser.add_argument('--laps', type=int, default=10)
    parser.add_argument('--max-duration-sec', type=float, default=0.0)
    parser.add_argument('--process-timeout-sec', type=float, default=0.0)
    parser.add_argument(
        '--map-file',
        default=os.path.join(
            repo_root,
            'f1tenth_planning',
            'maps',
            'my_track_map.yaml'))
    parser.add_argument(
        '--trajectory-file',
        default=os.path.join(
            repo_root,
            'f1tenth_planning',
            'trajectories',
            'my_track_raceline.csv'))
    parser.add_argument('--initial-pose-x', default='0.0')
    parser.add_argument('--initial-pose-y', default='0.0')
    parser.add_argument('--initial-pose-yaw', default='0.0')
    parser.add_argument('--global-localization', action='store_true',
                        help='Use global AMCL initialization instead of the provided initial pose')
    parser.add_argument('--realistic-plant', action=argparse.BooleanOptionalAction,
                        default=True,
                        help='Use hardware-calibrated ROS gym plant from MPC tune_realistic_v2.py')
    parser.add_argument('--sim-drive-uses-acceleration-field',
                        action=argparse.BooleanOptionalAction,
                        default=True,
                        help='Use AckermannDrive.acceleration as gym acceleration command')
    parser.add_argument('--avoidance-enabled', action='store_true')
    parser.add_argument('--monitor-strict-mode', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument(
        '--extra-launch-arg',
        action='append',
        help='Extra ros2 launch arg, e.g. amcl_max_particles:=2000')
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    if args.runs <= 0:
        print('--runs must be > 0')
        return 2
    if args.laps <= 0 and args.max_duration_sec <= 0.0:
        print('Either --laps must be > 0 or --max-duration-sec must be > 0')
        return 2

    os.makedirs(args.output_root, exist_ok=True)
    print(f'Output root: {args.output_root}')
    print(f'Runs/localizer: {args.runs}')
    print(f'Laps/run: {args.laps}')

    for localizer in args.localizers:
        for run_index in range(args.runs):
            code = run_one(args, localizer, args.output_root, run_index)
            print(f'{localizer} run {run_index + 1}/{args.runs}: exit code {code}')
            if code != 0:
                print(f'Stopping benchmark after failed run: {localizer} run {run_index + 1}')
                return 1

    print('Done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
