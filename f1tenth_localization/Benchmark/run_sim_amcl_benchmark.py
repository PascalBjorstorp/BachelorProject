#!/usr/bin/env python3
"""Run headless sim benchmark once for GPU AMCL and once for Nav2 AMCL."""

import argparse
from datetime import datetime
import os
import signal
import subprocess
import sys
import time
from typing import List


def run_one(args: argparse.Namespace, localizer: str, root: str) -> int:
    run_dir = os.path.join(root, localizer)
    os.makedirs(run_dir, exist_ok=True)

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
        f'realistic_plant:={str(args.realistic_plant).lower()}',
        f'sim_drive_uses_acceleration_field:={str(args.sim_drive_uses_acceleration_field).lower()}',
        f'lateral_planner_avoidance_enabled:={str(args.avoidance_enabled).lower()}',
        f'monitor_strict_mode:={str(args.monitor_strict_mode).lower()}',
    ]

    if args.extra_launch_arg:
        cmd.extend(args.extra_launch_arg)

    print(f'\n=== {localizer} benchmark ===')
    print(' '.join(cmd))
    env = os.environ.copy()
    env.setdefault('PYTHONUNBUFFERED', '1')
    env.setdefault('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    timeout = args.process_timeout_sec
    deadline = None if timeout <= 0.0 else time.monotonic() + timeout

    try:
        while True:
            code = proc.poll()
            if code is not None:
                return code
            if deadline is not None and time.monotonic() >= deadline:
                print(f'{localizer}: timeout, sending SIGINT')
                os.killpg(proc.pid, signal.SIGINT)
                try:
                    return proc.wait(timeout=20.0)
                except subprocess.TimeoutExpired:
                    print(f'{localizer}: SIGINT failed, sending SIGTERM')
                    os.killpg(proc.pid, signal.SIGTERM)
                    return proc.wait(timeout=10.0)
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
    parser.add_argument('--laps', type=int, default=5)
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
    parser.add_argument('--initial-pose-x', default='-0.79')
    parser.add_argument('--initial-pose-y', default='-4.88')
    parser.add_argument('--initial-pose-yaw', default='0.641322')
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
    os.makedirs(args.output_root, exist_ok=True)
    print(f'Output root: {args.output_root}')

    failures = []
    for localizer in args.localizers:
        code = run_one(args, localizer, args.output_root)
        print(f'{localizer}: exit code {code}')
        if code != 0:
            failures.append((localizer, code))

    if failures:
        print('Failures:')
        for localizer, code in failures:
            print(f'  {localizer}: {code}')
        return 1

    print('Done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
