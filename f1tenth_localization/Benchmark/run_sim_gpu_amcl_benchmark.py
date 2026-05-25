#!/usr/bin/env python3
"""Run the simulated benchmark for GPU AMCL only."""

import argparse
import os
import signal
import shutil
import subprocess
import sys
import time
from typing import List


BENCHMARK_NAME = 'AMCL_benchmark'

RECORD_TOPICS = [
    '/scan',
    '/scan_walls',
    '/scan_obstacles',
    '/ego_racecar/odom',
    '/ego_racecar/ground_truth',
    '/sensors/core',
    '/sensors/imu/raw',
    '/sensors/servo_position_command',
    '/commands/motor/current',
    '/commands/motor/brake',
    '/commands/motor/speed',
    '/commands/servo/position',
    '/imu/filtered_angular_velocity',
    '/odom_pose',
    '/amcl_pose',
    '/amcl_timing',
    '/amcl_gpu_timing',
    '/amcl_particle_count',
    '/amcl_kld_diagnostics',
    '/ekf_pose',
    '/tf',
    '/tf_static',
    '/map',
    '/map_metadata',
    '/initialpose',
    '/particlecloud',
    '/particlecloud_weighted_pre_resample',
    '/particle_cloud',
    '/local_raceline',
    '/drive',
    '/ackermann_cmd',
    '/clock',
]


def storage_id() -> str:
    requested = os.environ.get('ROSBAG_STORAGE_ID', 'mcap')
    if requested != 'mcap':
        return requested

    try:
        help_result = subprocess.run(
            ['ros2', 'bag', 'record', '-h'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
            check=False)
    except OSError:
        return requested

    if 'mcap' in help_result.stdout:
        return requested

    print('[warn] mcap storage plugin not detected; falling back to sqlite3')
    return 'sqlite3'


def raceline_heading_near_pose(trajectory_file: str,
                               x: float,
                               y: float,
                               fallback_yaw: float = 0.0) -> float:
    best_yaw = fallback_yaw
    best_d2 = float('inf')
    try:
        with open(trajectory_file, newline='') as handle:
            for raw in handle:
                line = raw.strip()
                if not line or line.startswith('#'):
                    continue
                parts = [part.strip() for part in line.split(',')]
                if len(parts) < 4:
                    continue
                try:
                    px = float(parts[1])
                    py = float(parts[2])
                    yaw = float(parts[3])
                except ValueError:
                    continue
                d2 = (px - x) * (px - x) + (py - y) * (py - y)
                if d2 < best_d2:
                    best_d2 = d2
                    best_yaw = yaw
    except OSError as exc:
        print(f'[warn] Cannot read trajectory for spawn yaw: {exc}; using {fallback_yaw}')
    return best_yaw


def stop_process(proc: subprocess.Popen, name: str) -> int:
    code = proc.poll()
    if code is not None:
        return code

    print(f'{name}: sending SIGINT')
    os.killpg(proc.pid, signal.SIGINT)
    try:
        return proc.wait(timeout=30.0)
    except subprocess.TimeoutExpired:
        print(f'{name}: SIGINT failed, sending SIGTERM')
        os.killpg(proc.pid, signal.SIGTERM)
        return proc.wait(timeout=10.0)


def bag_topic_message_count(run_dir: str, topic_name: str) -> int:
    metadata_path = os.path.join(run_dir, BENCHMARK_NAME, 'metadata.yaml')
    try:
        with open(metadata_path, 'r', encoding='utf-8') as handle:
            lines = handle.readlines()
    except OSError:
        return 0

    topic_marker = f'name: {topic_name}'
    for idx, line in enumerate(lines):
        if topic_marker not in line:
            continue
        for follow in lines[idx + 1:idx + 80]:
            stripped = follow.strip()
            if stripped == '- topic_metadata:' or stripped.startswith('name: '):
                break
            if stripped.startswith('message_count:'):
                try:
                    return int(stripped.split(':', 1)[1].strip())
                except ValueError:
                    return 0
    return 0


def validate_gpu_outputs(args: argparse.Namespace, run_dir: str) -> bool:
    pose_count = bag_topic_message_count(run_dir, '/amcl_pose')
    timing_count = bag_topic_message_count(run_dir, '/amcl_gpu_timing')
    if pose_count <= 0 or timing_count <= 0:
        print(
            '[error] GPU AMCL produced no usable AMCL data '
            f'(/amcl_pose={pose_count}, /amcl_gpu_timing={timing_count}). '
            'The GPU node likely failed before processing scans.')
        return False

    if args.cloud_publish_rate > 0.0:
        cloud_count = bag_topic_message_count(run_dir, '/particlecloud')
        if cloud_count <= 0:
            print(
                '[warn] Particle cloud publishing was requested but '
                f'/particlecloud has {cloud_count} recorded messages.')

    return True


def start_mcap_recording(args: argparse.Namespace,
                         run_dir: str,
                         repo_root: str,
                         env: dict) -> subprocess.Popen:
    bag_dir = os.path.join(run_dir, BENCHMARK_NAME)
    qos_file = os.path.join(
        repo_root,
        'f1tenth_system',
        'f1tenth_stack',
        'config',
        'localization_rosbag_qos.yaml')
    os.makedirs(os.path.dirname(bag_dir), exist_ok=True)

    cmd = [
        'ros2',
        'bag',
        'record',
        '-o', bag_dir,
        '-s', storage_id(),
        '--include-unpublished-topics',
        '--disable-keyboard-controls',
        '--qos-profile-overrides-path', qos_file,
        '--topics',
        *RECORD_TOPICS,
    ]

    print('\n=== foxglove mcap recording ===')
    print(' '.join(cmd))
    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    time.sleep(args.recording_warmup_sec)
    code = proc.poll()
    if code is not None:
        raise RuntimeError(f'ros2 bag record exited early with code {code}')
    print(f'Recording to: {bag_dir}')
    return proc


def run_gpu(args: argparse.Namespace) -> int:
    run_dir = os.path.join(args.output_root, BENCHMARK_NAME)
    if os.path.exists(run_dir):
        shutil.rmtree(run_dir)
    os.makedirs(run_dir, exist_ok=True)

    initial_pose_x = float(args.initial_pose_x)
    initial_pose_y = float(args.initial_pose_y)
    if args.initial_pose_yaw is None:
        initial_pose_yaw = raceline_heading_near_pose(
            args.trajectory_file, initial_pose_x, initial_pose_y)
    else:
        initial_pose_yaw = float(args.initial_pose_yaw)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(script_dir, '..', '..'))
    launch_file = os.path.join(
        repo_root,
        'f1tenth_system',
        'f1tenth_stack',
        'launch',
        'sim_amcl_benchmark.launch.py')

    cmd = [
        'ros2',
        'launch',
        launch_file,
        'localizer:=gpu',
        f'output_dir:={run_dir}',
        f'csv_name:={BENCHMARK_NAME}.csv',
        f'status_name:={BENCHMARK_NAME}_status.json',
        f'max_laps:={args.laps}',
        f'max_duration_sec:={args.max_duration_sec}',
        f'map_file:={args.map_file}',
        f'trajectory_file:={args.trajectory_file}',
        f'initial_pose_x:={initial_pose_x}',
        f'initial_pose_y:={initial_pose_y}',
        f'initial_pose_yaw:={initial_pose_yaw}',
        f'headless:={str(args.headless).lower()}',
        f'realistic_plant:={str(args.realistic_plant).lower()}',
        f'sim_odom_source:={args.sim_odom_source}',
        f'sim_drive_input_mode:={args.sim_drive_input_mode}',
        f'sim_drive_uses_acceleration_field:={str(args.sim_drive_uses_acceleration_field).lower()}',
        f'control_start_delay_sec:={args.control_start_delay_sec}',
        f'lateral_planner_avoidance_enabled:={str(args.avoidance_enabled).lower()}',
        f'monitor_strict_mode:={str(args.monitor_strict_mode).lower()}',
        f'system_monitor_cpu_sample_hz:={args.system_monitor_cpu_sample_hz}',
        f'system_monitor_gpu_sample_hz:={args.system_monitor_gpu_sample_hz}',
        f'system_monitor_csv_log_hz:={args.system_monitor_csv_log_hz}',
        f'system_monitor_long_csv_log_hz:={args.system_monitor_long_csv_log_hz}',
        f'system_monitor_memory_log_hz:={args.system_monitor_memory_log_hz}',
        f'system_monitor_memory_controller_log_hz:={args.system_monitor_memory_controller_log_hz}',
        f'system_monitor_emc_peak_bandwidth_mib_s:={args.system_monitor_emc_peak_bandwidth_mib_s}',
        f'mpc_raceline_speed_margin:={args.mpc_raceline_speed_margin}',
        f'amcl_global_initialization:={str(args.global_localization).lower()}',
        f'amcl_cloud_publish_rate:={args.cloud_publish_rate}',
        f'amcl_debug_pre_resample_particles:={str(args.debug_pre_resample_particles).lower()}',
        f'amcl_num_particles:={args.amcl_num_particles}',
        f'amcl_min_particles:={args.amcl_min_particles}',
        f'amcl_max_particles:={args.amcl_max_particles}',
        f'amcl_max_beams:={args.amcl_max_beams}',
        f'amcl_use_kld:={str(args.amcl_use_kld).lower()}',
        f'amcl_kld_epsilon:={args.amcl_kld_epsilon}',
        f'amcl_kld_z:={args.amcl_kld_z}',
        f'amcl_kld_bin_x:={args.amcl_kld_bin_x}',
        f'amcl_kld_bin_y:={args.amcl_kld_bin_y}',
        f'amcl_kld_bin_theta:={args.amcl_kld_bin_theta}',
        f'amcl_enable_recovery_injection:={str(args.amcl_enable_recovery_injection).lower()}',
        f'amcl_recovery_injection_ratio:={args.amcl_recovery_injection_ratio}',
        f'amcl_normalize_likelihood_by_beams:={str(args.amcl_normalize_likelihood_by_beams).lower()}',
        f'amcl_likelihood_scale:={args.amcl_likelihood_scale}',
        f'amcl_alpha1:={args.amcl_alpha1}',
        f'amcl_alpha2:={args.amcl_alpha2}',
        f'amcl_alpha3:={args.amcl_alpha3}',
        f'amcl_alpha4:={args.amcl_alpha4}',
        f'amcl_z_hit:={args.amcl_z_hit}',
        f'amcl_z_rand:={args.amcl_z_rand}',
        f'amcl_sigma_hit:={args.amcl_sigma_hit}',
        f'amcl_resample_threshold:={args.amcl_resample_threshold}',
        f'amcl_use_cluster_estimate:={str(args.amcl_use_cluster_estimate).lower()}',
        f'amcl_cluster_xy_bin_m:={args.amcl_cluster_xy_bin_m}',
        f'amcl_cluster_radius_m:={args.amcl_cluster_radius_m}',
        f'amcl_cluster_iterations:={args.amcl_cluster_iterations}',
        f'amcl_cluster_min_covariance:={args.amcl_cluster_min_covariance}',
        f'amcl_cluster_publish_min_weight:={args.amcl_cluster_publish_min_weight}',
        f'amcl_update_min_d:={args.amcl_update_min_d}',
        f'amcl_update_min_a:={args.amcl_update_min_a}',
        f'amcl_max_scan_age:={args.amcl_max_scan_age}',
        f'ekf_process_noise_scale:={args.ekf_process_noise_scale}',
    ]

    if args.extra_launch_arg:
        cmd.extend(args.extra_launch_arg)

    print('\n=== gpu benchmark ===')
    env = os.environ.copy()
    env.setdefault('PYTHONUNBUFFERED', '1')
    env.setdefault('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    # Force logs into the benchmark output. Some environments set ROS_LOG_DIR
    # to ~/.ros/log, which can be read-only in sandboxed runs and causes C++
    # nodes to abort before they publish anything.
    env['ROS_LOG_DIR'] = os.path.join(run_dir, 'ros_logs')
    os.makedirs(env['ROS_LOG_DIR'], exist_ok=True)

    if args.gpu_cache_profile:
        ncu = shutil.which('ncu')
        if ncu is None:
            raise RuntimeError('GPU cache profiling requested, but ncu was not found in PATH')
        system_dir = os.path.join(run_dir, 'system')
        os.makedirs(system_dir, exist_ok=True)
        gpu_cache_csv = os.path.join(system_dir, 'GpuCacheNsightCompute.csv')
        cmd = [
            ncu,
            '--target-processes', 'all',
            '--section', 'MemoryWorkloadAnalysis',
            '--csv',
            '--page', 'details',
            '--log-file', gpu_cache_csv,
            '--launch-skip', str(args.gpu_cache_launch_skip),
            '--launch-count', str(args.gpu_cache_launch_count),
            *cmd,
        ]
        print(f'GPU cache profile output: {gpu_cache_csv}')

    print(' '.join(cmd))

    recorder_proc = start_mcap_recording(args, run_dir, repo_root, env)
    proc = subprocess.Popen(cmd, env=env, start_new_session=True)
    deadline = (
        None if args.process_timeout_sec <= 0.0
        else time.monotonic() + args.process_timeout_sec
    )

    try:
        while True:
            code = proc.poll()
            if code is not None:
                stop_process(recorder_proc, 'rosbag')
                if code == 0 and not validate_gpu_outputs(args, run_dir):
                    return 1
                return code
            if deadline is not None and time.monotonic() >= deadline:
                print('gpu: timeout, sending SIGINT')
                code = stop_process(proc, 'gpu')
                stop_process(recorder_proc, 'rosbag')
                if code == 0 and not validate_gpu_outputs(args, run_dir):
                    return 1
                return code
            time.sleep(1.0)
    except KeyboardInterrupt:
        stop_process(proc, 'gpu')
        stop_process(recorder_proc, 'rosbag')
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
        'sim_benchmark')

    parser.add_argument('--output-root', default=default_root)
    parser.add_argument('--laps', type=int, default=3)
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
    parser.add_argument('--initial-pose-x', default='0.5')
    parser.add_argument('--initial-pose-y', default='0.2')
    parser.add_argument(
        '--initial-pose-yaw',
        default=None,
        help='Defaults to nearest raceline heading at initial x/y.')
    parser.add_argument('--realistic-plant', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--sim-odom-source',
                        choices=('vesc', 'ground_truth'),
                        default='vesc',
                        help='vesc uses simulated VESC/IMU sensors and vesc_to_odom; ground_truth uses old pose odom.')
    parser.add_argument('--sim-drive-input-mode',
                        choices=('vesc', 'ackermann'),
                        default='vesc',
                        help='vesc routes /ackermann_cmd through ackermann_to_vesc; ackermann feeds gym directly.')
    parser.add_argument('--sim-drive-uses-acceleration-field',
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--control-start-delay-sec', type=float, default=3.0,
                        help='Delay planner/MPC/drive stack so map and AMCL initialize before the car moves.')
    parser.add_argument('--avoidance-enabled', action='store_true')
    parser.add_argument('--headless', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--monitor-strict-mode', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--system-monitor-cpu-sample-hz', type=float, default=100.0)
    parser.add_argument('--system-monitor-gpu-sample-hz', type=float, default=50.0)
    parser.add_argument('--system-monitor-csv-log-hz', type=float, default=50.0)
    parser.add_argument('--system-monitor-long-csv-log-hz', type=float, default=1.0)
    parser.add_argument('--system-monitor-memory-log-hz', type=float, default=1.0)
    parser.add_argument('--system-monitor-memory-controller-log-hz', type=float, default=1.0)
    parser.add_argument('--system-monitor-emc-peak-bandwidth-mib-s', type=float, default=0.0,
                        help='Optional Jetson peak EMC bandwidth for estimated MiB/s; 0 disables bandwidth estimate.')
    parser.add_argument('--mpc-raceline-speed-margin', type=float, default=0.0,
                        help='Extra speed above raceline v_ref. Default 0.0 keeps localization tests on the planned profile.')
    parser.add_argument('--global-localization', '--global-initialization',
                        action=argparse.BooleanOptionalAction,
                        default=False,
                        dest='global_localization')
    parser.add_argument('--cloud-publish-rate', type=float, default=40.0,
                        help='Particle cloud rate in Hz; 40 matches the scan rate.')
    parser.add_argument('--debug-pre-resample-particles',
                        action=argparse.BooleanOptionalAction,
                        default=True,
                        help='Publish weighted particles before resampling.')
    parser.add_argument('--amcl-num-particles', type=int, default=1000)
    parser.add_argument('--amcl-min-particles', type=int, default=1000)
    parser.add_argument('--amcl-max-particles', type=int, default=1000)
    parser.add_argument('--amcl-max-beams', type=int, default=270)
    parser.add_argument('--amcl-use-kld', action='store_true')
    parser.add_argument('--amcl-kld-epsilon', type=float, default=0.02)
    parser.add_argument('--amcl-kld-z', type=float, default=1.96)
    parser.add_argument('--amcl-kld-bin-x', type=float, default=0.5)
    parser.add_argument('--amcl-kld-bin-y', type=float, default=0.5)
    parser.add_argument('--amcl-kld-bin-theta', type=float, default=0.1)
    parser.add_argument('--amcl-enable-recovery-injection',
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--amcl-recovery-injection-ratio', type=float, default=0.0)
    parser.add_argument('--amcl-normalize-likelihood-by-beams',
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--amcl-likelihood-scale', type=float, default=0.75)
    parser.add_argument('--amcl-alpha1', type=float, default=0.4)
    parser.add_argument('--amcl-alpha2', type=float, default=0.4)
    parser.add_argument('--amcl-alpha3', type=float, default=0.2)
    parser.add_argument('--amcl-alpha4', type=float, default=0.2)
    parser.add_argument('--amcl-z-hit', type=float, default=0.95)
    parser.add_argument('--amcl-z-rand', type=float, default=0.05)
    parser.add_argument('--amcl-sigma-hit', type=float, default=0.10)
    parser.add_argument('--amcl-resample-threshold', type=float, default=0.3)
    parser.add_argument('--amcl-use-cluster-estimate',
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--amcl-cluster-xy-bin-m', type=float, default=0.25)
    parser.add_argument('--amcl-cluster-radius-m', type=float, default=0.75)
    parser.add_argument('--amcl-cluster-iterations', type=int, default=3)
    parser.add_argument('--amcl-cluster-min-covariance', type=float, default=0.0001)
    parser.add_argument('--amcl-cluster-publish-min-weight', type=float, default=0.60)
    parser.add_argument('--amcl-update-min-d', type=float, default=0.05)
    parser.add_argument('--amcl-update-min-a', type=float, default=0.05)
    parser.add_argument('--amcl-max-scan-age', type=float, default=0.12)
    parser.add_argument('--ekf-process-noise-scale', type=float, default=0.1)
    parser.add_argument('--recording-warmup-sec', type=float, default=2.0,
                        help='Seconds to let ros2 bag subscribe before launching sim.')
    parser.add_argument('--gpu-cache-profile', action='store_true',
                        help='Run benchmark under Nsight Compute MemoryWorkloadAnalysis and write system/GpuCacheNsightCompute.csv. Use only for short profiling runs.')
    parser.add_argument('--gpu-cache-launch-skip', type=int, default=20,
                        help='CUDA kernel launches to skip before Nsight Compute starts collecting GPU cache metrics.')
    parser.add_argument('--gpu-cache-launch-count', type=int, default=40,
                        help='CUDA kernel launches collected by Nsight Compute when --gpu-cache-profile is used.')
    parser.add_argument(
        '--extra-launch-arg',
        action='append',
        help='Extra ros2 launch arg, e.g. amcl_max_particles:=4000')
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    print(f'Output root: {args.output_root}')
    print(f'Benchmark output: {os.path.join(args.output_root, BENCHMARK_NAME)}')
    code = run_gpu(args)
    print(f'gpu: exit code {code}')
    if code != 0:
        return 1
    print('Done.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
