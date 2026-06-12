#!/usr/bin/env python3
"""Parameter sweep runner for simulated GPU AMCL."""

import argparse
import csv
import json
import math
import os
import shlex
import shutil
import statistics
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from run_sim_gpu_amcl_benchmark import raceline_heading_near_pose, stop_process


BENCHMARK_NAME = 'AMCL_benchmark'

BASE_PARAMS: Dict[str, float] = {
    'sigma_hit': 0.12,
    'z_hit': 0.975,
    'z_rand': 0.025,
    'alpha1': 0.2,
    'alpha2': 0.2,
    'alpha3': 0.1,
    'alpha4': 0.1,
    'likelihood_scale': 1.0,
    'resample_threshold': 0.5,
    'process_noise_scale': 0.1,
    'transform_tolerance': 0.02,
    'max_beams': 270,
}


@dataclass(frozen=True)
class SweepCase:
    name: str
    group: str
    params: Dict[str, float]


def safe_name(text: str) -> str:
    allowed = []
    for char in text:
        if char.isalnum() or char in ('-', '_', '.'):
            allowed.append(char)
        else:
            allowed.append('_')
    return ''.join(allowed).strip('_')


def value_name(value: float) -> str:
    if isinstance(value, int):
        return str(value)
    return f'{value:g}'.replace('.', 'p').replace('-', 'm')


def with_base(**overrides: float) -> Dict[str, float]:
    params = dict(BASE_PARAMS)
    params.update(overrides)
    return params


def with_params(base: Dict[str, float], **overrides: float) -> Dict[str, float]:
    params = dict(base)
    params.update(overrides)
    return params


def alpha_scaled(base: Dict[str, float], scale: float) -> Dict[str, float]:
    return with_params(
        base,
        alpha1=0.2 * scale,
        alpha2=0.2 * scale,
        alpha3=0.1 * scale,
        alpha4=0.1 * scale)


def add_case(cases: List[SweepCase],
             name: str,
             group: str,
             params: Dict[str, float]) -> None:
    signature = tuple(sorted((key, round(float(value), 9))
                             for key, value in params.items()))
    for existing in cases:
        existing_signature = tuple(sorted((key, round(float(value), 9))
                                          for key, value in existing.params.items()))
        if existing_signature == signature:
            return
    cases.append(SweepCase(safe_name(name), group, params))


def add_single_param_cases(cases: List[SweepCase],
                           group: str,
                           param: str,
                           values: Iterable[float]) -> None:
    base_value = BASE_PARAMS[param]
    for value in values:
        if value == base_value:
            continue
        cases.append(SweepCase(
            name=safe_name(f'{group}_{param}_{value_name(value)}'),
            group=group,
            params=with_base(**{param: value})))


def add_single_param_cases_from_base(cases: List[SweepCase],
                                     group: str,
                                     param: str,
                                     values: Iterable[float],
                                     base_params: Dict[str, float]) -> None:
    base_value = base_params[param]
    for value in values:
        if value == base_value:
            continue
        params = dict(base_params)
        params[param] = value
        add_case(
            cases,
            f'{group}_{param}_{value_name(value)}',
            group,
            params)


def taguchi_l81_levels(num_factors: int) -> List[List[int]]:
    """Return an 81-run, three-level orthogonal design for up to 9 factors."""
    columns: List[Tuple[int, int, int, int]] = [
        (1, 0, 0, 0),
        (0, 1, 0, 0),
        (0, 0, 1, 0),
        (0, 0, 0, 1),
        (1, 1, 0, 0),
        (1, 2, 0, 0),
        (1, 0, 1, 0),
        (1, 0, 2, 0),
        (1, 0, 0, 1),
    ]
    if num_factors > len(columns):
        raise ValueError(f'taguchi_l81 supports at most {len(columns)} factors')

    rows: List[List[int]] = []
    for a in range(3):
        for b in range(3):
            for c in range(3):
                for d in range(3):
                    base = (a, b, c, d)
                    rows.append([
                        sum(coef * value for coef, value in zip(column, base)) % 3
                        for column in columns[:num_factors]
                    ])
    return rows


def taguchi_l81_cases() -> List[SweepCase]:
    fixed_params = {
        'transform_tolerance': 0.02,
        'max_beams': 270,
    }
    factors: List[Tuple[str, List[Dict[str, float]]]] = [
        ('s', [
            {'sigma_hit': 0.06},
            {'sigma_hit': 0.08},
            {'sigma_hit': 0.10},
        ]),
        ('z', [
            {'z_hit': 0.95, 'z_rand': 0.05},
            {'z_hit': 0.90, 'z_rand': 0.10},
            {'z_hit': 0.85, 'z_rand': 0.15},
        ]),
        ('l', [
            {'likelihood_scale': 0.75},
            {'likelihood_scale': 1.0},
            {'likelihood_scale': 3.0},
        ]),
        ('a1', [
            {'alpha1': 0.1},
            {'alpha1': 0.2},
            {'alpha1': 0.4},
        ]),
        ('a2', [
            {'alpha2': 0.1},
            {'alpha2': 0.2},
            {'alpha2': 0.4},
        ]),
        ('a3', [
            {'alpha3': 0.1},
            {'alpha3': 0.15},
            {'alpha3': 0.2},
        ]),
        ('a4', [
            {'alpha4': 0.1},
            {'alpha4': 0.15},
            {'alpha4': 0.2},
        ]),
        ('q', [
            {'process_noise_scale': 0.05},
            {'process_noise_scale': 0.2},
            {'process_noise_scale': 0.5},
        ]),
        ('r', [
            {'resample_threshold': 0.3},
            {'resample_threshold': 0.5},
            {'resample_threshold': 0.7},
        ]),
    ]

    cases: List[SweepCase] = []
    for index, levels in enumerate(taguchi_l81_levels(len(factors))):
        params = dict(fixed_params)
        level_names: List[str] = []
        for (factor_name, choices), level in zip(factors, levels):
            params.update(choices[level])
            level_names.append(f'{factor_name}{level + 1}')
        cases.append(SweepCase(
            safe_name(f'taguchi_l81_{index:03d}_{"_".join(level_names)}'),
            'taguchi_l81',
            params))
    return cases


def focused_amcl_l81_cases() -> List[SweepCase]:
    fixed_params = {
        'transform_tolerance': 0.02,
        'max_beams': 270,
        'process_noise_scale': 0.2,
    }
    factors: List[Tuple[str, List[Dict[str, float]]]] = [
        ('s', [
            {'sigma_hit': 0.05},
            {'sigma_hit': 0.06},
            {'sigma_hit': 0.07},
        ]),
        ('z', [
            {'z_hit': 0.85, 'z_rand': 0.15},
            {'z_hit': 0.90, 'z_rand': 0.10},
            {'z_hit': 0.95, 'z_rand': 0.05},
        ]),
        ('l', [
            {'likelihood_scale': 2.0},
            {'likelihood_scale': 3.0},
            {'likelihood_scale': 4.0},
        ]),
        ('a1', [
            {'alpha1': 0.1},
            {'alpha1': 0.2},
            {'alpha1': 0.4},
        ]),
        ('a2', [
            {'alpha2': 0.1},
            {'alpha2': 0.2},
            {'alpha2': 0.4},
        ]),
        ('a3', [
            {'alpha3': 0.1},
            {'alpha3': 0.15},
            {'alpha3': 0.2},
        ]),
        ('a4', [
            {'alpha4': 0.15},
            {'alpha4': 0.2},
            {'alpha4': 0.25},
        ]),
        ('r', [
            {'resample_threshold': 0.3},
            {'resample_threshold': 0.5},
            {'resample_threshold': 0.7},
        ]),
    ]

    cases: List[SweepCase] = []
    for index, levels in enumerate(taguchi_l81_levels(len(factors))):
        params = dict(fixed_params)
        level_names: List[str] = []
        for (factor_name, choices), level in zip(factors, levels):
            params.update(choices[level])
            level_names.append(f'{factor_name}{level + 1}')
        cases.append(SweepCase(
            safe_name(f'focused_amcl_l81_{index:03d}_{"_".join(level_names)}'),
            'focused_amcl_l81',
            params))
    return cases


def build_cases(mode: str) -> List[SweepCase]:
    cases = [SweepCase('baseline', 'baseline', dict(BASE_PARAMS))]

    focused_base = alpha_scaled(
        with_base(
            sigma_hit=0.08,
            z_hit=0.95,
            z_rand=0.05,
            process_noise_scale=0.1,
            resample_threshold=0.3),
        2.0)

    if mode == 'quick':
        add_single_param_cases(cases, 'sensor', 'sigma_hit', [0.08, 0.18])
        cases.extend([
            SweepCase('sensor_z_hit_0p95_z_rand_0p05', 'sensor',
                      with_base(z_hit=0.95, z_rand=0.05)),
            SweepCase('sensor_z_hit_0p90_z_rand_0p10', 'sensor',
                      with_base(z_hit=0.90, z_rand=0.10)),
            SweepCase('motion_alpha_scale_0p5', 'motion',
                      with_base(alpha1=0.1, alpha2=0.1, alpha3=0.05, alpha4=0.05)),
            SweepCase('motion_alpha_scale_2', 'motion',
                      with_base(alpha1=0.4, alpha2=0.4, alpha3=0.2, alpha4=0.2)),
        ])
        add_single_param_cases(cases, 'ekf', 'process_noise_scale', [0.05, 0.2])
        add_single_param_cases(cases, 'resample', 'resample_threshold', [0.3, 0.7])
        return cases

    if mode == 'focused':
        add_case(cases, 'focused_best_quick_combo', 'focused_start', focused_base)

        # Refine sensor model around quick winners: lower sigma, more random-beam tolerance.
        for sigma in (0.06, 0.08, 0.10, 0.12):
            for z_hit, z_rand in ((0.975, 0.025), (0.95, 0.05), (0.925, 0.075)):
                for likelihood_scale in (0.75, 1.0, 1.25):
                    add_case(
                        cases,
                        f'focused_sensor_sigma_{value_name(sigma)}'
                        f'_zhit_{value_name(z_hit)}'
                        f'_zrand_{value_name(z_rand)}'
                        f'_scale_{value_name(likelihood_scale)}',
                        'focused_sensor',
                        with_params(
                            focused_base,
                            sigma_hit=sigma,
                            z_hit=z_hit,
                            z_rand=z_rand,
                            likelihood_scale=likelihood_scale))

        # Refine odom/AMCL trust and resampling around quick winners.
        for alpha_scale in (1.5, 2.0, 2.5):
            for process_noise_scale in (0.05, 0.1, 0.2):
                for resample_threshold in (0.25, 0.30, 0.40):
                    params = alpha_scaled(focused_base, alpha_scale)
                    params.update({
                        'process_noise_scale': process_noise_scale,
                        'resample_threshold': resample_threshold,
                    })
                    add_case(
                        cases,
                        f'focused_trust_alpha_{value_name(alpha_scale)}'
                        f'_q_{value_name(process_noise_scale)}'
                        f'_resample_{value_name(resample_threshold)}',
                        'focused_trust',
                        params)

        for max_beams in (180, 270, 360, 540):
            add_case(
                cases,
                f'focused_max_beams_{max_beams}',
                'focused_beams',
                with_params(focused_base, max_beams=max_beams))

        return cases

    if mode == 'combo':
        for sigma in (0.06, 0.08):
            for z_hit, z_rand in ((0.975, 0.025), (0.90, 0.10), (0.85, 0.15)):
                for likelihood_scale in (1.0, 1.5, 2.0):
                    add_case(
                        cases,
                        f'combo_sigma_{value_name(sigma)}'
                        f'_zhit_{value_name(z_hit)}'
                        f'_zrand_{value_name(z_rand)}'
                        f'_scale_{value_name(likelihood_scale)}',
                        'combo_sensor',
                        with_base(
                            sigma_hit=sigma,
                            z_hit=z_hit,
                            z_rand=z_rand,
                            likelihood_scale=likelihood_scale))
        return cases

    if mode == 'wide':
        wide_base = {
            'sigma_hit': 0.10,
            'z_hit': 0.95,
            'z_rand': 0.05,
            'alpha1': 0.4,
            'alpha2': 0.4,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'likelihood_scale': 0.75,
            'resample_threshold': 0.3,
            'process_noise_scale': 0.2,
            'transform_tolerance': 0.02,
            'max_beams': 270,
        }
        cases = [SweepCase('baseline', 'baseline', dict(wide_base))]

        add_single_param_cases_from_base(
            cases, 'sensor', 'sigma_hit', [0.06, 0.08, 0.10, 0.12], wide_base)
        for z_hit, z_rand in ((0.99, 0.01), (0.95, 0.05), (0.90, 0.10), (0.85, 0.15)):
            params = dict(wide_base)
            params.update({'z_hit': z_hit, 'z_rand': z_rand})
            add_case(
                cases,
                f'sensor_z_hit_{value_name(z_hit)}_z_rand_{value_name(z_rand)}',
                'sensor',
                params)
        add_single_param_cases_from_base(
            cases, 'sensor', 'likelihood_scale',
            [0.25, 0.5, 0.75, 1.5, 2.0, 3.0],
            wide_base)
        add_single_param_cases_from_base(
            cases, 'sensor', 'max_beams', [180, 270, 540], wide_base)

        add_single_param_cases_from_base(
            cases, 'motion', 'alpha1', [0.1, 0.2, 0.4], wide_base)
        add_single_param_cases_from_base(
            cases, 'motion', 'alpha2', [0.1, 0.2, 0.4], wide_base)
        add_single_param_cases_from_base(
            cases, 'motion', 'alpha3', [0.1, 0.15, 0.2], wide_base)
        add_single_param_cases_from_base(
            cases, 'motion', 'alpha4', [0.1, 0.15, 0.2], wide_base)

        add_single_param_cases_from_base(
            cases, 'ekf', 'process_noise_scale',
            [0.05, 0.2, 0.5, 1.0, 2.0],
            wide_base)
        add_single_param_cases_from_base(
            cases, 'resample', 'resample_threshold',
            [0.2, 0.3, 0.5, 0.7, 0.8],
            wide_base)

        return cases

    if mode == 'taguchi_l81':
        return taguchi_l81_cases()

    if mode == 'focused_amcl_l81':
        return focused_amcl_l81_cases()

    add_single_param_cases(cases, 'sensor', 'sigma_hit',
                           [0.06, 0.08, 0.12, 0.18, 0.25])
    cases.extend([
        SweepCase('sensor_z_hit_0p99_z_rand_0p01', 'sensor',
                  with_base(z_hit=0.99, z_rand=0.01)),
        SweepCase('sensor_z_hit_0p95_z_rand_0p05', 'sensor',
                  with_base(z_hit=0.95, z_rand=0.05)),
        SweepCase('sensor_z_hit_0p90_z_rand_0p10', 'sensor',
                  with_base(z_hit=0.90, z_rand=0.10)),
        SweepCase('sensor_z_hit_0p85_z_rand_0p15', 'sensor',
                  with_base(z_hit=0.85, z_rand=0.15)),
    ])
    add_single_param_cases(cases, 'sensor', 'likelihood_scale', [0.5, 0.75, 1.5, 2.0])

    add_single_param_cases(cases, 'motion', 'alpha1', [0.05, 0.1, 0.2, 0.4])
    add_single_param_cases(cases, 'motion', 'alpha2', [0.05, 0.1, 0.2, 0.4])
    add_single_param_cases(cases, 'motion', 'alpha3', [0.025, 0.05, 0.1, 0.2])
    add_single_param_cases(cases, 'motion', 'alpha4', [0.025, 0.05, 0.1, 0.2])
    cases.extend([
        SweepCase('motion_alpha_scale_0p5', 'motion',
                  with_base(alpha1=0.1, alpha2=0.1, alpha3=0.05, alpha4=0.05)),
        SweepCase('motion_alpha_scale_2', 'motion',
                  with_base(alpha1=0.4, alpha2=0.4, alpha3=0.2, alpha4=0.2)),
    ])

    add_single_param_cases(cases, 'ekf', 'process_noise_scale', [0.05, 0.2, 0.5, 1.0])
    add_single_param_cases(cases, 'resample', 'resample_threshold', [0.3, 0.7])
    add_single_param_cases(cases, 'timing', 'transform_tolerance', [0.01, 0.05, 0.10])

    return cases


def percentile(values: List[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def mean_or_nan(values: List[float]) -> float:
    return statistics.fmean(values) if values else math.nan


def rmse_or_nan(values: List[float]) -> float:
    if not values:
        return math.nan
    return math.sqrt(statistics.fmean([value * value for value in values]))


def finite_float(text: str) -> Optional[float]:
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def summarize_run(run_dir: str,
                  expected_laps: int,
                  exit_code: int,
                  skip_first_sec: float) -> Dict[str, object]:
    status_path = os.path.join(run_dir, f'{BENCHMARK_NAME}_status.json')
    csv_path = os.path.join(run_dir, f'{BENCHMARK_NAME}.csv')
    status: Dict[str, object] = {}
    if os.path.exists(status_path):
        with open(status_path) as handle:
            status = json.load(handle)

    ekf_errors: List[float] = []
    yaw_errors: List[float] = []
    amcl_errors: List[float] = []
    amcl_yaw_errors: List[float] = []
    amcl_valid_rows = 0
    rows = 0
    first_amcl_row = 0

    first_wall_time_ns: Optional[int] = None

    if os.path.exists(csv_path):
        with open(csv_path, newline='') as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                rows += 1
                wall_time_ns: Optional[int] = None
                try:
                    wall_time_ns = int(row.get('wall_time_ns', ''))
                except (TypeError, ValueError):
                    wall_time_ns = None
                if wall_time_ns is not None and first_wall_time_ns is None:
                    first_wall_time_ns = wall_time_ns

                in_eval_window = True
                if (
                    skip_first_sec > 0.0
                    and wall_time_ns is not None
                    and first_wall_time_ns is not None
                ):
                    elapsed_sec = (wall_time_ns - first_wall_time_ns) * 1e-9
                    in_eval_window = elapsed_sec >= skip_first_sec

                if not in_eval_window:
                    continue

                ekf_err = finite_float(row.get('err_xy', ''))
                yaw_err = finite_float(row.get('err_yaw', ''))
                if ekf_err is not None:
                    ekf_errors.append(ekf_err)
                if yaw_err is not None:
                    yaw_errors.append(abs(yaw_err))

                amcl_x = finite_float(row.get('amcl_x', ''))
                amcl_y = finite_float(row.get('amcl_y', ''))
                gt_x = finite_float(row.get('gt_x', ''))
                gt_y = finite_float(row.get('gt_y', ''))
                amcl_yaw = finite_float(row.get('amcl_yaw', ''))
                gt_yaw = finite_float(row.get('gt_yaw', ''))
                if None not in (amcl_x, amcl_y, gt_x, gt_y):
                    amcl_valid_rows += 1
                    if first_amcl_row == 0:
                        first_amcl_row = rows
                    amcl_errors.append(math.hypot(amcl_x - gt_x, amcl_y - gt_y))
                if None not in (amcl_yaw, gt_yaw):
                    diff = math.atan2(math.sin(amcl_yaw - gt_yaw),
                                      math.cos(amcl_yaw - gt_yaw))
                    amcl_yaw_errors.append(abs(diff))

    laps = int(status.get('laps', 0) or 0)
    reason = str(status.get('reason', 'missing_status'))
    success = (
        exit_code == 0
        and reason == 'laps_complete'
        and laps >= expected_laps
        and rows > 0
        and amcl_valid_rows > 0
    )

    return {
        'exit_code': exit_code,
        'success': success,
        'reason': reason,
        'laps': laps,
        'rows': rows,
        'amcl_valid_rows': amcl_valid_rows,
        'amcl_valid_fraction': (amcl_valid_rows / rows) if rows else math.nan,
        'first_amcl_row': first_amcl_row,
        'ekf_mean_err_m': mean_or_nan(ekf_errors),
        'ekf_median_err_m': percentile(ekf_errors, 0.50),
        'ekf_p95_err_m': percentile(ekf_errors, 0.95),
        'ekf_max_err_m': max(ekf_errors) if ekf_errors else math.nan,
        'ekf_rmse_err_m': rmse_or_nan(ekf_errors),
        'ekf_mean_abs_yaw_rad': mean_or_nan(yaw_errors),
        'amcl_mean_err_m': mean_or_nan(amcl_errors),
        'amcl_median_err_m': percentile(amcl_errors, 0.50),
        'amcl_p95_err_m': percentile(amcl_errors, 0.95),
        'amcl_max_err_m': max(amcl_errors) if amcl_errors else math.nan,
        'amcl_rmse_err_m': rmse_or_nan(amcl_errors),
        'amcl_mean_abs_yaw_rad': mean_or_nan(amcl_yaw_errors),
    }


def command_for_case(args: argparse.Namespace,
                     case: SweepCase,
                     run_dir: str,
                     initial_pose_yaw: float) -> List[str]:
    params = case.params
    max_beams = int(args.fixed_max_beams) if args.fixed_max_beams > 0 else int(params["max_beams"])
    cmd = [
        'ros2',
        'launch',
        args.launch_file,
        'localizer:=gpu',
        f'output_dir:={run_dir}',
        f'csv_name:={BENCHMARK_NAME}.csv',
        f'status_name:={BENCHMARK_NAME}_status.json',
        f'max_laps:={args.laps}',
        f'max_duration_sec:={args.max_duration_sec}',
        f'map_file:={args.map_file}',
        f'trajectory_file:={args.trajectory_file}',
        f'initial_pose_x:={args.initial_pose_x}',
        f'initial_pose_y:={args.initial_pose_y}',
        f'initial_pose_yaw:={initial_pose_yaw}',
        'headless:=true',
        f'realistic_plant:={str(args.realistic_plant).lower()}',
        f'sim_odom_source:={args.sim_odom_source}',
        f'sim_drive_input_mode:={args.sim_drive_input_mode}',
        f'sim_drive_uses_acceleration_field:={str(args.sim_drive_uses_acceleration_field).lower()}',
        f'lateral_planner_avoidance_enabled:={str(args.avoidance_enabled).lower()}',
        f'monitor_strict_mode:={str(args.monitor_strict_mode).lower()}',
        f'mpc_raceline_speed_margin:={args.mpc_raceline_speed_margin}',
        f'amcl_global_initialization:={str(args.global_localization).lower()}',
        f'amcl_num_particles:={args.particles}',
        f'amcl_min_particles:={args.particles}',
        f'amcl_max_particles:={args.particles}',
        'amcl_use_kld:=false',
        f'amcl_update_min_d:={args.update_min_d}',
        f'amcl_update_min_a:={args.update_min_a}',
        f'amcl_cloud_publish_rate:={args.cloud_publish_rate}',
        f'amcl_debug_pre_resample_particles:={str(args.debug_pre_resample_particles).lower()}',
        f'amcl_max_beams:={max_beams}',
        'amcl_normalize_likelihood_by_beams:=true',
        f'amcl_likelihood_scale:={params["likelihood_scale"]}',
        'amcl_use_cluster_estimate:=true',
        f'amcl_cluster_xy_bin_m:={args.cluster_xy_bin_m}',
        f'amcl_cluster_radius_m:={args.cluster_radius_m}',
        f'amcl_cluster_iterations:={args.cluster_iterations}',
        f'amcl_cluster_min_covariance:={args.cluster_min_covariance}',
        f'amcl_cluster_publish_min_weight:={args.cluster_publish_min_weight}',
        f'amcl_alpha1:={params["alpha1"]}',
        f'amcl_alpha2:={params["alpha2"]}',
        f'amcl_alpha3:={params["alpha3"]}',
        f'amcl_alpha4:={params["alpha4"]}',
        f'amcl_z_hit:={params["z_hit"]}',
        f'amcl_z_rand:={params["z_rand"]}',
        f'amcl_sigma_hit:={params["sigma_hit"]}',
        f'amcl_resample_threshold:={params["resample_threshold"]}',
        f'amcl_max_scan_age:={args.max_scan_age}',
        f'ekf_process_noise_scale:={params["process_noise_scale"]}',
        f'ekf_transform_tolerance:={params["transform_tolerance"]}',
    ]
    if args.extra_launch_arg:
        cmd.extend(args.extra_launch_arg)
    return cmd


def load_workspace_env(repo_root: str) -> Dict[str, str]:
    env = os.environ.copy()
    setup_path = os.path.join(repo_root, 'install', 'setup.bash')
    if not os.path.exists(setup_path):
        return env

    cmd = f'source {shlex.quote(setup_path)} && env -0'
    result = subprocess.run(
        ['bash', '-lc', cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False)
    if result.returncode != 0:
        print(f'[warn] Failed to source {setup_path}; using current environment')
        return env

    loaded: Dict[str, str] = {}
    for item in result.stdout.split(b'\0'):
        if not item or b'=' not in item:
            continue
        key, value = item.split(b'=', 1)
        loaded[key.decode()] = value.decode()
    loaded.setdefault('PYTHONUNBUFFERED', '1')
    loaded.setdefault('RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    return loaded


def tail_text(path: str, lines: int = 30) -> str:
    try:
        with open(path, errors='replace') as handle:
            content = handle.readlines()
    except OSError:
        return ''
    return ''.join(content[-lines:])


def error_excerpt(path: str, lines: int = 30) -> str:
    patterns = (
        'ERROR',
        'Error',
        'Exception',
        'Traceback',
        'CUDA',
        'what():',
        'not found',
        'unrecognized',
        'Invalid',
    )
    try:
        with open(path, errors='replace') as handle:
            matches = [
                line for line in handle
                if any(pattern in line for pattern in patterns)
            ]
    except OSError:
        return ''
    return ''.join(matches[-lines:])


def run_command(cmd: List[str],
                timeout_sec: float,
                ros_log_dir: str,
                env: Dict[str, str],
                launch_log_path: str) -> int:
    env = dict(env)
    os.makedirs(ros_log_dir, exist_ok=True)
    os.makedirs(os.path.dirname(launch_log_path), exist_ok=True)
    env.setdefault('ROS_LOG_DIR', ros_log_dir)
    deadline = None if timeout_sec <= 0.0 else time.monotonic() + timeout_sec

    with open(launch_log_path, 'w') as launch_log:
        launch_log.write(' '.join(cmd))
        launch_log.write('\n\n')
        launch_log.flush()
        proc = subprocess.Popen(
            cmd,
            env=env,
            start_new_session=True,
            stdout=launch_log,
            stderr=subprocess.STDOUT)

        try:
            while True:
                code = proc.poll()
                if code is not None:
                    return code
                if deadline is not None and time.monotonic() >= deadline:
                    print('sweep: timeout, sending SIGINT')
                    return stop_process(proc, 'sweep')
                time.sleep(1.0)
        except KeyboardInterrupt:
            stop_process(proc, 'sweep')
            raise


def write_rows(path: str, rows: List[Dict[str, object]]) -> None:
    if not rows:
        return
    fieldnames: List[str] = []
    for row in rows:
        for key in row:
            if key not in fieldnames:
                fieldnames.append(key)
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def aggregate_results(cases: List[SweepCase],
                      run_rows: List[Dict[str, object]]) -> List[Dict[str, object]]:
    summary_rows: List[Dict[str, object]] = []
    for index, case in enumerate(cases):
        rows = [row for row in run_rows if row['case_index'] == index]
        good = [row for row in rows if row['success']]
        summary = {
            'case_index': index,
            'case_name': case.name,
            'group': case.group,
            'runs': len(rows),
            'success_runs': len(good),
            'failed_runs': len(rows) - len(good),
            'params_json': json.dumps(case.params, sort_keys=True),
        }
        for key in (
            'amcl_mean_err_m',
            'amcl_median_err_m',
            'amcl_p95_err_m',
            'amcl_max_err_m',
            'amcl_rmse_err_m',
            'amcl_mean_abs_yaw_rad',
            'ekf_mean_err_m',
            'ekf_median_err_m',
            'ekf_p95_err_m',
            'ekf_max_err_m',
            'ekf_rmse_err_m',
            'ekf_mean_abs_yaw_rad',
            'amcl_valid_fraction',
        ):
            values = [float(row[key]) for row in good if math.isfinite(float(row[key]))]
            summary[f'mean_{key}'] = mean_or_nan(values)
            summary[f'std_{key}'] = statistics.stdev(values) if len(values) > 1 else 0.0
        summary_rows.append(summary)

    def sort_key(row: Dict[str, object]) -> float:
        value = float(row.get('mean_amcl_mean_err_m', math.nan))
        return value if math.isfinite(value) else float('inf')

    return sorted(summary_rows, key=sort_key)


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
        'gpu_amcl_sweep')

    parser.add_argument('--output-root', default=default_root)
    parser.add_argument('--mode', choices=[
        'quick',
        'focused',
        'combo',
        'core',
        'wide',
        'taguchi_l81',
        'focused_amcl_l81',
    ],
                        default='core')
    parser.add_argument('--laps', type=int, default=5)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--skip-first-sec', type=float, default=5.0,
                        help='Ignore startup rows when computing pose-error metrics.')
    parser.add_argument('--max-duration-sec', type=float, default=0.0)
    parser.add_argument('--process-timeout-sec', type=float, default=0.0)
    parser.add_argument('--start-index', type=int, default=0)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--only-group', action='append')
    parser.add_argument('--resume', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--stop-on-failure', action='store_true')
    parser.add_argument('--map-file', default=os.path.join(
        repo_root, 'f1tenth_planning', 'maps', 'my_track_map.yaml'))
    parser.add_argument('--trajectory-file', default=os.path.join(
        repo_root,
        'f1tenth_localization',
        'Benchmark',
        'Matlab',
        'sim_benchmark',
        'my_track_raceline_vcap_4p0.csv'))
    parser.add_argument('--launch-file', default=os.path.join(
        repo_root,
        'f1tenth_system',
        'f1tenth_stack',
        'launch',
        'sim_amcl_benchmark.launch.py'))
    parser.add_argument('--initial-pose-x', type=float, default=0.5)
    parser.add_argument('--initial-pose-y', type=float, default=0.2)
    parser.add_argument('--initial-pose-yaw', type=float, default=None)
    parser.add_argument('--particles', type=int, default=1000)
    parser.add_argument('--realistic-plant', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--sim-odom-source', choices=('vesc', 'ground_truth', 'calibrated_drift'),
                        default='vesc')
    parser.add_argument('--sim-drive-input-mode', choices=('vesc', 'ackermann'),
                        default='vesc')
    parser.add_argument('--sim-drive-uses-acceleration-field',
                        action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--mpc-raceline-speed-margin', type=float, default=0.0)
    parser.add_argument('--global-localization', '--global-initialization',
                        action=argparse.BooleanOptionalAction,
                        default=False,
                        dest='global_localization')
    parser.add_argument('--avoidance-enabled', action='store_true')
    parser.add_argument('--monitor-strict-mode', action=argparse.BooleanOptionalAction,
                        default=True)
    parser.add_argument('--cloud-publish-rate', type=float, default=0.0)
    parser.add_argument('--debug-pre-resample-particles',
                        action=argparse.BooleanOptionalAction,
                        default=False)
    parser.add_argument('--cluster-xy-bin-m', type=float, default=0.25)
    parser.add_argument('--cluster-radius-m', type=float, default=0.75)
    parser.add_argument('--cluster-iterations', type=int, default=3)
    parser.add_argument('--cluster-min-covariance', type=float, default=0.0001)
    parser.add_argument('--cluster-publish-min-weight', type=float, default=0.60)
    parser.add_argument('--update-min-d', type=float, default=0.05)
    parser.add_argument('--update-min-a', type=float, default=0.05)
    parser.add_argument('--max-scan-age', type=float, default=0.04)
    parser.add_argument('--fixed-max-beams', type=int, default=0,
                        help='When positive, override every sweep case with this max_beams value.')
    parser.add_argument('--extra-launch-arg', action='append')
    return parser.parse_args(argv)


def main(argv: List[str]) -> int:
    args = parse_args(argv)
    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
    ros_env = load_workspace_env(repo_root)
    cases = build_cases(args.mode)
    if args.only_group:
        groups = set(args.only_group)
        cases = [case for case in cases if case.group in groups]
    if args.start_index > 0:
        cases = cases[args.start_index:]
    if args.limit > 0:
        cases = cases[:args.limit]

    initial_pose_yaw = (
        raceline_heading_near_pose(args.trajectory_file,
                                   args.initial_pose_x,
                                   args.initial_pose_y)
        if args.initial_pose_yaw is None
        else args.initial_pose_yaw
    )

    os.makedirs(args.output_root, exist_ok=True)
    cases_path = os.path.join(args.output_root, 'sweep_cases.json')
    with open(cases_path, 'w') as handle:
        json.dump([
            {'case_index': index, 'name': case.name, 'group': case.group,
             'params': case.params}
            for index, case in enumerate(cases)
        ], handle, indent=2)
        handle.write('\n')

    run_rows: List[Dict[str, object]] = []
    run_results_path = os.path.join(args.output_root, 'run_results.csv')
    summary_path = os.path.join(args.output_root, 'summary.csv')

    print(f'Sweep mode: {args.mode}')
    print(f'Cases: {len(cases)}')
    print(f'Repeats: {args.repeats}')
    print(f'Laps/run: {args.laps}')
    print(f'Skip first: {args.skip_first_sec:g} s')
    print(f'Output: {args.output_root}')
    print(
        'Fixed: '
        f'particles={args.particles}, KLD=false, global_init={args.global_localization}, '
        f'update_min_d={args.update_min_d:g}, update_min_a={args.update_min_a:g}')
    print(
        'Sim setup: '
        f'odom={args.sim_odom_source}, drive={args.sim_drive_input_mode}, '
        f'accel_field={args.sim_drive_uses_acceleration_field}, '
        f'speed_margin={args.mpc_raceline_speed_margin:g}')

    for case_index, case in enumerate(cases):
        case_dir = os.path.join(args.output_root, f'{case_index:03d}_{case.name}')
        for repeat in range(1, args.repeats + 1):
            run_dir = os.path.join(case_dir, f'run_{repeat:02d}')
            status_path = os.path.join(run_dir, f'{BENCHMARK_NAME}_status.json')
            if args.resume and os.path.exists(status_path):
                print(f'skip existing {case.name} repeat {repeat}')
                metrics = summarize_run(run_dir, args.laps, 0, args.skip_first_sec)
                run_rows.append({
                    'case_index': case_index,
                    'case_name': case.name,
                    'group': case.group,
                    'repeat': repeat,
                    'run_dir': run_dir,
                    'elapsed_sec': 0.0,
                    'params_json': json.dumps(case.params, sort_keys=True),
                    **metrics,
                })
                write_rows(run_results_path, run_rows)
                write_rows(summary_path, aggregate_results(cases, run_rows))
                continue
            if os.path.exists(run_dir):
                shutil.rmtree(run_dir)
            os.makedirs(run_dir, exist_ok=True)

            cmd = command_for_case(args, case, run_dir, initial_pose_yaw)
            launch_log_path = os.path.join(run_dir, 'launch.log')
            print(f'\n=== case {case_index}/{len(cases) - 1}: {case.name} repeat {repeat}/{args.repeats} ===')
            print(' '.join(cmd))
            if args.dry_run:
                continue

            started = time.time()
            exit_code = run_command(
                cmd,
                args.process_timeout_sec,
                os.path.join(args.output_root, 'ros_logs'),
                ros_env,
                launch_log_path)
            elapsed_sec = time.time() - started
            metrics = summarize_run(run_dir, args.laps, exit_code, args.skip_first_sec)
            row = {
                'case_index': case_index,
                'case_name': case.name,
                'group': case.group,
                'repeat': repeat,
                'run_dir': run_dir,
                'elapsed_sec': elapsed_sec,
                'params_json': json.dumps(case.params, sort_keys=True),
                **metrics,
            }
            run_rows.append(row)
            write_rows(run_results_path, run_rows)
            write_rows(summary_path, aggregate_results(cases, run_rows))

            print(
                'result: '
                f'success={metrics["success"]} '
                f'reason={metrics["reason"]} '
                f'laps={metrics["laps"]} '
                f'amcl_mean={metrics["amcl_mean_err_m"]:.4f} '
                f'ekf_mean={metrics["ekf_mean_err_m"]:.4f}')
            if not metrics['success']:
                excerpt = error_excerpt(launch_log_path)
                if excerpt:
                    print('launch.log errors:')
                    print(excerpt.rstrip())
                tail = tail_text(launch_log_path)
                if tail:
                    print('launch.log tail:')
                    print(tail.rstrip())
            if args.stop_on_failure and not metrics['success']:
                return 1

    if not args.dry_run:
        summary_rows = aggregate_results(cases, run_rows)
        write_rows(summary_path, summary_rows)
        print(f'\nSummary: {summary_path}')
        for row in summary_rows[:10]:
            print(
                f'{row["case_index"]:>3} {row["case_name"]}: '
                f'amcl_mean={row["mean_amcl_mean_err_m"]:.4f} '
                f'ekf_mean={row["mean_ekf_mean_err_m"]:.4f} '
                f'success={row["success_runs"]}/{row["runs"]}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
