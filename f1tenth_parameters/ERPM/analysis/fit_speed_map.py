#!/usr/bin/env python3
"""Fit a full-envelope, zero-intercept ERPM/static-speed map.

Two static maps are fitted independently:

* selected raw ERPM -> LiDAR ground speed: the VEL_TO_ERPM command path;
* measured VESC ERPM -> LiDAR ground speed: physical wheel-speed/odometry map.

Neither permits a non-zero intercept.  The analysis compares linear and
quadratic-through-origin models on independent hold-out plateaus.  A quadratic
static map is reported only when it materially improves hold-out ground-speed
prediction.  It is never silently approximated by a global ERPM offset.

High-current tyre slip is intentionally *not* absorbed into this static map;
it is identified from Stage 8/9 separately.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from common import (
    accepted_capture_windows,
    analysis_dir,
    coverage,
    dump_yaml,
    eval_origin_quadratic,
    fit_origin_quadratic,
    interpolate,
    invert_speed_to_erpm,
    load_yaml,
    session_original_values,
    stage_tables,
    straight_filter,
    summarize_windows,
)


def _summary(session: Path, stage: str, phase: str, cfg: dict) -> pd.DataFrame:
    tables = stage_tables(session, stage)
    windows = accepted_capture_windows(tables['events'], phase)
    result = straight_filter(summarize_windows(windows, tables, cfg), cfg)
    if not result.empty:
        result = result[np.abs(result.imu_ax_mps2) <= float(cfg['analysis']['max_static_abs_longitudinal_accel_mps2'])].copy()
        # A command/ERPM trace is not useful evidence of a speed map when the
        # vehicle did not actually move.  In particular, the archived campaign
        # marked large ERPM probes stable while odometry stayed near zero.  Use
        # LiDAR ground speed as the minimum-motion gate for every moving fit;
        # keep the stationary observability epoch available for its noise-floor
        # diagnostic.
        if phase != 'stationary_observability':
            result = result[
                np.abs(result.vx_lidar_mps) >= float(cfg['analysis']['min_lidar_speed_mps'])
            ].copy()
    return result


def _coverage(summary: pd.DataFrame, expected_speeds: list[float], reps: int) -> pd.DataFrame:
    rows = []
    if summary.empty:
        summary = pd.DataFrame(columns=['nominal_speed_mps'])
    actual = summary['nominal_speed_mps'].to_numpy(dtype=float) if 'nominal_speed_mps' in summary else np.empty(0)
    for speed in map(float, expected_speeds):
        count = int((np.isfinite(actual) & np.isclose(actual, speed, rtol=0.0, atol=1e-6)).sum())
        rows.append({
            'nominal_speed_mps': speed,
            'accepted_usable_trials': count,
            'expected_trials': int(reps),
            'coverage_ok': count >= int(reps),
        })
    frame = pd.DataFrame(rows)
    frame['all_expected_conditions_present'] = bool(frame.coverage_ok.all()) if not frame.empty else False
    return frame


def _prediction_metrics(actual_v: np.ndarray, predicted_v: np.ndarray) -> dict:
    residual = np.asarray(actual_v, dtype=float) - np.asarray(predicted_v, dtype=float)
    finite = residual[np.isfinite(residual)]
    if not len(finite):
        return {'rmse_mps': math.inf, 'bias_mps': math.nan, 'n': 0}
    return {
        'rmse_mps': float(np.sqrt(np.mean(finite ** 2))),
        'bias_mps': float(np.mean(finite)),
        'n': int(len(finite)),
    }


def _fit_static(erpm: np.ndarray, speed: np.ndarray) -> dict:
    """Fit E(v) in linear and quadratic-through-origin forms."""
    e = np.asarray(erpm, dtype=float)
    v = np.asarray(speed, dtype=float)
    mask = np.isfinite(e) & np.isfinite(v)
    e, v = e[mask], v[mask]
    if len(e) < 4:
        raise ValueError('insufficient finite static-map samples')
    g_linear = float(np.dot(v, e) / max(np.dot(v, v), 1e-12))
    linear_pred = g_linear * v
    linear_rmse = float(np.sqrt(np.mean((e - linear_pred) ** 2)))
    linear_denom = float(np.sum((e - np.mean(e)) ** 2))
    linear_r2 = float(1.0 - np.sum((e - linear_pred) ** 2) / linear_denom) if linear_denom > 1e-12 else math.nan
    quad_fit = fit_origin_quadratic(v, e)
    linear_only = {
        'gain_erpm_per_mps': g_linear,
        'quadratic_erpm_per_mps2': 0.0,
        'fit_rmse_erpm': linear_rmse,
        'fit_r2': linear_r2,
        'n': int(len(e)),
    }
    quadratic = {
        'gain_erpm_per_mps': quad_fit['linear'],
        'quadratic_erpm_per_mps2': quad_fit['quadratic'],
        'fit_rmse_erpm': quad_fit['rmse'],
        'fit_r2': quad_fit['r2'],
        'n': quad_fit['n'],
    }
    return {'linear': linear_only, 'quadratic': quadratic}


def _predict_speed(erpm: np.ndarray, coeff: dict) -> np.ndarray:
    return invert_speed_to_erpm(
        erpm,
        float(coeff['gain_erpm_per_mps']),
        float(coeff.get('quadratic_erpm_per_mps2', 0.0)),
    )


def _direct_erpm_to_speed_fit(erpm: np.ndarray, speed: np.ndarray) -> dict:
    """Fit v(E)=aE+bE|E| for an odometry report/LUT candidate."""
    fit = fit_origin_quadratic(erpm, speed)
    return {
        'gain_mps_per_erpm': float(fit['linear']),
        'quadratic_mps_per_erpm2': float(fit['quadratic']),
        'fit_rmse_mps': float(fit['rmse']),
        'fit_r2': float(fit['r2']),
        'n': int(fit['n']),
    }


def _pipeline_summary(session: Path, cfg: dict) -> tuple[pd.DataFrame, pd.DataFrame]:
    pipeline = _summary(session, '05_vel_to_erpm_pipeline_audit', 'vel_to_erpm_pipeline_audit', cfg)
    spec = cfg['vel_to_erpm_pipeline_audit']
    cov = _coverage(pipeline.rename(columns={'speed_command_mps': 'nominal_speed_mps'}), list(map(float, spec['speed_commands_mps'])), int(spec['repetitions']))
    return pipeline, cov


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    gates = cfg['analysis']['gates']

    train = _summary(session, '03_raw_erpm_map_training', 'raw_erpm_training', cfg)
    hold = _summary(session, '04_raw_erpm_map_holdout', 'raw_erpm_holdout', cfg)
    train_spec = cfg['raw_erpm_map_training']
    hold_spec = cfg['raw_erpm_map_holdout']
    train_cov = _coverage(train, list(map(float, train_spec['nominal_speeds_mps'])), int(train_spec['repetitions']))
    hold_cov = _coverage(hold, list(map(float, hold_spec['nominal_speeds_mps'])), int(hold_spec['repetitions']))
    train_cov.to_parquet(out / 'erpm_map_training_coverage.parquet', index=False)
    hold_cov.to_parquet(out / 'erpm_map_holdout_coverage.parquet', index=False)
    if train.empty or hold.empty or not bool(train_cov.coverage_ok.all()) or not bool(hold_cov.coverage_ok.all()):
        raise SystemExit('ERPM static-map coverage incomplete; inspect erpm_map_*_coverage.parquet')

    for table, label in ((train, 'training'), (hold, 'holdout')):
        if 'raw_erpm_target' not in table:
            raise SystemExit(f'{label} raw-ERPM captures are missing raw_erpm_target event metadata')
        table['commanded_erpm'] = table['selected_speed_erpm'].where(
            np.isfinite(table['selected_speed_erpm']), table['raw_erpm_target']
        )
        table['command_delivery_error_erpm'] = table['commanded_erpm'] - table['raw_erpm_target']
        for col in ('commanded_erpm', 'erpm_measured', 'vx_lidar_mps'):
            if not np.isfinite(table[col].to_numpy(dtype=float)).all():
                raise SystemExit(f'{label} static map contains non-finite {col}')

    delivery_cap = float(gates['max_raw_erpm_delivery_error_erpm'])
    train_delivery = float(np.max(np.abs(train.command_delivery_error_erpm.to_numpy(dtype=float))))
    hold_delivery = float(np.max(np.abs(hold.command_delivery_error_erpm.to_numpy(dtype=float))))
    if max(train_delivery, hold_delivery) > delivery_cap:
        raise SystemExit('raw ERPM command delivery error exceeds gate')

    command_models = _fit_static(train.commanded_erpm.to_numpy(float), train.vx_lidar_mps.to_numpy(float))
    measured_models = _fit_static(train.erpm_measured.to_numpy(float), train.vx_lidar_mps.to_numpy(float))

    # Evaluate on ground-speed holdout, which is the unit relevant to both
    # VEL_TO_ERPM command tracking and ERPM-derived odometry.
    command_linear_pred = _predict_speed(hold.commanded_erpm.to_numpy(float), command_models['linear'])
    command_quad_pred = _predict_speed(hold.commanded_erpm.to_numpy(float), command_models['quadratic'])
    command_linear_metrics = _prediction_metrics(hold.vx_lidar_mps.to_numpy(float), command_linear_pred)
    command_quad_metrics = _prediction_metrics(hold.vx_lidar_mps.to_numpy(float), command_quad_pred)
    measured_linear_pred = _predict_speed(hold.erpm_measured.to_numpy(float), measured_models['linear'])
    measured_quad_pred = _predict_speed(hold.erpm_measured.to_numpy(float), measured_models['quadratic'])
    measured_linear_metrics = _prediction_metrics(hold.vx_lidar_mps.to_numpy(float), measured_linear_pred)
    measured_quad_metrics = _prediction_metrics(hold.vx_lidar_mps.to_numpy(float), measured_quad_pred)

    policy = cfg['analysis']['static_map']
    improvement = float(policy.get(
        'minimum_holdout_improvement_fraction',
        policy.get('quadratic_selection_min_holdout_rmse_improvement_fraction', 0.05),
    ))
    q_floor = float(policy.get(
        'minimum_abs_quadratic_erpm_per_mps2',
        policy.get('min_abs_quadratic_erpm_per_mps2', 0.0),
    ))
    q_abs = abs(float(command_models['quadratic']['quadratic_erpm_per_mps2']))
    command_requires_quadratic = bool(
        q_abs >= q_floor
        and command_quad_metrics['rmse_mps'] <= command_linear_metrics['rmse_mps'] * (1.0 - improvement)
    )
    measured_q_abs = abs(float(measured_models['quadratic']['quadratic_erpm_per_mps2']))
    odom_requires_quadratic = bool(
        measured_q_abs >= q_floor
        and measured_quad_metrics['rmse_mps'] <= measured_linear_metrics['rmse_mps'] * (1.0 - improvement)
    )

    selected_command_model = command_models['quadratic'] if command_requires_quadratic else command_models['linear']
    selected_measured_model = measured_models['quadratic'] if odom_requires_quadratic else measured_models['linear']
    selected_command_metrics = command_quad_metrics if command_requires_quadratic else command_linear_metrics
    selected_measured_metrics = measured_quad_metrics if odom_requires_quadratic else measured_linear_metrics

    train['vx_pred_command_linear_mps'] = _predict_speed(train.commanded_erpm.to_numpy(float), command_models['linear'])
    train['vx_pred_command_quadratic_mps'] = _predict_speed(train.commanded_erpm.to_numpy(float), command_models['quadratic'])
    hold['vx_pred_command_linear_mps'] = command_linear_pred
    hold['vx_pred_command_quadratic_mps'] = command_quad_pred
    train['vx_pred_measured_linear_mps'] = _predict_speed(train.erpm_measured.to_numpy(float), measured_models['linear'])
    train['vx_pred_measured_quadratic_mps'] = _predict_speed(train.erpm_measured.to_numpy(float), measured_models['quadratic'])
    hold['vx_pred_measured_linear_mps'] = measured_linear_pred
    hold['vx_pred_measured_quadratic_mps'] = measured_quad_pred

    direct_odom = _direct_erpm_to_speed_fit(train.erpm_measured.to_numpy(float), train.vx_lidar_mps.to_numpy(float))
    direct_odom_hold_pred = eval_origin_quadratic(
        hold.erpm_measured.to_numpy(float),
        direct_odom['gain_mps_per_erpm'], direct_odom['quadratic_mps_per_erpm2'],
    )
    direct_odom_hold_metrics = _prediction_metrics(hold.vx_lidar_mps.to_numpy(float), direct_odom_hold_pred)

    original = session_original_values(session)
    current_odom_scale = float(original.get('odom_speed_scale', 1.0) or 1.0)
    # The production odometry equation is:
    #     v_odom = (ERPM_measured / speed_to_erpm_gain) * odom_speed_scale
    # because the campaign enforces speed_to_erpm_offset = 0. The scalar scale
    # must therefore be derived relative to the candidate *command* gain. Using
    # the separately fitted measured-ERPM slope here would make the scale trend
    # artificially toward one and hide a command-to-measured drivetrain gap.
    command_gain_for_odom = float(command_models['linear']['gain_erpm_per_mps'])
    odom_unscaled_v = hold.erpm_measured.to_numpy(float) / max(command_gain_for_odom, 1e-12)
    linear_scale = np.divide(
        hold.vx_lidar_mps.to_numpy(float), odom_unscaled_v,
        out=np.full(len(hold), np.nan), where=np.abs(odom_unscaled_v) > 0.08,
    )
    candidate_odom_scale = float(np.nanmedian(linear_scale)) if np.isfinite(linear_scale).any() else math.nan
    odom_linear_scaled_metrics = _prediction_metrics(
        hold.vx_lidar_mps.to_numpy(float), candidate_odom_scale * odom_unscaled_v,
    ) if math.isfinite(candidate_odom_scale) else {'rmse_mps': math.inf, 'bias_mps': math.nan, 'n': 0}

    launch = _summary(session, '02_low_speed_launch', 'low_speed_launch', cfg)
    stable_raw = launch[launch.vx_lidar_mps >= float(cfg['low_speed_launch']['minimum_lidar_speed_mps'])].copy() if not launch.empty else launch
    if stable_raw.empty:
        min_stable_raw_speed = math.nan
        min_stable_raw_erpm = math.nan
    else:
        low = stable_raw.sort_values('erpm_measured').iloc[0]
        min_stable_raw_speed = float(low.vx_lidar_mps)
        min_stable_raw_erpm = float(low.erpm_measured)

    pipeline, pipeline_cov = _pipeline_summary(session, cfg)
    pipeline_cov.to_parquet(out / 'vel_to_erpm_pipeline_coverage.parquet', index=False)
    pipeline.to_parquet(out / 'vel_to_erpm_pipeline_audit_trials.parquet', index=False)
    if pipeline.empty or not bool(pipeline_cov.coverage_ok.all()):
        raise SystemExit('VEL_TO_ERPM pipeline audit coverage incomplete')
    pipeline_stable = pipeline[pipeline.vx_lidar_mps >= float(cfg['low_speed_launch']['minimum_lidar_speed_mps'])].copy()
    min_pipeline_speed = float(pipeline_stable.sort_values('speed_command_mps').iloc[0].speed_command_mps) if not pipeline_stable.empty else math.nan

    stationary_tables = stage_tables(session, '01_longitudinal_observability')
    stationary_windows = accepted_capture_windows(stationary_tables['events'], 'stationary_observability')
    stationary_summary = summarize_windows(stationary_windows, stationary_tables, cfg)
    stationary_noise = float(stationary_summary.vx_lidar_std_mps.median()) if len(stationary_summary) else math.nan
    odom_deadband = max(float(cfg['analysis']['desired_odom_deadband_mps']), 3.0 * stationary_noise) if math.isfinite(stationary_noise) else float(cfg['analysis']['desired_odom_deadband_mps'])

    legacy_scalar_only_supported = not command_requires_quadratic and not odom_requires_quadratic
    accepted = bool(
        selected_command_metrics['rmse_mps'] <= float(gates['max_speed_map_holdout_rmse_mps'])
        and abs(float(selected_command_metrics['bias_mps'])) <= float(gates['max_speed_map_holdout_bias_mps'])
        and selected_measured_metrics['rmse_mps'] <= float(gates['max_odom_holdout_rmse_mps'])
        and math.isfinite(min_pipeline_speed)
    )

    nominal = train.groupby('nominal_speed_mps', as_index=False).agg(
        accepted_trials=('trial_id', 'count'),
        raw_erpm_target_median=('raw_erpm_target', 'median'),
        selected_erpm_median=('commanded_erpm', 'median'),
        selected_erpm_std=('commanded_erpm', 'std'),
        measured_erpm_median=('erpm_measured', 'median'),
        measured_erpm_std=('erpm_measured', 'std'),
        vx_lidar_median=('vx_lidar_mps', 'median'),
        vx_lidar_std=('vx_lidar_mps', 'std'),
    )
    nominal.to_parquet(out / 'erpm_map_nominal_condition_summary.parquet', index=False)
    train.to_parquet(out / 'erpm_map_training_trials.parquet', index=False)
    hold.to_parquet(out / 'erpm_map_holdout_trials.parquet', index=False)
    training_candidate_path = out / 'erpm_speed_map_training_report.yaml'
    training_candidate = load_yaml(training_candidate_path) if training_candidate_path.exists() else {}

    report = {
        'static_map_constraint': 'ERPM(ground_speed=0) = 0 exactly; no global speed_to_erpm offset is fitted or permitted.',
        'command_map': {
            'model_form': 'ERPM = k1*v + k2*v*abs(v)',
            'linear': command_models['linear'],
            'quadratic': command_models['quadratic'],
            'linear_holdout_ground_speed': command_linear_metrics,
            'quadratic_holdout_ground_speed': command_quad_metrics,
            'requires_quadratic': command_requires_quadratic,
            'selected_model': 'quadratic' if command_requires_quadratic else 'linear',
        },
        'measured_erpm_odometry_map': {
            'model_form': 'ERPM = k1*v + k2*v*abs(v)',
            'linear': measured_models['linear'],
            'quadratic': measured_models['quadratic'],
            'linear_holdout_ground_speed': measured_linear_metrics,
            'quadratic_holdout_ground_speed': measured_quad_metrics,
            'requires_quadratic': odom_requires_quadratic,
            'selected_model': 'quadratic' if odom_requires_quadratic else 'linear',
            'direct_erpm_to_ground_speed_candidate': direct_odom,
            'direct_erpm_to_ground_speed_holdout': direct_odom_hold_metrics,
        },
        'max_selector_delivery_error_training_erpm': train_delivery,
        'max_selector_delivery_error_holdout_erpm': hold_delivery,
        'selector_delivery_gate_erpm': delivery_cap,
        'candidate_speed_to_erpm_gain': float(selected_command_model['gain_erpm_per_mps']),
        'candidate_speed_to_erpm_offset': 0.0,
        'candidate_speed_to_erpm_quadratic': float(selected_command_model.get('quadratic_erpm_per_mps2', 0.0)),
        'candidate_speed_to_erpm_gain_bootstrap': training_candidate.get('candidate_speed_to_erpm_gain_bootstrap', {}),
        'candidate_odom_speed_scale': candidate_odom_scale,
        'candidate_odom_scale_reference_speed_to_erpm_gain': command_gain_for_odom,
        'candidate_odom_linear_scaled_holdout': odom_linear_scaled_metrics,
        'candidate_odom_erpm_to_speed_gain': direct_odom['gain_mps_per_erpm'],
        'candidate_odom_erpm_to_speed_quadratic': direct_odom['quadratic_mps_per_erpm2'],
        'current_odom_speed_scale': current_odom_scale,
        'minimum_stable_raw_speed_mps': min_stable_raw_speed,
        'minimum_stable_raw_erpm': min_stable_raw_erpm,
        'minimum_stable_vel_to_erpm_command_mps': min_pipeline_speed,
        'candidate_slow_start_threshold_mps': min_pipeline_speed,
        'candidate_slow_start_increment_mps': min_pipeline_speed,
        'candidate_stop_speed_deadzone_mps': float(cfg['analysis']['desired_stop_speed_deadband_mps']),
        'candidate_odom_speed_deadband_mps': odom_deadband,
        'stationary_lidar_velocity_std_mps': stationary_noise,
        'legacy_scalar_only_code_supported': legacy_scalar_only_supported,
        'requires_full_stack_upgrade_for_selected_static_map': not legacy_scalar_only_supported,
        'accepted_for_candidate': accepted,
        'gates': dict(gates),
        'notes': [
            'A non-zero speed_to_erpm_offset is deliberately prohibited. Low-speed launch is separately modelled by slow-start logic.',
            'Quadratic static-map evidence is retained even if the installed scalar production path cannot represent it directly. Nonlinear static maps remain eligible for reversible shadow deployment and require the documented full-stack port before permanent installation.',
            'Transient high-current slip is handled by the Stage 8/9 traction model, not absorbed into this settled static map.',
        ],
    }
    dump_yaml(out / 'erpm_speed_map_report.yaml', report)
    print(json.dumps(report, indent=2, default=str))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
