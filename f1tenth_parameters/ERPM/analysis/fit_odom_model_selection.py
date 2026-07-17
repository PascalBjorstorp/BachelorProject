#!/usr/bin/env python3
"""Select a causal wheel-odometry estimator from held-out LiDAR ground speed.

The selector deliberately treats three functions as separate objects:

* command map: requested ground speed -> ERPM command, constrained through (0, 0);
* wheel observation: measured ERPM -> static wheel-speed estimate, also through (0, 0);
* actual odometry estimator: optionally corrects the wheel observation for
  acceleration/current and fuses it causally with IMU acceleration.

No model is accepted based on training error.  Static-map selection uses Stage
04; dynamic model selection uses complete Stage 09 hold-out trajectories.
"""
from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Iterable

import numpy as np
import pandas as pd

from common import (
    accepted_capture_windows,
    analysis_dir,
    dump_yaml,
    load_yaml,
    motion_windows,
    stage_tables,
    summarize_windows,
    straight_filter,
)

EPS = 1e-9


@dataclass
class StaticModel:
    name: str
    command_kind: str
    odom_kind: str
    params: dict


def _finite(a: np.ndarray) -> np.ndarray:
    return np.asarray(a, dtype=float)


def _rmse(y: np.ndarray, p: np.ndarray) -> float:
    m = np.isfinite(y) & np.isfinite(p)
    return float(np.sqrt(np.mean((y[m] - p[m]) ** 2))) if np.any(m) else math.inf


def _bias(y: np.ndarray, p: np.ndarray) -> float:
    m = np.isfinite(y) & np.isfinite(p)
    return float(np.mean(p[m] - y[m])) if np.any(m) else math.nan


def _p95(y: np.ndarray, p: np.ndarray) -> float:
    m = np.isfinite(y) & np.isfinite(p)
    return float(np.quantile(np.abs(p[m] - y[m]), 0.95)) if np.any(m) else math.inf


def _metrics(y: np.ndarray, p: np.ndarray) -> dict:
    m = np.isfinite(y) & np.isfinite(p)
    if not np.any(m):
        return {'n': 0, 'rmse_mps': math.inf, 'bias_mps': math.nan, 'p95_abs_error_mps': math.inf}
    return {
        'n': int(m.sum()),
        'rmse_mps': _rmse(y[m], p[m]),
        'bias_mps': _bias(y[m], p[m]),
        'p95_abs_error_mps': _p95(y[m], p[m]),
    }


def _origin_linear(x: np.ndarray, y: np.ndarray) -> dict:
    x, y = _finite(x), _finite(y)
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 6:
        raise ValueError('insufficient data for origin linear model')
    c = float(np.dot(x, y) / max(np.dot(x, x), EPS))
    return {'linear': c, 'quadratic': 0.0}


def _origin_quadratic(x: np.ndarray, y: np.ndarray) -> dict:
    x, y = _finite(x), _finite(y)
    m = np.isfinite(x) & np.isfinite(y)
    x, y = x[m], y[m]
    if len(x) < 8:
        raise ValueError('insufficient data for origin quadratic model')
    X = np.column_stack([x, x * np.abs(x)])
    c, *_ = np.linalg.lstsq(X, y, rcond=None)
    return {'linear': float(c[0]), 'quadratic': float(c[1])}


def _eval_poly(x: np.ndarray, p: dict) -> np.ndarray:
    x = _finite(x)
    return float(p['linear']) * x + float(p.get('quadratic', 0.0)) * x * np.abs(x)


def _make_monotone_knots(x: np.ndarray, y: np.ndarray, max_knots: int) -> dict:
    """Create a zero-anchored monotone piecewise-linear x -> y model.

    A LUT has less extrapolation risk than high-order polynomial fitting.  The
    knots are medians of the configured conditions and are monotonised by a
    cumulative maximum.  It remains zero at exactly zero ERPM.
    """
    frame = pd.DataFrame({'x': _finite(x), 'y': _finite(y)}).dropna().sort_values('x')
    if len(frame) < 8:
        raise ValueError('insufficient data for monotone LUT')
    # Bin by quantiles; median is robust to remaining capture noise.
    bins = min(max_knots - 1, max(3, int(np.sqrt(len(frame)))))
    frame['bin'] = pd.qcut(frame['x'].rank(method='first'), q=bins, duplicates='drop')
    k = frame.groupby('bin', observed=True)[['x', 'y']].median().sort_values('x')
    xx = np.concatenate([[0.0], k.x.to_numpy(float)])
    yy = np.concatenate([[0.0], k.y.to_numpy(float)])
    # Monotonic forward map.  Signed reverse extension is handled at evaluation.
    pos = xx >= 0.0
    xx, yy = xx[pos], yy[pos]
    order = np.argsort(xx)
    xx, yy = xx[order], yy[order]
    unique = np.concatenate([[True], np.diff(xx) > 1e-9])
    xx, yy = xx[unique], yy[unique]
    yy = np.maximum.accumulate(yy)
    if len(xx) < 2 or not np.all(np.diff(xx) > 0):
        raise ValueError('invalid monotone LUT knots')
    return {'x_knots': [float(v) for v in xx], 'y_knots': [float(v) for v in yy]}


def _eval_lut(x: np.ndarray, p: dict) -> np.ndarray:
    x = _finite(x)
    sign = np.sign(x)
    ax = np.abs(x)
    xx = np.asarray(p['x_knots'], dtype=float)
    yy = np.asarray(p['y_knots'], dtype=float)
    # Linear extrapolation from final segment, retaining monotonicity.
    slope = (yy[-1] - yy[-2]) / max(xx[-1] - xx[-2], EPS)
    value = np.interp(ax, xx, yy)
    over = ax > xx[-1]
    value[over] = yy[-1] + slope * (ax[over] - xx[-1])
    return sign * value


def _model_eval(model: StaticModel, erpm: np.ndarray) -> np.ndarray:
    if model.odom_kind == 'lut':
        return _eval_lut(erpm, model.params['odom'])
    return _eval_poly(erpm, model.params['odom'])


def _model_command_eval(model: StaticModel, speed: np.ndarray) -> np.ndarray:
    if model.command_kind == 'lut':
        return _eval_lut(speed, model.params['command'])
    return _eval_poly(speed, model.params['command'])


def _build_static_models(train: pd.DataFrame, cfg: dict) -> dict[str, StaticModel]:
    e = train.erpm_measured.to_numpy(float)
    v = train.vx_lidar_mps.to_numpy(float)
    # Command model uses actual delivered ERPM when present; this is the map to
    # deploy in AckermannToVesc.  The odometry map uses measured VESC ERPM.
    commanded = train.commanded_erpm.to_numpy(float)
    models: dict[str, StaticModel] = {}
    for name, fit in [('linear', _origin_linear), ('quadratic', _origin_quadratic)]:
        models[name] = StaticModel(
            name=name,
            command_kind='poly', odom_kind='poly',
            params={'command': fit(v, commanded), 'odom': fit(e, v)},
        )
    knots = int(cfg['analysis']['static_map']['lut_max_knots'])
    models['monotone_lut'] = StaticModel(
        name='monotone_lut', command_kind='lut', odom_kind='lut',
        params={'command': _make_monotone_knots(v, commanded, knots), 'odom': _make_monotone_knots(e, v, knots)},
    )
    return models


def _static_summary(session: Path, stage: str, phase: str, cfg: dict) -> pd.DataFrame:
    tables = stage_tables(session, stage)
    windows = accepted_capture_windows(tables['events'], phase)
    summary = straight_filter(summarize_windows(windows, tables, cfg), cfg)
    if summary.empty:
        return summary
    summary = summary.copy()
    summary['commanded_erpm'] = summary.get('selected_speed_erpm', pd.Series(np.nan, index=summary.index))
    fallback = summary.get('raw_erpm_target', pd.Series(np.nan, index=summary.index))
    summary['commanded_erpm'] = summary.commanded_erpm.where(np.isfinite(summary.commanded_erpm), fallback)
    # Static candidates only see windows that are actually settled longitudinally.
    cap = float(cfg['analysis']['max_static_abs_longitudinal_accel_mps2'])
    return summary[np.abs(summary.imu_ax_mps2) <= cap].copy()


def _stationary_imu_ax_bias(session: Path) -> tuple[float, dict]:
    """Estimate a fixed longitudinal IMU bias from Stage 1 neutral capture.

    The fused candidate integrates acceleration, so a stationary bias must not
    be allowed to masquerade as vehicle acceleration.  This is a causal
    deployment parameter, not a retrospective smoothing operation.
    """
    t = stage_tables(session, '01_longitudinal_observability')
    windows = accepted_capture_windows(t['events'], 'stationary_observability')
    imu = t['imu']
    values: list[float] = []
    for _, w in windows.iterrows():
        f = imu[(imu.bag_ns >= int(w.start_ns)) & (imu.bag_ns <= int(w.end_ns))]
        if not f.empty and 'ax' in f:
            values.extend(f.ax.to_numpy(float).tolist())
    a = np.asarray(values, float)
    a = a[np.isfinite(a)]
    diagnostic = {
        'n': int(len(a)),
        'observed_median_mps2': float(np.median(a)) if len(a) else math.nan,
        'observed_std_mps2': float(np.std(a)) if len(a) else math.nan,
        'applied_bias_mps2': 0.0,
        'applied_to_candidate': False,
        'policy': 'bringup calibrates IMU bias on every launch; session-stationary data is diagnostic only',
    }
    return 0.0, diagnostic


def _interp(frame: pd.DataFrame, target_t: np.ndarray, column: str) -> np.ndarray:
    if frame.empty or column not in frame or 'bag_ns' not in frame:
        return np.full(len(target_t), np.nan)
    f = frame[['bag_ns', column]].dropna().sort_values('bag_ns')
    if len(f) < 2:
        return np.full(len(target_t), np.nan)
    return np.interp(target_t.astype(float), f.bag_ns.to_numpy(float), f[column].to_numpy(float), left=np.nan, right=np.nan)


def _dynamic_samples(session: Path, stages: Iterable[tuple[str, Iterable[str]]], cfg: dict) -> pd.DataFrame:
    rows: list[pd.DataFrame] = []
    for stage, phases in stages:
        t = stage_tables(session, stage)
        windows = accepted_capture_windows(t['events'], list(phases))
        lidar = motion_windows(t).copy()
        if windows.empty or lidar.empty:
            continue
        if 'valid' in lidar:
            lidar = lidar[lidar.valid.astype(bool)]
        for _, w in windows.iterrows():
            li = lidar[(lidar.bag_ns >= int(w.start_ns)) & (lidar.bag_ns <= int(w.end_ns))].copy()
            if li.empty:
                continue
            # Preserve complete trial identity: no random sample-level split.
            li['stage'] = stage
            li['phase'] = str(w.get('phase', ''))
            li['trial_id'] = str(w.get('trial_id', ''))
            li['condition_id'] = str(w.get('condition_id', ''))
            li['erpm'] = _interp(t['vesc'], li.bag_ns.to_numpy(np.int64), 'erpm')
            li['motor_current'] = _interp(t['vesc'], li.bag_ns.to_numpy(np.int64), 'motor_current')
            li['input_current'] = _interp(t['vesc'], li.bag_ns.to_numpy(np.int64), 'input_current')
            li['imu_ax'] = _interp(t['imu'], li.bag_ns.to_numpy(np.int64), 'ax')
            li['imu_ay'] = _interp(t['imu'], li.bag_ns.to_numpy(np.int64), 'ay')
            li['imu_gz'] = _interp(t['imu'], li.bag_ns.to_numpy(np.int64), 'gz')
            li['selected_current'] = _interp(t['motor_selected_current'], li.bag_ns.to_numpy(np.int64), 'value')
            li['selected_brake'] = _interp(t['motor_selected_brake'], li.bag_ns.to_numpy(np.int64), 'value')
            li['vx_truth'] = li['vx'].to_numpy(float)
            rows.append(li)
    if not rows:
        return pd.DataFrame()
    out = pd.concat(rows, ignore_index=True)
    a = cfg['analysis']
    valid = (
        np.isfinite(out.vx_truth.to_numpy(float))
        & np.isfinite(out.erpm.to_numpy(float))
        & np.isfinite(out.imu_ax.to_numpy(float))
        & (np.abs(out.imu_gz.to_numpy(float)) <= float(a['max_straight_yaw_rate_rad_s']))
        & (np.abs(out.imu_ay.to_numpy(float)) <= float(a['max_abs_lateral_accel_mps2']))
    )
    return out[valid].sort_values(['trial_id', 'bag_ns']).reset_index(drop=True)


def _causal_lowpass_by_trial(frame: pd.DataFrame, tau: float) -> np.ndarray:
    out = np.full(len(frame), np.nan)
    for _, idx in frame.groupby('trial_id', sort=False).groups.items():
        ii = np.asarray(list(idx), dtype=int)
        part = frame.loc[ii].sort_values('bag_ns')
        t = part.bag_ns.to_numpy(float) * 1e-9
        a = part.imu_ax.to_numpy(float)
        f = np.empty(len(part), dtype=float)
        f[0] = a[0]
        for k in range(1, len(part)):
            dt = float(np.clip(t[k] - t[k - 1], 0.0, 0.1))
            alpha = dt / max(tau + dt, EPS)
            f[k] = f[k - 1] + alpha * (a[k] - f[k - 1])
        out[part.index.to_numpy(int)] = f
    return out


def _design_residual_features(v_static: np.ndarray, a: np.ndarray, current: np.ndarray) -> np.ndarray:
    gate = np.clip(np.abs(v_static) / 0.20, 0.0, 1.0)
    ap = np.maximum(a, 0.0)
    an = np.maximum(-a, 0.0)
    ip = np.maximum(current, 0.0)
    ib = np.maximum(-current, 0.0)
    # Each correction is gated to zero at rest, preserving v(ERPM=0)=0.
    return gate[:, None] * np.column_stack([ap, an, ip, ib, ap * np.abs(v_static), an * np.abs(v_static)])


def _ridge(X: np.ndarray, y: np.ndarray, lam: float) -> np.ndarray:
    X, y = np.asarray(X, float), np.asarray(y, float)
    m = np.isfinite(y) & np.all(np.isfinite(X), axis=1)
    X, y = X[m], y[m]
    if len(X) < X.shape[1] + 8:
        raise ValueError('insufficient dynamic samples for residual model')
    scale = np.sqrt(np.mean(X * X, axis=0))
    scale[scale < 1e-9] = 1.0
    Z = X / scale
    c = np.linalg.solve(Z.T @ Z + float(lam) * np.eye(Z.shape[1]), Z.T @ y)
    return c / scale


def _apply_adaptive(static: np.ndarray, a: np.ndarray, current: np.ndarray, coeff: np.ndarray, limit: float) -> np.ndarray:
    X = _design_residual_features(static, a, current)
    correction = X @ np.asarray(coeff, float)
    correction = np.clip(correction, -float(limit), float(limit))
    return static + correction


def _fused(frame: pd.DataFrame, wheel_obs: np.ndarray, a: np.ndarray, *, w_coast: float, w_high: float, accel_transition: float, min_weight: float) -> np.ndarray:
    out = np.full(len(frame), np.nan)
    # Expected order is not required; each trial is independently causal.
    for _, idx in frame.groupby('trial_id', sort=False).groups.items():
        part = frame.loc[list(idx)].sort_values('bag_ns')
        loc = part.index.to_numpy(int)
        t = part.bag_ns.to_numpy(float) * 1e-9
        wv = wheel_obs[loc]
        av = a[loc]
        state = float(wv[0]) if np.isfinite(wv[0]) else 0.0
        result = np.empty(len(part), dtype=float)
        result[0] = state
        for k in range(1, len(part)):
            dt = float(np.clip(t[k] - t[k - 1], 0.0, 0.10))
            pred = state + dt * av[k]
            blend = math.exp(-abs(float(av[k])) / max(float(accel_transition), EPS))
            weight = float(w_high) + (float(w_coast) - float(w_high)) * blend
            weight = float(np.clip(weight, min_weight, 1.0))
            obs = wv[k]
            state = pred if not np.isfinite(obs) else pred + weight * (obs - pred)
            result[k] = state
        out[loc] = result
    return out


def _regime_table(frame: pd.DataFrame, prediction: np.ndarray, cfg: dict) -> pd.DataFrame:
    sel = cfg['analysis']['odometry_model_selection']
    speed_edges = list(map(float, sel['speed_regimes_mps']))
    accel_coast, accel_high = map(float, sel['acceleration_regimes_mps2'])
    rows: list[dict] = []
    v = frame.vx_truth.to_numpy(float)
    a = frame.a_filt.to_numpy(float)
    for lo, hi, name in zip(speed_edges[:-1], speed_edges[1:], ['low_speed', 'mid_speed', 'high_speed']):
        m = (np.abs(v) >= lo) & (np.abs(v) < hi)
        rows.append({'regime': name, **_metrics(v[m], prediction[m])})
    for name, m in {
        'coast_or_low_accel': np.abs(a) <= accel_coast,
        'high_drive_accel': a >= accel_high,
        'high_brake_accel': a <= -accel_high,
    }.items():
        rows.append({'regime': name, **_metrics(v[m], prediction[m])})
    return pd.DataFrame(rows)


def _passes_regimes(table: pd.DataFrame, cfg: dict) -> tuple[bool, list[str]]:
    gates = cfg['analysis']['gates']
    min_n = int(cfg['analysis']['odometry_model_selection']['min_holdout_samples_per_regime'])
    faults: list[str] = []
    by = {str(r.regime): r for _, r in table.iterrows()}
    for name in ['low_speed', 'mid_speed', 'high_speed', 'coast_or_low_accel', 'high_drive_accel', 'high_brake_accel']:
        r = by.get(name)
        if r is None or int(r['n']) < min_n:
            faults.append(f'{name}: insufficient hold-out samples')
            continue
        if not np.isfinite(float(r['rmse_mps'])):
            faults.append(f'{name}: non-finite RMSE')
    if 'high_drive_accel' in by:
        r = by['high_drive_accel']
        if float(r['rmse_mps']) > float(gates['max_high_accel_odom_rmse_mps']) or abs(float(r['bias_mps'])) > float(gates['max_high_accel_odom_bias_mps']):
            faults.append('high_drive_accel: RMSE/bias gate')
    if 'high_brake_accel' in by:
        r = by['high_brake_accel']
        if float(r['rmse_mps']) > float(gates['max_high_brake_odom_rmse_mps']) or abs(float(r['bias_mps'])) > float(gates['max_high_brake_odom_bias_mps']):
            faults.append('high_brake_accel: RMSE/bias gate')
    return (not faults), faults


def _score(global_metrics: dict, regimes: pd.DataFrame) -> float:
    # Macro-average prevents the plentiful calm samples from hiding a bad high
    # acceleration region. Bias and 95th-percentile error both matter to MPC.
    if regimes.empty:
        return math.inf
    valid = regimes[np.isfinite(regimes.rmse_mps.to_numpy(float))]
    if valid.empty:
        return math.inf
    macro = float(np.mean(valid.rmse_mps.to_numpy(float) + 0.5 * np.abs(valid.bias_mps.to_numpy(float)) + 0.20 * valid.p95_abs_error_mps.to_numpy(float)))
    return 0.5 * float(global_metrics['rmse_mps']) + 0.5 * macro


def _fit_dynamic_models(static_model: StaticModel, train: pd.DataFrame, hold: pd.DataFrame, cfg: dict) -> dict[str, dict]:
    sel = cfg['analysis']['odometry_model_selection']
    tau_values = list(map(float, sel['accel_filter_time_constants_s']))
    best: dict[str, dict] = {}
    for tau in tau_values:
        tr = train.copy().reset_index(drop=True)
        ho = hold.copy().reset_index(drop=True)
        tr['a_filt'] = _causal_lowpass_by_trial(tr, tau)
        ho['a_filt'] = _causal_lowpass_by_trial(ho, tau)
        static_tr = _model_eval(static_model, tr.erpm.to_numpy(float))
        static_ho = _model_eval(static_model, ho.erpm.to_numpy(float))
        coeff = _ridge(
            _design_residual_features(static_tr, tr.a_filt.to_numpy(float), tr.motor_current.to_numpy(float)),
            tr.vx_truth.to_numpy(float) - static_tr,
            float(sel['ridge_lambda']),
        )
        adaptive_tr = _apply_adaptive(static_tr, tr.a_filt.to_numpy(float), tr.motor_current.to_numpy(float), coeff, float(sel['correction_limit_mps']))
        adaptive_ho = _apply_adaptive(static_ho, ho.a_filt.to_numpy(float), ho.motor_current.to_numpy(float), coeff, float(sel['correction_limit_mps']))
        candidates: dict[str, np.ndarray] = {
            'adaptive_wheel': adaptive_ho,
        }
        for wc in map(float, sel['fused_wheel_weight_coast']):
            for wh in map(float, sel['fused_wheel_weight_high_demand']):
                if wh > wc:
                    continue
                for at in map(float, sel['fused_accel_transition_mps2']):
                    key = f'fused_adaptive__tau{tau:.3f}__wc{wc:.3f}__wh{wh:.3f}__at{at:.3f}'
                    candidates[key] = _fused(
                        ho, adaptive_ho, ho.a_filt.to_numpy(float),
                        w_coast=wc, w_high=wh, accel_transition=at,
                        min_weight=float(sel['fused_min_wheel_weight']),
                    )
        for key, pred in candidates.items():
            reg = _regime_table(ho, pred, cfg)
            metrics = _metrics(ho.vx_truth.to_numpy(float), pred)
            ok, faults = _passes_regimes(reg, cfg)
            entry = {
                'prediction': pred,
                'coefficients': [float(v) for v in coeff],
                'accel_filter_tau_s': tau,
                'global': metrics,
                'regimes': reg,
                'passes_regime_gates': ok,
                'faults': faults,
                'score': _score(metrics, reg),
            }
            if key.startswith('fused_adaptive'):
                _, *parts = key.split('__')
                entry['fusion'] = {p.split('0', 1)[0]: p for p in parts}  # human trace only
                entry['wheel_weight_coast'] = wc
                entry['wheel_weight_high_demand'] = wh
                entry['accel_transition_mps2'] = at
                target_name = 'fused_adaptive'
            else:
                target_name = key
            previous = best.get(target_name)
            if previous is None or (entry['passes_regime_gates'], -entry['score']) > (previous['passes_regime_gates'], -previous['score']):
                best[target_name] = entry
    return best


def _candidate_patch(command_model: StaticModel, wheel_model: StaticModel,
                     selected_name: str, result: dict, cfg: dict, *, imu_ax_bias_mps2: float) -> dict:
    """Build the deployable candidate patch from independently selected maps.

    The command map and the wheel-speed observation may have different best
    structures.  That is intentional: desired-speed -> ERPM is an actuation
    inverse, while measured ERPM -> ground speed is a slip-corrupted sensor
    observation.  Both remain exactly zero at zero input.
    """
    command = command_model.params['command']
    odom = wheel_model.params['odom']
    command_kind = (
        'lut' if command_model.command_kind == 'lut'
        else ('quadratic' if abs(float(command.get('quadratic', 0.0))) > EPS else 'linear')
    )
    wheel_kind = (
        'lut' if wheel_model.odom_kind == 'lut'
        else ('quadratic' if abs(float(odom.get('quadratic', 0.0))) > EPS else 'linear')
    )
    ack: dict = {
        'speed_command_model': command_kind,
        'speed_to_erpm_gain': float(command.get('linear', 0.0)),
        'speed_to_erpm_quadratic': float(command.get('quadratic', 0.0)),
        'speed_to_erpm_offset': 0.0,
    }
    if command_model.command_kind == 'lut':
        ack['speed_command_lut_speed_mps'] = command['x_knots']
        ack['speed_command_lut_erpm'] = command['y_knots']

    od: dict = {
        'longitudinal_speed_model': selected_name,
        'odom_wheel_model': wheel_kind,
        'odom_erpm_to_speed_linear': float(odom.get('linear', 0.0)),
        'odom_erpm_to_speed_quadratic': float(odom.get('quadratic', 0.0)),
        'odom_accel_filter_tau_s': float(result.get('accel_filter_tau_s', 0.05)),
        'odom_imu_ax_bias_mps2': float(imu_ax_bias_mps2),
        'odom_speed_correction_limit_mps': float(
            cfg['analysis']['odometry_model_selection']['correction_limit_mps']
        ),
    }
    if wheel_model.odom_kind == 'lut':
        od['odom_erpm_lut_erpm'] = odom['x_knots']
        od['odom_erpm_lut_speed_mps'] = odom['y_knots']
    if selected_name in {'adaptive_wheel', 'fused_adaptive'}:
        coeff = result['coefficients']
        od.update({
            'odom_correction_accel_drive': coeff[0],
            'odom_correction_accel_brake': coeff[1],
            'odom_correction_current_drive': coeff[2],
            'odom_correction_current_brake': coeff[3],
            'odom_correction_accel_speed_drive': coeff[4],
            'odom_correction_accel_speed_brake': coeff[5],
        })
    if selected_name == 'fused_adaptive':
        od.update({
            'odom_fusion_wheel_weight_coast': float(result['wheel_weight_coast']),
            'odom_fusion_wheel_weight_high_demand': float(result['wheel_weight_high_demand']),
            'odom_fusion_accel_transition_mps2': float(result['accel_transition_mps2']),
            'odom_fusion_min_wheel_weight': float(
                cfg['analysis']['odometry_model_selection']['fused_min_wheel_weight']
            ),
        })
    return {
        'ackermann_to_vesc_node': {'ros__parameters': ack},
        'vesc_to_odom_node': {'ros__parameters': od},
    }


def _static_coverage(summary: pd.DataFrame, configured_speeds: list[float], repetitions: int) -> pd.DataFrame:
    """Coverage keyed by commanded nominal speed, never echo/ERPM float values."""
    rows: list[dict] = []
    for speed in map(float, configured_speeds):
        if summary.empty or 'nominal_speed_mps' not in summary:
            count = 0
        else:
            count = int(np.isclose(summary.nominal_speed_mps.to_numpy(float), speed, atol=1e-6).sum())
        rows.append({
            'nominal_speed_mps': speed,
            'accepted_usable_trials': count,
            'required_trials': int(repetitions),
            'coverage_ok': bool(count >= int(repetitions)),
        })
    return pd.DataFrame(rows)


def _legacy_wheel_model(session: Path, fallback: StaticModel) -> StaticModel:
    """Represent the installed scalar wheel estimate as a model-selection baseline.

    The calibration transaction forces the nonphysical global intercept to zero,
    so the baseline comparison is a zero-intercept scalar mapping using the
    original gain and odometry scale where available.
    """
    path = session / 'vesc_config_transaction' / 'vesc.yaml.original_bytes'
    try:
        import yaml
        doc = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        glob = doc.get('/**', {}).get('ros__parameters', {})
        od = doc.get('vesc_to_odom_node', {}).get('ros__parameters', {})
        gain = float(glob.get('speed_to_erpm_gain'))
        scale = float(od.get('odom_speed_scale', 1.0))
        if gain > EPS and np.isfinite(scale):
            c = scale / gain
            return StaticModel(
                name='legacy_scalar', command_kind=fallback.command_kind,
                odom_kind='poly',
                params={'command': fallback.params['command'], 'odom': {'linear': c, 'quadratic': 0.0}},
            )
    except Exception:
        pass
    return StaticModel(
        name='legacy_scalar', command_kind=fallback.command_kind,
        odom_kind='poly',
        params={'command': fallback.params['command'], 'odom': dict(fallback.params['odom'])},
    )

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('session', type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / 'calibration_config_snapshot.yaml')
    out = analysis_dir(session)
    gates = cfg['analysis']['gates']

    # ------------------------------------------------------------------
    # Static data: select command inverse and wheel observation separately.
    # ------------------------------------------------------------------
    static_train = _static_summary(session, '03_raw_erpm_map_training', 'raw_erpm_map_training', cfg)
    static_hold = _static_summary(session, '04_raw_erpm_map_holdout', 'raw_erpm_map_holdout', cfg)
    if static_train.empty or static_hold.empty:
        raise SystemExit('static model selection requires usable accepted Stage 3 and Stage 4 captures')

    train_cov = _static_coverage(
        static_train,
        list(cfg['raw_erpm_map_training']['nominal_speeds_mps']),
        int(cfg['raw_erpm_map_training']['repetitions']),
    )
    hold_cov = _static_coverage(
        static_hold,
        list(cfg['raw_erpm_map_holdout']['nominal_speeds_mps']),
        int(cfg['raw_erpm_map_holdout']['repetitions']),
    )
    train_cov.to_parquet(out / 'odometry_static_training_coverage.parquet', index=False)
    hold_cov.to_parquet(out / 'odometry_static_holdout_coverage.parquet', index=False)
    static_coverage_ok = bool(len(train_cov)) and bool(train_cov.coverage_ok.all()) and bool(len(hold_cov)) and bool(hold_cov.coverage_ok.all())

    models = _build_static_models(static_train, cfg)
    command_results: dict[str, dict] = {}
    wheel_results: dict[str, dict] = {}
    for name, model in models.items():
        command_pred = _model_command_eval(model, static_hold.vx_lidar_mps.to_numpy(float))
        odom_pred = _model_eval(model, static_hold.erpm_measured.to_numpy(float))
        cm = _metrics(static_hold.commanded_erpm.to_numpy(float), command_pred)
        command_results[name] = {
            'n': cm['n'],
            'rmse_erpm': cm['rmse_mps'],
            'bias_erpm': cm['bias_mps'],
            'p95_abs_error_erpm': cm['p95_abs_error_mps'],
        }
        wheel_results[name] = _metrics(static_hold.vx_lidar_mps.to_numpy(float), odom_pred)

    # The inverse command map is judged separately from the speed estimator.
    # ERPM is not m/s.  A map is instead compared by relative delivery residual:
    # use RMSE normalised by its own non-zero hold-out ERPM span and retain bias
    # for reporting. This avoids applying a velocity threshold to ERPM units.
    delivered = static_hold.commanded_erpm.to_numpy(float)
    command_span = float(np.nanquantile(np.abs(delivered), 0.90)) if len(delivered) else math.nan
    for name, m in command_results.items():
        relative_rmse = m['rmse_erpm'] / max(command_span, 1.0)
        relative_bias = m['bias_erpm'] / max(command_span, 1.0)
        command_results[name]['relative_rmse'] = relative_rmse
        command_results[name]['relative_bias'] = relative_bias
        command_results[name]['passes_delivery_gate'] = bool(
            np.isfinite(relative_rmse)
            and relative_rmse <= float(gates['max_command_map_relative_holdout_rmse'])
            and abs(relative_bias) <= float(gates['max_command_map_relative_holdout_bias'])
        )

    command_complexity = {'linear': 0, 'quadratic': 1, 'monotone_lut': 2}
    command_eligible = [n for n, r in command_results.items() if r['passes_delivery_gate']]
    if command_eligible:
        best = min(command_results[n]['relative_rmse'] for n in command_eligible)
        near = [n for n in command_eligible if command_results[n]['relative_rmse'] <= best * (1.0 + float(cfg['analysis']['static_map']['minimum_holdout_improvement_fraction']))]
        command_name = sorted(near, key=lambda n: command_complexity.get(n, 99))[0]
        command_accepted = True
        command_note = 'Selected independently from the wheel observation using zero-intercept Stage 4 delivery hold-outs.'
    else:
        command_name = min(command_results, key=lambda n: command_results[n]['relative_rmse'])
        command_accepted = False
        command_note = 'No zero-intercept command map passed the static delivery gate; best diagnostic candidate is reported only.'
    command_model = models[command_name]

    # `legacy_scalar` compares the installed scalar odometry against the new
    # candidates. It is not assumed to be the correct wheel model.
    legacy_model = _legacy_wheel_model(session, models['linear'])
    wheel_models: dict[str, StaticModel] = {
        'linear': models['linear'],
        'quadratic': models['quadratic'],
        'monotone_lut': models['monotone_lut'],
        'legacy_scalar': legacy_model,
    }
    wheel_results['legacy_scalar'] = _metrics(
        static_hold.vx_lidar_mps.to_numpy(float),
        _model_eval(legacy_model, static_hold.erpm_measured.to_numpy(float)),
    )

    # ------------------------------------------------------------------
    # Dynamic data: fit only training manoeuvres, select only on Stage 09.
    # ------------------------------------------------------------------
    train_stages = [
        ('06_raw_erpm_response', ['erpm_step_response']),
        ('08_raw_current_training', ['raw_drive_current_pulse', 'raw_brake_current_pulse']),
        ('10_accel_to_current_interface', ['accel_to_current_pulse']),
    ]
    hold_stages = [
        ('09_raw_current_holdout', ['raw_drive_current_pulse', 'raw_brake_current_pulse']),
    ]
    imu_ax_bias, imu_bias_report = _stationary_imu_ax_bias(session)
    dynamic_train = _dynamic_samples(session, train_stages, cfg)
    dynamic_hold = _dynamic_samples(session, hold_stages, cfg)
    if not dynamic_train.empty:
        dynamic_train = dynamic_train.copy()
        dynamic_train['imu_ax'] = dynamic_train['imu_ax'].to_numpy(float) - imu_ax_bias
    if not dynamic_hold.empty:
        dynamic_hold = dynamic_hold.copy()
        dynamic_hold['imu_ax'] = dynamic_hold['imu_ax'].to_numpy(float) - imu_ax_bias
    if dynamic_train.empty or dynamic_hold.empty:
        raise SystemExit('dynamic model selection requires usable accepted Stage 6/8/10 training and Stage 9 hold-out samples')

    all_results: dict[str, dict] = {}
    family_rank = {
        'legacy_scalar': 0,
        'static_linear': 1,
        'static_quadratic': 2,
        'static_lut': 3,
        'adaptive_wheel': 4,
        'fused_adaptive': 5,
    }
    wheel_label = {'linear': 'linear', 'quadratic': 'quadratic', 'monotone_lut': 'lut', 'legacy_scalar': 'legacy'}

    # Evaluate every wheel observation in every causal estimator family. This is
    # the central model-selection cross-product; the best dynamic estimator is
    # not forced to inherit the best calm/steady static map.
    for wname, wmodel in wheel_models.items():
        static_pred = _model_eval(wmodel, dynamic_hold.erpm.to_numpy(float))
        family = 'legacy_scalar' if wname == 'legacy_scalar' else f'static_{wheel_label[wname]}'
        reg = _regime_table(dynamic_hold.assign(a_filt=_causal_lowpass_by_trial(dynamic_hold.reset_index(drop=True), 0.05)), static_pred, cfg)
        gm = _metrics(dynamic_hold.vx_truth.to_numpy(float), static_pred)
        ok, faults = _passes_regimes(reg, cfg)
        static_metrics = wheel_results[wname]
        static_gate = (
            static_metrics['rmse_mps'] <= float(gates['max_odom_holdout_rmse_mps'])
            and abs(static_metrics['bias_mps']) <= float(gates['max_odom_holdout_bias_mps'])
            and static_metrics['p95_abs_error_mps'] <= float(gates['max_odom_holdout_p95_abs_error_mps'])
        )
        all_results[f'{family}__wheel_{wname}'] = {
            'family': family,
            'wheel_model_name': wname,
            'prediction': static_pred,
            'global': gm,
            'regimes': reg,
            'static_holdout': static_metrics,
            'passes_regime_gates': bool(ok and static_gate),
            'faults': list(faults) + ([] if static_gate else ['static Stage 4 odometry RMSE/bias/p95 gate']),
            'score': _score(gm, reg),
        }

    for wname in ['linear', 'quadratic', 'monotone_lut']:
        wmodel = wheel_models[wname]
        dynamic_models = _fit_dynamic_models(wmodel, dynamic_train, dynamic_hold, cfg)
        for family, value in dynamic_models.items():
            entry = dict(value)
            entry['family'] = family
            entry['wheel_model_name'] = wname
            static_metrics = wheel_results[wname]
            static_gate = (
                static_metrics['rmse_mps'] <= float(gates['max_odom_holdout_rmse_mps'])
                and abs(static_metrics['bias_mps']) <= float(gates['max_odom_holdout_bias_mps'])
                and static_metrics['p95_abs_error_mps'] <= float(gates['max_odom_holdout_p95_abs_error_mps'])
            )
            entry['static_holdout'] = static_metrics
            entry['passes_regime_gates'] = bool(entry['passes_regime_gates'] and static_gate)
            if not static_gate:
                entry['faults'] = list(entry['faults']) + ['static Stage 4 odometry RMSE/bias/p95 gate']
            all_results[f'{family}__wheel_{wname}'] = entry

    rows: list[dict] = []
    for label, value in all_results.items():
        rows.append({
            'model_label': label,
            'family': value['family'],
            'wheel_model': value['wheel_model_name'],
            'score': float(value['score']),
            'passes_regime_gates': bool(value['passes_regime_gates']),
            **value['global'],
            'static_rmse_mps': float(value['static_holdout']['rmse_mps']),
            'static_bias_mps': float(value['static_holdout']['bias_mps']),
            'fault_count': len(value['faults']),
        })
        reg = value['regimes'].copy()
        reg['model_label'] = label
        reg['family'] = value['family']
        reg['wheel_model'] = value['wheel_model_name']
        reg.to_parquet(out / f'odometry_model_regimes_{label}.parquet', index=False)
    leaderboard = pd.DataFrame(rows).sort_values(['passes_regime_gates', 'score'], ascending=[False, True])
    leaderboard.to_parquet(out / 'odometry_model_leaderboard.parquet', index=False)

    eligible = leaderboard[leaderboard.passes_regime_gates.astype(bool)].copy()
    if eligible.empty:
        selected_label = str(leaderboard.iloc[0].model_label)
        accepted = False
        selection_note = 'No candidate passed every static and dynamic regime gate. The least-bad candidate is reported for diagnosis only.'
    else:
        best_score = float(eligible.iloc[0].score)
        tie = eligible[eligible.score <= best_score * (1.0 + float(cfg['analysis']['odometry_model_selection']['tie_score_fraction']))]
        selected_label = str(sorted(
            tie.model_label.astype(str).tolist(),
            key=lambda label: (
                family_rank.get(all_results[label]['family'], 99),
                command_complexity.get(all_results[label]['wheel_model_name'], 99),
                label,
            ),
        )[0])
        accepted = True
        selection_note = 'Selected from the full command-map/wheel-observation/causal-estimator cross-product on complete held-out trajectories; complexity broke only a near score tie.'

    selected = all_results[selected_label]
    wheel_model = wheel_models[selected['wheel_model_name']]
    patch = _candidate_patch(command_model, wheel_model, selected['family'], selected, cfg, imu_ax_bias_mps2=imu_ax_bias)
    dump_yaml(out / 'selected_odometry_candidate_patch.yaml', patch)
    report = {
        'objective': 'Minimise held-out LiDAR-ground-speed error across calm, high-drive and high-brake regimes using a bounded causal estimator.',
        'static_training_coverage_ok': bool(len(train_cov)) and bool(train_cov.coverage_ok.all()),
        'static_holdout_coverage_ok': bool(len(hold_cov)) and bool(hold_cov.coverage_ok.all()),
        'static_coverage_ok': static_coverage_ok,
        'stationary_imu_ax_bias': imu_bias_report,
        'command_map_selected': command_name,
        'command_map_results': command_results,
        'command_map_accepted': command_accepted,
        'command_map_note': command_note,
        'wheel_observation_results': wheel_results,
        'selected_model_label': selected_label,
        'selected_family': selected['family'],
        'selected_wheel_observation': selected['wheel_model_name'],
        'selected_global_holdout': selected['global'],
        'selected_static_holdout': selected['static_holdout'],
        'selected_regime_holdout_file': f'odometry_model_regimes_{selected_label}.parquet',
        'selected_faults': selected['faults'],
        'accepted_for_shadow_deployment_verification': bool(accepted and command_accepted and static_coverage_ok),
        'selection_note': selection_note,
        'candidate_patch_file': 'selected_odometry_candidate_patch.yaml',
        'candidate_contract': {
            'zero_speed_command_erpm': 0.0,
            'no_global_erpm_offset': True,
            'command_map_and_wheel_observation_are_separate': True,
            'only_deployable_causal_models_were_considered': True,
            'model_selected_on_whole_held_out_trials': True,
            'fit_measurement_unit': 'robust multi-registration LiDAR motion window',
            'session_stationary_imu_bias_applied': False,
        },
    }
    dump_yaml(out / 'odometry_model_selection_report.yaml', report)
    print(json.dumps(report, indent=2, default=str))
    return 0 if report['accepted_for_shadow_deployment_verification'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
