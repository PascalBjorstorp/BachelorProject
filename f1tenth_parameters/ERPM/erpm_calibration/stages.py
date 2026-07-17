"""Operator-driven, bag-first full-envelope longitudinal calibration stages.

The campaign separates three effects that must not be collapsed into one fit:

* zero-intercept quasi-static ERPM/wheel-speed mapping;
* speed-dependent losses at zero motor current; and
* transient current/traction/slip behaviour at high demand.

Runtime odometry is used only for safety and wait-until-stable scheduling.  All
identified ground-speed and acceleration quantities are computed offline from
raw LiDAR scans and IMU/VESC telemetry.
"""
from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Callable, Iterable

import numpy as np

from .config import dump_json
from .runtime import CalibrationNode, finish_node, start_node
from .ui import banner, checklist, note, pause_for_reposition, require_ready, review_trial

WINDOW = (
    'imu_ax', 'imu_ay', 'imu_gz', 'odom_vx', 'odom_vy', 'candidate_odom_vx', 'candidate_odom_vy', 'erpm',
    'motor_current_a', 'input_current_a', 'battery_v', 'motor_temp_c',
    'fet_temp_c', 'selected_speed_erpm', 'selected_current_a',
    'selected_brake_a', 'straight_assist_trim_rad',
)


class TrialCounter:
    def __init__(self, cfg: dict[str, Any]) -> None:
        self.cfg = cfg
        self.accepted = 0

    def count(self, node: CalibrationNode) -> None:
        self.accepted += 1
        every = int(self.cfg['session']['cooling_pause_every_accepted_trials'])
        if every > 0 and self.accepted % every == 0:
            seconds = float(self.cfg['session']['cooling_pause_s'])
            node.event.emit('cooling_pause_start', accepted_trials=self.accepted, duration_s=seconds)
            print(f'\nCooling pause: {seconds:.0f} s after {self.accepted} accepted motion trials.')
            node.neutral()
            node.spin(seconds)
            node.event.emit('cooling_pause_end', accepted_trials=self.accepted)


def _id(prefix: str, attempt: int) -> str:
    return f'{prefix}__attempt_{attempt:02d}'


def _raw_erpm(speed_to_erpm_gain: float, _ignored_offset: float, speed_mps: float) -> float:
    """Generate a raw ERPM target with a physically constrained zero intercept.

    The calibration deliberately never applies the existing YAML offset while
    creating raw plateau levels.  A non-zero global offset would imply a
    non-zero wheel-speed target at a zero ground-speed request.  Launch
    compensation is characterised separately in Stage 2/5 through slow-start
    behaviour, not hidden inside the static ERPM map.
    """
    return float(speed_to_erpm_gain * speed_mps)


def _pre_capture_settle_s(cfg: dict[str, Any], speed_mps: float) -> float:
    startup = cfg.get('motion_startup', {})
    distance = max(0.0, float(startup.get('pre_capture_settle_distance_m', 0.0)))
    minimum = max(0.0, float(startup.get('pre_capture_settle_min_s', 0.0)))
    maximum = max(minimum, float(startup.get('pre_capture_settle_max_s', minimum)))
    speed = abs(float(speed_mps))
    high_threshold = float(startup.get('pre_capture_settle_high_speed_threshold_mps', 4.0))
    high_minimum = float(startup.get('pre_capture_settle_high_speed_min_s', 0.0))
    if speed >= high_threshold and high_minimum > 0.0:
        minimum = max(minimum, high_minimum)
    if distance <= 0.0:
        return min(maximum, minimum)
    by_distance = distance / max(speed, 0.05)
    return min(maximum, max(minimum, by_distance))


def _capture_for_speed(
    spec: dict[str, Any], speed_mps: float, *, map_key: str, default_key: str,
) -> float:
    """Return the frozen room-derived duration for one configured speed.

    Legacy standalone configurations do not contain a generated map, so they
    retain their reviewed scalar duration.  Unified-session snapshots contain
    exact per-condition values and are matched numerically to avoid depending
    on whether YAML loaded a key as a string or a float.
    """
    default = float(spec[default_key])
    values = spec.get(map_key, {})
    if not isinstance(values, dict) or not values:
        return default
    requested = float(speed_mps)
    candidates: list[tuple[float, float]] = []
    for key, value in values.items():
        try:
            candidates.append((abs(float(key) - requested), float(value)))
        except (TypeError, ValueError):
            continue
    if not candidates:
        return default
    error, duration = min(candidates, key=lambda item: item[0])
    if error > 1.0e-6 * max(1.0, abs(requested)):
        return default
    if not math.isfinite(duration) or duration <= 0.0:
        raise RuntimeError(f'{map_key} contains an invalid duration for {requested:.3f} m/s')
    return duration


def _full_circle_plan(cfg: dict[str, Any], *, speed_mps: float,
                      radius_m: float, minimum_capture_s: float) -> dict[str, Any]:
    """Plan one footprint-aware constant-radius revolution when it fits."""
    policy = cfg.get('room_capture_policy', {})
    if not isinstance(policy, dict) or not policy:
        return {
            'capture_mode': 'bounded_arc',
            'capture_s': float(minimum_capture_s),
            'planned_revolutions': 0.0,
            'nominal_radius_m': float(radius_m),
        }
    speed = abs(float(speed_mps))
    radius = float(radius_m)
    body_radius = float(policy['vehicle_circumscribed_radius_m'])
    braking = float(policy['conservative_braking_mps2'])
    target_side = float(policy['target_circle_side_m'])
    stop_extension = speed * speed / (2.0 * braking)
    max_radius = 0.5 * (target_side - 2.0 * body_radius - stop_extension)
    revolutions = float(policy.get('full_circle_revolutions', 1.0))
    if radius <= max_radius + 1.0e-9:
        duration = max(float(minimum_capture_s), revolutions * 2.0 * math.pi * radius / speed)
        return {
            'capture_mode': 'full_circle',
            'capture_s': duration,
            'planned_revolutions': revolutions,
            'nominal_radius_m': radius,
            'maximum_fitting_radius_m': max_radius,
            'planned_stop_extension_m': stop_extension,
        }
    return {
        'capture_mode': 'bounded_arc',
        'capture_s': float(minimum_capture_s),
        'planned_revolutions': 0.0,
        'nominal_radius_m': radius,
        'maximum_fitting_radius_m': max_radius,
        'planned_stop_extension_m': stop_extension,
    }


def _decision(
    node: CalibrationNode, *, stage: str, condition_id: str, trial_id: str,
    attempt: int, auto_ok: bool, summary: dict[str, Any],
) -> str:
    effective_auto_ok = auto_ok and summary.get('straight_runtime_gate') is not False
    decision = review_trial(
        label=f'{stage}: {condition_id}, attempt {attempt}',
        automatic_ok=effective_auto_ok,
        automatic_summary=summary,
    )
    node.event.emit(
        'trial_decision', stage=stage, condition_id=condition_id,
        trial_id=trial_id, attempt=attempt, decision=decision,
        accepted=decision == 'accepted', automatic_ok=effective_auto_ok,
    )
    if decision == 'redo' and not effective_auto_ok:
        node.fail_stop()
        node.event.emit(
            'failed_attempt_reset_required', stage=stage,
            condition_id=condition_id, trial_id=trial_id, attempt=attempt,
        )
        require_ready(
            'FAILED ATTEMPT STOPPED. Reposition the car at the start; '
            'type READY only after it is physically reset, or ABORT'
        )
        node.event.emit(
            'failed_attempt_reset_confirmed', stage=stage,
            condition_id=condition_id, trial_id=trial_id, attempt=attempt,
        )
    return decision


def _capture_speed_ok(summary: dict[str, Any] | None, target_mps: float) -> bool:
    if summary is None:
        return False
    try:
        measured_f = float(summary.get('odom_vx_mean'))
    except (TypeError, ValueError):
        measured_f = math.nan
    tolerance = max(0.25, 0.15 * abs(float(target_mps)))
    odom_ok = math.isfinite(measured_f) and abs(measured_f - float(target_mps)) <= tolerance
    try:
        erpm_f = float(summary.get('erpm_mean'))
        selected_f = float(summary.get('selected_speed_erpm_mean'))
    except (TypeError, ValueError):
        return odom_ok
    erpm_tol = max(150.0, 0.12 * abs(selected_f))
    erpm_ok = (
        math.isfinite(erpm_f)
        and math.isfinite(selected_f)
        and abs(selected_f) > 1.0
        and abs(erpm_f - selected_f) <= erpm_tol
    )
    return odom_ok or erpm_ok


def _straight_ok(node: CalibrationNode, cfg: dict[str, Any]) -> bool:
    """Runtime straight-line gate.

    A failed straight gate forces REDO through ``_decision``. The car may keep
    correcting with the assist, but a trial with excessive yaw/lateral motion is
    not valid longitudinal calibration evidence.
    """
    p = cfg['preflight']
    return (
        math.isfinite(node.latest.imu_gz)
        and math.isfinite(node.latest.imu_ay)
        and abs(node.latest.imu_gz) <= float(p['max_straight_yaw_rate_rad_s'])
        and abs(node.latest.imu_ay) <= float(p['max_straight_lateral_accel_mps2'])
    )



def _candidate_command_erpm(speed_mps: float, patch: dict[str, Any]) -> float:
    """Evaluate the selected zero-intercept candidate command map for staging.

    Stage 12 uses raw ERPM only to establish a repeatable initial-speed state
    before the candidate acceleration mapper takes over.  It must still use
    the selected command map rather than silently falling back to a scalar
    approximation when a quadratic/LUT command map won Stage 4.
    """
    params = patch.get('ackermann_to_vesc_node', {}).get('ros__parameters', {})
    kind = str(params.get('speed_command_model', 'linear')).lower()
    v = float(speed_mps)
    if kind == 'lut':
        x = np.asarray(params.get('speed_command_lut_speed_mps', []), dtype=float)
        y = np.asarray(params.get('speed_command_lut_erpm', []), dtype=float)
        if len(x) >= 2 and len(x) == len(y) and np.all(np.diff(x) > 0.0):
            sign = 1.0 if v >= 0.0 else -1.0
            av = abs(v)
            val = float(np.interp(av, x, y))
            if av > x[-1]:
                val = float(y[-1] + (y[-1] - y[-2]) / max(x[-1] - x[-2], 1e-9) * (av - x[-1]))
            return sign * val
    gain = float(params.get('speed_to_erpm_gain', 0.0))
    quad = float(params.get('speed_to_erpm_quadratic', 0.0))
    return gain * v + quad * v * abs(v)


def _initial_erpm(speed_mps: float, gain: float, offset: float,
                  speed_command_patch: dict[str, Any] | None) -> float:
    """Evaluate the best currently known speed->ERPM map for setup speed holds.

    Early stages must start from the installed scalar map because no campaign
    evidence exists yet. Later stages can benefit from an interim command-map
    estimate, improving initial-speed establishment for response, coast-down,
    raw-current and acceleration-interface tests without requiring manual
    operator retuning between phases.
    """
    return _candidate_command_erpm(speed_mps, speed_command_patch) if speed_command_patch else _raw_erpm(gain, offset, speed_mps)

def _require_envelope(cfg: dict[str, Any], *, speed: float | None = None,
                      acceleration: float | None = None,
                      brake: bool = False) -> None:
    env = cfg['operating_envelope']
    if speed is not None and abs(float(speed)) > float(env['maximum_test_speed_mps']) + 1e-9:
        raise RuntimeError(f'configured command {speed:.3f} m/s exceeds maximum_test_speed_mps')
    if acceleration is not None:
        requested = float(acceleration)
        if not math.isfinite(requested):
            raise RuntimeError(f'configured acceleration {acceleration!r} is not finite')
        limit_key = 'maximum_test_brake_mps2' if brake else 'maximum_test_accel_mps2'
        limit = float(env.get(limit_key, math.inf))
        if abs(requested) > limit + 1e-9:
            raise RuntimeError(f'configured acceleration {requested:.3f} m/s² exceeds {limit_key}={limit:.3f} m/s²')


def _pulse_duration(cfg: dict[str, Any], fraction: float) -> float:
    spec = cfg['operating_envelope']['high_demand_pulse_duration_s']
    f = abs(float(fraction))
    if f <= 0.25:
        return float(spec['low_fraction'])
    if f <= 0.55:
        return float(spec['medium_fraction'])
    return float(spec['high_fraction'])


def _safe_pulse_duration(cfg: dict[str, Any], fraction: float, *, initial_speed: float, polarity: str) -> float:
    """Return the configured pulse time without imposing a software accel cap."""
    duration = _pulse_duration(cfg, fraction)
    minimum = float(cfg['operating_envelope']['dynamic_capture_min_s'])
    if duration + 1e-9 < minimum:
        raise RuntimeError(
            f'configured {polarity} pulse duration {duration:.3f} s for initial speed '
            f'{initial_speed:.3f} m/s, fraction {fraction:.3f} is below the minimum '
            f'identifiable duration {minimum:.3f} s.'
        )
    return duration


def _run_raw_erpm_plateau(
    node: CalibrationNode, *, cfg: dict[str, Any], counter: TrialCounter,
    stage: str, condition_id: str, raw_erpm: float, nominal_speed: float,
    capture_s: float, phase: str,
) -> list[dict[str, Any]]:
    _require_envelope(cfg, speed=nominal_speed)
    records: list[dict[str, Any]] = []
    attempt = 1
    while True:
        trial = _id(condition_id, attempt)
        pause_for_reposition(
            f'Position car at the straight-line start. {condition_id}; '
            f'raw target={raw_erpm:.1f} ERPM.\nAttempt {attempt}; REDO has no limit.'
        )
        node.event.emit(
            'trial_start', stage=stage, condition_id=condition_id,
            trial_id=trial, attempt=attempt, nominal_speed_mps=nominal_speed,
            raw_erpm_target=raw_erpm, planned_capture_s=float(capture_s),
        )
        startup = node.establish_raw_erpm(
            target_erpm=raw_erpm, startup_speed_mps=nominal_speed,
            segment_id=condition_id, trial_id=trial,
        )
        summary: dict[str, Any] | None = None
        settle_s = 0.0
        if startup.get('stable'):
            settle_s = _pre_capture_settle_s(cfg, nominal_speed)
            if settle_s > 0.0:
                node.hold(
                    kind='raw_erpm', target=raw_erpm, duration_s=settle_s,
                    phase=f'{phase}_pre_capture_settle',
                    segment_id=condition_id, trial_id=trial,
                    capture=False, window_fields=(),
                    nominal_speed_mps=nominal_speed, raw_erpm_target=raw_erpm,
                    pre_capture_settle_s=settle_s,
                )
            summary = node.hold(
                kind='raw_erpm', target=raw_erpm, duration_s=capture_s,
                phase=phase, segment_id=condition_id, trial_id=trial,
                capture=True, window_fields=WINDOW,
                nominal_speed_mps=nominal_speed, raw_erpm_target=raw_erpm,
                planned_capture_s=float(capture_s),
            )
        straight = _straight_ok(node, cfg) if summary is not None else False
        node.active_stop()
        speed_gate = _capture_speed_ok(summary, nominal_speed)
        auto = bool(startup.get('stable')) and summary is not None and speed_gate
        decision = _decision(
            node, stage=stage, condition_id=condition_id, trial_id=trial,
            attempt=attempt, auto_ok=auto,
            summary={'startup': startup, 'pre_capture_settle_s': settle_s, 'straight_runtime_gate': straight, 'capture_speed_gate': speed_gate, 'capture': summary or {}},
        )
        records.append({
            'trial_id': trial, 'attempt': attempt, 'decision': decision,
            'startup': startup, 'capture': summary,
            'pre_capture_settle_s': settle_s,
            'planned_capture_s': float(capture_s),
            'raw_erpm_target': raw_erpm, 'nominal_speed_mps': nominal_speed,
        })
        if decision == 'accepted':
            counter.count(node)
            return records
        if decision == 'skipped':
            return records
        attempt += 1


def _run_ackermann_plateau(
    node: CalibrationNode, *, cfg: dict[str, Any], counter: TrialCounter,
    stage: str, condition_id: str, speed: float, capture_s: float, phase: str,
) -> list[dict[str, Any]]:
    _require_envelope(cfg, speed=speed)
    records: list[dict[str, Any]] = []
    attempt = 1
    while True:
        trial = _id(condition_id, attempt)
        pause_for_reposition(
            f'Position car at the straight-line start. {condition_id}; '
            f'speed command={speed:.3f} m/s.\nAttempt {attempt}; REDO has no limit.'
        )
        node.event.emit(
            'trial_start', stage=stage, condition_id=condition_id,
            trial_id=trial, attempt=attempt, speed_command_mps=speed,
            planned_capture_s=float(capture_s),
        )
        startup = node.establish_ackermann_speed(
            speed_mps=speed, segment_id=condition_id, trial_id=trial,
        )
        summary: dict[str, Any] | None = None
        settle_s = 0.0
        if startup.get('stable'):
            settle_s = _pre_capture_settle_s(cfg, speed)
            if settle_s > 0.0:
                node.hold(
                    kind='ackermann_speed', target=speed, duration_s=settle_s,
                    phase=f'{phase}_pre_capture_settle',
                    segment_id=condition_id, trial_id=trial,
                    capture=False, window_fields=(),
                    speed_command_mps=speed,
                    pre_capture_settle_s=settle_s,
                )
            summary = node.hold(
                kind='ackermann_speed', target=speed, duration_s=capture_s,
                phase=phase, segment_id=condition_id, trial_id=trial,
                capture=True, window_fields=WINDOW, speed_command_mps=speed,
                planned_capture_s=float(capture_s),
            )
        straight = _straight_ok(node, cfg) if summary is not None else False
        node.active_stop()
        speed_gate = _capture_speed_ok(summary, speed)
        observability_probe = stage == '01_longitudinal_observability'
        auto = bool(startup.get('stable')) and summary is not None and (speed_gate or observability_probe)
        decision = _decision(
            node, stage=stage, condition_id=condition_id, trial_id=trial,
            attempt=attempt, auto_ok=auto,
            summary={
                'startup': startup,
                'pre_capture_settle_s': settle_s,
                'straight_runtime_gate': straight,
                'capture_speed_gate': speed_gate,
                'observability_allows_speed_sensor_mismatch': observability_probe and not speed_gate,
                'capture': summary or {},
            },
        )
        records.append({
            'trial_id': trial, 'attempt': attempt, 'decision': decision,
            'startup': startup, 'capture': summary, 'pre_capture_settle_s': settle_s,
            'speed_command_mps': speed, 'planned_capture_s': float(capture_s),
        })
        if decision == 'accepted':
            counter.count(node)
            return records
        if decision == 'skipped':
            return records
        attempt += 1


def command_chain_audit(cfg: dict[str, Any], stage_dir: Path) -> dict[str, Any]:
    banner('STAGE 0 — MOTOR COMMAND-CHAIN AUDIT', 'Car must be lifted; wheels may rotate briefly at low ERPM.')
    checklist([
        'Car is securely on a stand.',
        'No MPC/MPCC/planner/teleoperation/normal bringup is running.',
        'Emergency stop is immediately reachable.',
    ])
    require_ready()
    node = start_node('erpm_command_chain_audit', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    p = cfg['preflight']
    results: list[dict[str, Any]] = []
    try:
        delta = float(p['min_command_change_erpm'])
        for label, target in [('zero', 0.0), ('positive', delta), ('zero_return', 0.0)]:
            summary = node.hold(
                kind='raw_erpm', target=target, duration_s=0.6,
                phase='command_chain_audit', segment_id=label,
                trial_id=f'audit_{label}', capture=True, window_fields=WINDOW,
                raw_erpm_target=target,
            )
            results.append({'label': label, 'command_kind': 'raw_erpm', 'target': target, **summary})
        current_delta = float(p['min_command_change_current_a'])
        for label, kind, target in [
            ('current_zero', 'raw_current', 0.0),
            ('current_positive', 'raw_current', current_delta),
            ('brake_zero', 'raw_brake', 0.0),
            ('brake_positive', 'raw_brake', current_delta),
            ('neutral', 'neutral', 0.0),
        ]:
            summary = node.hold(
                kind=kind, target=target, duration_s=0.35,
                phase='command_chain_audit', segment_id=label,
                trial_id=f'audit_{label}', capture=True, window_fields=WINDOW,
            )
            results.append({'label': label, 'command_kind': kind, 'target': target, **summary})
        pos = next(x for x in results if x['label'] == 'positive')
        erpm_err = abs(float(pos['selected_speed_erpm_mean']) - delta)
        cur = next(x for x in results if x['label'] == 'current_positive')
        current_err = abs(float(cur['selected_current_a_mean']) - current_delta)
        brk = next(x for x in results if x['label'] == 'brake_positive')
        brake_err = abs(float(brk['selected_brake_a_mean']) - current_delta)
        if erpm_err > float(p['raw_motor_tolerance_erpm']):
            raise RuntimeError(f'motor selector raw ERPM error {erpm_err:.2f} exceeds tolerance')
        if max(current_err, brake_err) > float(p['raw_motor_tolerance_current_a']):
            raise RuntimeError('motor selector raw current/brake error exceeds tolerance: '
                               f'current={current_err:.2f}, brake={brake_err:.2f} A')
        result = {
            'status': 'pass', 'selected_erpm_error': erpm_err,
            'selected_current_error_a': current_err,
            'selected_brake_error_a': brake_err, 'samples': results,
        }
        dump_json(stage_dir / 'runtime_result.json', result)
        print('PASS — selector errors: '
              f'ERPM={erpm_err:.2f}, current={current_err:.2f} A, brake={brake_err:.2f} A')
        return result
    finally:
        finish_node(node)


def longitudinal_observability(cfg: dict[str, Any], stage_dir: Path,
                                gain: float, offset: float,
                                counter: TrialCounter) -> dict[str, Any]:
    banner('STAGE 1 — LONGITUDINAL SENSOR OBSERVABILITY',
           'Stationary noise floor plus low-, medium-, and high-speed straight LiDAR/IMU checks.')
    require_ready('Place car stationary on the floor. Type READY to capture stationary data, or ABORT')
    node = start_node('erpm_observability', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    try:
        stationary = node.hold(
            kind='neutral', target=0.0,
            duration_s=float(cfg['observability']['stationary_duration_s']),
            phase='stationary_observability', segment_id='stationary',
            trial_id='stationary', capture=True, window_fields=WINDOW,
        )
        probes: list[dict[str, Any]] = []
        for speed in cfg['observability']['straight_probe_speeds_mps']:
            for rep in range(1, int(cfg['observability']['straight_probe_repetitions']) + 1):
                cid = f'observability_speed_{float(speed):.3f}_rep_{rep:02d}'
                probes += _run_ackermann_plateau(
                    node, cfg=cfg, counter=counter,
                    stage='01_longitudinal_observability', condition_id=cid,
                    speed=float(speed),
                    capture_s=_capture_for_speed(
                        cfg['observability'], float(speed),
                        map_key='straight_probe_capture_s_by_speed',
                        default_key='straight_probe_capture_s',
                    ),
                    phase='straight_observability',
                )
        result = {'stationary': stationary, 'probes': probes}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def low_speed_launch(cfg: dict[str, Any], stage_dir: Path, gain: float,
                     offset: float, counter: TrialCounter) -> dict[str, Any]:
    banner('STAGE 2 — LOW-SPEED LAUNCH / DEAD-BAND',
           'Find the smallest repeatable ground-motion command and launch delay. This is separate from the zero-intercept ERPM map.')
    node = start_node('erpm_low_speed_launch', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['low_speed_launch']
        for speed in spec['nominal_speeds_mps']:
            for rep in range(1, int(spec['repetitions']) + 1):
                cid = f'launch_speed_{float(speed):.3f}_rep_{rep:02d}'
                records += _run_raw_erpm_plateau(
                    node, cfg=cfg, counter=counter, stage='02_low_speed_launch',
                    condition_id=cid,
                    raw_erpm=_raw_erpm(gain, offset, float(speed)),
                    nominal_speed=float(speed), capture_s=_capture_for_speed(
                        spec, float(speed), map_key='capture_s_by_speed', default_key='capture_s',
                    ),
                    phase='low_speed_launch',
                )
        result = {'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def raw_erpm_map(cfg: dict[str, Any], stage_dir: Path, gain: float,
                 offset: float, counter: TrialCounter, *, holdout: bool) -> dict[str, Any]:
    num = '04' if holdout else '03'
    title = 'HOLD-OUT' if holdout else 'TRAINING'
    banner(f'STAGE {num} — FULL-ENVELOPE ZERO-INTERCEPT ERPM MAP {title}',
           'Raw ERPM is the input; LiDAR ground speed is the reference. The later fit is constrained through the origin and compares linear vs quadratic static maps.')
    node = start_node('erpm_map_holdout' if holdout else 'erpm_map_training', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['raw_erpm_map_holdout' if holdout else 'raw_erpm_map_training']
        stage = f'{num}_raw_erpm_map_{"holdout" if holdout else "training"}'
        phase = 'raw_erpm_holdout' if holdout else 'raw_erpm_training'
        levels = list(map(float, spec['nominal_speeds_mps']))
        ordered: list[float] = []
        for i in range((len(levels) + 1) // 2):
            ordered.append(levels[i])
            j = len(levels) - 1 - i
            if j != i:
                ordered.append(levels[j])
        for speed in ordered:
            for rep in range(1, int(spec['repetitions']) + 1):
                cid = f'{"holdout" if holdout else "training"}_speed_{speed:.3f}_rep_{rep:02d}'
                records += _run_raw_erpm_plateau(
                    node, cfg=cfg, counter=counter, stage=stage,
                    condition_id=cid, raw_erpm=_raw_erpm(gain, offset, speed),
                    nominal_speed=speed, capture_s=_capture_for_speed(
                        spec, speed, map_key='capture_s_by_speed', default_key='capture_s',
                    ), phase=phase,
                )
        result = {'holdout': holdout, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def vel_to_erpm_pipeline_audit(cfg: dict[str, Any], stage_dir: Path,
                               counter: TrialCounter) -> dict[str, Any]:
    banner('STAGE 5 — FULL-ENVELOPE VEL_TO_ERPM PIPELINE AUDIT',
           'Audits desired-speed → AckermannToVesc → selected ERPM → VESC ERPM → ground speed over the operating envelope.')
    node = start_node('erpm_vel_pipeline', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['vel_to_erpm_pipeline_audit']
        for speed in spec['speed_commands_mps']:
            for rep in range(1, int(spec['repetitions']) + 1):
                records += _run_ackermann_plateau(
                    node, cfg=cfg, counter=counter,
                    stage='05_vel_to_erpm_pipeline_audit',
                    condition_id=f'vel_command_{float(speed):.3f}_rep_{rep:02d}',
                    speed=float(speed), capture_s=_capture_for_speed(
                        spec, float(speed), map_key='capture_s_by_speed', default_key='capture_s',
                    ),
                    phase='vel_to_erpm_pipeline',
                )
        result = {'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def raw_erpm_response(cfg: dict[str, Any], stage_dir: Path, gain: float,
                      offset: float, counter: TrialCounter,
                      speed_command_patch: dict[str, Any] | None = None,
                      *, validation: bool = False) -> dict[str, Any]:
    banner('ERPM / GROUND-SPEED RESPONSE HOLD-OUT' if validation else 'ERPM / GROUND-SPEED STEP RESPONSE TRAINING',
           'Distinct speed steps validate the response model without refitting it.' if validation
           else 'Identifies command delivery, VESC ERPM tracking, and ground-speed delay/rate.')
    node = start_node('erpm_step_response_validation' if validation else 'erpm_step_response', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['raw_erpm_response']
        steps = spec.get('validation_steps_mps', []) if validation else spec['steps_mps']
        repetitions = int(spec.get('validation_repetitions', spec['repetitions'])) if validation else int(spec['repetitions'])
        if not steps:
            raise RuntimeError('ERPM response validation has no configured hold-out steps')
        for baseline_speed, target_speed in steps:
            _require_envelope(cfg, speed=float(baseline_speed))
            _require_envelope(cfg, speed=float(target_speed))
            baseline_erpm = _initial_erpm(float(baseline_speed), gain, offset, speed_command_patch)
            target_erpm = _initial_erpm(float(target_speed), gain, offset, speed_command_patch)
            response_capture_s = _capture_for_speed(
                spec, float(target_speed),
                map_key='response_capture_s_by_target_speed',
                default_key='response_capture_s',
            )
            for rep in range(1, repetitions + 1):
                cid = f"{'erpm_response_validation' if validation else 'erpm_step'}_{float(baseline_speed):.3f}_to_{float(target_speed):.3f}_rep_{rep:02d}"
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit(
                        'trial_start', stage='06a_raw_erpm_response_validation' if validation else '06_raw_erpm_response',
                        condition_id=cid, trial_id=trial, attempt=attempt,
                        baseline_erpm=baseline_erpm, target_erpm=target_erpm,
                        baseline_speed_mps=float(baseline_speed), target_speed_mps=float(target_speed),
                        planned_response_capture_s=response_capture_s,
                    )
                    startup = node.establish_raw_erpm(target_erpm=baseline_erpm, startup_speed_mps=float(baseline_speed), segment_id=cid, trial_id=trial)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        node.hold(
                            kind='raw_erpm', target=baseline_erpm,
                            duration_s=float(spec['pre_step_hold_s']),
                            phase='erpm_step_baseline', segment_id=cid, trial_id=trial,
                            capture=True, window_fields=WINDOW,
                            baseline_erpm=baseline_erpm, target_erpm=target_erpm,
                        )
                        summary = node.hold(
                            kind='raw_erpm', target=target_erpm,
                            duration_s=response_capture_s,
                            phase='erpm_step_response', segment_id=cid, trial_id=trial,
                            capture=True, window_fields=WINDOW,
                            baseline_erpm=baseline_erpm, target_erpm=target_erpm,
                            baseline_speed_mps=float(baseline_speed), target_speed_mps=float(target_speed),
                            planned_capture_s=response_capture_s,
                        )
                    straight = _straight_ok(node, cfg) if summary else False
                    node.active_stop()
                    auto = bool(startup.get('stable')) and summary is not None
                    decision = _decision(
                        node, stage='06a_raw_erpm_response_validation' if validation else '06_raw_erpm_response', condition_id=cid,
                        trial_id=trial, attempt=attempt, auto_ok=auto,
                        summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
                    )
                    records.append({
                        'trial_id': trial, 'condition_id': cid, 'decision': decision,
                        'startup': startup, 'capture': summary,
                        'planned_capture_s': response_capture_s,
                    })
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {'validation': validation, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def coastdown(cfg: dict[str, Any], stage_dir: Path, gain: float,
              offset: float, counter: TrialCounter,
              speed_command_patch: dict[str, Any] | None = None,
              *, validation: bool = False) -> dict[str, Any]:
    banner('COAST-DOWN DRAG HOLD-OUT' if validation else 'STAGE 7 — FULL-ENVELOPE COAST-DOWN / DRAG',
           'Uses distinct initial speeds to validate the frozen drag model.' if validation
           else 'Reaches each speed, then selects raw zero motor current. This identifies drag rather than VEL_TO_ERPM braking.')
    node = start_node('erpm_coastdown_validation' if validation else 'erpm_coastdown', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['coastdown']
        speeds = spec.get('validation_initial_speeds_mps', []) if validation else spec['initial_speeds_mps']
        repetitions = int(spec.get('validation_repetitions', spec['repetitions'])) if validation else int(spec['repetitions'])
        default_capture_key = 'validation_coast_capture_s' if validation else 'coast_capture_s'
        if default_capture_key not in spec:
            default_capture_key = 'coast_capture_s'
        if not speeds:
            raise RuntimeError('coast-down validation has no configured hold-out initial speeds')
        stage = '07a_coastdown_validation' if validation else '07_coastdown'
        for initial_speed in speeds:
            _require_envelope(cfg, speed=float(initial_speed))
            initial_erpm = _initial_erpm(float(initial_speed), gain, offset, speed_command_patch)
            capture_s = _capture_for_speed(
                spec, float(initial_speed),
                map_key='coast_capture_s_by_initial_speed',
                default_key=default_capture_key,
            )
            for rep in range(1, repetitions + 1):
                cid = f'{"coastdown_validation" if validation else "coastdown"}_{float(initial_speed):.3f}_rep_{rep:02d}'
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit(
                        'trial_start', stage=stage, condition_id=cid,
                        trial_id=trial, attempt=attempt,
                        initial_speed_mps=float(initial_speed), initial_erpm=initial_erpm,
                        planned_capture_s=capture_s,
                    )
                    startup = node.establish_raw_erpm(target_erpm=initial_erpm, startup_speed_mps=float(initial_speed), segment_id=cid, trial_id=trial)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        node.hold(
                            kind='raw_erpm', target=initial_erpm,
                            duration_s=float(spec['pre_coast_hold_s']),
                            phase='coastdown_baseline', segment_id=cid, trial_id=trial,
                            capture=False, initial_speed_mps=float(initial_speed),
                        )
                        summary = node.hold(
                            kind='raw_current', target=0.0,
                            duration_s=capture_s,
                            phase='coastdown', segment_id=cid, trial_id=trial,
                            capture=True, window_fields=WINDOW,
                            initial_speed_mps=float(initial_speed), initial_erpm=initial_erpm,
                            planned_capture_s=capture_s,
                        )
                    straight = _straight_ok(node, cfg) if summary else False
                    node.active_stop()
                    auto = bool(startup.get('stable')) and summary is not None
                    decision = _decision(
                        node, stage=stage, condition_id=cid,
                        trial_id=trial, attempt=attempt, auto_ok=auto,
                        summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
                    )
                    records.append({
                        'trial_id': trial, 'condition_id': cid, 'decision': decision,
                        'startup': startup, 'capture': summary,
                        'planned_capture_s': capture_s,
                    })
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {'validation': validation, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def _current_conditions(cfg: dict[str, Any], section: str, polarity: str) -> Iterable[tuple[float, float, float, float]]:
    """Yield a safe, explicitly feasible current-condition schedule.

    A Cartesian current×entry-speed grid is not always physically executable:
    max drive current near the top-speed ceiling can exceed the remaining speed
    headroom before enough LiDAR samples exist.  The config therefore supports
    per-entry-speed fraction lists.  Legacy Cartesian fields remain accepted
    only for backwards compatibility.
    """
    spec = cfg[section]
    env = cfg['operating_envelope']
    limit = float(env['approved_drive_test_current_a'] if polarity == 'drive' else env['approved_brake_test_current_a'])
    entries = spec.get(f'{polarity}_conditions')
    if entries is None:
        speeds = spec[f'{polarity}_initial_speeds_mps']
        fractions = spec[f'{polarity}_current_fractions']
        entries = [{'initial_speed_mps': speed, 'current_fractions': fractions} for speed in speeds]
    for entry in entries:
        speed = float(entry['initial_speed_mps'])
        _require_envelope(cfg, speed=speed)
        for fraction in map(float, entry['current_fractions']):
            if not (0.0 < fraction):
                raise RuntimeError(f'invalid {polarity} current fraction: {fraction}')
            duration = _safe_pulse_duration(cfg, fraction, initial_speed=speed, polarity=polarity)
            yield speed, fraction, limit * fraction, duration


def _current_pulses(cfg: dict[str, Any], stage_dir: Path, gain: float,
                    offset: float, counter: TrialCounter, *, holdout: bool,
                    speed_command_patch: dict[str, Any] | None = None) -> dict[str, Any]:
    num = '09' if holdout else '08'
    title = 'HOLD-OUT' if holdout else 'TRAINING'
    banner(f'STAGE {num} — FULL-ENVELOPE RAW CURRENT {title}',
           'Current pulses span low/mid/high initial speed and up to the configured high-current envelope. Data identify acceleration, traction saturation and longitudinal slip.')
    node = start_node('erpm_current_holdout' if holdout else 'erpm_current_training', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        section = 'raw_current_holdout' if holdout else 'raw_current_training'
        stage = f'{num}_raw_current_{"holdout" if holdout else "training"}'
        reps = int(cfg[section]['repetitions'])
        for polarity, kind in [('drive', 'raw_current'), ('brake', 'raw_brake')]:
            for initial_speed, fraction, amps, pulse_s in _current_conditions(cfg, section, polarity):
                initial_erpm = _initial_erpm(initial_speed, gain, offset, speed_command_patch)
                for rep in range(1, reps + 1):
                    cid = f'{polarity}_v_{initial_speed:.2f}_frac_{fraction:.3f}_rep_{rep:02d}'
                    attempt = 1
                    while True:
                        trial = _id(cid, attempt)
                        pause_for_reposition(
                            f'Position car at straight start for {cid}. '
                            f'Pulse={amps:.1f} A for {pulse_s:.2f} s. Attempt {attempt}.'
                        )
                        node.event.emit(
                            'trial_start', stage=stage, condition_id=cid,
                            trial_id=trial, attempt=attempt, polarity=polarity,
                            current_command_a=amps, current_fraction=fraction,
                            initial_speed_mps=initial_speed, initial_erpm=initial_erpm,
                            pulse_duration_s=pulse_s,
                        )
                        startup = node.establish_raw_erpm(target_erpm=initial_erpm, startup_speed_mps=float(initial_speed), segment_id=cid, trial_id=trial)
                        summary: dict[str, Any] | None = None
                        recovery: dict[str, Any] | None = None
                        if startup.get('stable'):
                            # Both zero-current reference windows are captured.
                            # The offline fitter uses robust LiDAR-window slopes
                            # during the pulse and these references to reject a
                            # pulse corrupted by drift or a scene change.
                            baseline = node.hold(
                                kind='raw_erpm', target=initial_erpm,
                                duration_s=float(cfg[section]['pre_pulse_hold_s']),
                                phase='current_pulse_baseline', segment_id=cid,
                                trial_id=trial, capture=True, window_fields=WINDOW, polarity=polarity,
                                current_command_a=amps, current_fraction=fraction,
                                initial_speed_mps=initial_speed,
                            )
                            summary = node.hold(
                                kind=kind, target=amps, duration_s=pulse_s,
                                phase=f'raw_{polarity}_current_pulse',
                                segment_id=cid, trial_id=trial, capture=True,
                                window_fields=WINDOW, polarity=polarity,
                                current_command_a=amps, current_fraction=fraction,
                                initial_speed_mps=initial_speed,
                                pulse_duration_s=pulse_s,
                            )
                            recovery = node.hold(
                                kind=kind, target=0.0,
                                duration_s=float(cfg[section].get('post_pulse_capture_s', 0.60)),
                                phase=f'raw_{polarity}_current_recovery',
                                segment_id=cid, trial_id=trial, capture=True,
                                window_fields=WINDOW, polarity=polarity,
                                current_command_a=0.0, current_fraction=fraction,
                                initial_speed_mps=initial_speed,
                                pulse_duration_s=pulse_s,
                            )
                        straight = _straight_ok(node, cfg) if summary else False
                        node.active_stop()
                        auto = bool(startup.get('stable')) and summary is not None and recovery is not None
                        decision = _decision(
                            node, stage=stage, condition_id=cid, trial_id=trial,
                            attempt=attempt, auto_ok=auto,
                            summary={'startup': startup, 'straight_runtime_gate': straight, 'baseline_capture': baseline if startup.get('stable') else {}, 'pulse_capture': summary or {}, 'recovery_capture': recovery or {}},
                        )
                        records.append({
                            'trial_id': trial, 'condition_id': cid, 'decision': decision,
                            'startup': startup, 'capture': summary, 'recovery': recovery,
                            'polarity': polarity, 'current_fraction': fraction,
                            'initial_speed_mps': initial_speed,
                        })
                        if decision == 'accepted':
                            counter.count(node)
                            break
                        if decision == 'skipped':
                            break
                        attempt += 1
        result = {'holdout': holdout, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def _run_accel_grid(
    node: CalibrationNode, *, cfg: dict[str, Any], stage: str,
    phase: str, initial_speeds: Iterable[float], accelerations: Iterable[float],
    repetitions: int, pre_pulse_s: float, capture_s: float,
    gain: float, offset: float, counter: TrialCounter,
    initial_erpm_fn: Callable[[float], float] | None = None,
) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for initial_speed in map(float, initial_speeds):
        _require_envelope(cfg, speed=initial_speed)
        initial_erpm = initial_erpm_fn(initial_speed) if initial_erpm_fn is not None else _raw_erpm(gain, offset, initial_speed)
        for acceleration in map(float, accelerations):
            _require_envelope(cfg, acceleration=acceleration, brake=acceleration < 0)
            duration = float(capture_s)
            for rep in range(1, int(repetitions) + 1):
                cid = f'accel_v_{initial_speed:.2f}_cmd_{acceleration:+.2f}_rep_{rep:02d}'
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit(
                        'trial_start', stage=stage, condition_id=cid, trial_id=trial,
                        attempt=attempt, acceleration_command_mps2=acceleration,
                        initial_speed_mps=initial_speed, initial_erpm=initial_erpm,
                        pulse_duration_s=duration,
                    )
                    startup = node.establish_raw_erpm(target_erpm=initial_erpm, startup_speed_mps=float(initial_speed), segment_id=cid, trial_id=trial)
                    baseline: dict[str, Any] | None = None
                    summary: dict[str, Any] | None = None
                    recovery: dict[str, Any] | None = None
                    if startup.get('stable'):
                        baseline = node.hold(
                            kind='raw_erpm', target=initial_erpm, duration_s=pre_pulse_s,
                            phase='accel_interface_baseline', segment_id=cid, trial_id=trial,
                            capture=True, window_fields=WINDOW,
                            acceleration_command_mps2=acceleration, initial_speed_mps=initial_speed,
                        )
                        summary = node.hold(
                            kind='ackermann_accel', target=acceleration,
                            speed_hint=initial_speed, duration_s=duration,
                            phase=phase, segment_id=cid, trial_id=trial, capture=True,
                            window_fields=WINDOW, acceleration_command_mps2=acceleration,
                            initial_speed_mps=initial_speed, pulse_duration_s=duration,
                        )
                        recovery = node.hold(
                            kind='ackermann_accel', target=0.0, speed_hint=initial_speed,
                            duration_s=pre_pulse_s, phase='accel_interface_recovery',
                            segment_id=cid, trial_id=trial, capture=True, window_fields=WINDOW,
                            acceleration_command_mps2=0.0, initial_speed_mps=initial_speed,
                        )
                    straight = _straight_ok(node, cfg) if summary else False
                    node.active_stop()
                    auto = bool(startup.get('stable')) and summary is not None and recovery is not None
                    decision = _decision(
                        node, stage=stage, condition_id=cid, trial_id=trial,
                        attempt=attempt, auto_ok=auto,
                        summary={
                            'startup': startup, 'straight_runtime_gate': straight,
                            'baseline_capture': baseline or {}, 'capture': summary or {},
                            'recovery_capture': recovery or {},
                        },
                    )
                    records.append({
                        'trial_id': trial, 'condition_id': cid, 'decision': decision,
                        'startup': startup, 'baseline': baseline, 'capture': summary,
                        'recovery': recovery,
                    })
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
    return records


def accel_to_current_interface(cfg: dict[str, Any], stage_dir: Path,
                               gain: float, offset: float,
                               counter: TrialCounter,
                               speed_command_patch: dict[str, Any] | None = None,
                               *, validation: bool = False) -> dict[str, Any]:
    banner('ACCEL_TO_CURRENT INTERFACE HOLD-OUT' if validation else 'STAGE 10 — FULL-ENVELOPE ACCEL_TO_CURRENT INTERFACE AUDIT',
           'New speed/acceleration points validate both current routing and realised ground acceleration.' if validation
           else 'Temporary candidate gains are active. This audits desired acceleration → selected current/brake and realised ground acceleration.')
    node = start_node('erpm_accel_interface_validation' if validation else 'erpm_accel_interface', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    try:
        spec = cfg['accel_to_current_interface']
        speeds = spec.get('validation_initial_speeds_mps', []) if validation else spec['initial_speeds_mps']
        accelerations = spec.get('validation_acceleration_commands_mps2', []) if validation else spec['acceleration_commands_mps2']
        repetitions = int(spec.get('validation_repetitions', spec['repetitions'])) if validation else int(spec['repetitions'])
        capture_s = float(spec.get('validation_pulse_capture_s', spec.get('pulse_capture_s', cfg['operating_envelope']['high_demand_pulse_duration_s']['low_fraction']))) if validation else float(spec.get('pulse_capture_s', cfg['operating_envelope']['high_demand_pulse_duration_s']['low_fraction']))
        if not speeds or not accelerations:
            raise RuntimeError('ACCEL_TO_CURRENT validation has no configured hold-out grid')
        records = _run_accel_grid(
            node, cfg=cfg, stage='10a_accel_to_current_interface_validation' if validation else '10_accel_to_current_interface',
            phase='accel_to_current_pulse',
            initial_speeds=speeds, accelerations=accelerations, repetitions=repetitions,
            pre_pulse_s=float(spec['pre_pulse_hold_s']),
            capture_s=capture_s,
            gain=gain, offset=offset, counter=counter,
            initial_erpm_fn=(lambda speed: _candidate_command_erpm(speed, speed_command_patch)) if speed_command_patch is not None else None,
        )
        result = {'validation': validation, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def _lateral_conditions(cfg: dict[str, Any], *, validation: bool) -> list[dict[str, float | str]]:
    """Build a balanced quasi-steady steering grid from explicit conditions."""
    spec = cfg['lateral_stiffness']
    speeds = spec.get('validation_speeds_mps', []) if validation else spec.get('speeds_mps', [])
    angles = spec.get('validation_steering_angles_rad', []) if validation else spec.get('steering_angles_rad', [])
    signs = spec.get('validation_signs', spec.get('signs', [-1.0, 1.0])) if validation else spec.get('signs', [-1.0, 1.0])
    nominal_wheelbase = float(spec.get('nominal_wheelbase_m', cfg.get('hardware', {}).get('wheelbase_m', 0.324)))
    if nominal_wheelbase <= 0.0:
        raise RuntimeError('lateral_stiffness nominal_wheelbase_m must be positive')
    result: list[dict[str, float | str]] = []
    for speed in map(float, speeds):
        _require_envelope(cfg, speed=speed)
        for magnitude in map(float, angles):
            if magnitude <= 0.0:
                raise RuntimeError('lateral_stiffness steering angles must be positive magnitudes')
            for sign in map(float, signs):
                if sign == 0.0:
                    raise RuntimeError('lateral_stiffness signs may not contain zero')
                angle = math.copysign(magnitude, sign)
                radius = nominal_wheelbase / abs(math.tan(angle))
                result.append({
                    'speed_mps': speed,
                    'steering_angle_rad': angle,
                    'turn_direction': 'left' if angle > 0.0 else 'right',
                    'nominal_radius_m': radius,
                    'nominal_lateral_accel_mps2': speed * speed / radius,
                })
    if not result:
        raise RuntimeError('lateral_stiffness has no configured quasi-steady conditions')
    return result


def quasi_steady_lateral(cfg: dict[str, Any], stage_dir: Path,
                         counter: TrialCounter, *, validation: bool = False) -> dict[str, Any]:
    """Capture a room-safe quasi-steady lateral grid for effective tyre stiffness.

    Every configured condition that fits starts and settles on its constant-
    radius path, then records one complete circle.  The full revolution gives
    substantially more independent time windows without requiring a larger
    room. A bounded-arc fallback is retained for future shallow conditions.
    """
    title = 'QUASI-STEADY TYRE-STIFFNESS HOLD-OUT' if validation else 'STAGE 12 — QUASI-STEADY TYRE-STIFFNESS TRAINING'
    subtitle = (
        'Distinct speed/steering arcs validate the frozen linear and nonlinear tyre candidates; they are never refitted here.'
        if validation else
        'Uses balanced left/right, speed and steering arcs. Heading assist is disabled during every intentional turn.'
    )
    banner(title, subtitle)
    node = start_node(
        'erpm_lateral_stiffness_validation' if validation else 'erpm_lateral_stiffness',
        cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'},
    )
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['lateral_stiffness']
        repetitions = int(spec.get('validation_repetitions', spec['repetitions'])) if validation else int(spec['repetitions'])
        pre_turn_s = float(spec.get('pre_turn_hold_s', 0.35))
        turn_settle_s = float(spec.get('turn_settle_s', 0.70))
        minimum_capture_s = float(spec.get('validation_capture_s', spec['capture_s'])) if validation else float(spec['capture_s'])
        stage = '12a_quasi_steady_lateral_validation' if validation else '12_quasi_steady_lateral_training'
        for condition in _lateral_conditions(cfg, validation=validation):
            speed = float(condition['speed_mps'])
            angle = float(condition['steering_angle_rad'])
            capture_plan = _full_circle_plan(
                cfg, speed_mps=speed,
                radius_m=float(condition['nominal_radius_m']),
                minimum_capture_s=minimum_capture_s,
            )
            capture_s = float(capture_plan['capture_s'])
            for rep in range(1, repetitions + 1):
                cid = (
                    f'lateral_{"validation_" if validation else ""}v_{speed:.2f}_'
                    f'delta_{angle:+.3f}_rep_{rep:02d}'
                )
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(
                        f'Place car at the marked circle start for {cid}. '
                        f'It will turn {condition["turn_direction"]} at {speed:.2f} m/s; '
                        f'nominal radius {float(condition["nominal_radius_m"]):.2f} m; '
                        f'{capture_plan["capture_mode"]}, {capture_s:.1f} s recorded. Attempt {attempt}.'
                    )
                    node.event.emit(
                        'trial_start', stage=stage, condition_id=cid, trial_id=trial,
                        attempt=attempt, speed_command_mps=speed,
                        steering_angle_rad=angle,
                        nominal_radius_m=float(condition['nominal_radius_m']),
                        nominal_lateral_accel_mps2=float(condition['nominal_lateral_accel_mps2']),
                        turn_direction=str(condition['turn_direction']),
                        capture_mode=str(capture_plan['capture_mode']),
                        planned_capture_s=capture_s,
                        planned_revolutions=float(capture_plan['planned_revolutions']),
                    )
                    initial_circle_mode = capture_plan['capture_mode'] == 'full_circle'
                    startup = node.establish_ackermann_speed(
                        speed_mps=speed, segment_id=cid, trial_id=trial,
                        steering_angle_rad=angle if initial_circle_mode else 0.0,
                    )
                    active_plan = dict(capture_plan)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        if pre_turn_s > 0.0 and not initial_circle_mode:
                            node.hold(
                                kind='ackermann_speed', target=speed, duration_s=pre_turn_s,
                                phase='lateral_straight_baseline', segment_id=cid, trial_id=trial,
                                capture=False, steering_angle_rad=0.0,
                                speed_command_mps=speed, steering_angle_target_rad=angle,
                            )
                        if turn_settle_s > 0.0:
                            node.hold(
                                kind='ackermann_speed', target=speed, duration_s=turn_settle_s,
                                phase='lateral_turn_settle', segment_id=cid, trial_id=trial,
                                capture=False, steering_angle_rad=angle,
                                speed_command_mps=speed, steering_angle_target_rad=angle,
                            )
                        if initial_circle_mode and math.isfinite(node.latest.imu_gz) and abs(node.latest.imu_gz) >= 0.02:
                            onboard_radius = speed / abs(float(node.latest.imu_gz))
                            active_plan = _full_circle_plan(
                                cfg, speed_mps=speed, radius_m=onboard_radius,
                                minimum_capture_s=minimum_capture_s,
                            )
                            active_plan['planning_source'] = 'onboard IMU turn-radius estimate after circular settle'
                            if (
                                active_plan['capture_mode'] != capture_plan['capture_mode']
                                or abs(float(active_plan['capture_s']) - capture_s) > 0.05
                            ):
                                note(
                                    'Onboard turn estimate refined this pass to '
                                    f"{active_plan['capture_mode'].replace('_', ' ')} for "
                                    f"{float(active_plan['capture_s']):.1f} s."
                                )
                        circle_mode = active_plan['capture_mode'] == 'full_circle'
                        active_capture_s = float(active_plan['capture_s'])
                        node.event.emit(
                            'lateral_capture_plan', condition_id=cid, trial_id=trial,
                            **active_plan,
                        )
                        summary = node.hold(
                            kind='ackermann_speed', target=speed, duration_s=active_capture_s,
                            phase='lateral_quasi_steady', segment_id=cid, trial_id=trial,
                            capture=True, steering_angle_rad=angle, window_fields=WINDOW,
                            speed_command_mps=speed, steering_angle_target_rad=angle,
                            nominal_radius_m=float(condition['nominal_radius_m']),
                            nominal_lateral_accel_mps2=float(condition['nominal_lateral_accel_mps2']),
                            turn_direction=str(condition['turn_direction']),
                            capture_mode=str(active_plan['capture_mode']),
                            planned_capture_s=active_capture_s,
                            planned_revolutions=float(active_plan['planned_revolutions']),
                            planning_radius_m=float(active_plan['nominal_radius_m']),
                            planning_source=str(active_plan.get('planning_source', 'nominal kinematic radius')),
                            target_abs_yaw_change_rad=(
                                2.0 * math.pi * float(active_plan['planned_revolutions'])
                                if circle_mode else None
                            ),
                            minimum_duration_s=(0.50 * active_capture_s if circle_mode else 0.0),
                            maximum_duration_s=(1.50 * active_capture_s if circle_mode else None),
                        )
                    else:
                        circle_mode = initial_circle_mode
                    node.active_stop(steering_angle_rad=angle)
                    circle_complete = (
                        not circle_mode
                        or bool((summary or {}).get('yaw_target_reached', False))
                    )
                    auto = bool(startup.get('stable')) and summary is not None and circle_complete
                    decision = _decision(
                        node, stage=stage, condition_id=cid, trial_id=trial, attempt=attempt,
                        auto_ok=auto, summary={
                            'startup': startup,
                            'quasi_steady_capture': summary or {},
                            'circle_complete': circle_complete,
                        },
                    )
                    records.append({
                        'trial_id': trial, 'condition_id': cid, 'decision': decision,
                        'startup': startup, 'capture': summary, **condition,
                        **active_plan,
                    })
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {
            'validation': validation,
            'capture_policy': 'one complete circle when the footprint-aware room check permits it',
            'records': records,
        }
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def run_stage(stage_name: str, cfg: dict[str, Any], stage_dir: Path,
              gain: float, offset: float, counter: TrialCounter,
              speed_command_patch: dict[str, Any] | None = None) -> dict[str, Any]:
    table = {
        '00_command_chain_audit': lambda: command_chain_audit(cfg, stage_dir),
        '01_longitudinal_observability': lambda: longitudinal_observability(cfg, stage_dir, gain, offset, counter),
        '02_low_speed_launch': lambda: low_speed_launch(cfg, stage_dir, gain, offset, counter),
        '03_raw_erpm_map_training': lambda: raw_erpm_map(cfg, stage_dir, gain, offset, counter, holdout=False),
        '04_raw_erpm_map_holdout': lambda: raw_erpm_map(cfg, stage_dir, gain, offset, counter, holdout=True),
        '05_vel_to_erpm_pipeline_audit': lambda: vel_to_erpm_pipeline_audit(cfg, stage_dir, counter),
        '06_raw_erpm_response': lambda: raw_erpm_response(cfg, stage_dir, gain, offset, counter, speed_command_patch),
        '06a_raw_erpm_response_validation': lambda: raw_erpm_response(cfg, stage_dir, gain, offset, counter, speed_command_patch, validation=True),
        '07_coastdown': lambda: coastdown(cfg, stage_dir, gain, offset, counter, speed_command_patch),
        '07a_coastdown_validation': lambda: coastdown(cfg, stage_dir, gain, offset, counter, speed_command_patch, validation=True),
        '08_raw_current_training': lambda: _current_pulses(cfg, stage_dir, gain, offset, counter, holdout=False, speed_command_patch=speed_command_patch),
        '09_raw_current_holdout': lambda: _current_pulses(cfg, stage_dir, gain, offset, counter, holdout=True, speed_command_patch=speed_command_patch),
        '10_accel_to_current_interface': lambda: accel_to_current_interface(cfg, stage_dir, gain, offset, counter, speed_command_patch),
        '10a_accel_to_current_interface_validation': lambda: accel_to_current_interface(cfg, stage_dir, gain, offset, counter, speed_command_patch, validation=True),
        '12_quasi_steady_lateral_training': lambda: quasi_steady_lateral(cfg, stage_dir, counter, validation=False),
        '12a_quasi_steady_lateral_validation': lambda: quasi_steady_lateral(cfg, stage_dir, counter, validation=True),
    }
    return table[stage_name]()


def candidate_velocity_verification(cfg: dict[str, Any], stage_dir: Path,
                                    counter: TrialCounter) -> dict[str, Any]:
    banner('STAGE 11 — TEMPORARY CANDIDATE VEL_TO_ERPM VERIFICATION',
           'The candidate is active only in the reversible transaction. The full operating-speed envelope is verified.')
    node = start_node('erpm_candidate_velocity_verification', cfg, {'imu', 'odom', 'candidate_odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['candidate_verification']
        for speed in spec['velocity_holdout_commands_mps']:
            for rep in range(1, int(spec['velocity_repetitions']) + 1):
                records += _run_ackermann_plateau(
                    node, cfg=cfg, counter=counter,
                    stage='11_candidate_velocity_verification',
                    condition_id=f'candidate_vel_{float(speed):.3f}_rep_{rep:02d}',
                    speed=float(speed), capture_s=_capture_for_speed(
                        spec, float(speed), map_key='velocity_capture_s_by_speed', default_key='capture_s',
                    ),
                    phase='candidate_velocity_verification',
                )
        result = {'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def candidate_accel_verification(cfg: dict[str, Any], stage_dir: Path,
                                 gain: float, offset: float,
                                 counter: TrialCounter, candidate_patch: dict[str, Any]) -> dict[str, Any]:
    banner('STAGE 12 — TEMPORARY CANDIDATE ACCEL_TO_CURRENT VERIFICATION',
           'The candidate current/drag model is active only in the reversible transaction. The full speed/acceleration grid is verified.')
    node = start_node('erpm_candidate_accel_verification', cfg, {'imu', 'odom', 'candidate_odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    try:
        spec = cfg['candidate_verification']
        records = _run_accel_grid(
            node, cfg=cfg, stage='12_candidate_accel_verification',
            phase='candidate_accel_verification',
            initial_speeds=spec['acceleration_initial_speeds_mps'],
            accelerations=spec['acceleration_holdout_commands_mps2'],
            repetitions=int(spec['acceleration_repetitions']),
            pre_pulse_s=1.2,
            capture_s=float(spec['capture_s']), gain=gain, offset=offset,
            counter=counter, initial_erpm_fn=lambda speed: _candidate_command_erpm(speed, candidate_patch),
        )
        result = {'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def _cross_axis_conditions(spec: dict[str, Any]) -> list[dict[str, float]]:
    conditions = spec.get('conditions')
    if conditions is not None:
        return [
            {
                'speed_mps': float(c['speed_mps']),
                'steering_angle_rad': float(c['steering_angle_rad']),
                'expected_lateral_accel_mps2': float(c.get('expected_lateral_accel_mps2', math.nan)),
            }
            for c in conditions
        ]
    return [
        {
            'speed_mps': float(speed),
            'steering_angle_rad': float(angle),
            'expected_lateral_accel_mps2': math.nan,
        }
        for angle in spec['steering_angle_rad']
        for speed in spec['speed_commands_mps']
    ]


def candidate_cross_axis_verification(cfg: dict[str, Any], stage_dir: Path,
                                      counter: TrialCounter) -> dict[str, Any]:
    """Independent candidate-speed hold-out with calibrated non-zero steering.

    This stage is not used for longitudinal fitting. It prevents a straight-only
    wheel-speed estimator from being called validated when it degrades as the
    vehicle experiences the steering/yaw states in which MPC actually operates.
    """
    banner('STAGE 13 — CANDIDATE CROSS-AXIS SPEED HOLD-OUT',
           'The candidate odometry remains active. Use the already-calibrated steering map; this stage validates only candidate speed against LiDAR.')
    node = start_node('erpm_candidate_cross_axis', cfg, {'imu', 'odom', 'candidate_odom', 'vesc', 'scan', 'selected_speed'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['cross_axis_validation']
        if not bool(spec.get('enabled', False)):
            result = {'enabled': False, 'records': []}
            dump_json(stage_dir / 'runtime_result.json', result)
            return result
        for condition in _cross_axis_conditions(spec):
            speed = float(condition['speed_mps'])
            angle = float(condition['steering_angle_rad'])
            expected_lat = float(condition['expected_lateral_accel_mps2'])
            for rep in range(1, int(spec['repetitions']) + 1):
                cid = f'cross_axis_delta_{angle:+.3f}_speed_{speed:.3f}_rep_{rep:02d}'
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(
                        f'Position at the marked start. {cid}; candidate speed={speed:.3f} m/s, steering={angle:+.3f} rad.\nAttempt {attempt}; REDO has no limit.'
                    )
                    node.event.emit(
                        'trial_start', stage='13_candidate_cross_axis_verification',
                        condition_id=cid, trial_id=trial, attempt=attempt,
                        speed_command_mps=speed, steering_angle_rad=angle,
                        expected_lateral_accel_mps2=expected_lat,
                    )
                    startup = node.establish_ackermann_speed(speed_mps=speed, segment_id=cid, trial_id=trial, steering_angle_rad=0.0)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        pre_turn_s = float(spec.get('pre_turn_hold_s', 0.0))
                        if pre_turn_s > 0.0:
                            node.hold(
                                kind='ackermann_speed', target=speed,
                                duration_s=pre_turn_s,
                                phase='candidate_cross_axis_turn_settle',
                                segment_id=cid, trial_id=trial, capture=False,
                                steering_angle_rad=angle,
                                speed_command_mps=speed,
                                expected_lateral_accel_mps2=expected_lat,
                            )
                        summary = node.hold(
                            kind='ackermann_speed', target=speed,
                            duration_s=float(spec['capture_s']),
                            phase='candidate_cross_axis_verification',
                            segment_id=cid, trial_id=trial, capture=True,
                            steering_angle_rad=angle, window_fields=WINDOW,
                            speed_command_mps=speed,
                            expected_lateral_accel_mps2=expected_lat,
                        )
                    node.active_stop()
                    auto = bool(startup.get('stable')) and summary is not None
                    decision = _decision(node, stage='13_candidate_cross_axis_verification', condition_id=cid, trial_id=trial, attempt=attempt, auto_ok=auto, summary={'startup': startup, 'capture': summary or {}})
                    records.append({'trial_id': trial, 'condition_id': cid, 'decision': decision, 'startup': startup, 'capture': summary, 'speed_command_mps': speed, 'steering_angle_rad': angle, 'expected_lateral_accel_mps2': expected_lat})
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {'enabled': True, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def post_calibration_steering_dynamics(cfg: dict[str, Any], stage_dir: Path,
                                       counter: TrialCounter) -> dict[str, Any]:
    """Post-calibration steering/yaw dynamics data.

    This is deliberately after ERPM/current and turn-slip calibration.  It is a
    bagged validation/identification dataset for later lateral-dynamics work:
    establish speed with zero steering, then step steering and record yaw-rate,
    lateral acceleration, LiDAR speed and candidate odometry response.
    """
    banner('STAGE 14 — POST-CALIBRATION STEERING DYNAMICS',
           'Candidate odometry remains active. Each trial reaches speed straight, then applies a steering step for later high-speed steering/yaw fitting.')
    node = start_node('erpm_post_calibration_steering', cfg, {'imu', 'odom', 'candidate_odom', 'vesc', 'scan', 'selected_speed'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['post_calibration_steering_dynamics']
        if not bool(spec.get('enabled', False)):
            result = {'enabled': False, 'records': []}
            dump_json(stage_dir / 'runtime_result.json', result)
            return result
        for condition in _cross_axis_conditions(spec):
            speed = float(condition['speed_mps'])
            angle = float(condition['steering_angle_rad'])
            expected_lat = float(condition['expected_lateral_accel_mps2'])
            _require_envelope(cfg, speed=speed)
            for rep in range(1, int(spec['repetitions']) + 1):
                cid = f'steering_step_delta_{angle:+.3f}_speed_{speed:.3f}_rep_{rep:02d}'
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(
                        f'Position at the marked start. {cid}; speed={speed:.3f} m/s, steering step={angle:+.3f} rad.\nAttempt {attempt}; REDO has no limit.'
                    )
                    node.event.emit(
                        'trial_start', stage='14_post_calibration_steering_dynamics',
                        condition_id=cid, trial_id=trial, attempt=attempt,
                        speed_command_mps=speed, steering_angle_rad=angle,
                        expected_lateral_accel_mps2=expected_lat,
                    )
                    startup = node.establish_ackermann_speed(speed_mps=speed, segment_id=cid, trial_id=trial, steering_angle_rad=0.0)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        node.hold(
                            kind='ackermann_speed', target=speed,
                            duration_s=float(spec['pre_turn_hold_s']),
                            phase='post_calibration_steering_baseline',
                            segment_id=cid, trial_id=trial, capture=False,
                            steering_angle_rad=0.0,
                            speed_command_mps=speed,
                            steering_step_target_rad=angle,
                            expected_lateral_accel_mps2=expected_lat,
                        )
                        summary = node.hold(
                            kind='ackermann_speed', target=speed,
                            duration_s=float(spec['capture_s']),
                            phase='post_calibration_steering_step',
                            segment_id=cid, trial_id=trial, capture=True,
                            steering_angle_rad=angle, window_fields=WINDOW,
                            speed_command_mps=speed,
                            steering_step_target_rad=angle,
                            expected_lateral_accel_mps2=expected_lat,
                        )
                    node.active_stop()
                    auto = bool(startup.get('stable')) and summary is not None
                    decision = _decision(
                        node, stage='14_post_calibration_steering_dynamics',
                        condition_id=cid, trial_id=trial, attempt=attempt,
                        auto_ok=auto, summary={'startup': startup, 'steering_step_capture': summary or {}},
                    )
                    records.append({
                        'trial_id': trial, 'condition_id': cid, 'decision': decision,
                        'startup': startup, 'capture': summary,
                        'speed_command_mps': speed,
                        'steering_angle_rad': angle,
                        'expected_lateral_accel_mps2': expected_lat,
                    })
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {'enabled': True, 'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)
