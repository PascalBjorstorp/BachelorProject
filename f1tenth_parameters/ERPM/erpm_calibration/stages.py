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
from .ui import banner, checklist, pause_for_reposition, require_ready, review_trial

WINDOW = (
    'imu_ax', 'imu_ay', 'imu_gz', 'odom_vx', 'odom_vy', 'candidate_odom_vx', 'candidate_odom_vy', 'erpm',
    'motor_current_a', 'input_current_a', 'battery_v', 'motor_temp_c',
    'fet_temp_c', 'selected_speed_erpm', 'selected_current_a',
    'selected_brake_a',
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


def _decision(
    node: CalibrationNode, *, stage: str, condition_id: str, trial_id: str,
    attempt: int, auto_ok: bool, summary: dict[str, Any],
) -> str:
    decision = review_trial(
        label=f'{stage}: {condition_id}, attempt {attempt}',
        automatic_ok=auto_ok,
        automatic_summary=summary,
    )
    node.event.emit(
        'trial_decision', stage=stage, condition_id=condition_id,
        trial_id=trial_id, attempt=attempt, decision=decision,
        accepted=decision == 'accepted', automatic_ok=auto_ok,
    )
    return decision


def _straight_ok(node: CalibrationNode, cfg: dict[str, Any]) -> bool:
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
        cap = float(env['maximum_test_brake_mps2'] if brake or acceleration < 0 else env['maximum_test_accel_mps2'])
        if abs(float(acceleration)) > cap + 1e-9:
            raise RuntimeError(f'configured acceleration {acceleration:.3f} exceeds operating-envelope cap {cap:.3f}')


def _pulse_duration(cfg: dict[str, Any], fraction: float) -> float:
    spec = cfg['operating_envelope']['high_demand_pulse_duration_s']
    f = abs(float(fraction))
    if f <= 0.25:
        return float(spec['low_fraction'])
    if f <= 0.55:
        return float(spec['medium_fraction'])
    return float(spec['high_fraction'])


def _safe_pulse_duration(cfg: dict[str, Any], fraction: float, *, initial_speed: float, polarity: str) -> float:
    """Limit pulse time to the declared speed/stopping envelope.

    This is conservative: it uses the maximum configured acceleration, not an
    optimistic expected value.  An infeasible configured condition fails before
    driving rather than becoming an uncontrolled over-speed event or a capture
    too short for LiDAR identification.
    """
    env = cfg['operating_envelope']
    nominal = _pulse_duration(cfg, fraction)
    minimum = float(env['dynamic_capture_min_s'])
    if polarity == 'drive':
        headroom = float(env['maximum_test_speed_mps']) - float(env['speed_guard_margin_mps']) - float(initial_speed)
        cap = float(env['maximum_test_accel_mps2']) * abs(float(fraction))
    else:
        headroom = float(initial_speed) - float(env['brake_guard_margin_mps'])
        cap = float(env['maximum_test_brake_mps2']) * abs(float(fraction))
    maximum = headroom / max(cap, 1e-9)
    duration = min(nominal, maximum)
    if duration + 1e-9 < minimum:
        raise RuntimeError(
            f'infeasible {polarity} pulse: initial speed {initial_speed:.3f} m/s, fraction {fraction:.3f} '
            f'allows only {duration:.3f} s before the configured envelope; minimum identifiable duration is {minimum:.3f} s. '
            'Use a longer track/higher reviewed speed envelope or remove this condition from the schedule.'
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
            raw_erpm_target=raw_erpm,
        )
        startup = node.establish_raw_erpm(
            target_erpm=raw_erpm, segment_id=condition_id, trial_id=trial,
        )
        summary: dict[str, Any] | None = None
        if startup.get('stable'):
            summary = node.hold(
                kind='raw_erpm', target=raw_erpm, duration_s=capture_s,
                phase=phase, segment_id=condition_id, trial_id=trial,
                capture=True, window_fields=WINDOW,
                nominal_speed_mps=nominal_speed, raw_erpm_target=raw_erpm,
            )
        straight = _straight_ok(node, cfg) if summary is not None else False
        node.neutral()
        auto = bool(startup.get('stable')) and summary is not None and straight
        decision = _decision(
            node, stage=stage, condition_id=condition_id, trial_id=trial,
            attempt=attempt, auto_ok=auto,
            summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
        )
        records.append({
            'trial_id': trial, 'attempt': attempt, 'decision': decision,
            'startup': startup, 'capture': summary,
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
        )
        startup = node.establish_ackermann_speed(
            speed_mps=speed, segment_id=condition_id, trial_id=trial,
        )
        summary: dict[str, Any] | None = None
        if startup.get('stable'):
            summary = node.hold(
                kind='ackermann_speed', target=speed, duration_s=capture_s,
                phase=phase, segment_id=condition_id, trial_id=trial,
                capture=True, window_fields=WINDOW, speed_command_mps=speed,
            )
        straight = _straight_ok(node, cfg) if summary is not None else False
        node.neutral()
        auto = bool(startup.get('stable')) and summary is not None and straight
        decision = _decision(
            node, stage=stage, condition_id=condition_id, trial_id=trial,
            attempt=attempt, auto_ok=auto,
            summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
        )
        records.append({
            'trial_id': trial, 'attempt': attempt, 'decision': decision,
            'startup': startup, 'capture': summary, 'speed_command_mps': speed,
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
                probes += _run_raw_erpm_plateau(
                    node, cfg=cfg, counter=counter,
                    stage='01_longitudinal_observability', condition_id=cid,
                    raw_erpm=_raw_erpm(gain, offset, float(speed)),
                    nominal_speed=float(speed),
                    capture_s=float(cfg['observability']['straight_probe_capture_s']),
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
                    nominal_speed=float(speed), capture_s=float(spec['capture_s']),
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
                    nominal_speed=speed, capture_s=float(spec['capture_s']), phase=phase,
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
                    speed=float(speed), capture_s=float(spec['capture_s']),
                    phase='vel_to_erpm_pipeline',
                )
        result = {'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def raw_erpm_response(cfg: dict[str, Any], stage_dir: Path, gain: float,
                      offset: float, counter: TrialCounter,
                      speed_command_patch: dict[str, Any] | None = None) -> dict[str, Any]:
    banner('STAGE 6 — FULL-ENVELOPE ERPM / GROUND-SPEED STEP RESPONSE',
           'Identifies command delivery, VESC ERPM tracking, and high-demand ground-speed delay/rate.')
    node = start_node('erpm_step_response', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['raw_erpm_response']
        for baseline_speed, target_speed in spec['steps_mps']:
            _require_envelope(cfg, speed=float(baseline_speed))
            _require_envelope(cfg, speed=float(target_speed))
            baseline_erpm = _initial_erpm(float(baseline_speed), gain, offset, speed_command_patch)
            target_erpm = _initial_erpm(float(target_speed), gain, offset, speed_command_patch)
            for rep in range(1, int(spec['repetitions']) + 1):
                cid = f'erpm_step_{float(baseline_speed):.3f}_to_{float(target_speed):.3f}_rep_{rep:02d}'
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit(
                        'trial_start', stage='06_raw_erpm_response',
                        condition_id=cid, trial_id=trial, attempt=attempt,
                        baseline_erpm=baseline_erpm, target_erpm=target_erpm,
                        baseline_speed_mps=float(baseline_speed), target_speed_mps=float(target_speed),
                    )
                    startup = node.establish_raw_erpm(target_erpm=baseline_erpm, segment_id=cid, trial_id=trial)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        node.hold(
                            kind='raw_erpm', target=baseline_erpm,
                            duration_s=float(spec['pre_step_hold_s']),
                            phase='erpm_step_baseline', segment_id=cid, trial_id=trial,
                            capture=False, baseline_erpm=baseline_erpm, target_erpm=target_erpm,
                        )
                        summary = node.hold(
                            kind='raw_erpm', target=target_erpm,
                            duration_s=float(spec['response_capture_s']),
                            phase='erpm_step_response', segment_id=cid, trial_id=trial,
                            capture=True, window_fields=WINDOW,
                            baseline_erpm=baseline_erpm, target_erpm=target_erpm,
                            baseline_speed_mps=float(baseline_speed), target_speed_mps=float(target_speed),
                        )
                    straight = _straight_ok(node, cfg) if summary else False
                    node.neutral()
                    auto = bool(startup.get('stable')) and summary is not None and straight
                    decision = _decision(
                        node, stage='06_raw_erpm_response', condition_id=cid,
                        trial_id=trial, attempt=attempt, auto_ok=auto,
                        summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
                    )
                    records.append({'trial_id': trial, 'condition_id': cid, 'decision': decision, 'startup': startup, 'capture': summary})
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {'records': records}
        dump_json(stage_dir / 'runtime_result.json', result)
        return result
    finally:
        finish_node(node)


def coastdown(cfg: dict[str, Any], stage_dir: Path, gain: float,
              offset: float, counter: TrialCounter,
              speed_command_patch: dict[str, Any] | None = None) -> dict[str, Any]:
    banner('STAGE 7 — FULL-ENVELOPE COAST-DOWN / DRAG',
           'Reaches each speed, then selects raw zero motor current. This identifies drag rather than VEL_TO_ERPM braking.')
    node = start_node('erpm_coastdown', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    records: list[dict[str, Any]] = []
    try:
        spec = cfg['coastdown']
        for initial_speed in spec['initial_speeds_mps']:
            _require_envelope(cfg, speed=float(initial_speed))
            initial_erpm = _initial_erpm(float(initial_speed), gain, offset, speed_command_patch)
            for rep in range(1, int(spec['repetitions']) + 1):
                cid = f'coastdown_{float(initial_speed):.3f}_rep_{rep:02d}'
                attempt = 1
                while True:
                    trial = _id(cid, attempt)
                    pause_for_reposition(f'Position car at straight start for {cid}. Attempt {attempt}.')
                    node.event.emit(
                        'trial_start', stage='07_coastdown', condition_id=cid,
                        trial_id=trial, attempt=attempt,
                        initial_speed_mps=float(initial_speed), initial_erpm=initial_erpm,
                    )
                    startup = node.establish_raw_erpm(target_erpm=initial_erpm, segment_id=cid, trial_id=trial)
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
                            duration_s=float(spec['coast_capture_s']),
                            phase='coastdown', segment_id=cid, trial_id=trial,
                            capture=True, window_fields=WINDOW,
                            initial_speed_mps=float(initial_speed), initial_erpm=initial_erpm,
                        )
                    straight = _straight_ok(node, cfg) if summary else False
                    node.neutral()
                    auto = bool(startup.get('stable')) and summary is not None and straight
                    decision = _decision(
                        node, stage='07_coastdown', condition_id=cid,
                        trial_id=trial, attempt=attempt, auto_ok=auto,
                        summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
                    )
                    records.append({'trial_id': trial, 'condition_id': cid, 'decision': decision, 'startup': startup, 'capture': summary})
                    if decision == 'accepted':
                        counter.count(node)
                        break
                    if decision == 'skipped':
                        break
                    attempt += 1
        result = {'records': records}
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
            if not (0.0 < fraction <= 1.0):
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
                        startup = node.establish_raw_erpm(target_erpm=initial_erpm, segment_id=cid, trial_id=trial)
                        summary: dict[str, Any] | None = None
                        recovery: dict[str, Any] | None = None
                        high_demand = fraction >= float(cfg['traction_transients']['minimum_current_fraction_for_transient_metrics'])
                        if startup.get('stable'):
                            node.hold(
                                kind='raw_erpm', target=initial_erpm,
                                duration_s=float(cfg[section]['pre_pulse_hold_s']),
                                phase='current_pulse_baseline', segment_id=cid,
                                trial_id=trial, capture=False, polarity=polarity,
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
                            # High-current data need an explicit post-release
                            # interval. This lets offline analysis identify
                            # slip peak, onset and recovery rather than keeping
                            # only a single pulse-average acceleration value.
                            if high_demand:
                                recovery = node.hold(
                                    kind=kind, target=0.0,
                                    duration_s=float(cfg['traction_transients']['slip_recovery_capture_s']),
                                    phase=f'raw_{polarity}_current_recovery',
                                    segment_id=cid, trial_id=trial, capture=True,
                                    window_fields=WINDOW, polarity=polarity,
                                    current_command_a=0.0, current_fraction=fraction,
                                    initial_speed_mps=initial_speed,
                                    pulse_duration_s=pulse_s,
                                )
                        straight = _straight_ok(node, cfg) if summary else False
                        node.neutral()
                        auto = bool(startup.get('stable')) and summary is not None and straight and (not high_demand or recovery is not None)
                        decision = _decision(
                            node, stage=stage, condition_id=cid, trial_id=trial,
                            attempt=attempt, auto_ok=auto,
                            summary={'startup': startup, 'straight_runtime_gate': straight, 'pulse_capture': summary or {}, 'recovery_capture': recovery or {}},
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
            duration = min(float(capture_s), _pulse_duration(cfg, min(1.0, abs(acceleration) / max(
                float(cfg['operating_envelope']['maximum_test_accel_mps2']), 1e-9))))
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
                    startup = node.establish_raw_erpm(target_erpm=initial_erpm, segment_id=cid, trial_id=trial)
                    summary: dict[str, Any] | None = None
                    if startup.get('stable'):
                        node.hold(
                            kind='raw_erpm', target=initial_erpm, duration_s=pre_pulse_s,
                            phase='accel_interface_baseline', segment_id=cid, trial_id=trial,
                            capture=False, acceleration_command_mps2=acceleration,
                            initial_speed_mps=initial_speed,
                        )
                        summary = node.hold(
                            kind='ackermann_accel', target=acceleration,
                            speed_hint=initial_speed, duration_s=duration,
                            phase=phase, segment_id=cid, trial_id=trial, capture=True,
                            window_fields=WINDOW, acceleration_command_mps2=acceleration,
                            initial_speed_mps=initial_speed, pulse_duration_s=duration,
                        )
                    straight = _straight_ok(node, cfg) if summary else False
                    node.neutral()
                    auto = bool(startup.get('stable')) and summary is not None and straight
                    decision = _decision(
                        node, stage=stage, condition_id=cid, trial_id=trial,
                        attempt=attempt, auto_ok=auto,
                        summary={'startup': startup, 'straight_runtime_gate': straight, 'capture': summary or {}},
                    )
                    records.append({'trial_id': trial, 'condition_id': cid, 'decision': decision, 'startup': startup, 'capture': summary})
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
                               speed_command_patch: dict[str, Any] | None = None) -> dict[str, Any]:
    banner('STAGE 10 — FULL-ENVELOPE ACCEL_TO_CURRENT INTERFACE AUDIT',
           'Temporary bootstrap gains are active. This audits desired acceleration → selected current/brake over the full speed and acceleration grid.')
    node = start_node('erpm_accel_interface', cfg, {'imu', 'odom', 'vesc', 'scan', 'selected_speed', 'selected_current', 'selected_brake'})
    try:
        spec = cfg['accel_to_current_interface']
        records = _run_accel_grid(
            node, cfg=cfg, stage='10_accel_to_current_interface',
            phase='accel_to_current_pulse',
            initial_speeds=spec['initial_speeds_mps'],
            accelerations=spec['acceleration_commands_mps2'],
            repetitions=int(spec['repetitions']),
            pre_pulse_s=float(spec['pre_pulse_hold_s']),
            capture_s=float(cfg['operating_envelope']['high_demand_pulse_duration_s']['low_fraction']),
            gain=gain, offset=offset, counter=counter,
            initial_erpm_fn=(lambda speed: _candidate_command_erpm(speed, speed_command_patch)) if speed_command_patch is not None else None,
        )
        result = {'records': records}
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
        '07_coastdown': lambda: coastdown(cfg, stage_dir, gain, offset, counter, speed_command_patch),
        '08_raw_current_training': lambda: _current_pulses(cfg, stage_dir, gain, offset, counter, holdout=False, speed_command_patch=speed_command_patch),
        '09_raw_current_holdout': lambda: _current_pulses(cfg, stage_dir, gain, offset, counter, holdout=True, speed_command_patch=speed_command_patch),
        '10_accel_to_current_interface': lambda: accel_to_current_interface(cfg, stage_dir, gain, offset, counter, speed_command_patch),
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
                    speed=float(speed), capture_s=float(spec['capture_s']),
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
        for angle in spec['steering_angle_rad']:
            for speed in spec['speed_commands_mps']:
                for rep in range(1, int(spec['repetitions']) + 1):
                    cid = f'cross_axis_delta_{float(angle):+.3f}_speed_{float(speed):.3f}_rep_{rep:02d}'
                    attempt = 1
                    while True:
                        trial = _id(cid, attempt)
                        pause_for_reposition(
                            f'Position at the marked start. {cid}; candidate speed={float(speed):.3f} m/s, steering={float(angle):+.3f} rad.\nAttempt {attempt}; REDO has no limit.'
                        )
                        node.event.emit('trial_start', stage='13_candidate_cross_axis_verification', condition_id=cid, trial_id=trial, attempt=attempt, speed_command_mps=float(speed), steering_angle_rad=float(angle))
                        startup = node.establish_ackermann_speed(speed_mps=float(speed), segment_id=cid, trial_id=trial, steering_angle_rad=float(angle))
                        summary: dict[str, Any] | None = None
                        if startup.get('stable'):
                            summary = node.hold(kind='ackermann_speed', target=float(speed), duration_s=float(spec['capture_s']), phase='candidate_cross_axis_verification', segment_id=cid, trial_id=trial, capture=True, steering_angle_rad=float(angle), window_fields=WINDOW, speed_command_mps=float(speed))
                        node.neutral()
                        auto = bool(startup.get('stable')) and summary is not None
                        decision = _decision(node, stage='13_candidate_cross_axis_verification', condition_id=cid, trial_id=trial, attempt=attempt, auto_ok=auto, summary={'startup': startup, 'capture': summary or {}})
                        records.append({'trial_id': trial, 'condition_id': cid, 'decision': decision, 'startup': startup, 'capture': summary, 'speed_command_mps': float(speed), 'steering_angle_rad': float(angle)})
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
