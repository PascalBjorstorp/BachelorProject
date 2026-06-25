#!/usr/bin/env python3
"""Open-loop F1TENTH single-track plant used for identification.

This reproduces the hardware-like plant in test_sim_drive.c: RK4 integration,
rate-limited steering, optional first-order actuator lag, Pacejka-like lateral
forces, longitudinal load transfer, rolling resistance, and coastdown drag.
It intentionally contains no MPC code or sensor noise.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Iterable
import json
import math
from pathlib import Path
import numpy as np

G = 9.81
PHYSICAL_MAX_ACCEL = 7.31


@dataclass
class PlantParameters:
    # Actuation
    steer_delay_s: float = 0.0
    accel_delay_s: float = 0.0
    steer_angle_max: float = 0.39
    steer_rate_max: float = 2.8492
    steer_tau_s: float = 0.0
    steer_gain: float = 1.0085301459687404
    steer_gain_high_slip: float = 0.6541720766809247
    accel_tau_pos_s: float = 0.05
    accel_tau_neg_s: float = 0.12
    accel_gain_pos: float = 0.575
    accel_gain_neg: float = 1.0
    drag_c0: float = 0.0
    drag_c1: float = 0.05
    drag_c2: float = 0.04
    roll_resistance_n: float = 2.79
    # Vehicle / tyres
    mass_kg: float = 3.57912
    iz_kgm2: float = 0.035
    lf_m: float = 0.166
    lr_m: float = 0.160
    h_cg_m: float = 0.0703
    mu_front: float = 0.775687
    mu_rear: float = 0.6565520426481404
    csf: float = 4.78281642069513
    csr: float = 2.73123678240426
    csf_high_slip: float = 2.4199490875105907
    csr_high_slip: float = 2.73123678240426
    pacejka_c_front: float = 1.8031639754063644
    pacejka_c_rear: float = 1.7681655069132207
    slip_blend_start_front: float = 0.1643527788471148
    slip_blend_end_front: float = 0.5319307735091576
    slip_blend_start_rear: float = 0.2502122916247753
    slip_blend_end_rear: float = 0.47793678552502167
    # Tyre relaxation length: lateral force lags the slip angle with time
    # constant sigma/v. This is the dominant unmodelled transient-yaw effect; at
    # sigma->0 the model recovers the instantaneous-force behaviour.
    rel_len_front_m: float = 0.08
    rel_len_rear_m: float = 0.08
    combined_slip_gain: float = 0.10359393575265835
    front_peak_drop: float = 0.11804981810838257
    front_peak_drop_start: float = 0.13813810031946996
    front_peak_drop_end: float = 0.48938120479012814
    front_peak_drop_pow: float = 1.03
    front_combined_gain: float = 0.13366870620631957
    front_peak_floor: float = 0.2708096984131235
    v_switch_mps: float = 7.319
    v_min_mps: float = 0.5
    v_max_mps: float = 21.6

    @classmethod
    def from_json(cls, path: str | Path) -> "PlantParameters":
        raw = json.loads(Path(path).read_text())
        raw.pop("metadata", None)
        known = {name: raw[name] for name in cls.__dataclass_fields__ if name in raw}
        return cls(**known)

    def save_json(self, path: str | Path, **metadata) -> None:
        payload = asdict(self)
        if metadata:
            payload["metadata"] = metadata
        Path(path).write_text(json.dumps(payload, indent=2) + "\n")


@dataclass
class PlantState:
    x: float
    y: float
    delta: float
    speed: float
    yaw: float
    yaw_rate: float
    beta: float
    alpha_f_lag: float = 0.0
    alpha_r_lag: float = 0.0
    accel_applied: float = 0.0
    steer_filter: float = 0.0

    def dynamic_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.delta, self.speed, self.yaw, self.yaw_rate,
                         self.beta, self.alpha_f_lag, self.alpha_r_lag], dtype=float)

    @classmethod
    def from_dynamic(cls, x: np.ndarray, accel_applied: float, steer_filter: float) -> "PlantState":
        return cls(*[float(v) for v in x], accel_applied=float(accel_applied), steer_filter=float(steer_filter))


def _smoothstep(value: float, start: float, end: float) -> float:
    if end <= start:
        return 0.0
    u = min(1.0, max(0.0, (value - start) / (end - start)))
    return u * u * (3.0 - 2.0 * u)


def _rhs(state: np.ndarray, steer_velocity: float, acceleration: float, p: PlantParameters) -> np.ndarray:
    x, y, delta, speed, yaw, yaw_rate, beta, alpha_f_lag, alpha_r_lag = state
    lwb = p.lf_m + p.lr_m
    if speed < 0.5:
        delta_eff = p.steer_gain * delta
        steer_eff = p.steer_gain * steer_velocity
        tan_delta = math.tan(delta_eff)
        beta_hat = math.atan(tan_delta * p.lr_m / lwb)
        beta_dot = (1.0 / (1.0 + (tan_delta * p.lr_m / lwb) ** 2.0)) * (
            p.lr_m / (lwb * math.cos(delta_eff) ** 2.0)
        ) * steer_eff
        return np.array([
            speed * math.cos(yaw + beta_hat),
            speed * math.sin(yaw + beta_hat),
            steer_velocity,
            acceleration,
            speed * math.cos(beta_hat) * tan_delta / lwb,
            (acceleration * math.cos(beta) * tan_delta
             - speed * math.sin(beta) * tan_delta * beta_dot
             + speed * math.cos(beta) * steer_eff / math.cos(delta_eff) ** 2.0) / lwb,
            beta_dot,
            0.0, 0.0,
        ])

    vx = speed * math.cos(beta)
    vy = speed * math.sin(beta)
    vx_safe = max(vx, 0.5)
    fzf = p.mass_kg * (G * p.lr_m - acceleration * p.h_cg_m) / lwb
    fzr = p.mass_kg * (G * p.lf_m + acceleration * p.h_cg_m) / lwb

    alpha_f_raw = p.steer_gain * delta - math.atan2(vy + p.lf_m * yaw_rate, vx_safe)
    steer_blend = _smoothstep(abs(alpha_f_raw), p.slip_blend_start_front, p.slip_blend_end_front)
    steer_gain_eff = p.steer_gain + (p.steer_gain_high_slip - p.steer_gain) * steer_blend
    delta_eff = steer_gain_eff * delta
    # Instantaneous (steady-state) slip angles drive the relaxation states.
    alpha_f_static = delta_eff - math.atan2(vy + p.lf_m * yaw_rate, vx_safe)
    alpha_r_static = -math.atan2(vy - p.lr_m * yaw_rate, vx_safe)
    # First-order lag with length-based time constant sigma/v: lateral force
    # follows the lagged slip, not the instantaneous one. This is what gives the
    # correct transient-yaw timing the static model was missing.
    rate_f = vx_safe / max(p.rel_len_front_m, 1e-3)
    rate_r = vx_safe / max(p.rel_len_rear_m, 1e-3)
    d_alpha_f = (alpha_f_static - alpha_f_lag) * rate_f
    d_alpha_r = (alpha_r_static - alpha_r_lag) * rate_r
    alpha_f = alpha_f_lag
    alpha_r = alpha_r_lag
    sf, sr = abs(alpha_f), abs(alpha_r)
    blend_f = _smoothstep(sf, p.slip_blend_start_front, p.slip_blend_end_front)
    blend_r = _smoothstep(sr, p.slip_blend_start_rear, p.slip_blend_end_rear)
    csf = p.csf + (p.csf_high_slip - p.csf) * blend_f
    csr = p.csr + (p.csr_high_slip - p.csr) * blend_r

    accel_usage = min(1.0, abs(acceleration) / PHYSICAL_MAX_ACCEL)
    combined_scale = max(0.30, 1.0 - p.combined_slip_gain * accel_usage)
    drop_blend = _smoothstep(sf, p.front_peak_drop_start, p.front_peak_drop_end)
    if p.front_peak_drop_pow > 1.0:
        drop_blend **= p.front_peak_drop_pow
    front_peak_scale = max(p.front_peak_floor, 1.0 - p.front_peak_drop * drop_blend)
    front_combined_scale = max(p.front_peak_floor, 1.0 - p.front_combined_gain * accel_usage * drop_blend)
    front_peak_scale *= front_combined_scale

    bf = csf / p.pacejka_c_front
    br = csr / p.pacejka_c_rear
    df = p.mu_front * fzf * front_peak_scale
    dr = p.mu_rear * fzr
    fyf = combined_scale * df * math.sin(p.pacejka_c_front * math.atan(bf * alpha_f))
    fyr = combined_scale * dr * math.sin(p.pacejka_c_rear * math.atan(br * alpha_r))

    fx = p.mass_kg * acceleration - p.roll_resistance_n
    v_abs = abs(vx)
    drag_accel = p.drag_c0 + p.drag_c1 * v_abs + p.drag_c2 * v_abs * v_abs
    if drag_accel > 0.0:
        fx -= p.mass_kg * drag_accel
    cdelta, sdelta = math.cos(delta_eff), math.sin(delta_eff)
    dvx = (fx - fyf * sdelta + p.mass_kg * vy * yaw_rate) / p.mass_kg
    dvy = (fyf * cdelta + fyr - p.mass_kg * vx * yaw_rate) / p.mass_kg
    v_safe = max(speed, 1e-3)
    v_sq = max(speed * speed, 1e-3)
    return np.array([
        speed * math.cos(yaw + beta),
        speed * math.sin(yaw + beta),
        steer_velocity,
        (vx * dvx + vy * dvy) / v_safe,
        yaw_rate,
        (p.lf_m * fyf * cdelta - p.lr_m * fyr) / p.iz_kgm2,
        (vx * dvy - vy * dvx) / v_sq,
        d_alpha_f,
        d_alpha_r,
    ])


def _wrap(yaw: float) -> float:
    return (yaw + math.pi) % (2.0 * math.pi) - math.pi


def step(state: PlantState, steer_command: float, accel_command: float, dt: float,
         p: PlantParameters) -> PlantState:
    """Advance the deterministic plant by one integration step."""
    steer_command = float(np.clip(steer_command, -p.steer_angle_max, p.steer_angle_max))
    accel_command = float(np.clip(accel_command, -PHYSICAL_MAX_ACCEL, PHYSICAL_MAX_ACCEL))
    # First-order command smoothing, then physical rate-limited steering state.
    steer_filter = steer_command if p.steer_tau_s <= 1e-6 else state.steer_filter + min(1.0, dt / p.steer_tau_s) * (steer_command - state.steer_filter)
    diff = steer_filter - state.delta
    steer_velocity = 0.0 if abs(diff) <= 1e-4 else math.copysign(p.steer_rate_max, diff)
    if state.delta >= p.steer_angle_max and steer_velocity > 0.0:
        steer_velocity = 0.0
    if state.delta <= -p.steer_angle_max and steer_velocity < 0.0:
        steer_velocity = 0.0

    gain = p.accel_gain_pos if accel_command >= 0.0 else p.accel_gain_neg
    target_accel = accel_command * gain
    tau = p.accel_tau_pos_s if target_accel >= 0.0 else p.accel_tau_neg_s
    accel_applied = target_accel if tau <= 1e-6 else state.accel_applied + min(1.0, dt / tau) * (target_accel - state.accel_applied)
    if state.speed > p.v_switch_mps:
        accel_applied = min(accel_applied, PHYSICAL_MAX_ACCEL * p.v_switch_mps / state.speed)
    accel_applied = float(np.clip(accel_applied, -PHYSICAL_MAX_ACCEL, PHYSICAL_MAX_ACCEL))
    if state.speed <= p.v_min_mps and accel_applied < 0.0:
        accel_applied = 0.0
    if state.speed >= p.v_max_mps and accel_applied > 0.0:
        accel_applied = 0.0

    q = state.dynamic_vector()
    k1 = _rhs(q, steer_velocity, accel_applied, p)
    k2 = _rhs(q + 0.5 * dt * k1, steer_velocity, accel_applied, p)
    k3 = _rhs(q + 0.5 * dt * k2, steer_velocity, accel_applied, p)
    k4 = _rhs(q + dt * k3, steer_velocity, accel_applied, p)
    q_next = q + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    q_next[2] = float(np.clip(q_next[2], -p.steer_angle_max, p.steer_angle_max))
    q_next[3] = float(np.clip(q_next[3], p.v_min_mps, p.v_max_mps))
    q_next[4] = _wrap(float(q_next[4]))
    q_next[7] = float(np.clip(q_next[7], -1.5, 1.5))
    q_next[8] = float(np.clip(q_next[8], -1.5, 1.5))
    return PlantState.from_dynamic(q_next, accel_applied=accel_applied, steer_filter=steer_filter)


def zoh(times: np.ndarray, values: np.ndarray, query_time: float) -> float:
    i = int(np.searchsorted(times, query_time, side="right") - 1)
    i = min(max(i, 0), len(values) - 1)
    return float(values[i])


def rollout(sample_times: np.ndarray, command_times: np.ndarray, steer: np.ndarray, accel: np.ndarray,
            initial: PlantState, p: PlantParameters, sim_dt: float = 0.002) -> dict[str, np.ndarray]:
    """Simulate a segment and report the state at each requested sample time."""
    if len(sample_times) < 1:
        return {}
    if np.any(np.diff(sample_times) < 0.0):
        raise ValueError("sample_times must be monotonic")
    current = initial
    now = float(sample_times[0])
    rows = [current]
    for target in sample_times[1:]:
        target = float(target)
        while now < target - 1e-10:
            h = min(sim_dt, target - now)
            u_delta = zoh(command_times, steer, now - p.steer_delay_s)
            u_accel = zoh(command_times, accel, now - p.accel_delay_s)
            current = step(current, u_delta, u_accel, h, p)
            now += h
        rows.append(current)
    return {
        "x": np.array([s.x for s in rows]), "y": np.array([s.y for s in rows]),
        "yaw": np.array([s.yaw for s in rows]), "delta": np.array([s.delta for s in rows]),
        "speed": np.array([s.speed for s in rows]), "yaw_rate": np.array([s.yaw_rate for s in rows]),
        "beta": np.array([s.beta for s in rows]), "accel_applied": np.array([s.accel_applied for s in rows]),
    }


def initial_state_from_observation(row, p: "PlantParameters | None" = None) -> PlantState:
    """Seed the rollout from the ICP-consistent velocity state.

    The position residual is measured against the ICP pose, so the velocity that
    propagates it must come from the same source. odom velocities are biased
    relative to ICP and were the dominant multiple-shooting error; the ICP-derived
    speed/sideslip/yaw-rate columns (icp_*) keep the initial condition consistent
    with the truth. odom is only a fallback if those columns are absent.
    """
    has_icp = hasattr(row, "icp_speed") and np.isfinite(row.icp_speed)
    if has_icp:
        speed = max(0.5, float(row.icp_speed))
        beta = float(row.icp_beta)
        yaw_rate = float(row.icp_yaw_rate)
    else:
        vx, vy = float(row.odom_vx), float(row.odom_vy)
        speed = max(0.5, math.hypot(vx, vy))
        beta = math.atan2(vy, max(vx, 1e-3))
        yaw_rate = float(row.odom_wz)
    delta = float(row.delta_feedback_rad) if np.isfinite(row.delta_feedback_rad) else float(row.cmd_steering_angle)
    # Seed the relaxation states at the steady-state slip for this initial
    # condition so each shooting window starts without a spurious force transient.
    if p is not None and speed >= 0.5:
        vx = speed * math.cos(beta)
        vy = speed * math.sin(beta)
        vx_safe = max(vx, 0.5)
        alpha_f_lag = p.steer_gain * delta - math.atan2(vy + p.lf_m * yaw_rate, vx_safe)
        alpha_r_lag = -math.atan2(vy - p.lr_m * yaw_rate, vx_safe)
    else:
        alpha_f_lag = alpha_r_lag = 0.0
    return PlantState(
        x=float(row.x), y=float(row.y), delta=delta, speed=speed, yaw=float(row.yaw),
        yaw_rate=yaw_rate, beta=beta, alpha_f_lag=alpha_f_lag, alpha_r_lag=alpha_r_lag,
        accel_applied=float(row.cmd_acceleration), steer_filter=delta,
    )
