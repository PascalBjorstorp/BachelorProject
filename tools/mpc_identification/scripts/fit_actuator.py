#!/usr/bin/env python3
"""Identify steering transport delay, scale, bias, lag, and rate limit.

The response signal is `/sensors/servo_position_command` converted through the
same inverse polynomial used by mpc_hardware_node.c. This topic may be an echo
of the VESC command rather than a physical encoder; therefore the result must be
read as an actuator-command model unless independent wheel-angle feedback is
added later.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import numpy as np
import pandas as pd
from scipy.optimize import least_squares


def servo_to_angle(servo: np.ndarray, gain: float, offset: float, c0: float, c1: float, c2: float) -> np.ndarray:
    corrected = (servo - offset) / gain
    sign = np.sign(corrected)
    magnitude = np.abs(corrected)
    disc = c1 * c1 - 4.0 * c2 * (c0 - magnitude)
    out = corrected.copy()
    good = disc >= 0.0
    out[good] = sign[good] * (-c1 + np.sqrt(disc[good])) / (2.0 * c2)
    return out


def zoh(times: np.ndarray, values: np.ndarray, query: float) -> float:
    i = int(np.searchsorted(times, query, side="right") - 1)
    return float(values[min(max(i, 0), len(values) - 1)])


def simulate(sample_times: np.ndarray, command_times: np.ndarray, command: np.ndarray,
             params: np.ndarray, dt: float = 0.0025) -> np.ndarray:
    delay, gain, bias, tau, rate = params
    target_t = sample_times[0]
    delta = float(command[0] * gain + bias)
    filt = delta
    out = [delta]
    for t_end in sample_times[1:]:
        while target_t < t_end - 1e-12:
            h = min(dt, t_end - target_t)
            u = np.clip(gain * zoh(command_times, command, target_t - delay) + bias, -0.39, 0.39)
            if tau > 1e-5:
                filt += min(1.0, h / tau) * (u - filt)
            else:
                filt = u
            change = np.clip(filt - delta, -rate * h, rate * h)
            delta = np.clip(delta + change, -0.39, 0.39)
            target_t += h
        out.append(delta)
    return np.asarray(out)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--drive", type=Path, required=True)
    parser.add_argument("--servo-feedback", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--servo-gain", type=float, default=-0.7284)
    parser.add_argument("--servo-offset", type=float, default=0.55)
    parser.add_argument("--servo-c0", type=float, default=0.001490)
    parser.add_argument("--servo-c1", type=float, default=0.918061)
    parser.add_argument("--servo-c2", type=float, default=0.589566)
    parser.add_argument("--sample-period", type=float, default=0.020)
    parser.add_argument("--integration-dt", type=float, default=0.010)
    parser.add_argument("--max-nfev", type=int, default=200)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    drive = pd.read_csv(args.drive).sort_values("t_s")
    servo = pd.read_csv(args.servo_feedback).sort_values("t_s")
    raw_times = servo.t_s.to_numpy()
    raw_delta = servo_to_angle(servo.value.to_numpy(), args.servo_gain, args.servo_offset, args.servo_c0, args.servo_c1, args.servo_c2)
    # Uniform fitting grid limits the objective to ~10k values and removes log-time jitter.
    sample_t = np.arange(raw_times[0], raw_times[-1], args.sample_period)
    measured = np.interp(sample_t, raw_times, raw_delta)
    cmd_t = drive.t_s.to_numpy()
    cmd = drive.steering_angle.to_numpy()

    # Delay/scaling initialization from a cheap grid. The nonlinear step handles lag/rate afterwards.
    delay_candidates = np.arange(0.0, 0.101, args.sample_period)
    best = None
    for d in delay_candidates:
        shifted = np.interp(sample_t - d, cmd_t, cmd, left=cmd[0], right=cmd[-1])
        a = np.column_stack([shifted, np.ones_like(shifted)])
        scale, bias = np.linalg.lstsq(a, measured, rcond=None)[0]
        mse = float(np.mean((scale * shifted + bias - measured) ** 2))
        if best is None or mse < best[0]:
            best = (mse, d, scale, bias)
    x0 = np.array([best[1], best[2], best[3], 0.01, 2.85])
    lb = np.array([0.0, 0.6, -0.08, 0.0, 0.5])
    ub = np.array([0.15, 1.4, 0.08, 0.20, 6.0])

    def residual(x: np.ndarray) -> np.ndarray:
        return simulate(sample_t, cmd_t, cmd, x, dt=args.integration_dt) - measured

    # x_scale="jac" balances the very different parameter magnitudes (delay/bias
    # ~0.01 against rate ~3). f_scale=0.01 rad keeps Huber quadratic within normal
    # tracking error while rejecting servo glitches.
    fit = least_squares(residual, x0=x0, bounds=(lb, ub), loss="huber", f_scale=0.01,
                        x_scale="jac", max_nfev=args.max_nfev, verbose=1)
    predicted = simulate(sample_t, cmd_t, cmd, fit.x, dt=args.integration_dt)
    result = {
        "steer_delay_s": float(fit.x[0]), "steer_gain": float(fit.x[1]), "steer_bias_rad": float(fit.x[2]),
        "steer_tau_s": float(fit.x[3]), "steer_rate_max": float(fit.x[4]),
        "rmse_rad": float(np.sqrt(np.mean((predicted - measured) ** 2))),
        "n_samples": int(len(sample_t)), "cost": float(fit.cost), "success": bool(fit.success), "message": fit.message,
        "warning": "Servo feedback topic is a command-domain signal in the current hardware node; this is not proof of physical wheel-angle tracking."
    }
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    pd.DataFrame({"t_s": sample_t, "delta_observed": measured, "delta_predicted": predicted}).to_csv(
        args.output.with_name(args.output.stem + "_trace.csv"), index=False
    )
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
