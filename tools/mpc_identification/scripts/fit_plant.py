#!/usr/bin/env python3
"""Stage-wise open-loop fitting of the dynamic single-track plant.

Stages are deliberately narrow to avoid compensating errors:
  longitudinal: straight-ish segments fit drive/brake delay/gain/lag/drag.
  lateral: turning segments fit tyre scales and yaw inertia.

The optimizer uses multiple shooting: every segment starts at the observed ICP
pose and odometry state. It therefore measures one-to-two-second predictive
accuracy instead of accumulating a whole-lap initialization error.
"""
from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path
from typing import Sequence

import numpy as np
import pandas as pd
from scipy.optimize import least_squares

from geometry import wrap_angle
from plant_model import PlantParameters, initial_state_from_observation, rollout


# Physical priors (nominal value, allowed spread) used to regularise the fit.
# Open-loop windowed fitting of a single-track plant is ill-conditioned (the front
# force decomposition csf/mu/steer_gain is degenerate), so an unregularised fit
# rails parameters to unphysical effective values that minimise windowed error but
# WRECK closed-loop fidelity: the nominal controller then under/over-rotates and
# crashes where the real car did not. Pulling each parameter toward its measured
# physical value (Tikhonov / Gaussian prior) breaks the degeneracy and keeps the
# identified plant a real car the existing controller can drive. Tight spreads =
# trust the physical value (geometry, inertia); wide spreads = let data lead.
PHYSICAL_PRIORS: dict[str, tuple[float, float]] = {
    # Front grip (csf, mu_front) is anchored near the nominal the proven
    # controller drives, so the identified plant never understeers below a
    # controllable car. The open-loop fit otherwise drops front grip to overstate
    # understeer, which is closed-loop-fatal at the hairpin. Rear/high-slip are
    # looser.
    "csf": (4.78, 1.0), "csr": (2.73, 2.0),
    "csf_high_slip": (3.0, 3.0), "csr_high_slip": (3.0, 3.0),
    "mu_front": (0.775, 0.05), "mu_rear": (0.656, 0.08),
    "iz_kgm2": (0.038, 0.006), "lf_m": (0.166, 0.006), "lr_m": (0.160, 0.006),
    "steer_gain": (0.95, 0.07), "pacejka_c_front": (1.80, 0.15), "pacejka_c_rear": (1.77, 0.15),
    "rel_len_front_m": (0.10, 0.10), "rel_len_rear_m": (0.10, 0.10),
    "front_peak_drop": (0.12, 0.10),
    "accel_gain_pos": (0.8, 0.4), "accel_gain_neg": (1.0, 0.4),
    "accel_tau_pos_s": (0.05, 0.06), "accel_tau_neg_s": (0.12, 0.10),
    "drag_c0": (0.0, 0.3), "drag_c1": (0.05, 0.2), "drag_c2": (0.04, 0.1),
    "accel_delay_s": (0.0, 0.05),
}


def measure_accel_gains(observed: pd.DataFrame, drive: pd.DataFrame,
                        max_steer: float = 0.08) -> dict[str, float]:
    """Directly identify throttle/brake effectiveness and constant drag.

    Regresses the achieved longitudinal acceleration (from the smoothed ICP speed)
    against the commanded acceleration on near-straight samples, separately for
    throttle and brake. This is far more reliable than fitting the gains through
    cornering-contaminated multiple-shooting windows, which blame corner speed-loss
    on weak throttle and leave the simulated car accelerating too hard / braking
    too little -- the exact error that lets it overspeed the hairpin and crash.
    """
    from scipy.signal import savgol_filter
    t = observed.t_s.to_numpy()
    sp = observed.icp_speed.to_numpy() if "icp_speed" in observed.columns else observed.speed_body_mps.to_numpy()
    tu = np.arange(t[0], t[-1], 0.02)
    accel = np.gradient(savgol_filter(np.interp(tu, t, sp), 21, 2), tu)
    cmd = np.interp(tu, drive.t_s.to_numpy(), drive.acceleration.to_numpy())
    steer = np.abs(np.interp(tu, drive.t_s.to_numpy(), drive.steering_angle.to_numpy()))
    speed = np.interp(tu, t, sp)
    straight = steer < max_steer
    # Throttle gain and a constant offset (rolling resistance + low-speed drag).
    pos = straight & (cmd > 0.5)
    a_pos = np.column_stack([cmd[pos], np.ones(pos.sum())])
    gain_pos, offset = np.linalg.lstsq(a_pos, accel[pos], rcond=None)[0]
    # Brake gain (offset folded in via the same constant decel).
    neg = straight & (cmd < -0.5)
    gain_neg = float(np.linalg.lstsq(cmd[neg, None], accel[neg] - offset, rcond=None)[0][0])
    return {
        "accel_gain_pos": float(np.clip(gain_pos, 0.3, 1.1)),
        "accel_gain_neg": float(np.clip(gain_neg, 0.3, 1.2)),
        "constant_decel_mps2": float(-offset),
        "n_pos": int(pos.sum()), "n_neg": int(neg.sum()),
    }


def select_segments(segments: pd.DataFrame, stage: str, split: str, maximum: int,
                    longitudinal_max_steer: float, longitudinal_max_yaw_rate: float) -> pd.DataFrame:
    q = segments[segments.split == split].copy()
    if stage == "longitudinal":
        # This track has no long zero-steer straight. Select the least-coupled
        # high-speed windows instead of requiring unrealistically small steering.
        q = q[(q.mean_abs_steer <= longitudinal_max_steer) &
              (q.mean_abs_yaw_rate <= longitudinal_max_yaw_rate)]
    elif stage == "lateral":
        q = q[(q.mean_abs_steer >= 0.06) & (q.mean_speed >= 2.0)]
    q = q.sort_values(["mean_speed", "segment_id"], ascending=[False, True])
    return q.head(maximum)


def mutate(p: PlantParameters, names: Sequence[str], values: np.ndarray) -> PlantParameters:
    q = copy.deepcopy(p)
    for name, value in zip(names, values):
        setattr(q, name, float(value))
    return q


def residuals(params: PlantParameters, observed: pd.DataFrame, segments: pd.DataFrame,
              drive: pd.DataFrame, stage: str, sim_dt: float) -> np.ndarray:
    cmd_t = drive.t_s.to_numpy()
    cmd_delta = drive.steering_angle.to_numpy()
    cmd_accel = drive.acceleration.to_numpy()
    pieces = []
    for seg in segments.itertuples(index=False):
        sample = observed.iloc[seg.start_index:seg.end_index + 1]
        times = sample.t_s.to_numpy()
        pred = rollout(times, cmd_t, cmd_delta, cmd_accel, initial_state_from_observation(sample.iloc[0], params), params, sim_dt=sim_dt)
        speed_ref = _ref(sample, "icp_speed", "speed_body_mps")
        yaw_rate_ref = _ref(sample, "icp_yaw_rate", "odom_wz")
        if stage == "longitudinal":
            # State x/y are excluded because these mostly straight segments are used
            # to identify drivetrain rather than coupled tyre dynamics.
            pieces.append((pred["speed"] - speed_ref) / 0.20)
        else:
            pieces.append((pred["x"] - sample.x.to_numpy()) / 0.06)
            pieces.append((pred["y"] - sample.y.to_numpy()) / 0.06)
            pieces.append(wrap_angle(pred["yaw"] - sample.yaw.to_numpy()) / 0.06)
            pieces.append((pred["yaw_rate"] - yaw_rate_ref) / 0.40)
    return np.concatenate(pieces) if pieces else np.array([1e6])


def _ref(sample: pd.DataFrame, primary: str, fallback: str) -> np.ndarray:
    """Prefer the ICP-consistent reference; fall back to odom if unavailable."""
    if primary in sample.columns and np.all(np.isfinite(sample[primary].to_numpy())):
        return sample[primary].to_numpy()
    return sample[fallback].to_numpy()


def evaluate(params: PlantParameters, observed: pd.DataFrame, segments: pd.DataFrame,
             drive: pd.DataFrame, sim_dt: float) -> tuple[pd.DataFrame, dict]:
    cmd_t = drive.t_s.to_numpy()
    cmd_delta = drive.steering_angle.to_numpy()
    cmd_accel = drive.acceleration.to_numpy()
    rows = []
    for seg in segments.itertuples(index=False):
        sample = observed.iloc[seg.start_index:seg.end_index + 1]
        pred = rollout(sample.t_s.to_numpy(), cmd_t, cmd_delta, cmd_accel,
                       initial_state_from_observation(sample.iloc[0], params), params, sim_dt=sim_dt)
        speed_ref = _ref(sample, "icp_speed", "speed_body_mps")
        yaw_rate_ref = _ref(sample, "icp_yaw_rate", "odom_wz")
        for j, (_, obs) in enumerate(sample.iterrows()):
            rows.append({
                "segment_id": seg.segment_id, "lap": seg.lap, "split": seg.split, "t_s": obs.t_s,
                "x_obs": obs.x, "y_obs": obs.y, "yaw_obs": obs.yaw, "speed_obs": speed_ref[j],
                "yaw_rate_obs": yaw_rate_ref[j], "x_pred": pred["x"][j], "y_pred": pred["y"][j],
                "yaw_pred": pred["yaw"][j], "speed_pred": pred["speed"][j], "yaw_rate_pred": pred["yaw_rate"][j],
                "delta_pred": pred["delta"][j], "accel_applied_pred": pred["accel_applied"][j],
            })
    frame = pd.DataFrame(rows)
    dx = frame.x_pred - frame.x_obs
    dy = frame.y_pred - frame.y_obs
    pos = np.hypot(dx, dy)
    metrics = {
        "n_samples": int(len(frame)),
        "position_rmse_m": float(np.sqrt(np.mean(pos ** 2))),
        "position_mae_m": float(np.mean(np.abs(pos))),
        "yaw_rmse_rad": float(np.sqrt(np.mean(wrap_angle(frame.yaw_pred - frame.yaw_obs) ** 2))),
        "speed_rmse_mps": float(np.sqrt(np.mean((frame.speed_pred - frame.speed_obs) ** 2))),
        "yaw_rate_rmse_radps": float(np.sqrt(np.mean((frame.yaw_rate_pred - frame.yaw_rate_obs) ** 2))),
    }
    return frame, metrics


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("stage", choices=["longitudinal", "lateral"])
    parser.add_argument("--observed", type=Path, required=True)
    parser.add_argument("--segments", type=Path, required=True)
    parser.add_argument("--drive", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True, help="Input/base plant parameters JSON")
    parser.add_argument("--output", type=Path, required=True, help="Fitted parameters JSON")
    parser.add_argument("--trace-output", type=Path, required=True)
    parser.add_argument("--metrics-output", type=Path, required=True)
    parser.add_argument("--split", choices=["train", "validation", "test"], default="train")
    parser.add_argument("--max-segments", type=int, default=12)
    parser.add_argument("--longitudinal-max-steer", type=float, default=0.14)
    parser.add_argument("--longitudinal-max-yaw-rate", type=float, default=1.0)
    # Match the C simulator step (SIM_DT=0.002) exactly so identified parameters
    # are not biased by any integration-resolution mismatch between fit and sim.
    parser.add_argument("--sim-dt", type=float, default=0.002)
    # Generous budget for a thorough multi-parameter fit; tolerances stop early
    # once converged. Raise further for an even longer run.
    parser.add_argument("--max-nfev", type=int, default=1500)
    # Strength of the pull toward PHYSICAL_PRIORS. Keeps the plant a real,
    # controllable car instead of railing to unphysical effective values.
    parser.add_argument("--reg-weight", type=float, default=0.30)
    # Residuals are pre-normalised to ~1 sigma, so f_scale controls how far into
    # the informative dynamic deviation Huber stays quadratic before robustifying
    # genuine outliers (e.g. a bad ICP pose). 1.0 was discarding real signal.
    parser.add_argument("--f-scale", type=float, default=3.0)
    args = parser.parse_args()
    for path in [args.output, args.trace_output, args.metrics_output]:
        path.parent.mkdir(parents=True, exist_ok=True)

    observed = pd.read_csv(args.observed)
    segments = pd.read_csv(args.segments)
    drive = pd.read_csv(args.drive).sort_values("t_s")
    selected = select_segments(
        segments, args.stage, args.split, args.max_segments,
        args.longitudinal_max_steer, args.longitudinal_max_yaw_rate,
    )
    if len(selected) < 4:
        raise RuntimeError(f"Only {len(selected)} usable {args.stage} segments. Relax filters or inspect data.")
    base = PlantParameters.from_json(args.parameters)

    if args.stage == "longitudinal":
        # Throttle/brake effectiveness and the constant decel are measured
        # DIRECTLY from achieved-vs-commanded acceleration (robust, physical),
        # then held fixed. The corner-contaminated windowed fit cannot be trusted
        # with the gains -- it made the car accelerate too hard / brake too little
        # and overspeed the hairpin. The window fit only refines the actuator lag
        # and the speed-dependent drag here.
        meas = measure_accel_gains(observed, drive)
        base.accel_gain_pos = meas["accel_gain_pos"]
        base.accel_gain_neg = meas["accel_gain_neg"]
        # Map the measured constant deceleration onto rolling resistance (a*m=F).
        base.roll_resistance_n = float(np.clip(meas["constant_decel_mps2"] * base.mass_kg, 0.0, 8.0))
        base.drag_c0 = 0.0
        print(f"[longitudinal] measured gains: pos={meas['accel_gain_pos']:.3f} "
              f"neg={meas['accel_gain_neg']:.3f} const_decel={meas['constant_decel_mps2']:.3f} m/s^2 "
              f"-> roll_res={base.roll_resistance_n:.2f} N (n_pos={meas['n_pos']}, n_neg={meas['n_neg']})")
        names = ["accel_delay_s", "accel_tau_pos_s", "accel_tau_neg_s", "drag_c1", "drag_c2"]
        lb = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        ub = np.array([0.15, 0.30, 0.40, 0.50, 0.25])
    else:
        # Rich PHYSICAL set, regularised toward PHYSICAL_PRIORS to break the
        # front-force degeneracy. steer_gain is deliberately NOT fitted: it is
        # degenerate with front stiffness (holding it at nominal 1.0 gives the
        # same open-loop accuracy) but an off-nominal steering scale makes the
        # plant steer unlike the controller's model and crashes the closed loop.
        # We keep the kinematic steering response matched to the controller and
        # let the genuine understeer fall into front grip/stiffness instead.
        names = ["csf", "csr", "csf_high_slip", "csr_high_slip", "mu_front", "mu_rear",
                 "iz_kgm2", "lf_m", "lr_m", "pacejka_c_front",
                 "rel_len_front_m", "rel_len_rear_m", "front_peak_drop"]
        lb = np.array([1.0, 1.0, 0.5, 0.5, 0.45, 0.45, 0.030, 0.150, 0.145, 1.3, 0.02, 0.02, 0.0])
        ub = np.array([12.0, 12.0, 10.0, 10.0, 1.10, 1.10, 0.060, 0.182, 0.176, 2.1, 0.60, 0.60, 0.45])
    x0 = np.array([getattr(base, n) for n in names], dtype=float)

    prior_mu = np.array([PHYSICAL_PRIORS.get(n, (getattr(base, n), 1e9))[0] for n in names])
    prior_sd = np.array([PHYSICAL_PRIORS.get(n, (0.0, 1e9))[1] for n in names])

    def objective(x: np.ndarray) -> np.ndarray:
        data_res = residuals(mutate(base, names, x), observed, selected, drive, args.stage, args.sim_dt)
        # Tikhonov pull toward the physical prior, scaled by sqrt(N_data) so its
        # strength is invariant to how many residual samples the split produced.
        reg_res = args.reg_weight * np.sqrt(max(len(data_res), 1)) * (x - prior_mu) / prior_sd
        return np.concatenate([data_res, reg_res])

    before = objective(x0)
    # x_scale="jac" rescales the step per parameter from the Jacobian columns. It
    # is essential here: the parameters span ~0.0025 (delay) to ~12 (cornering
    # stiffness), and a single global scale otherwise starves the small ones.
    result = least_squares(objective, x0=x0, bounds=(lb, ub), loss="huber", f_scale=args.f_scale,
                           x_scale="jac", max_nfev=args.max_nfev, verbose=1)
    fitted = mutate(base, names, result.x)
    after = objective(result.x)
    fitted.save_json(args.output, stage=args.stage, success=bool(result.success), message=result.message,
                     selected_segments=int(len(selected)), optimized_fields=names)

    traces, metrics = evaluate(fitted, observed, selected, drive, args.sim_dt)
    traces.to_csv(args.trace_output, index=False)
    diagnostics = {
        "stage": args.stage, "success": bool(result.success), "message": result.message,
        "selected_segment_count": int(len(selected)), "optimized_fields": names,
        "longitudinal_selection": {
            "max_mean_abs_steer": args.longitudinal_max_steer,
            "max_mean_abs_yaw_rate": args.longitudinal_max_yaw_rate,
        } if args.stage == "longitudinal" else None,
        "initial_weighted_rmse": float(np.sqrt(np.mean(before ** 2))),
        "final_weighted_rmse": float(np.sqrt(np.mean(after ** 2))),
        "parameters": {n: float(getattr(fitted, n)) for n in names},
        "metrics": metrics,
    }
    args.metrics_output.write_text(json.dumps(diagnostics, indent=2) + "\n")
    print(json.dumps(diagnostics, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
