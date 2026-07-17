#!/usr/bin/env python3
"""Fit the ERPM candidate from training captures before hold-out validation."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from common import analysis_dir, dump_yaml, load_yaml, session_original_values
from fit_speed_map import _coverage, _fit_static, _summary


def _bootstrap_origin_gain(erpm: np.ndarray, speed: np.ndarray, *,
                           resamples: int = 1000, seed: int = 20260717) -> dict:
    """Trial-resampled interval for the deployable through-origin gain."""
    e = np.asarray(erpm, dtype=float)
    v = np.asarray(speed, dtype=float)
    finite = np.isfinite(e) & np.isfinite(v)
    e, v = e[finite], v[finite]
    gains: list[float] = []
    if len(e) >= 4 and resamples > 0:
        rng = np.random.default_rng(seed)
        for _ in range(resamples):
            indices = rng.integers(0, len(e), len(e))
            bv, be = v[indices], e[indices]
            denominator = float(np.dot(bv, bv))
            if denominator <= 1.0e-12:
                continue
            gain = float(np.dot(bv, be) / denominator)
            if math.isfinite(gain) and gain > 0.0:
                gains.append(gain)
    return {
        "method": "nonparametric bootstrap over independent accepted trials",
        "requested_resamples": int(resamples),
        "valid_resamples": int(len(gains)),
        "gain_erpm_per_mps_95pct": (
            [float(value) for value in np.quantile(gains, [0.025, 0.975])] if gains else []
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    out = analysis_dir(session)
    train = _summary(session, "03_raw_erpm_map_training", "raw_erpm_training", cfg)
    train.to_parquet(out / "erpm_map_training_trials.parquet", index=False)
    spec = cfg["raw_erpm_map_training"]
    cov = _coverage(train, list(map(float, spec["nominal_speeds_mps"])), int(spec["repetitions"]))
    cov.to_parquet(out / "erpm_map_training_coverage.parquet", index=False)
    failures: list[str] = []
    if train.empty or not bool(cov.coverage_ok.all()):
        failures.append("ERPM training coverage incomplete")
    if not train.empty:
        train = train.copy()
        train["commanded_erpm"] = train["selected_speed_erpm"].where(
            np.isfinite(train["selected_speed_erpm"]), train["raw_erpm_target"]
        )
    try:
        command = _fit_static(train.commanded_erpm.to_numpy(float), train.vx_lidar_mps.to_numpy(float))
        measured = _fit_static(train.erpm_measured.to_numpy(float), train.vx_lidar_mps.to_numpy(float))
    except (AttributeError, ValueError, KeyError) as exc:
        failures.append(str(exc))
        command = {"linear": {"gain_erpm_per_mps": math.nan, "quadratic_erpm_per_mps2": 0.0}}
        measured = {"linear": {"gain_erpm_per_mps": math.nan, "quadratic_erpm_per_mps2": 0.0}}
    gain = float(command["linear"].get("gain_erpm_per_mps", math.nan))
    statistics_cfg = cfg.get("analysis", {}).get("statistics", {})
    gain_bootstrap = _bootstrap_origin_gain(
        train.commanded_erpm.to_numpy(float) if not train.empty else np.empty(0),
        train.vx_lidar_mps.to_numpy(float) if not train.empty else np.empty(0),
        resamples=int(statistics_cfg.get("bootstrap_resamples", 1000)),
        seed=int(statistics_cfg.get("bootstrap_seed", 20260717)),
    )
    measured_erpm = train.erpm_measured.to_numpy(float) if not train.empty else np.empty(0)
    speed = train.vx_lidar_mps.to_numpy(float) if not train.empty else np.empty(0)
    unscaled = measured_erpm / max(gain, 1e-12)
    scale_values = np.divide(speed, unscaled, out=np.full(len(speed), np.nan), where=np.abs(unscaled) > 0.08)
    odom_scale = float(np.nanmedian(scale_values)) if np.isfinite(scale_values).any() else math.nan
    stationary = _summary(session, "01_longitudinal_observability", "stationary_observability", cfg)
    stationary_noise = float(stationary.vx_lidar_std_mps.median()) if not stationary.empty else math.nan
    low_speed = _summary(session, "02_low_speed_launch", "low_speed_launch", cfg)
    stable = low_speed[low_speed.vx_lidar_mps >= float(cfg["low_speed_launch"]["minimum_lidar_speed_mps"])] if not low_speed.empty else low_speed
    slow_start = float(stable.vx_lidar_mps.min()) if not stable.empty else float(cfg["low_speed_launch"]["minimum_lidar_speed_mps"])
    original = session_original_values(session)
    if not math.isfinite(odom_scale):
        odom_scale = float(original.get("odom_speed_scale", 1.0) or 1.0)
    report = {
        "training_only": True,
        "candidate_speed_to_erpm_gain": gain,
        "candidate_speed_to_erpm_offset": 0.0,
        "candidate_speed_to_erpm_quadratic": 0.0,
        "candidate_odom_speed_scale": odom_scale,
        "candidate_odom_scale_reference_speed_to_erpm_gain": gain,
        "candidate_slow_start_threshold_mps": slow_start,
        "candidate_slow_start_increment_mps": slow_start,
        "candidate_stop_speed_deadzone_mps": float(cfg["analysis"]["desired_stop_speed_deadband_mps"]),
        "candidate_odom_speed_deadband_mps": max(float(cfg["analysis"]["desired_odom_deadband_mps"]), 3.0 * stationary_noise) if math.isfinite(stationary_noise) else float(cfg["analysis"]["desired_odom_deadband_mps"]),
        "training_points": int(len(train)),
        "training_coverage_ok": not failures,
        "training_command_model": command,
        "training_measured_model": measured,
        "candidate_speed_to_erpm_gain_bootstrap": gain_bootstrap,
        "original_odom_speed_scale": float(original.get("odom_speed_scale", 1.0) or 1.0),
        "accepted_for_update": not failures and math.isfinite(gain) and gain > 0.0,
        "failures": failures,
        "validation_note": "The next hold-out stage validates these applied values before the full pipeline audit.",
    }
    dump_yaml(out / "erpm_speed_map_training_report.yaml", report)
    print(report)
    if not report["accepted_for_update"]:
        raise SystemExit("ERPM training candidate rejected: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
