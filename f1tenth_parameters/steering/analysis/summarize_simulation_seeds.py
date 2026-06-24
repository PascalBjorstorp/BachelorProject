#!/usr/bin/env python3
"""Assemble simulation-seed outputs from the steering calibration session."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd


def _read_parquet(path: Path) -> pd.DataFrame:
    return pd.read_parquet(path) if path.exists() else pd.DataFrame()


def _read_json(path: Path) -> dict | None:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None


def _median_or_none(series: pd.Series) -> float | None:
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy(dtype=float)
    return float(np.median(values)) if len(values) else None


def _max_abs_or_none(series: pd.Series) -> float | None:
    values = pd.to_numeric(series, errors="coerce").dropna().to_numpy(dtype=float)
    return float(np.max(np.abs(values))) if len(values) else None


def _group_table(frame: pd.DataFrame, keys: list[str], fields: list[str]) -> list[dict]:
    if frame.empty:
        return []
    keep = [field for field in fields if field in frame.columns]
    grouped = frame.groupby(keys, dropna=False)[keep].median(numeric_only=True).reset_index()
    return json.loads(grouped.to_json(orient="records"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    out = session / "analysis"

    static_train = _read_parquet(out / "static_map_training_segments.parquet")
    static_hold = _read_parquet(out / "static_map_holdout_segments.parquet")
    response = _read_parquet(out / "command_to_effective_steering_response_metrics.parquet")
    response_group = _read_parquet(out / "effective_steering_response_group_summary.parquet")
    response_summary = _read_json(out / "command_to_effective_steering_response_summary.json")
    steering_summary = _read_json(out / "steering_calibration_review_summary.json")
    static_candidate = _read_json(out / "candidate_static_steering_map.json")
    imu_bias = _read_json(out / "imu_bias.json")

    accepted_static = static_train[static_train.accepted.astype(bool)].copy() if "accepted" in static_train else static_train
    valid_static_hold = static_hold[static_hold.accepted.astype(bool)].copy() if "accepted" in static_hold else static_hold
    valid_response = response[response.effective_response_valid.astype(bool)].copy() if "effective_response_valid" in response else response

    summary = {
        "scope": {
            "purpose": "Early simulation / lateral-model seed values available from the steering calibration session without adding new operator stages.",
            "usable_now": [
                "effective steering lag and time constant",
                "speed-dependent effective steering gain",
                "speed-dependent yaw-rate gain",
                "speed-dependent curvature gain",
                "speed-dependent lateral-acceleration gain",
                "static hysteresis and repeatability",
            ],
            "not_claimed": [
                "full Pacejka coefficients",
                "front/rear axle cornering stiffness separation",
                "slip-angle or self-aligning-torque identification",
                "pure actuator shaft dynamics without a steering-angle sensor",
            ],
        },
        "stationary_imu_bias_removed": imu_bias,
        "static_steering_observations": {
            "candidate_static_map_available": static_candidate is not None,
            "training_points": None if static_candidate is None else static_candidate.get("training_points"),
            "holdout_points": None if static_candidate is None else static_candidate.get("holdout_points"),
            "holdout_rmse_rad": None if static_candidate is None else static_candidate.get("holdout_rmse_rad"),
            "max_observed_curvature_inv_m": _max_abs_or_none(accepted_static.get("curvature_inv_m", pd.Series(dtype=float))),
            "max_observed_yaw_rate_rad_s": _max_abs_or_none(accepted_static.get("yaw_rate", pd.Series(dtype=float))),
            "max_observed_lateral_accel_kinematic_mps2": _max_abs_or_none(accepted_static.get("lateral_accel_kinematic_mps2", pd.Series(dtype=float))),
            "max_observed_imu_lateral_accel_mps2": _max_abs_or_none(accepted_static.get("imu_ay_mean", pd.Series(dtype=float))),
            "median_by_side_fraction": _group_table(
                accepted_static,
                ["side", "fraction_key"],
                ["fraction", "raw_servo_echo", "delta_eq_rad", "curvature_inv_m", "yaw_rate", "lateral_accel_kinematic_mps2", "imu_ay_mean", "vx_lidar"],
            ),
            "holdout_median_by_side_fraction": _group_table(
                valid_static_hold,
                ["side", "fraction_key"],
                ["fraction", "raw_servo_echo", "delta_eq_rad", "curvature_inv_m", "yaw_rate", "lateral_accel_kinematic_mps2", "imu_ay_mean", "vx_lidar"],
            ),
        },
        "dynamic_response_seeds": {
            "response_summary": response_summary,
            "median_effective_delay_10pct_s": None if response_summary is None else response_summary.get("median_effective_delay_10pct_s"),
            "median_effective_rise_10_90_s": None if response_summary is None else response_summary.get("median_effective_rise_10_90_s"),
            "median_effective_settling_5pct_s": None if response_summary is None else response_summary.get("median_effective_settling_5pct_s"),
            "median_fopdt_delay_s": None if response_summary is None else response_summary.get("median_fopdt_delay_s"),
            "median_fopdt_tau_s": None if response_summary is None else response_summary.get("median_fopdt_tau_s"),
            "max_observed_effective_yaw_rate_rad_s": _max_abs_or_none(valid_response.get("effective_final_yaw_rate_rad_s", pd.Series(dtype=float))),
            "max_observed_effective_curvature_inv_m": _max_abs_or_none(valid_response.get("effective_final_curvature_inv_m", pd.Series(dtype=float))),
            "max_observed_effective_lateral_accel_kinematic_mps2": _max_abs_or_none(valid_response.get("effective_final_lateral_accel_kinematic_mps2", pd.Series(dtype=float))),
            "max_observed_effective_imu_lateral_accel_mps2": _max_abs_or_none(valid_response.get("effective_final_imu_lateral_accel_mps2", pd.Series(dtype=float))),
            "median_by_speed_side_fraction": _group_table(
                valid_response,
                ["segment", "speed_mps", "side", "fraction"],
                [
                    "effective_static_gain_rad_per_servo",
                    "effective_curvature_gain_inv_m_per_servo",
                    "effective_yaw_rate_gain_rad_s_per_servo",
                    "effective_lateral_accel_gain_mps2_per_servo",
                    "effective_delay_10pct_s",
                    "effective_rise_10_90_s",
                    "effective_settling_5pct_s",
                    "fopdt_delay_s",
                    "fopdt_tau_s",
                    "effective_final_curvature_inv_m",
                    "effective_final_yaw_rate_rad_s",
                    "effective_final_lateral_accel_kinematic_mps2",
                    "effective_final_imu_lateral_accel_mps2",
                ],
            ),
            "group_summary_rows": json.loads(response_group.to_json(orient="records")) if len(response_group) else [],
        },
        "review_summary": steering_summary,
    }
    (out / "steering_simulation_seed_report.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
