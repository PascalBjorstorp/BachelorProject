#!/usr/bin/env python3
"""Identify the turn-induced longitudinal-slip correction for ERPM odometry.

Real-bag analysis showed the scalar/quadratic ERPM->speed map is accurate on
straights but that the driven wheels over-read ground speed while cornering: the
wheel speed exceeds the LiDAR ground speed by an amount that grows with cornering
load (lateral acceleration), with negligible sideslip and no time lag. A single
ERPM gain therefore cannot give trustworthy odometry in turns.

This stage fits a causal correction from controlled steady arcs (Stage 13,
non-zero steering), using window-averaged LiDAR speed as the independent truth:

    a_lat   = v_wheel * |yaw_rate|                       (causal: ERPM + gyro)
    slip    = 1 - v_ground / v_wheel  =  c1*a_lat (+ c2*a_lat^2)
    v_odom  = v_wheel * (1 - clip(c1*a_lat + c2*a_lat^2, 0, clip_fraction))

The correction is constrained through the origin in a_lat, so straight-line
motion (a_lat = 0) is left exactly as the already-validated ERPM map predicts;
it only removes speed in turns. The coefficient is fit on a training subset of
arc conditions and validated on held-out conditions, and the resulting parameter
is appended to selected_odometry_candidate_patch.yaml for the production port.
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
    dump_yaml,
    expected_grid_coverage,
    load_yaml,
    stage_tables,
    summarize_windows,
)

STAGE = "13_candidate_cross_axis_verification"
PHASE = "candidate_cross_axis_verification"


def _skip(out: Path, reason: str) -> int:
    report = {"status": "skipped", "reason": reason, "accepted_for_candidate": False}
    dump_yaml(out / "turn_slip_report.yaml", report)
    print(json.dumps(report, indent=2))
    return 0


def _wheel_gain(session: Path, out: Path) -> tuple[float, float]:
    """Selected ERPM->wheel-speed map (linear, quadratic) from the odom patch."""
    patch = out / "selected_odometry_candidate_patch.yaml"
    if patch.is_file():
        params = load_yaml(patch).get("vesc_to_odom_node", {}).get("ros__parameters", {})
        lin = float(params.get("odom_erpm_to_speed_linear", 0.0))
        quad = float(params.get("odom_erpm_to_speed_quadratic", 0.0))
        if abs(lin) > 0.0:
            return lin, quad
    speed_map = out / "erpm_speed_map_report.yaml"
    if speed_map.is_file():
        direct = (load_yaml(speed_map).get("measured_erpm_odometry_map", {})
                  .get("direct_erpm_to_ground_speed_candidate", {}))
        lin = float(direct.get("gain_mps_per_erpm", 0.0))
        quad = float(direct.get("quadratic_mps_per_erpm2", 0.0))
        if abs(lin) > 0.0:
            return lin, quad
    raise FileNotFoundError("no selected ERPM->speed map; run fit_odom_model_selection first")


def _condition_split(conditions: list[str]) -> tuple[set[str], set[str]]:
    """Deterministic train/holdout split across distinct (angle, speed) cells."""
    ordered = sorted(set(conditions))
    holdout = {c for i, c in enumerate(ordered) if i % 2 == 1}
    train = {c for c in ordered if c not in holdout}
    return train, holdout


def fit_turn_slip_coefficient(a_lat: np.ndarray, slip_fraction: np.ndarray) -> tuple[float, float]:
    """Origin-constrained fit slip = c1*a_lat; returns (c1, R^2).

    Through the origin so a_lat=0 (straight line) implies zero correction, which
    leaves the already-validated straight-line ERPM gain untouched.
    """
    a = np.asarray(a_lat, float); s = np.asarray(slip_fraction, float)
    m = np.isfinite(a) & np.isfinite(s)
    a, s = a[m], s[m]
    if len(a) < 2:
        return 0.0, math.nan
    c1 = float(np.dot(a, s) / max(np.dot(a, a), 1e-12))
    denom = float(np.sum((s - s.mean()) ** 2))
    r2 = float(1.0 - np.sum((s - c1 * a) ** 2) / denom) if denom > 1e-12 else math.nan
    return c1, r2


def apply_turn_slip(v_wheel: np.ndarray, a_lat: np.ndarray, c1: float, clip: float) -> np.ndarray:
    """v_odom = v_wheel * (1 - clip(c1*a_lat, 0, clip))."""
    frac = np.clip(c1 * np.asarray(a_lat, float), 0.0, clip)
    return np.asarray(v_wheel, float) * (1.0 - frac)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    out = analysis_dir(session)
    sx = cfg.get("cross_axis_validation", {})
    if not bool(sx.get("enabled", False)) or not bool(sx.get("fit_turn_slip", False)):
        return _skip(out, "cross_axis_validation.fit_turn_slip not enabled")
    if not (session / STAGE).exists():
        return _skip(out, f"{STAGE} not present in session")

    ts = cfg["analysis"].get("turn_slip", {})
    gates = cfg["analysis"]["gates"]
    lin, quad = _wheel_gain(session, out)

    tables = stage_tables(session, STAGE)
    windows = accepted_capture_windows(tables["events"], PHASE)
    if windows.empty:
        return _skip(out, "no accepted cross-axis arc windows")
    summary = summarize_windows(windows, tables, cfg)
    if summary.empty:
        return _skip(out, "no summarisable cross-axis windows")

    # Window-averaged truth and causal regressors.
    summary = summary[
        (summary.lidar_valid_fraction >= float(ts.get("min_valid_lidar_fraction", 0.6)))
        & np.isfinite(summary.vx_lidar_mps) & np.isfinite(summary.erpm_measured)
    ].copy()
    summary["v_ground"] = summary.vx_lidar_mps
    summary["v_wheel"] = lin * summary.erpm_measured + quad * summary.erpm_measured * np.abs(summary.erpm_measured)
    summary["a_lat"] = summary.v_wheel * np.abs(summary.imu_gz_rad_s)
    floor = float(ts.get("min_speed_mps", 0.5))
    summary = summary[(summary.v_ground > floor) & (summary.v_wheel > floor)].copy()
    summary["slip_fraction"] = 1.0 - summary.v_ground / summary.v_wheel
    summary["condition_cell"] = (summary.steering_angle_rad.abs().round(4).astype(str) + "|" +
                                 summary.speed_command_mps.round(3).astype(str))
    summary.to_parquet(out / "turn_slip_samples.parquet", index=False)

    # Coverage over the configured arc grid (both magnitudes count once per cell).
    grid = [(float(a), float(v)) for a in sx["steering_angle_rad"] for v in sx["speed_commands_mps"]]
    cov = expected_grid_coverage(summary, fields=["steering_angle_rad", "speed_command_mps"],
                                 expected_grid=grid, expected_repetitions=int(sx.get("repetitions", 1)),
                                 tolerances={"steering_angle_rad": 1e-6, "speed_command_mps": 1e-6},
                                 unique_by=["trial_id"])
    cov.to_parquet(out / "turn_slip_coverage.parquet", index=False)

    if len(summary) < 6:
        return _skip(out, f"too few usable arc windows ({len(summary)})")
    train_cells, holdout_cells = _condition_split(summary.condition_cell.tolist())
    train = summary[summary.condition_cell.isin(train_cells)]
    hold = summary[summary.condition_cell.isin(holdout_cells)]
    if len(train) < 3 or len(hold) < 3:
        return _skip(out, "insufficient train/holdout arc conditions after split")

    c1, r2 = fit_turn_slip_coefficient(train.a_lat.to_numpy(float), train.slip_fraction.to_numpy(float))
    clip = float(ts.get("correction_clip_fraction", 0.25))

    def correct(df: pd.DataFrame) -> np.ndarray:
        return apply_turn_slip(df.v_wheel.to_numpy(float), df.a_lat.to_numpy(float), c1, clip)

    def metrics(pred_v, truth):
        r = pred_v - truth
        r = r[np.isfinite(r)]
        return {"n": int(len(r)), "rmse_mps": float(np.sqrt(np.mean(r**2))) if len(r) else math.inf,
                "bias_mps": float(np.mean(r)) if len(r) else math.nan}
    base_hold = metrics(hold.v_wheel.to_numpy(float), hold.v_ground.to_numpy(float))
    corr_hold = metrics(correct(hold), hold.v_ground.to_numpy(float))

    accepted = bool(
        bool(cov.coverage_ok.all())
        and c1 >= float(ts.get("min_coefficient_per_mps2", 0.002))
        and (math.isfinite(r2) and r2 >= float(ts.get("min_train_r2", 0.4)))
        and corr_hold["rmse_mps"] <= float(gates.get("max_turn_slip_holdout_rmse_mps", 0.15))
        and abs(corr_hold["bias_mps"]) <= float(gates.get("max_turn_slip_holdout_bias_mps", 0.07))
        and corr_hold["rmse_mps"] < base_hold["rmse_mps"]
    )
    report = {
        "status": "fitted",
        "model": "v_odom = v_wheel * (1 - clip(c1 * v_wheel*|yaw_rate|, 0, clip_fraction))",
        "lateral_accel_regressor": "v_wheel * |imu_gz|",
        "turn_slip_coeff_per_mps2": c1,
        "correction_clip_fraction": clip,
        "train_r2": r2,
        "train_windows": int(len(train)),
        "holdout_windows": int(len(hold)),
        "holdout_uncorrected": base_hold,
        "holdout_corrected": corr_hold,
        "holdout_rmse_reduction_mps": base_hold["rmse_mps"] - corr_hold["rmse_mps"],
        "straight_line_preserved": True,
        "coverage_ok": bool(cov.coverage_ok.all()),
        "gates": {
            "max_turn_slip_holdout_rmse_mps": float(gates.get("max_turn_slip_holdout_rmse_mps", 0.15)),
            "max_turn_slip_holdout_bias_mps": float(gates.get("max_turn_slip_holdout_bias_mps", 0.07)),
            "min_coefficient_per_mps2": float(ts.get("min_coefficient_per_mps2", 0.002)),
            "min_train_r2": float(ts.get("min_train_r2", 0.4)),
        },
        "accepted_for_candidate": accepted,
        "wheel_gain_used": {"odom_erpm_to_speed_linear": lin, "odom_erpm_to_speed_quadratic": quad},
    }
    dump_yaml(out / "turn_slip_report.yaml", report)

    # Append the correction to the deployable odom patch (does not touch the
    # straight-line gain). The production port applies it as in `model` above.
    patch_path = out / "selected_odometry_candidate_patch.yaml"
    if patch_path.is_file():
        patch = load_yaml(patch_path)
        od = patch.setdefault("vesc_to_odom_node", {}).setdefault("ros__parameters", {})
        od["odom_turn_slip_coeff_per_mps2"] = c1 if accepted else 0.0
        od["odom_turn_slip_clip_fraction"] = clip
        od["odom_turn_slip_accepted"] = accepted
        dump_yaml(patch_path, patch)

    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
