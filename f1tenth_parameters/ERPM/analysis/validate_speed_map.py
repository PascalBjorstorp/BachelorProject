#!/usr/bin/env python3
"""Validate the already-applied training ERPM map on independent hold-out runs."""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from common import analysis_dir, dump_yaml, load_yaml
from fit_speed_map import _coverage, _predict_speed, _summary


def _metrics(actual: np.ndarray, predicted: np.ndarray) -> dict:
    residual = np.asarray(actual, dtype=float) - np.asarray(predicted, dtype=float)
    residual = residual[np.isfinite(residual)]
    return {
        "rmse_mps": float(np.sqrt(np.mean(residual ** 2))) if len(residual) else math.inf,
        "bias_mps": float(np.mean(residual)) if len(residual) else math.nan,
        "n": int(len(residual)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    out = analysis_dir(session)
    training = load_yaml(out / "erpm_speed_map_training_report.yaml")
    hold = _summary(session, "04_raw_erpm_map_holdout", "raw_erpm_holdout", cfg)
    hold.to_parquet(out / "erpm_map_holdout_trials.parquet", index=False)
    spec = cfg["raw_erpm_map_holdout"]
    cov = _coverage(hold, list(map(float, spec["nominal_speeds_mps"])), int(spec["repetitions"]))
    cov.to_parquet(out / "erpm_map_holdout_coverage.parquet", index=False)
    failures: list[str] = []
    if hold.empty or not bool(cov.coverage_ok.all()):
        failures.append("ERPM hold-out coverage incomplete")
    if not hold.empty:
        hold = hold.copy()
        hold["commanded_erpm"] = hold["selected_speed_erpm"].where(
            np.isfinite(hold["selected_speed_erpm"]), hold["raw_erpm_target"]
        )
    gain = float(training.get("candidate_speed_to_erpm_gain", math.nan))
    scale = float(training.get("candidate_odom_speed_scale", math.nan))
    command_pred = hold.commanded_erpm.to_numpy(float) / max(gain, 1e-12) if not hold.empty else np.empty(0)
    measured_pred = hold.erpm_measured.to_numpy(float) / max(gain, 1e-12) * scale if not hold.empty else np.empty(0)
    actual = hold.vx_lidar_mps.to_numpy(float) if not hold.empty else np.empty(0)
    command_metrics = _metrics(actual, command_pred)
    measured_metrics = _metrics(actual, measured_pred)
    gates = cfg["analysis"]["gates"]
    if command_metrics["n"] == 0 or command_metrics["rmse_mps"] > float(gates["max_speed_map_holdout_rmse_mps"]):
        failures.append("applied command speed-map hold-out RMSE exceeds gate")
    if command_metrics["n"] == 0 or abs(command_metrics["bias_mps"]) > float(gates["max_speed_map_holdout_bias_mps"]):
        failures.append("applied command speed-map hold-out bias exceeds gate")
    if measured_metrics["n"] == 0 or measured_metrics["rmse_mps"] > float(gates.get("max_odom_holdout_rmse_mps", gates["max_speed_map_holdout_rmse_mps"])):
        failures.append("applied measured-ERPM odometry scale hold-out RMSE exceeds gate")

    # The training stage intentionally applies only the production scalar map
    # before C.  The independent hold-out is where we decide whether that code
    # shape is adequate.  If a quadratic-through-origin candidate materially
    # improves C, silently retaining the scalar value would be exactly the
    # false validation this campaign is designed to prevent.
    static_policy = cfg.get("analysis", {}).get("static_map", {})
    improvement_needed = float(static_policy.get("minimum_holdout_improvement_fraction", 0.10))
    quadratic_floor = float(static_policy.get("minimum_abs_quadratic_erpm_per_mps2", 0.0))
    command_models = training.get("training_command_model", {})
    measured_models = training.get("training_measured_model", {})
    nonlinear_comparison: dict[str, dict] = {}
    nonlinear_required = False
    for name, input_values, models in (
        ("command", hold.commanded_erpm.to_numpy(float) if not hold.empty else np.empty(0), command_models),
        ("measured_erpm_odometry", hold.erpm_measured.to_numpy(float) if not hold.empty else np.empty(0), measured_models),
    ):
        linear = models.get("linear", {}) if isinstance(models, dict) else {}
        quadratic = models.get("quadratic", {}) if isinstance(models, dict) else {}
        try:
            linear_prediction = _predict_speed(input_values, linear)
            quadratic_prediction = _predict_speed(input_values, quadratic)
            linear_metrics = _metrics(actual, linear_prediction)
            quadratic_metrics = _metrics(actual, quadratic_prediction)
            quadratic_value = abs(float(quadratic.get("quadratic_erpm_per_mps2", math.nan)))
        except (KeyError, TypeError, ValueError):
            linear_metrics = {"rmse_mps": math.inf, "bias_mps": math.nan, "n": 0}
            quadratic_metrics = {"rmse_mps": math.inf, "bias_mps": math.nan, "n": 0}
            quadratic_value = math.nan
        material = bool(
            math.isfinite(quadratic_value)
            and quadratic_value >= quadratic_floor
            and math.isfinite(float(linear_metrics["rmse_mps"]))
            and math.isfinite(float(quadratic_metrics["rmse_mps"]))
            and float(quadratic_metrics["rmse_mps"]) <= float(linear_metrics["rmse_mps"]) * (1.0 - improvement_needed)
        )
        nonlinear_required = nonlinear_required or material
        nonlinear_comparison[name] = {
            "frozen_training_linear": linear_metrics,
            "frozen_training_quadratic": quadratic_metrics,
            "quadratic_abs_erpm_per_mps2": quadratic_value,
            "minimum_material_improvement_fraction": improvement_needed,
            "quadratic_materially_better_on_holdout": material,
        }
    if nonlinear_required:
        failures.append(
            "independent hold-out materially prefers a quadratic static map; "
            "the installed scalar VEL_TO_ERPM/odometry path must be upgraded before promotion"
        )
    report = {
        "status": "pass" if not failures else "fail",
        "accepted_for_validation": not failures,
        "candidate_from_training": {
            "speed_to_erpm_gain": gain,
            "odom_speed_scale": scale,
            "gain_bootstrap": training.get("candidate_speed_to_erpm_gain_bootstrap", {}),
        },
        "holdout_command_ground_speed": command_metrics,
        "holdout_measured_erpm_ground_speed": measured_metrics,
        "nonlinear_model_comparison": nonlinear_comparison,
        "requires_full_stack_upgrade_for_selected_static_map": nonlinear_required,
        "coverage": cov.to_dict(orient="records"),
        "failures": failures,
        "note": "This report evaluates frozen training models on new data; it does not refit them. A nonlinear win blocks scalar promotion even though race control itself uses ACCEL_TO_CURRENT.",
    }
    dump_yaml(out / "erpm_speed_map_validation_report.yaml", report)
    print(report)
    if failures:
        raise SystemExit("ERPM applied-map validation failed: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
