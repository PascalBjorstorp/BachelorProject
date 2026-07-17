#!/usr/bin/env python3
"""Check the actual linear VESC steering patch on the static-map hold-out."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


def _inverse_steering_correction(corrected: np.ndarray, c2: float, c1: float, c0: float) -> np.ndarray:
    """Invert the production sign-preserving steering correction safely.

    Validation must evaluate the exact map that was rebuilt for C.  The common
    linear candidate uses identity coefficients, but retaining this inversion
    prevents a future supported polynomial patch from being accidentally
    validated as if it were linear.
    """
    values = np.asarray(corrected, dtype=float)
    output = np.full(values.shape, np.nan, dtype=float)
    for index, value in np.ndenumerate(values):
        if not math.isfinite(float(value)):
            continue
        magnitude = abs(float(value))
        if magnitude <= 1e-12:
            output[index] = 0.0
            continue
        if abs(c2) <= 1e-12:
            if abs(c1) > 1e-12:
                angle = (magnitude - c0) / c1
            else:
                angle = math.nan
        else:
            discriminant = c1 * c1 - 4.0 * c2 * (c0 - magnitude)
            if discriminant < 0.0:
                angle = math.nan
            else:
                roots = (
                    (-c1 + math.sqrt(discriminant)) / (2.0 * c2),
                    (-c1 - math.sqrt(discriminant)) / (2.0 * c2),
                )
                nonnegative = [root for root in roots if math.isfinite(root) and root >= -1e-10]
                angle = min(nonnegative, key=lambda root: abs(
                    c2 * root * root + c1 * root + c0 - magnitude
                )) if nonnegative else math.nan
        if math.isfinite(angle) and angle >= -1e-10:
            output[index] = math.copysign(max(0.0, angle), float(value))
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = yaml.safe_load((session / "calibration_config_snapshot.yaml").read_text(encoding="utf-8")) or {}
    analysis = session / "analysis"
    table_path = analysis / "static_map_holdout_segments.parquet"
    patch_path = analysis / "steering_map_vesc_patch.yaml"
    candidate_path = analysis / "candidate_static_steering_map.json"
    if not table_path.exists() or not patch_path.exists() or not candidate_path.exists():
        raise SystemExit("applied static-map validation requires hold-out segments, steering_map_vesc_patch.yaml and the training candidate")
    table = pd.read_parquet(table_path)
    table = table[table.accepted.astype(bool)].copy() if len(table) and "accepted" in table else table.iloc[0:0]
    patch = yaml.safe_load(patch_path.read_text(encoding="utf-8")) or {}
    params = patch.get("global", {})
    gain = float(params.get("steering_angle_to_servo_gain", math.nan))
    offset = float(params.get("steering_angle_to_servo_offset", math.nan))
    c2 = float(params.get("steering_correction_c2", 0.0))
    c1 = float(params.get("steering_correction_c1", 1.0))
    c0 = float(params.get("steering_correction_c0", 0.0))
    actual = table.delta_eq_rad.to_numpy(float) if len(table) else np.empty(0)
    raw = table.raw_servo_echo.to_numpy(float) if len(table) else np.empty(0)
    corrected = (raw - offset) / gain if len(raw) and math.isfinite(gain) and abs(gain) > 1e-12 else np.full(len(raw), np.nan)
    predicted = _inverse_steering_correction(corrected, c2, c1, c0)
    residual = predicted - actual
    finite = np.isfinite(residual)
    rmse = float(np.sqrt(np.mean(residual[finite] ** 2))) if finite.any() else math.inf
    bias = float(np.mean(residual[finite])) if finite.any() else math.nan
    gates = cfg["analysis"]["map"]
    candidate = json.loads(candidate_path.read_text(encoding="utf-8"))
    map_raw = np.asarray(candidate.get("raw_servo", []), dtype=float)
    map_delta = np.asarray(candidate.get("delta_eq_rad", []), dtype=float)
    nonlinear_prediction = np.interp(raw, map_raw, map_delta) if len(raw) and len(map_raw) >= 2 and len(map_raw) == len(map_delta) else np.full(len(raw), np.nan)
    nonlinear_residual = nonlinear_prediction - actual
    nonlinear_finite = np.isfinite(nonlinear_residual)
    nonlinear_rmse = float(np.sqrt(np.mean(nonlinear_residual[nonlinear_finite] ** 2))) if nonlinear_finite.any() else math.inf
    improvement = (rmse - nonlinear_rmse) / max(rmse, 1e-12) if math.isfinite(rmse) and math.isfinite(nonlinear_rmse) else -math.inf
    nonlinear_required = bool(
        nonlinear_finite.any()
        and improvement >= float(gates.get("minimum_linear_patch_holdout_improvement_fraction", 0.15))
    )
    expected_sign = float(gates.get("expected_servo_gain_sign", -1.0))
    failures: list[str] = []
    if not len(table):
        failures.append("no accepted static-map hold-out rows")
    if not math.isfinite(gain) or gain * expected_sign <= 0.0:
        failures.append(f"applied steering gain does not have expected sign {expected_sign:+.0f}")
    if len(table) < int(gates["min_holdout_points"]):
        failures.append(f"applied-map hold-out points {len(table)} < {gates['min_holdout_points']}")
    if not math.isfinite(rmse) or rmse > float(gates["max_holdout_rmse_rad"]):
        failures.append("actual applied linear patch hold-out RMSE exceeds gate")
    if not math.isfinite(bias) or abs(bias) > float(gates["max_abs_holdout_bias_rad"]):
        failures.append("actual applied linear patch hold-out bias exceeds gate")
    if nonlinear_required:
        failures.append(
            "hold-out materially prefers the frozen nonlinear/piecewise steering map; "
            "implement a supported nonlinear steering map before promotion"
        )
    report = {
        "status": "pass" if not failures else "fail",
        "accepted_for_validation": not failures,
        "applied_patch": {
            "steering_angle_to_servo_gain": gain,
            "steering_angle_to_servo_offset": offset,
            "steering_correction_c2": c2,
            "steering_correction_c1": c1,
            "steering_correction_c0": c0,
        },
        "expected_servo_gain_sign": expected_sign,
        "holdout_points": int(len(table)),
        "holdout_rmse_rad": rmse,
        "holdout_bias_rad": bias,
        "frozen_piecewise_training_map_holdout_rmse_rad": nonlinear_rmse,
        "piecewise_vs_applied_linear_holdout_improvement_fraction": improvement,
        "requires_nonlinear_steering_map": nonlinear_required,
        "failures": failures,
        "note": "Predicted steering angle is obtained by inverting the full rebuilt VESC raw-servo map (gain, offset and correction coefficients). The frozen training piecewise map is compared on C only to detect a production-model mismatch; it is never refitted on hold-out data.",
    }
    (analysis / "static_map_applied_validation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    if nonlinear_required:
        (analysis / "nonlinear_steering_map_request.json").write_text(json.dumps({
            "required": True,
            "reason": "Independent hold-out materially improves with the frozen piecewise steering map over the supported linear VESC map.",
            "linear_holdout_rmse_rad": rmse,
            "piecewise_holdout_rmse_rad": nonlinear_rmse,
            "improvement_fraction": improvement,
            "next_action": "Implement a bounded monotone nonlinear steering-angle-to-servo map in both AckermannToVesc and VescToOdom, then restart steering_static_training.",
        }, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if failures:
        raise SystemExit("applied static-map validation failed: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
