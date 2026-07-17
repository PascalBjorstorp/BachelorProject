#!/usr/bin/env python3
"""Fit a deployable steering map from training captures only.

The hold-out is deliberately absent here.  This lets the runner apply the
training candidate, rebuild, and then use the next stage as an independent
validation of the applied map.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from fit_static_map import (
    TRAINING_APPROACHES,
    _condition_summary,
    _coverage_report,
    add_nominal_condition_keys,
    map_interpolate,
    rear_axle_geometry,
    segment_rows,
)
from imu_bias import correction_enabled, estimate_imu_bias


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = yaml.safe_load((args.config or session / "calibration_config_snapshot.yaml").read_text(encoding="utf-8")) or {}
    centre = json.loads((session / "analysis" / "centre_trim_offline.json").read_text(encoding="utf-8"))
    criteria = cfg["analysis"]["map"]
    static_cfg = cfg["static_map"]
    bias = estimate_imu_bias(session, apply_correction=correction_enabled(cfg))
    rear_x, rear_y = rear_axle_geometry(cfg)
    train = segment_rows(session / "04_static_map_training", float(cfg["hardware"]["wheelbase_m"]),
                         float(criteria["trim_s"]), criteria, bias,
                         rear_axle_x_m=rear_x, rear_axle_y_m=rear_y)
    analysis = session / "analysis"
    analysis.mkdir(exist_ok=True)
    train.to_parquet(analysis / "static_map_training_segments.parquet", index=False)
    accepted = add_nominal_condition_keys(train[train.accepted].copy()) if len(train) else train
    coverage, coverage_failures = _coverage_report(
        accepted, fractions=list(static_cfg["training_fractions"]),
        approaches=TRAINING_APPROACHES,
        required_count=int(static_cfg["training_sweep_repetitions"]), label="training",
    )
    coverage.to_parquet(analysis / "static_map_training_condition_coverage.parquet", index=False)
    failures = list(coverage_failures)
    if len(accepted) < int(criteria["min_training_points"]):
        failures.append(f"training points {len(accepted)} < {criteria['min_training_points']}")
    try:
        summary = _condition_summary(accepted)
        x, y = map_interpolate(summary, float(centre["centre_servo_raw"]))
    except (ValueError, KeyError) as exc:
        x, y, summary = np.array([]), np.array([]), pd.DataFrame()
        failures.append(str(exc))
    candidate = {
        "centre_servo_raw": float(centre["centre_servo_raw"]),
        "measurement_reference": (
            "LiDAR robust-window yaw rate and rear-axle ground speed; IMU yaw/acceleration are recorded as cross-checks only."
        ),
        "rear_axle_in_base_link": {"x_m": rear_x, "y_m": rear_y},
        "gyro_z_bias": bias.to_dict(),
        "raw_servo": x.tolist(),
        "delta_eq_rad": y.tolist(),
        "local_gain_rad_per_servo": np.gradient(y, x).tolist() if len(x) >= 2 else [],
        "training_points": int(len(accepted)),
        "training_nominal_conditions": int(len(summary)),
        "condition_coverage": {
            "training_required_per_condition": int(static_cfg["training_sweep_repetitions"]),
            "training_status": "pass" if not coverage_failures else "fail",
        },
        "validation": {
            "status": "not_run",
            "note": "Independent hold-out runs after this candidate is applied.",
        },
        "accepted_for_deployment": not failures,
        "accepted_for_update": not failures,
        "failures": failures,
    }
    (analysis / "candidate_static_steering_map.json").write_text(json.dumps(candidate, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(candidate, indent=2))
    if failures:
        raise SystemExit("static-map training gate failed: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
