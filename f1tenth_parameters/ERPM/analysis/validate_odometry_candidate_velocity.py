#!/usr/bin/env python3
"""Validate the selected shadow command/odometry model on fresh speed holds."""
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
    expected_numeric_coverage,
    load_yaml,
    stage_tables,
    straight_filter,
    summarize_windows,
)


def _metrics(measured: np.ndarray, predicted: np.ndarray) -> dict[str, float | int]:
    valid = np.isfinite(measured) & np.isfinite(predicted)
    if not np.any(valid):
        return {"n": 0, "rmse_mps": math.inf, "bias_mps": math.nan, "p95_abs_error_mps": math.inf}
    residual = predicted[valid] - measured[valid]
    return {
        "n": int(valid.sum()),
        "rmse_mps": float(np.sqrt(np.mean(residual * residual))),
        "bias_mps": float(np.mean(residual)),
        "p95_abs_error_mps": float(np.quantile(np.abs(residual), 0.95)),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    tables = stage_tables(session, "11_candidate_velocity_verification")
    captures = accepted_capture_windows(tables["events"], "candidate_velocity_verification")
    trials = straight_filter(summarize_windows(captures, tables, cfg), cfg)

    candidate = tables.get("candidate_odom", pd.DataFrame())
    candidate_values: dict[str, float] = {}
    for _, capture in captures.iterrows():
        subset = candidate[
            (candidate.bag_ns >= int(capture.start_ns))
            & (candidate.bag_ns <= int(capture.end_ns))
        ] if not candidate.empty else candidate
        candidate_values[str(capture.get("trial_id", ""))] = (
            float(np.nanmedian(subset.vx.to_numpy(float)))
            if not subset.empty and "vx" in subset else math.nan
        )
    if not trials.empty and "trial_id" in trials:
        trials = trials.copy()
        trials["candidate_odom_vx_mps"] = trials.trial_id.astype(str).map(candidate_values)

    output = analysis_dir(session)
    trials.to_parquet(output / "odometry_candidate_velocity_trials.parquet", index=False)
    spec = cfg["candidate_verification"]
    coverage = expected_numeric_coverage(
        trials,
        "speed_command_mps",
        spec["velocity_holdout_commands_mps"],
        int(spec["velocity_repetitions"]),
    )
    coverage.to_parquet(output / "odometry_candidate_velocity_coverage.parquet", index=False)
    lidar = trials.get("vx_lidar_mps", pd.Series(dtype=float)).to_numpy(float)
    command = trials.get("speed_command_mps", pd.Series(dtype=float)).to_numpy(float)
    odom = trials.get("candidate_odom_vx_mps", pd.Series(dtype=float)).to_numpy(float)
    command_metrics = _metrics(lidar, command)
    odom_metrics = _metrics(lidar, odom)
    gates = cfg["analysis"]["gates"]
    coverage_ok = bool(len(coverage)) and bool(coverage.coverage_ok.all())
    accepted = bool(
        coverage_ok
        and command_metrics["rmse_mps"] <= float(gates["max_speed_map_holdout_rmse_mps"])
        and abs(float(command_metrics["bias_mps"])) <= float(gates["max_speed_map_holdout_bias_mps"])
        and odom_metrics["rmse_mps"] <= float(gates["max_odom_holdout_rmse_mps"])
        and abs(float(odom_metrics["bias_mps"])) <= float(gates["max_odom_holdout_bias_mps"])
        and odom_metrics["p95_abs_error_mps"] <= float(gates["max_odom_holdout_p95_abs_error_mps"])
    )
    report = {
        "accepted_for_validation": accepted,
        "coverage_ok": coverage_ok,
        "commanded_speed_vs_lidar": command_metrics,
        "candidate_odometry_vs_lidar": odom_metrics,
        "fit_data_reused": False,
        "measurement_unit": "accepted robust LiDAR motion window summarized per fresh hold-out pass",
    }
    dump_yaml(output / "odometry_candidate_velocity_validation_report.yaml", report)
    print(json.dumps(report, indent=2))
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(main())
