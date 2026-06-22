#!/usr/bin/env python3
"""Report ICP accuracy diagnostics from the observability bag before map fitting."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from trials import accepted_trial_ids


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    stage = session / "03_sensor_observability" / "derived"
    events = pd.read_parquet(stage / "events.parquet")
    lidar = pd.read_parquet(stage / "lidar_velocity.parquet")
    accepted = accepted_trial_ids(events)
    starts = events[(events.get("event") == "phase_start") &
                    (events.get("phase") == "observability_stationary") &
                    (events.get("trial_id").astype(str).isin(accepted))]
    stationary = pd.DataFrame()
    if len(starts):
        start = starts.iloc[-1]
        ends = events[(events.get("event") == "phase_end") &
                      (events.get("trial_id").astype(str) == str(start.trial_id)) &
                      (events.bag_ns > start.bag_ns)]
        if len(ends):
            stationary = lidar[(lidar.bag_ns >= start.bag_ns) & (lidar.bag_ns <= ends.iloc[0].bag_ns)].copy()
    valid = stationary[stationary.valid] if len(stationary) and "valid" in stationary else pd.DataFrame()
    report = {
        "stationary_pairs": int(len(stationary)),
        "stationary_valid_pairs": int(len(valid)),
        "stationary_valid_fraction": float(len(valid) / len(stationary)) if len(stationary) else None,
    }
    if len(valid):
        speed = np.hypot(valid.vx.to_numpy(float), valid.vy.to_numpy(float))
        report.update({
            "stationary_speed_median_mps": float(np.median(speed)),
            "stationary_speed_p95_mps": float(np.quantile(speed, 0.95)),
            "point_to_line_rmse_median_m": float(valid.icp_rmse_m.median()),
            "point_to_line_rmse_p95_m": float(valid.icp_rmse_m.quantile(0.95)),
            "condition_median": float(valid.hessian_condition_number.median()),
            "condition_p95": float(valid.hessian_condition_number.quantile(0.95)),
            "yaw_seed_residual_p95_rad": float(np.abs(valid.yaw_seed_residual_rad).quantile(0.95)),
            "solver_converged_fraction": float(valid.solver_converged.mean()),
        })
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    (output / "icp_observability_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
