#!/usr/bin/env python3
"""Verify LiDAR motion observability before any LiDAR-derived calibration fit.

The preflight deliberately judges the measurement mode used by the fitters:
robust multi-registration motion windows during a real straight drive.  A
stationary scan-registration diagnostic is retained, but is not allowed to
reject an otherwise good moving-scene measurement merely because point-to-line
ICP is poorly conditioned at exactly zero translation.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import pandas as pd

from lidar_windows import read_motion
from trials import accepted_trial_ids


def _phase_rows(events: pd.DataFrame, frame: pd.DataFrame, phase: str) -> pd.DataFrame:
    """Return rows inside accepted capture windows for one phase."""
    accepted = accepted_trial_ids(events)
    starts = events[(events.get("event") == "phase_start") & (events.get("phase") == phase)]
    ends = events[(events.get("event") == "phase_end") & (events.get("phase") == phase)]
    parts: list[pd.DataFrame] = []
    for _, start in starts.iterrows():
        trial_id = str(start.get("trial_id", ""))
        if trial_id not in accepted:
            continue
        matches = ends[(ends.get("trial_id").astype(str) == trial_id) & (ends.bag_ns > start.bag_ns)]
        if matches.empty:
            continue
        end = matches.sort_values("bag_ns").iloc[0]
        parts.append(frame[(frame.bag_ns >= int(start.bag_ns)) & (frame.bag_ns <= int(end.bag_ns))].copy())
    return pd.concat(parts, ignore_index=True) if parts else frame.iloc[0:0].copy()


def _summary(frame: pd.DataFrame, *, prefix: str) -> dict[str, object]:
    valid = frame[frame.valid.astype(bool)].copy() if len(frame) and "valid" in frame else frame.iloc[0:0]
    report: dict[str, object] = {
        f"{prefix}_rows": int(len(frame)),
        f"{prefix}_valid_rows": int(len(valid)),
        f"{prefix}_valid_fraction": float(len(valid) / len(frame)) if len(frame) else None,
    }
    if valid.empty:
        return report
    speed = np.hypot(valid.vx.to_numpy(float), valid.vy.to_numpy(float))
    report.update({
        f"{prefix}_speed_median_mps": float(np.median(speed)),
        f"{prefix}_speed_p95_mps": float(np.quantile(speed, 0.95)),
        f"{prefix}_point_to_line_rmse_median_m": float(valid.icp_rmse_m.median()),
        f"{prefix}_point_to_line_rmse_p95_m": float(valid.icp_rmse_m.quantile(0.95)),
        f"{prefix}_condition_median": float(valid.hessian_condition_number.median()),
        f"{prefix}_condition_p95": float(valid.hessian_condition_number.quantile(0.95)),
        f"{prefix}_yaw_seed_residual_p95_rad": float(np.abs(valid.yaw_seed_residual_rad).quantile(0.95)),
    })
    return report


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--stage-directory", default="03_sensor_observability")
    args = parser.parse_args()
    session = args.session.resolve()
    stage = session / args.stage_directory / "derived"
    events = pd.read_parquet(stage / "events.parquet")
    pair_rows = pd.read_parquet(stage / "lidar_velocity.parquet")
    window_rows = read_motion(stage)

    stationary_pairs = _phase_rows(events, pair_rows, "observability_stationary")
    stationary_windows = _phase_rows(events, window_rows, "observability_stationary")
    moving_pairs = _phase_rows(events, pair_rows, "observability_straight_capture")
    moving_windows = _phase_rows(events, window_rows, "observability_straight_capture")

    report: dict[str, object] = {
        "measurement_used_for_gate": "robust multi-registration LiDAR motion windows during accepted straight passes",
        "stationary_measurement_role": "diagnostic only; point-to-line ICP at exactly zero translation can be geometrically degenerate",
        **_summary(stationary_pairs, prefix="stationary_pair"),
        **_summary(stationary_windows, prefix="stationary_window"),
        **_summary(moving_pairs, prefix="moving_pair"),
        **_summary(moving_windows, prefix="moving_window"),
    }
    # Backward-compatible aliases make old report readers show the stationary
    # diagnostics without accidentally treating them as the approval gate.
    report["stationary_pairs"] = report["stationary_pair_rows"]
    report["stationary_valid_pairs"] = report["stationary_pair_valid_rows"]
    report["stationary_valid_fraction"] = report["stationary_pair_valid_fraction"]
    report["stationary_speed_p95_mps"] = report.get("stationary_pair_speed_p95_mps")
    report["point_to_line_rmse_p95_m"] = report.get("stationary_pair_point_to_line_rmse_p95_m")
    report["moving_valid_windows"] = report["moving_window_valid_rows"]
    report["moving_window_valid_fraction"] = report["moving_window_valid_fraction"]
    report["moving_window_speed_median_mps"] = report.get("moving_window_speed_median_mps", math.nan)

    output = session / "analysis"
    output.mkdir(exist_ok=True)
    (output / "icp_observability_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
