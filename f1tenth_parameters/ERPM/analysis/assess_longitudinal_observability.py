#!/usr/bin/env python3
"""Gate the session's longitudinal LiDAR/IMU observability before fitting.

This is deliberately a quality/precondition report, not an odometry fit.  It
uses the same robust LiDAR-window summaries that later speed, coast-down and
current analyses consume, so a passing stationary command audit cannot hide an
unusable moving measurement in the actual room.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from common import (
    accepted_capture_windows,
    analysis_dir,
    dump_yaml,
    load_yaml,
    motion_windows,
    stage_tables,
    straight_filter,
    summarize_windows,
)


def _coverage(frame, speeds: list[float], repetitions: int):
    import pandas as pd

    actual = frame.get("speed_command_mps", pd.Series(dtype=float)).to_numpy(float)
    rows = []
    for speed in map(float, speeds):
        count = int((np.isfinite(actual) & np.isclose(actual, speed, rtol=0.0, atol=1.0e-6)).sum())
        rows.append({
            "speed_command_mps": speed,
            "accepted_usable_trials": count,
            "expected_trials": int(repetitions),
            "coverage_ok": count >= int(repetitions),
        })
    result = pd.DataFrame(rows)
    result["all_expected_conditions_present"] = bool(result.coverage_ok.all()) if len(result) else False
    return result


def _stationary_diagnostic(tables: dict, cfg: dict) -> dict:
    import pandas as pd

    windows = accepted_capture_windows(tables["events"], "stationary_observability")
    summary = summarize_windows(windows, tables, cfg)
    lidar = motion_windows(tables)
    if summary.empty:
        return {"captured_trials": 0, "note": "No stationary summary was available; it is diagnostic only."}
    values = summary.vx_lidar_mps.to_numpy(float) if "vx_lidar_mps" in summary else np.empty(0)
    values = values[np.isfinite(values)]
    parts = [
        lidar[(lidar.bag_ns >= int(window.start_ns)) & (lidar.bag_ns <= int(window.end_ns))]
        for _, window in windows.iterrows()
    ]
    raw_all = pd.concat(parts, ignore_index=True) if parts else lidar.iloc[0:0]
    raw = raw_all[raw_all.valid.astype(bool)] if not raw_all.empty and "valid" in raw_all else raw_all.iloc[0:0]
    raw_speed = np.hypot(raw.vx.to_numpy(float), raw.vy.to_numpy(float)) if len(raw) else np.empty(0)
    return {
        "captured_trials": int(len(summary)),
        "median_window_vx_mps": float(np.median(values)) if len(values) else math.nan,
        "p95_abs_window_vx_mps": float(np.quantile(np.abs(values), 0.95)) if len(values) else math.nan,
        "diagnostic_valid_registration_speed_p95_mps": float(np.quantile(raw_speed, 0.95)) if len(raw_speed) else math.nan,
        "note": "Stationary ICP/registration noise is recorded but never used as the moving-observability approval gate.",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    tables = stage_tables(session, "01_longitudinal_observability")
    moving_windows = accepted_capture_windows(tables["events"], "straight_observability")
    moving_all = summarize_windows(moving_windows, tables, cfg)
    moving = straight_filter(moving_all, cfg)
    out = analysis_dir(session)
    moving_all.to_parquet(out / "longitudinal_observability_all_trials.parquet", index=False)
    moving.to_parquet(out / "longitudinal_observability_trials.parquet", index=False)
    spec = cfg["observability"]
    coverage = _coverage(
        moving, list(map(float, spec["straight_probe_speeds_mps"])), int(spec["straight_probe_repetitions"]),
    )
    coverage.to_parquet(out / "longitudinal_observability_coverage.parquet", index=False)
    failures: list[str] = []
    if moving.empty:
        failures.append("no straight moving LiDAR-window trial passed the shared longitudinal quality filter")
    if not bool(len(coverage)) or not bool(coverage.coverage_ok.all()):
        failures.append("longitudinal moving-probe coverage is incomplete")
    quality = {
        "usable_trials": int(len(moving)),
        "all_trials": int(len(moving_all)),
        "median_lidar_speed_mps": float(np.nanmedian(moving.vx_lidar_mps.to_numpy(float))) if len(moving) else math.nan,
        "minimum_lidar_valid_fraction": float(np.nanmin(moving.lidar_valid_fraction.to_numpy(float))) if len(moving) else math.nan,
        "maximum_abs_imu_yaw_rate_rad_s": float(np.nanmax(np.abs(moving.imu_gz_rad_s.to_numpy(float)))) if len(moving) else math.nan,
        "maximum_abs_imu_lateral_accel_mps2": float(np.nanmax(np.abs(moving.imu_ay_mps2.to_numpy(float)))) if len(moving) else math.nan,
    }
    report = {
        "measurement_used_for_gate": "one robust multi-registration LiDAR-window summary per accepted straight pass",
        "accepted_for_downstream": not failures,
        "coverage": coverage.to_dict(orient="records"),
        "moving_quality": quality,
        "stationary_diagnostic": _stationary_diagnostic(tables, cfg),
        "failures": failures,
        "note": "This stage is a fresh moving-sensor quality gate before any ERPM fit. LiDAR remains the analysis reference; odometry is only a runtime scheduler/safety signal.",
    }
    dump_yaml(out / "longitudinal_observability_report.yaml", report)
    print(report)
    if failures:
        raise SystemExit("longitudinal observability rejected: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
