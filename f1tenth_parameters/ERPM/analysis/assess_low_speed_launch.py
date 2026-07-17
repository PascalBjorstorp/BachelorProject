#!/usr/bin/env python3
"""Analyse the low-speed launch/dead-band capture immediately after collection.

It does not fit a VEL_TO_ERPM gain before the full independent static-map grid
exists.  It does prove that every planned launch condition produced usable
LiDAR ground-motion evidence and records the first repeatable ground-speed
threshold that the later training fit may turn into a slow-start setting.
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from common import analysis_dir, dump_yaml, load_yaml
from fit_speed_map import _coverage, _summary


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = load_yaml(session / "calibration_config_snapshot.yaml")
    out = analysis_dir(session)
    summary = _summary(session, "02_low_speed_launch", "low_speed_launch", cfg)
    summary.to_parquet(out / "low_speed_launch_trials.parquet", index=False)
    spec = cfg["low_speed_launch"]
    coverage = _coverage(
        summary, list(map(float, spec["nominal_speeds_mps"])), int(spec["repetitions"]),
    )
    coverage.to_parquet(out / "low_speed_launch_coverage.parquet", index=False)
    stable_floor = float(spec["minimum_lidar_speed_mps"])
    stable = summary[summary.vx_lidar_mps >= stable_floor].copy() if not summary.empty else summary
    failures: list[str] = []
    if summary.empty or not bool(coverage.coverage_ok.all()):
        failures.append("low-speed launch coverage is incomplete")
    if stable.empty:
        failures.append(
            f"no launch capture reached the configured repeatable ground-speed floor {stable_floor:.3f} m/s"
        )
    if stable.empty:
        threshold = {"nominal_speed_mps": math.nan, "lidar_ground_speed_mps": math.nan,
                     "commanded_erpm": math.nan, "measured_erpm": math.nan}
    else:
        first = stable.sort_values(["vx_lidar_mps", "erpm_measured"]).iloc[0]
        threshold = {
            "nominal_speed_mps": float(first.nominal_speed_mps),
            "lidar_ground_speed_mps": float(first.vx_lidar_mps),
            "commanded_erpm": float(first.selected_speed_erpm)
            if math.isfinite(float(first.selected_speed_erpm)) else float(first.raw_erpm_target),
            "measured_erpm": float(first.erpm_measured),
        }
    report = {
        "accepted_for_downstream": not failures,
        "coverage": coverage.to_dict(orient="records"),
        "configured_repeatable_ground_speed_floor_mps": stable_floor,
        "first_repeatable_launch": threshold,
        "usable_trials": int(len(summary)),
        "stable_trials": int(len(stable)),
        "note": (
            "No speed-map parameter is updated here: the following full static training grid fits the "
            "zero-intercept map and consumes this threshold as a separately identified slow-start input."
        ),
        "failures": failures,
    }
    dump_yaml(out / "low_speed_launch_report.yaml", report)
    print(report)
    if failures:
        raise SystemExit("low-speed launch rejected: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
