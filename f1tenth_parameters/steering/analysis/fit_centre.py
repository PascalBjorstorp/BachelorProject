#!/usr/bin/env python3
"""Offline re-fit of raw-servo straight-ahead centre from accepted MCAP trials."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from imu_bias import estimate_imu_bias
from trials import accepted_trial_ids


def intervals(events: pd.DataFrame, phase: str) -> list[tuple[dict, dict]]:
    accepted = accepted_trial_ids(events)
    starts = events[(events.get("event") == "phase_start") & (events.get("phase") == phase)]
    ends = events[(events.get("event") == "phase_end") & (events.get("phase") == phase)]
    output = []
    for _, start in starts.iterrows():
        trial_id = str(start.get("trial_id"))
        if trial_id not in accepted:
            continue
        matches = ends[(ends.get("trial_id").astype(str) == trial_id) & (ends["bag_ns"] > start["bag_ns"])]
        if len(matches):
            output.append((start.to_dict(), matches.iloc[0].to_dict()))
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--trim-s", type=float, default=1.0)
    args = parser.parse_args()
    session = args.session.resolve()
    stage = session / "01_zero_curvature_centre" / "derived"
    events = pd.read_parquet(stage / "events.parquet")
    imu = pd.read_parquet(stage / "imu.parquet")
    echo = pd.read_parquet(stage / "servo_echo.parquet")
    # Remove the stationary gyro-z offset (drift-tracked over the session);
    # otherwise the centre is identified at the servo command where the *biased*
    # yaw rate is zero, not the true zero.
    bias = estimate_imu_bias(session)
    rows = []
    for start, end in intervals(events, "centre_trim_capture"):
        a, b = int(start["bag_ns"] + args.trim_s * 1e9), int(end["bag_ns"] - args.trim_s * 1e9)
        im = imu[(imu.bag_ns >= a) & (imu.bag_ns <= b)]
        ec = echo[(echo.bag_ns >= a) & (echo.bag_ns <= b)]
        if len(im) < 10 or len(ec) < 3:
            continue
        gz_bias = bias.gz_at(0.5 * (a + b))
        rows.append({
            "trial_id": start.get("trial_id"),
            "raw_servo_target": float(start.get("raw_servo_target")),
            "raw_servo_echo": float(ec.value.mean()),
            "yaw_rate_rad_s": float(im.gz.mean()) - gz_bias,
            "yaw_rate_std_rad_s": float(im.gz.std()),
            "gyro_z_bias_rad_s": float(gz_bias),
            "sample_count": int(len(im)),
        })
    table = pd.DataFrame(rows)
    if len(table) < 6:
        raise SystemExit("fewer than six accepted centre-trim capture windows after export")
    # Fit against actual emitted command, not requested command.
    coef = np.polyfit(table.raw_servo_echo, table.yaw_rate_rad_s, 1)
    if abs(coef[0]) < 1e-8:
        raise SystemExit("centre-trim data do not excite measurable yaw response")
    result = {
        "centre_servo_raw": float(-coef[1] / coef[0]),
        "yaw_vs_servo_slope_rad_s_per_servo": float(coef[0]),
        "accepted_training_points": int(len(table)),
        "trim_s": float(args.trim_s),
        "gyro_z_bias": bias.to_dict(),
    }
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    table.to_parquet(output / "centre_trim_points.parquet", index=False)
    (output / "centre_trim_offline.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
