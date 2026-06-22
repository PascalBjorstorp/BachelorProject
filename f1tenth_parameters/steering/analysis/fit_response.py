#!/usr/bin/env python3
"""Estimate command-to-curvature response metrics from the final MCAP bag."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd

from trials import accepted_trial_ids


def first_crossing(t: np.ndarray, y: np.ndarray, threshold: float) -> float | None:
    idx = np.where(y >= threshold)[0]
    return float(t[idx[0]]) if len(idx) else None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    candidate = json.loads((session / "analysis" / "candidate_static_steering_map.json").read_text(encoding="utf-8"))
    x = np.asarray(candidate["raw_servo"], dtype=float)
    y = np.asarray(candidate["delta_eq_rad"], dtype=float)
    stage = session / "06_command_to_curvature_response" / "derived"
    events = pd.read_parquet(stage / "events.parquet")
    echo = pd.read_parquet(stage / "servo_echo.parquet")
    imu = pd.read_parquet(stage / "imu.parquet")
    lidar = pd.read_parquet(stage / "lidar_velocity.parquet")
    accepted = accepted_trial_ids(events)
    step_events = events[(events.get("event") == "response_step_command") &
                         (events.get("trial_id").astype(str).isin(accepted))]
    rows = []
    for _, event in step_events.iterrows():
        trial = event.get("trial_index")
        trial_id = str(event.get("trial_id"))
        start = int(event.bag_ns)
        # Stop at the response-return event for the same accepted attempt.
        end_events = events[(events.get("event") == "response_return_command") &
                            (events.get("trial_id").astype(str) == trial_id) & (events.bag_ns > start)]
        if not len(end_events):
            continue
        end = int(end_events.iloc[0].bag_ns)
        e = echo[(echo.bag_ns >= start) & (echo.bag_ns <= end)].copy()
        if len(e) < 5:
            continue
        e["t_s"] = (e.bag_ns - start) * 1e-9
        e["delta_from_servo"] = np.interp(e.value, x, y)
        initial = float(e.delta_from_servo.iloc[: max(1, len(e)//10)].mean())
        final = float(e.delta_from_servo.iloc[-max(1, len(e)//4):].mean())
        amplitude = final - initial
        if abs(amplitude) < 1e-4:
            continue
        normalized = (e.delta_from_servo.to_numpy() - initial) / amplitude
        t = e.t_s.to_numpy(dtype=float)
        delay = first_crossing(t, normalized, 0.10)
        t10 = first_crossing(t, normalized, 0.10)
        t90 = first_crossing(t, normalized, 0.90)
        rate = float(np.nanmax(np.abs(np.gradient(e.delta_from_servo.to_numpy(dtype=float), t)))) if len(t) > 2 else np.nan
        # Curvature response is calculated only where independent LiDAR velocity is valid.
        im = imu[(imu.bag_ns >= start) & (imu.bag_ns <= end)].copy()
        lv = lidar[(lidar.bag_ns >= start) & (lidar.bag_ns <= end) & (lidar.valid)].copy()
        curvature_final = np.nan
        if len(im) and len(lv):
            gz = np.interp(lv.bag_ns.to_numpy(dtype=float), im.bag_ns.to_numpy(dtype=float), im.gz.to_numpy(dtype=float))
            curvature = gz / lv.vx.to_numpy(dtype=float)
            curvature_final = float(np.nanmean(curvature[-max(1, len(curvature)//4):]))
        rows.append({
            "trial_id": trial_id, "trial_index": trial, "speed_mps": event.get("speed_mps"), "side": event.get("side"),
            "fraction": event.get("fraction"), "raw_target": event.get("raw_target"),
            "command_to_servo_delay_10pct_s": delay,
            "servo_rise_10_90_s": None if t10 is None or t90 is None else float(t90 - t10),
            "effective_servo_rate_peak_rad_s": rate,
            "servo_delta_initial_rad": initial, "servo_delta_final_rad": final,
            "curvature_final_inv_m": curvature_final,
        })
    table = pd.DataFrame(rows)
    output = session / "analysis"
    table.to_parquet(output / "command_to_curvature_response_metrics.parquet", index=False)
    summary = {"trials": int(len(table))}
    if len(table):
        summary.update({
            "median_delay_10pct_s": float(table.command_to_servo_delay_10pct_s.median()),
            "median_rise_10_90_s": float(table.servo_rise_10_90_s.median()),
            "median_peak_effective_rate_rad_s": float(table.effective_servo_rate_peak_rad_s.median()),
        })
    (output / "command_to_curvature_response_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
