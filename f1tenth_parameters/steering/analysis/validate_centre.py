#!/usr/bin/env python3
"""Validate that the applied steering centre makes the car track straight."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from lidar_windows import read_motion
from trials import accepted_trial_ids


def _windows(events: pd.DataFrame, phase: str) -> list[tuple[dict, dict]]:
    accepted = accepted_trial_ids(events)
    starts = events[(events.get("event") == "phase_start") & (events.get("phase") == phase)]
    ends = events[(events.get("event") == "phase_end") & (events.get("phase") == phase)]
    output: list[tuple[dict, dict]] = []
    for _, start in starts.iterrows():
        trial_id = str(start.get("trial_id"))
        if trial_id not in accepted:
            continue
        matches = ends[(ends.get("trial_id").astype(str) == trial_id) & (ends["bag_ns"] > start["bag_ns"])]
        if len(matches):
            output.append((start.to_dict(), matches.iloc[0].to_dict()))
    return output


def _integral(time_ns: np.ndarray, values: np.ndarray) -> float:
    if len(values) < 2:
        return float("nan")
    order = np.argsort(time_ns)
    t = time_ns[order].astype(float) * 1e-9
    y = values[order].astype(float)
    return float(np.trapz(y, t))


def _finite_or_none(value: float) -> float | None:
    return float(value) if np.isfinite(value) else None


def _as_float_or_nan(value: object) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return float("nan")
    return result if np.isfinite(result) else float("nan")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--trim-s", type=float, default=None)
    args = parser.parse_args()
    session = args.session.resolve()
    snapshot_path = session / "steering_calibration_config_snapshot.yaml"
    if not snapshot_path.exists():
        snapshot_path = session / "calibration_config_snapshot.yaml"
    snapshot = yaml.safe_load(snapshot_path.read_text(encoding="utf-8")) or {}
    limits = snapshot.get("centre_trim", {})
    imu_crosscheck_gated = bool(limits.get("require_validation_imu_crosscheck", False))
    trim_s = float(args.trim_s if args.trim_s is not None else limits.get("validation_trim_s", 0.25))
    stage = session / "01a_zero_curvature_validation" / "derived"
    required = [stage / "events.parquet", stage / "imu.parquet"]
    missing = [str(path) for path in required if not path.exists()]
    if missing or not (stage / "lidar_window_motion.parquet").exists() and not (stage / "lidar_velocity.parquet").exists():
        raise SystemExit("centre validation requires exported IMU, events and LiDAR tables: " + ", ".join(missing))
    events = pd.read_parquet(required[0])
    imu = pd.read_parquet(required[1])
    lidar = read_motion(stage)
    rows: list[dict] = []
    phase = "centre_validation_capture"
    for start, end in _windows(events, phase):
        a = int(start["bag_ns"] + trim_s * 1e9)
        b = int(end["bag_ns"] - trim_s * 1e9)
        if b <= a:
            continue
        lv_all = lidar[(lidar.bag_ns >= a) & (lidar.bag_ns <= b)]
        lv = lv_all[lv_all.valid.astype(bool)] if len(lv_all) and "valid" in lv_all else lv_all.iloc[0:0]
        im = imu[(imu.bag_ns >= a) & (imu.bag_ns <= b)]
        duration_s = max(0.0, (b - a) * 1e-9)
        valid_fraction = float(len(lv) / max(1, len(lv_all)))
        icp_yaw = float(lv.yaw_rate_icp.median()) if len(lv) else float("nan")
        lateral = float(lv.vy.median()) if len(lv) else float("nan")
        longitudinal = float(lv.vx.median()) if len(lv) else float("nan")
        imu_yaw = float(im.gz.mean()) if len(im) else float("nan")
        heading_change = _integral(lv.bag_ns.to_numpy(float), lv.yaw_rate_icp.to_numpy(float)) if len(lv) else float("nan")
        failures: list[str] = []
        if valid_fraction < float(limits.get("min_validation_lidar_valid_fraction", 0.70)):
            failures.append("LiDAR valid fraction below gate")
        if not np.isfinite(longitudinal) or longitudinal <= 0.10:
            failures.append("LiDAR longitudinal speed is missing or too low")
        if not np.isfinite(icp_yaw) or abs(icp_yaw) > float(limits.get("max_abs_validation_icp_yaw_rate_rad_s", 0.015)):
            failures.append("LiDAR ICP yaw rate exceeds straightness gate")
        if not np.isfinite(lateral) or abs(lateral) > float(limits.get("max_abs_validation_lateral_velocity_mps", 0.05)):
            failures.append("LiDAR lateral velocity exceeds straightness gate")
        if not np.isfinite(heading_change) or abs(heading_change) > float(limits.get("max_abs_validation_heading_change_rad", 0.035)):
            failures.append("integrated LiDAR heading change exceeds straightness gate")
        disagreement = abs(icp_yaw - imu_yaw) if np.isfinite(icp_yaw) and np.isfinite(imu_yaw) else float("nan")
        if imu_crosscheck_gated and (
            not np.isfinite(disagreement)
            or disagreement > float(limits.get("max_validation_sensor_yaw_disagreement_rad_s", 0.04))
        ):
            failures.append("LiDAR/IMU yaw-rate disagreement exceeds gate")
        rows.append({
            "trial_id": str(start.get("trial_id")),
            "condition_id": str(start.get("condition_id", start.get("segment_id", ""))),
            "validation_lane_direction": start.get("validation_lane_direction"),
            "validation_speed_mps": _as_float_or_nan(start.get("validation_speed_mps", start.get("speed_mps"))),
            "raw_servo": float(start.get("raw_servo_target", float("nan"))),
            "duration_s": duration_s,
            "lidar_pairs": int(len(lv_all)),
            "lidar_valid_pairs": int(len(lv)),
            "lidar_valid_fraction": valid_fraction,
            "lidar_vx_mps": longitudinal,
            "lidar_vy_mps": lateral,
            "yaw_rate_icp_rad_s": icp_yaw,
            "yaw_rate_imu_rad_s": imu_yaw,
            "sensor_yaw_disagreement_rad_s": disagreement,
            "heading_change_rad": heading_change,
            "accepted_for_validation": not failures,
            "failures": "; ".join(failures),
        })
    table = pd.DataFrame(rows)
    configured_conditions = list(limits.get("validation_conditions", []))
    requested = (
        sum(int(condition.get("repetitions", 1)) for condition in configured_conditions)
        if configured_conditions else int(limits.get("validation_repetitions", 4))
    )
    failures = []
    if len(table) < requested:
        failures.append(f"accepted validation trials {len(table)} < {requested}")
    if len(table) and not bool(table.accepted_for_validation.all()):
        failures.extend(
            f"{row.trial_id}: {row.failures}" for row in table[~table.accepted_for_validation].itertuples()
        )
    if not len(table):
        failures.append("no accepted validation windows")
    coverage: list[dict] = []
    if configured_conditions:
        for condition in configured_conditions:
            direction = str(condition.get("lane_direction", ""))
            speed = float(condition["speed_mps"])
            expected = int(condition.get("repetitions", 1))
            rows_for_condition = table[
                (table.validation_lane_direction.astype(str) == direction)
                & np.isclose(table.validation_speed_mps.to_numpy(float), speed, rtol=0.0, atol=1.0e-6)
            ] if len(table) else table
            count = int(len(rows_for_condition))
            passing = int(rows_for_condition.accepted_for_validation.sum()) if count else 0
            coverage.append({
                "lane_direction": direction,
                "speed_mps": speed,
                "expected_trials": expected,
                "accepted_windows": count,
                "passing_trials": passing,
            })
            if count < expected:
                failures.append(
                    f"validation condition {direction} at {speed:.2f} m/s has {count} accepted windows < {expected}"
                )
            if passing < expected:
                failures.append(
                    f"validation condition {direction} at {speed:.2f} m/s has {passing} passing trials < {expected}"
                )
    runtime_path = session / "01a_zero_curvature_validation" / "runtime_result.json"
    runtime = json.loads(runtime_path.read_text(encoding="utf-8")) if runtime_path.exists() else {}
    centre_raw = float(runtime.get("centre_servo_raw", float("nan")))
    if not np.isfinite(centre_raw):
        centre_raw = None
    report = {
        "status": "pass" if not failures else "fail",
        "accepted_for_validation": not failures,
        "centre_servo_raw": centre_raw,
        "trim_s": trim_s,
        "accepted_trials": int(len(table)),
        "requested_repetitions": requested,
        "condition_coverage": coverage,
        "passing_trials": int(table.accepted_for_validation.sum()) if len(table) else 0,
        "failures": failures,
        "thresholds": {
            "min_validation_lidar_valid_fraction": float(limits.get("min_validation_lidar_valid_fraction", 0.70)),
            "max_abs_validation_icp_yaw_rate_rad_s": float(limits.get("max_abs_validation_icp_yaw_rate_rad_s", 0.015)),
            "max_abs_validation_lateral_velocity_mps": float(limits.get("max_abs_validation_lateral_velocity_mps", 0.05)),
            "max_abs_validation_heading_change_rad": float(limits.get("max_abs_validation_heading_change_rad", 0.035)),
            "max_validation_sensor_yaw_disagreement_rad_s": float(limits.get("max_validation_sensor_yaw_disagreement_rad_s", 0.04)),
            "imu_crosscheck_gated": imu_crosscheck_gated,
        },
        "operator_rule": "ACCEPT only when the vehicle physically tracks straight; visible drift requires REDO.",
    }
    if len(table):
        report.update({
            "median_icp_yaw_rate_rad_s": _finite_or_none(float(table.yaw_rate_icp_rad_s.median())),
            "median_lateral_velocity_mps": _finite_or_none(float(table.lidar_vy_mps.median())),
            "max_abs_heading_change_rad": _finite_or_none(float(np.nanmax(np.abs(table.heading_change_rad)))) if table.heading_change_rad.notna().any() else None,
        })
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    table.to_parquet(output / "centre_validation_trials.parquet", index=False)
    (output / "centre_validation_report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    if failures:
        raise SystemExit("centre physical-validation gate failed: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
