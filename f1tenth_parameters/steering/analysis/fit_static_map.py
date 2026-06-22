#!/usr/bin/env python3
"""Fit and validate the raw-servo to effective-steering static map offline."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from trials import accepted_trial_ids


def capture_intervals(events: pd.DataFrame) -> list[tuple[dict, dict]]:
    accepted = accepted_trial_ids(events)
    starts = events[(events.get("event") == "phase_start") & (events.get("phase") == "static_map_capture")]
    ends = events[(events.get("event") == "phase_end") & (events.get("phase") == "static_map_capture")]
    out = []
    for _, start in starts.iterrows():
        trial_id = str(start.get("trial_id"))
        if trial_id not in accepted:
            continue
        matches = ends[(ends.get("trial_id").astype(str) == trial_id) & (ends.bag_ns > start.bag_ns)]
        if len(matches):
            out.append((start.to_dict(), matches.iloc[0].to_dict()))
    return out


def latest_target(events: pd.DataFrame, before_ns: int, trial_id: str) -> dict:
    candidates = events[(events.get("event") == "static_map_target") &
                        (events.get("trial_id").astype(str) == str(trial_id)) &
                        (events.bag_ns <= before_ns)]
    if not len(candidates):
        return {}
    return candidates.iloc[-1].to_dict()


def segment_rows(stage_dir: Path, wheelbase: float, trim_s: float, criteria: dict) -> pd.DataFrame:
    d = stage_dir / "derived"
    events = pd.read_parquet(d / "events.parquet")
    imu = pd.read_parquet(d / "imu.parquet")
    echo = pd.read_parquet(d / "servo_echo.parquet")
    lidar = pd.read_parquet(d / "lidar_velocity.parquet")
    rows = []
    for start, end in capture_intervals(events):
        target = latest_target(events, int(start["bag_ns"]), str(start.get("trial_id")))
        if not target:
            continue
        a = int(start["bag_ns"] + trim_s * 1e9)
        b = int(end["bag_ns"] - trim_s * 1e9)
        im = imu[(imu.bag_ns >= a) & (imu.bag_ns <= b)]
        ec = echo[(echo.bag_ns >= a) & (echo.bag_ns <= b)]
        lv_all = lidar[(lidar.bag_ns >= a) & (lidar.bag_ns <= b)]
        lv = lv_all[lv_all.valid]
        if len(im) < 10 or len(ec) < 3 or len(lv_all) < 3 or len(lv) < 3:
            continue
        valid_fraction = float(len(lv) / len(lv_all))
        vx, gz = float(lv.vx.mean()), float(im.gz.mean())
        vx_std, gz_std = float(lv.vx.std()), float(im.gz.std())
        rmse = float(lv.icp_rmse_m.mean())
        accepted = (
            valid_fraction >= float(criteria["min_valid_scan_fraction"]) and
            abs(vx) >= float(criteria["min_lidar_speed_mps"]) and
            vx_std <= float(criteria["max_lidar_speed_std_mps"]) and
            gz_std <= float(criteria["max_yaw_std_rad_s"]) and
            rmse <= float(criteria["max_icp_rmse_m"])
        )
        delta = float(np.arctan(wheelbase * gz / vx))
        rows.append({
            "trial_id": start.get("trial_id"), "segment_id": start["segment_id"], "raw_servo_target": target.get("raw_servo_target"),
            "raw_servo_echo": float(ec.value.mean()), "side": target.get("side"),
            "fraction": target.get("fraction"), "approach": target.get("approach"),
            "vx_lidar": vx, "vx_lidar_std": vx_std, "yaw_rate": gz, "yaw_rate_std": gz_std,
            "icp_rmse_m": rmse, "valid_scan_fraction": valid_fraction, "delta_eq_rad": delta, "accepted": bool(accepted),
        })
    return pd.DataFrame(rows)


def map_interpolate(training: pd.DataFrame, centre_servo: float) -> tuple[np.ndarray, np.ndarray]:
    # Median across repeated outward/inward captures. Include the experimentally
    # identified centre as delta=0 so the two side maps meet at a physical origin.
    grouped = training.groupby("raw_servo_echo", as_index=False).delta_eq_rad.median().sort_values("raw_servo_echo")
    x = np.r_[centre_servo, grouped.raw_servo_echo.to_numpy(dtype=float)]
    y = np.r_[0.0, grouped.delta_eq_rad.to_numpy(dtype=float)]
    order = np.argsort(x)
    x, y = x[order], y[order]
    unique, indices = np.unique(x, return_index=True)
    return unique, y[indices]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--config", type=Path, default=None)
    args = parser.parse_args()
    session = args.session.resolve()
    config_path = args.config or (session / "calibration_config_snapshot.yaml")
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    centre = json.loads((session / "analysis" / "centre_trim_offline.json").read_text(encoding="utf-8"))
    criteria = cfg["analysis"]["map"]
    wheelbase = float(cfg["hardware"]["wheelbase_m"])
    train = segment_rows(session / "04_static_map_training", wheelbase, float(criteria["trim_s"]), criteria)
    holdout = segment_rows(session / "05_static_map_holdout", wheelbase, float(criteria["trim_s"]), criteria)
    if len(train) == 0:
        raise SystemExit("no usable static-map training segments")
    train.to_parquet(session / "analysis" / "static_map_training_segments.parquet", index=False)
    holdout.to_parquet(session / "analysis" / "static_map_holdout_segments.parquet", index=False)
    accepted = train[train.accepted].copy()
    if len(accepted) < 6:
        raise SystemExit("fewer than six accepted static-map points; inspect LiDAR motion quality")
    x, y = map_interpolate(accepted, float(centre["centre_servo_raw"]))
    candidate = {"centre_servo_raw": float(centre["centre_servo_raw"]), "raw_servo": x.tolist(), "delta_eq_rad": y.tolist()}
    valid = holdout[holdout.accepted].copy()
    if len(valid):
        valid["delta_pred_rad"] = np.interp(valid.raw_servo_echo, x, y)
        valid["error_rad"] = valid.delta_pred_rad - valid.delta_eq_rad
        candidate["holdout_points"] = int(len(valid))
        candidate["holdout_rmse_rad"] = float(np.sqrt(np.mean(valid.error_rad ** 2)))
        candidate["holdout_bias_rad"] = float(valid.error_rad.mean())
        valid.to_parquet(session / "analysis" / "static_map_holdout_evaluated.parquet", index=False)
    else:
        candidate["holdout_points"] = 0
        candidate["holdout_rmse_rad"] = None
        candidate["holdout_bias_rad"] = None
    (session / "analysis" / "candidate_static_steering_map.json").write_text(json.dumps(candidate, indent=2) + "\n")
    print(json.dumps(candidate, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
