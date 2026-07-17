#!/usr/bin/env python3
"""Offline re-fit of raw-servo straight-ahead centre from accepted MCAP trials."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

from imu_bias import correction_enabled, estimate_imu_bias
from lidar_windows import read_motion
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


def _centre_config(session: Path) -> dict:
    """Read the frozen campaign thresholds, with conservative fallbacks."""
    for name in ("steering_calibration_config_snapshot.yaml", "calibration_config_snapshot.yaml"):
        path = session / name
        if not path.exists():
            continue
        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if isinstance(value, dict):
            return value.get("centre_trim", {})
    return {}


def _stationary_bias_correction_enabled(session: Path) -> bool:
    """Read the full frozen profile rather than centre-only thresholds."""
    for name in ("steering_calibration_config_snapshot.yaml", "calibration_config_snapshot.yaml"):
        path = session / name
        if not path.exists():
            continue
        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        if isinstance(value, dict):
            return correction_enabled(value)
    return True


def _fit_metrics(x: np.ndarray, y: np.ndarray, coef: np.ndarray) -> dict[str, float]:
    predicted = np.polyval(coef, x)
    residual = y - predicted
    ss_res = float(np.sum(residual ** 2))
    ss_tot = float(np.sum((y - np.mean(y)) ** 2))
    r2 = 1.0 if ss_tot <= 1e-15 and ss_res <= 1e-15 else 1.0 - ss_res / ss_tot if ss_tot > 0 else float("-inf")
    return {
        "fit_r2": float(r2),
        "fit_rmse_rad_s": float(np.sqrt(np.mean(residual ** 2))),
    }


def evaluate_centre_fit(raw_servo_echo: np.ndarray, yaw_rate: np.ndarray,
                        limits: dict) -> dict[str, object]:
    """Apply the fail-closed centre gate to already aggregated trial points.

    This function is deliberately independent of ROS, MCAP and Parquet so the
    safety contract can be regression-tested with synthetic data.
    """
    x = np.asarray(raw_servo_echo, dtype=float)
    y = np.asarray(yaw_rate, dtype=float)
    min_points = int(limits.get("min_accepted_training_points", 6))
    min_unique_points = int(limits.get("min_unique_servo_points", 4))
    min_span = float(limits.get("min_training_span_servo", 0.04))
    max_extrapolation = float(limits.get("max_fit_extrapolation_servo", 0.0))
    expected_sign = int(limits.get("expected_yaw_rate_slope_sign", -1))
    min_r2 = float(limits.get("min_fit_r2", 0.80))
    max_rmse = float(limits.get("max_fit_rmse_rad_s", 0.012))
    max_nearest_yaw = float(limits.get(
        "max_nearest_observed_abs_yaw_rate_rad_s",
        limits.get("max_abs_yaw_rate_for_candidate_rad_s", 0.02),
    ))
    failures: list[str] = []
    result: dict[str, object] = {
        "accepted_training_points": int(len(x)),
        "raw_servo_span": 0.0,
        "raw_servo_min": None,
        "raw_servo_max": None,
    }
    if len(x) != len(y):
        failures.append("servo/yaw point arrays have different lengths")
    if len(x) < min_points:
        failures.append(f"accepted points {len(x)} < {min_points}")
    if len(x) and (not np.isfinite(x).all() or not np.isfinite(y).all()):
        failures.append("centre fit contains non-finite servo or yaw data")
    if len(x) and np.isfinite(x).all() and np.isfinite(y).all():
        span = float(np.max(x) - np.min(x))
        result.update({
            "raw_servo_span": span,
            "raw_servo_min": float(np.min(x)),
            "raw_servo_max": float(np.max(x)),
        })
        if len(np.unique(np.round(x, 8))) < min_unique_points:
            failures.append(f"unique servo points < {min_unique_points}")
        if span < min_span:
            failures.append(f"servo span {span:.6f} < {min_span:.6f}")

    if len(x) >= 2 and len(x) == len(y) and np.isfinite(x).all() and np.isfinite(y).all():
        coef = np.polyfit(x, y, 1)
        slope, intercept = float(coef[0]), float(coef[1])
        centre = float(-intercept / slope) if abs(slope) >= 1e-8 else None
        result.update({
            "yaw_vs_servo_slope_rad_s_per_servo": slope,
            "centre_servo_raw": centre,
            **_fit_metrics(x, y, coef),
        })
        if centre is None or not np.isfinite(centre):
            failures.append("centre fit has no finite non-zero slope")
        else:
            if expected_sign and np.sign(slope) != np.sign(expected_sign):
                failures.append(f"yaw-vs-servo slope sign {slope:.6f} disagrees with expected sign {expected_sign}")
            observed_min = float(np.min(x))
            observed_max = float(np.max(x))
            extrapolation = max(observed_min - centre, centre - observed_max, 0.0)
            nearest_yaw = float(np.min(np.abs(y)))
            result["fit_extrapolation_servo"] = float(extrapolation)
            result["nearest_observed_abs_yaw_rate_rad_s"] = nearest_yaw
            if extrapolation > max_extrapolation:
                failures.append(f"fitted centre extrapolates {extrapolation:.6f} servo > {max_extrapolation:.6f}")
            if nearest_yaw > max_nearest_yaw:
                failures.append(f"nearest observed |yaw rate| {nearest_yaw:.6f} > {max_nearest_yaw:.6f} rad/s")
    else:
        failures.append("centre fit requires finite data and at least two points")

    if "fit_r2" in result and float(result["fit_r2"]) < min_r2:
        failures.append(f"fit R² {float(result['fit_r2']):.4f} < {min_r2:.4f}")
    if "fit_rmse_rad_s" in result and float(result["fit_rmse_rad_s"]) > max_rmse:
        failures.append(f"fit RMSE {float(result['fit_rmse_rad_s']):.6f} > {max_rmse:.6f} rad/s")
    result["thresholds"] = {
        "min_accepted_training_points": min_points,
        "min_unique_servo_points": min_unique_points,
        "min_training_span_servo": min_span,
        "max_fit_extrapolation_servo": max_extrapolation,
        "expected_yaw_rate_slope_sign": expected_sign,
        "min_fit_r2": min_r2,
        "max_fit_rmse_rad_s": max_rmse,
        "max_nearest_observed_abs_yaw_rate_rad_s": max_nearest_yaw,
    }
    result["failures"] = failures
    result["accepted_for_update"] = not failures
    result["status"] = "pass" if not failures else "fail"
    return result


def bootstrap_centre_fit(raw_servo_echo: np.ndarray, yaw_rate: np.ndarray,
                         *, resamples: int = 1000, seed: int = 20260717) -> dict[str, object]:
    """Trial-resampled uncertainty for the LiDAR centre and local slope."""
    x = np.asarray(raw_servo_echo, dtype=float)
    y = np.asarray(yaw_rate, dtype=float)
    finite = np.isfinite(x) & np.isfinite(y)
    x, y = x[finite], y[finite]
    centres: list[float] = []
    slopes: list[float] = []
    if len(x) >= 3 and resamples > 0:
        rng = np.random.default_rng(seed)
        for _ in range(resamples):
            indices = rng.integers(0, len(x), len(x))
            bx, by = x[indices], y[indices]
            if len(np.unique(np.round(bx, 10))) < 2:
                continue
            slope, intercept = np.polyfit(bx, by, 1)
            if not np.isfinite(slope) or not np.isfinite(intercept) or abs(slope) < 1.0e-8:
                continue
            centre = -intercept / slope
            if np.isfinite(centre):
                centres.append(float(centre))
                slopes.append(float(slope))
    quantile = lambda values: [float(v) for v in np.quantile(values, [0.025, 0.975])] if values else []
    return {
        "method": "nonparametric bootstrap over independent accepted trials",
        "requested_resamples": int(resamples),
        "valid_resamples": int(len(centres)),
        "centre_servo_raw_95pct": quantile(centres),
        "yaw_vs_servo_slope_rad_s_per_servo_95pct": quantile(slopes),
    }


def _onboard_fit_limits(limits: dict) -> dict:
    """Relax quality thresholds for useful *coarse* IMU/odometry estimates.

    They still need an observed zero crossing and the expected sign.  They do
    not need to equal the LiDAR fit's precision because their job is to locate
    and cross-check the compact grid, not to replace the final measurement.
    """
    output = dict(limits)
    output["min_fit_r2"] = float(limits.get("min_onboard_fit_r2", 0.50))
    output["max_fit_rmse_rad_s"] = float(limits.get("max_onboard_fit_rmse_rad_s", 0.050))
    output["max_nearest_observed_abs_yaw_rate_rad_s"] = float(
        limits.get("max_onboard_nearest_abs_yaw_rate_rad_s", 0.10)
    )
    return output


def onboard_centre_consensus(imu_fit: dict, odom_fit: dict, limits: dict) -> dict[str, object]:
    """Summarise on-board centre estimates without hiding their limitations.

    Odometry yaw is useful because it is available immediately, but it is not
    fully independent: it includes the deployed steering map.  Therefore only
    a mutually consistent IMU+odom pair becomes a hard cross-check; one sensor
    alone is retained as a helpful estimate and warning, never a final update.
    """
    candidates: list[tuple[str, float]] = []
    for name, fit in (("imu", imu_fit), ("odom", odom_fit)):
        value = fit.get("centre_servo_raw") if isinstance(fit, dict) else None
        if bool(fit.get("accepted_for_update", False)) and value is not None:
            try:
                numeric = float(value)
            except (TypeError, ValueError):
                continue
            if np.isfinite(numeric):
                candidates.append((name, numeric))
    values = np.asarray([value for _, value in candidates], dtype=float)
    spread = float(np.max(values) - np.min(values)) if len(values) else None
    maximum_spread = float(limits.get("max_onboard_sensor_spread_servo", 0.020))
    consensus = float(np.median(values)) if len(values) else None
    reliable = len(values) >= 2 and spread is not None and spread <= maximum_spread
    return {
        "centre_servo_raw": consensus,
        "sensors_used": [name for name, _ in candidates],
        "sensor_centres_servo": {name: value for name, value in candidates},
        "sensor_spread_servo": spread,
        "max_sensor_spread_servo": maximum_spread,
        "reliable_crosscheck": reliable,
        "note": (
            "IMU and odometry agree sufficiently for a coarse cross-check."
            if reliable else
            "No reliable IMU+odometry consensus; keep the available estimates as diagnostics only."
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--trim-s", type=float, default=None,
                        help="Interior time removed from both ends; default is frozen centre_trim.fit_trim_s")
    args = parser.parse_args()
    session = args.session.resolve()
    limits = _centre_config(session)
    trim_s = float(args.trim_s if args.trim_s is not None else limits.get("fit_trim_s", 0.25))
    stage = session / "01_zero_curvature_centre" / "derived"
    events = pd.read_parquet(stage / "events.parquet")
    imu = pd.read_parquet(stage / "imu.parquet")
    echo = pd.read_parquet(stage / "servo_echo.parquet")
    odom_path = stage / "odom.parquet"
    odom = pd.read_parquet(odom_path) if odom_path.exists() else pd.DataFrame()
    lidar_path = stage / "lidar_window_motion.parquet"
    if not lidar_path.exists() and not (stage / "lidar_velocity.parquet").exists():
        raise SystemExit(f"centre fit requires independent LiDAR motion data: {lidar_path}")
    lidar = read_motion(stage)
    # A stationary epoch is useful as a diagnostic, but it is not required for
    # this stage.  Each bringup performs its own odometry startup calibration,
    # and a single early ground-bias number can become stale.  The primary fit
    # below is independent LiDAR ICP; IMU is retained as a cross-check only.
    bias = estimate_imu_bias(
        session,
        apply_correction=_stationary_bias_correction_enabled(session),
    )
    max_lateral = float(limits.get("max_abs_lidar_lateral_velocity_mps", 0.15))
    min_forward_speed = float(limits.get("min_lidar_forward_speed_mps", 0.20))
    rows = []
    for start, end in intervals(events, "centre_trim_capture"):
        a, b = int(start["bag_ns"] + trim_s * 1e9), int(end["bag_ns"] - trim_s * 1e9)
        if b <= a:
            continue
        im = imu[(imu.bag_ns >= a) & (imu.bag_ns <= b)]
        ec = echo[(echo.bag_ns >= a) & (echo.bag_ns <= b)]
        od = odom[(odom.bag_ns >= a) & (odom.bag_ns <= b)] if len(odom) else odom
        lv_all = lidar[(lidar.bag_ns >= a) & (lidar.bag_ns <= b)]
        lv = lv_all[lv_all.valid.astype(bool)] if len(lv_all) and "valid" in lv_all else lv_all.iloc[0:0]
        forward_speed = float(lv.vx.median()) if len(lv) else float("nan")
        if (len(im) < 10 or len(ec) < 3 or len(lv) < 3
                or not np.isfinite(forward_speed) or forward_speed < min_forward_speed):
            continue
        gz_bias = bias.gz_at(0.5 * (a + b))
        rows.append({
            "trial_id": start.get("trial_id"),
            "raw_servo_target": float(start.get("raw_servo_target")),
            "raw_servo_echo": float(ec.value.mean()),
            "yaw_rate_imu_rad_s": float(im.gz.mean()) - gz_bias,
            "yaw_rate_odom_rad_s": float(od.wz.mean()) if len(od) and "wz" in od else float("nan"),
            "yaw_rate_rad_s": float(lv.yaw_rate_icp.mean()),
            "yaw_rate_icp_rad_s": float(lv.yaw_rate_icp.mean()),
            "lidar_vx_mps": float(lv.vx.mean()),
            "lidar_vy_mps": float(lv.vy.mean()),
            "lidar_vy_abs_mps": float(abs(lv.vy.mean())),
            "lidar_valid_count": int(len(lv)),
            "lidar_pair_count": int(len(lv_all)),
            "lidar_valid_fraction": float(len(lv) / max(1, len(lv_all))),
            "yaw_rate_std_rad_s": float(im.gz.std()),
            "gyro_z_bias_rad_s": float(gz_bias),
            "sample_count": int(len(im)),
        })
    table = pd.DataFrame(rows)
    eligible = table[
        np.isfinite(table.lidar_vy_mps.to_numpy(dtype=float)) &
        (np.abs(table.lidar_vy_mps.to_numpy(dtype=float)) <= max_lateral)
    ].copy() if len(table) else table
    x = eligible.raw_servo_echo.to_numpy(dtype=float) if len(eligible) else np.array([])
    icp_yaw = eligible.yaw_rate_icp_rad_s.to_numpy(dtype=float) if len(eligible) else np.array([])
    imu_yaw = eligible.yaw_rate_imu_rad_s.to_numpy(dtype=float) if len(eligible) else np.array([])
    odom_yaw = eligible.yaw_rate_odom_rad_s.to_numpy(dtype=float) if len(eligible) else np.array([])
    primary = evaluate_centre_fit(x, icp_yaw, limits)
    primary["bootstrap"] = bootstrap_centre_fit(
        x,
        icp_yaw,
        resamples=int(limits.get("bootstrap_resamples", 1000)),
        seed=int(limits.get("bootstrap_seed", 20260717)),
    )
    # The on-board signals provide the useful coarse estimate the grid is based
    # around.  They cannot replace LiDAR fine fitting: raw IMU can retain a
    # small bias and odometry incorporates the currently deployed servo map.
    onboard_limits = _onboard_fit_limits(limits)
    imu_fit = evaluate_centre_fit(x, imu_yaw, onboard_limits)
    odom_fit = evaluate_centre_fit(x, odom_yaw, onboard_limits)
    onboard = onboard_centre_consensus(imu_fit, odom_fit, limits)
    result = dict(primary)
    result.update({
        "primary_sensor": "lidar_icp_fine_fit",
        "trim_s": trim_s,
        "gyro_z_bias": bias.to_dict(),
        "imu_fit": imu_fit,
        "odom_fit": odom_fit,
        "onboard_coarse_estimate": onboard,
        "all_captured_trials": int(len(table)),
        "fit_eligible_trials": int(len(eligible)),
        "excluded_nonstraight_trials": table.loc[~table.index.isin(eligible.index), "trial_id"].astype(str).tolist() if len(table) else [],
        "max_abs_lidar_lateral_velocity_mps": max_lateral,
        "min_lidar_forward_speed_mps": min_forward_speed,
    })
    trusted_imu = bool(limits.get("require_trusted_imu_crosscheck", False)) and not bool(bias.used_fallback_zero)
    result["imu_crosscheck_gated"] = trusted_imu
    if primary.get("centre_servo_raw") is not None and imu_fit.get("centre_servo_raw") is not None:
        disagreement = abs(float(primary["centre_servo_raw"]) - float(imu_fit["centre_servo_raw"]))
        result["sensor_centre_disagreement_servo"] = disagreement
        max_disagreement = float(limits.get("max_sensor_centre_disagreement_servo", 0.012))
        result["max_sensor_centre_disagreement_servo"] = max_disagreement
        if trusted_imu and disagreement > max_disagreement:
            result["failures"].append(
                f"LiDAR/IMU centre disagreement {disagreement:.6f} servo > {max_disagreement:.6f}"
            )
        elif disagreement > max_disagreement:
            result["imu_crosscheck_warning"] = (
                f"LiDAR/IMU centre disagreement {disagreement:.6f} servo exceeds the diagnostic threshold; "
                "it does not veto this LiDAR-primary fit because no trusted stationary IMU epoch exists."
            )
    else:
        result["sensor_centre_disagreement_servo"] = None
        if trusted_imu:
            result["failures"].append("trusted IMU cross-check did not produce a finite centre")
        else:
            result["imu_crosscheck_warning"] = "IMU cross-check did not produce a finite centre; LiDAR remains primary."
    lidar_centre = result.get("centre_servo_raw")
    onboard_centre = onboard.get("centre_servo_raw")
    if lidar_centre is not None and onboard_centre is not None:
        disagreement = abs(float(lidar_centre) - float(onboard_centre))
        maximum = float(limits.get("max_lidar_onboard_disagreement_servo", 0.030))
        result["lidar_onboard_centre_disagreement_servo"] = disagreement
        result["max_lidar_onboard_disagreement_servo"] = maximum
        if bool(onboard.get("reliable_crosscheck", False)) and disagreement > maximum:
            message = (
                f"LiDAR/on-board consensus centre disagreement {disagreement:.6f} servo > {maximum:.6f}"
            )
            if bool(limits.get("require_onboard_consensus_when_available", True)):
                result["failures"].append(message)
            else:
                result["onboard_consensus_warning"] = message
        elif disagreement > maximum:
            result["onboard_consensus_warning"] = (
                f"LiDAR/on-board estimate disagreement {disagreement:.6f} servo exceeds the diagnostic threshold; "
                "the on-board sensors did not form a reliable mutual consensus."
            )
    else:
        result["lidar_onboard_centre_disagreement_servo"] = None
    result["accepted_for_update"] = not result["failures"]
    result["status"] = "pass" if result["accepted_for_update"] else "fail"
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    table.to_parquet(output / "centre_trim_points.parquet", index=False)
    (output / "centre_trim_offline.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    if not bool(result["accepted_for_update"]):
        raise SystemExit("centre-trim offline gate failed: " + "; ".join(result["failures"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
