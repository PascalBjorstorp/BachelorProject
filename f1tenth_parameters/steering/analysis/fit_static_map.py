#!/usr/bin/env python3
"""Fit and validate the raw-servo to effective-steering static map offline.

The experimental key is the commanded nominal condition:

    side + configured fraction + approach direction

The servo command echo is deliberately *not* used as a grouping key.  It is a
measured regressor inside each repeated condition and is retained in the output
for command-path diagnostics and the final interpolation axis.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml

from imu_bias import correction_enabled, estimate_imu_bias
from lidar_windows import read_motion
from trials import accepted_trial_ids


TRAINING_APPROACHES = ("outward", "inward")
HOLDOUT_APPROACHES = ("shuffled",)


def _mad_std(values: pd.Series | np.ndarray) -> float:
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if array.size == 0:
        return float("nan")
    median = float(np.median(array))
    return float(1.4826 * np.median(np.abs(array - median)))


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


def _fraction_key(value: Any) -> str:
    """Stable condition label for a configured fraction, not a measured value."""
    if pd.isna(value):
        return "missing"
    return f"{float(value):.6f}"


def add_nominal_condition_keys(rows: pd.DataFrame) -> pd.DataFrame:
    """Add deterministic keys based only on the commanded experiment condition."""
    result = rows.copy()
    if len(result) == 0:
        result["fraction_key"] = pd.Series(dtype="string")
        result["condition_key"] = pd.Series(dtype="string")
        result["condition_approach_key"] = pd.Series(dtype="string")
        return result
    result["side"] = result["side"].astype(str)
    result["approach"] = result["approach"].astype(str)
    result["fraction_key"] = result["fraction"].map(_fraction_key)
    result["condition_key"] = result["side"] + "|f=" + result["fraction_key"]
    result["condition_approach_key"] = result["condition_key"] + "|approach=" + result["approach"]
    return result


def rear_axle_velocity(vx_base: float, vy_base: float, yaw_rate: float,
                       rear_axle_x_m: float, rear_axle_y_m: float) -> tuple[float, float]:
    """Translate a base-link velocity measurement to the rear-axle midpoint.

    The low-speed bicycle relation used by the static map is written at the
    rear axle: ``tan(delta) = L * r / v_x,rear``.  LiDAR motion is deliberately
    transformed to ``base_link`` first, so using it directly here silently
    assumes that ``base_link`` is on the rear axle.  Keeping this small rigid
    body translation explicit makes the map valid when the base frame is
    mounted elsewhere on the chassis.
    """
    values = (vx_base, vy_base, yaw_rate, rear_axle_x_m, rear_axle_y_m)
    if not all(np.isfinite(float(value)) for value in values):
        return float("nan"), float("nan")
    return (
        float(vx_base - yaw_rate * rear_axle_y_m),
        float(vy_base + yaw_rate * rear_axle_x_m),
    )


def rear_axle_geometry(cfg: dict) -> tuple[float, float]:
    """Read the rear-axle reference from a frozen calibration configuration.

    A legacy standalone steering session may not have the newer direct
    metrology fields, in which case the historical base-link-at-rear-axle
    assumption remains explicit. Unified sessions overwrite both fields before
    this fitter is allowed to run.
    """
    hardware = cfg.get("hardware", {}) if isinstance(cfg, dict) else {}
    try:
        return (
            float(hardware.get("base_link_to_rear_axle_x_m", 0.0)),
            float(hardware.get("base_link_to_rear_axle_y_m", 0.0)),
        )
    except (TypeError, ValueError) as exc:
        raise ValueError("static steering map has invalid base_link-to-rear-axle geometry") from exc


def segment_rows(stage_dir: Path, wheelbase: float, trim_s: float, criteria: dict,
                 bias=None, *, rear_axle_x_m: float = 0.0,
                 rear_axle_y_m: float = 0.0) -> pd.DataFrame:
    d = stage_dir / "derived"
    events = pd.read_parquet(d / "events.parquet")
    imu = pd.read_parquet(d / "imu.parquet")
    echo = pd.read_parquet(d / "servo_echo.parquet")
    lidar = read_motion(d)
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
        # A unified session treats the stationary epoch as a diagnostic by
        # default: bringup recalibrates the IMU for each launched stack.  The
        # profile can explicitly retain the legacy correction policy, but it is
        # never silently assumed here.
        gz_bias = bias.gz_at(0.5 * (a + b)) if bias is not None else 0.0
        ay_bias = bias.ay_bias if bias is not None and bias.correction_applied else 0.0
        vx_base = float(lv.vx.median())
        vy_base = float(lv.vy.median()) if "vy" in lv else float("nan")
        vx_std = _mad_std(lv.vx)
        # The robust LiDAR multi-registration window is the independent
        # vehicle-motion reference.  IMU yaw remains a high-rate diagnostic,
        # not the source of the raw-servo-to-effective-steering map.
        lidar_gz = float(lv.yaw_rate_icp.median()) if "yaw_rate_icp" in lv else float("nan")
        lidar_gz_std = _mad_std(lv.yaw_rate_icp) if "yaw_rate_icp" in lv else float("nan")
        imu_gz = float(im.gz.median()) - gz_bias
        imu_gz_std = _mad_std(im.gz)
        ay_mean, ay_std = float(im.ay.median()) - ay_bias, _mad_std(im.ay)
        rmse = float(lv.icp_rmse_m.median())
        vx_rear, vy_rear = rear_axle_velocity(
            vx_base, vy_base, lidar_gz, rear_axle_x_m, rear_axle_y_m,
        )
        accepted = (
            valid_fraction >= float(criteria["min_valid_scan_fraction"]) and
            abs(vx_rear) >= float(criteria["min_lidar_speed_mps"]) and
            vx_std <= float(criteria["max_lidar_speed_std_mps"]) and
            np.isfinite(lidar_gz) and
            lidar_gz_std <= float(criteria.get("max_lidar_yaw_std_rad_s", criteria["max_yaw_std_rad_s"])) and
            rmse <= float(criteria["max_icp_rmse_m"])
        )
        delta = float(np.arctan(wheelbase * lidar_gz / vx_rear)) if np.isfinite(lidar_gz) and np.isfinite(vx_rear) and abs(vx_rear) > 1e-9 else float("nan")
        rows.append({
            "trial_id": start.get("trial_id"),
            "condition_id": target.get("condition_id", start.get("segment_id")),
            "segment_id": start["segment_id"],
            "raw_servo_target": target.get("raw_servo_target"),
            "raw_servo_echo": float(ec.value.mean()),
            "side": target.get("side"),
            "fraction": target.get("fraction"),
            "approach": target.get("approach"),
            "sweep": target.get("sweep"),
            "vx_lidar_base_link": vx_base,
            "vy_lidar_base_link": vy_base,
            "vx_lidar": vx_rear,
            "vy_lidar_rear_axle": vy_rear,
            "vx_lidar_std": vx_std,
            "yaw_rate": lidar_gz,
            "yaw_rate_std": lidar_gz_std,
            "yaw_rate_lidar_rad_s": lidar_gz,
            "yaw_rate_lidar_std_rad_s": lidar_gz_std,
            "yaw_rate_imu_rad_s": imu_gz,
            "yaw_rate_imu_std_rad_s": imu_gz_std,
            "imu_minus_lidar_yaw_rate_rad_s": imu_gz - lidar_gz,
            "gyro_z_bias_applied_rad_s": gz_bias,
            "imu_ay_mean": ay_mean,
            "imu_ay_std": ay_std,
            "curvature_inv_m": float(lidar_gz / vx_rear) if np.isfinite(lidar_gz) and np.isfinite(vx_rear) and abs(vx_rear) > 1e-9 else float("nan"),
            "lateral_accel_kinematic_mps2": float(vx_rear * lidar_gz) if np.isfinite(vx_rear) and np.isfinite(lidar_gz) else float("nan"),
            "rear_axle_in_base_link_x_m": float(rear_axle_x_m),
            "rear_axle_in_base_link_y_m": float(rear_axle_y_m),
            "icp_rmse_m": rmse,
            "valid_scan_fraction": valid_fraction,
            "delta_eq_rad": delta,
            "accepted": bool(accepted),
        })
    return add_nominal_condition_keys(pd.DataFrame(rows))


def _condition_summary(accepted: pd.DataFrame) -> pd.DataFrame:
    """Collapse only repeated *nominal* conditions, never exact echo floats."""
    return (
        accepted.groupby(["side", "fraction_key"], dropna=False)
        .agg(
            fraction=("fraction", "first"),
            raw_servo_target_median=("raw_servo_target", "median"),
            raw_servo_echo_median=("raw_servo_echo", "median"),
            raw_servo_echo_mean=("raw_servo_echo", "mean"),
            raw_servo_echo_std=("raw_servo_echo", "std"),
            condition_sample_count=("delta_eq_rad", "count"),
            delta_eq_median=("delta_eq_rad", "median"),
            delta_eq_mean=("delta_eq_rad", "mean"),
            delta_eq_std=("delta_eq_rad", "std"),
        )
        .reset_index()
        .sort_values("raw_servo_echo_median")
        .reset_index(drop=True)
    )


def _repeatability_summary(accepted: pd.DataFrame) -> pd.DataFrame:
    """Scatter for repeated same-side/fraction/approach conditions."""
    return (
        accepted.groupby(["side", "fraction_key", "approach"], dropna=False)
        .agg(
            fraction=("fraction", "first"),
            raw_servo_target_median=("raw_servo_target", "median"),
            raw_servo_echo_median=("raw_servo_echo", "median"),
            raw_servo_echo_mean=("raw_servo_echo", "mean"),
            raw_servo_echo_std=("raw_servo_echo", "std"),
            repeat_count=("delta_eq_rad", "count"),
            delta_eq_mean=("delta_eq_rad", "mean"),
            delta_eq_median=("delta_eq_rad", "median"),
            repeatability_std_rad=("delta_eq_rad", "std"),
        )
        .reset_index()
        .sort_values(["side", "fraction", "approach"])
        .reset_index(drop=True)
    )


def _hysteresis_summary(repeatability: pd.DataFrame) -> pd.DataFrame:
    """Outward/inward median difference at the same nominal command condition."""
    if len(repeatability) == 0:
        return pd.DataFrame(columns=["side", "fraction_key", "fraction", "outward", "inward", "hysteresis_delta_rad"])
    pivot = (
        repeatability.pivot_table(
            index=["side", "fraction_key", "fraction"],
            columns="approach",
            values="delta_eq_median",
            aggfunc="first",
        )
        .reset_index()
        .sort_values(["side", "fraction"])
        .reset_index(drop=True)
    )
    if "outward" in pivot.columns and "inward" in pivot.columns:
        pivot["hysteresis_delta_rad"] = pivot["outward"] - pivot["inward"]
    else:
        pivot["hysteresis_delta_rad"] = np.nan
    return pivot


def _expected_condition_coverage(
    fractions: list[float],
    approaches: tuple[str, ...],
    required_count: int,
) -> pd.DataFrame:
    rows = []
    for side in ("high_raw", "low_raw"):
        for fraction in fractions:
            for approach in approaches:
                rows.append({
                    "side": side,
                    "fraction": float(fraction),
                    "fraction_key": _fraction_key(fraction),
                    "approach": approach,
                    "required_count": int(required_count),
                })
    return pd.DataFrame(rows)


def _coverage_report(
    accepted: pd.DataFrame,
    *,
    fractions: list[float],
    approaches: tuple[str, ...],
    required_count: int,
    label: str,
) -> tuple[pd.DataFrame, list[str]]:
    """Return explicit condition coverage and human-readable gate failures."""
    expected = _expected_condition_coverage(fractions, approaches, required_count)
    observed = (
        accepted.groupby(["side", "fraction_key", "approach"], dropna=False)
        .size()
        .rename("accepted_count")
        .reset_index()
    )
    report = expected.merge(observed, how="left", on=["side", "fraction_key", "approach"])
    report["accepted_count"] = report["accepted_count"].fillna(0).astype(int)
    report["coverage_status"] = np.where(
        report["accepted_count"] >= report["required_count"], "pass", "under_replicated"
    )
    failures = []
    for row in report[report.coverage_status != "pass"].itertuples(index=False):
        failures.append(
            f"{label} {row.side} fraction={float(row.fraction):.6f} approach={row.approach}: "
            f"{int(row.accepted_count)} accepted < required {int(row.required_count)}"
        )
    return report, failures


def map_interpolate(condition_summary: pd.DataFrame, centre_servo: float) -> tuple[np.ndarray, np.ndarray]:
    """Construct a single candidate map from median measured echo per nominal condition."""
    if len(condition_summary) == 0:
        raise ValueError("no nominal static-map conditions available")
    x = np.r_[centre_servo, condition_summary.raw_servo_echo_median.to_numpy(dtype=float)]
    y = np.r_[0.0, condition_summary.delta_eq_median.to_numpy(dtype=float)]
    order = np.argsort(x)
    x, y = x[order], y[order]
    if not np.all(np.isfinite(x)) or not np.all(np.isfinite(y)):
        raise ValueError("non-finite static-map interpolation values")
    if np.any(np.diff(x) <= 1e-9):
        raise ValueError(
            "two nominal static-map conditions have indistinguishable measured servo echo; "
            "inspect command clipping/deadband before fitting a map"
        )
    return x, y


def _safe_median(series: pd.Series) -> float | None:
    values = series.dropna().to_numpy(dtype=float)
    return float(np.median(values)) if len(values) else None


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
    static_cfg = cfg["static_map"]
    wheelbase = float(cfg["hardware"]["wheelbase_m"])
    bias = estimate_imu_bias(session, apply_correction=correction_enabled(cfg))
    rear_x, rear_y = rear_axle_geometry(cfg)
    train = segment_rows(
        session / "04_static_map_training", wheelbase, float(criteria["trim_s"]), criteria, bias,
        rear_axle_x_m=rear_x, rear_axle_y_m=rear_y,
    )
    holdout = segment_rows(
        session / "05_static_map_holdout", wheelbase, float(criteria["trim_s"]), criteria, bias,
        rear_axle_x_m=rear_x, rear_axle_y_m=rear_y,
    )
    if len(train) == 0:
        raise SystemExit("no usable static-map training segments")
    analysis_dir = session / "analysis"
    analysis_dir.mkdir(exist_ok=True)
    (analysis_dir / "imu_bias.json").write_text(json.dumps(bias.to_dict(), indent=2) + "\n", encoding="utf-8")
    train.to_parquet(analysis_dir / "static_map_training_segments.parquet", index=False)
    holdout.to_parquet(analysis_dir / "static_map_holdout_segments.parquet", index=False)

    accepted = add_nominal_condition_keys(train[train.accepted].copy())
    valid = add_nominal_condition_keys(holdout[holdout.accepted].copy())
    min_training = int(criteria["min_training_points"])
    if len(accepted) < min_training:
        raise SystemExit(
            f"static-map training gate failed: {len(accepted)} accepted points < required {min_training}; "
            "inspect LiDAR motion quality and repeat rejected conditions"
        )

    # Coverage is a hard gate. It prevents skipped/failed conditions from being
    # hidden by a global point count or by NaN repeatability/hysteresis metrics.
    train_coverage, train_coverage_failures = _coverage_report(
        accepted,
        fractions=list(static_cfg["training_fractions"]),
        approaches=TRAINING_APPROACHES,
        required_count=int(static_cfg["training_sweep_repetitions"]),
        label="training",
    )
    holdout_coverage, holdout_coverage_failures = _coverage_report(
        valid,
        fractions=list(static_cfg["validation_fractions"]),
        approaches=HOLDOUT_APPROACHES,
        required_count=int(static_cfg["validation_repetitions"]),
        label="holdout",
    )
    coverage = pd.concat([
        train_coverage.assign(dataset="training"),
        holdout_coverage.assign(dataset="holdout"),
    ], ignore_index=True)
    coverage.to_parquet(analysis_dir / "static_map_condition_coverage.parquet", index=False)

    # Repeated condition summaries are keyed by nominal command condition. Echo
    # is deliberately preserved as a measured axis, never used as an exact key.
    repeatability = _repeatability_summary(accepted)
    approaches = _hysteresis_summary(repeatability)
    condition_summary = _condition_summary(accepted)
    repeatability.to_parquet(analysis_dir / "static_map_repeatability.parquet", index=False)
    approaches.to_parquet(analysis_dir / "static_map_hysteresis.parquet", index=False)
    condition_summary.to_parquet(analysis_dir / "static_map_nominal_condition_summary.parquet", index=False)

    hysteresis = approaches["hysteresis_delta_rad"].dropna()
    try:
        x, y = map_interpolate(condition_summary, float(centre["centre_servo_raw"]))
        interpolation_failure: str | None = None
    except ValueError as exc:
        x = np.array([], dtype=float)
        y = np.array([], dtype=float)
        interpolation_failure = str(exc)
    slope = np.gradient(y, x).tolist() if len(x) >= 2 else []

    candidate: dict[str, Any] = {
        "centre_servo_raw": float(centre["centre_servo_raw"]),
        "rear_axle_in_base_link": {"x_m": rear_x, "y_m": rear_y},
        "gyro_z_bias": bias.to_dict(),
        "raw_servo": x.tolist(),
        "delta_eq_rad": y.tolist(),
        "local_gain_rad_per_servo": slope,
        "training_points": int(len(accepted)),
        "training_nominal_conditions": int(len(condition_summary)),
        "hysteresis_median_abs_rad": float(np.median(np.abs(hysteresis))) if len(hysteresis) else None,
        "hysteresis_max_abs_rad": float(np.max(np.abs(hysteresis))) if len(hysteresis) else None,
        "repeatability_median_std_rad": _safe_median(repeatability["repeatability_std_rad"]),
        "interpolation_failure": interpolation_failure,
        "condition_coverage": {
            "training_required_per_condition": int(static_cfg["training_sweep_repetitions"]),
            "holdout_required_per_condition": int(static_cfg["validation_repetitions"]),
            "training_status": "pass" if not train_coverage_failures else "fail",
            "holdout_status": "pass" if not holdout_coverage_failures else "fail",
        },
    }

    if len(valid) and len(x):
        valid["delta_pred_rad"] = np.interp(valid.raw_servo_echo, x, y)
        valid["error_rad"] = valid.delta_pred_rad - valid.delta_eq_rad
        candidate["holdout_points"] = int(len(valid))
        candidate["holdout_rmse_rad"] = float(np.sqrt(np.mean(valid.error_rad ** 2)))
        candidate["holdout_bias_rad"] = float(valid.error_rad.mean())
        valid.to_parquet(analysis_dir / "static_map_holdout_evaluated.parquet", index=False)
    else:
        candidate["holdout_points"] = int(len(valid))
        candidate["holdout_rmse_rad"] = None
        candidate["holdout_bias_rad"] = None
        if len(valid):
            valid.to_parquet(analysis_dir / "static_map_holdout_evaluated.parquet", index=False)

    # Hold-out validation is an acceptance gate, not a report-only diagnostic.
    gates = {
        "min_training_points": min_training,
        "min_holdout_points": int(criteria["min_holdout_points"]),
        "max_holdout_rmse_rad": float(criteria["max_holdout_rmse_rad"]),
        "max_abs_holdout_bias_rad": float(criteria["max_abs_holdout_bias_rad"]),
        "max_hysteresis_median_abs_rad": float(criteria["max_hysteresis_median_abs_rad"]),
        "max_repeatability_median_std_rad": float(criteria["max_repeatability_median_std_rad"]),
        "training_repetitions_per_nominal_condition": int(static_cfg["training_sweep_repetitions"]),
        "holdout_repetitions_per_nominal_condition": int(static_cfg["validation_repetitions"]),
    }
    failures: list[str] = []
    failures.extend(train_coverage_failures)
    failures.extend(holdout_coverage_failures)
    if interpolation_failure is not None:
        failures.append(interpolation_failure)
    holdout_points = int(candidate["holdout_points"])
    if holdout_points < gates["min_holdout_points"]:
        failures.append(f"holdout points {holdout_points} < {gates['min_holdout_points']}")
    rmse = candidate["holdout_rmse_rad"]
    if rmse is None or float(rmse) > gates["max_holdout_rmse_rad"]:
        failures.append(f"holdout RMSE {rmse} exceeds {gates['max_holdout_rmse_rad']:.6f} rad")
    bias = candidate["holdout_bias_rad"]
    if bias is None or abs(float(bias)) > gates["max_abs_holdout_bias_rad"]:
        failures.append(
            f"absolute holdout bias {None if bias is None else abs(float(bias))} "
            f"exceeds {gates['max_abs_holdout_bias_rad']:.6f} rad"
        )
    hysteresis_median = candidate["hysteresis_median_abs_rad"]
    if hysteresis_median is None or float(hysteresis_median) > gates["max_hysteresis_median_abs_rad"]:
        failures.append(f"median hysteresis {hysteresis_median} exceeds {gates['max_hysteresis_median_abs_rad']:.6f} rad")
    repeatability_median = candidate["repeatability_median_std_rad"]
    if repeatability_median is None or float(repeatability_median) > gates["max_repeatability_median_std_rad"]:
        failures.append(
            f"median repeatability standard deviation {repeatability_median} "
            f"exceeds {gates['max_repeatability_median_std_rad']:.6f} rad"
        )

    validation = {
        "status": "pass" if not failures else "fail",
        "gates": gates,
        "failures": failures,
        "accepted_for_deployment": not failures,
    }
    candidate["validation"] = validation
    candidate["accepted_for_deployment"] = not failures
    (analysis_dir / "static_map_validation.json").write_text(json.dumps(validation, indent=2) + "\n")
    (analysis_dir / "candidate_static_steering_map.json").write_text(json.dumps(candidate, indent=2) + "\n")
    print(json.dumps(candidate, indent=2))
    if failures:
        raise SystemExit("static-map hold-out validation failed: " + "; ".join(failures))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
