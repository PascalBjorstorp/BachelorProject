"""Uniform trial-level statistical evidence for every calibration stage."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml


# Prefer one-row-per-trial artefacts. Dense time-series files are deliberately
# absent: overlapping LiDAR windows must not be advertised as independent N.
TABLE_CANDIDATES: dict[str, tuple[str, ...]] = {
    "steering_observability": ("03_sensor_observability/derived/lidar_window_motion.parquet",),
    "steering_centre": ("analysis/centre_trim_points.parquet",),
    "steering_centre_validation": ("analysis/centre_validation_trials.parquet",),
    "steering_static_training": ("analysis/static_map_training_segments.parquet",),
    "steering_static_holdout": ("analysis/static_map_holdout_segments.parquet",),
    "steering_response": ("analysis/command_to_effective_steering_response_metrics.parquet",),
    "steering_response_validation": ("analysis/validation_command_to_effective_steering_response_metrics.parquet",),
    "longitudinal_observability": ("analysis/longitudinal_observability_trials.parquet",),
    "low_speed_launch": ("analysis/low_speed_launch_trials.parquet",),
    "erpm_map_training": ("analysis/erpm_map_training_trials.parquet",),
    "erpm_map_holdout": ("analysis/erpm_map_holdout_trials.parquet",),
    "vel_to_erpm_audit": ("analysis/vel_to_erpm_pipeline_audit_trials.parquet",),
    "erpm_response": ("analysis/erpm_response_trials.parquet",),
    "erpm_response_validation": ("analysis/erpm_response_validation_trials.parquet",),
    "coastdown": ("analysis/coastdown_trials.parquet", "analysis/coastdown_samples.parquet"),
    "coastdown_validation": (
        "analysis/coastdown_validation_trials.parquet", "analysis/coastdown_validation_samples.parquet",
    ),
    "current_training": ("analysis/current_model_training_trials.parquet",),
    "current_holdout": ("analysis/current_model_holdout_trials.parquet",),
    "accel_interface": ("analysis/accel_to_current_interface_trials.parquet",),
    "accel_interface_validation": ("analysis/accel_to_current_interface_validation_trials.parquet",),
    "odometry_candidate_velocity_validation": (
        "analysis/odometry_candidate_velocity_trials.parquet",
        "analysis/candidate_velocity_verification_trials.parquet",
    ),
    "odometry_candidate_accel_validation": (
        "analysis/candidate_accel_verification_trials.parquet",
        "analysis/candidate_dynamic_speed_samples.parquet",
    ),
    "lateral_stiffness_training": ("analysis/lateral_stiffness_training_trials.parquet",),
    "lateral_stiffness_validation": ("analysis/lateral_stiffness_validation_trials.parquet",),
}

RUNTIME_SAMPLE_STAGES = {"steering_command_audit", "motor_command_audit"}
DIRECT_STAGES = {"physical_metrology", "steering_endstops"}

ACCEPTANCE_COLUMNS = (
    "measurement_valid", "accepted_for_validation", "accepted",
    "effective_response_valid", "valid",
)

HEADLINE_COLUMNS: dict[str, tuple[str, ...]] = {
    "steering_observability": ("icp_rmse_m", "vx", "yaw_rate_icp"),
    "steering_centre": ("yaw_rate_icp_rad_s", "raw_servo_echo"),
    "steering_centre_validation": ("lidar_vy_mps", "yaw_rate_icp_rad_s"),
    "steering_static_training": ("delta_eq_rad",),
    "steering_static_holdout": ("delta_eq_rad",),
    "steering_response": ("fopdt_tau_s", "effective_delay_10pct_s"),
    "steering_response_validation": ("fopdt_tau_s", "effective_delay_10pct_s"),
    "longitudinal_observability": ("vx_lidar_mps", "lidar_valid_fraction"),
    "low_speed_launch": ("vx_lidar_mps",),
    "erpm_map_training": ("vx_lidar_mps", "erpm_measured"),
    "erpm_map_holdout": ("vx_lidar_mps", "erpm_measured"),
    "vel_to_erpm_audit": ("vx_lidar_mps",),
    "erpm_response": ("ground_speed_tau_s", "ground_speed_delay_10pct_s"),
    "erpm_response_validation": ("ground_speed_tau_s", "ground_speed_delay_10pct_s"),
    "coastdown": ("initial_speed_mps",),
    "coastdown_validation": ("initial_speed_mps",),
    "current_training": ("net_accel_mps2",),
    "current_holdout": ("net_accel_mps2",),
    "accel_interface": ("ground_accel_residual_mps2",),
    "accel_interface_validation": ("ground_accel_residual_mps2",),
    "odometry_candidate_velocity_validation": ("candidate_odom_vx_mps", "vx_lidar_mps"),
    "odometry_candidate_accel_validation": ("candidate_odom_accel_mps2", "lidar_accel_mps2"),
    "lateral_stiffness_training": ("fy_front_N", "fy_rear_N", "turn_slip_fraction"),
    "lateral_stiffness_validation": ("fy_front_N", "fy_rear_N", "turn_slip_fraction"),
}

# Used only when an analysis table does not carry its canonical condition_id.
# This keeps repetition counts useful for old sessions and hand-built fixtures
# without treating a trial identifier itself as a repeated test condition.
CONDITION_FIELDS: dict[str, tuple[str, ...]] = {
    "steering_command_audit": ("label",),
    "steering_observability": ("phase",),
    "steering_centre": ("raw_servo_target",),
    "steering_centre_validation": ("validation_lane_direction", "validation_speed_mps"),
    "steering_static_training": ("side", "fraction", "sweep_direction"),
    "steering_static_holdout": ("side", "fraction", "sweep_direction"),
    "steering_response": ("speed_mps", "side", "target_fraction"),
    "steering_response_validation": ("speed_mps", "side", "target_fraction"),
    "motor_command_audit": ("label",),
    "longitudinal_observability": ("speed_command_mps",),
    "low_speed_launch": ("nominal_speed_mps",),
    "erpm_map_training": ("nominal_speed_mps",),
    "erpm_map_holdout": ("nominal_speed_mps",),
    "vel_to_erpm_audit": ("speed_command_mps",),
    "erpm_response": ("baseline_speed_mps", "target_speed_mps"),
    "erpm_response_validation": ("baseline_speed_mps", "target_speed_mps"),
    "coastdown": ("initial_speed_mps",),
    "coastdown_validation": ("initial_speed_mps",),
    "current_training": ("polarity", "initial_speed_mps", "current_fraction"),
    "current_holdout": ("polarity", "initial_speed_mps", "current_fraction"),
    "accel_interface": ("initial_speed_mps", "acceleration_command_mps2"),
    "accel_interface_validation": ("initial_speed_mps", "acceleration_command_mps2"),
    "odometry_candidate_velocity_validation": ("speed_command_mps",),
    "odometry_candidate_accel_validation": ("initial_speed_mps", "acceleration_command_mps2"),
    "lateral_stiffness_training": ("speed_command_mps", "steering_angle_rad"),
    "lateral_stiffness_validation": ("speed_command_mps", "steering_angle_rad"),
}

METRIC_TOKENS = (
    "rmse", "mae", "r2", "bias", "std", "95pct", "confidence",
    "condition_number", "p95", "median", "coverage", "trials", "samples", "points",
)


def _plain(value: Any) -> Any:
    if isinstance(value, (np.integer, np.floating, np.bool_)):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {str(key): _plain(child) for key, child in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(child) for child in value]
    return value


def _reported_metrics(value: Any, prefix: str = "") -> dict[str, Any]:
    output: dict[str, Any] = {}
    if isinstance(value, dict):
        for key, child in value.items():
            path = f"{prefix}.{key}" if prefix else str(key)
            if isinstance(child, dict):
                output.update(_reported_metrics(child, path))
            elif isinstance(child, (list, tuple)):
                if any(token in str(key).lower() for token in METRIC_TOKENS):
                    output[path] = _plain(child)
            elif isinstance(child, (int, float, np.integer, np.floating)) and not isinstance(child, bool):
                if any(token in str(key).lower() for token in METRIC_TOKENS):
                    output[path] = _plain(child)
    return output


def _load_table(session: Path, stage: Any) -> tuple[pd.DataFrame, str | None]:
    for relative in TABLE_CANDIDATES.get(stage.key, ()):
        path = session / relative
        if path.is_file():
            return pd.read_parquet(path), relative
    runtime = session / stage.directory / "runtime_result.json"
    if runtime.is_file():
        try:
            value = json.loads(runtime.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            value = {}
        samples = value.get("samples", []) if isinstance(value, dict) else []
        if isinstance(samples, list) and samples:
            return pd.DataFrame(samples), str(runtime.relative_to(session))
    return pd.DataFrame(), None


def _accepted(frame: pd.DataFrame) -> tuple[pd.DataFrame, str | None]:
    for name in ACCEPTANCE_COLUMNS:
        if name in frame:
            values = frame[name]
            if pd.api.types.is_bool_dtype(values) or set(values.dropna().unique()).issubset({0, 1, True, False}):
                return frame[values.fillna(False).astype(bool)].copy(), name
    return frame.copy(), None


def _independent_numeric(frame: pd.DataFrame, column: str) -> np.ndarray:
    values = pd.to_numeric(frame[column], errors="coerce")
    if "trial_id" in frame and frame.trial_id.notna().any():
        grouped = pd.DataFrame({"trial_id": frame.trial_id.astype(str), "value": values})
        values = grouped.groupby("trial_id", sort=False).value.median()
    result = values.to_numpy(dtype=float)
    return result[np.isfinite(result)]


def _bootstrap_median_interval(values: np.ndarray, seed: int, resamples: int = 1000) -> list[float]:
    if not len(values):
        return []
    if len(values) == 1:
        return [float(values[0]), float(values[0])]
    rng = np.random.default_rng(seed)
    draws = rng.choice(values, size=(resamples, len(values)), replace=True)
    medians = np.median(draws, axis=1)
    return [float(item) for item in np.quantile(medians, [0.025, 0.975])]


def _column_summary(frame: pd.DataFrame, column: str, stage_key: str) -> dict[str, Any]:
    values = _independent_numeric(frame, column)
    digest = hashlib.sha256(f"{stage_key}:{column}".encode("utf-8")).digest()
    seed = int.from_bytes(digest[:4], "little")
    return {
        "independent_n": int(len(values)),
        "mean": float(np.mean(values)) if len(values) else None,
        "sample_std": float(np.std(values, ddof=1)) if len(values) > 1 else None,
        "median": float(np.median(values)) if len(values) else None,
        "p05": float(np.quantile(values, 0.05)) if len(values) else None,
        "p95": float(np.quantile(values, 0.95)) if len(values) else None,
        "bootstrap_median_95pct": _bootstrap_median_interval(values, seed),
    }


def _condition_keys(frame: pd.DataFrame, stage_key: str) -> pd.Series | None:
    if "condition_id" in frame:
        values = frame.condition_id.fillna("").astype(str).str.strip()
        if values.ne("").any():
            return values
    fields = CONDITION_FIELDS.get(stage_key, ())
    if not fields or any(field not in frame for field in fields):
        return None
    columns = []
    for field in fields:
        values = frame[field]
        if pd.api.types.is_numeric_dtype(values):
            columns.append(pd.to_numeric(values, errors="coerce").round(8).astype(str))
        else:
            columns.append(values.fillna("").astype(str).str.strip())
    keys = columns[0]
    for values in columns[1:]:
        keys = keys.str.cat(values, sep="|")
    return keys


def summarize_stage_statistics(session: Path, stage: Any, analysis: dict[str, Any]) -> dict[str, Any]:
    """Write full statistics and return a compact manifest/GUI summary."""
    session = Path(session)
    frame, source = _load_table(session, stage)
    usable, acceptance_column = _accepted(frame)
    direct = stage.key in DIRECT_STAGES
    independent_units = (
        int(usable.trial_id.astype(str).nunique())
        if "trial_id" in usable and usable.trial_id.notna().any() else int(len(usable))
    )
    if direct and independent_units == 0 and analysis:
        independent_units = 1
    condition_keys = _condition_keys(usable, stage.key)
    condition_count = int(condition_keys.nunique()) if condition_keys is not None and len(condition_keys) else None
    repetitions: dict[str, Any] = {}
    if condition_keys is not None and len(usable):
        units = usable.trial_id.astype(str) if "trial_id" in usable else usable.index.astype(str)
        counts = pd.DataFrame({"condition": condition_keys, "unit": units}).groupby("condition").unit.nunique().to_numpy(dtype=int)
        if len(counts):
            repetitions = {
                "minimum_per_condition": int(counts.min()),
                "median_per_condition": float(np.median(counts)),
                "maximum_per_condition": int(counts.max()),
            }

    numeric: dict[str, Any] = {}
    if not direct and len(usable):
        for column in usable.select_dtypes(include=[np.number]).columns:
            if column in {"bag_ns", "start_ns", "end_ns", "trial_index"}:
                continue
            numeric[str(column)] = _column_summary(usable, str(column), stage.key)

    headline_column = next(
        (name for name in HEADLINE_COLUMNS.get(stage.key, ()) if name in numeric),
        next(iter(numeric), None),
    )
    headline = {"field": headline_column, **numeric[headline_column]} if headline_column else None
    metrics = _reported_metrics(analysis)
    evidence = {
        "stage": stage.key,
        "source_table": source,
        "sampling_note": (
            "Direct/manual value; repeated-sample standard deviation and confidence interval are not claimed."
            if direct else
            "Statistics use one aggregate per trial when trial_id is available; overlapping LiDAR windows are not counted as independent samples."
        ),
        "total_rows": int(len(frame)),
        "accepted_rows": int(len(usable)),
        "rejected_rows": int(len(frame) - len(usable)),
        "acceptance_column": acceptance_column,
        "independent_units": independent_units,
        "condition_count": condition_count,
        "repetitions": repetitions,
        "headline_distribution": headline,
        "numeric_distributions": numeric,
        "reported_fit_and_validation_accuracy": metrics,
    }
    output = session / "analysis" / "statistics" / f"{stage.key}.yaml"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(_plain(evidence), sort_keys=False), encoding="utf-8")
    return {
        "artifact": str(output.relative_to(session)),
        "total_rows": evidence["total_rows"],
        "accepted_rows": evidence["accepted_rows"],
        "independent_units": independent_units,
        "condition_count": condition_count,
        **repetitions,
        "headline_distribution": headline,
        "reported_accuracy_metric_count": len(metrics),
        "sampling": "direct" if direct else "trial_level",
    }


def statistics_footer(session: Path, stage_key: str) -> str | None:
    """Return one compact line suitable for a plot footer."""
    path = Path(session) / "analysis" / "statistics" / f"{stage_key}.yaml"
    if not path.is_file():
        return None
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return None
    if "Direct/manual" in str(value.get("sampling_note", "")):
        return "Direct/manual measurement — no repeated-sample confidence interval claimed"
    base = (
        f"accepted {value.get('accepted_rows', 0)}/{value.get('total_rows', 0)}; "
        f"independent trials {value.get('independent_units', 0)}"
    )
    if value.get("condition_count") is not None:
        base += f"; conditions {value['condition_count']}"
    headline = value.get("headline_distribution")
    if isinstance(headline, dict) and headline.get("field") and headline.get("median") is not None:
        interval = headline.get("bootstrap_median_95pct") or []
        base += f"; {headline['field']} median={headline['median']:.4g}"
        if len(interval) == 2:
            base += f" (bootstrap 95% [{interval[0]:.4g}, {interval[1]:.4g}])"
    return base
