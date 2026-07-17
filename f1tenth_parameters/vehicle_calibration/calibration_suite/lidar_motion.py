#!/usr/bin/env python3
"""Displacement-targeted LiDAR motion for the unified calibration campaign.

The legacy estimators compared consecutive scans.  That makes forward motion
effectively unobservable at the low speeds used indoors: at 0.6 m/s and 40 Hz
the vehicle moves only 1.5 cm between scans.  This adapter reuses the proven
point-to-line ICP implementation, but pairs a scan with an older scan once the
expected displacement is roughly 12 cm.  It writes the normal
``derived/lidar_velocity.parquet`` schema so every existing analysis script
uses the more observable measurement without a hidden alternate data path.

No odometry position, velocity, or pose is fed into ICP.  IMU yaw is used only
as a rotational seed, exactly as in the legacy estimator.
"""
from __future__ import annotations

import argparse
import importlib
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
STEERING_ROOT = ROOT.parent / "steering"
ERPM_ROOT = ROOT.parent / "ERPM"


def _load_legacy(kind: str):
    analysis_root = (STEERING_ROOT if kind == "steering" else ERPM_ROOT) / "analysis"
    sys.path.insert(0, str(analysis_root))
    # Every invocation is a new process, so importing this generic name cannot
    # collide with the other suite's estimator.
    return importlib.import_module("estimate_lidar_motion"), analysis_root


def _windows(kind: str, derived: Path, estimator: Any) -> list[dict[str, Any]]:
    """Return accepted capture windows, never allowing a pair across a move."""
    if kind == "steering":
        return estimator.capture_windows(derived)

    sys.path.insert(0, str(ERPM_ROOT / "analysis"))
    from common import accepted_capture_windows  # pylint: disable=import-outside-toplevel
    import pandas as pd  # pylint: disable=import-outside-toplevel

    events_path = derived / "events.parquet"
    if not events_path.exists():
        return []
    events = pd.read_parquet(events_path)
    frame = accepted_capture_windows(events)
    if frame.empty:
        return []
    result: list[dict[str, Any]] = []
    for row in frame.to_dict(orient="records"):
        # Retain only the declared experiment-speed metadata used to choose a
        # scan baseline.  It is not odometry and never becomes an ICP seed or
        # a fitted motion observation.
        record: dict[str, Any] = {
            "start_ns": int(row["start_ns"]),
            "end_ns": int(row["end_ns"]),
            "trial_id": str(row.get("trial_id", "")),
            "phase": None if row.get("phase") is None else str(row.get("phase")),
            "segment_id": None if row.get("segment_id") is None else str(row.get("segment_id")),
        }
        for key in (
            "target_speed_mps", "nominal_speed_mps", "speed_mps",
            "speed_command_mps", "initial_speed_mps", "baseline_speed_mps",
            "speed_hint_mps",
        ):
            value = row.get(key)
            if value is not None and not pd.isna(value):
                record[key] = value
        result.append(record)
    return sorted(result, key=lambda item: item["start_ns"])


def _window_for(t_ns: int, windows: list[dict[str, Any]], index: int) -> tuple[dict[str, Any] | None, int]:
    while index < len(windows) and t_ns > int(windows[index]["end_ns"]):
        index += 1
    if index < len(windows) and int(windows[index]["start_ns"]) <= t_ns <= int(windows[index]["end_ns"]):
        return windows[index], index
    return None, index


def _window_speed_hint(window: dict[str, Any] | None) -> float | None:
    """Use the declared test condition only to choose a safe scan baseline.

    It is not an ICP translation seed and never enters a fitted measurement.
    Without it, the first 3 m/s pair would be selected using the low-speed
    floor, creating an unnecessarily long baseline before ICP has a velocity
    estimate of its own.
    """
    if window is None:
        return None
    for key in (
        "target_speed_mps", "nominal_speed_mps", "speed_mps", "speed_command_mps",
        "initial_speed_mps", "baseline_speed_mps", "speed_hint_mps",
    ):
        try:
            value = abs(float(window.get(key)))
        except (TypeError, ValueError):
            continue
        if math.isfinite(value) and value > 0.0:
            return value
    return None


def _integrated_yaw(imu_t_ns: np.ndarray, imu_gz: np.ndarray, start_ns: int, end_ns: int) -> float:
    """Integrate raw gyro yaw over a matching baseline, with a safe fallback."""
    mask = (imu_t_ns >= start_ns) & (imu_t_ns <= end_ns) & np.isfinite(imu_gz)
    if int(mask.sum()) >= 2:
        time_s = imu_t_ns[mask].astype(float) * 1.0e-9
        gyro = imu_gz[mask].astype(float)
        integrate = getattr(np, "trapezoid", np.trapz)
        return float(integrate(gyro, time_s))
    midpoint = 0.5 * (start_ns + end_ns)
    rate = float(np.interp(midpoint, imu_t_ns, imu_gz, left=np.nan, right=np.nan))
    return rate * (end_ns - start_ns) * 1.0e-9 if math.isfinite(rate) else math.nan


def _baseline_settings(icp: dict[str, Any]) -> dict[str, float | str]:
    """Read an explicit campaign baseline, with safe defaults for old snapshots."""
    raw = icp.get("displacement_baseline", {}) or {}
    mode = str(raw.get("mode", icp.get("pairing_mode", "displacement_targeted"))).lower()
    values: dict[str, float | str] = {
        "mode": mode,
        "target_displacement_m": float(raw.get("target_displacement_m", 0.12)),
        "min_baseline_s": float(raw.get("min_baseline_s", 0.04)),
        "max_baseline_s": float(raw.get("max_baseline_s", 0.32)),
        "predicted_speed_floor_mps": float(raw.get("predicted_speed_floor_mps", 0.30)),
    }
    if values["mode"] not in {"displacement_targeted", "consecutive"}:
        raise ValueError(f"unsupported LiDAR pairing mode: {values['mode']}")
    if not (0.0 < float(values["min_baseline_s"]) < float(values["max_baseline_s"])):
        raise ValueError("LiDAR baseline requires 0 < min_baseline_s < max_baseline_s")
    if float(values["target_displacement_m"]) <= 0.0:
        raise ValueError("LiDAR target_displacement_m must be positive")
    return values


def select_baseline_index(
    timestamps_ns: list[int],
    *,
    predicted_speed_mps: float,
    target_displacement_m: float,
    min_baseline_s: float,
    max_baseline_s: float,
    predicted_speed_floor_mps: float,
) -> int | None:
    """Choose the newest old scan that reaches the displacement target.

    If a slow/stationary vehicle cannot reach the target before the maximum
    baseline, use the oldest admissible scan.  This preserves a useful noise
    measurement instead of claiming a consecutive pair is informative.
    """
    if len(timestamps_ns) < 2:
        return None
    current = int(timestamps_ns[-1])
    fallback: int | None = None
    speed = max(abs(float(predicted_speed_mps)), float(predicted_speed_floor_mps))
    for index in range(len(timestamps_ns) - 2, -1, -1):
        dt_s = (current - int(timestamps_ns[index])) * 1.0e-9
        if dt_s < min_baseline_s:
            continue
        if dt_s > max_baseline_s:
            break
        fallback = index
        if speed * dt_s >= target_displacement_m:
            return index
    return fallback


def _base_transform(result: Any, hardware: dict[str, Any]) -> tuple[np.ndarray, np.ndarray, float]:
    laser_offset_base = np.array([
        float(hardware.get("laser_to_base_x_m", 0.0)),
        float(hardware.get("laser_to_base_y_m", 0.0)),
    ])
    laser_yaw = float(hardware.get("laser_to_base_yaw_rad", 0.0))
    cosine, sine = math.cos(laser_yaw), math.sin(laser_yaw)
    r_base_laser = np.array([[cosine, -sine], [sine, cosine]])
    r_base = r_base_laser @ result.R @ r_base_laser.T
    delta_base = r_base_laser @ result.t + (np.eye(2) - r_base) @ laser_offset_base
    delta_yaw = math.atan2(float(r_base[1, 0]), float(r_base[0, 0]))
    return r_base, delta_base, delta_yaw


def _quality_reasons(result: Any, yaw_seed_residual: float, icp: dict[str, Any]) -> list[str]:
    reasons: list[str] = []
    if not result.converged:
        reasons.append("solver_not_converged")
    if result.correspondence_count < int(icp["min_correspondences"]):
        reasons.append("too_few_correspondences")
    if result.inlier_ratio < float(icp["min_inlier_ratio"]):
        reasons.append("low_inlier_ratio")
    if result.rmse_m > float(icp["max_pair_rmse_m"]):
        reasons.append("high_point_to_line_rmse")
    if result.condition_number > float(icp["max_hessian_condition_number"]):
        reasons.append("poor_geometry_conditioning")
    if abs(yaw_seed_residual) > float(icp["max_imu_yaw_residual_rad"]):
        reasons.append("imu_yaw_inconsistent")
    if math.hypot(result.dx_std_m, result.dy_std_m) > float(icp.get("max_pair_translation_std_m", math.inf)):
        reasons.append("high_translation_uncertainty")
    if result.yaw_std_rad > float(icp.get("max_pair_yaw_std_rad", math.inf)):
        reasons.append("high_yaw_uncertainty")
    return reasons


def _window_aggregation_settings(icp: dict[str, Any]) -> dict[str, float | bool]:
    """Return the robust motion-window policy used by downstream fitters.

    ICP is still the geometric registration primitive, but no fitted vehicle
    parameter is allowed to treat one scan-pair registration as an independent
    velocity observation.  A 40 Hz lidar produces highly correlated pair
    estimates, and a single pair is dominated by range/feature noise at the
    low speeds used in the room.  The analysis product below therefore combines
    a short, explicitly recorded time window of accepted registrations before
    any steering or longitudinal fitter consumes it.
    """
    raw = icp.get("window_aggregation", {}) or {}
    settings: dict[str, float | bool] = {
        "enabled": bool(raw.get("enabled", True)),
        "window_s": float(raw.get("window_s", 0.50)),
        "stride_s": float(raw.get("stride_s", 0.10)),
        "minimum_window_s": float(raw.get("minimum_window_s", 0.35)),
        "min_valid_pairs": float(raw.get("min_valid_pairs", 5)),
        "min_valid_fraction": float(raw.get("min_valid_fraction", 0.65)),
    }
    if not settings["enabled"]:
        return settings
    if not (0.05 <= float(settings["minimum_window_s"]) <= float(settings["window_s"])):
        raise ValueError("LiDAR window aggregation requires 0.05 <= minimum_window_s <= window_s")
    if not (0.02 <= float(settings["stride_s"]) <= float(settings["window_s"])):
        raise ValueError("LiDAR window aggregation requires 0.02 <= stride_s <= window_s")
    if int(settings["min_valid_pairs"]) < 2:
        raise ValueError("LiDAR window aggregation requires at least two valid registrations")
    if not 0.0 < float(settings["min_valid_fraction"]) <= 1.0:
        raise ValueError("LiDAR window aggregation min_valid_fraction must lie in (0, 1]")
    return settings


def _robust_mad_std(values: np.ndarray) -> float:
    finite = np.asarray(values, dtype=float)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return math.nan
    median = float(np.median(finite))
    return float(1.4826 * np.median(np.abs(finite - median)))


def aggregate_motion_windows(table: Any, windows: list[dict[str, Any]],
                             settings: dict[str, float | bool]) -> Any:
    """Aggregate quality-gated scan registrations into robust time windows.

    The resulting rows are deliberately *not* a higher-rate ground-truth
    stream.  They are robust, overlapping local summaries intended for steady
    fits and low-bandwidth response/acceleration analysis.  Raw scan-pair rows
    remain in ``lidar_velocity.parquet`` for diagnostics and reproducibility.
    """
    import pandas as pd

    columns = [
        "bag_ns", "window_start_ns", "window_end_ns", "window_duration_s", "window_index",
        "trial_id", "phase", "segment_id", "raw_pair_count", "valid_pair_count",
        "valid_pair_fraction", "vx", "vy", "yaw_rate_icp", "icp_rmse_m",
        "hessian_condition_number", "yaw_seed_residual_rad", "vx_mad_std_mps",
        "vy_mad_std_mps", "yaw_rate_mad_std_rad_s", "valid", "reason",
    ]
    if table is None or len(table) == 0 or not windows or not bool(settings["enabled"]):
        return pd.DataFrame(columns=columns)

    duration_ns = int(round(float(settings["window_s"]) * 1.0e9))
    stride_ns = int(round(float(settings["stride_s"]) * 1.0e9))
    minimum_ns = int(round(float(settings["minimum_window_s"]) * 1.0e9))
    min_pairs = int(settings["min_valid_pairs"])
    min_fraction = float(settings["min_valid_fraction"])
    rows: list[dict[str, Any]] = []

    for capture in windows:
        start_ns = int(capture["start_ns"])
        end_ns = int(capture["end_ns"])
        if end_ns - start_ns < minimum_ns:
            continue
        left_ns = start_ns
        index = 0
        while left_ns + minimum_ns <= end_ns:
            right_ns = min(end_ns, left_ns + duration_ns)
            raw = table[(table.bag_ns >= left_ns) & (table.bag_ns <= right_ns)].copy()
            valid = raw[raw.valid.astype(bool)].copy() if len(raw) and "valid" in raw else raw.iloc[0:0]
            raw_count = int(len(raw))
            valid_count = int(len(valid))
            valid_fraction = float(valid_count / raw_count) if raw_count else 0.0
            enough = valid_count >= min_pairs and valid_fraction >= min_fraction
            record: dict[str, Any] = {
                "bag_ns": int((left_ns + right_ns) // 2),
                "window_start_ns": left_ns,
                "window_end_ns": right_ns,
                "window_duration_s": float((right_ns - left_ns) * 1.0e-9),
                "window_index": index,
                "trial_id": str(capture.get("trial_id", "")),
                "phase": capture.get("phase"),
                "segment_id": capture.get("segment_id"),
                "raw_pair_count": raw_count,
                "valid_pair_count": valid_count,
                "valid_pair_fraction": valid_fraction,
                "valid": bool(enough),
                "reason": "" if enough else "insufficient_valid_registrations_in_window",
            }
            for field in ("vx", "vy", "yaw_rate_icp", "icp_rmse_m", "hessian_condition_number", "yaw_seed_residual_rad"):
                values = valid[field].to_numpy(dtype=float) if field in valid else np.empty(0, dtype=float)
                record[field] = float(np.nanmedian(values)) if len(values) and np.isfinite(values).any() else math.nan
            record["vx_mad_std_mps"] = _robust_mad_std(valid.vx.to_numpy(dtype=float)) if "vx" in valid else math.nan
            record["vy_mad_std_mps"] = _robust_mad_std(valid.vy.to_numpy(dtype=float)) if "vy" in valid else math.nan
            record["yaw_rate_mad_std_rad_s"] = (
                _robust_mad_std(valid.yaw_rate_icp.to_numpy(dtype=float))
                if "yaw_rate_icp" in valid else math.nan
            )
            rows.append(record)
            index += 1
            left_ns += stride_ns
    return pd.DataFrame(rows, columns=columns)


def estimate(bag_dir: Path, config_path: Path, kind: str, *, output_derived: Path | None = None) -> Path:
    estimator, _ = _load_legacy(kind)
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    icp = cfg["analysis"]["icp"]
    hardware = cfg["hardware"]
    baseline = _baseline_settings(icp)
    derived = bag_dir.parent / "derived"
    output_derived = output_derived or derived
    output_derived.mkdir(parents=True, exist_ok=True)
    imu_t, imu_gz = estimator.load_imu(derived)
    windows = _windows(kind, derived, estimator)

    try:
        import pandas as pd
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise SystemExit("Source the ROS workspace and install offline dependencies: " + repr(exc)) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if "/scan" not in types:
        raise SystemExit("bag contains no /scan topic")
    scan_type = get_message(types["/scan"])

    deskew_enabled = bool(icp.get("motion_deskew", True))
    seed_enabled = bool(icp.get("constant_velocity_seed", True))
    max_scan_period_s = float(icp.get("max_scan_period_s", cfg.get("analysis", {}).get("max_lidar_gap_s", 0.20)))
    buffer: list[dict[str, Any]] = []
    last_v_laser = np.zeros(2, dtype=float)
    last_omega = 0.0
    predicted_speed = float(baseline["predicted_speed_floor_mps"])
    rows: list[dict[str, Any]] = []
    window_index = 0
    active_window_key: tuple[Any, ...] | None = None

    while reader.has_next():
        topic, raw, bag_ns = reader.read_next()
        if topic != "/scan":
            continue
        scan = deserialize_message(raw, scan_type)
        timestamp_ns = estimator.stamp_ns(scan, bag_ns)
        window, window_index = _window_for(timestamp_ns, windows, window_index)
        if windows and window is None:
            buffer.clear()
            active_window_key = None
            continue
        window_key = None if window is None else (window["trial_id"], window["phase"], window["segment_id"])
        if window_key != active_window_key:
            buffer.clear()
            last_v_laser = np.zeros(2, dtype=float)
            last_omega = 0.0
            predicted_speed = float(baseline["predicted_speed_floor_mps"])
            active_window_key = window_key

        points, rel_t = estimator.scan_points(scan, int(icp["downsample"]))
        if hasattr(estimator, "filter_self_points"):
            points, rel_t = estimator.filter_self_points(points, rel_t, hardware)
        omega_now = float(np.interp(timestamp_ns, imu_t, imu_gz, left=np.nan, right=np.nan))
        if not math.isfinite(omega_now):
            omega_now = last_omega
        if deskew_enabled:
            points = estimator.deskew_scan(points, rel_t, last_v_laser, omega_now)
        buffer.append({"timestamp_ns": timestamp_ns, "points": points})

        if str(baseline["mode"]) == "consecutive":
            while len(buffer) > 2:
                buffer.pop(0)
            target_index = 0 if len(buffer) == 2 else None
        else:
            max_ns = int(float(baseline["max_baseline_s"]) * 1.0e9)
            while len(buffer) > 1 and timestamp_ns - int(buffer[0]["timestamp_ns"]) > max_ns:
                buffer.pop(0)
            speed_hint = _window_speed_hint(window)
            target_index = select_baseline_index(
                [int(item["timestamp_ns"]) for item in buffer],
                predicted_speed_mps=max(predicted_speed, speed_hint or 0.0),
                target_displacement_m=float(baseline["target_displacement_m"]),
                min_baseline_s=float(baseline["min_baseline_s"]),
                max_baseline_s=float(baseline["max_baseline_s"]),
                predicted_speed_floor_mps=float(baseline["predicted_speed_floor_mps"]),
            )
        if target_index is None:
            continue

        target = buffer[target_index]
        source = buffer[-1]
        target_ns = int(target["timestamp_ns"])
        dt_s = (timestamp_ns - target_ns) * 1.0e-9
        metadata = {
            "bag_ns": int(timestamp_ns),
            "previous_bag_ns": target_ns,
            "dt_s": float(dt_s),
            "baseline_frames": int(len(buffer) - 1 - target_index),
            "baseline_dt_s": float(dt_s),
            "trial_id": None if window is None else window["trial_id"],
            "phase": None if window is None else window["phase"],
            "segment_id": None if window is None else window["segment_id"],
            "baseline_speed_hint_mps": _window_speed_hint(window),
        }
        minimum_dt = 0.005 if str(baseline["mode"]) == "consecutive" else float(baseline["min_baseline_s"])
        maximum_dt = max_scan_period_s if str(baseline["mode"]) == "consecutive" else float(baseline["max_baseline_s"])
        if not (minimum_dt <= dt_s <= maximum_dt):
            rows.append({**metadata, "valid": False, "reason": "scan_period_out_of_range"})
            continue
        yaw_seed = _integrated_yaw(imu_t, imu_gz, target_ns, timestamp_ns)
        if not math.isfinite(yaw_seed):
            rows.append({**metadata, "valid": False, "reason": "missing_imu_yaw_seed"})
            continue
        translation_seed = last_v_laser * dt_s if seed_enabled else None
        try:
            normals = estimator.pca_normals(target["points"], int(icp["normal_neighbors"]))
            result = estimator.robust_point_to_line_icp(
                source["points"], target["points"], normals,
                yaw_seed=yaw_seed,
                max_iter=int(icp["max_iterations"]),
                max_corr=float(icp["max_correspondence_m"]),
                trim_fraction=float(icp["trim_fraction"]),
                min_corr=int(icp["min_correspondences"]),
                translation_tolerance_m=float(icp["translation_update_tolerance_m"]),
                rotation_tolerance_rad=float(icp["rotation_update_tolerance_rad"]),
                relative_cost_tolerance=float(icp["relative_cost_tolerance"]),
                translation_seed=translation_seed,
                huber_delta_scale=float(icp.get("huber_delta_scale", 2.5)),
                range_weight_reference_m=float(icp["range_weight_reference_m"]) if icp.get("range_weight_reference_m") is not None else None,
                minimum_range_weight=float(icp.get("minimum_range_weight", 0.20)),
            )
        except Exception as exc:  # the rejected pair remains visible in Parquet
            rows.append({**metadata, "valid": False, "reason": repr(exc)})
            continue
        _, delta_base, delta_yaw = _base_transform(result, hardware)
        yaw_seed_residual = estimator.wrap_angle(delta_yaw - yaw_seed)
        reasons = _quality_reasons(result, yaw_seed_residual, icp)
        valid = not reasons
        if valid:
            last_v_laser = result.t / dt_s
            last_omega = math.atan2(float(result.R[1, 0]), float(result.R[0, 0])) / dt_s
            predicted_speed = float(np.linalg.norm(last_v_laser))
        rows.append({
            **metadata,
            "vx": float(delta_base[0] / dt_s),
            "vy": float(delta_base[1] / dt_s),
            "yaw_rate_icp": float(delta_yaw / dt_s),
            "yaw_rate_imu_seed": float(yaw_seed / dt_s),
            "yaw_seed_residual_rad": float(yaw_seed_residual),
            "icp_rmse_m": result.rmse_m,
            "correspondences": result.correspondence_count,
            "inlier_ratio": result.inlier_ratio,
            "iterations": result.iterations,
            "solver_converged": result.converged,
            "final_cost": result.final_cost,
            "hessian_condition_number": result.condition_number,
            "yaw_std_rad": result.yaw_std_rad,
            "dx_std_m": result.dx_std_m,
            "dy_std_m": result.dy_std_m,
            "valid": valid,
            "reason": ";".join(reasons),
        })

    table = pd.DataFrame(rows)
    output = output_derived / "lidar_velocity.parquet"
    table.to_parquet(output, index=False)
    aggregation = _window_aggregation_settings(icp)
    window_table = aggregate_motion_windows(table, windows, aggregation)
    window_output = output_derived / "lidar_window_motion.parquet"
    window_table.to_parquet(window_output, index=False)
    summary = {
        "output": str(output),
        "window_output": str(window_output),
        "scan_pairs": int(len(table)),
        "valid_pairs": int(table["valid"].sum()) if len(table) and "valid" in table else 0,
        "motion_measurement": (
            "IMU-yaw-seeded, odometry-unseeded point-to-line registrations over displacement-targeted "
            "scan baselines; all parameter fitters consume robust multi-registration time windows, not individual pairs"
        ),
        "pairing": baseline,
        "window_aggregation": {
            **aggregation,
            "windows_written": int(len(window_table)),
            "valid_windows": int(window_table["valid"].sum()) if len(window_table) else 0,
        },
        "capture_window_filter": {
            "enabled": bool(windows),
            "windows": len(windows),
            "scope": "accepted trial capture windows only; pairs never cross operator moves",
        },
        "motion_deskew_enabled": deskew_enabled,
        "constant_velocity_translation_seed_enabled": seed_enabled,
        "robust_weighting": {
            "huber_delta_scale": float(icp.get("huber_delta_scale", 2.5)),
            "range_weight_reference_m": icp.get("range_weight_reference_m"),
            "minimum_range_weight": float(icp.get("minimum_range_weight", 0.20)),
            "wls_convention": "square-root weights in solve; identical weights in covariance and cost",
        },
        "pair_uncertainty_gates": {
            "max_translation_std_m": icp.get("max_pair_translation_std_m"),
            "max_yaw_std_rad": icp.get("max_pair_yaw_std_rad"),
        },
        "self_scan_filter": hardware.get("self_scan_filter", {}) or {},
        "laser_to_base_geometry_used": {
            "x_m": float(hardware.get("laser_to_base_x_m", 0.0)),
            "y_m": float(hardware.get("laser_to_base_y_m", 0.0)),
            "z_m": float(hardware.get("laser_to_base_z_m", 0.0)),
            "yaw_rad": float(hardware.get("laser_to_base_yaw_rad", 0.0)),
            "base_frame_id": hardware.get("base_frame_id"),
            "laser_frame_id": hardware.get("laser_frame_id"),
        },
    }
    (output_derived / "lidar_motion_summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--kind", choices=("steering", "erpm"), required=True)
    parser.add_argument("--output-derived", type=Path,
                        help="Optional output directory for a non-mutating offline replay.")
    args = parser.parse_args()
    bag = args.bag.resolve()
    if not (bag / "metadata.yaml").exists():
        raise SystemExit(f"not a bag: {bag}")
    estimate(bag, args.config.resolve(), args.kind,
             output_derived=args.output_derived.resolve() if args.output_derived else None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
