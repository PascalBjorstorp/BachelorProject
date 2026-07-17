"""Bounded live on-board guidance for the steering-centre capture.

This module deliberately stops short of declaring a calibration result.  It
uses the direction of measured IMU/odometry yaw to place a compact LiDAR grid,
then the offline LiDAR fit and independent physical validation decide whether a
new offset may actually be applied.
"""
from __future__ import annotations

from typing import Any

import numpy as np


def fit_onboard_zero(raw_servo: list[float], yaw_rate: list[float], config: dict[str, Any]) -> dict[str, Any]:
    """Fit a bounded zero crossing from one on-board yaw signal."""
    raw = np.asarray(raw_servo, dtype=float)
    yaw = np.asarray(yaw_rate, dtype=float)
    finite = np.isfinite(raw) & np.isfinite(yaw)
    raw, yaw = raw[finite], yaw[finite]
    result: dict[str, Any] = {
        "samples": int(len(raw)),
        "valid": False,
        "centre_servo_raw": None,
        "failures": [],
    }
    if len(raw) < 3:
        result["failures"].append("fewer than three finite probe samples")
        return result
    if len(np.unique(np.round(raw, 8))) < 3:
        result["failures"].append("fewer than three distinct raw-servo probe values")
        return result
    slope, intercept = (float(value) for value in np.polyfit(raw, yaw, 1))
    prediction = slope * raw + intercept
    residual = yaw - prediction
    ss_residual = float(np.sum(residual ** 2))
    ss_total = float(np.sum((yaw - np.mean(yaw)) ** 2))
    r2 = 1.0 if ss_total <= 1e-15 and ss_residual <= 1e-15 else (1.0 - ss_residual / ss_total if ss_total > 0 else float("-inf"))
    rmse = float(np.sqrt(np.mean(residual ** 2)))
    centre = float(-intercept / slope) if abs(slope) > 1e-8 else None
    result.update({
        "yaw_vs_servo_slope_rad_s_per_servo": slope,
        "fit_r2": r2,
        "fit_rmse_rad_s": rmse,
        "centre_servo_raw": centre,
        "raw_servo_min": float(np.min(raw)),
        "raw_servo_max": float(np.max(raw)),
    })
    expected_sign = int(config.get("expected_yaw_rate_slope_sign", -1))
    if centre is None or not np.isfinite(centre):
        result["failures"].append("probe has no finite yaw zero crossing")
    if expected_sign and np.sign(slope) != np.sign(expected_sign):
        result["failures"].append("probe yaw/servo slope has the unexpected sign")
    if r2 < float(config.get("min_onboard_fit_r2", 0.50)):
        result["failures"].append("probe R² is below the on-board guidance threshold")
    if rmse > float(config.get("max_onboard_fit_rmse_rad_s", 0.050)):
        result["failures"].append("probe RMSE exceeds the on-board guidance threshold")
    if centre is not None and np.isfinite(centre):
        extrapolation = max(float(np.min(raw)) - centre, centre - float(np.max(raw)), 0.0)
        result["fit_extrapolation_servo"] = extrapolation
        if extrapolation > float(config.get("max_onboard_probe_extrapolation_servo", 0.0)):
            result["failures"].append("probe zero crossing lies outside its observed bracket")
    result["valid"] = not result["failures"]
    return result


def choose_provisional_centre(seed_servo: float, imu_fit: dict[str, Any], odom_fit: dict[str, Any],
                              config: dict[str, Any]) -> dict[str, Any]:
    """Choose a *provisional* direction-aware centre, never a final update."""
    candidates: list[tuple[str, float]] = []
    for name, fit in (("imu", imu_fit), ("odom", odom_fit)):
        value = fit.get("centre_servo_raw") if isinstance(fit, dict) else None
        if bool(fit.get("valid", False)) and value is not None and np.isfinite(float(value)):
            candidates.append((name, float(value)))
    spread_limit = float(config.get("max_onboard_sensor_spread_servo", 0.020))
    value = float(seed_servo)
    source = "deployed_seed_fallback"
    warning: str | None = None
    if len(candidates) == 2:
        imu_value, odom_value = candidates[0][1], candidates[1][1]
        if abs(imu_value - odom_value) <= spread_limit:
            value = 0.5 * (imu_value + odom_value)
            source = "imu_odom_consensus"
        else:
            # The IMU measures vehicle motion directly, whereas odometry
            # contains the currently deployed steering map.  It is still useful
            # as a warning, but the IMU is the better directional fallback.
            value = imu_value
            source = "imu_with_odom_disagreement"
            warning = f"IMU/odom provisional centres differ by {abs(imu_value - odom_value):.6f} servo"
    elif len(candidates) == 1:
        source, value = f"{candidates[0][0]}_only", candidates[0][1]
    maximum_shift = float(config.get("max_onboard_guided_shift_servo", 0.030))
    if abs(value - seed_servo) > maximum_shift:
        warning = (
            f"on-board provisional shift {value - seed_servo:+.6f} exceeds the bounded "
            f"{maximum_shift:.6f}-servo limit; retaining the deployed seed"
        )
        value, source = float(seed_servo), "deployed_seed_outlier_guard"
    return {
        "centre_servo_raw": float(value),
        "source": source,
        "seed_servo_raw": float(seed_servo),
        "shift_from_seed_servo": float(value - seed_servo),
        "sensor_centres_servo": {name: candidate for name, candidate in candidates},
        "max_sensor_spread_servo": spread_limit,
        "max_guided_shift_servo": maximum_shift,
        "warning": warning,
        "note": "Provisional live guidance only; LiDAR fitting and new-data validation remain mandatory.",
    }


def fine_grid_targets(seed_servo: float, provisional_servo: float, offsets: list[float],
                      lower: float, upper: float) -> list[float]:
    """Return a compact fine grid that includes both the old and guided centre."""
    raw_targets = [float(provisional_servo) + float(offset) for offset in offsets]
    raw_targets.append(float(seed_servo))
    unique = sorted({round(value, 8) for value in raw_targets})
    if not unique or unique[0] < lower or unique[-1] > upper:
        raise RuntimeError(f"guided fine grid {unique} exceeds raw-servo domain [{lower:.5f}, {upper:.5f}]")
    return [float(value) for value in unique]
