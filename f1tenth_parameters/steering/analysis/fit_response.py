#!/usr/bin/env python3
"""Identify command-path and effective steering-response dynamics.

This analysis deliberately distinguishes two different quantities:

* **Command-path timing**: raw request -> selector -> servo command bus ->
  VESC-driver command echo.  This proves command delivery and software/ROS
  latency, but is *not* a measurement of servo-shaft position.
* **Effective vehicle steering dynamics**: raw-servo step -> curvature and
  equivalent bicycle steering angle inferred from IMU yaw rate and independently
  LiDAR-derived longitudinal velocity.  This is the quantity usable by the MPC
  plant because it includes actuator, linkage, tyre and yaw-response dynamics.

No value in this file is labelled a physical servo-shaft delay/rate: the vehicle
has no wheel-angle or servo-position sensor.  See the emitted summary JSON for
that observability limitation.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
import yaml
from scipy.optimize import least_squares
from scipy.signal import savgol_filter

from trials import accepted_trial_ids


CHANNELS: dict[str, str] = {
    "raw": "servo_raw",
    "selector": "servo_selected",
    "servo_bus": "servo_command",
    "driver_command_echo": "servo_echo",
}


def _read(path: Path) -> pd.DataFrame:
    return pd.read_parquet(path) if path.exists() else pd.DataFrame()


def _num(value: Any, default: float = math.nan) -> float:
    try:
        value = float(value)
        return value if math.isfinite(value) else default
    except (TypeError, ValueError):
        return default


def _field(frame: pd.DataFrame, name: str) -> pd.Series:
    return frame[name] if name in frame.columns else pd.Series(dtype=float)


def _nearest_gap_ns(source_t: np.ndarray, query_t: np.ndarray) -> np.ndarray:
    """Distance from each query time to the nearest source time."""
    index = np.searchsorted(source_t, query_t)
    right = np.minimum(index, len(source_t) - 1)
    left = np.maximum(index - 1, 0)
    return np.minimum(np.abs(query_t - source_t[left]), np.abs(source_t[right] - query_t))


def _sustained_crossing(t: np.ndarray, y: np.ndarray, threshold: float, hold_s: float) -> float | None:
    """First rising crossing that remains above threshold for ``hold_s``."""
    if len(t) == 0:
        return None
    candidates = np.flatnonzero(y >= threshold)
    for idx in candidates:
        end = int(np.searchsorted(t, t[idx] + hold_s, side="left"))
        if end <= idx:
            continue
        if end >= len(t):
            end = len(t) - 1
        section = y[idx : end + 1]
        if len(section) and np.all(np.isfinite(section)) and float(np.min(section)) >= threshold:
            return float(t[idx])
    return None


def _settling_time(t: np.ndarray, normalized: np.ndarray, band: float, hold_s: float) -> float | None:
    """First time response remains within [1-band, 1+band] for the remainder."""
    if len(t) == 0:
        return None
    err = np.abs(normalized - 1.0)
    for idx in range(len(t)):
        end = int(np.searchsorted(t, t[idx] + hold_s, side="left"))
        if end <= idx:
            continue
        if end >= len(t):
            end = len(t) - 1
        if np.all(np.isfinite(err[idx : end + 1])) and float(np.max(err[idx:])) <= band:
            return float(t[idx])
    return None


def _fit_fopdt(t: np.ndarray, normalized: np.ndarray) -> dict[str, float | bool | None]:
    """Fit normalized first-order-plus-dead-time response: K(1-exp(-(t-L)/tau))."""
    finite = np.isfinite(t) & np.isfinite(normalized) & (t >= 0.0)
    t, y = t[finite], normalized[finite]
    if len(t) < 12 or float(t[-1]) <= 0.15:
        return {"fopdt_fit_ok": False, "fopdt_delay_s": None, "fopdt_tau_s": None,
                "fopdt_gain": None, "fopdt_rmse_normalized": None}
    t10 = _sustained_crossing(t, y, 0.10, 0.04)
    t90 = _sustained_crossing(t, y, 0.90, 0.04)
    if t10 is not None and t90 is not None and t90 > t10:
        tau0 = max(0.015, (t90 - t10) / 2.197)
        delay0 = max(0.0, t10 - 0.105 * tau0)
    else:
        tau0, delay0 = max(0.05, float(t[-1]) / 4.0), 0.0
    max_delay = max(0.05, min(1.5, 0.75 * float(t[-1])))
    max_tau = max(0.20, 2.0 * float(t[-1]))

    def model(p: np.ndarray) -> np.ndarray:
        delay, tau, gain = p
        elapsed = np.maximum(t - delay, 0.0)
        return gain * (1.0 - np.exp(-elapsed / tau))

    try:
        result = least_squares(
            lambda p: model(p) - y,
            x0=np.array([min(delay0, max_delay), min(tau0, max_tau), 1.0]),
            bounds=(np.array([0.0, 0.005, 0.20]), np.array([max_delay, max_tau, 2.00])),
            loss="soft_l1",
            f_scale=0.04,
            max_nfev=300,
        )
        rmse = float(np.sqrt(np.mean((model(result.x) - y) ** 2)))
        return {
            "fopdt_fit_ok": bool(result.success),
            "fopdt_delay_s": float(result.x[0]),
            "fopdt_tau_s": float(result.x[1]),
            "fopdt_gain": float(result.x[2]),
            "fopdt_rmse_normalized": rmse,
        }
    except Exception:
        return {"fopdt_fit_ok": False, "fopdt_delay_s": None, "fopdt_tau_s": None,
                "fopdt_gain": None, "fopdt_rmse_normalized": None}


def _smoothed_peak_rate(t: np.ndarray, y: np.ndarray) -> float | None:
    finite = np.isfinite(t) & np.isfinite(y)
    t, y = t[finite], y[finite]
    if len(t) < 7:
        return None
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return None
    # For peak-rate reporting only. Timing metrics use the unsmoothed series.
    window = int(round(0.075 / dt))
    window = max(5, window + (1 - window % 2))
    if window >= len(y):
        window = len(y) - 1 if len(y) % 2 == 0 else len(y)
    if window < 5:
        return None
    smooth = savgol_filter(y, window_length=window, polyorder=2, mode="interp")
    return float(np.nanmax(np.abs(np.gradient(smooth, t))))


def _series_metrics(
    t: np.ndarray,
    y: np.ndarray,
    *,
    min_amplitude: float,
    onset_hold_s: float,
    settle_band_fraction: float,
    settle_hold_s: float,
) -> dict[str, float | bool | None]:
    """Compute robust response metrics from a series with time zero at command."""
    finite = np.isfinite(t) & np.isfinite(y)
    t, y = t[finite], y[finite]
    if len(t) < 12:
        return {"effective_response_valid": False, "reason": "too_few_samples"}
    baseline_mask = (t >= -1.0) & (t <= -0.10)
    post = t >= 0.0
    if not np.any(baseline_mask):
        baseline_mask = (t >= 0.0) & (t <= min(0.20, 0.10 * float(t.max())))
    if not np.any(post):
        return {"effective_response_valid": False, "reason": "no_post_command_samples"}
    final_start = max(0.0, float(t.max()) - 0.75)
    final_mask = t >= final_start
    initial = float(np.median(y[baseline_mask]))
    final = float(np.median(y[final_mask]))
    amplitude = final - initial
    if not math.isfinite(amplitude) or abs(amplitude) < min_amplitude:
        return {
            "effective_response_valid": False,
            "reason": "response_amplitude_below_noise_floor",
            "effective_initial_rad": initial,
            "effective_final_rad": final,
            "effective_amplitude_rad": amplitude,
        }
    t_post = t[post]
    normalized = (y[post] - initial) / amplitude
    t10 = _sustained_crossing(t_post, normalized, 0.10, onset_hold_s)
    t50 = _sustained_crossing(t_post, normalized, 0.50, onset_hold_s)
    t90 = _sustained_crossing(t_post, normalized, 0.90, onset_hold_s)
    settling = _settling_time(t_post, normalized, settle_band_fraction, settle_hold_s)
    fopdt = _fit_fopdt(t_post, normalized)
    return {
        "effective_response_valid": True,
        "reason": "",
        "effective_initial_rad": initial,
        "effective_final_rad": final,
        "effective_amplitude_rad": float(amplitude),
        "effective_delay_10pct_s": t10,
        "effective_delay_50pct_s": t50,
        "effective_rise_10_90_s": None if t10 is None or t90 is None else float(t90 - t10),
        "effective_settling_5pct_s": settling,
        "effective_overshoot_fraction": float(np.nanmax(normalized) - 1.0),
        "effective_peak_rate_rad_s": _smoothed_peak_rate(t_post, y[post]),
        **fopdt,
    }


def _channel_transition(
    frame: pd.DataFrame,
    start_ns: int,
    end_ns: int,
    target: float,
    *,
    onset_hold_s: float,
) -> dict[str, float | None]:
    """Delivery timing of a command-channel, not mechanical actuator timing."""
    if frame.empty or "bag_ns" not in frame or "value" not in frame:
        return {"initial": None, "final": None, "command_latency_10pct_s": None,
                "command_latency_50pct_s": None}
    before = frame[(frame.bag_ns >= start_ns - 1_000_000_000) & (frame.bag_ns < start_ns)]
    after = frame[(frame.bag_ns >= start_ns) & (frame.bag_ns <= end_ns)].copy()
    if len(after) == 0:
        return {"initial": None, "final": None, "command_latency_10pct_s": None,
                "command_latency_50pct_s": None}
    initial = float(np.median(before.value.to_numpy(dtype=float))) if len(before) else float(after.value.iloc[0])
    amplitude = float(target - initial)
    if abs(amplitude) < 1e-6:
        return {"initial": initial, "final": float(after.value.iloc[-1]), "command_latency_10pct_s": None,
                "command_latency_50pct_s": None}
    t = (after.bag_ns.to_numpy(dtype=float) - float(start_ns)) * 1e-9
    normalized = (after.value.to_numpy(dtype=float) - initial) / amplitude
    return {
        "initial": initial,
        "final": float(np.median(after.value.to_numpy(dtype=float)[-max(1, len(after)//5):])),
        "command_latency_10pct_s": _sustained_crossing(t, normalized, 0.10, onset_hold_s),
        "command_latency_50pct_s": _sustained_crossing(t, normalized, 0.50, onset_hold_s),
    }


def _interpolate_channel(frame: pd.DataFrame, t_ns: np.ndarray) -> np.ndarray:
    if frame.empty or "bag_ns" not in frame or "value" not in frame:
        return np.full(len(t_ns), np.nan)
    t = frame.bag_ns.to_numpy(dtype=float)
    y = frame.value.to_numpy(dtype=float)
    order = np.argsort(t)
    t, y = t[order], y[order]
    if len(t) == 0:
        return np.full(len(t_ns), np.nan)
    return np.interp(t_ns.astype(float), t, y, left=np.nan, right=np.nan)


def _effective_series(
    imu: pd.DataFrame,
    lidar: pd.DataFrame,
    *,
    start_ns: int,
    end_ns: int,
    wheelbase_m: float,
    min_speed_mps: float,
    max_lidar_gap_s: float,
) -> pd.DataFrame:
    """Time-align IMU yaw rate with valid LiDAR longitudinal speed."""
    im = imu[(imu.bag_ns >= start_ns - 1_000_000_000) & (imu.bag_ns <= end_ns)].copy()
    lv = lidar[(lidar.bag_ns >= start_ns - 1_000_000_000) & (lidar.bag_ns <= end_ns)].copy()
    if im.empty or lv.empty or "valid" not in lv:
        return pd.DataFrame()
    lv = lv[lv.valid.astype(bool)].copy()
    lv = lv[np.isfinite(lv.vx.to_numpy(dtype=float))]
    if len(im) < 12 or len(lv) < 6:
        return pd.DataFrame()
    im = im.sort_values("bag_ns")
    lv = lv.sort_values("bag_ns")
    t_ns = im.bag_ns.to_numpy(dtype=np.int64)
    lt = lv.bag_ns.to_numpy(dtype=np.int64)
    vx = np.interp(t_ns.astype(float), lt.astype(float), lv.vx.to_numpy(dtype=float), left=np.nan, right=np.nan)
    nearest_gap_s = _nearest_gap_ns(lt.astype(float), t_ns.astype(float)) * 1e-9
    gz = im.gz.to_numpy(dtype=float)
    valid = (
        np.isfinite(vx) & np.isfinite(gz) &
        (np.abs(vx) >= min_speed_mps) &
        (nearest_gap_s <= max_lidar_gap_s)
    )
    curvature = np.full(len(t_ns), np.nan)
    delta = np.full(len(t_ns), np.nan)
    curvature[valid] = gz[valid] / vx[valid]
    delta[valid] = np.arctan(wheelbase_m * curvature[valid])
    return pd.DataFrame({
        "bag_ns": t_ns,
        "t_command_s": (t_ns.astype(float) - float(start_ns)) * 1e-9,
        "vx_lidar_mps": vx,
        "imu_yaw_rate_rad_s": gz,
        "lidar_nearest_gap_s": nearest_gap_s,
        "effective_valid": valid,
        "curvature_inv_m": curvature,
        "delta_eq_rad": delta,
    })


def _segment_end(events: pd.DataFrame, trial_id: str, event_name: str, start_ns: int) -> int | None:
    subset = events[(events.event == event_name) & (events.trial_id.astype(str) == trial_id) &
                    (events.bag_ns > start_ns)]
    return int(subset.iloc[0].bag_ns) if len(subset) else None


def _return_phase_end(events: pd.DataFrame, trial_id: str, start_ns: int) -> int | None:
    subset = events[(events.event == "phase_end") & (events.phase == "response_return") &
                    (events.trial_id.astype(str) == trial_id) & (events.bag_ns > start_ns)]
    return int(subset.iloc[0].bag_ns) if len(subset) else None


def _command_baseline(frame: pd.DataFrame, start_ns: int, fallback: float) -> float:
    if frame.empty:
        return fallback
    prior = frame[(frame.bag_ns >= start_ns - 1_000_000_000) & (frame.bag_ns < start_ns)]
    if len(prior):
        return float(np.median(prior.value.to_numpy(dtype=float)))
    return fallback


def _analyse_segment(
    *,
    trial_id: str,
    segment: str,
    start_ns: int,
    end_ns: int,
    raw_target: float,
    metadata: dict[str, Any],
    channel_frames: dict[str, pd.DataFrame],
    imu: pd.DataFrame,
    lidar: pd.DataFrame,
    cfg: dict[str, Any],
) -> tuple[dict[str, Any], pd.DataFrame]:
    response_cfg = cfg["response"]
    analysis_cfg = cfg["analysis"]["response"]
    baseline_raw = _command_baseline(channel_frames["raw"], start_ns, raw_target)
    row: dict[str, Any] = {
        **metadata,
        "trial_id": trial_id,
        "segment": segment,
        "command_bag_ns": int(start_ns),
        "segment_end_bag_ns": int(end_ns),
        "raw_servo_initial": baseline_raw,
        "raw_servo_target": float(raw_target),
        "raw_servo_step": float(raw_target - baseline_raw),
    }
    for channel, frame in channel_frames.items():
        result = _channel_transition(frame, start_ns, end_ns, raw_target,
                                     onset_hold_s=float(response_cfg["command_onset_hold_s"]))
        prefix = f"{channel}_"
        row.update({prefix + key: value for key, value in result.items()})
    effective = _effective_series(
        imu, lidar,
        start_ns=start_ns,
        end_ns=end_ns,
        wheelbase_m=float(cfg["hardware"]["wheelbase_m"]),
        min_speed_mps=float(analysis_cfg["min_lidar_speed_mps"]),
        max_lidar_gap_s=float(analysis_cfg["max_lidar_interpolation_gap_s"]),
    )
    if effective.empty:
        row.update({"effective_response_valid": False, "reason": "missing_or_insufficient_imu_lidar_data"})
        return row, effective
    valid_fraction = float(effective.effective_valid.mean())
    row["effective_valid_sample_fraction"] = valid_fraction
    metrics = _series_metrics(
        effective.t_command_s.to_numpy(dtype=float),
        effective.delta_eq_rad.to_numpy(dtype=float),
        min_amplitude=float(analysis_cfg["min_effective_delta_amplitude_rad"]),
        onset_hold_s=float(response_cfg["effective_onset_hold_s"]),
        settle_band_fraction=float(response_cfg["settling_band_fraction"]),
        settle_hold_s=float(response_cfg["settling_hold_s"]),
    )
    if valid_fraction < float(analysis_cfg["min_effective_valid_sample_fraction"]):
        metrics["effective_response_valid"] = False
        metrics["reason"] = "insufficient_valid_lidar_speed_fraction"
    row.update(metrics)
    if bool(row.get("effective_response_valid")):
        delta_step = _num(row.get("effective_amplitude_rad"))
        raw_step = _num(row.get("raw_servo_step"))
        row["effective_static_gain_rad_per_servo"] = delta_step / raw_step if abs(raw_step) > 1e-6 else math.nan
        final_curvature = math.tan(_num(row.get("effective_final_rad"))) / float(cfg["hardware"]["wheelbase_m"])
        row["effective_final_curvature_inv_m"] = final_curvature
    else:
        row["effective_static_gain_rad_per_servo"] = math.nan
        row["effective_final_curvature_inv_m"] = math.nan
    for channel, frame in channel_frames.items():
        effective[channel] = _interpolate_channel(frame, effective.bag_ns.to_numpy(dtype=np.int64))
    effective["trial_id"] = trial_id
    effective["segment"] = segment
    for key, value in metadata.items():
        effective[key] = value
    return row, effective


def _group_summary(table: pd.DataFrame) -> pd.DataFrame:
    if table.empty:
        return table
    use = table[table.effective_response_valid.astype(bool)].copy() if "effective_response_valid" in table else pd.DataFrame()
    if use.empty:
        return pd.DataFrame()
    columns = [
        "effective_delay_10pct_s", "effective_rise_10_90_s", "effective_settling_5pct_s",
        "effective_peak_rate_rad_s", "effective_overshoot_fraction", "fopdt_delay_s", "fopdt_tau_s",
        "fopdt_gain", "fopdt_rmse_normalized", "effective_static_gain_rad_per_servo",
    ]
    columns = [c for c in columns if c in use.columns]
    grouped = use.groupby(["segment", "speed_mps", "side", "fraction"], dropna=False)[columns].agg(["count", "median", "mean", "std"])
    grouped.columns = [f"{a}_{b}" for a, b in grouped.columns]
    return grouped.reset_index()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()
    session = args.session.resolve()
    cfg = yaml.safe_load((session / "calibration_config_snapshot.yaml").read_text(encoding="utf-8"))
    stage = session / "06_command_to_curvature_response" / "derived"
    events = _read(stage / "events.parquet")
    imu = _read(stage / "imu.parquet")
    lidar = _read(stage / "lidar_velocity.parquet")
    if events.empty:
        raise SystemExit("response stage contains no exported events")
    channel_frames = {name: _read(stage / f"{file_name}.parquet") for name, file_name in CHANNELS.items()}
    accepted = accepted_trial_ids(events)
    steps = events[(events.event == "response_step_command") & (events.trial_id.astype(str).isin(accepted))].copy()
    rows: list[dict[str, Any]] = []
    series: list[pd.DataFrame] = []
    for _, event in steps.iterrows():
        trial_id = str(event.trial_id)
        step_start = int(event.bag_ns)
        return_start = _segment_end(events, trial_id, "response_return_command", step_start)
        if return_start is None:
            continue
        return_end = _return_phase_end(events, trial_id, return_start)
        if return_end is None:
            continue
        metadata = {
            "trial_index": event.get("trial_index"),
            "speed_mps": _num(event.get("speed_mps")),
            "side": event.get("side"),
            "fraction": _num(event.get("fraction")),
            "repetition": event.get("repetition"),
        }
        step_row, step_series = _analyse_segment(
            trial_id=trial_id, segment="outbound_step", start_ns=step_start, end_ns=return_start,
            raw_target=_num(event.get("raw_target")), metadata=metadata, channel_frames=channel_frames,
            imu=imu, lidar=lidar, cfg=cfg,
        )
        rows.append(step_row)
        if not step_series.empty:
            series.append(step_series)
        return_event = events[(events.event == "response_return_command") &
                              (events.trial_id.astype(str) == trial_id) &
                              (events.bag_ns == return_start)]
        return_target = _num(return_event.iloc[0].get("raw_target")) if len(return_event) else _num(event.get("raw_target"))
        return_row, return_series = _analyse_segment(
            trial_id=trial_id, segment="return_to_centre", start_ns=return_start, end_ns=return_end,
            raw_target=return_target, metadata=metadata, channel_frames=channel_frames,
            imu=imu, lidar=lidar, cfg=cfg,
        )
        rows.append(return_row)
        if not return_series.empty:
            series.append(return_series)
    table = pd.DataFrame(rows)
    output = session / "analysis"
    output.mkdir(exist_ok=True)
    # The legacy filename is retained, but the column names now make the
    # distinction between command path and effective vehicle response explicit.
    table.to_parquet(output / "command_to_curvature_response_metrics.parquet", index=False)
    table.to_parquet(output / "command_to_effective_steering_response_metrics.parquet", index=False)
    if series:
        pd.concat(series, ignore_index=True).to_parquet(output / "effective_steering_response_timeseries.parquet", index=False)
    group = _group_summary(table)
    if not group.empty:
        group.to_parquet(output / "effective_steering_response_group_summary.parquet", index=False)
    valid = table[table.effective_response_valid.astype(bool)] if len(table) and "effective_response_valid" in table else pd.DataFrame()
    summary: dict[str, Any] = {
        "measurement_scope": {
            "command_path": "raw request -> selector -> servo bus -> VESC-driver command echo",
            "effective_steering": "raw servo step -> IMU yaw rate / LiDAR velocity -> curvature -> equivalent bicycle steering angle",
            "not_measured": [
                "physical servo-shaft angle", "physical servo-shaft velocity", "road-wheel angle",
                "separate servo, linkage, tyre and vehicle-yaw contributions",
            ],
        },
        "segments_total": int(len(table)),
        "segments_with_valid_effective_response": int(len(valid)),
        "outbound_step_segments": int((table.segment == "outbound_step").sum()) if len(table) else 0,
        "return_segments": int((table.segment == "return_to_centre").sum()) if len(table) else 0,
    }
    if len(valid):
        fields = [
            "effective_delay_10pct_s", "effective_rise_10_90_s", "effective_settling_5pct_s",
            "effective_peak_rate_rad_s", "effective_overshoot_fraction", "fopdt_delay_s", "fopdt_tau_s",
        ]
        for field in fields:
            if field in valid:
                values = pd.to_numeric(valid[field], errors="coerce").dropna()
                summary[f"median_{field}"] = float(values.median()) if len(values) else None
    for channel in CHANNELS:
        field = f"{channel}_command_latency_50pct_s"
        if field in table:
            values = pd.to_numeric(table[field], errors="coerce").dropna()
            summary[f"median_{field}"] = float(values.median()) if len(values) else None
    (output / "command_to_effective_steering_response_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    # Backward-compatible summary name for existing workflow references.
    (output / "command_to_curvature_response_summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
