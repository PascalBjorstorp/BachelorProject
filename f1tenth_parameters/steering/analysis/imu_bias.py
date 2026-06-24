#!/usr/bin/env python3
"""Stationary IMU bias estimation for the steering calibration analysis.

Every steering quantity in this pipeline is derived from the IMU yaw rate via
the kinematic relation ``delta_eq = arctan(L * gz / vx)``.  A constant gyro-z
(yaw-rate) bias of only a few milliradians per second therefore shifts the
identified zero-curvature centre and the entire raw-servo -> steering map.
MEMS gyros routinely carry such an offset, and the per-scan-pair ICP yaw
residual gate (``max_imu_yaw_residual_rad``) is far too coarse to remove a
slowly varying constant: a 0.01 rad/s bias over a 0.05 s scan pair is only
5e-4 rad of residual, well under the gate, yet it is a real steady-state error.

The offset is also not perfectly constant: MEMS gyro bias drifts with
temperature over a session.  We therefore estimate it from *every* available
stationary epoch (the on-stand Stage 0 audit and the on-ground Stage 3
observability capture) and, when those epochs disagree by more than their own
within-epoch noise, model the gyro bias as a clamped linear function of time so
each driving stage is corrected with the bias appropriate to *its* moment in
the session.  Outside the bracketed epochs the bias is held constant (never
extrapolated).  This mirrors the longitudinal IMU-bias handling in the ERPM
campaign (``fit_odom_model_selection._stationary_imu_ax_bias``) and extends it
with drift tracking.

The lateral- and longitudinal-accelerometer (``ay``/``ax``) stationary offsets
are estimated and reported too.  At rest they capture the static gravity
projection plus sensor offset, so subtracting them yields the *dynamic*
lateral/longitudinal acceleration about the resting attitude.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from trials import accepted_trial_ids

STATIONARY_STAGE = "03_sensor_observability"
STATIONARY_PHASE = "observability_stationary"
STAND_STAGE = "00_command_chain_audit"
_MIN_EPOCH_SAMPLES = 20
_MIN_DRIFT_SPAN_S = 20.0


@dataclass
class ImuBias:
    """Stationary IMU offsets, with an optional clamped linear gz drift model."""

    gz_intercept: float = 0.0
    gz_slope_per_ns: float = 0.0
    t_ref_ns: float = 0.0
    t_min_ns: float = 0.0
    t_max_ns: float = 0.0
    ay_bias: float = 0.0
    ax_bias: float = 0.0
    model: str = "zero"
    n: int = 0
    used_fallback_zero: bool = True
    epochs: list[dict] = field(default_factory=list)

    def gz_at(self, t_ns):
        """Gyro-z bias at one or many bag timestamps (clamped outside epochs)."""
        if self.gz_slope_per_ns == 0.0:
            if np.isscalar(t_ns):
                return float(self.gz_intercept)
            return np.full(np.shape(t_ns), float(self.gz_intercept), dtype=float)
        clamped = np.clip(np.asarray(t_ns, dtype=float), self.t_min_ns, self.t_max_ns)
        value = self.gz_intercept + self.gz_slope_per_ns * (clamped - self.t_ref_ns)
        return float(value) if np.isscalar(t_ns) else value

    def to_dict(self) -> dict:
        return {
            "model": self.model,
            "gz_bias_intercept_rad_s": self.gz_intercept,
            "gz_bias_slope_rad_s_per_s": self.gz_slope_per_ns * 1e9,
            "gz_bias_at_first_epoch_rad_s": self.gz_at(self.t_min_ns) if self.epochs else self.gz_intercept,
            "gz_bias_at_last_epoch_rad_s": self.gz_at(self.t_max_ns) if self.epochs else self.gz_intercept,
            "gz_drift_over_session_rad_s": (self.gz_at(self.t_max_ns) - self.gz_at(self.t_min_ns)) if self.epochs else 0.0,
            "ay_bias_mps2": self.ay_bias,
            "ax_bias_mps2": self.ax_bias,
            "stationary_samples": self.n,
            "used_fallback_zero": self.used_fallback_zero,
            "epochs": self.epochs,
            "note": ("gz is subtracted from every yaw rate before steering identification; when >=2 "
                     "stationary epochs disagree beyond their noise it is modelled as a clamped linear "
                     "drift in time. ay/ax offsets include the resting gravity projection."),
        }


def _stationary_windows(events: pd.DataFrame, phase: str) -> list[tuple[int, int]]:
    accepted = accepted_trial_ids(events)
    starts = events[(events.get("event") == "phase_start") & (events.get("phase") == phase)]
    ends = events[(events.get("event") == "phase_end") & (events.get("phase") == phase)]
    out: list[tuple[int, int]] = []
    for _, start in starts.iterrows():
        trial_id = str(start.get("trial_id"))
        # When no operator decisions were recorded, accept every window.
        if accepted and trial_id not in accepted:
            continue
        matches = ends[(ends.get("trial_id").astype(str) == trial_id) & (ends.bag_ns > start.bag_ns)]
        if len(matches):
            out.append((int(start.bag_ns), int(matches.iloc[0].bag_ns)))
    return out


def _epoch(frame: pd.DataFrame, stage: str, trim_s: float) -> dict | None:
    gz = frame["gz"].to_numpy(dtype=float)
    gz = gz[np.isfinite(gz)]
    if len(gz) < _MIN_EPOCH_SAMPLES:
        return None
    epoch = {
        "stage": stage,
        "t_ns": float(np.median(frame["bag_ns"].to_numpy(dtype=float))),
        "gz_median_rad_s": float(np.median(gz)),
        "gz_std_rad_s": float(np.std(gz)),
        "samples": int(len(gz)),
    }
    for channel in ("ay", "ax"):
        if channel in frame:
            values = frame[channel].to_numpy(dtype=float)
            values = values[np.isfinite(values)]
            epoch[f"{channel}_median_mps2"] = float(np.median(values)) if len(values) else 0.0
    return epoch


def estimate_imu_bias(session: Path, *, trim_s: float = 1.0) -> ImuBias:
    """Build an :class:`ImuBias` from a session's stationary captures."""
    session = Path(session)
    epoch_frames: list[tuple[str, pd.DataFrame]] = []

    stage = session / STATIONARY_STAGE / "derived"
    events_path, imu_path = stage / "events.parquet", stage / "imu.parquet"
    if events_path.exists() and imu_path.exists():
        events = pd.read_parquet(events_path)
        imu = pd.read_parquet(imu_path)
        for start_ns, end_ns in _stationary_windows(events, STATIONARY_PHASE):
            a, b = start_ns + int(trim_s * 1e9), end_ns - int(trim_s * 1e9)
            if b <= a:
                a, b = start_ns, end_ns
            window = imu[(imu.bag_ns >= a) & (imu.bag_ns <= b)]
            if len(window):
                epoch_frames.append((STATIONARY_STAGE, window))

    stand = session / STAND_STAGE / "derived" / "imu.parquet"
    if stand.exists():
        window = pd.read_parquet(stand)
        if len(window):
            epoch_frames.append((STAND_STAGE, window))

    epochs = [e for e in (_epoch(frame, stage, trim_s) for stage, frame in epoch_frames) if e is not None]
    if not epochs:
        return ImuBias(model="zero", used_fallback_zero=True, epochs=[])

    epochs.sort(key=lambda e: e["t_ns"])
    total = int(sum(e["samples"] for e in epochs))
    ay_bias = float(np.median([e["ay_median_mps2"] for e in epochs if "ay_median_mps2" in e] or [0.0]))
    ax_bias = float(np.median([e["ax_median_mps2"] for e in epochs if "ax_median_mps2" in e] or [0.0]))
    t = np.array([e["t_ns"] for e in epochs], dtype=float)
    gz = np.array([e["gz_median_rad_s"] for e in epochs], dtype=float)
    weights = np.array([e["samples"] for e in epochs], dtype=float)

    span_s = float((t[-1] - t[0]) * 1e-9)
    drift = abs(gz[-1] - gz[0])
    noise = max(float(np.max([e["gz_std_rad_s"] for e in epochs])), 1e-6)
    if len(epochs) >= 2 and span_s >= _MIN_DRIFT_SPAN_S and drift > noise:
        # Weighted least-squares line through the epoch medians, anchored at the
        # first epoch time. Held constant outside [t_min, t_max] when evaluated.
        slope, intercept = np.polyfit(t - t[0], gz, 1, w=weights)
        return ImuBias(
            gz_intercept=float(intercept), gz_slope_per_ns=float(slope), t_ref_ns=float(t[0]),
            t_min_ns=float(t[0]), t_max_ns=float(t[-1]), ay_bias=ay_bias, ax_bias=ax_bias,
            model="linear_drift", n=total, used_fallback_zero=False, epochs=epochs,
        )
    constant = float(np.average(gz, weights=weights))
    return ImuBias(
        gz_intercept=constant, gz_slope_per_ns=0.0, t_ref_ns=float(t[0]),
        t_min_ns=float(t[0]), t_max_ns=float(t[-1]), ay_bias=ay_bias, ax_bias=ax_bias,
        model="constant", n=total, used_fallback_zero=False, epochs=epochs,
    )
