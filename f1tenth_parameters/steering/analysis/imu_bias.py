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
temperature over a session.  We therefore estimate it from every available
*on-ground* stationary epoch (the dedicated early Stage 1b IMU capture and the
on-ground Stage 3 observability baseline) and, when those epochs disagree by
more than their own within-epoch noise, model the gyro bias as a clamped linear
function of time so each driving stage is corrected with the bias appropriate to
*its* moment in the session.  Outside the bracketed epochs the bias is held
constant (never extrapolated).  The on-stand Stage 0 audit is deliberately *not*
used here: on a stand the resting attitude differs from the driving attitude, so
its ay/ax gravity projection would be wrong.  This mirrors the longitudinal IMU-bias handling in the ERPM
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
GROUND_EARLY_STAGE = "01b_imu_bias_ground"
_MIN_EPOCH_SAMPLES = 20
_MIN_DRIFT_SPAN_S = 20.0


def correction_enabled(config: dict | None, *, default: bool = True) -> bool:
    """Read the explicit stationary-bias policy from a calibration profile.

    Older standalone steering sessions retain their historical behaviour unless
    they opt out.  The unified campaign sets this false because each launched
    stack already runs its own startup calibration.
    """
    if not isinstance(config, dict):
        return bool(default)
    analysis = config.get("analysis", {})
    if not isinstance(analysis, dict):
        return bool(default)
    policy = analysis.get("imu_bias", {})
    if not isinstance(policy, dict):
        return bool(default)
    return bool(policy.get("apply_stationary_correction", default))


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
    # The production bringup independently establishes its own startup bias on
    # every launch.  A stationary epoch from a different calibration stack can
    # therefore be valuable evidence about sensor health without being safe to
    # subtract from later runs.  Unified sessions default to this diagnostic
    # mode; legacy callers can explicitly opt in to correction.
    correction_applied: bool = True
    epochs: list[dict] = field(default_factory=list)

    def gz_at(self, t_ns):
        """Gyro-z bias at one or many bag timestamps (clamped outside epochs)."""
        if not self.correction_applied:
            if np.isscalar(t_ns):
                return 0.0
            return np.zeros(np.shape(t_ns), dtype=float)
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
            "correction_applied": self.correction_applied,
            "epochs": self.epochs,
            "note": (
                "Stationary values are "
                + ("subtracted from steering analysis; when >=2 epochs disagree beyond their noise, "
                   "gz is modelled as a clamped linear drift. ay/ax offsets include resting gravity projection."
                   if self.correction_applied else
                   "diagnostic only and are not subtracted. The active bringup performs a fresh startup "
                   "IMU calibration on every stack launch, so a prior-session/static epoch must not be "
                   "treated as a persistent calibration constant.")
            ),
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


def estimate_imu_bias(session: Path, *, trim_s: float = 1.0,
                      apply_correction: bool = True) -> ImuBias:
    """Build an :class:`ImuBias` from a session's stationary captures.

    ``apply_correction`` deliberately controls use separately from estimation.
    When false, the raw stationary values and drift evidence remain in the
    report, but :meth:`ImuBias.gz_at` returns zero so a previous stack launch
    cannot inject a stale bias correction into a later one.
    """
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

    ground_early = session / GROUND_EARLY_STAGE / "derived" / "imu.parquet"
    if ground_early.exists():
        window = pd.read_parquet(ground_early)
        if len(window):
            epoch_frames.append((GROUND_EARLY_STAGE, window))

    epochs = [e for e in (_epoch(frame, stage, trim_s) for stage, frame in epoch_frames) if e is not None]
    if not epochs:
        return ImuBias(model="zero", used_fallback_zero=True,
                       correction_applied=apply_correction, epochs=[])

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
            model="linear_drift", n=total, used_fallback_zero=False,
            correction_applied=apply_correction, epochs=epochs,
        )
    constant = float(np.average(gz, weights=weights))
    return ImuBias(
        gz_intercept=constant, gz_slope_per_ns=0.0, t_ref_ns=float(t[0]),
        t_min_ns=float(t[0]), t_max_ns=float(t[-1]), ay_bias=ay_bias, ax_bias=ax_bias,
        model="constant", n=total, used_fallback_zero=False,
        correction_applied=apply_correction, epochs=epochs,
    )
