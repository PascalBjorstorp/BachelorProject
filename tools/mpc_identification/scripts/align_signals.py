#!/usr/bin/env python3
"""Create a scan-rate, time-aligned observed trajectory for system identification."""
from __future__ import annotations

import argparse
from pathlib import Path
import numpy as np
import pandas as pd

from scipy.signal import savgol_filter

from geometry import project_to_raceline, wrap_angle


def icp_kinematics(t: np.ndarray, x: np.ndarray, y: np.ndarray, yaw: np.ndarray,
                   grid_hz: float = 100.0, smooth_s: float = 0.12) -> dict[str, np.ndarray]:
    """Derive a self-consistent velocity state from the ICP pose itself.

    Position truth in this pipeline is the scan-to-map ICP pose, but the velocity
    state was previously taken from odom, which is biased relative to ICP (speed
    +0.24 m/s, sideslip understated ~2x, yaw-rate +0.07 rad/s). Those biases were
    the dominant fit error: every multiple-shooting window started with the wrong
    velocity and drifted. Differentiating a smoothed ICP trajectory yields speed,
    sideslip, and yaw-rate that are consistent with the position we fit against.

    The signal is resampled to a uniform grid first so Savitzky-Golay smoothing is
    well-defined despite the irregular 4-50 ms scan spacing, then sampled back to
    the original ICP times.
    """
    yaw_u = np.unwrap(yaw)
    t0, t1 = float(t[0]), float(t[-1])
    n = max(int((t1 - t0) * grid_hz), len(t))
    tu = np.linspace(t0, t1, n)
    dt = tu[1] - tu[0]
    win = max(5, int(round(smooth_s / dt)) | 1)  # odd window
    xs = savgol_filter(np.interp(tu, t, x), win, 3)
    ys = savgol_filter(np.interp(tu, t, y), win, 3)
    yaws = savgol_filter(np.interp(tu, t, yaw_u), win, 3)
    vx = np.gradient(xs, tu)
    vy = np.gradient(ys, tu)
    yaw_rate = np.gradient(yaws, tu)
    speed = np.hypot(vx, vy)
    course = np.arctan2(vy, vx)
    beta = wrap_angle(course - yaws)
    # Sideslip is meaningless near standstill; hold it at zero there.
    beta = np.where(speed > 0.4, beta, 0.0)
    sample = lambda v: np.interp(t, tu, v)
    return {
        "icp_speed": sample(speed), "icp_vx_world": sample(vx), "icp_vy_world": sample(vy),
        "icp_beta": sample(beta), "icp_yaw_rate": sample(yaw_rate),
    }


def interpolate_linear(source: pd.DataFrame, t: np.ndarray, columns: list[str]) -> dict[str, np.ndarray]:
    source = source.sort_values("t_s")
    source_t = source.t_s.to_numpy()
    result = {}
    for col in columns:
        result[col] = np.interp(t, source_t, source[col].to_numpy(), left=np.nan, right=np.nan)
    return result


def zero_order_hold(source: pd.DataFrame, t: np.ndarray, columns: list[str]) -> dict[str, np.ndarray]:
    source = source.sort_values("t_s").reset_index(drop=True)
    source_t = source.t_s.to_numpy()
    ix = np.searchsorted(source_t, t, side="right") - 1
    valid = ix >= 0
    ix = np.clip(ix, 0, len(source) - 1)
    result = {}
    for col in columns:
        values = source[col].to_numpy()[ix].astype(float)
        values[~valid] = np.nan
        result[col] = values
    return result


def servo_to_angle(servo: np.ndarray, gain: float, offset: float, c0: float, c1: float, c2: float) -> np.ndarray:
    corrected = (servo - offset) / gain
    sign = np.sign(corrected)
    abs_corr = np.abs(corrected)
    if abs(c2) > 1e-12:
        disc = c1 * c1 - 4.0 * c2 * (c0 - abs_corr)
        out = np.full_like(servo, np.nan, dtype=float)
        good = disc >= 0.0
        out[good] = sign[good] * (-c1 + np.sqrt(disc[good])) / (2.0 * c2)
        out[~good] = corrected[~good]
        return out
    return corrected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--icp", type=Path, required=True)
    parser.add_argument("--odom", type=Path, required=True)
    parser.add_argument("--drive", type=Path, required=True)
    parser.add_argument("--servo-feedback", type=Path, required=True)
    parser.add_argument("--imu-yaw-rate", type=Path, required=True)
    parser.add_argument("--raceline", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--servo-gain", type=float, default=-0.7284)
    parser.add_argument("--servo-offset", type=float, default=0.55)
    parser.add_argument("--servo-c0", type=float, default=0.001490)
    parser.add_argument("--servo-c1", type=float, default=0.918061)
    parser.add_argument("--servo-c2", type=float, default=0.589566)
    parser.add_argument("--keep-invalid-icp", action="store_true")
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    icp = pd.read_csv(args.icp).sort_values("t_s").reset_index(drop=True)
    if not args.keep_invalid_icp:
        icp = icp[icp.valid.astype(bool)].copy().reset_index(drop=True)
    if len(icp) < 20:
        raise RuntimeError("Too few valid ICP poses; inspect ICP map/quality thresholds before fitting.")
    t = icp.t_s.to_numpy()
    out = icp.copy()

    odom = pd.read_csv(args.odom)
    for key, value in interpolate_linear(odom, t, ["vx", "vy", "wz"]).items():
        out[f"odom_{key}"] = value
    drive = pd.read_csv(args.drive)
    for key, value in zero_order_hold(drive, t, ["steering_angle", "speed", "acceleration"]).items():
        out[f"cmd_{key}"] = value
    servo = pd.read_csv(args.servo_feedback)
    raw_servo = zero_order_hold(servo, t, ["value"])["value"]
    out["servo_feedback_raw"] = raw_servo
    out["delta_feedback_rad"] = servo_to_angle(
        raw_servo, args.servo_gain, args.servo_offset, args.servo_c0, args.servo_c1, args.servo_c2
    )
    imu = pd.read_csv(args.imu_yaw_rate)
    out["imu_yaw_rate"] = interpolate_linear(imu, t, ["value"])["value"]

    # Self-consistent velocity state derived from the ICP position truth.
    for key, value in icp_kinematics(t, out["x"].to_numpy(), out["y"].to_numpy(), out["yaw"].to_numpy()).items():
        out[key] = value

    raceline = np.loadtxt(args.raceline, delimiter=",", comments="#")
    projection = project_to_raceline(out[["x", "y"]].to_numpy(), raceline)
    for key, value in projection.items():
        out[key] = value
    out["epsi"] = wrap_angle(out["yaw"].to_numpy() - out["path_yaw"].to_numpy())
    out["speed_body_mps"] = np.hypot(out["odom_vx"], out["odom_vy"])
    out["lap_s_unwrapped"] = np.unwrap(out["path_s"].to_numpy() * 2.0 * np.pi / raceline[-1, 0]) * raceline[-1, 0] / (2.0 * np.pi)
    out.to_csv(args.output, index=False)
    print(f"Wrote {args.output} with {len(out)} ICP-observed samples")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
