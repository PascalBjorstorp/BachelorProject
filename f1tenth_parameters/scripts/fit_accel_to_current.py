#!/usr/bin/env python3
"""
Fit acceleration-to-current gains and coastdown drag feedforward from recorded CSV logs.

Default behavior: use ALL runs (all matching files) instead of picking a single run.

Inputs (all runs used by default):
  - f1tenth_parameters/data/motor_torque_*.csv
  - f1tenth_parameters/data/rolling_resistance_*.csv

Outputs:
  - Recommended vesc_ackermann params for ACCEL_TO_CURRENT mode:
      accel_to_current_gain   [A/(m/s^2)]
      accel_to_brake_gain     [A/(m/s^2)]
      accel_drag_coulomb      [m/s^2]  (coastdown decel magnitude)

Model:
  F_net = m * a_x ≈ Kt_eff * I_motor + b
  a_drag(v) ≈ accel_drag_coulomb (+ viscous/quadratic terms if you extend fit)
"""

from __future__ import annotations

import argparse
import glob
from pathlib import Path

import numpy as np
import pandas as pd


def _list_files(pattern: str) -> list[str]:
    files = sorted(glob.glob(pattern))
    if not files:
        raise FileNotFoundError(f"No files match pattern: {pattern}")
    return files


def _fit_line(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    # y = a*x + b
    A = np.vstack([x, np.ones_like(x)]).T
    a, b = np.linalg.lstsq(A, y, rcond=None)[0]
    return float(a), float(b)


def _finite(values: list[float]) -> list[float]:
    return [float(v) for v in values if np.isfinite(v)]


def _fit_motor_file(path: str, *, mass: float, args: argparse.Namespace) -> dict:
    df = pd.read_csv(path)
    for col in ("phase", "imu_ax", "motor_current", "odom_vx"):
        if col not in df.columns:
            raise KeyError(f"{path}: missing column '{col}'")

    acc = df[df["phase"] == "acceleration"].copy()
    acc = acc[np.isfinite(acc["imu_ax"]) & np.isfinite(acc["motor_current"]) & np.isfinite(acc["odom_vx"])]
    acc = acc[
        (acc["odom_vx"] > args.min_speed)
        & (acc["motor_current"] > args.accel_current_min)
        & (acc["motor_current"] < args.accel_current_max)
        & (acc["imu_ax"] > args.min_ax_accel)
    ]

    br = df[df["phase"] == "braking"].copy()
    br = br[np.isfinite(br["imu_ax"]) & np.isfinite(br["motor_current"]) & np.isfinite(br["odom_vx"])]
    br = br[
        (br["odom_vx"] > args.min_speed)
        & (br["motor_current"] > args.brake_current_min)
        & (br["motor_current"] < args.brake_current_max)
        & (br["imu_ax"] < args.max_ax_brake)
    ]

    out: dict = {"path": path, "acc": acc, "br": br}

    if len(acc) >= 50:
        Kt_eff_acc, b_acc = _fit_line(acc["motor_current"].to_numpy(), mass * acc["imu_ax"].to_numpy())
        out.update({"Kt_eff_acc": Kt_eff_acc, "b_acc": b_acc, "n_acc": int(len(acc))})
    else:
        out.update({"Kt_eff_acc": np.nan, "b_acc": np.nan, "n_acc": int(len(acc))})

    if len(br) >= 50:
        Kt_eff_br, b_br = _fit_line(br["motor_current"].to_numpy(), mass * br["imu_ax"].to_numpy())
        out.update({"Kt_eff_brake": Kt_eff_br, "b_brake": b_br, "n_brake": int(len(br))})
    else:
        out.update({"Kt_eff_brake": np.nan, "b_brake": np.nan, "n_brake": int(len(br))})

    return out


def _fit_rolling_file(path: str, *, args: argparse.Namespace) -> dict:
    df = pd.read_csv(path)
    for col in ("phase", "timestamp_s", "odom_vx"):
        if col not in df.columns:
            raise KeyError(f"{path}: missing column '{col}'")

    co = df[df["phase"] == "coast"].copy()
    co = co[np.isfinite(co["timestamp_s"]) & np.isfinite(co["odom_vx"])]
    co = co[co["odom_vx"] > args.coast_min_speed]

    if len(co) < 20:
        return {"path": path, "a_coast": np.nan, "n_coast": int(len(co))}

    # linear v(t) fit during higher-speed part of coast
    a_vfit, _v0 = np.polyfit(co["timestamp_s"].to_numpy(), co["odom_vx"].to_numpy(), 1)
    return {"path": path, "a_coast": float(abs(a_vfit)), "n_coast": int(len(co))}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mass", type=float, default=3.314, help="Vehicle mass [kg].")
    parser.add_argument("--motor-file", type=str, default="", help="Use single motor torque CSV path.")
    parser.add_argument("--rolling-file", type=str, default="", help="Use single rolling resistance CSV path.")
    parser.add_argument(
        "--motor-glob",
        type=str,
        default="f1tenth_parameters/data/motor_torque_*.csv",
        help="Glob for motor torque runs (used when --motor-file not set).",
    )
    parser.add_argument(
        "--rolling-glob",
        type=str,
        default="f1tenth_parameters/data/rolling_resistance_*.csv",
        help="Glob for rolling resistance runs (used when --rolling-file not set).",
    )
    parser.add_argument("--min-speed", type=float, default=0.30, help="Min speed [m/s] for fits.")
    parser.add_argument("--accel-current-min", type=float, default=5.0, help="Min +current [A] for accel fit.")
    parser.add_argument("--accel-current-max", type=float, default=55.0, help="Max +current [A] for accel fit.")
    parser.add_argument("--brake-current-min", type=float, default=-55.0, help="Min (most negative) current [A] for brake fit.")
    parser.add_argument("--brake-current-max", type=float, default=-5.0, help="Max (least negative) current [A] for brake fit.")
    parser.add_argument("--min-ax-accel", type=float, default=0.10, help="Min ax [m/s^2] for accel fit.")
    parser.add_argument("--max-ax-brake", type=float, default=-0.20, help="Max ax [m/s^2] for brake fit (negative).")
    parser.add_argument("--coast-min-speed", type=float, default=1.0, help="Min speed [m/s] for coastdown slope fit.")
    args = parser.parse_args()

    mass = float(args.mass)

    motor_files = [str(Path(args.motor_file))] if args.motor_file else [str(Path(p)) for p in _list_files(args.motor_glob)]
    rolling_files = [str(Path(args.rolling_file))] if args.rolling_file else [str(Path(p)) for p in _list_files(args.rolling_glob)]

    motor_results = [_fit_motor_file(p, mass=mass, args=args) for p in motor_files]
    acc_slopes = _finite([r["Kt_eff_acc"] for r in motor_results])
    br_slopes = _finite([r["Kt_eff_brake"] for r in motor_results])

    acc_all = pd.concat([r["acc"] for r in motor_results if r.get("n_acc", 0) >= 50], ignore_index=True)
    br_all = pd.concat([r["br"] for r in motor_results if r.get("n_brake", 0) >= 50], ignore_index=True)
    if len(acc_all) < 100:
        raise RuntimeError(f"Not enough total accel samples after filtering across runs: n={len(acc_all)}")
    if len(br_all) < 100:
        raise RuntimeError(f"Not enough total brake samples after filtering across runs: n={len(br_all)}")

    Kt_eff_acc_all, b_acc_all = _fit_line(acc_all["motor_current"].to_numpy(), mass * acc_all["imu_ax"].to_numpy())
    Kt_eff_br_all, b_br_all = _fit_line(br_all["motor_current"].to_numpy(), mass * br_all["imu_ax"].to_numpy())

    roll_results = [_fit_rolling_file(p, args=args) for p in rolling_files]
    a_coast_vals = _finite([r["a_coast"] for r in roll_results])
    if not a_coast_vals:
        raise RuntimeError("No usable rolling resistance runs (coast fit failed for all).")
    a_coast_mean = float(np.mean(a_coast_vals))
    a_coast_std = float(np.std(a_coast_vals, ddof=1)) if len(a_coast_vals) > 1 else 0.0
    a_coast_med = float(np.median(a_coast_vals))

    accel_to_current_gain = mass / Kt_eff_acc_all
    accel_to_brake_gain = mass / abs(Kt_eff_br_all)

    print("=== ACCEL_TO_CURRENT parameter fit ===")
    if args.motor_file:
        print(f"motor_file:   {motor_files[0]}")
    else:
        print(f"motor_runs:   {len(motor_files)} files (glob={args.motor_glob})")
    if args.rolling_file:
        print(f"rolling_file: {rolling_files[0]}")
    else:
        print(f"rolling_runs: {len(rolling_files)} files (glob={args.rolling_glob})")
    print("")
    if acc_slopes:
        acc_mean = float(np.mean(acc_slopes))
        acc_std = float(np.std(acc_slopes, ddof=1)) if len(acc_slopes) > 1 else 0.0
        print(f"Kt_eff_acc (per-run):   mean={acc_mean:.4f} std={acc_std:.4f} N/A")
    if br_slopes:
        br_mean = float(np.mean(br_slopes))
        br_std = float(np.std(br_slopes, ddof=1)) if len(br_slopes) > 1 else 0.0
        print(f"Kt_eff_brake (per-run): mean={br_mean:.4f} std={br_std:.4f} N/A")
    print(f"Kt_eff_acc (all runs):   {Kt_eff_acc_all:.4f} N/A   (b={b_acc_all:.2f} N, n={len(acc_all)})")
    print(f"Kt_eff_brake (all runs): {Kt_eff_br_all:.4f} N/A   (b={b_br_all:.2f} N, n={len(br_all)})")
    print(f"a_coast (per-run):      mean={a_coast_mean:.3f} std={a_coast_std:.3f} median={a_coast_med:.3f} m/s^2")
    print("")
    print("Recommended vesc params:")
    print(f"  accel_to_current_gain: {accel_to_current_gain:.2f}")
    print(f"  accel_to_brake_gain:   {accel_to_brake_gain:.2f}")
    print(f"  accel_drag_coulomb:    {a_coast_med:.2f}")
    print("")
    print("YAML snippet:")
    print(f"  accel_to_current_gain: {accel_to_current_gain:.2f}")
    print(f"  accel_to_brake_gain: {accel_to_brake_gain:.2f}")
    print(f"  accel_drag_coulomb: {a_coast_med:.2f}")
    print("  accel_drag_viscous: 0.0")
    print("  accel_drag_quadratic: 0.0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

