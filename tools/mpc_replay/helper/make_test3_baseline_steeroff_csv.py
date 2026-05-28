#!/usr/bin/env python3
"""
Build a single Baseline-vs-SteerOff comparison CSV from existing Test 3
ablation_*.csv files. Lap-filters to race laps 2..MAX_LAP (warmup and
post-lap-11 are dropped).

Columns (one row per solver call, both runs concatenated):
  run                      "Baseline" or "SteerOff"
  stamp_ns                 bag record timestamp
  lap                      race-lap number (2..MAX_LAP)
  pos_x, pos_y             vehicle position [m]
  speed                    commanded longitudinal speed from /drive [m/s]
  steer                    commanded steering angle from /drive [rad]
  actual_steer_est         actuator-limited steering estimate [rad]
  steer_rate               actuator-limited steering rate [rad/s]
  abs_steer_rate           |steer_rate|
  command_steer_rate       raw command-jump rate Δsteer/Δt [rad/s]
  abs_command_steer_rate   |command_steer_rate|
  iterations               ADMM iterations per solve
  lateral_dev              distance from Baseline lap-2 reference path [m]

Usage:
  python3 make_test3_baseline_steeroff_csv.py \
    --baseline-csv .../ablation_Baseline.csv \
    --steeroff-csv .../ablation_SteerOff.csv \
    --out          .../baseline_vs_steeroff.csv
"""

import argparse
import csv
from pathlib import Path

import numpy as np


DEFAULT_STEERING_RATE_LIMIT_RAD_S = 2.849   # same as plot_test3_ablation.py


# ---------- I/O ----------

def load_ablation(path):
    rows = list(csv.DictReader(open(path)))
    return {
        "stamp_ns":   np.array([int(r["stamp_ns"]) for r in rows], dtype=np.int64),
        "pos_x":      np.array([float(r["pos_x"])  for r in rows]),
        "pos_y":      np.array([float(r["pos_y"])  for r in rows]),
        "iterations": np.array([float(r["iterations"]) for r in rows]),
        "steer":      np.array([float(r["steer"])  for r in rows]),
        "speed":      np.array([float(r["speed"])  for r in rows]),
    }


def drop_warmup(d):
    mask = (d["pos_x"] ** 2 + d["pos_y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in d.items()}


# ---------- Lap detection (same as Test 2/3) ----------

def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    n = len(pos_x)
    if n < 2:
        return [(0, n)]
    sx, sy = pos_x[0], pos_y[0]
    dist = np.sqrt((pos_x - sx) ** 2 + (pos_y - sy) ** 2)
    seg = np.sqrt(np.diff(pos_x) ** 2 + np.diff(pos_y) ** 2)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    starts = [0]; last = 0.0; in_zone = True
    for i in range(1, n):
        inside = dist[i] < min_radius
        if inside and not in_zone and (cum[i] - last) > min_lap_length:
            starts.append(i); last = cum[i]
        in_zone = inside
    starts.append(n)
    return [(starts[i], starts[i + 1]) for i in range(len(starts) - 1)]


def assign_laps(d, max_lap):
    laps = detect_laps(d["pos_x"], d["pos_y"])
    kept = [(i, laps[i - 1]) for i in range(2, max_lap + 1) if i <= len(laps)]
    if not kept:
        raise SystemExit("no race laps in keep range 2..max_lap")
    lap_col = np.full(len(d["pos_x"]), -1, dtype=np.int64)
    for lap_num, (s, e) in kept:
        lap_col[s:e] = lap_num
    d["lap"] = lap_col
    return d, kept


# ---------- Steering-rate derivations (same as plot_test3_ablation.py) ----------

def add_steer_rate(d, rate_limit=DEFAULT_STEERING_RATE_LIMIT_RAD_S):
    stamp_s = d["stamp_ns"].astype(np.float64) * 1e-9
    dt = np.diff(stamp_s, prepend=stamp_s[0])
    nominal_dt = float(np.median(dt[dt > 0])) if (dt > 0).any() else 0.005
    dt = np.where(dt > 0, dt, nominal_dt)

    command_dsteer = np.diff(d["steer"], prepend=d["steer"][0])
    d["command_steer_rate"]     = command_dsteer / dt
    d["abs_command_steer_rate"] = np.abs(d["command_steer_rate"])

    actual = np.empty_like(d["steer"])
    if len(actual):
        actual[0] = d["steer"][0]
        for i in range(1, len(actual)):
            max_delta = rate_limit * dt[i]
            steer_diff = d["steer"][i] - actual[i - 1]
            actual[i] = actual[i - 1] + np.clip(steer_diff, -max_delta, max_delta)
    d["actual_steer_est"] = actual
    actual_dsteer = np.diff(actual, prepend=actual[0])
    d["steer_rate"]     = actual_dsteer / dt
    d["abs_steer_rate"] = np.abs(d["steer_rate"])
    return d


def add_lateral_dev(d, ref_xy):
    pts = np.column_stack([d["pos_x"], d["pos_y"]])
    try:
        from scipy.spatial import cKDTree
        tree = cKDTree(ref_xy)
        dists, _ = tree.query(pts)
    except ImportError:
        dists = np.empty(len(pts))
        for i in range(0, len(pts), 1000):
            end = min(i + 1000, len(pts))
            d2 = ((pts[i:end, None, :] - ref_xy[None, :, :]) ** 2).sum(axis=2)
            dists[i:end] = np.sqrt(d2.min(axis=1))
    d["lateral_dev"] = dists
    return d


# ---------- Main ----------

COLUMNS = [
    "run", "stamp_ns", "lap", "pos_x", "pos_y",
    "speed", "steer", "actual_steer_est",
    "steer_rate", "abs_steer_rate",
    "command_steer_rate", "abs_command_steer_rate",
    "iterations", "lateral_dev",
]


def write_combined(out_path, runs):
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(COLUMNS)
        for run_name, d in runs:
            keep = d["lap"] > 0
            for i in np.where(keep)[0]:
                w.writerow([
                    run_name,
                    int(d["stamp_ns"][i]),
                    int(d["lap"][i]),
                    f"{d['pos_x'][i]:.6f}",
                    f"{d['pos_y'][i]:.6f}",
                    f"{d['speed'][i]:.6f}",
                    f"{d['steer'][i]:.6f}",
                    f"{d['actual_steer_est'][i]:.6f}",
                    f"{d['steer_rate'][i]:.6f}",
                    f"{d['abs_steer_rate'][i]:.6f}",
                    f"{d['command_steer_rate'][i]:.6f}",
                    f"{d['abs_command_steer_rate'][i]:.6f}",
                    f"{d['iterations'][i]:.0f}",
                    f"{d['lateral_dev'][i]:.6f}",
                ])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline-csv", required=True,
                    help="ablation_Baseline.csv from a Test 3 run")
    ap.add_argument("--steeroff-csv", required=True,
                    help="ablation_SteerOff.csv from a Test 3 run")
    ap.add_argument("--out", required=True, help="output combined CSV")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Keep race laps 2..MAX_LAP (default 11)")
    ap.add_argument("--steering-rate-limit", type=float,
                    default=DEFAULT_STEERING_RATE_LIMIT_RAD_S,
                    help="Actuator steering-rate limit [rad/s] (default 2.849)")
    args = ap.parse_args()

    base = drop_warmup(load_ablation(args.baseline_csv))
    steo = drop_warmup(load_ablation(args.steeroff_csv))

    base, base_kept = assign_laps(base, args.max_lap)
    steo, _         = assign_laps(steo, args.max_lap)
    print(f"Baseline: kept {len(base_kept)} race laps "
          f"({(base['lap'] > 0).sum()} rows)")
    print(f"SteerOff: kept {sum(1 for n in np.unique(steo['lap']) if n > 0)} race laps "
          f"({(steo['lap'] > 0).sum()} rows)")

    add_steer_rate(base, args.steering_rate_limit)
    add_steer_rate(steo, args.steering_rate_limit)

    # Reference path = Baseline's first kept lap (lap 2)
    _, (ref_s, ref_e) = base_kept[0]
    ref_xy = np.column_stack([base["pos_x"][ref_s:ref_e],
                              base["pos_y"][ref_s:ref_e]])
    add_lateral_dev(base, ref_xy)
    add_lateral_dev(steo, ref_xy)

    write_combined(Path(args.out), [("Baseline", base), ("SteerOff", steo)])
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
