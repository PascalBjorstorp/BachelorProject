#!/usr/bin/env python3
"""
MPC Weight Sweep (Comprehensive) — parallel evaluation for my_track.

Sweeps weight parameters AND solver/timing parameters with 16 parallel cores.

Weight sets:
  "Spielberg" = old defaults: Q_LAT=340, Q_HDG=1000, HORIZON=20, PRED_DT=0.04
  "Hardware"  = best found for my_track (determined by this sweep)

Usage:
    python3 weight_sweep.py [--cores 16]
"""

import subprocess
import csv
import os
import sys
import itertools
from concurrent.futures import ProcessPoolExecutor, as_completed
import time

# ─── Configuration ────────────────────────────────────────────────────────────

WORKSPACE = os.path.dirname(os.path.abspath(__file__))       # MPC/
BINARY = os.path.join(WORKSPACE, "test_sim_drive")
RACELINE = os.path.join(WORKSPACE, "..", "f1tenth_planning", "trajectories",
                        "my_track_raceline.csv")
OUTPUT_CSV = os.path.join(WORKSPACE, "sweep_results.csv")

N_CORES = 16

# ═══════════════════════════════════════════════════════════════════════════════
# Sweep Ranges
# ═══════════════════════════════════════════════════════════════════════════════

# --- Weight parameters ---
Q_LAT_VALUES   = [340, 1000, 3000, 5000, 7500, 10000, 15000]         # 7
Q_HDG_VALUES   = [500, 1000, 3000, 5000, 7500, 10000]                # 6
Q_VEL_VALUES   = [10, 26, 50]                                        # 3
R_STEER_VALUES = [0.01, 0.05, 0.15, 0.5, 1.0]                       # 5

# --- Solver parameters ---
HORIZON_VALUES  = [10, 15, 20]                                        # 3
PRED_DT_VALUES  = [0.03, 0.04, 0.05, 0.06]                           # 4
MAX_ITER_VALUES = [15, 20, 40]                                        # 3

# --- Additional weights (Phase 2 fine-tuning) ---
W_JERK_VALUES    = [0.1, 0.3, 1.0]                                   # 3
Q_LAT_VEL_VALUES = [30, 69, 150]                                     # 3

# Phase 1: 7*6*3*5 * 3*4*3 = 630 * 36 = 22,680 combinations
# Phase 2: top-50 * (3*3-1) = 50 * 8 = 400 extra combinations
# Total:   ~23,080

FIXED_ENV = {
    "Q_YAW":        "22",
    "R_ACCEL":      "0.01",
    "W_ACCEL_RATE": "0.1",
}

# ─── Runner ───────────────────────────────────────────────────────────────────

def run_one(params):
    """Run a single test_sim_drive. params is a dict of env overrides."""
    env = os.environ.copy()
    env.update(FIXED_ENV)
    for k, v in params.items():
        if k != "_label":
            env[k] = str(v)
    env["RACELINE_PATH"] = RACELINE
    env["MPC_TUNING_CSV"] = "1"

    try:
        result = subprocess.run(
            [BINARY],
            env=env,
            capture_output=True,
            text=True,
            timeout=180
        )
        stdout = result.stdout
    except subprocess.TimeoutExpired:
        out = dict(params)
        out.update({"crashed": 1, "error": "timeout"})
        return out
    except Exception as e:
        out = dict(params)
        out.update({"crashed": 1, "error": str(e)})
        return out

    # Parse CSV line
    csv_line = None
    for line in stdout.splitlines():
        if line.startswith("CSV,"):
            csv_line = line
            break

    if csv_line is None:
        crashed = "CRASH" in stdout or "WALL CRASH" in stdout
        out = dict(params)
        out.update({"crashed": 1 if crashed else -1, "error": "no_csv_output"})
        return out

    fields = csv_line.split(",")
    if len(fields) < 15:
        out = dict(params)
        out.update({"crashed": -1, "error": "bad_csv"})
        return out

    # Count steer reversals from stdout
    reversals = -1
    for line in stdout.splitlines():
        if "Steer reversals:" in line:
            try:
                reversals = int(line.split(":")[-1].strip())
            except ValueError:
                pass
            break

    out = dict(params)
    out.update({
        "tests_passed": int(fields[1]),
        "tests_failed": int(fields[2]),
        "max_lat_err": float(fields[3]),
        "avg_lat_err": float(fields[4]),
        "max_hdg_err": float(fields[5]),
        "avg_hdg_err": float(fields[6]),
        "max_vx": float(fields[7]),
        "avg_solve_us": float(fields[8]),
        "max_solve_us": float(fields[9]),
        "wall_collisions": int(fields[10]),
        "time_above_5ms": float(fields[11]),
        "max_vel_err": float(fields[12]),
        "avg_vel_err": float(fields[13]),
        "avg_iters": float(fields[14]),
        "steer_reversals": reversals,
        "crashed": 1 if int(fields[10]) > 0 else 0,
        "error": ""
    })
    return out


def run_batch(combos_dicts, cores, label=""):
    """Run a batch of configurations in parallel."""
    total = len(combos_dicts)
    print(f"\n{'='*60}")
    print(f"  {label}: {total} configurations, {cores} cores")
    print(f"  Est. time: ~{total / cores:.0f}s ({total / cores / 60:.1f} min)")
    print(f"{'='*60}")

    results = []
    completed = 0
    t0 = time.time()

    with ProcessPoolExecutor(max_workers=cores) as executor:
        futures = {executor.submit(run_one, d): d for d in combos_dicts}

        for future in as_completed(futures):
            completed += 1
            result = future.result()
            results.append(result)
            if completed % 200 == 0 or completed == total:
                n_ok = sum(1 for r in results if r.get("crashed", 1) == 0)
                n_crash = sum(1 for r in results if r.get("crashed", 1) == 1)
                elapsed = time.time() - t0
                rate = completed / elapsed if elapsed > 0 else 0
                eta = (total - completed) / rate if rate > 0 else 0
                print(f"  [{completed:5d}/{total}] OK={n_ok:4d} CRASH={n_crash:4d}  "
                      f"({rate:.0f}/s, ETA {eta:.0f}s)")

    elapsed = time.time() - t0
    n_ok = sum(1 for r in results if r.get("crashed", 1) == 0)
    n_crash = sum(1 for r in results if r.get("crashed", 1) == 1)
    print(f"  Done in {elapsed:.1f}s — OK={n_ok} CRASH={n_crash}")
    return results


# ─── Sorting ──────────────────────────────────────────────────────────────────

def sort_key(r):
    """Sort by: no-crash first, then avg_lat_err, then steer_reversals."""
    if r.get("crashed", 1) != 0:
        return (1, 999.0, 999)
    return (0, r.get("avg_lat_err", 999.0), r.get("steer_reversals", 999))


# ─── Main ─────────────────────────────────────────────────────────────────────

def main():
    cores = N_CORES
    for i, arg in enumerate(sys.argv):
        if arg == "--cores" and i + 1 < len(sys.argv):
            cores = int(sys.argv[i + 1])

    if not os.path.isfile(BINARY):
        print(f"ERROR: Binary not found: {BINARY}")
        sys.exit(1)
    if not os.path.isfile(RACELINE):
        print(f"ERROR: Raceline not found: {RACELINE}")
        sys.exit(1)

    print(f"=== MPC Comprehensive Weight Sweep ===")
    print(f"Binary:   {BINARY}")
    print(f"Raceline: {RACELINE}")
    print(f"Cores:    {cores}")

    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 1: Core grid (weights × solver × timing)
    # ═══════════════════════════════════════════════════════════════════════
    phase1_combos = list(itertools.product(
        Q_LAT_VALUES, Q_HDG_VALUES, Q_VEL_VALUES, R_STEER_VALUES,
        HORIZON_VALUES, PRED_DT_VALUES, MAX_ITER_VALUES
    ))
    phase1_dicts = []
    for q_lat, q_hdg, q_vel, r_steer, horizon, pred_dt, max_iter in phase1_combos:
        phase1_dicts.append({
            "Q_LAT": q_lat, "Q_HDG": q_hdg, "Q_VEL": q_vel,
            "R_STEER": r_steer,
            "HORIZON": horizon, "PRED_DT": pred_dt, "MAX_ITER": max_iter,
            "W_JERK": 0.3, "Q_LAT_VEL": 69,
            "_label": "phase1",
        })

    print(f"\nPhase 1 sweep ranges:")
    print(f"  Q_LAT:    {Q_LAT_VALUES}")
    print(f"  Q_HDG:    {Q_HDG_VALUES}")
    print(f"  Q_VEL:    {Q_VEL_VALUES}")
    print(f"  R_STEER:  {R_STEER_VALUES}")
    print(f"  HORIZON:  {HORIZON_VALUES}")
    print(f"  PRED_DT:  {PRED_DT_VALUES}")
    print(f"  MAX_ITER: {MAX_ITER_VALUES}")
    print(f"  (Fixed: W_JERK=0.3, Q_LAT_VEL=69, Q_YAW=22, R_ACCEL=0.01)")
    print(f"  Total Phase 1: {len(phase1_dicts)}")

    phase1_results = run_batch(phase1_dicts, cores, "Phase 1: Core Grid")

    # ═══════════════════════════════════════════════════════════════════════
    # PHASE 2: Fine-tune top-50 with W_JERK and Q_LAT_VEL variations
    # ═══════════════════════════════════════════════════════════════════════
    phase1_ok = [r for r in phase1_results if r.get("crashed", 1) == 0]
    phase1_ok.sort(key=sort_key)
    top_n = phase1_ok[:50]

    if len(top_n) > 0:
        phase2_dicts = []
        for base in top_n:
            for w_jerk in W_JERK_VALUES:
                for q_lat_vel in Q_LAT_VEL_VALUES:
                    # Skip the default combo (already tested in Phase 1)
                    if w_jerk == 0.3 and q_lat_vel == 69:
                        continue
                    d = {
                        "Q_LAT": base["Q_LAT"], "Q_HDG": base["Q_HDG"],
                        "Q_VEL": base["Q_VEL"], "R_STEER": base["R_STEER"],
                        "HORIZON": base["HORIZON"], "PRED_DT": base["PRED_DT"],
                        "MAX_ITER": base["MAX_ITER"],
                        "W_JERK": w_jerk, "Q_LAT_VEL": q_lat_vel,
                        "_label": "phase2",
                    }
                    phase2_dicts.append(d)

        phase2_results = run_batch(phase2_dicts, cores,
                                   f"Phase 2: Fine-tune top-{len(top_n)} with W_JERK/Q_LAT_VEL")
    else:
        phase2_results = []
        print("\nPhase 2 skipped: no surviving configs from Phase 1")

    # ═══════════════════════════════════════════════════════════════════════
    # Combine and sort all results
    # ═══════════════════════════════════════════════════════════════════════
    all_results = phase1_results + phase2_results
    all_results.sort(key=sort_key)

    # Write CSV
    fieldnames = ["Q_LAT", "Q_HDG", "Q_VEL", "R_STEER",
                  "HORIZON", "PRED_DT", "MAX_ITER", "W_JERK", "Q_LAT_VEL",
                  "crashed", "wall_collisions", "max_lat_err", "avg_lat_err",
                  "max_hdg_err", "avg_hdg_err", "steer_reversals",
                  "max_vx", "avg_vel_err", "max_vel_err",
                  "avg_solve_us", "max_solve_us", "avg_iters",
                  "tests_passed", "tests_failed", "time_above_5ms",
                  "_label", "error"]
    with open(OUTPUT_CSV, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for r in all_results:
            writer.writerow(r)

    # ═══════════════════════════════════════════════════════════════════════
    # Summary
    # ═══════════════════════════════════════════════════════════════════════
    total = len(all_results)
    n_ok = sum(1 for r in all_results if r.get("crashed", 1) == 0)
    n_crash = sum(1 for r in all_results if r.get("crashed", 1) == 1)
    print(f"\n{'='*60}")
    print(f"  SWEEP COMPLETE")
    print(f"  Total: {total}  |  No crash: {n_ok}  |  Crashed: {n_crash}")
    print(f"  Results: {OUTPUT_CSV}")
    print(f"{'='*60}")

    # Spielberg baseline
    spiel = [r for r in all_results
             if r.get("Q_LAT") == 340 and r.get("Q_HDG") == 1000
             and r.get("Q_VEL") == 26 and abs(float(r.get("R_STEER", 0)) - 0.15) < 0.001
             and r.get("HORIZON") == 20 and abs(float(r.get("PRED_DT", 0)) - 0.04) < 0.001
             and r.get("MAX_ITER") == 20]
    if spiel:
        s = spiel[0]
        print(f"\n  === 'Spielberg' Baseline ===")
        print(f"  Q_LAT=340, Q_HDG=1000, Q_VEL=26, R_STEER=0.15")
        print(f"  HORIZON=20, PRED_DT=0.04, MAX_ITER=20")
        print(f"  Crashed: {'YES' if s.get('crashed',1) else 'NO'}")
        if s.get("crashed", 1) == 0:
            print(f"  Max lat: {s.get('max_lat_err','?')}  Avg lat: {s.get('avg_lat_err','?')}")

    # Best "Hardware" config
    ok_results = [r for r in all_results if r.get("crashed", 1) == 0]
    if ok_results:
        best = ok_results[0]
        print(f"\n  === Best 'Hardware' Weight Set ===")
        for k in ["Q_LAT", "Q_HDG", "Q_VEL", "R_STEER", "HORIZON", "PRED_DT",
                   "MAX_ITER", "W_JERK", "Q_LAT_VEL"]:
            print(f"    {k:12s} = {best.get(k, '?')}")
        print(f"    max_lat_err    = {best.get('max_lat_err', '?')} m")
        print(f"    avg_lat_err    = {best.get('avg_lat_err', '?')} m")
        print(f"    steer_reversals= {best.get('steer_reversals', '?')}")
        print(f"    wall_collisions= {best.get('wall_collisions', '?')}")
        print(f"    max_vx         = {best.get('max_vx', '?')} m/s")
        print(f"    avg_solve_us   = {best.get('avg_solve_us', '?')} us")

        # Top 20
        print(f"\n  === Top 20 Configurations ===")
        header = (f"{'#':>3}  {'Q_LAT':>6}  {'Q_HDG':>6}  {'Q_VEL':>5}  {'R_STR':>5}  "
                  f"{'HOR':>3}  {'P_DT':>5}  {'ITER':>4}  {'JERK':>4}  {'QLAV':>4}  "
                  f"{'MaxLat':>7}  {'AvgLat':>7}  {'Rev':>4}  {'MaxVx':>5}  {'SlvUs':>6}")
        print(f"  {header}")
        for i, r in enumerate(ok_results[:20]):
            line = (f"{i+1:3d}  {float(r.get('Q_LAT',0)):6.0f}  {float(r.get('Q_HDG',0)):6.0f}  "
                    f"{float(r.get('Q_VEL',0)):5.0f}  {float(r.get('R_STEER',0)):5.2f}  "
                    f"{int(r.get('HORIZON',0)):3d}  {float(r.get('PRED_DT',0)):5.3f}  "
                    f"{int(r.get('MAX_ITER',0)):4d}  {float(r.get('W_JERK',0)):4.1f}  "
                    f"{float(r.get('Q_LAT_VEL',0)):4.0f}  "
                    f"{r.get('max_lat_err',0):7.4f}  {r.get('avg_lat_err',0):7.4f}  "
                    f"{r.get('steer_reversals',0):4d}  "
                    f"{r.get('max_vx',0):5.2f}  {r.get('avg_solve_us',0):6.1f}")
            print(f"  {line}")


if __name__ == "__main__":
    main()
