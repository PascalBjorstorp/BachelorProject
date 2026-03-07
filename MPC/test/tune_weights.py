#!/usr/bin/env python3
"""
MPC Weight Tuning Script — Exhaustive Search
=============================================
Systematically tests weight combinations for the Riccati-ADMM MPC controller.
Results are saved to a CSV report sorted by composite score.

Usage:
    python3 test/tune_weights.py                   # Full exhaustive sweep
    python3 test/tune_weights.py --quick            # Quick sweep (fewer combos)
    python3 test/tune_weights.py --single Q_LAT=100 Q_HDG=200

Environment: Requires test_sim_drive binary built in MPC/ directory.
"""

import subprocess
import os
import sys
import csv
import itertools
import time
from datetime import datetime

# ─── Env var names that match the C code in mpc_riccati.c ───────────────────
# MPC_W_LAT_ERROR   → weight_lateral_error     (Q_LAT)
# MPC_W_HEADING     → weight_heading_error      (Q_HDG)
# MPC_W_VELOCITY    → weight_velocity           (Q_VEL)
# MPC_W_LAT_VEL     → weight_lateral_velocity   (Q_LAT_VEL)
# MPC_W_YAW_RATE    → weight_yaw_rate           (Q_YAW)
# MPC_W_STEER_EFFORT → weight_steering_effort   (R_STEER)
# MPC_W_STEER_RATE  → weight_steering_rate      (W_JERK)
# MPC_W_TORQUE_RATE → weight_acceleration_rate  (W_ACCEL_RATE)

# ─── Base weights matching current codebase ─────────────────────────────────
BASE_WEIGHTS = {
    "Q_LAT":       125.0,
    "Q_HDG":       300.0,
    "Q_VEL":       30.0,
    "Q_LAT_VEL":   60.0,
    "Q_YAW":       20.0,
    "R_STEER":     0.35,
    "W_JERK":      0.5,
    "W_ACCEL_RATE": 0.01,
}

# Friendly name → C env name used by test_sim_drive.c
# The test binary reads these directly (line ~240 of test_sim_drive.c),
# NOT the MPC_W_* names from mpc_riccati.c's get_default_configuration().
WEIGHT_TO_ENV = {
    "Q_LAT":        "Q_LAT",
    "Q_HDG":        "Q_HDG",
    "Q_VEL":        "Q_VEL",
    "Q_LAT_VEL":    "Q_LAT_VEL",
    "Q_YAW":        "Q_YAW",
    "R_STEER":      "R_STEER",
    "W_JERK":       "W_JERK",
    "W_ACCEL_RATE": "W_ACCEL_RATE",
}

# ─── Weight value ranges for exhaustive search ──────────────────────────────
WEIGHT_VALUES = {
    "Q_LAT":       [25, 50, 75, 100, 125, 150, 175, 200, 250, 300],
    "Q_HDG":       [25, 50, 75, 100, 125, 150, 175, 200, 250, 300],
    "Q_VEL":       [4, 6, 8, 10, 12, 16, 20, 30, 40, 50],
    "Q_LAT_VEL":   [10, 30, 60, 100, 150],
    "Q_YAW":       [1, 3, 5, 10, 20],
    "R_STEER":     [0.1, 0.2, 0.35, 0.5, 1.0],
    "W_JERK":      [0.5, 1.5, 2.5, 5.0, 10.0],
    "W_ACCEL_RATE": [0.001, 0.01, 0.1],
}

QUICK_VALUES = {
    "Q_LAT":       [50, 75, 100, 150],
    "Q_HDG":       [60, 100, 150, 200],
    "Q_VEL":       [8, 12, 20, 30],
    "Q_LAT_VEL":   [30, 60, 100],
    "Q_YAW":       [3, 5, 10],
    "R_STEER":     [0.2, 0.35, 0.5],
    "W_JERK":      [1.5, 2.5, 5.0],
}


def run_test(weights: dict, binary: str = "./test_sim_drive") -> dict:
    """Run a single test with given weights, return parsed results."""
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"

    # Map friendly names to C env var names
    for friendly, value in weights.items():
        env_name = WEIGHT_TO_ENV.get(friendly)
        if env_name:
            env[env_name] = str(value)

    try:
        result = subprocess.run(
            [binary],
            capture_output=True, text=True, timeout=120, env=env
        )
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "passed": 0, "failed": 6}
    except FileNotFoundError:
        print(f"ERROR: Binary '{binary}' not found. Build first.")
        sys.exit(1)

    # Parse CSV line from output
    for line in result.stdout.splitlines():
        if line.startswith("CSV,"):
            parts = line.split(",")
            try:
                return {
                    "status": "OK",
                    "passed": int(parts[1]),
                    "failed": int(parts[2]),
                    "max_lat_err": float(parts[3]),
                    "avg_lat_err": float(parts[4]),
                    "max_hdg_err": float(parts[5]),
                    "avg_hdg_err": float(parts[6]),
                    "max_vx": float(parts[7]),
                    "avg_solve_us": float(parts[8]),
                    "max_solve_us": float(parts[9]),
                    "wall_collisions": int(parts[10]),
                    "time_above_5ms": float(parts[11]),
                    "max_vel_err": float(parts[12]) if len(parts) > 12 else 12.0,
                    "avg_vel_err": float(parts[13]) if len(parts) > 13 else 5.0,
                    "avg_iters": float(parts[14]) if len(parts) > 14 else 0.0,
                }
            except (IndexError, ValueError):
                pass

    return {"status": "NO_CSV", "passed": 0, "failed": 6}


def compute_score(r: dict) -> float:
    """Composite score (lower = better).

    Priority hierarchy:
      Catastrophic: wall_collisions, test failures  → instant penalty
      Primary:      time_above_5ms (speed), avg_vel_err (velocity tracking)
      Secondary:    avg_lat_err (lateral safety)
      Tertiary:     avg_iters (computation efficiency)
    """
    if r["status"] != "OK" or r["failed"] > 0:
        return 999.0

    # Catastrophic: wall collisions
    if r["wall_collisions"] > 0:
        return 500.0 + r["wall_collisions"] * 100.0

    score = (
        # PRIMARY — speed & velocity tracking (70% of score budget)
        r["avg_vel_err"] * 20.0 +                     # velocity tracking error
        r["max_vel_err"] * 4.0 +                       # worst-case velocity error
        max(0, 25 - r["time_above_5ms"]) * 3.0 +       # penalize slow driving
        max(0, 12.0 - r["max_vx"]) * 8.0 +             # penalize not reaching speed

        # SECONDARY — lateral safety (20% of score budget)
        r["avg_lat_err"] * 5.0 +                       # lateral deviation
        r["max_lat_err"] * 1.5 +                       # worst-case lateral
        r["avg_hdg_err"] * 2.0 +                       # heading tracking

        # TERTIARY — computation (10% of score budget)
        r.get("avg_iters", 0) * 0.5 +                  # fewer iterations better
        r["avg_solve_us"] * 0.003                      # solver cost
    )
    return round(score, 3)


def generate_one_at_a_time(values_dict):
    """One weight varied at a time, others at baseline."""
    combos = [("BASELINE", dict(BASE_WEIGHTS))]

    for wname, values in values_dict.items():
        for v in values:
            if abs(v - BASE_WEIGHTS.get(wname, -999)) < 1e-6:
                continue
            w = dict(BASE_WEIGHTS)
            w[wname] = v
            combos.append((f"{wname}={v}", w))

    return combos


def generate_pairwise(values_dict):
    """All pairwise combinations of weight values."""
    combos = []
    weight_names = list(values_dict.keys())

    for w1, w2 in itertools.combinations(weight_names, 2):
        vals1 = values_dict[w1]
        vals2 = values_dict[w2]
        for v1 in vals1:
            for v2 in vals2:
                # Skip if both are baseline
                if (abs(v1 - BASE_WEIGHTS.get(w1, -999)) < 1e-6 and
                    abs(v2 - BASE_WEIGHTS.get(w2, -999)) < 1e-6):
                    continue
                w = dict(BASE_WEIGHTS)
                w[w1] = v1
                w[w2] = v2
                combos.append((f"{w1}={v1}+{w2}={v2}", w))

    return combos


def generate_triple_grid():
    """Exhaustive grid over the 3 most impactful weights: Q_LAT, Q_HDG, Q_VEL."""
    combos = []
    lat_vals = [25, 50, 75, 100, 125, 150, 175, 200, 250, 300]
    hdg_vals = [25, 50, 75, 100, 125, 150, 175, 200, 250, 300]
    vel_vals = [4, 6, 8, 10, 12, 16, 20, 30, 40, 50]

    for ql in lat_vals:
        for qh in hdg_vals:
            for qv in vel_vals:
                if ql == 100 and qh == 100 and qv == 12:
                    continue
                w = dict(BASE_WEIGHTS)
                w["Q_LAT"] = ql
                w["Q_HDG"] = qh
                w["Q_VEL"] = qv
                combos.append((f"LAT={ql}+HDG={qh}+VEL={qv}", w))

    return combos


def generate_secondary_sweep(best_primary):
    """Sweep secondary weights on top of the best primary combination."""
    combos = []
    secondary = {
        "Q_LAT_VEL": [10, 30, 60, 100, 150],
        "Q_YAW":     [1, 3, 5, 10, 20],
        "R_STEER":   [0.1, 0.2, 0.35, 0.5, 1.0],
        "W_JERK":    [0.5, 1.5, 2.5, 5.0, 10.0],
    }

    # One-at-a-time secondary on best primary
    for wname, values in secondary.items():
        for v in values:
            if abs(v - best_primary.get(wname, BASE_WEIGHTS.get(wname, -999))) < 1e-6:
                continue
            w = dict(best_primary)
            w[wname] = v
            combos.append((f"BEST+{wname}={v}", w))

    # Pairwise secondary on best primary
    sec_names = list(secondary.keys())
    for s1, s2 in itertools.combinations(sec_names, 2):
        for v1 in secondary[s1][::2]:  # Every other value to limit combos
            for v2 in secondary[s2][::2]:
                w = dict(best_primary)
                w[s1] = v1
                w[s2] = v2
                combos.append((f"BEST+{s1}={v1}+{s2}={v2}", w))

    return combos


def generate_velocity_focused():
    """Aggressively velocity-focused combinations."""
    combos = []
    configs = [
        {"Q_VEL": 20, "Q_LAT": 50, "Q_HDG": 60},
        {"Q_VEL": 40, "Q_LAT": 50, "Q_HDG": 60},
        {"Q_VEL": 60, "Q_LAT": 50, "Q_HDG": 60},
        {"Q_VEL": 80, "Q_LAT": 40, "Q_HDG": 50},
        {"Q_VEL": 100, "Q_LAT": 40, "Q_HDG": 50},
        {"Q_VEL": 20, "Q_LAT": 75, "Q_HDG": 100},
        {"Q_VEL": 40, "Q_LAT": 75, "Q_HDG": 100},
        {"Q_VEL": 80, "Q_LAT": 75, "Q_HDG": 100},
        {"Q_VEL": 100, "Q_LAT": 75, "Q_HDG": 100},
        {"Q_VEL": 40, "Q_LAT_VEL": 20},
        {"Q_VEL": 60, "Q_LAT_VEL": 20},
        {"Q_VEL": 80, "Q_LAT_VEL": 10},
        {"Q_VEL": 100, "Q_LAT_VEL": 10},
        {"Q_VEL": 40, "R_STEER": 0.1, "W_JERK": 1.0},
        {"Q_VEL": 60, "R_STEER": 0.1, "W_JERK": 1.0},
        {"Q_VEL": 80, "R_STEER": 0.1, "W_JERK": 0.5},
        {"Q_VEL": 100, "Q_LAT": 50, "Q_HDG": 80, "Q_LAT_VEL": 20, "R_STEER": 0.1},
        {"Q_VEL": 80, "Q_LAT": 60, "Q_HDG": 100, "Q_LAT_VEL": 30, "R_STEER": 0.2},
        {"Q_VEL": 50, "Q_LAT": 75, "Q_HDG": 100, "Q_LAT_VEL": 40, "R_STEER": 0.2},
    ]
    for cfg in configs:
        w = dict(BASE_WEIGHTS)
        w.update(cfg)
        label = "+".join(f"{k}={v}" for k, v in cfg.items())
        combos.append((label, w))
    return combos


def generate_fine_tuning(best_weights):
    """Phase 3: Fine-tuning around the best found configuration.

    For each weight, test ±10%, ±25%, ±50% of its current value.
    Then test all pairwise ±10% combinations.
    """
    combos = []
    perturbations = [0.50, 0.75, 0.90, 1.10, 1.25, 1.50]

    # Single-weight perturbations: 8 weights × 6 perturbations = 48
    for wname, base_val in best_weights.items():
        if base_val == 0:
            continue
        for mult in perturbations:
            new_val = round(base_val * mult, 6)
            w = dict(best_weights)
            w[wname] = new_val
            pct = int((mult - 1.0) * 100)
            sign = "+" if pct >= 0 else ""
            combos.append((f"FT:{wname}{sign}{pct}%", w))

    # Pairwise ±10% combinations: C(8,2) × 4 = 112
    weight_names = list(best_weights.keys())
    pair_mults = [0.90, 1.10]
    for w1, w2 in itertools.combinations(weight_names, 2):
        v1_base = best_weights[w1]
        v2_base = best_weights[w2]
        if v1_base == 0 or v2_base == 0:
            continue
        for m1 in pair_mults:
            for m2 in pair_mults:
                w = dict(best_weights)
                w[w1] = round(v1_base * m1, 6)
                w[w2] = round(v2_base * m2, 6)
                p1 = "+10%" if m1 > 1 else "-10%"
                p2 = "+10%" if m2 > 1 else "-10%"
                combos.append((f"FT:{w1}{p1}+{w2}{p2}", w))

    return combos


def deduplicate(combos):
    """Remove duplicate weight combinations."""
    seen = set()
    unique = []
    for label, weights in combos:
        key = tuple(sorted(weights.items()))
        if key not in seen:
            seen.add(key)
            unique.append((label, weights))
    return unique


def main():
    quick = "--quick" in sys.argv
    single = "--single" in sys.argv

    script_dir = os.path.dirname(os.path.abspath(__file__))
    mpc_dir = os.path.dirname(script_dir)
    os.chdir(mpc_dir)
    binary = "./test_sim_drive"

    if not os.path.exists(binary):
        print("Building test_sim_drive...")
        subprocess.run([
            "gcc", "-D_GNU_SOURCE", "-O3", "-std=c99", "-Wall", "-ffast-math",
            "-Wno-unused-variable", "-Wno-unused-but-set-variable",
            "-Iinclude",
            "test/test_sim_drive.c", "src/mpc_riccati.c", "src/riccati_solver.c",
            "src/vehicle_model.c", "src/fp_math.c",
            "-o", "test_sim_drive", "-lm"
        ], check=True)

    if single:
        weights = dict(BASE_WEIGHTS)
        for arg in sys.argv[2:]:
            if "=" in arg:
                k, v = arg.split("=", 1)
                weights[k] = float(v)
        print(f"Testing: {weights}")
        r = run_test(weights, binary)
        score = compute_score(r)
        print(f"Result: {r}")
        print(f"Score:  {score}")
        return

    # ─── Phase 1: Generate all combinations ─────────────────────────────
    values = QUICK_VALUES if quick else WEIGHT_VALUES

    combos = []
    combos += generate_one_at_a_time(values)
    combos += generate_pairwise(values)
    if not quick:
        combos += generate_triple_grid()
        combos += generate_velocity_focused()

    combos = deduplicate(combos)
    total = len(combos)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_results_{timestamp}.csv"

    print(f"\n{'='*80}")
    mode = "QUICK" if quick else "EXHAUSTIVE"
    print(f"MPC Weight Tuning — {mode} — {total} Phase 1 combinations")
    print(f"{'='*80}")
    print(f"Base: Q_LAT={BASE_WEIGHTS['Q_LAT']}, Q_HDG={BASE_WEIGHTS['Q_HDG']}, "
          f"Q_VEL={BASE_WEIGHTS['Q_VEL']}, Q_LAT_VEL={BASE_WEIGHTS['Q_LAT_VEL']}, "
          f"Q_YAW={BASE_WEIGHTS['Q_YAW']}, R_STEER={BASE_WEIGHTS['R_STEER']}, "
          f"W_JERK={BASE_WEIGHTS['W_JERK']}")
    print()

    results = []
    t0 = time.time()
    passed_count = 0
    failed_count = 0

    for i, (label, weights) in enumerate(combos):
        elapsed = time.time() - t0
        rate = (i + 1) / max(elapsed, 0.01)
        eta = (total - i - 1) / max(rate, 0.01)
        print(f"[{i+1:4d}/{total}] {label:50s} ", end="", flush=True)

        r = run_test(weights, binary)
        score = compute_score(r)
        r["label"] = label
        r["score"] = score
        r.update(weights)
        results.append(r)

        if r["status"] != "OK":
            failed_count += 1
            print(f"  -> {r['status']}  (ETA {eta:.0f}s)")
        elif r["failed"] > 0:
            failed_count += 1
            print(f"  -> FAIL {r['failed']}  lat={r.get('max_lat_err','?'):.3f}  "
                  f"walls={r.get('wall_collisions','?')}  (ETA {eta:.0f}s)")
        else:
            passed_count += 1
            print(f"  -> PASS  sc={score:6.2f}  "
                  f"vErr={r['avg_vel_err']:.2f}  "
                  f"lat={r['avg_lat_err']:.3f}/{r['max_lat_err']:.3f}  "
                  f"vx={r['max_vx']:.1f}  "
                  f"t5={r['time_above_5ms']:.0f}s  (ETA {eta:.0f}s)")

    # ─── Phase 2: Secondary sweep on best primary result ────────────────
    if results and not quick:
        primary_results = sorted(results, key=lambda x: x.get("score", 999))
        best = primary_results[0]
        if best.get("score", 999) < 999:
            print(f"\n{'='*80}")
            print(f"Phase 2: Secondary sweep on best ({best['label']}, score={best['score']:.2f})")
            print(f"{'='*80}")

            best_weights = {k: best[k] for k in BASE_WEIGHTS.keys()}
            phase2_combos = generate_secondary_sweep(best_weights)
            phase2_combos = deduplicate(phase2_combos)
            p2_total = len(phase2_combos)

            for i, (label, weights) in enumerate(phase2_combos):
                print(f"[P2 {i+1:3d}/{p2_total}] {label:50s} ", end="", flush=True)
                r = run_test(weights, binary)
                score = compute_score(r)
                r["label"] = label
                r["score"] = score
                r.update(weights)
                results.append(r)

                if r["status"] != "OK" or r["failed"] > 0:
                    failed_count += 1
                    print(f"  -> FAIL")
                else:
                    passed_count += 1
                    print(f"  -> PASS  sc={score:6.2f}  "
                          f"vErr={r['avg_vel_err']:.2f}  "
                          f"lat={r['avg_lat_err']:.3f}/{r['max_lat_err']:.3f}")

    # ─── Phase 3: Fine-tuning around the best found ─────────────────────
    if results and not quick:
        all_sorted = sorted(results, key=lambda x: x.get("score", 999))
        best_so_far = all_sorted[0]
        if best_so_far.get("score", 999) < 999:
            best_weights = {k: best_so_far[k] for k in BASE_WEIGHTS.keys()}
            phase3_combos = generate_fine_tuning(best_weights)
            phase3_combos = deduplicate(phase3_combos)

            # Also remove any already-tested combinations
            tested_keys = set()
            for r in results:
                key = tuple(sorted((k, r.get(k)) for k in BASE_WEIGHTS.keys()))
                tested_keys.add(key)
            phase3_combos = [(l, w) for l, w in phase3_combos
                             if tuple(sorted(w.items())) not in tested_keys]

            p3_total = len(phase3_combos)
            print(f"\n{'='*80}")
            print(f"Phase 3: Fine-tuning ({p3_total} combos) around best ")
            print(f"  {best_so_far['label']} (score={best_so_far['score']:.2f})")
            print(f"  Weights: " + ", ".join(f"{k}={best_weights[k]}" for k in BASE_WEIGHTS.keys()))
            print(f"{'='*80}")

            for i, (label, weights) in enumerate(phase3_combos):
                elapsed = time.time() - t0
                print(f"[P3 {i+1:3d}/{p3_total}] {label:50s} ", end="", flush=True)
                r = run_test(weights, binary)
                score = compute_score(r)
                r["label"] = label
                r["score"] = score
                r.update(weights)
                results.append(r)

                if r["status"] != "OK" or r["failed"] > 0:
                    failed_count += 1
                    print(f"  -> FAIL")
                else:
                    passed_count += 1
                    print(f"  -> PASS  sc={score:6.2f}  "
                          f"vErr={r['avg_vel_err']:.2f}  "
                          f"lat={r['avg_lat_err']:.3f}/{r['max_lat_err']:.3f}  "
                          f"t5={r['time_above_5ms']:.0f}s")

    # ─── Sort and report ────────────────────────────────────────────────
    results.sort(key=lambda x: x.get("score", 999))

    elapsed = time.time() - t0
    print(f"\n{'='*80}")
    print(f"Completed {len(results)} tests in {elapsed:.1f}s "
          f"({passed_count} passed, {failed_count} failed)")
    print(f"{'='*80}")

    # Write CSV
    if results:
        fieldnames = (["label", "score", "passed", "failed",
                       "max_lat_err", "avg_lat_err", "max_hdg_err", "avg_hdg_err",
                       "max_vx", "avg_vel_err", "max_vel_err",
                       "avg_solve_us", "max_solve_us",
                       "wall_collisions", "time_above_5ms", "avg_iters", "status"]
                      + list(BASE_WEIGHTS.keys()))
        with open(outfile, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"Results saved to: {outfile}")

    # Print top 20
    print(f"\n{'='*80}")
    print("TOP 20 CONFIGURATIONS (lowest score = best)")
    print(f"{'='*80}")
    fmt = "{:<4} {:<50} {:>7} {:>7} {:>6} {:>7} {:>7} {:>5} {:>5}"
    print(fmt.format("Rank", "Label", "Score", "AvgVel", "MaxVx",
                      "MaxLat", "AvgLat", "T>5s", "Walls"))
    print("-" * 108)
    passing_results = [r for r in results if r.get("score", 999) < 999]
    for i, r in enumerate(passing_results[:20]):
        print(fmt.format(
            i+1, r['label'][:50], f"{r['score']:.2f}",
            f"{r['avg_vel_err']:.3f}", f"{r['max_vx']:.1f}",
            f"{r['max_lat_err']:.3f}", f"{r['avg_lat_err']:.3f}",
            f"{r['time_above_5ms']:.0f}", f"{r['wall_collisions']}"))

    if passing_results:
        best = passing_results[0]
        print(f"\nBest: {best['label']} (score={best['score']:.2f})")
        print(f"  Q_LAT={best['Q_LAT']}, Q_HDG={best['Q_HDG']}, Q_VEL={best['Q_VEL']}, "
              f"Q_LAT_VEL={best['Q_LAT_VEL']}, Q_YAW={best['Q_YAW']}, "
              f"R_STEER={best['R_STEER']}, W_JERK={best['W_JERK']}")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
