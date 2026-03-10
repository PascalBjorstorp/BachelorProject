#!/usr/bin/env python3
"""
MPC Weight Tuning Script — Thorough Multi-Phase Search
=======================================================
Systematically tests weight + solver + wall combinations for the CPU
Riccati-ADMM MPC controller.  Results are saved to a CSV report.

Phases:
  1. One-at-a-time sweep of ALL tuneable parameters (wide range)
  2. Full grid over primary weights (Q_LAT × Q_HDG × Q_VEL)
  3. Secondary weights grid (Q_LAT_VEL × Q_YAW × R_STEER × W_JERK)
  4. Solver parameter grid (RHO × RHO_U × ALPHA)
  5. Wall parameter grid (WALL_END × WALL_SOFT_K)
  6. Velocity-focused configurations
  7. Fine-tuning ±5/10/25/50% around best found config
  8. Random perturbation around best (100 random neighbors)

Usage:
    python3 test/tune_weights.py                   # Full thorough sweep
    python3 test/tune_weights.py --quick            # Quick sweep
    python3 test/tune_weights.py --single Q_LAT=100 Q_HDG=200
"""

import subprocess
import os
import sys
import csv
import itertools
import time
import random
from datetime import datetime

# ─── Tunable parameters and their env var names ─────────────────────────────
ALL_PARAMS = {
    # State weights (from test_sim_drive.c)
    "Q_LAT":        "Q_LAT",
    "Q_HDG":        "Q_HDG",
    "Q_VEL":        "Q_VEL",
    "Q_LAT_VEL":    "Q_LAT_VEL",
    "Q_YAW":        "Q_YAW",
    # Control weights
    "R_STEER":      "R_STEER",
    "R_ACCEL":      "R_ACCEL",
    "W_JERK":       "W_JERK",
    "W_ACCEL_RATE": "W_ACCEL_RATE",
    # Solver parameters (from mpc_riccati.c)
    "RHO":          "RHO",
    "RHO_U":        "RHO_U",
    "ALPHA":        "ALPHA",
    "TOL":          "TOL",
    "MAX_ITER":     "MAX_ITER",
    # Wall constraints (from mpc_riccati.c)
    "WALL_END":     "WALL_END",
    "WALL_SOFT_K":  "WALL_SOFT_K",
}

# ─── Current best configuration (CPU sweep winner) ─────────────────────────
BASE = {
    "Q_LAT":        270.0,
    "Q_HDG":        660.0,
    "Q_VEL":        50.0,
    "Q_LAT_VEL":    60.0,
    "Q_YAW":        22.0,
    "R_STEER":      0.15,
    "R_ACCEL":      0.01,
    "W_JERK":       0.3,
    "W_ACCEL_RATE": 0.1,
    "RHO":          32.0,
    "RHO_U":        20.0,
    "ALPHA":        0.93,
    "TOL":          5.0,
    "MAX_ITER":     20,
    "WALL_END":     6,
    "WALL_SOFT_K":  3000.0,
}

# ─── Sweep ranges ───────────────────────────────────────────────────────────
FULL_VALUES = {
    "Q_LAT":        [50, 100, 150, 200, 250, 300, 400, 500, 700, 1000],
    "Q_HDG":        [100, 200, 400, 600, 800, 1000, 1500, 2000, 3000],
    "Q_VEL":        [5, 10, 15, 20, 30, 50, 80, 100, 150],
    "Q_LAT_VEL":    [10, 20, 40, 60, 80, 100, 150, 200],
    "Q_YAW":        [1, 5, 10, 20, 40, 60, 100],
    "R_STEER":      [0.05, 0.10, 0.15, 0.25, 0.4, 0.6, 1.0],
    "R_ACCEL":      [0.001, 0.005, 0.01, 0.05, 0.1],
    "W_JERK":       [0.05, 0.1, 0.2, 0.3, 0.5, 1.0, 2.0, 5.0],
    "W_ACCEL_RATE": [0.01, 0.05, 0.1, 0.3, 0.5, 1.0],
    "RHO":          [5, 10, 15, 20, 30, 50, 80, 100],
    "RHO_U":        [5, 10, 15, 20, 30, 50, 80],
    "ALPHA":        [1.0, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8],
    "TOL":          [1.0, 3.0, 5.0, 10.0, 20.0],
    "MAX_ITER":     [5, 10, 15, 20, 30, 50],
    "WALL_END":     [3, 5, 8, 10, 12, 15, 20],
    "WALL_SOFT_K":  [0, 500, 1000, 3000, 5000, 10000],
}

QUICK_VALUES = {
    "Q_LAT":        [150, 300, 500],
    "Q_HDG":        [500, 1000, 2000],
    "Q_VEL":        [15, 30, 60],
    "Q_LAT_VEL":    [30, 60, 100],
    "Q_YAW":        [10, 20, 40],
    "R_STEER":      [0.10, 0.15, 0.25],
    "W_JERK":       [0.1, 0.3, 0.5],
    "RHO":          [15, 30, 50],
    "RHO_U":        [10, 20, 30],
    "ALPHA":        [1.0, 1.2, 1.4],
    "WALL_END":     [5, 8, 12],
    "WALL_SOFT_K":  [0, 3000],
}


def run_test(params: dict, binary: str) -> dict:
    """Run a single test with given parameters, return parsed results."""
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"

    for name, value in params.items():
        env_name = ALL_PARAMS.get(name, name)
        env[env_name] = str(value)

    try:
        result = subprocess.run(
            [binary], capture_output=True, text=True, timeout=180, env=env
        )
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "passed": 0, "failed": 6}
    except FileNotFoundError:
        print(f"ERROR: Binary '{binary}' not found.")
        sys.exit(1)

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

    Priority:
      Catastrophic: wall collisions → instant penalty
      Primary:      time_above_5ms, avg_vel_err
      Secondary:    avg_lat_err, max_lat_err
      Tertiary:     avg_iters, solve time
    """
    if r["status"] != "OK" or r["failed"] > 0:
        return 999.0

    if r["wall_collisions"] > 0:
        return 500.0 + r["wall_collisions"] * 100.0

    score = (
        r["avg_vel_err"] * 20.0 +
        r["max_vel_err"] * 3.0 +
        max(0, 30 - r["time_above_5ms"]) * 4.0 +
        max(0, 12.0 - r["max_vx"]) * 8.0 +
        r["avg_lat_err"] * 8.0 +
        r["max_lat_err"] * 2.0 +
        r["avg_hdg_err"] * 3.0 +
        r.get("avg_iters", 0) * 0.5 +
        r["avg_solve_us"] * 0.003
    )
    return round(score, 3)


# ─── Combination generators ─────────────────────────────────────────────────

def gen_one_at_a_time(values_dict):
    """Vary each parameter one at a time, others at baseline."""
    combos = [("BASELINE", dict(BASE))]
    for name, values in values_dict.items():
        for v in values:
            if abs(v - BASE.get(name, -999)) < 1e-6:
                continue
            w = dict(BASE)
            w[name] = v
            combos.append((f"{name}={v}", w))
    return combos


def gen_primary_grid(values_dict):
    """Full grid over Q_LAT × Q_HDG × Q_VEL."""
    combos = []
    for ql in values_dict.get("Q_LAT", [BASE["Q_LAT"]]):
        for qh in values_dict.get("Q_HDG", [BASE["Q_HDG"]]):
            for qv in values_dict.get("Q_VEL", [BASE["Q_VEL"]]):
                w = dict(BASE)
                w["Q_LAT"] = ql
                w["Q_HDG"] = qh
                w["Q_VEL"] = qv
                combos.append((f"L={ql}+H={qh}+V={qv}", w))
    return combos


def gen_secondary_grid():
    """Grid over secondary weights around baseline primary."""
    combos = []
    for qlv in [20, 40, 60, 80, 120]:
        for qy in [5, 10, 20, 40]:
            for rs in [0.08, 0.15, 0.25, 0.4]:
                for wj in [0.1, 0.3, 0.5, 1.0]:
                    w = dict(BASE)
                    w["Q_LAT_VEL"] = qlv
                    w["Q_YAW"] = qy
                    w["R_STEER"] = rs
                    w["W_JERK"] = wj
                    combos.append((f"LV={qlv}+Y={qy}+RS={rs}+WJ={wj}", w))
    return combos


def gen_solver_grid():
    """Grid over ADMM solver parameters."""
    combos = []
    for rho in [10, 20, 30, 50, 80]:
        for rho_u in [5, 10, 20, 30, 50]:
            for alpha in [1.0, 1.1, 1.2, 1.3, 1.5]:
                w = dict(BASE)
                w["RHO"] = rho
                w["RHO_U"] = rho_u
                w["ALPHA"] = alpha
                combos.append((f"rho={rho}+ru={rho_u}+a={alpha}", w))
    return combos


def gen_wall_grid():
    """Grid over wall constraint parameters."""
    combos = []
    for we in [3, 5, 8, 10, 15, 20]:
        for wk in [0, 1000, 3000, 5000, 10000]:
            w = dict(BASE)
            w["WALL_END"] = we
            w["WALL_SOFT_K"] = wk
            combos.append((f"WE={we}+SK={wk}", w))
    return combos


def gen_velocity_focused():
    """Aggressively velocity-focused configurations."""
    combos = []
    configs = [
        {"Q_VEL": 50, "Q_LAT": 200, "Q_HDG": 700},
        {"Q_VEL": 80, "Q_LAT": 200, "Q_HDG": 700},
        {"Q_VEL": 100, "Q_LAT": 200, "Q_HDG": 700},
        {"Q_VEL": 150, "Q_LAT": 200, "Q_HDG": 700},
        {"Q_VEL": 50, "Q_LAT": 300, "Q_HDG": 1000},
        {"Q_VEL": 80, "Q_LAT": 300, "Q_HDG": 1000},
        {"Q_VEL": 100, "Q_LAT": 300, "Q_HDG": 1000, "R_STEER": 0.10},
        {"Q_VEL": 150, "Q_LAT": 300, "Q_HDG": 1000, "R_STEER": 0.10},
        {"Q_VEL": 80, "Q_LAT": 500, "Q_HDG": 1500},
        {"Q_VEL": 100, "Q_LAT": 500, "Q_HDG": 1500},
        {"Q_VEL": 50, "Q_LAT_VEL": 30, "Q_YAW": 10},
        {"Q_VEL": 80, "Q_LAT_VEL": 30, "Q_YAW": 10},
        {"Q_VEL": 100, "Q_LAT_VEL": 20, "Q_YAW": 5},
        {"Q_VEL": 100, "Q_LAT": 400, "Q_HDG": 1200, "R_STEER": 0.10, "W_JERK": 0.1},
        {"Q_VEL": 150, "Q_LAT": 400, "Q_HDG": 1200, "R_STEER": 0.10, "W_JERK": 0.1},
        {"Q_VEL": 80, "Q_LAT": 300, "Q_HDG": 800, "ALPHA": 1.3},
        {"Q_VEL": 100, "Q_LAT": 300, "Q_HDG": 800, "RHO": 50, "RHO_U": 30},
    ]
    for cfg in configs:
        w = dict(BASE)
        w.update(cfg)
        label = "+".join(f"{k}={v}" for k, v in cfg.items())
        combos.append((label, w))
    return combos


def gen_fine_tuning(best_weights):
    """Fine-tuning around best config: ±5/10/25/50% for each parameter."""
    combos = []
    perturbations = [0.50, 0.75, 0.90, 0.95, 1.05, 1.10, 1.25, 1.50]

    for name, base_val in best_weights.items():
        if base_val == 0 or name in ("MAX_ITER",):
            continue
        for mult in perturbations:
            new_val = round(base_val * mult, 6)
            if name in ("WALL_END", "MAX_ITER"):
                new_val = max(1, int(new_val))
            elif name == "WALL_SOFT_K":
                new_val = max(0, round(new_val))
            w = dict(best_weights)
            w[name] = new_val
            pct = int((mult - 1.0) * 100)
            sign = "+" if pct >= 0 else ""
            combos.append((f"FT:{name}{sign}{pct}%", w))

    # Pairwise ±10% of the 6 most important parameters
    key_params = ["Q_LAT", "Q_HDG", "Q_VEL", "R_STEER", "RHO", "ALPHA"]
    pair_mults = [0.90, 1.10]
    for w1, w2 in itertools.combinations(key_params, 2):
        v1_base = best_weights.get(w1, 0)
        v2_base = best_weights.get(w2, 0)
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


def gen_random_neighbors(best_weights, n=100):
    """Random perturbations around best config."""
    combos = []
    random.seed(42)
    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ITER", "WALL_SOFT_K") and best_weights[k] != 0]
    for i in range(n):
        w = dict(best_weights)
        num_perturb = random.randint(2, min(5, len(tune_params)))
        params_to_perturb = random.sample(tune_params, num_perturb)
        for name in params_to_perturb:
            mult = random.uniform(0.6, 1.5)
            w[name] = round(w[name] * mult, 6)
            if name in ("WALL_END",):
                w[name] = max(1, int(w[name]))
        combos.append((f"RND_{i}", w))
    return combos


def deduplicate(combos):
    """Remove duplicate parameter combinations."""
    seen = set()
    unique = []
    for label, params in combos:
        key = tuple(sorted((k, round(v, 4) if isinstance(v, float) else v)
                           for k, v in params.items()))
        if key not in seen:
            seen.add(key)
            unique.append((label, params))
    return unique


def run_phase(phase_name, combos, binary, results, t0):
    """Run a sweep phase and return (passed, failed) counts."""
    combos = deduplicate(combos)

    # Remove already-tested
    tested_keys = set()
    for r in results:
        key = tuple(sorted((k, round(r.get(k, 0), 4) if isinstance(r.get(k, 0), float)
                            else r.get(k, 0))
                           for k in BASE.keys()))
        tested_keys.add(key)
    combos = [(l, p) for l, p in combos
              if tuple(sorted((k, round(p.get(k, 0), 4) if isinstance(p.get(k, 0), float)
                               else p.get(k, 0))
                              for k in BASE.keys())) not in tested_keys]

    if not combos:
        print(f"  ({phase_name}: all configs already tested, skipping)")
        return 0, 0

    total = len(combos)
    print(f"\n{'='*80}")
    print(f"{phase_name} — {total} configurations")
    print(f"{'='*80}")

    passed = 0
    failed = 0
    for i, (label, params) in enumerate(combos):
        elapsed = time.time() - t0
        rate = max(len(results), 1) / max(elapsed, 0.01)
        eta = (total - i - 1) / max(rate, 0.01)
        print(f"  [{i+1:4d}/{total}] {label:55s} ", end="", flush=True)

        r = run_test(params, binary)
        score = compute_score(r)
        r["label"] = label
        r["score"] = score
        r["phase"] = phase_name
        r.update(params)
        results.append(r)

        if r["status"] != "OK" or r["failed"] > 0:
            failed += 1
            print(f"FAIL  (ETA {eta:.0f}s)")
        elif r["wall_collisions"] > 0:
            failed += 1
            print(f"wc={r['wall_collisions']}  lat={r['max_lat_err']:.3f}  (ETA {eta:.0f}s)")
        else:
            passed += 1
            print(f"sc={score:7.2f}  vErr={r['avg_vel_err']:.2f}  "
                  f"lat={r['avg_lat_err']:.3f}/{r['max_lat_err']:.3f}  "
                  f"vx={r['max_vx']:.1f}  t5={r['time_above_5ms']:.0f}s  "
                  f"it={r['avg_iters']:.1f}  (ETA {eta:.0f}s)")

    return passed, failed


def main():
    quick = "--quick" in sys.argv
    single = "--single" in sys.argv

    script_dir = os.path.dirname(os.path.abspath(__file__))
    mpc_dir = os.path.dirname(script_dir)
    os.chdir(mpc_dir)
    binary = "./test_sim_drive"

    # Build optimized binary
    print("Building optimized test binary...")
    ret = subprocess.run([
        "gcc", "-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Iinclude",
        "test/test_sim_drive.c", "src/mpc_riccati.c", "src/riccati_solver.c",
        "src/vehicle_model.c", "src/fp_math.c",
        "-o", "test_sim_drive", "-lm"
    ], capture_output=True, text=True)
    if ret.returncode != 0:
        print(f"BUILD FAILED:\n{ret.stderr}")
        sys.exit(1)
    print("  Build OK\n")

    if single:
        params = dict(BASE)
        for arg in sys.argv[2:]:
            if "=" in arg:
                k, v = arg.split("=", 1)
                try:
                    params[k] = int(v)
                except ValueError:
                    params[k] = float(v)
        print(f"Testing: {params}")
        r = run_test(params, binary)
        score = compute_score(r)
        print(f"Result: {r}")
        print(f"Score:  {score}")
        return

    values = QUICK_VALUES if quick else FULL_VALUES
    results = []
    t0 = time.time()
    total_passed = 0
    total_failed = 0

    # ─── Phase 1: One-at-a-time sweep ───────────────────────────────────
    p, f = run_phase("Phase 1: One-at-a-time",
                     gen_one_at_a_time(values), binary, results, t0)
    total_passed += p; total_failed += f

    # ─── Phase 2: Primary weights grid ──────────────────────────────────
    p, f = run_phase("Phase 2: Primary grid (Q_LAT × Q_HDG × Q_VEL)",
                     gen_primary_grid(values), binary, results, t0)
    total_passed += p; total_failed += f

    if not quick:
        # ─── Phase 3: Secondary weights grid ────────────────────────────
        p, f = run_phase("Phase 3: Secondary grid (Q_LAT_VEL × Q_YAW × R_STEER × W_JERK)",
                         gen_secondary_grid(), binary, results, t0)
        total_passed += p; total_failed += f

        # ─── Phase 4: Solver parameters ────────────────────────────────
        p, f = run_phase("Phase 4: Solver grid (RHO × RHO_U × ALPHA)",
                         gen_solver_grid(), binary, results, t0)
        total_passed += p; total_failed += f

        # ─── Phase 5: Wall parameters ──────────────────────────────────
        p, f = run_phase("Phase 5: Wall grid (WALL_END × WALL_SOFT_K)",
                         gen_wall_grid(), binary, results, t0)
        total_passed += p; total_failed += f

        # ─── Phase 6: Velocity-focused configs ─────────────────────────
        p, f = run_phase("Phase 6: Velocity-focused",
                         gen_velocity_focused(), binary, results, t0)
        total_passed += p; total_failed += f

    # ─── Phase 7: Fine-tuning around best ───────────────────────────────
    passing = [r for r in results if r.get("score", 999) < 500]
    if passing:
        best = min(passing, key=lambda x: x["score"])
        best_params = {k: best.get(k, BASE[k]) for k in BASE.keys()}
        print(f"\n  Best so far: {best['label']} (score={best['score']:.2f})")

        p, f = run_phase("Phase 7: Fine-tuning around best",
                         gen_fine_tuning(best_params), binary, results, t0)
        total_passed += p; total_failed += f

        # Update best
        passing = [r for r in results if r.get("score", 999) < 500]
        best = min(passing, key=lambda x: x["score"])
        best_params = {k: best.get(k, BASE[k]) for k in BASE.keys()}

        # ─── Phase 8: Random neighbors ──────────────────────────────────
        n_random = 100 if not quick else 30
        p, f = run_phase(f"Phase 8: Random neighbors ({n_random})",
                         gen_random_neighbors(best_params, n_random),
                         binary, results, t0)
        total_passed += p; total_failed += f

    # ─── Sort and report ────────────────────────────────────────────────
    results.sort(key=lambda x: x.get("score", 999))

    elapsed = time.time() - t0
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_results_{timestamp}.csv"

    print(f"\n{'='*80}")
    print(f"COMPLETED {len(results)} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_passed}  Failed: {total_failed}")
    print(f"{'='*80}")

    # Write CSV
    if results:
        fieldnames = (["label", "phase", "score", "passed", "failed",
                       "max_lat_err", "avg_lat_err", "max_hdg_err", "avg_hdg_err",
                       "max_vx", "avg_vel_err", "max_vel_err",
                       "avg_solve_us", "max_solve_us",
                       "wall_collisions", "time_above_5ms", "avg_iters", "status"]
                      + list(BASE.keys()))
        with open(outfile, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"Results saved to: {outfile}")

    # Print top 30
    print(f"\n{'='*80}")
    print("TOP 30 CONFIGURATIONS (lowest score = best)")
    print(f"{'='*80}")
    fmt = "{:<4} {:<55} {:>7} {:>6} {:>7} {:>6} {:>5} {:>5} {:>4}"
    print(fmt.format("Rank", "Label", "Score", "AvgVE", "MaxLat", "AvgLt",
                      "MaxVx", "T>5s", "Itr"))
    print("-" * 108)
    passing_results = [r for r in results if r.get("score", 999) < 500]
    for i, r in enumerate(passing_results[:30]):
        print(fmt.format(
            i+1, r['label'][:55], f"{r['score']:.2f}",
            f"{r['avg_vel_err']:.2f}", f"{r['max_lat_err']:.3f}",
            f"{r['avg_lat_err']:.3f}", f"{r['max_vx']:.1f}",
            f"{r['time_above_5ms']:.0f}", f"{r['avg_iters']:.1f}"))

    if passing_results:
        best = passing_results[0]
        print(f"\n  BEST: {best['label']} (score={best['score']:.2f})")
        print(f"    Q_LAT={best.get('Q_LAT')}, Q_HDG={best.get('Q_HDG')}, "
              f"Q_VEL={best.get('Q_VEL')}, Q_LAT_VEL={best.get('Q_LAT_VEL')}, "
              f"Q_YAW={best.get('Q_YAW')}")
        print(f"    R_STEER={best.get('R_STEER')}, W_JERK={best.get('W_JERK')}, "
              f"W_ACCEL_RATE={best.get('W_ACCEL_RATE')}, R_ACCEL={best.get('R_ACCEL')}")
        print(f"    RHO={best.get('RHO')}, RHO_U={best.get('RHO_U')}, "
              f"ALPHA={best.get('ALPHA')}, TOL={best.get('TOL')}")
        print(f"    WALL_END={best.get('WALL_END')}, WALL_SOFT_K={best.get('WALL_SOFT_K')}")
        print(f"    max_lat={best['max_lat_err']:.3f}, avg_lat={best['avg_lat_err']:.3f}, "
              f"max_vx={best['max_vx']:.2f}, t>5={best['time_above_5ms']:.1f}s, "
              f"walls={best['wall_collisions']}")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
