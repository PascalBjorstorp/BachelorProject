#!/usr/bin/env python3
"""
Comprehensive MPC Weight Tuning for REALISTIC_SIM=1 Mode
=========================================================
Sweeps weights, horizons, wall margins, wall stiffness,
and multiple raceline variants.  Optimized for maximum velocity
while maintaining safety (no wall crashes) under all realistic effects.

Usage:
    python3 test/tune_realistic.py                   # Full sweep
    python3 test/tune_realistic.py --quick            # Quick subset
    python3 test/tune_realistic.py --phase 2          # Run single phase
"""

import subprocess, os, sys, csv, itertools, time, random
from datetime import datetime

# ─── Environment ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TRAJ_DIR = os.path.join(os.path.dirname(PROJECT_DIR),
                         "f1tenth_planning", "trajectories")

# ─── Raceline variants (pipeline-generated with different wall clearances) ───
RACELINES = {
    "cl020_orig": os.path.join(TRAJ_DIR, "Spielberg_raceline_clearance_0.20.csv"),
    "cl020": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl020.csv"),
    "cl030": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl030.csv"),
    "cl045": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl045.csv"),
    "cl050": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl050.csv"),
}

# ─── Tunable parameters ─────────────────────────────────────────────────────
BASE = {
    "Q_LAT":        400.0,
    "Q_HDG":        1000.0,
    "Q_VEL":        30.0,
    "Q_LAT_VEL":    69.0,
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
    "WALL_END":     16,
    "WALL_SOFT_K":  5000.0,
    "WALL_MARGIN":  0.40,
    "HORIZON":      20,
}

# ─── Special env vars (not weights, passed as env directly) ──────────────────
SPECIAL_PARAMS = {"WALL_MARGIN", "HORIZON", "RACELINE_PATH"}

# ─── Sweep ranges ───────────────────────────────────────────────────────────
FULL_VALUES = {
    "Q_LAT":        [200, 300, 400, 500, 600, 800],
    "Q_HDG":        [400, 600, 800, 900, 1000, 1200, 1500],
    "Q_VEL":        [10, 15, 20, 25, 30, 35, 40, 50],
    "Q_LAT_VEL":    [20, 40, 60, 80, 120],
    "Q_YAW":        [10, 22, 40, 60, 100],
    "R_STEER":      [0.08, 0.12, 0.15, 0.20, 0.30],
    "R_ACCEL":      [0.005, 0.01, 0.02, 0.05],
    "W_JERK":       [0.1, 0.2, 0.3, 0.5, 1.0],
    "W_ACCEL_RATE": [0.05, 0.1, 0.2, 0.5],
    "HORIZON":      [18, 19, 20],
    "WALL_MARGIN":  [0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.70],
    "WALL_END":     [8, 12, 16, 20],
    "WALL_SOFT_K":  [0, 500, 1000, 3000, 5000, 10000],
    "RHO":          [20, 30, 50, 80],
    "RHO_U":        [10, 20, 30, 50],
    "ALPHA":        [0.93, 1.0, 1.2, 1.4],
}

QUICK_VALUES = {
    "Q_LAT":        [300, 400, 600],
    "Q_HDG":        [600, 1000, 1200],
    "Q_VEL":        [20, 30, 40],
    "Q_LAT_VEL":    [40, 60, 100],
    "Q_YAW":        [22, 40],
    "HORIZON":      [18, 19, 20],
    "WALL_MARGIN":  [0.10, 0.20, 0.40, 0.70],
    "WALL_END":     [10, 12, 16, 20],
    "WALL_SOFT_K":  [0, 3000, 5000, 10000],
}


# ─── Test runner ─────────────────────────────────────────────────────────────
def run_test(params: dict, binary: str, raceline: str = None) -> dict:
    """Run a single REALISTIC_SIM=1 test with given parameters."""
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"
    env["REALISTIC_SIM"] = "1"

    if raceline:
        env["RACELINE_PATH"] = raceline

    for name, value in params.items():
        env[name] = str(value)

    try:
        result = subprocess.run(
            [binary], capture_output=True, text=True, timeout=240, env=env
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
    """Composite score optimized for max velocity + safety.

    Priority: wall crashes → max velocity → time above 5m/s
    """
    if r["status"] != "OK" or r["failed"] > 0:
        return 999.0
    if r["wall_collisions"] > 0:
        return 500.0 + r["wall_collisions"] * 100.0

    # Primary: velocity (lower is better, so penalize low velocity)
    velocity_penalty = max(0, 12.0 - r["max_vx"]) * 15.0
    time_penalty = max(0, 60 - r["time_above_5ms"]) * 2.0

    # Secondary: tracking quality
    tracking = (
        r["avg_lat_err"] * 5.0 +
        r["max_lat_err"] * 1.0 +
        r["avg_vel_err"] * 5.0 +
        r["avg_hdg_err"] * 2.0
    )

    # Tertiary: solver efficiency
    solver = r.get("avg_iters", 0) * 0.3 + r["avg_solve_us"] * 0.002

    return round(velocity_penalty + time_penalty + tracking + solver, 3)


# ─── Combination generators ─────────────────────────────────────────────────

def gen_one_at_a_time(values_dict):
    """Vary each parameter one at a time."""
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
    """Grid: Q_LAT × Q_HDG × Q_VEL × HORIZON."""
    combos = []
    ql_vals = values_dict.get("Q_LAT", [BASE["Q_LAT"]])
    qh_vals = values_dict.get("Q_HDG", [BASE["Q_HDG"]])
    qv_vals = values_dict.get("Q_VEL", [BASE["Q_VEL"]])
    h_vals = values_dict.get("HORIZON", [BASE["HORIZON"]])
    for ql, qh, qv, h in itertools.product(ql_vals, qh_vals, qv_vals, h_vals):
        w = dict(BASE)
        w["Q_LAT"] = ql; w["Q_HDG"] = qh; w["Q_VEL"] = qv; w["HORIZON"] = h
        combos.append((f"L={ql}+H={qh}+V={qv}+N={h}", w))
    return combos


def gen_wall_grid(values_dict):
    """Grid: WALL_MARGIN × WALL_END × WALL_SOFT_K."""
    combos = []
    wm_vals = values_dict.get("WALL_MARGIN", [BASE["WALL_MARGIN"]])
    we_vals = values_dict.get("WALL_END", [BASE["WALL_END"]])
    wk_vals = values_dict.get("WALL_SOFT_K", [BASE["WALL_SOFT_K"]])
    for wm, we, wk in itertools.product(wm_vals, we_vals, wk_vals):
        w = dict(BASE)
        w["WALL_MARGIN"] = wm; w["WALL_END"] = we; w["WALL_SOFT_K"] = wk
        combos.append((f"WM={wm}+WE={we}+WK={wk}", w))
    return combos


def gen_secondary_grid(values_dict):
    """Grid: Q_LAT_VEL × Q_YAW × R_STEER × W_JERK."""
    combos = []
    qlv_vals = values_dict.get("Q_LAT_VEL", [40, 60, 100])
    qy_vals = values_dict.get("Q_YAW", [10, 22, 40])
    rs_vals = values_dict.get("R_STEER", [0.10, 0.15, 0.25])
    wj_vals = values_dict.get("W_JERK", [0.1, 0.3, 0.5])
    for qlv, qy, rs, wj in itertools.product(qlv_vals, qy_vals, rs_vals, wj_vals):
        w = dict(BASE)
        w["Q_LAT_VEL"] = qlv; w["Q_YAW"] = qy; w["R_STEER"] = rs; w["W_JERK"] = wj
        combos.append((f"LV={qlv}+Y={qy}+RS={rs}+WJ={wj}", w))
    return combos


def gen_solver_grid():
    """Grid: RHO × RHO_U × ALPHA."""
    combos = []
    for rho in [20, 30, 50, 80]:
        for rho_u in [10, 20, 30, 50]:
            for alpha in [0.93, 1.0, 1.2, 1.4]:
                w = dict(BASE)
                w["RHO"] = rho; w["RHO_U"] = rho_u; w["ALPHA"] = alpha
                combos.append((f"rho={rho}+ru={rho_u}+a={alpha}", w))
    return combos


def gen_velocity_push():
    """Configurations targeting max velocity with various safety levels."""
    combos = []
    configs = [
        # Aggressive velocity tracking, strong lateral control
        {"Q_VEL": 100, "Q_LAT": 500, "Q_HDG": 1200},
        {"Q_VEL": 150, "Q_LAT": 500, "Q_HDG": 1200},
        {"Q_VEL": 100, "Q_LAT": 600, "Q_HDG": 1500},
        {"Q_VEL": 150, "Q_LAT": 600, "Q_HDG": 1500},
        # High lateral weight
        {"Q_VEL": 80, "Q_LAT": 800, "Q_HDG": 1200},
        {"Q_VEL": 100, "Q_LAT": 800, "Q_HDG": 1200},
        # Strong wall margin
        {"Q_VEL": 100, "WALL_MARGIN": 0.95, "WALL_END": 20},
        {"Q_VEL": 150, "WALL_MARGIN": 0.95, "WALL_END": 20},
        # Low wall margin (aggressive)
        {"Q_VEL": 100, "WALL_MARGIN": 0.50, "WALL_END": 16},
        {"Q_VEL": 100, "WALL_MARGIN": 0.60, "WALL_END": 16},
        # Different horizon with high velocity
        {"Q_VEL": 100, "HORIZON": 18},
        {"Q_VEL": 100, "HORIZON": 20},
        # Low R_STEER for agile steering
        {"Q_VEL": 100, "R_STEER": 0.08, "W_JERK": 0.1},
        {"Q_VEL": 100, "R_STEER": 0.08, "Q_LAT": 600},
        # Low yaw rate penalty for faster cornering
        {"Q_VEL": 100, "Q_YAW": 10, "Q_LAT_VEL": 30},
        {"Q_VEL": 100, "Q_YAW": 10, "Q_LAT": 600},
        # High ADMM alpha (over-relaxation) + velocity
        {"Q_VEL": 100, "ALPHA": 1.2, "RHO": 50},
        {"Q_VEL": 100, "ALPHA": 1.4, "RHO": 50},
        # Soft vs hard wall constraints
        {"Q_VEL": 100, "WALL_SOFT_K": 0},
        {"Q_VEL": 100, "WALL_SOFT_K": 500},
        {"Q_VEL": 100, "WALL_SOFT_K": 1000},
        {"Q_VEL": 100, "WALL_SOFT_K": 10000},
    ]
    for cfg in configs:
        w = dict(BASE)
        w.update(cfg)
        label = "+".join(f"{k}={v}" for k, v in cfg.items())
        combos.append((label, w))
    return combos


def gen_fine_tuning(best_weights, pct_range=(0.85, 0.90, 0.95, 1.05, 1.10, 1.15)):
    """Fine-tuning around best config."""
    combos = []
    skip = {"MAX_ITER", "HORIZON"}
    for name, base_val in best_weights.items():
        if base_val == 0 or name in skip:
            continue
        for mult in pct_range:
            new_val = round(base_val * mult, 6)
            if name in ("WALL_END",):
                new_val = max(1, int(new_val))
            elif name in ("WALL_SOFT_K",):
                new_val = max(0, round(new_val))
            w = dict(best_weights)
            w[name] = new_val
            pct = int((mult - 1.0) * 100)
            sign = "+" if pct >= 0 else ""
            combos.append((f"FT:{name}{sign}{pct}%", w))

    # Pairwise perturbation of key params
    key_params = ["Q_LAT", "Q_HDG", "Q_VEL", "R_STEER", "WALL_MARGIN"]
    for w1, w2 in itertools.combinations(key_params, 2):
        v1 = best_weights.get(w1, 0)
        v2 = best_weights.get(w2, 0)
        if v1 == 0 or v2 == 0:
            continue
        for m1, m2 in [(0.9, 1.1), (1.1, 0.9), (0.9, 0.9), (1.1, 1.1)]:
            w = dict(best_weights)
            w[w1] = round(v1 * m1, 6)
            w[w2] = round(v2 * m2, 6)
            p1 = "+10%" if m1 > 1 else "-10%"
            p2 = "+10%" if m2 > 1 else "-10%"
            combos.append((f"FT:{w1}{p1}+{w2}{p2}", w))
    return combos


def gen_random_neighbors(best_weights, n=150):
    """Random perturbations around best config."""
    combos = []
    random.seed(42)
    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ITER",) and best_weights[k] != 0]
    for i in range(n):
        w = dict(best_weights)
        num_perturb = random.randint(2, min(6, len(tune_params)))
        params_to_perturb = random.sample(tune_params, num_perturb)
        for name in params_to_perturb:
            if name == "HORIZON":
                w[name] = random.choice([18, 19, 20])
            else:
                mult = random.uniform(0.6, 1.5)
                w[name] = round(w[name] * mult, 6)
                if name in ("WALL_END",):
                    w[name] = max(1, int(w[name]))
                elif name in ("WALL_SOFT_K",):
                    w[name] = max(0, round(w[name]))
        combos.append((f"RND_{i}", w))
    return combos


# ─── Deduplication ───────────────────────────────────────────────────────────
def deduplicate(combos):
    seen = set()
    unique = []
    for label, params in combos:
        key = tuple(sorted((k, round(v, 4) if isinstance(v, float) else v)
                           for k, v in params.items()))
        if key not in seen:
            seen.add(key)
            unique.append((label, params))
    return unique


# ─── Phase runner ────────────────────────────────────────────────────────────
def run_phase(phase_name, combos, binary, results, t0,
              raceline=None, raceline_label=""):
    """Run a sweep phase. Returns (passed, failed)."""
    combos = deduplicate(combos)
    if not combos:
        print(f"  ({phase_name}: empty, skipping)")
        return 0, 0

    total = len(combos)
    suffix = f" [{raceline_label}]" if raceline_label else ""
    print(f"\n{'='*80}")
    print(f"{phase_name}{suffix} — {total} configurations")
    print(f"{'='*80}")

    passed = failed = 0
    for i, (label, params) in enumerate(combos):
        elapsed = time.time() - t0
        rate = max(len(results), 1) / max(elapsed, 0.01)
        eta = (total - i - 1) / max(rate, 0.01)
        tag = f"{label}|{raceline_label}" if raceline_label else label
        print(f"  [{i+1:4d}/{total}] {tag:60s} ", end="", flush=True)

        r = run_test(params, binary, raceline)
        score = compute_score(r)
        r["label"] = label
        r["score"] = score
        r["phase"] = phase_name
        r["raceline"] = raceline_label or "default"
        r.update(params)
        results.append(r)

        if r["status"] != "OK" or r["failed"] > 0:
            failed += 1
            print(f"FAIL  (ETA {eta:.0f}s)")
        elif r["wall_collisions"] > 0:
            failed += 1
            print(f"wc={r['wall_collisions']}  (ETA {eta:.0f}s)")
        else:
            passed += 1
            print(f"sc={score:7.2f}  vx={r['max_vx']:.1f}  "
                  f"t5={r['time_above_5ms']:.0f}s  "
                  f"lat={r['avg_lat_err']:.3f}  "
                  f"ve={r['avg_vel_err']:.2f}  (ETA {eta:.0f}s)")

    return passed, failed


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    quick = "--quick" in sys.argv
    phase_only = None
    raceline_arg = None
    for i, arg in enumerate(sys.argv):
        if arg == "--phase" and i + 1 < len(sys.argv):
            phase_only = int(sys.argv[i + 1])
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_arg = sys.argv[i + 1]

    os.chdir(PROJECT_DIR)
    binary = "./test_sim_drive"

    # Build optimized binary
    print("Building optimized test binary with REALISTIC_SIM support...")
    ret = subprocess.run([
        "gcc", "-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Wno-unknown-pragmas",
        "-Iinclude",
        "test/test_sim_drive.c", "src/mpc_riccati.c", "src/riccati_solver.c",
        "src/vehicle_model.c", "src/fp_math.c",
        "-o", "test_sim_drive", "-lm"
    ], capture_output=True, text=True)
    if ret.returncode != 0:
        print(f"BUILD FAILED:\n{ret.stderr}")
        sys.exit(1)
    print("  Build OK\n")

    # Check available racelines
    available_racelines = {}
    for tag, path in RACELINES.items():
        if os.path.exists(path):
            available_racelines[tag] = path
            print(f"  Raceline [{tag}]: {path}")
        else:
            print(f"  Raceline [{tag}]: NOT FOUND ({path})")

    if not available_racelines:
        print("ERROR: No racelines found!")
        sys.exit(1)

    # Use specified or default raceline for main sweep
    if raceline_arg and raceline_arg in available_racelines:
        primary_tag = raceline_arg
    else:
        primary_tag = "cl045"
    primary_path = available_racelines.get(primary_tag)
    if not primary_path:
        primary_tag = list(available_racelines.keys())[0]
        primary_path = available_racelines[primary_tag]

    # Per-raceline WALL_MARGIN defaults (max feasible = effective_wall - buffer)
    PER_RACELINE_WM = {
        "cl020": 0.10,   # effective=0.18, use 0.10
        "cl030": 0.20,   # effective=0.27, use 0.20
        "cl045": 0.40,   # effective=0.41, use 0.40
        "cl050": 0.40,   # effective=0.44, use 0.40
    }
    if primary_tag in PER_RACELINE_WM:
        BASE["WALL_MARGIN"] = PER_RACELINE_WM[primary_tag]
        print(f"  Adjusted WALL_MARGIN to {BASE['WALL_MARGIN']} for [{primary_tag}]")
    print(f"\n  Primary raceline: [{primary_tag}]")

    values = QUICK_VALUES if quick else FULL_VALUES
    results = []
    t0 = time.time()
    total_p = total_f = 0

    def should_run(phase_num):
        return phase_only is None or phase_only == phase_num

    # ─── Phase 1: One-at-a-time (primary raceline) ──────────────────────
    if should_run(1):
        p, f = run_phase("Phase 1: One-at-a-time",
                         gen_one_at_a_time(values), binary, results, t0,
                         primary_path, primary_tag)
        total_p += p; total_f += f

    # ─── Phase 2: Primary weights + horizon grid ────────────────────────
    if should_run(2):
        p, f = run_phase("Phase 2: Primary grid (Q_LAT×Q_HDG×Q_VEL×HORIZON)",
                         gen_primary_grid(values), binary, results, t0,
                         primary_path, primary_tag)
        total_p += p; total_f += f

    # ─── Phase 3: Wall/corridor grid ────────────────────────────────────
    if should_run(3):
        p, f = run_phase("Phase 3: Wall grid (WALL_MARGIN×WALL_END×WALL_SOFT_K)",
                         gen_wall_grid(values), binary, results, t0,
                         primary_path, primary_tag)
        total_p += p; total_f += f

    if not quick:
        # ─── Phase 4: Secondary weights grid ────────────────────────────
        if should_run(4):
            p, f = run_phase("Phase 4: Secondary grid",
                             gen_secondary_grid(values), binary, results, t0,
                             primary_path, primary_tag)
            total_p += p; total_f += f

        # ─── Phase 5: Solver parameters ────────────────────────────────
        if should_run(5):
            p, f = run_phase("Phase 5: Solver grid",
                             gen_solver_grid(), binary, results, t0,
                             primary_path, primary_tag)
            total_p += p; total_f += f

        # ─── Phase 6: Velocity push configs ────────────────────────────
        if should_run(6):
            p, f = run_phase("Phase 6: Velocity push",
                             gen_velocity_push(), binary, results, t0,
                             primary_path, primary_tag)
            total_p += p; total_f += f

    # ─── Phase 7: Fine-tuning around best ───────────────────────────────
    if should_run(7):
        passing = [r for r in results if r.get("score", 999) < 500]
        if passing:
            best = min(passing, key=lambda x: x["score"])
            best_params = {k: best.get(k, BASE[k]) for k in BASE.keys()}
            print(f"\n  Best so far: {best['label']} (score={best['score']:.2f}, "
                  f"vx={best['max_vx']:.1f}, t5={best['time_above_5ms']:.0f})")

            p, f = run_phase("Phase 7: Fine-tuning",
                             gen_fine_tuning(best_params), binary, results, t0,
                             primary_path, primary_tag)
            total_p += p; total_f += f

    # ─── Phase 8: Random neighbors ──────────────────────────────────────
    if should_run(8):
        passing = [r for r in results if r.get("score", 999) < 500]
        if passing:
            best = min(passing, key=lambda x: x["score"])
            best_params = {k: best.get(k, BASE[k]) for k in BASE.keys()}
            n = 150 if not quick else 50
            p, f = run_phase(f"Phase 8: Random ({n})",
                             gen_random_neighbors(best_params, n), binary, results, t0,
                             primary_path, primary_tag)
            total_p += p; total_f += f

    # ─── Phase 9: Cross-raceline validation ─────────────────────────────
    if should_run(9):
        passing = [r for r in results if r.get("score", 999) < 500]
        if passing and len(available_racelines) > 1:
            best = min(passing, key=lambda x: x["score"])
            best_params = {k: best.get(k, BASE[k]) for k in BASE.keys()}
            top_configs = sorted(passing, key=lambda x: x["score"])[:10]

            for tag, path in available_racelines.items():
                if tag == primary_tag:
                    continue
                cross_combos = [(f"CROSS:{r['label']}", {k: r.get(k, BASE[k]) for k in BASE.keys()})
                                for r in top_configs]
                p, f = run_phase(f"Phase 9: Cross-validation",
                                 cross_combos, binary, results, t0, path, tag)
                total_p += p; total_f += f

    # ─── Results ─────────────────────────────────────────────────────────
    results.sort(key=lambda x: x.get("score", 999))
    elapsed = time.time() - t0
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_realistic_{timestamp}.csv"

    print(f"\n{'='*80}")
    print(f"COMPLETED {len(results)} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    print(f"{'='*80}")

    # Write CSV
    if results:
        fieldnames = (["label", "phase", "raceline", "score",
                       "passed", "failed", "max_lat_err", "avg_lat_err",
                       "max_hdg_err", "avg_hdg_err", "max_vx",
                       "avg_vel_err", "max_vel_err",
                       "avg_solve_us", "max_solve_us",
                       "wall_collisions", "time_above_5ms", "avg_iters", "status"]
                      + list(BASE.keys()))
        with open(outfile, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"Results saved to: {outfile}")

    # Print top 30
    passing = [r for r in results if r.get("score", 999) < 500]
    if passing:
        print(f"\n{'='*80}")
        print("TOP 30 (lowest score = best)")
        print(f"{'='*80}")
        fmt = "{:<4} {:<50} {:>7} {:>6} {:>5} {:>5} {:>5} {:>4} {:>4} {:>6} {:>6}"
        print(fmt.format("Rank", "Label", "Score", "AvgVE", "MaxVx", "T>5s",
                          "AvgLt", "WM", "WE", "WK", "N"))
        print("-" * 110)
        for i, r in enumerate(passing[:30]):
            print(fmt.format(
                i+1, r['label'][:50], f"{r['score']:.1f}",
                f"{r['avg_vel_err']:.2f}", f"{r['max_vx']:.1f}",
                f"{r['time_above_5ms']:.0f}",
                f"{r['avg_lat_err']:.3f}",
                f"{r.get('WALL_MARGIN', '-')}",
                f"{r.get('WALL_END', '-')}",
                f"{r.get('WALL_SOFT_K', '-')}",
                f"{r.get('HORIZON', '-')}"))

        best = passing[0]
        print(f"\n  BEST CONFIGURATION:")
        print(f"    Score: {best['score']:.2f}")
        print(f"    Max velocity: {best['max_vx']:.2f} m/s")
        print(f"    Time > 5 m/s: {best['time_above_5ms']:.1f} s")
        print(f"    Avg lat err:  {best['avg_lat_err']:.4f} m")
        print(f"    Avg vel err:  {best['avg_vel_err']:.2f} m/s")
        print(f"    Walls:        {best['wall_collisions']}")
        print(f"    ---")
        for k in sorted(BASE.keys()):
            print(f"    {k:15s} = {best.get(k, BASE[k])}")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
