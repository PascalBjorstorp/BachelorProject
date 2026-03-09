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
    python3 test/tune_realistic.py --jobs 8           # 8 parallel workers
    python3 test/tune_realistic.py -j 0               # Auto-detect CPU count
"""

import subprocess, os, sys, csv, itertools, time, random
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

# ─── Environment ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TRAJ_DIR = os.path.join(os.path.dirname(PROJECT_DIR),
                         "f1tenth_planning", "trajectories")

# ─── Raceline variants (pipeline-generated with different wall clearances) ───
RACELINES = {
    "cl020": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl020.csv"),
    "cl030": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl030.csv"),
    "cl045": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl045.csv"),
    "cl050": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl050.csv"),
}

# ─── Tunable parameters ─────────────────────────────────────────────────────
# NOTE: BASE updated to best-found config (from raceline cl050) on 2026-03-09
BASE = {
    "Q_LAT":        340.0,
    "Q_HDG":        1000.0,
    "Q_VEL":        26.0,
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
    "WALL_END":     18,
    "WALL_STRIDE":  1,
    "WALL_SOFT_K":  0.0,
    "WALL_MARGIN":  0.15,
    "HORIZON":      20,
    "PRED_DT":      0.03,
}

# ─── Special env vars (not weights, passed as env directly) ──────────────────
SPECIAL_PARAMS = {"WALL_MARGIN", "WALL_STRIDE", "HORIZON", "RACELINE_PATH", "PRED_DT"}

# ─── Sweep ranges ───────────────────────────────────────────────────────────
FULL_VALUES = {
    "Q_LAT":        [150, 200, 250, 300, 320, 340, 350, 360, 380, 400, 450, 500, 550, 600, 650, 700, 750, 800],
    "Q_HDG":        [100, 180, 190, 200, 210, 220, 300, 400, 600, 800, 900, 1000, 1200, 1500],
    "Q_VEL":        [4, 6, 8, 10, 15, 20, 22, 24, 25, 26, 28, 30, 35],
    "Q_LAT_VEL":    [15, 20, 30, 40, 45, 55, 60, 80, 120],
    "Q_YAW":        [5, 10, 15, 18, 22, 30, 40, 60],
    "R_STEER":      [0.04, 0.06, 0.08, 0.09, 0.12, 0.15, 0.18, 0.20, 0.30, 0.5],
    "R_ACCEL":      [0.01, 0.012, 0.015, 0.02, 0.05, 0.1],
    "W_JERK":       [0.08, 0.1, 0.14, 0.2, 0.3, 0.5, 1.0],
    "W_ACCEL_RATE": [0.05, 0.08, 0.1, 0.15, 0.2, 0.5, 1.0],
    "HORIZON":      [15, 20, 25, 30, 35, 40],
    "WALL_MARGIN":  [0.00, 0.05, 0.10, 0.15, 0.20, 0.30, 0.35, 0.40, 0.45, 0.50, 0.70],
    "WALL_END":     [12, 16, 18, 20, 22, 24, 28],
    "WALL_STRIDE":  [1, 2, 3, 4],
    "WALL_SOFT_K":  [0, 500, 1000, 2000, 3000, 5000, 7000, 10000],
    "RHO":          [10, 20, 30, 40, 50, 60, 70, 80],
    "RHO_U":        [10, 15, 20, 25, 30, 40, 50],
    "ALPHA":        [0.93, 0.95, 1.0, 1.05, 1.15, 1.2, 1.4],
    "PRED_DT":      [0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1],
}

QUICK_VALUES = {
    "Q_LAT":        [300, 400, 600],
    "Q_HDG":        [600, 1000, 1200],
        "Q_VEL":        [20, 30],
    "Q_LAT_VEL":    [40, 60, 100],
    "Q_YAW":        [22, 40],
    "HORIZON":      [18, 19, 20],
    "WALL_MARGIN":  [0.10, 0.20, 0.40, 0.70],
    "WALL_END":     [10, 12, 16, 20],
    "WALL_STRIDE":  [1, 2, 3],
    "WALL_SOFT_K":  [0, 3000, 5000, 10000],
    "PRED_DT":      [0.02, 0.04, 0.06, 0.1],
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
    """Grid: Q_LAT × Q_HDG × Q_VEL × HORIZON × PRED_DT."""
    combos = []
    ql_vals = values_dict.get("Q_LAT", [BASE["Q_LAT"]])
    qh_vals = values_dict.get("Q_HDG", [BASE["Q_HDG"]])
    qv_vals = values_dict.get("Q_VEL", [BASE["Q_VEL"]])
    h_vals = values_dict.get("HORIZON", [BASE["HORIZON"]])
    pd_vals = values_dict.get("PRED_DT", [BASE["PRED_DT"]])
    for ql, qh, qv, h, pd in itertools.product(ql_vals, qh_vals, qv_vals, h_vals, pd_vals):
        w = dict(BASE)
        w["Q_LAT"] = ql; w["Q_HDG"] = qh; w["Q_VEL"] = qv; w["HORIZON"] = h; w["PRED_DT"] = pd
        combos.append((f"L={ql}+H={qh}+V={qv}+N={h}+dt={pd}", w))
    return combos


def gen_wall_grid(values_dict):
    """Grid: WALL_MARGIN × WALL_END × WALL_STRIDE × WALL_SOFT_K.
    Skips combos where WALL_END > current HORIZON (from BASE/cascade)."""
    combos = []
    wm_vals = values_dict.get("WALL_MARGIN", [BASE["WALL_MARGIN"]])
    we_vals = values_dict.get("WALL_END", [BASE["WALL_END"]])
    ws_vals = values_dict.get("WALL_STRIDE", [BASE["WALL_STRIDE"]])
    wk_vals = values_dict.get("WALL_SOFT_K", [BASE["WALL_SOFT_K"]])
    horizon = BASE.get("HORIZON", 20)
    for wm, we, ws, wk in itertools.product(wm_vals, we_vals, ws_vals, wk_vals):
        if we > horizon:
            continue  # WALL_END can't exceed HORIZON
        w = dict(BASE)
        w["WALL_MARGIN"] = wm; w["WALL_END"] = we
        w["WALL_STRIDE"] = ws; w["WALL_SOFT_K"] = wk
        combos.append((f"WM={wm}+WE={we}+WS={ws}+WK={wk}", w))
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


def gen_fine_tuning(best_weights, pct_range=(0.80, 0.85, 0.90, 0.95, 1.05, 1.10, 1.15, 1.20)):
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
    key_params = ["Q_LAT", "Q_HDG", "Q_VEL", "R_STEER", "WALL_MARGIN", "HORIZON"]
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
                w[name] = random.choice(range(0, 40))
            elif name == "PRED_DT":
                w[name] = random.choice([0.02, 0.04, 0.05, 0.06, 0.08, 0.1, 0.2])
            elif name == "WALL_STRIDE":
                w[name] = random.choice([1, 2, 3, 4])
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


# ─── Incremental CSV writer ──────────────────────────────────────────────────
class IncrementalCSV:
    """Appends results to CSV file after each test for crash safety."""
    def __init__(self, filepath, fieldnames):
        self.filepath = filepath
        self.fieldnames = fieldnames
        self._header_written = False

    def write_row(self, row):
        mode = "a" if self._header_written else "w"
        with open(self.filepath, mode, newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames,
                                    extrasaction="ignore")
            if not self._header_written:
                writer.writeheader()
                self._header_written = True
            writer.writerow(row)


# ─── Worker function for parallel execution ──────────────────────────────────
def _run_single(args):
    """Worker: run one test and return scored result. Picklable for multiprocessing."""
    label, params, binary, raceline, raceline_label, phase_name = args
    r = run_test(params, binary, raceline)
    score = compute_score(r)
    r["label"] = label
    r["score"] = score
    r["phase"] = phase_name
    r["raceline"] = raceline_label or "default"
    r.update(params)
    return r


# ─── Phase runner ────────────────────────────────────────────────────────────
def run_phase(phase_name, combos, binary, results, t0,
              raceline=None, raceline_label="", num_workers=1,
              csv_writer=None):
    """Run a sweep phase. Returns (passed, failed)."""
    combos = deduplicate(combos)
    if not combos:
        print(f"  ({phase_name}: empty, skipping)")
        return 0, 0

    total = len(combos)
    suffix = f" [{raceline_label}]" if raceline_label else ""
    print(f"\n{'='*80}")
    print(f"{phase_name}{suffix} — {total} configurations"
          f" ({num_workers} workers)")
    print(f"{'='*80}")

    passed = failed = 0

    if num_workers <= 1:
        # Sequential execution (original behavior)
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
            if csv_writer:
                csv_writer.write_row(r)

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
    else:
        # Parallel execution
        work_items = [
            (label, params, binary, raceline, raceline_label, phase_name)
            for label, params in combos
        ]
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            futures = {executor.submit(_run_single, item): item
                       for item in work_items}
            for future in as_completed(futures):
                done_count += 1
                r = future.result()
                results.append(r)
                if csv_writer:
                    csv_writer.write_row(r)

                tag = r["label"]
                if raceline_label:
                    tag = f"{tag}|{raceline_label}"
                score = r["score"]

                elapsed = time.time() - t0
                rate = max(done_count, 1) / max(elapsed, 0.01)
                eta = (total - done_count) / max(rate, 0.01)

                if r["status"] != "OK" or r["failed"] > 0:
                    failed += 1
                    print(f"  [{done_count:4d}/{total}] {tag:60s} "
                          f"FAIL  (ETA {eta:.0f}s)")
                elif r["wall_collisions"] > 0:
                    failed += 1
                    print(f"  [{done_count:4d}/{total}] {tag:60s} "
                          f"wc={r['wall_collisions']}  (ETA {eta:.0f}s)")
                else:
                    passed += 1
                    print(f"  [{done_count:4d}/{total}] {tag:60s} "
                          f"sc={score:7.2f}  vx={r['max_vx']:.1f}  "
                          f"t5={r['time_above_5ms']:.0f}s  "
                          f"lat={r['avg_lat_err']:.3f}  "
                          f"ve={r['avg_vel_err']:.2f}  (ETA {eta:.0f}s)")

    return passed, failed


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    quick = "--quick" in sys.argv
    phase_only = None
    raceline_arg = None
    num_workers = 1
    for i, arg in enumerate(sys.argv):
        if arg == "--phase" and i + 1 < len(sys.argv):
            phase_only = int(sys.argv[i + 1])
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_arg = sys.argv[i + 1]
        if arg == "--jobs" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "-j" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])

    if num_workers <= 0:
        num_workers = multiprocessing.cpu_count()
    print(f"  Workers: {num_workers} "
          f"({'sequential' if num_workers == 1 else 'parallel'})")

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

    # If --raceline specified, filter to just that one
    if raceline_arg:
        if raceline_arg in available_racelines:
            available_racelines = {raceline_arg: available_racelines[raceline_arg]}
            print(f"  Filtered to raceline: [{raceline_arg}]")
        else:
            print(f"  WARNING: --raceline {raceline_arg} not found, using all available")

    # Per-raceline WALL_MARGIN defaults (max feasible = effective_wall - buffer)
    PER_RACELINE_WM = {
        "cl020": 0.10,   # effective=0.18, use 0.10
        "cl030": 0.20,   # effective=0.27, use 0.20
        "cl045": 0.40,   # effective=0.41, use 0.40
        "cl050": 0.40,   # effective=0.44, use 0.40
    }
    values = QUICK_VALUES if quick else FULL_VALUES
    results = []
    t0 = time.time()
    total_p = total_f = 0

    # Incremental CSV writer — results saved after each test
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_realistic_{timestamp}.csv"
    fieldnames = (["label", "phase", "raceline", "score",
                   "passed", "failed", "max_lat_err", "avg_lat_err",
                   "max_hdg_err", "avg_hdg_err", "max_vx",
                   "avg_vel_err", "max_vel_err",
                   "avg_solve_us", "max_solve_us",
                   "wall_collisions", "time_above_5ms", "avg_iters", "status"]
                  + list(BASE.keys()))
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results file: {outfile} (incremental)")
    def should_run(phase_num):
        return phase_only is None or phase_only == phase_num

    # Helper: get best params for this raceline from results so far
    def get_best_params(rl_tag):
        """Return best-so-far params dict for given raceline, or None."""
        rl_results = [r for r in results
                      if r.get("raceline") == rl_tag and r.get("score", 999) < 500]
        if not rl_results:
            return None
        best = min(rl_results, key=lambda x: x["score"])
        best_p = {k: best.get(k, BASE[k]) for k in BASE.keys()}
        print(f"  Cascading from: {best['label']} "
              f"(score={best['score']:.2f}, vx={best.get('max_vx', 0):.1f})")
        return best_p

    def update_base(new_params):
        """Temporarily update BASE dict to cascade best-so-far into generators."""
        if new_params:
            for k in BASE:
                if k in new_params:
                    BASE[k] = new_params[k]

    # ─── Run all phases across all available racelines ───────────────────
    original_base = dict(BASE)  # Save pristine BASE

    for rl_tag, rl_path in available_racelines.items():
        # Reset BASE to original for each raceline
        for k, v in original_base.items():
            BASE[k] = v

        # Adjust WALL_MARGIN per raceline
        if rl_tag in PER_RACELINE_WM:
            BASE["WALL_MARGIN"] = PER_RACELINE_WM[rl_tag]
        rl_base = dict(BASE)  # Save per-raceline starting BASE

        print(f"\n{'#'*80}")
        print(f"# Raceline: [{rl_tag}]  WALL_MARGIN={BASE['WALL_MARGIN']}")
        print(f"{'#'*80}")

        # ─── Phase 1: One-at-a-time (from original BASE) ────────────
        if should_run(1):
            p, f = run_phase("Phase 1: One-at-a-time",
                             gen_one_at_a_time(values), binary, results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        # ─── Phase 2: Primary grid (from original BASE) ─────────────
        if should_run(2):
            p, f = run_phase("Phase 2: Primary grid (Q_LAT×Q_HDG×Q_VEL×HORIZON×PRED_DT)",
                             gen_primary_grid(values), binary, results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        # ─── CASCADE: update BASE to best of Phases 1+2 ─────────────
        cascade_params = get_best_params(rl_tag)
        if cascade_params:
            update_base(cascade_params)
            # Preserve per-raceline WALL_MARGIN for wall sweep
            if rl_tag in PER_RACELINE_WM:
                BASE["WALL_MARGIN"] = PER_RACELINE_WM[rl_tag]

        # ─── Phase 3: Wall grid (cascaded from best of 1+2) ─────────
        if should_run(3):
            p, f = run_phase("Phase 3: Wall grid (WM×WE×WS×WK)",
                             gen_wall_grid(values), binary, results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        if not quick:
            # ─── CASCADE: update BASE to best of Phases 1-3 ─────────
            cascade_params = get_best_params(rl_tag)
            if cascade_params:
                update_base(cascade_params)

            # ─── Phase 4: Secondary grid (cascaded from best of 1-3) ─
            if should_run(4):
                p, f = run_phase("Phase 4: Secondary grid",
                                 gen_secondary_grid(values), binary, results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

            # ─── CASCADE: update BASE to best of Phases 1-4 ─────────
            cascade_params = get_best_params(rl_tag)
            if cascade_params:
                update_base(cascade_params)

            # ─── Phase 5: Solver parameters (cascaded from best of 1-4)
            if should_run(5):
                p, f = run_phase("Phase 5: Solver grid",
                                 gen_solver_grid(), binary, results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

            # ─── CASCADE: update BASE to best of Phases 1-5 ─────────
            cascade_params = get_best_params(rl_tag)
            if cascade_params:
                update_base(cascade_params)

            # ─── Phase 6: Velocity push (cascaded from best of 1-5) ──
            if should_run(6):
                p, f = run_phase("Phase 6: Velocity push",
                                 gen_velocity_push(), binary, results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

        # ─── Phase 7: Fine-tuning around best (cascaded) ────────────
        if should_run(7):
            best_params = get_best_params(rl_tag)
            if best_params:
                p, f = run_phase("Phase 7: Fine-tuning",
                                 gen_fine_tuning(best_params), binary, results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

        # ─── Phase 8: Random neighbors (cascaded) ───────────────────
        if should_run(8):
            best_params = get_best_params(rl_tag)
            if best_params:
                n = 150 if not quick else 50
                p, f = run_phase(f"Phase 8: Random ({n})",
                                 gen_random_neighbors(best_params, n), binary, results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

        # Reset BASE to original for next raceline
        for k, v in original_base.items():
            BASE[k] = v

    # ─── Results ─────────────────────────────────────────────────────────
    results.sort(key=lambda x: x.get("score", 999))
    elapsed = time.time() - t0

    print(f"\n{'='*80}")
    print(f"COMPLETED {len(results)} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    print(f"{'='*80}")

    # Write sorted final CSV (incremental file has insertion order)
    sorted_outfile = outfile.replace(".csv", "_sorted.csv")
    if results:
        with open(sorted_outfile, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"Incremental results: {outfile}")
        print(f"Sorted results:      {sorted_outfile}")

    # Print top 30
    passing = [r for r in results if r.get("score", 999) < 500]
    if passing:
        print(f"\n{'='*80}")
        print("TOP 30 (lowest score = best)")
        print(f"{'='*80}")
        fmt = "{:<4} {:<50} {:>7} {:>6} {:>5} {:>5} {:>5} {:>4} {:>4} {:>3} {:>6} {:>6} {:>6}"
        print(fmt.format("Rank", "Label", "Score", "AvgVE", "MaxVx", "T>5s",
                          "AvgLt", "WM", "WE", "WS", "WK", "N", "PdDT"))
        print("-" * 130)
        for i, r in enumerate(passing[:30]):
            print(fmt.format(
                i+1, r['label'][:50], f"{r['score']:.1f}",
                f"{r['avg_vel_err']:.2f}", f"{r['max_vx']:.1f}",
                f"{r['time_above_5ms']:.0f}",
                f"{r['avg_lat_err']:.3f}",
                f"{r.get('WALL_MARGIN', '-')}",
                f"{r.get('WALL_END', '-')}",
                f"{r.get('WALL_STRIDE', '-')}",
                f"{r.get('WALL_SOFT_K', '-')}",
                f"{r.get('HORIZON', '-')}",
                f"{r.get('PRED_DT', '-')}"  ))

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
