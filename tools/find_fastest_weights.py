#!/usr/bin/env python3
"""
Fastest-lap MPC weight optimiser.

Loads calibrated real-car plant parameters from optimized_real_car_env.sh
and runs a two-phase search for the MPC weights that minimise average lap
time with zero wall collisions over a 100-second closed-loop run.

Phase 1: Broad random search (500 candidates, 12 parallel workers)
Phase 2: Refined DE optimisation seeded from the best Phase-1 results

Usage (from repo root):
    python3 tools/find_fastest_weights.py
"""

import os
import sys
import subprocess
import tempfile
import random
import numpy as np
import pandas as pd
from concurrent.futures import ProcessPoolExecutor, as_completed
from scipy.optimize import differential_evolution
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ── Paths ──────────────────────────────────────────────────────────────────
REPO_ROOT  = "/home/akselmo/Documents/GitHub/BachelorProject"
CPU_BINARY = os.path.join(REPO_ROOT, "MPC/test_sim_drive")
CPU_CWD    = os.path.join(REPO_ROOT, "MPC")
ENV_SCRIPT = os.path.join(REPO_ROOT, "tools/output/optimized_real_car_env.sh")
OUT_DIR    = os.path.join(REPO_ROOT, "tools/output")

N_WORKERS        = 12
PHASE1_SAMPLES   = 500
SIM_DURATION     = "100.0"

# ── Load calibrated plant environment ──────────────────────────────────────
def load_plant_env():
    plant = {}
    if os.path.exists(ENV_SCRIPT):
        with open(ENV_SCRIPT) as f:
            for line in f:
                line = line.strip()
                if line.startswith("export "):
                    k, v = line[7:].split("=", 1)
                    plant[k] = v
    return plant

PLANT_ENV = load_plant_env()
if not PLANT_ENV:
    print(f"ERROR: {ENV_SCRIPT} not found. Run optimize_sim_to_bag_1to1.py first.")
    sys.exit(1)
print(f"Loaded calibrated plant: {len(PLANT_ENV)} variables")
for k, v in PLANT_ENV.items():
    print(f"  {k}={v}")

# ── Parse sim output ───────────────────────────────────────────────────────
def parse_output(stdout):
    crashed   = "WALL CRASH" in stdout or "[FAIL] No wall collisions" in stdout
    avg_lap   = None
    avg_speed = 0.0
    max_ey    = 9.9
    avg_ey    = 9.9
    laps      = 0

    for line in stdout.splitlines():
        def fval(key):
            if key in line:
                parts = line.strip().split()
                try: return float(parts[-2])
                except: pass
            return None
        v = fval("Avg lap time:")
        if v is not None: avg_lap = v
        v = fval("Avg velocity:")
        if v is not None: avg_speed = v
        v = fval("Max lateral error:")
        if v is not None: max_ey = v
        v = fval("Avg lateral error:")
        if v is not None: avg_ey = v
        if "Completed laps:" in line:
            try: laps = int(line.strip().split()[-1])
            except: pass

    return crashed, avg_lap, avg_speed, max_ey, avg_ey, laps

# ── Single sim run ─────────────────────────────────────────────────────────
def run_sim(weights):
    q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw = weights

    env = os.environ.copy()
    env.update(PLANT_ENV)   # calibrated plant
    env.update({
        "Q_LAT":      f"{q_lat:.4f}",
        "Q_HDG":      f"{q_hdg:.4f}",
        "Q_VEL":      f"{q_vel:.4f}",
        "R_STEER":    f"{r_steer:.4f}",
        "W_JERK":     f"{w_jerk:.4f}",
        "Q_LAT_VEL":  f"{q_lat_vel:.4f}",
        "Q_YAW":      f"{q_yaw:.4f}",
        "SIM_DURATION": SIM_DURATION,
    })

    try:
        r = subprocess.run([CPU_BINARY], env=env, capture_output=True,
                           text=True, cwd=CPU_CWD, timeout=140)
        return parse_output(r.stdout)
    except subprocess.TimeoutExpired:
        return True, None, 0.0, 9.9, 9.9, 0

# ── Phase-1 random search ──────────────────────────────────────────────────
def phase1():
    print(f"\n{'='*60}")
    print(f" PHASE 1 — Random Search ({PHASE1_SAMPLES} candidates, {N_WORKERS} workers)")
    print(f"{'='*60}\n")

    rng = random.Random(42)
    candidates = []
    for _ in range(PHASE1_SAMPLES):
        candidates.append((
            rng.uniform(500,  6000),   # Q_LAT
            rng.uniform(3,    150),    # Q_HDG
            rng.uniform(50,   500),    # Q_VEL
            rng.uniform(0.05, 3.0),    # R_STEER
            rng.uniform(0.05, 5.0),    # W_JERK
            rng.uniform(1.0,  30.0),   # Q_LAT_VEL
            rng.uniform(0.2,  5.0),    # Q_YAW
        ))

    passing = []
    done = 0

    with ProcessPoolExecutor(max_workers=N_WORKERS) as ex:
        futs = {ex.submit(run_sim, c): c for c in candidates}
        for fut in as_completed(futs):
            done += 1
            c = futs[fut]
            crashed, avg_lap, avg_speed, max_ey, avg_ey, laps = fut.result()
            if done % 50 == 0:
                print(f"  Progress: {done}/{PHASE1_SAMPLES}")
                sys.stdout.flush()
            if not crashed and avg_lap is not None and laps >= 4:
                print(f"  [PASS] laps={laps} avg_lap={avg_lap:.3f}s speed={avg_speed:.3f}m/s"
                      f" max_ey={max_ey:.3f}m"
                      f"  Q_LAT={c[0]:.1f} Q_HDG={c[1]:.1f} Q_VEL={c[2]:.1f}"
                      f" R_STEER={c[3]:.3f} W_JERK={c[4]:.3f}")
                passing.append((avg_lap, avg_speed, max_ey, avg_ey, laps, c))

    passing.sort(key=lambda x: x[0])  # fastest lap first
    print(f"\nPhase 1 complete: {len(passing)}/{PHASE1_SAMPLES} passed")
    if passing:
        b = passing[0]
        print(f"Best Phase-1: avg_lap={b[0]:.3f}s speed={b[1]:.3f}m/s max_ey={b[2]:.3f}m")
    return passing

# ── Phase-2 DE loss (module-level so it is picklable) ─────────────────────
def de_loss(params):
    q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw = params
    crashed, avg_lap, avg_speed, max_ey, avg_ey, laps = run_sim(params)

    if crashed or avg_lap is None or laps < 4:
        return 9999.0

    # Optimise purely for lap time; penalise high lateral error lightly
    score = avg_lap + 0.5 * max(0, max_ey - 0.50)
    print(f"  [P2] lap={avg_lap:.3f}s speed={avg_speed:.3f}m/s"
          f" max_ey={max_ey:.3f}m laps={laps} score={score:.4f}"
          f"  Q_LAT={q_lat:.1f} Q_HDG={q_hdg:.2f} Q_VEL={q_vel:.1f}"
          f" R_STEER={r_steer:.3f} W_JERK={w_jerk:.3f}")
    sys.stdout.flush()
    return score

# ── Phase-2 DE refinement ──────────────────────────────────────────────────
def phase2(phase1_best):
    print(f"\n{'='*60}")
    print(f" PHASE 2 — DE Refinement (seeded from Phase-1 top results)")
    print(f"{'='*60}\n")

    bounds = [
        (200,  8000),   # Q_LAT
        (2,    200),    # Q_HDG
        (30,   600),    # Q_VEL
        (0.02, 4.0),    # R_STEER
        (0.02, 6.0),    # W_JERK
        (0.5,  40.0),   # Q_LAT_VEL
        (0.1,  8.0),    # Q_YAW
    ]

    # Seed population from best Phase-1 results (up to 10) + perturbations
    top = [r[5] for r in phase1_best[:10]]
    rng = np.random.default_rng(7)
    pop_size = max(15, len(top) * 2)
    init = []
    for seed in top:
        init.append(list(seed))
        for _ in range(2):
            perturbed = [
                np.clip(seed[i] * rng.uniform(0.85, 1.15), bounds[i][0], bounds[i][1])
                for i in range(7)
            ]
            init.append(perturbed)
    while len(init) < pop_size:
        init.append([rng.uniform(lo, hi) for lo, hi in bounds])
    init = np.array(init[:pop_size])

    result = differential_evolution(
        de_loss, bounds,
        init=init,
        strategy='best1bin',
        maxiter=25,
        popsize=pop_size // 7,
        tol=1e-3,
        mutation=(0.4, 1.0),
        recombination=0.8,
        seed=7,
        workers=N_WORKERS,
        updating='deferred',
        disp=True,
    )

    return result

# ── Main ───────────────────────────────────────────────────────────────────
def main():
    print("="*60)
    print(" FASTEST-LAP MPC WEIGHT OPTIMISER")
    print(f" Plant: calibrated from {ENV_SCRIPT}")
    print("="*60)

    # Phase 1
    p1_results = phase1()
    if not p1_results:
        print("\n[FAIL] Phase 1 found no passing configurations.")
        return 1

    # Phase 2
    p2_result = phase2(p1_results)

    q_lat, q_hdg, q_vel, r_steer, w_jerk, q_lat_vel, q_yaw = p2_result.x

    print("\n" + "="*60)
    print(" FASTEST COLLISION-FREE MPC WEIGHTS")
    print("="*60)
    print(f"  export Q_LAT={q_lat:.4f}")
    print(f"  export Q_HDG={q_hdg:.4f}")
    print(f"  export Q_VEL={q_vel:.4f}")
    print(f"  export R_STEER={r_steer:.4f}")
    print(f"  export W_JERK={w_jerk:.4f}")
    print(f"  export Q_LAT_VEL={q_lat_vel:.4f}")
    print(f"  export Q_YAW={q_yaw:.4f}")
    print(f"  Objective score: {p2_result.fun:.4f}")

    # Save to file
    weights_sh = os.path.join(OUT_DIR, "fastest_weights.sh")
    with open(weights_sh, "w") as f:
        f.write("#!/usr/bin/env bash\n")
        f.write("# Fastest-lap MPC weights — calibrated real-car plant\n")
        for k, v in zip(
            ["Q_LAT","Q_HDG","Q_VEL","R_STEER","W_JERK","Q_LAT_VEL","Q_YAW"],
            p2_result.x
        ):
            f.write(f"export {k}={v:.6f}\n")
    print(f"\nSaved weights → {weights_sh}")

    # Final validation run printed to terminal
    print("\n--- Final Validation Run ---")
    crashed, avg_lap, avg_speed, max_ey, avg_ey, laps = run_sim(p2_result.x)
    print(f"  Crashed:   {crashed}")
    print(f"  Laps:      {laps}")
    print(f"  Avg lap:   {avg_lap} s")
    print(f"  Avg speed: {avg_speed} m/s")
    print(f"  Max |e_y|: {max_ey} m")
    print(f"  Avg |e_y|: {avg_ey} m")

    return 0

if __name__ == '__main__':
    sys.exit(main())
