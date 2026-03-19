#!/usr/bin/env python3
"""
MPCC Weight Tuning — Comprehensive Standalone Simulation Sweep
===============================================================
Sweeps MPCC controller weights (Lifted ODE, ADMM+Riccati) via env vars
and the standalone test_sim_drive binary (gym-matching vehicle model).

Supports two modes:
  Spielberg  — Large Spielberg track with multiple clearance racelines
  Hardware   — Small SLAM-mapped track (~22m, 0.27-1.4m wide)

After finding good parameters offline, validate on the full ROS2
simulation using run_sim.sh with the same env vars.

Usage:
    python3 test/tune_mpcc.py Spielberg                # Full Spielberg sweep
    python3 test/tune_mpcc.py Hardware                 # Full Hardware sweep
    python3 test/tune_mpcc.py Hardware --quick          # Quick Hardware sweep
    python3 test/tune_mpcc.py Hardware --phase 2        # Run single phase
    python3 test/tune_mpcc.py Hardware --jobs 8         # 8 parallel workers
    python3 test/tune_mpcc.py Hardware -j 0             # Auto-detect CPU count
    python3 test/tune_mpcc.py Hardware --cascade-top 20 # Cascade top-20
    python3 test/tune_mpcc.py Hardware --validate       # Validate best on ROS2
"""

import subprocess, os, sys, csv, itertools, time, random
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

# ─── Paths ───────────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MPCC_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_DIR = os.path.dirname(MPCC_DIR)
MPC_DIR = os.path.join(PROJECT_DIR, "MPC")
TRAJ_DIR = os.path.join(PROJECT_DIR, "f1tenth_planning", "trajectories")

BINARY = os.path.join(MPCC_DIR, "test_sim_drive")
RUN_SIM_SH = os.path.join(MPCC_DIR, "run_sim.sh")

# ═══════════════════════════════════════════════════════════════════════════════
# MODE-SPECIFIC CONFIGURATIONS
# ═══════════════════════════════════════════════════════════════════════════════

# ─── Spielberg: large track, multiple clearance racelines ────────────────────
SPIELBERG_RACELINES = {
    "cl020": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl020.csv"),
    "cl030": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl030.csv"),
    "cl045": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl045.csv"),
    "cl050": os.path.join(TRAJ_DIR, "Spielberg_raceline_pipeline_cl050.csv"),
    "default": os.path.join(TRAJ_DIR, "Spielberg_raceline.csv"),
}

SPIELBERG_BASE = {
    # Frenet tracking
    "Q_N":              100.0,
    "Q_ALPHA":          10.0,
    "Q_PROGRESS":       1.0,
    # State regularization
    "Q_VX":             15.0,
    "VX_REF":           12.0,
    "Q_VY":             0.5,
    "Q_OMEGA":          0.1,
    # Control effort
    "R_DELTA":          0.1,
    "R_AX":             0.01,
    "R_VTHETA":         0.5,
    # Control rate
    "W_DELTA_RATE":     2.0,
    "W_AX_RATE":        0.1,
    "W_VTHETA_RATE":    0.1,
    # Terminal
    "Q_N_TERM":         100.0,
    "Q_ALPHA_TERM":     10.0,
    "Q_PROGRESS_TERM":  5.0,
    # ADMM solver
    "ADMM_RHO":         1.0,
    "ADMM_MAX_ITER":    100,
    "ADMM_TOL":         0.01,
    # Horizon
    "HORIZON":          20,
    "DT":               0.05,
    "V_THETA_MAX":      3.5,
}

# ─── Hardware: small SLAM-mapped track (~22m, 0.27-1.4m wide) ────────────────
HARDWARE_RACELINES = {
    "my_track": os.path.join(TRAJ_DIR, "my_track_raceline.csv"),
}

HARDWARE_BASE = {
    # Frenet tracking — higher weights for tight track
    "Q_N":              500.0,
    "Q_ALPHA":          50.0,
    "Q_PROGRESS":       1.0,
    # State regularization
    "Q_VX":             15.0,
    "VX_REF":           2.0,
    "Q_VY":             1.0,
    "Q_OMEGA":          0.5,
    # Control effort
    "R_DELTA":          0.5,
    "R_AX":             0.01,
    "R_VTHETA":         1.0,
    # Control rate
    "W_DELTA_RATE":     5.0,
    "W_AX_RATE":        0.1,
    "W_VTHETA_RATE":    0.1,
    # Terminal
    "Q_N_TERM":         500.0,
    "Q_ALPHA_TERM":     50.0,
    "Q_PROGRESS_TERM":  5.0,
    # ADMM solver
    "ADMM_RHO":         1.0,
    "ADMM_MAX_ITER":    100,
    "ADMM_TOL":         0.01,
    # Horizon
    "HORIZON":          10,
    "DT":               0.06,
    "V_THETA_MAX":      3.0,
}

# ─── Active config (set by mode selection in main()) ─────────────────────────
RACELINES = {}
BASE = {}
MODE = "Spielberg"
CASCADE_TOP_N = 1
MAX_ALLOWED_COLLISIONS = 0

# ─── Sweep ranges (Spielberg) ────────────────────────────────────────────────
SPIELBERG_FULL_VALUES = {
    "Q_N":              [20, 50, 80, 100, 150, 200, 300, 500, 800, 1000],
    "Q_ALPHA":          [1, 5, 10, 20, 50, 100, 200],
    "Q_PROGRESS":       [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0],
    "Q_VX":             [0, 5, 10, 15, 25, 50, 100],
    "VX_REF":           [3.0, 5.0, 8.0, 10.0, 12.0, 15.0],
    "Q_VY":             [0.1, 0.5, 1.0, 5.0, 10.0],
    "Q_OMEGA":          [0.01, 0.1, 0.5, 1.0, 5.0],
    "R_DELTA":          [0.01, 0.05, 0.1, 0.5, 1.0, 5.0],
    "R_AX":             [0.001, 0.01, 0.05, 0.1, 0.5],
    "R_VTHETA":         [0.01, 0.1, 0.5, 1.0, 5.0, 10.0],
    "W_DELTA_RATE":     [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0],
    "W_AX_RATE":        [0.01, 0.05, 0.1, 0.5, 1.0],
    "W_VTHETA_RATE":    [0.01, 0.05, 0.1, 0.5, 1.0],
    "Q_N_TERM":         [50, 100, 200, 500, 1000],
    "Q_ALPHA_TERM":     [5, 10, 50, 100],
    "Q_PROGRESS_TERM":  [1, 5, 10, 20, 50],
    "ADMM_RHO":         [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0],
    "ADMM_MAX_ITER":    [30, 50, 100, 200],
    "ADMM_TOL":         [0.001, 0.005, 0.01, 0.05],
    "HORIZON":          [10, 15, 20, 25, 30, 40],
    "DT":               [0.03, 0.04, 0.05, 0.06, 0.08, 0.10],
    "V_THETA_MAX":      [2.0, 3.0, 3.5, 5.0, 8.0, 12.0],
}

SPIELBERG_QUICK_VALUES = {
    "Q_N":              [50, 100, 200, 500],
    "Q_ALPHA":          [5, 10, 50],
    "Q_PROGRESS":       [0.5, 1.0, 5.0, 10.0],
    "Q_VX":             [0, 15, 50],
    "R_DELTA":          [0.05, 0.1, 1.0],
    "R_VTHETA":         [0.1, 0.5, 5.0],
    "W_DELTA_RATE":     [0.5, 2.0, 10.0],
    "HORIZON":          [10, 20, 30],
    "DT":               [0.04, 0.05, 0.06],
    "ADMM_RHO":         [0.5, 1.0, 5.0],
    "V_THETA_MAX":      [3.0, 5.0, 8.0, 12.0, 15.0],
}

# ─── Sweep ranges (Hardware) ────────────────────────────────────────────────
HARDWARE_FULL_VALUES = {
    "Q_N":              [100, 200, 300, 500, 800, 1000, 2000, 5000, 10000],
    "Q_ALPHA":          [5, 10, 20, 50, 100, 200, 500],
    "Q_PROGRESS":       [0.1, 0.5, 1.0, 2.0, 5.0, 10.0],
    "Q_VX":             [0, 5, 10, 15, 25, 50, 100],
    "VX_REF":           [1.0, 1.5, 2.0, 3.0, 4.0],
    "Q_VY":             [0.1, 0.5, 1.0, 5.0, 10.0],
    "Q_OMEGA":          [0.1, 0.5, 1.0, 5.0, 10.0],
    "R_DELTA":          [0.01, 0.05, 0.1, 0.5, 1.0, 5.0, 10.0],
    "R_AX":             [0.001, 0.01, 0.05, 0.1],
    "R_VTHETA":         [0.1, 0.5, 1.0, 5.0, 10.0],
    "W_DELTA_RATE":     [0.5, 1.0, 2.0, 5.0, 10.0, 20.0],
    "W_AX_RATE":        [0.01, 0.05, 0.1, 0.5, 1.0],
    "W_VTHETA_RATE":    [0.01, 0.05, 0.1, 0.5, 1.0],
    "Q_N_TERM":         [100, 200, 500, 1000, 5000],
    "Q_ALPHA_TERM":     [10, 50, 100, 200],
    "Q_PROGRESS_TERM":  [1, 5, 10, 20],
    "ADMM_RHO":         [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0],
    "ADMM_MAX_ITER":    [30, 50, 100, 200],
    "ADMM_TOL":         [0.001, 0.005, 0.01, 0.05],
    "HORIZON":          [6, 8, 10, 12, 15, 20],
    "DT":               [0.03, 0.04, 0.05, 0.06, 0.08],
    "V_THETA_MAX":      [1.5, 2.0, 3.0, 3.5, 5.0],
}

HARDWARE_QUICK_VALUES = {
    "Q_N":              [200, 500, 1000, 5000],
    "Q_ALPHA":          [10, 50, 200],
    "Q_PROGRESS":       [0.5, 1.0, 5.0],
    "Q_VX":             [0, 15, 50],
    "R_DELTA":          [0.1, 0.5, 5.0],
    "R_VTHETA":         [0.5, 1.0, 5.0],
    "W_DELTA_RATE":     [1.0, 5.0, 20.0],
    "HORIZON":          [8, 10, 15],
    "DT":               [0.04, 0.06, 0.08],
    "ADMM_RHO":         [0.5, 1.0, 5.0],
    "V_THETA_MAX":      [2.0, 3.0, 5.0],
}

# Active sweep values (set by mode selection in main())
FULL_VALUES = {}
QUICK_VALUES = {}


# ═══════════════════════════════════════════════════════════════════════════════
# Build
# ═══════════════════════════════════════════════════════════════════════════════

def build_binary():
    """Compile the standalone MPCC sim-drive test binary."""
    print("--- Building MPCC test_sim_drive binary ---")
    cmd = [
        "gcc",
        "-D_GNU_SOURCE", "-O3", "-std=c99", "-Wall", "-ffast-math",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Wno-unknown-pragmas",
        f"-I{MPCC_DIR}/include",
        f"-I{MPC_DIR}/include",
        f"{MPCC_DIR}/test/test_sim_drive.c",
        f"{MPCC_DIR}/src/mpcc.c",
        f"{MPCC_DIR}/src/mpcc_vehicle_model.c",
        f"{MPCC_DIR}/src/qp_solver_mpcc.c",
        f"{MPCC_DIR}/src/fp_math_mpcc.c",
        "-lm",
        "-o", BINARY,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print(f"BUILD FAILED:\n{result.stderr}")
        sys.exit(1)
    print(f"  Built: {BINARY}\n")


# ═══════════════════════════════════════════════════════════════════════════════
# Test Runner
# ═══════════════════════════════════════════════════════════════════════════════

def run_test(params: dict, raceline: str = None) -> dict:
    """Run a single test with given parameters. Returns parsed metrics."""
    env = os.environ.copy()
    env["MPCC_TUNING_CSV"] = "1"

    if raceline:
        env["RACELINE_PATH"] = raceline

    for name, value in params.items():
        env[name] = str(value)

    try:
        result = subprocess.run(
            [BINARY], capture_output=True, text=True, timeout=600, env=env
        )
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "passed": 0, "failed": 6}
    except FileNotFoundError:
        print(f"ERROR: Binary '{BINARY}' not found.")
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
    """Composite score — lower is better.

    Any wall collision is an automatic failure (score 500+).

    Spielberg mode (large track, high speeds):
      Primary: max velocity + time above 5 m/s
      Secondary: tracking quality (lat/hdg/vel errors)
      Tertiary: solver efficiency

    Hardware mode (small track, low speeds ~2-5 m/s):
      Primary: tracking quality (avg_lat_err, avg_vel_err)
      Secondary: heading tracking, max lateral error
      Tertiary: velocity, solver efficiency
    """
    if r["status"] != "OK":
        return 999.0

    collisions = r["wall_collisions"]
    if collisions > MAX_ALLOWED_COLLISIONS:
        return 500.0 + collisions * 100.0

    if MODE == "Hardware":
        tracking = (
            r["avg_lat_err"] * 50.0 +
            r["avg_vel_err"] * 20.0 +
            r["max_lat_err"] * 10.0 +
            r["avg_hdg_err"] * 15.0
        )
        velocity_penalty = max(0, 3.0 - r["max_vx"]) * 10.0
        solver = r.get("avg_iters", 0) * 0.2 + r["avg_solve_us"] * 0.001
        return round(tracking + velocity_penalty + solver, 3)
    else:
        # Spielberg scoring
        velocity_penalty = max(0, 12.0 - r["max_vx"]) * 15.0
        time_penalty = max(0, 60 - r["time_above_5ms"]) * 2.0
        tracking = (
            r["avg_lat_err"] * 5.0 +
            r["max_lat_err"] * 1.0 +
            r["avg_vel_err"] * 5.0 +
            r["avg_hdg_err"] * 2.0
        )
        solver = r.get("avg_iters", 0) * 0.3 + r["avg_solve_us"] * 0.002
        return round(velocity_penalty + time_penalty + tracking + solver, 3)


# ═══════════════════════════════════════════════════════════════════════════════
# Combination Generators
# ═══════════════════════════════════════════════════════════════════════════════

def gen_one_at_a_time(values_dict):
    """Vary each parameter one at a time from baseline."""
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
    """Grid over key MPCC parameters:
    Q_N × Q_ALPHA × Q_PROGRESS × HORIZON × V_THETA_MAX (× DT for Hardware)."""
    qn_vals  = values_dict.get("Q_N", [BASE["Q_N"]])
    qa_vals  = values_dict.get("Q_ALPHA", [BASE["Q_ALPHA"]])
    qp_vals  = values_dict.get("Q_PROGRESS", [BASE["Q_PROGRESS"]])
    h_vals   = values_dict.get("HORIZON", [BASE["HORIZON"]])
    vt_vals  = values_dict.get("V_THETA_MAX", [BASE["V_THETA_MAX"]])

    combos = []
    if MODE == "Hardware":
        dt_vals = values_dict.get("DT", [BASE["DT"]])
        for qn, qa, qp, h, vt, dt in itertools.product(
                qn_vals, qa_vals, qp_vals, h_vals, vt_vals, dt_vals):
            w = dict(BASE)
            w["Q_N"] = qn; w["Q_ALPHA"] = qa; w["Q_PROGRESS"] = qp
            w["HORIZON"] = h; w["V_THETA_MAX"] = vt; w["DT"] = dt
            combos.append((f"QN{qn}_QA{qa}_QP{qp}_H{h}_VT{vt}_DT{dt}", w))
    else:
        for qn, qa, qp, h, vt in itertools.product(
                qn_vals, qa_vals, qp_vals, h_vals, vt_vals):
            w = dict(BASE)
            w["Q_N"] = qn; w["Q_ALPHA"] = qa; w["Q_PROGRESS"] = qp
            w["HORIZON"] = h; w["V_THETA_MAX"] = vt
            combos.append((f"QN{qn}_QA{qa}_QP{qp}_H{h}_VT{vt}", w))
    return combos


def gen_secondary_grid(values_dict):
    """Grid: Q_VX × Q_VY × Q_OMEGA × R_DELTA × W_DELTA_RATE."""
    combos = []
    qvx_vals = values_dict.get("Q_VX", [BASE["Q_VX"]])
    qvy_vals = values_dict.get("Q_VY", [BASE["Q_VY"]])
    qom_vals = values_dict.get("Q_OMEGA", [BASE["Q_OMEGA"]])
    rd_vals  = values_dict.get("R_DELTA", [BASE["R_DELTA"]])
    wd_vals  = values_dict.get("W_DELTA_RATE", [BASE["W_DELTA_RATE"]])

    for qvx, qvy, qom, rd, wd in itertools.product(
            qvx_vals, qvy_vals, qom_vals, rd_vals, wd_vals):
        w = dict(BASE)
        w["Q_VX"] = qvx; w["Q_VY"] = qvy; w["Q_OMEGA"] = qom
        w["R_DELTA"] = rd; w["W_DELTA_RATE"] = wd
        combos.append((f"QVX{qvx}_QVY{qvy}_QO{qom}_RD{rd}_WDR{wd}", w))
    return combos


def gen_solver_grid():
    """Grid: ADMM_RHO × ADMM_MAX_ITER × ADMM_TOL."""
    combos = []
    for rho in [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0]:
        for mi in [30, 50, 100, 200]:
            for tol in [0.001, 0.01, 0.05]:
                w = dict(BASE)
                w["ADMM_RHO"] = rho; w["ADMM_MAX_ITER"] = mi; w["ADMM_TOL"] = tol
                combos.append((f"rho={rho}+mi={mi}+tol={tol}", w))
    return combos


def gen_velocity_push():
    """Configurations targeting max velocity with various safety levels."""
    combos = []
    configs = [
        # High progress reward
        {"Q_PROGRESS": 10.0, "Q_N": 200, "Q_ALPHA": 50},
        {"Q_PROGRESS": 20.0, "Q_N": 200, "Q_ALPHA": 50},
        {"Q_PROGRESS": 10.0, "Q_N": 500, "Q_ALPHA": 100},
        {"Q_PROGRESS": 20.0, "Q_N": 500, "Q_ALPHA": 100},
        # High velocity tracking
        {"Q_VX": 100, "VX_REF": 12.0, "Q_N": 200},
        {"Q_VX": 100, "VX_REF": 15.0, "Q_N": 200},
        {"Q_VX": 100, "VX_REF": 12.0, "Q_N": 500},
        # Large V_THETA_MAX (progress uncapped)
        {"V_THETA_MAX": 8.0, "Q_PROGRESS": 5.0},
        {"V_THETA_MAX": 12.0, "Q_PROGRESS": 5.0},
        {"V_THETA_MAX": 8.0, "Q_PROGRESS": 10.0},
        # Agile steering with velocity
        {"R_DELTA": 0.01, "W_DELTA_RATE": 0.5, "Q_PROGRESS": 5.0},
        {"R_DELTA": 0.05, "W_DELTA_RATE": 1.0, "Q_PROGRESS": 5.0},
        # Different horizons
        {"HORIZON": 30, "Q_PROGRESS": 5.0},
        {"HORIZON": 40, "Q_PROGRESS": 5.0},
        {"HORIZON": 15, "DT": 0.08, "Q_PROGRESS": 5.0},
        # Low yaw rate penalty for cornering
        {"Q_OMEGA": 0.01, "Q_PROGRESS": 5.0, "Q_N": 200},
        # High ADMM rho (stiff constraints)
        {"ADMM_RHO": 10.0, "Q_PROGRESS": 5.0},
        {"ADMM_RHO": 20.0, "Q_PROGRESS": 5.0},
    ]
    for cfg in configs:
        w = dict(BASE)
        w.update(cfg)
        label = "+".join(f"{k}={v}" for k, v in cfg.items())
        combos.append((label, w))
    return combos


def gen_fine_tuning(best_weights,
                    pct_range=(0.80, 0.85, 0.90, 0.95, 1.05, 1.10, 1.15, 1.20)):
    """Fine-tuning around best config — +/-5-20% perturbations."""
    combos = []
    skip = {"ADMM_MAX_ITER", "HORIZON"}
    for name, base_val in best_weights.items():
        if base_val == 0 or name in skip:
            continue
        for mult in pct_range:
            new_val = round(base_val * mult, 6)
            if name in ("HORIZON",):
                new_val = max(1, int(new_val))
            elif name in ("ADMM_MAX_ITER",):
                new_val = max(1, int(new_val))
            w = dict(best_weights)
            w[name] = new_val
            pct = int((mult - 1.0) * 100)
            sign = "+" if pct >= 0 else ""
            combos.append((f"FT:{name}{sign}{pct}%", w))

    # Pairwise perturbation of key params
    key_params = ["Q_N", "Q_ALPHA", "Q_PROGRESS", "R_DELTA", "V_THETA_MAX", "HORIZON",
                  "ADMM_RHO", "W_DELTA_RATE"]
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
                   if k not in ("ADMM_MAX_ITER",) and best_weights[k] != 0]
    for i in range(n):
        w = dict(best_weights)
        num_perturb = random.randint(2, min(6, len(tune_params)))
        params_to_perturb = random.sample(tune_params, num_perturb)
        for name in params_to_perturb:
            if name == "HORIZON":
                w[name] = random.choice(range(5, 41))
            elif name == "DT":
                w[name] = random.choice([0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1])
            else:
                mult = random.uniform(0.6, 1.5)
                w[name] = round(w[name] * mult, 6)
        combos.append((f"RND_{i}", w))
    return combos


# ═══════════════════════════════════════════════════════════════════════════════
# Deduplication
# ═══════════════════════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════════════════════
# Incremental CSV Writer
# ═══════════════════════════════════════════════════════════════════════════════

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


# ═══════════════════════════════════════════════════════════════════════════════
# Worker function for parallel execution
# ═══════════════════════════════════════════════════════════════════════════════

def _run_single(args):
    """Worker: run one test and return scored result. Picklable for multiprocessing."""
    label, params, raceline, raceline_label, phase_name = args
    r = run_test(params, raceline)
    score = compute_score(r)
    r["label"] = label
    r["score"] = score
    r["phase"] = phase_name
    r["raceline"] = raceline_label or "default"
    r.update(params)
    return r


# ═══════════════════════════════════════════════════════════════════════════════
# Phase Runner
# ═══════════════════════════════════════════════════════════════════════════════

def run_phase(phase_name, combos, results, t0,
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
        for i, (label, params) in enumerate(combos):
            elapsed = time.time() - t0
            rate = max(len(results), 1) / max(elapsed, 0.01)
            eta = (total - i - 1) / max(rate, 0.01)
            tag = f"{label}|{raceline_label}" if raceline_label else label
            print(f"  [{i+1:4d}/{total}] {tag:60s} ", end="", flush=True)

            r = run_test(params, raceline)
            score = compute_score(r)
            r["label"] = label
            r["score"] = score
            r["phase"] = phase_name
            r["raceline"] = raceline_label or "default"
            r.update(params)
            results.append(r)
            if csv_writer:
                csv_writer.write_row(r)

            if r["status"] != "OK":
                failed += 1
                print(f"FAIL  (ETA {eta:.0f}s)")
            elif r["wall_collisions"] > MAX_ALLOWED_COLLISIONS:
                failed += 1
                print(f"wc={r['wall_collisions']}  (ETA {eta:.0f}s)")
            else:
                passed += 1
                wc_tag = f"wc={r['wall_collisions']} " if r["wall_collisions"] > 0 else ""
                print(f"sc={score:7.2f}  vx={r['max_vx']:.1f}  "
                      f"t5={r['time_above_5ms']:.0f}s  "
                      f"lat={r['avg_lat_err']:.3f}  "
                      f"ve={r.get('avg_vel_err', 0):.2f}  {wc_tag}(ETA {eta:.0f}s)")
    else:
        work_items = [
            (label, params, raceline, raceline_label, phase_name)
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

                if r["status"] != "OK":
                    failed += 1
                    print(f"  [{done_count:4d}/{total}] {tag:60s} "
                          f"FAIL  (ETA {eta:.0f}s)")
                elif r["wall_collisions"] > MAX_ALLOWED_COLLISIONS:
                    failed += 1
                    print(f"  [{done_count:4d}/{total}] {tag:60s} "
                          f"wc={r['wall_collisions']}  (ETA {eta:.0f}s)")
                else:
                    passed += 1
                    wc_tag = f"wc={r['wall_collisions']} " if r["wall_collisions"] > 0 else ""
                    print(f"  [{done_count:4d}/{total}] {tag:60s} "
                          f"sc={score:7.2f}  vx={r['max_vx']:.1f}  "
                          f"t5={r['time_above_5ms']:.0f}s  "
                          f"lat={r['avg_lat_err']:.3f}  "
                          f"ve={r.get('avg_vel_err', 0):.2f}  {wc_tag}(ETA {eta:.0f}s)")

    return passed, failed


# ═══════════════════════════════════════════════════════════════════════════════
# ROS2 Validation
# ═══════════════════════════════════════════════════════════════════════════════

def validate_on_ros2(params: dict, raceline: str = None, duration: int = 120):
    """Run the full ROS2 sim with the given parameters via run_sim.sh."""
    print("\n" + "="*60)
    print("ROS2 Validation (run_sim.sh)")
    print("="*60)

    env = os.environ.copy()
    for name, value in params.items():
        env[name] = str(value)

    cmd = [RUN_SIM_SH, str(duration)]
    if raceline:
        cmd.append(raceline)

    print(f"  Running: {' '.join(cmd)}")
    print(f"  Env overrides: {len(params)} parameters")
    for k, v in sorted(params.items()):
        if abs(v - BASE.get(k, -999)) > 1e-6:
            print(f"    {k}={v}")

    result = subprocess.run(cmd, capture_output=True, text=True, timeout=300, env=env)
    print(f"\n  Exit code: {result.returncode}")
    for line in result.stdout.splitlines()[-10:]:
        print(f"  {line}")

    return result.returncode


# ═══════════════════════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    global RACELINES, BASE, FULL_VALUES, QUICK_VALUES, MODE, CASCADE_TOP_N, MAX_ALLOWED_COLLISIONS

    # ─── Parse CLI arguments ─────────────────────────────────────────────
    quick = "--quick" in sys.argv
    phase_only = None
    raceline_arg = None
    num_workers = 1
    cascade_top = 1
    mode_arg = None
    do_validate = "--validate" in sys.argv
    no_build = "--no-build" in sys.argv

    positional_args = [a for a in sys.argv[1:] if not a.startswith("-")]
    skip_next = set()
    for i, arg in enumerate(sys.argv):
        if arg in ("--phase", "--raceline", "--jobs", "-j", "--cascade-top") and i + 1 < len(sys.argv):
            skip_next.add(sys.argv[i + 1])
    positional_args = [a for a in positional_args if a not in skip_next]
    if positional_args:
        mode_arg = positional_args[0]

    for i, arg in enumerate(sys.argv):
        if arg == "--phase" and i + 1 < len(sys.argv):
            phase_only = int(sys.argv[i + 1])
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_arg = sys.argv[i + 1]
        if arg in ("--jobs", "-j") and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "--cascade-top" and i + 1 < len(sys.argv):
            cascade_top = int(sys.argv[i + 1])

    # ─── Mode selection ──────────────────────────────────────────────────
    if mode_arg and mode_arg.lower() in ("hardware", "hw"):
        MODE = "Hardware"
        RACELINES.update(HARDWARE_RACELINES)
        BASE.update(HARDWARE_BASE)
        FULL_VALUES.update(HARDWARE_FULL_VALUES)
        QUICK_VALUES.update(HARDWARE_QUICK_VALUES)
        MAX_ALLOWED_COLLISIONS = 0
    elif mode_arg and mode_arg.lower() in ("spielberg", "sp"):
        MODE = "Spielberg"
        RACELINES.update(SPIELBERG_RACELINES)
        BASE.update(SPIELBERG_BASE)
        FULL_VALUES.update(SPIELBERG_FULL_VALUES)
        QUICK_VALUES.update(SPIELBERG_QUICK_VALUES)
        MAX_ALLOWED_COLLISIONS = 0
    else:
        print("ERROR: First argument must be mode: Spielberg or Hardware")
        print("Usage: python3 test/tune_mpcc.py <Spielberg|Hardware> [options]")
        sys.exit(1)

    CASCADE_TOP_N = max(1, cascade_top)

    if num_workers <= 0:
        num_workers = max(1, multiprocessing.cpu_count() - 1)

    print(f"\n  Mode:          {MODE}")
    print(f"  Workers:       {num_workers} "
          f"({'sequential' if num_workers == 1 else 'parallel'})")
    print(f"  Cascade top-N: {CASCADE_TOP_N}")
    print(f"  Max collisions allowed: {MAX_ALLOWED_COLLISIONS}")
    print(f"  Quick mode:    {quick}")

    os.chdir(MPCC_DIR)

    # Build
    if not no_build:
        build_binary()

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

    if raceline_arg:
        if raceline_arg in available_racelines:
            available_racelines = {raceline_arg: available_racelines[raceline_arg]}
            print(f"  Filtered to raceline: [{raceline_arg}]")
        else:
            print(f"  WARNING: --raceline {raceline_arg} not found, using all available")

    values = QUICK_VALUES if quick else FULL_VALUES
    results = []
    t0 = time.time()
    total_p = total_f = 0

    # Incremental CSV
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = os.path.join(MPCC_DIR, "tuning_results")
    os.makedirs(results_dir, exist_ok=True)
    outfile = os.path.join(results_dir, f"tuning_{MODE.lower()}_{timestamp}.csv")
    fieldnames = (["label", "phase", "raceline", "score",
                   "passed", "failed", "max_lat_err", "avg_lat_err",
                   "max_hdg_err", "avg_hdg_err", "max_vx",
                   "avg_vel_err", "max_vel_err",
                   "avg_solve_us", "max_solve_us",
                   "wall_collisions", "time_above_5ms", "avg_iters", "status"]
                  + list(BASE.keys()))
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results file: {outfile} (incremental)\n")

    def should_run(phase_num):
        return phase_only is None or phase_only == phase_num

    def get_top_n_params(rl_tag, n=None):
        """Return list of up to N best-so-far params dicts for given raceline."""
        if n is None:
            n = CASCADE_TOP_N
        rl_results = [r for r in results
                      if r.get("raceline") == rl_tag and r.get("score", 999) < 500]
        if not rl_results:
            return []
        rl_results.sort(key=lambda x: x["score"])
        seen = set()
        unique = []
        for r in rl_results:
            key = tuple(sorted((k, round(r.get(k, BASE[k]), 4)
                                if isinstance(r.get(k, BASE[k]), float)
                                else r.get(k, BASE[k]))
                               for k in BASE.keys()))
            if key not in seen:
                seen.add(key)
                params = {k: r.get(k, BASE[k]) for k in BASE.keys()}
                unique.append((r, params))
            if len(unique) >= n:
                break
        if unique:
            for i, (r, _) in enumerate(unique):
                print(f"  Top-{i+1}: {r['label']} "
                      f"(score={r['score']:.2f}, vx={r.get('max_vx', 0):.1f}, "
                      f"wc={r.get('wall_collisions', 0)})")
        return [p for _, p in unique]

    def get_best_params(rl_tag):
        top = get_top_n_params(rl_tag, n=1)
        return top[0] if top else None

    def update_base(new_params):
        if new_params:
            for k in BASE:
                if k in new_params:
                    BASE[k] = new_params[k]

    # ─── Run all phases across all available racelines ───────────────────
    original_base = dict(BASE)

    for rl_tag, rl_path in available_racelines.items():
        # Reset BASE for each raceline
        for k, v in original_base.items():
            BASE[k] = v

        print(f"\n{'#'*80}")
        print(f"# Mode: {MODE}  Raceline: [{rl_tag}]")
        print(f"{'#'*80}")

        # ─── Phase 1: One-at-a-time ─────────────────────────────────
        if should_run(1):
            p, f = run_phase("Phase 1: One-at-a-time",
                             gen_one_at_a_time(values), results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        # ─── Phase 2: Primary grid ──────────────────────────────────
        if should_run(2):
            p, f = run_phase("Phase 2: Primary grid (Q_N*Q_ALPHA*Q_PROGRESS*HORIZON*V_THETA_MAX)",
                             gen_primary_grid(values), results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        # ─── CASCADE: run phases 3+ for each of top-N from 1+2 ──────
        top_configs = get_top_n_params(rl_tag)
        if not top_configs:
            top_configs = [dict(original_base)]

        for ci, cascade_base in enumerate(top_configs):
            if CASCADE_TOP_N > 1:
                print(f"\n  >>> CASCADE branch {ci+1}/{len(top_configs)} <<<")

            for k, v in cascade_base.items():
                BASE[k] = v

            # ─── Phase 3: Secondary grid ────────────────────────────
            if should_run(3):
                p, f = run_phase(f"Phase 3: Secondary grid [branch {ci+1}]",
                                 gen_secondary_grid(values), results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

            # CASCADE: update to best of 1-3
            cascade_params = get_best_params(rl_tag)
            if cascade_params:
                update_base(cascade_params)

            if not quick:
                # ─── Phase 4: Solver parameters ─────────────────────
                if should_run(4):
                    p, f = run_phase(f"Phase 4: Solver grid [branch {ci+1}]",
                                     gen_solver_grid(), results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

                # CASCADE: update to best of 1-4
                cascade_params = get_best_params(rl_tag)
                if cascade_params:
                    update_base(cascade_params)

                # ─── Phase 5: Velocity push ─────────────────────────
                if should_run(5):
                    p, f = run_phase(f"Phase 5: Velocity push [branch {ci+1}]",
                                     gen_velocity_push(), results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

            # ─── Phase 6: Fine-tuning around best ───────────────────
            if should_run(6):
                best_params = get_best_params(rl_tag)
                if best_params:
                    p, f = run_phase(f"Phase 6: Fine-tuning [branch {ci+1}]",
                                     gen_fine_tuning(best_params), results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

            # ─── Phase 7: Random neighbors ──────────────────────────
            if should_run(7):
                best_params = get_best_params(rl_tag)
                if best_params:
                    n = 150 if not quick else 50
                    p, f = run_phase(f"Phase 7: Random ({n}) [branch {ci+1}]",
                                     gen_random_neighbors(best_params, n), results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

        # Reset BASE for next raceline
        for k, v in original_base.items():
            BASE[k] = v

    # ─── Results ─────────────────────────────────────────────────────────
    results.sort(key=lambda x: x.get("score", 999))
    elapsed = time.time() - t0

    print(f"\n{'='*80}")
    print(f"[{MODE}] COMPLETED {len(results)} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    print(f"{'='*80}")

    # Write sorted final CSV
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
        print(f"[{MODE}] TOP 30 (lowest score = best)")
        print(f"{'='*80}")
        fmt = "{:<4} {:<50} {:>7} {:>6} {:>5} {:>5} {:>5} {:>5} {:>5} {:>3}"
        print(fmt.format("Rank", "Label", "Score", "AvgVE", "MaxVx", "T>5s",
                          "AvgLt", "QN", "QA", "QP"))
        print("-" * 110)
        for i, r in enumerate(passing[:30]):
            print(fmt.format(
                i+1, r['label'][:50], f"{r['score']:.1f}",
                f"{r.get('avg_vel_err', 0):.2f}", f"{r['max_vx']:.1f}",
                f"{r['time_above_5ms']:.0f}",
                f"{r['avg_lat_err']:.3f}",
                f"{r.get('Q_N', '-')}",
                f"{r.get('Q_ALPHA', '-')}",
                f"{r.get('Q_PROGRESS', '-')}"))

        best = passing[0]
        print(f"\n  BEST CONFIGURATION ({MODE}):")
        print(f"    Score: {best['score']:.2f}")
        print(f"    Max velocity: {best['max_vx']:.2f} m/s")
        print(f"    Time > 5 m/s: {best['time_above_5ms']:.1f} s")
        print(f"    Avg lat err:  {best['avg_lat_err']:.4f} m")
        print(f"    Avg vel err:  {best.get('avg_vel_err', 0):.2f} m/s")
        print(f"    Walls:        {best['wall_collisions']}")
        print(f"    ---")
        for k in sorted(BASE.keys()):
            print(f"    {k:20s} = {best.get(k, BASE[k])}")

        # Print as env var command
        print(f"\n  Run with:")
        env_parts = []
        for k in sorted(BASE.keys()):
            v = best.get(k, BASE[k])
            env_parts.append(f"{k}={v}")
        print(f"    {' '.join(env_parts)} ./test_sim_drive")

        # Validate on ROS2
        if do_validate:
            best_params = {k: best.get(k, BASE[k]) for k in BASE.keys()}
            validate_on_ros2(best_params)

    return 0


if __name__ == "__main__":
    main()
