#!/usr/bin/env python3
"""
MPCC Weight Tuning — Hardware Map
==================================
Sweeps MPCC controller weights (Global Frame Liniger MPCC, ADMM+Riccati)
via env vars and the standalone test_sim_drive binary.

Usage:
    python3 test/tune_mpcc.py                         # Full sweep (all CPUs)
    python3 test/tune_mpcc.py --jobs 8                # Use 8 parallel workers
    python3 test/tune_mpcc.py -j 4                    # Use 4 workers
    python3 test/tune_mpcc.py --objective racer       # Optimize for speed (default)
    python3 test/tune_mpcc.py --objective tracker     # Optimize for tracking
    python3 test/tune_mpcc.py --raceline my_track_raceline.csv

The sweep runs 8 phases:
    Phase 1: One-at-a-time parameter sensitivity
    Phase 2: Primary grid (Q_CONTOURING x Q_LAG x Q_PROGRESS x HORIZON x DT)
    Phase 3: (Skipped — no wall-margin concept in MPCC)
    Phase 4: Secondary grid (Q_VY x Q_OMEGA x R_DELTA x W_DELTA_RATE x V_THETA_MAX)
    Phase 5: Solver parameters (ADMM_RHO x ADMM_MAX_ITER x ADMM_TOL)
    Phase 6: Fine-tuning around best config
    Phase 7: Random neighbor exploration
    Phase 8: Random exploitation around branch best

Top 10 configurations from Phase 2 are screened with a smaller Phase 4 sweep.
The single global-best configuration is then optimized through Phases 5-8 for
10 consecutive passes.
"""

import subprocess
import os
import sys
import csv
import itertools
import time
import random
import hashlib
import multiprocessing
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED

# ==============================================================================
# PATHS
# ==============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MPCC_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_DIR = os.path.dirname(MPCC_DIR)
TRAJ_DIR = os.path.join(PROJECT_DIR, "f1tenth_planning", "trajectories")

# ==============================================================================
# HARDWARE MAP CONFIGURATION
# ==============================================================================

DEFAULT_RACELINE_NAME = "my_track_raceline.csv"
RACELINE_PATH = os.path.join(TRAJ_DIR, DEFAULT_RACELINE_NAME)
RACELINE_TAG = "my_track"

# Base configuration — starting point for all sweeps (tuned for hardware map)
BASE_CONFIG = {
    # Contouring tracking
    "Q_CONTOURING":      1000.0,
    "Q_LAG":             700.0,
    "Q_PROGRESS":        5.0,
    # State regularization (Q_VX/VX_REF fixed — pure MPCC determines speed via progress cost)
    "Q_VX":              0.0,
    "VX_REF":            5.0,
    "Q_VY":              3.5,
    "Q_OMEGA":           0.7,
    # Control effort (R_AX/W_AX_RATE fixed — minimal impact on MPCC behavior)
    "R_DELTA":           6.5,
    "R_AX":              0.014149,
    "R_VTHETA":          1.0,
    # Control rate
    "W_DELTA_RATE":      2.0,
    "W_AX_RATE":         0.1,
    "W_VTHETA_RATE":     0.1,
    # Terminal
    "Q_CONTOURING_TERM": 450.0,
    "Q_LAG_TERM":        950.0,
    "Q_PROGRESS_TERM":   5.0,
    # ADMM solver
    "ADMM_RHO":          17.0,
    "ADMM_MAX_ITER":     50,
    "ADMM_TOL":          0.05,
    # Horizon
    "HORIZON":           7,
    "DT":                0.035,
    "V_THETA_MAX":       8.0,
}

# Override base for racer objective (push speed harder)
RACER_BASE_OVERRIDES = {
    "Q_PROGRESS": 8.0,
    "V_THETA_MAX": 10.0,
}

# ==============================================================================
# SWEEP VALUE RANGES — PHASE 2 (Primary Grid)
# ==============================================================================

PHASE2_VALUES = {
    "Q_CONTOURING": [200, 500, 700, 1000, 1500, 2000, 3000, 5000],
    "Q_LAG":        [150, 350, 500, 700, 1000, 1500, 2500, 5000],
    "Q_PROGRESS":   [0.3, 0.5, 1.0, 3.0, 5.0, 8.0, 15.0],
    "HORIZON":      [5, 7, 8, 10, 12, 15, 20],
    "DT":           [0.02, 0.025, 0.03, 0.035, 0.04, 0.05, 0.06],
}

# ==============================================================================
# SWEEP VALUE RANGES — ALL PARAMETERS (one-at-a-time and fine-tuning)
# ==============================================================================

FULL_SWEEP_VALUES = {
    "Q_CONTOURING":      [50, 200, 500, 700, 1000, 1500, 2000, 3000, 5000, 8000],
    "Q_LAG":             [50, 150, 350, 500, 700, 1000, 1500, 2500, 5000],
    "Q_PROGRESS":        [0.1, 0.3, 0.5, 1.0, 3.0, 5.0, 8.0, 15.0, 25.0],
    "Q_VY":              [0.1, 0.5, 1.0, 3.5, 5.0, 10.0],
    "Q_OMEGA":           [0.05, 0.1, 0.3, 0.5, 0.7, 1.0, 3.0, 5.0],
    "R_DELTA":           [0.01, 0.1, 0.5, 1.0, 3.0, 6.5, 10.0, 20.0],
    "R_VTHETA":          [0.1, 0.5, 1.0, 2.0, 5.0, 10.0],
    "W_DELTA_RATE":      [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0],
    "W_VTHETA_RATE":     [0.01, 0.05, 0.1, 0.5, 1.0],
    "Q_CONTOURING_TERM": [100, 200, 450, 700, 1000, 2000],
    "Q_LAG_TERM":        [200, 500, 950, 1500, 3000, 5000],
    "Q_PROGRESS_TERM":   [1, 3, 5, 10, 20],
    "HORIZON":           [5, 7, 8, 10, 12, 15, 20],
    "DT":                [0.02, 0.025, 0.03, 0.035, 0.04, 0.05, 0.06, 0.08],
    "ADMM_RHO":          [0.5, 1.0, 2.0, 5.0, 10.0, 17.0, 25.0, 50.0],
    "ADMM_MAX_ITER":     [30, 50, 100, 150, 200, 300],
    "ADMM_TOL":          [0.001, 0.005, 0.01, 0.03, 0.05, 0.1],
    "V_THETA_MAX":       [1.0, 2.0, 4.0, 6.0, 8.0, 10.0, 15.0],
}

# ==============================================================================
# PHASE 4: Secondary Grid Values
# Q_VY x Q_OMEGA x R_DELTA x W_DELTA_RATE x V_THETA_MAX
# ==============================================================================

PHASE4_VALUES = {
    "Q_VY":         [0.5, 1.0, 3.5, 5.0, 10.0],
    "Q_OMEGA":      [0.1, 0.5, 0.7, 1.0, 3.0],
    "R_DELTA":      [0.5, 1.0, 3.0, 6.5, 10.0],
    "W_DELTA_RATE": [0.5, 1.0, 2.0, 5.0, 10.0],
    "V_THETA_MAX":  [4.0, 6.0, 8.0, 10.0, 15.0],
}

# ==============================================================================
# PHASE 5: Solver Grid Values
# ADMM_RHO x ADMM_MAX_ITER x ADMM_TOL
# ==============================================================================

PHASE5_VALUES = {
    "ADMM_RHO":      [1.0, 5.0, 10.0, 17.0, 25.0, 50.0],
    "ADMM_MAX_ITER": [30, 50, 100, 150, 200],
    "ADMM_TOL":      [0.005, 0.01, 0.03, 0.05, 0.1],
}

# ==============================================================================
# RANDOM NEIGHBOR PROFILES
# ==============================================================================

RANDOM_PROFILES = {
    "racer": {
        "num_perturb_range": (3, 7),
        "default_multipliers": [0.85, 0.95, 1.0, 1.1, 1.2],
        "param_multipliers": {
            "Q_CONTOURING":  [0.85, 0.95, 1.0, 1.1, 1.2],
            "Q_LAG":         [0.85, 0.95, 1.0, 1.1, 1.2],
            "Q_PROGRESS":    [0.9, 1.0, 1.1, 1.2, 1.4, 1.6],
            "Q_VY":          [0.7, 0.9, 1.0, 1.15, 1.3],
            "Q_OMEGA":       [0.7, 0.9, 1.0, 1.15, 1.3],
            "R_DELTA":       [0.8, 0.9, 1.0, 1.1, 1.25],
            "R_VTHETA":      [0.8, 0.9, 1.0, 1.15, 1.3],
            "W_DELTA_RATE":  [0.8, 0.9, 1.0, 1.15, 1.3],
            "W_VTHETA_RATE": [0.7, 0.85, 1.0, 1.2, 1.4],
            "ADMM_RHO":      [0.75, 0.9, 1.0, 1.15, 1.35],
            "V_THETA_MAX":   [0.8, 0.9, 1.0, 1.15, 1.3],
        },
        "discrete": {
            "HORIZON":       [5, 7, 8, 10, 12, 15, 20],
            "DT":            [0.025, 0.03, 0.035, 0.04, 0.05, 0.06],
            "ADMM_MAX_ITER": [30, 50, 100, 150, 200],
        },
    },
    "tracker": {
        "num_perturb_range": (3, 6),
        "default_multipliers": [0.85, 0.95, 1.0, 1.1, 1.2],
        "param_multipliers": {
            "Q_CONTOURING":  [0.9, 0.97, 1.0, 1.08, 1.18],
            "Q_LAG":         [0.9, 0.97, 1.0, 1.08, 1.18],
            "Q_PROGRESS":    [0.85, 0.95, 1.0, 1.1, 1.2],
            "Q_VY":          [0.8, 0.9, 1.0, 1.15, 1.3],
            "Q_OMEGA":       [0.8, 0.9, 1.0, 1.15, 1.3],
            "R_DELTA":       [0.85, 0.95, 1.0, 1.1, 1.2],
            "R_VTHETA":      [0.85, 0.95, 1.0, 1.1, 1.2],
            "W_DELTA_RATE":  [0.85, 0.95, 1.0, 1.1, 1.2],
            "W_VTHETA_RATE": [0.7, 0.85, 1.0, 1.2, 1.4],
            "ADMM_RHO":      [0.85, 0.95, 1.0, 1.1, 1.2],
            "V_THETA_MAX":   [0.85, 0.95, 1.0, 1.1, 1.2],
        },
        "discrete": {
            "HORIZON":       [5, 7, 8, 10, 12, 15],
            "DT":            [0.025, 0.03, 0.035, 0.04, 0.05],
            "ADMM_MAX_ITER": [30, 50, 100, 150, 200],
        },
    },
    "racer_exploit": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.96, 0.99, 1.0, 1.03, 1.07],
        "param_multipliers": {
            "Q_CONTOURING":  [0.96, 0.99, 1.0, 1.03, 1.07],
            "Q_LAG":         [0.96, 0.99, 1.0, 1.03, 1.07],
            "Q_PROGRESS":    [0.97, 1.0, 1.03, 1.06, 1.1],
            "Q_VY":          [0.92, 0.98, 1.0, 1.05, 1.1],
            "Q_OMEGA":       [0.92, 0.98, 1.0, 1.05, 1.1],
            "R_DELTA":       [0.94, 0.99, 1.0, 1.04, 1.08],
            "R_VTHETA":      [0.94, 0.99, 1.0, 1.04, 1.08],
            "W_DELTA_RATE":  [0.92, 0.98, 1.0, 1.05, 1.1],
            "W_VTHETA_RATE": [0.9, 0.97, 1.0, 1.05, 1.1],
            "ADMM_RHO":      [0.9, 0.97, 1.0, 1.06, 1.12],
            "V_THETA_MAX":   [0.94, 0.99, 1.0, 1.04, 1.08],
        },
        "discrete": {
            "HORIZON":       [5, 7, 8, 10, 12, 15],
            "DT":            [0.025, 0.03, 0.035, 0.04, 0.05],
            "ADMM_MAX_ITER": [50, 100, 150, 200],
        },
    },
    "tracker_exploit": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.95, 0.98, 1.0, 1.02, 1.05],
        "param_multipliers": {
            "Q_CONTOURING":  [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_LAG":         [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_PROGRESS":    [0.95, 0.98, 1.0, 1.03, 1.06],
            "Q_VY":          [0.9, 0.97, 1.0, 1.05, 1.1],
            "Q_OMEGA":       [0.9, 0.97, 1.0, 1.05, 1.1],
            "R_DELTA":       [0.92, 0.98, 1.0, 1.05, 1.1],
            "R_VTHETA":      [0.92, 0.98, 1.0, 1.05, 1.1],
            "W_DELTA_RATE":  [0.92, 0.98, 1.0, 1.05, 1.1],
            "W_VTHETA_RATE": [0.9, 0.97, 1.0, 1.05, 1.1],
            "ADMM_RHO":      [0.92, 0.98, 1.0, 1.05, 1.1],
            "V_THETA_MAX":   [0.92, 0.98, 1.0, 1.05, 1.1],
        },
        "discrete": {
            "HORIZON":       [5, 7, 8, 10, 12],
            "DT":            [0.03, 0.035, 0.04, 0.05],
            "ADMM_MAX_ITER": [50, 100, 150],
        },
    },
}

# ==============================================================================
# CONSTANTS
# ==============================================================================

INT_PARAMS = {"HORIZON", "ADMM_MAX_ITER"}

CASCADE_TOP_N = 10
SEED = 42
GLOBAL_OPTIMIZATION_PASSES = 10
PHASE7_RANDOM_COUNT = {"racer": 4400, "tracker": 3600}
PHASE8_RANDOM_COUNT = {"racer": 2400, "tracker": 1800}

# Print order matching MPCC config structure
MPCC_PRINT_ORDER = (
    "Q_CONTOURING", "Q_LAG", "Q_PROGRESS",
    "Q_VY", "Q_OMEGA",
    "R_DELTA", "R_VTHETA",
    "W_DELTA_RATE", "W_VTHETA_RATE",
    "Q_CONTOURING_TERM", "Q_LAG_TERM", "Q_PROGRESS_TERM",
    "ADMM_RHO", "ADMM_MAX_ITER", "ADMM_TOL",
    "HORIZON", "DT", "V_THETA_MAX",
)

# Working copy of base config (modified during cascade)
BASE = {}


def infer_raceline_tag(path: str) -> str:
    """Create a compact label from raceline filename."""
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem.endswith("_raceline"):
        stem = stem[: -len("_raceline")]
    return stem or "unknown"


def resolve_raceline_path(path_arg: str) -> str:
    """Resolve raceline path argument to an absolute path."""
    if os.path.isabs(path_arg):
        return path_arg
    traj_candidate = os.path.join(TRAJ_DIR, path_arg)
    if os.path.exists(traj_candidate):
        return os.path.abspath(traj_candidate)
    return os.path.abspath(os.path.join(MPCC_DIR, path_arg))


def iter_ordered_base_keys():
    """Yield BASE keys in MPCC config order, then any remaining."""
    seen = set()
    for key in MPCC_PRINT_ORDER:
        if key in BASE:
            seen.add(key)
            yield key
    for key in BASE.keys():
        if key not in seen:
            yield key


# ==============================================================================
# UTILITY FUNCTIONS
# ==============================================================================

def canonicalize_params(params: dict) -> dict:
    """Normalize params to the values MPCC actually receives via env parsing."""
    out = dict(params)
    for k in INT_PARAMS:
        if k in out:
            out[k] = int(float(out[k]))
    return out


def is_valid_config(params: dict) -> bool:
    """Check if configuration is valid."""
    h = int(params.get("HORIZON", BASE.get("HORIZON", 7)))
    if h < 2 or h > 50:
        return False
    dt = float(params.get("DT", BASE.get("DT", 0.035)))
    if dt < 0.01 or dt > 0.2:
        return False
    return True


def config_hash(params: dict) -> str:
    """Create unique hash for a config to avoid duplicates."""
    eff = canonicalize_params(params)
    key = tuple(sorted((k, round(v, 4) if isinstance(v, float) else v)
                       for k, v in eff.items()))
    return hashlib.md5(str(key).encode()).hexdigest()[:12]


# ==============================================================================
# TEST RUNNER
# ==============================================================================

def run_test(params: dict, binary: str) -> dict:
    """Run a single MPCC test with given parameters. Returns parsed metrics."""
    env = os.environ.copy()
    env["MPCC_TUNING_CSV"] = "1"
    env["RACELINE_PATH"] = RACELINE_PATH

    effective_params = canonicalize_params(params)
    for name, value in effective_params.items():
        env[name] = str(value)

    try:
        result = subprocess.run(
            [binary], capture_output=True, text=True, timeout=600, env=env
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
                    "return_code": result.returncode,
                    "passed": int(parts[1]),
                    "failed": int(parts[2]),
                    "max_contouring_err": float(parts[3]),
                    "avg_contouring_err": float(parts[4]),
                    "max_heading_err": float(parts[5]),
                    "avg_heading_err": float(parts[6]),
                    "max_vx": float(parts[7]),
                    "avg_solve_us": float(parts[8]),
                    "max_solve_us": float(parts[9]),
                    "wall_collisions": int(parts[10]),
                    "time_above_5ms": float(parts[11]),
                    "max_vel_err": float(parts[12]) if len(parts) > 12 else 0.0,
                    "avg_vel": float(parts[13]) if len(parts) > 13 else 0.0,
                    "avg_iters": float(parts[14]) if len(parts) > 14 else 0.0,
                    "avg_rho": float(parts[15]) if len(parts) > 15 else 0.0,
                    "avg_rho_u": float(parts[16]) if len(parts) > 16 else 0.0,
                    "avg_adapt_updates": float(parts[17]) if len(parts) > 17 else 0.0,
                    "avg_clip_events": float(parts[18]) if len(parts) > 18 else 0.0,
                }
            except (IndexError, ValueError):
                pass

    if result.returncode != 0:
        return {"status": "EXIT_FAIL", "return_code": result.returncode,
                "passed": 0, "failed": 6}
    return {"status": "NO_CSV", "return_code": result.returncode,
            "passed": 0, "failed": 6}


# ==============================================================================
# SCORING
# ==============================================================================

def is_safe_result(r: dict) -> bool:
    """True when run is valid and collision-free."""
    return r.get("status") == "OK" and int(r.get("wall_collisions", 999)) == 0


def compute_tracker_score(r: dict) -> float:
    """Tracker score: minimize contouring/heading errors (lower is better).

    MPCC uses global-frame contouring control — contouring_err is the
    perpendicular distance from the reference path, heading_err is the
    heading deviation from the path tangent. We weight these heavily since
    tracker mode cares about path following accuracy.
    """
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return 2000.0 + 100.0 * float(r.get("wall_collisions", 0))

    tracking = (
        r["avg_contouring_err"] * 80.0 +
        r["max_contouring_err"] * 15.0 +
        r["avg_heading_err"] * 35.0 +
        r["max_heading_err"] * 8.0 +
        r.get("max_vel_err", 0) * 4.0
    )
    solver = r.get("avg_iters", 0) * 0.2 + r["avg_solve_us"] * 0.001
    return round(tracking + solver, 3)


def compute_racer_score(r: dict) -> float:
    """Racer score: maximize average velocity while staying collision-free.

    MPCC naturally finds its own racing line via progress maximization,
    so the primary metric is how fast it goes.
    """
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return 2000.0 + 100.0 * float(r.get("wall_collisions", 0))

    return round(-r.get("avg_vel", 0.0), 6)


def apply_scores(r: dict, objective: str) -> dict:
    """Attach scores to a result row."""
    r["tracker_score"] = compute_tracker_score(r)
    r["racer_score"] = compute_racer_score(r)
    r["score"] = r["tracker_score"] if objective == "tracker" else r["racer_score"]
    return r


# ==============================================================================
# CONFIG GENERATORS
# ==============================================================================

def gen_one_at_a_time() -> list:
    """Phase 1: Vary each parameter one at a time from baseline."""
    combos = [("BASELINE", dict(BASE))]

    for name, values in FULL_SWEEP_VALUES.items():
        for v in values:
            if abs(v - BASE.get(name, -999)) < 1e-6:
                continue
            w = dict(BASE)
            w[name] = v
            if is_valid_config(w):
                combos.append((f"{name}={v}", w))

    return combos


def gen_primary_grid() -> list:
    """Phase 2: Primary grid — core MPCC contouring weights + discretization.

    Q_CONTOURING x Q_LAG x Q_PROGRESS x HORIZON x DT
    """
    combos = []

    qc_vals = PHASE2_VALUES["Q_CONTOURING"]
    ql_vals = PHASE2_VALUES["Q_LAG"]
    qp_vals = PHASE2_VALUES["Q_PROGRESS"]
    h_vals = PHASE2_VALUES["HORIZON"]
    dt_vals = PHASE2_VALUES["DT"]

    for qc, ql, qp, h, dt in itertools.product(
            qc_vals, ql_vals, qp_vals, h_vals, dt_vals):
        w = dict(BASE)
        w["Q_CONTOURING"] = qc
        w["Q_LAG"] = ql
        w["Q_PROGRESS"] = qp
        w["HORIZON"] = h
        w["DT"] = dt

        if is_valid_config(w):
            combos.append((f"QC={qc}+QL={ql}+QP={qp}+N={h}+dt={dt}", w))

    return combos


def gen_secondary_grid() -> list:
    """Phase 4: Secondary parameters — state reg., control effort, v_theta.

    Q_VY x Q_OMEGA x R_DELTA x W_DELTA_RATE x V_THETA_MAX
    """
    combos = []

    qvy_vals = PHASE4_VALUES["Q_VY"]
    qom_vals = PHASE4_VALUES["Q_OMEGA"]
    rd_vals = PHASE4_VALUES["R_DELTA"]
    wdr_vals = PHASE4_VALUES["W_DELTA_RATE"]
    vtm_vals = PHASE4_VALUES["V_THETA_MAX"]

    for qvy, qom, rd, wdr, vtm in itertools.product(
            qvy_vals, qom_vals, rd_vals, wdr_vals, vtm_vals):
        w = dict(BASE)
        w["Q_VY"] = qvy
        w["Q_OMEGA"] = qom
        w["R_DELTA"] = rd
        w["W_DELTA_RATE"] = wdr
        w["V_THETA_MAX"] = vtm
        combos.append((
            f"VY={qvy}+O={qom}+RD={rd}+WDR={wdr}+VT={vtm}", w))

    return combos


def gen_solver_grid() -> list:
    """Phase 5: Solver parameters — ADMM_RHO x ADMM_MAX_ITER x ADMM_TOL."""
    combos = []

    rho_vals = PHASE5_VALUES["ADMM_RHO"]
    mi_vals = PHASE5_VALUES["ADMM_MAX_ITER"]
    tol_vals = PHASE5_VALUES["ADMM_TOL"]

    for rho, mi, tol in itertools.product(rho_vals, mi_vals, tol_vals):
        w = dict(BASE)
        w["ADMM_RHO"] = rho
        w["ADMM_MAX_ITER"] = mi
        w["ADMM_TOL"] = tol
        combos.append((f"rho={rho}+mi={mi}+tol={tol}", w))

    return combos


def gen_fine_tuning(best_weights: dict) -> list:
    """Phase 6: Fine-tuning around best config — single & pairwise perturbations."""
    combos = []
    pct_range = (0.80, 0.85, 0.90, 0.92, 0.95, 0.97,
                 1.03, 1.05, 1.08, 1.10, 1.15, 1.20)
    skip = {"ADMM_MAX_ITER", "HORIZON"}

    # Single parameter perturbations
    for name, base_val in best_weights.items():
        if base_val == 0 or name in skip:
            continue
        for mult in pct_range:
            new_val = round(base_val * mult, 6)
            if name in INT_PARAMS:
                new_val = int(new_val)

            w = dict(best_weights)
            w[name] = new_val
            pct = int((mult - 1.0) * 100)
            sign = "+" if pct >= 0 else ""
            combos.append((f"FT:{name}{sign}{pct}%", w))

    # Pairwise perturbations of key params
    key_params = ["Q_CONTOURING", "Q_LAG", "Q_PROGRESS", "R_DELTA",
                  "V_THETA_MAX", "ADMM_RHO", "W_DELTA_RATE", "DT"]
    for w1, w2 in itertools.combinations(key_params, 2):
        v1 = best_weights.get(w1, 0)
        v2 = best_weights.get(w2, 0)
        if v1 == 0 or v2 == 0:
            continue
        for m1, m2 in [(0.9, 1.1), (1.1, 0.9), (0.9, 0.9), (1.1, 1.1),
                       (0.95, 1.05), (1.05, 0.95)]:
            w = dict(best_weights)
            w[w1] = round(v1 * m1, 6)
            w[w2] = round(v2 * m2, 6)
            if w1 in INT_PARAMS:
                w[w1] = int(w[w1])
            if w2 in INT_PARAMS:
                w[w2] = int(w[w2])
            p1 = f"+{int((m1-1)*100)}%" if m1 > 1 else f"{int((m1-1)*100)}%"
            p2 = f"+{int((m2-1)*100)}%" if m2 > 1 else f"{int((m2-1)*100)}%"
            combos.append((f"FT:{w1}{p1}+{w2}{p2}", w))

    return combos


def gen_random_neighbors(best_weights: dict, n: int, objective: str,
                         profile_override: str | None = None,
                         seed_offset: int = 0) -> list:
    """Generate random perturbations around best config."""
    combos = []
    rng = random.Random(SEED + seed_offset)
    profile_name = profile_override if profile_override else objective
    profile = RANDOM_PROFILES.get(profile_name, RANDOM_PROFILES["racer"])

    discrete = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_multipliers = profile.get("default_multipliers",
                                      [0.85, 0.95, 1.0, 1.1, 1.2])
    min_perturb, max_perturb = profile.get("num_perturb_range", (3, 6))

    tune_params = [k for k in best_weights.keys()
                   if k not in ("ADMM_MAX_ITER",) and best_weights[k] != 0]

    i = 0
    attempts = 0
    max_attempts = max(100, n * 20)

    while i < n and attempts < max_attempts:
        attempts += 1
        w = dict(best_weights)
        num_perturb = rng.randint(min_perturb,
                                  min(max_perturb, len(tune_params)))
        params_to_perturb = rng.sample(tune_params, num_perturb)

        for name in params_to_perturb:
            if name in discrete:
                w[name] = rng.choice(discrete[name])
            else:
                mult = rng.choice(param_multipliers.get(name,
                                                        default_multipliers))
                w[name] = round(w[name] * mult, 6)

            if name in INT_PARAMS:
                w[name] = int(float(w[name]))
                if name == "HORIZON":
                    w[name] = max(2, min(50, w[name]))

        if is_valid_config(w):
            combos.append((f"RND_{i}", w))
            i += 1

    return combos


# ==============================================================================
# DEDUPLICATION
# ==============================================================================

def deduplicate(combos: list) -> list:
    """Remove duplicate configurations."""
    seen = set()
    unique = []

    for label, params in combos:
        key = config_hash(params)
        if key not in seen:
            seen.add(key)
            unique.append((label, params))

    return unique


# ==============================================================================
# CSV WRITER
# ==============================================================================

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


# ==============================================================================
# PARALLEL WORKER
# ==============================================================================

def _run_single(args):
    """Worker: run one test and return scored result."""
    label, params, binary, phase_name, objective = args
    r = run_test(params, binary)
    r = apply_scores(r, objective)
    r["label"] = label
    r["phase"] = phase_name
    r["raceline"] = RACELINE_TAG
    r.update(canonicalize_params(params))
    return r


# ==============================================================================
# PHASE RUNNER
# ==============================================================================

def run_phase(phase_name: str, combos: list, binary: str, results: list,
              t0: float, num_workers: int, csv_writer, objective: str) -> tuple:
    """Run a sweep phase. Returns (passed, failed)."""
    combos = deduplicate(combos)

    if not combos:
        print(f"  ({phase_name}: empty, skipping)")
        return 0, 0

    total = len(combos)
    print(f"\n{'='*80}")
    print(f"{phase_name} — {total} configurations ({num_workers} workers)")
    print(f"{'='*80}")

    passed = failed = 0

    if num_workers <= 1:
        for i, (label, params) in enumerate(combos):
            elapsed = time.time() - t0
            rate = max(len(results), 1) / max(elapsed, 0.01)
            eta = (total - i - 1) / max(rate, 0.01)
            print(f"  [{i+1:4d}/{total}] {label:55s} ", end="", flush=True)

            r = run_test(params, binary)
            r = apply_scores(r, objective)
            r["label"] = label
            r["phase"] = phase_name
            r["raceline"] = RACELINE_TAG
            r.update(canonicalize_params(params))
            results.append(r)

            if csv_writer:
                csv_writer.write_row(r)

            if r["status"] != "OK":
                failed += 1
                print(f"FAIL  (ETA {eta:.0f}s)")
            elif not is_safe_result(r):
                failed += 1
                print(f"unsafe wc={r.get('wall_collisions', '?')}  "
                      f"(ETA {eta:.0f}s)")
            else:
                passed += 1
                print(f"sc={r['score']:7.2f}  avgv={r.get('avg_vel', 0.0):.2f}"
                      f"  ec={r['avg_contouring_err']:.3f}  (ETA {eta:.0f}s)")
    else:
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            it = ((label, params, binary, phase_name, objective)
                  for label, params in combos)
            max_in_flight = max(num_workers * 4, num_workers + 2)
            futures = set()

            for _ in range(min(total, max_in_flight)):
                try:
                    futures.add(executor.submit(_run_single, next(it)))
                except StopIteration:
                    break

            while futures:
                done, futures = wait(futures, return_when=FIRST_COMPLETED)
                for future in done:
                    try:
                        futures.add(executor.submit(_run_single, next(it)))
                    except StopIteration:
                        pass

                    done_count += 1
                    r = future.result()
                    results.append(r)

                    if csv_writer:
                        csv_writer.write_row(r)

                    elapsed = time.time() - t0
                    rate = max(done_count, 1) / max(elapsed, 0.01)
                    eta = (total - done_count) / max(rate, 0.01)

                    if r["status"] != "OK":
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] "
                              f"{r['label']:55s} FAIL  (ETA {eta:.0f}s)")
                    elif not is_safe_result(r):
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] "
                              f"{r['label']:55s} "
                              f"unsafe wc={r.get('wall_collisions', '?')}  "
                              f"(ETA {eta:.0f}s)")
                    else:
                        passed += 1
                        print(f"  [{done_count:4d}/{total}] "
                              f"{r['label']:55s} "
                              f"sc={r['score']:7.2f}  "
                              f"avgv={r.get('avg_vel', 0.0):.2f}  "
                              f"ec={r['avg_contouring_err']:.3f}  "
                              f"(ETA {eta:.0f}s)")

    return passed, failed


# ==============================================================================
# SANITY CHECK
# ==============================================================================

def sanity_check_params(binary: str):
    """Verify key swept parameters actually affect output."""
    print("\nRunning parameter effect sanity check...")

    baseline = run_test(dict(BASE), binary)
    baseline_sig = (
        baseline.get("status"),
        round(baseline.get("avg_contouring_err", 0.0), 4),
        round(baseline.get("avg_heading_err", 0.0), 4),
        round(baseline.get("max_vx", 0.0), 2),
    )

    probes = [
        ("Q_CONTOURING", BASE.get("Q_CONTOURING", 1000) * 1.5),
        ("Q_LAG", BASE.get("Q_LAG", 700) * 1.5),
        ("ADMM_RHO", BASE.get("ADMM_RHO", 17) * 1.5),
        ("HORIZON", min(50, int(BASE.get("HORIZON", 7) + 3))),
    ]

    ineffective = []
    for name, new_val in probes:
        p = dict(BASE)
        p[name] = new_val

        rr = run_test(p, binary)
        sig = (
            rr.get("status"),
            round(rr.get("avg_contouring_err", 0.0), 4),
            round(rr.get("avg_heading_err", 0.0), 4),
            round(rr.get("max_vx", 0.0), 2),
        )
        if sig == baseline_sig:
            ineffective.append(name)

    if ineffective:
        print(f"  WARNING: Parameters with no detected effect: "
              f"{', '.join(ineffective)}")
        print("           Check env-variable plumbing in MPCC code.")
    else:
        print("  All tested parameters show effect on output — OK")


# ==============================================================================
# RESULT HELPERS
# ==============================================================================

def get_top_n_params(results: list, n: int = CASCADE_TOP_N) -> list:
    """Return list of up to N best params dicts."""
    safe = [r for r in results if is_safe_result(r)]

    if not safe:
        unsafe = sorted(results, key=lambda x: (
            0 if x.get("status") == "OK" else 1,
            x.get("wall_collisions", 999),
            x.get("score", 99999)
        ))
        safe = unsafe[:n] if unsafe else []
        if safe:
            print("  WARNING: No safe candidates yet; "
                  "using least-bad configs for cascade.")
    else:
        safe.sort(key=lambda x: x.get("score", 999999.0))

    seen = set()
    unique = []
    for r in safe:
        key = config_hash({k: r.get(k, BASE[k]) for k in BASE.keys()})
        if key not in seen:
            seen.add(key)
            params = {k: r.get(k, BASE[k]) for k in BASE.keys()}
            unique.append((r, params))
        if len(unique) >= n:
            break

    if unique:
        for i, (r, _) in enumerate(unique):
            print(f"  Top-{i+1}: {r['label'][:50]} "
                  f"(score={r.get('score', 0.0):.2f}, "
                  f"avgv={r.get('avg_vel', 0):.2f})")

    return [p for _, p in unique]


def update_base(new_params: dict):
    """Update BASE dict with new values."""
    if new_params:
        for k in BASE:
            if k in new_params:
                BASE[k] = new_params[k]


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    global BASE, RACELINE_PATH, RACELINE_TAG

    # Parse arguments
    num_workers = multiprocessing.cpu_count()
    objective = "racer"
    raceline_override = None

    for i, arg in enumerate(sys.argv):
        if arg in ("--jobs", "-j") and i + 1 < len(sys.argv):
            try:
                num_workers = int(sys.argv[i + 1])
            except ValueError:
                print(f"WARNING: invalid --jobs value '{sys.argv[i + 1]}', "
                      "using CPU count")
                num_workers = multiprocessing.cpu_count()
            if num_workers <= 0:
                num_workers = multiprocessing.cpu_count()
        if arg == "--objective" and i + 1 < len(sys.argv):
            objective = sys.argv[i + 1].strip().lower()
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_override = sys.argv[i + 1].strip()

    if objective not in ("racer", "tracker"):
        print("ERROR: --objective must be 'racer' or 'tracker'")
        sys.exit(1)

    if raceline_override:
        RACELINE_PATH = resolve_raceline_path(raceline_override)
    else:
        RACELINE_PATH = os.path.abspath(RACELINE_PATH)
    RACELINE_TAG = infer_raceline_tag(RACELINE_PATH)

    # Initialize BASE config
    BASE.update(BASE_CONFIG)
    if objective == "racer":
        BASE.update(RACER_BASE_OVERRIDES)

    print(f"\n{'='*80}")
    print("MPCC Weight Tuning — Hardware Map")
    print(f"{'='*80}")
    print(f"  Workers:           {num_workers}")
    print(f"  Objective:         {objective}")
    print(f"  Cascade:           top {CASCADE_TOP_N}")
    print(f"  Global passes:     {GLOBAL_OPTIMIZATION_PASSES}")
    print(f"  Phase7 random:     {PHASE7_RANDOM_COUNT.get(objective, 3600)}")
    print(f"  Phase8 random:     {PHASE8_RANDOM_COUNT.get(objective, 1800)}")
    print(f"  Raceline:          {RACELINE_PATH}")
    print(f"  Raceline tag:      {RACELINE_TAG}")

    os.chdir(MPCC_DIR)

    # Build binary
    binary_name = f"test_sim_drive_{os.getpid()}_{int(time.time())}"
    binary = os.path.join(MPCC_DIR, binary_name)

    print("\nBuilding MPCC test binary...")
    ret = subprocess.run([
        "gcc",
        "-D_GNU_SOURCE", "-O3", "-std=c99", "-Wall", "-ffast-math",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Wno-unused-function", "-Wno-unknown-pragmas",
        f"-I{MPCC_DIR}/include",
        f"{MPCC_DIR}/test/test_sim_drive.c",
        f"{MPCC_DIR}/src/mpcc.c",
        f"{MPCC_DIR}/src/mpcc_vehicle_model.c",
        f"{MPCC_DIR}/src/qp_solver_mpcc.c",
        "-lm",
        "-o", binary,
    ], capture_output=True, text=True)

    if ret.returncode != 0:
        print(f"BUILD FAILED:\n{ret.stderr}")
        sys.exit(1)
    print("  Build OK")

    # Check raceline exists
    if not os.path.exists(RACELINE_PATH):
        print(f"ERROR: Raceline not found: {RACELINE_PATH}")
        sys.exit(1)

    # Sanity check
    sanity_check_params(binary)

    # Setup
    results = []
    t0 = time.time()
    total_p = total_f = 0

    # CSV writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = os.path.join(MPCC_DIR, "tuning_results")
    os.makedirs(results_dir, exist_ok=True)
    outfile = os.path.join(results_dir,
                           f"tuning_hardware_{objective}_{timestamp}.csv")
    fieldnames = (
        ["label", "phase", "raceline", "score", "tracker_score", "racer_score",
         "passed", "failed", "max_contouring_err", "avg_contouring_err",
         "max_heading_err", "avg_heading_err", "max_vx", "avg_vel",
         "max_vel_err", "avg_solve_us", "max_solve_us",
         "wall_collisions", "time_above_5ms", "avg_iters",
         "avg_rho", "avg_rho_u", "avg_adapt_updates", "avg_clip_events",
         "status", "return_code"]
        + list(BASE.keys())
    )
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results: {outfile}\n")

    # ========== PHASE 1: One-at-a-time ==========
    p, f = run_phase("Phase 1: One-at-a-time sensitivity",
                     gen_one_at_a_time(), binary, results, t0,
                     num_workers, csv_writer, objective)
    total_p += p
    total_f += f

    # ========== PHASE 2: Primary grid ==========
    combos = gen_primary_grid()
    print(f"\n  Phase 2 will test {len(combos):,} configurations")
    p, f = run_phase(
        "Phase 2: Primary grid "
        "(Q_CONTOURING x Q_LAG x Q_PROGRESS x HORIZON x DT)",
        combos, binary, results, t0, num_workers, csv_writer, objective)
    total_p += p
    total_f += f

    # Get top N for cascade
    print("\n  Selecting top configs for cascade...")
    top_configs = get_top_n_params(results)
    if not top_configs:
        top_configs = [dict(BASE)]

    # ========== PHASE 3: Skipped ==========
    print("\n  Phase 3: Skipped (no wall-margin concept in MPCC)")

    # ========== PHASE 4: Secondary grid (seed screening) ==========
    print(f"\n{'='*80}")
    print(f"Phase 4 seed screening from top {len(top_configs)} Phase-2 configs")
    print(f"{'='*80}")

    for ci, cascade_base in enumerate(top_configs):
        print(f"\n{'#'*80}")
        print(f"# PHASE 4 SEED {ci+1}/{len(top_configs)}")
        print(f"{'#'*80}")

        update_base(cascade_base)
        p, f = run_phase(
            f"Phase 4: Secondary grid [seed {ci+1}/{len(top_configs)}]",
            gen_secondary_grid(), binary, results, t0,
            num_workers, csv_writer, objective)
        total_p += p
        total_f += f

    # Promote one global best after seed screening
    best = get_top_n_params(results, n=1)
    if best:
        update_base(best[0])

    # ========== PHASES 5-8: Global optimization loop ==========
    for pi in range(GLOBAL_OPTIMIZATION_PASSES):
        print(f"\n{'#'*80}")
        print(f"# GLOBAL OPTIMIZATION PASS "
              f"{pi+1}/{GLOBAL_OPTIMIZATION_PASSES}")
        print(f"{'#'*80}")

        # Phase 5: solver parameter sweep
        p, f = run_phase(
            f"Phase 5: Solver parameters "
            f"[pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
            gen_solver_grid(), binary, results, t0,
            num_workers, csv_writer, objective)
        total_p += p
        total_f += f

        top = get_top_n_params(results, n=1)
        if top:
            update_base(top[0])

        # Phase 6: fine tuning around current global best
        best = get_top_n_params(results, n=1)
        if best:
            p, f = run_phase(
                f"Phase 6: Fine-tuning "
                f"[pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
                gen_fine_tuning(best[0]), binary, results, t0,
                num_workers, csv_writer, objective)
            total_p += p
            total_f += f

            top = get_top_n_params(results, n=1)
            if top:
                update_base(top[0])

        # Phase 7: random exploration around global best
        best = get_top_n_params(results, n=1)
        if best:
            n_random = PHASE7_RANDOM_COUNT.get(objective, 3600)
            p, f = run_phase(
                f"Phase 7: Random neighbors ({n_random}) "
                f"[pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
                gen_random_neighbors(best[0], n_random, objective,
                                     seed_offset=7000 + pi),
                binary, results, t0, num_workers, csv_writer, objective)
            total_p += p
            total_f += f

            top = get_top_n_params(results, n=1)
            if top:
                update_base(top[0])

        # Phase 8: random exploitation around updated global best
        best = get_top_n_params(results, n=1)
        if best:
            n_random = PHASE8_RANDOM_COUNT.get(objective, 1800)
            p, f = run_phase(
                f"Phase 8: Random exploitation ({n_random}) "
                f"[pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
                gen_random_neighbors(best[0], n_random, objective,
                                     profile_override=f"{objective}_exploit",
                                     seed_offset=9000 + pi),
                binary, results, t0, num_workers, csv_writer, objective)
            total_p += p
            total_f += f

            top = get_top_n_params(results, n=1)
            if top:
                update_base(top[0])

    # ========== FINAL RESULTS ==========
    results.sort(key=lambda x: x.get("score", 999999.0))
    elapsed = time.time() - t0

    print(f"\n{'='*80}")
    print(f"COMPLETED {len(results):,} tests in "
          f"{elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    print(f"{'='*80}")

    # Write sorted results
    sorted_file = outfile.replace(".csv", "_sorted.csv")
    if results:
        with open(sorted_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames,
                                    extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"Results: {sorted_file}")

    # Show top results
    safe = [r for r in results if is_safe_result(r)]
    if safe:
        print(f"\n{'='*80}")
        print(f"TOP 20 RESULTS ({objective} objective)")
        print(f"{'='*80}")

        fmt = ("{:<4} {:<45} {:>8} {:>6} {:>6} {:>6} {:>6} "
               "{:>6} {:>6} {:>3}")
        print(fmt.format("Rank", "Label", "Score", "AvgV", "MaxVx",
                          "AvgEc", "QC", "QL", "QP", "WC"))
        print("-" * 105)

        top = sorted(safe, key=lambda x: x.get("score", 999999.0))[:20]
        for i, r in enumerate(top):
            print(fmt.format(
                i+1,
                r['label'][:45],
                f"{r.get('score', 0.0):.2f}",
                f"{r.get('avg_vel', 0.0):.2f}",
                f"{r['max_vx']:.1f}",
                f"{r['avg_contouring_err']:.4f}",
                f"{r.get('Q_CONTOURING', '-')}",
                f"{r.get('Q_LAG', '-')}",
                f"{r.get('Q_PROGRESS', '-')}",
                f"{r.get('wall_collisions', '-')}",
            ))

        best = top[0]
        print(f"\nBEST CONFIGURATION:")
        print(f"  Score:        {best.get('score', 0.0):.2f}")
        print(f"  Avg velocity: {best.get('avg_vel', 0.0):.2f} m/s")
        print(f"  Max velocity: {best.get('max_vx', 0.0):.2f} m/s")
        print(f"  Avg contouring err: {best['avg_contouring_err']:.4f} m")
        print(f"  Walls:        {best['wall_collisions']}")
        print(f"  ---")
        for k in iter_ordered_base_keys():
            print(f"  {k:20s} = {best.get(k, BASE[k])}")

        # Print as env var command
        print(f"\n  Run with:")
        env_parts = []
        for k in iter_ordered_base_keys():
            v = best.get(k, BASE[k])
            env_parts.append(f"{k}={v}")
        print(f"    {' '.join(env_parts)} ./test_sim_drive")

    # Cleanup
    try:
        os.remove(binary)
    except OSError:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
