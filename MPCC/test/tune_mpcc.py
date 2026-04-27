#!/usr/bin/env python3
"""
MPCC Weight Tuning — Locked Horizon/DT
=======================================
Sweeps MPCC controller weights with HORIZON=20 and DT=0.05 locked.
Total prediction window: 20 × 0.05 = 1.00 s.

Objective: maximize average speed with ZERO wall collisions (hard constraint).
           Any collision → score = 1000 + collisions (never beats safe config).

Usage:
    python3 test/tune_mpcc.py                         # Full sweep (all CPUs)
    python3 test/tune_mpcc.py --jobs 8                # Use 8 parallel workers
    python3 test/tune_mpcc.py -j 4                    # Use 4 workers
    python3 test/tune_mpcc.py --objective racer       # Optimize for speed (default)
    python3 test/tune_mpcc.py --objective tracker     # Optimize for tracking
    python3 test/tune_mpcc.py --raceline my_track_raceline.csv
"""

import subprocess
import os
import sys
import csv
import json
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

# ==============================================================================
# LOCKED PREDICTION WINDOW  (never swept)
# ==============================================================================
LOCKED_HORIZON = 20
LOCKED_DT      = 0.05
# cross_call_rate_scale = control_dt / prediction_dt = (1/200 Hz) / 0.05 s
LOCKED_CROSS_CALL_SCALE = round((1.0 / 200.0) / LOCKED_DT, 6)  # = 0.1

BASE_CONFIG = {
    # Contouring tracking
    "Q_CONTOURING":      960.0,
    "Q_LAG":             100.0,
    "Q_PROGRESS":        15.6,

    # State regularization — increased for 12x stronger tires
    "Q_VX":              30.0,
    "VX_REF":            4.0,
    "Q_VY":              0.5,
    "Q_OMEGA":           3.0,

    # Control effort — R_DELTA raised for ADMM convergence on tight curves
    "R_DELTA":           100.0,
    "R_AX":              0.05225,
    "R_VTHETA":          0.1,

    # Control rate smoothness — delta rate raised for stability
    "W_DELTA_RATE":      2.0,
    "W_AX_RATE":         0.488,
    "W_VTHETA_RATE":     0.1105,

    # Terminal weights — MUST be >= running weights
    "Q_CONTOURING_TERM": 4800.0,
    "Q_LAG_TERM":        800.0,
    "Q_PROGRESS_TERM":   41.4,

    # ADMM solver — more iterations for harder problem
    "ADMM_RHO":          5.0,
    "ADMM_MAX_ITER":     300,
    "ADMM_TOL":          0.02,

    # V_THETA_MAX >= vx_max so reference keeps up with vehicle
    "V_THETA_MAX":       15.0,

    # LOCKED — not swept
    "HORIZON":           LOCKED_HORIZON,
    "DT":                LOCKED_DT,
    "CROSS_CALL_SCALE":  LOCKED_CROSS_CALL_SCALE,
}

RACER_BASE_OVERRIDES = {
    "Q_CONTOURING":      960.0,
    "Q_LAG":             100.0,
    "Q_PROGRESS":        15.6,
    "Q_VY":              0.5,
    "Q_OMEGA":           3.0,
    "Q_VX":              30.0,
    "VX_REF":            4.0,
    "R_DELTA":           100.0,
    "R_AX":              0.05225,
    "R_VTHETA":          0.1,
    "W_DELTA_RATE":      2.0,
    "W_AX_RATE":         0.488,
    "W_VTHETA_RATE":     0.1105,
    "Q_CONTOURING_TERM": 4800.0,
    "Q_LAG_TERM":        800.0,
    "Q_PROGRESS_TERM":   41.4,
    "ADMM_RHO":          5.0,
    "ADMM_MAX_ITER":     300,
    "ADMM_TOL":          0.02,
    "V_THETA_MAX":       15.0,
}

# ==============================================================================
# SWEEP VALUE RANGES
# ==============================================================================

PHASE2_VALUES = {
    "Q_CONTOURING":      [300, 500, 800, 1000],
    "Q_LAG":             [50, 100, 150, 200, 300],
    "Q_PROGRESS":        [8, 10, 12, 15, 18, 20],
    "Q_CONTOURING_TERM": [1000, 2000, 3000, 4000],
    "Q_LAG_TERM":        [400, 800, 1500],
}

FULL_SWEEP_VALUES = {
    "Q_CONTOURING":      [300, 500, 800, 1000, 1500],
    "Q_LAG":             [50, 100, 150, 200, 300, 500],
    "Q_PROGRESS":        [8, 10, 12, 15, 18, 20, 25],
    "Q_VY":              [0.5, 1.0, 1.5, 3.0, 5.0, 10.0],
    "Q_OMEGA":           [0.3, 0.5, 0.8, 1.5, 3.0, 5.0],
    "R_DELTA":           [100.0, 130.0, 160.0, 200.0],
    "R_VTHETA":          [0.05, 0.1, 0.2, 0.3],
    "W_DELTA_RATE":      [2.0, 3.0, 4.0, 5.0],
    "W_VTHETA_RATE":     [0.05, 0.1, 0.13, 0.3, 0.5, 1.0],
    "Q_CONTOURING_TERM": [1000, 2000, 3000, 4000, 8000],
    "Q_LAG_TERM":        [400, 800, 1500, 3000],
    "Q_PROGRESS_TERM":   [20, 30, 40, 50, 60],
    "V_THETA_MAX":       [8.0, 10.0, 12.0, 15.0],
    "Q_VX":              [0.0, 0.5, 1.0, 1.5, 3.0, 5.0],
    "VX_REF":            [2.0, 3.0, 4.0, 5.0, 6.0, 8.0],
    "R_AX":              [0.02, 0.055, 0.1, 0.3, 1.0],
    "W_AX_RATE":         [0.1, 0.3, 0.61, 1.0, 2.0],
}

PHASE4_VALUES = {
    "Q_VY":         [0.5, 1.0, 1.54, 3.0, 5.0],
    "Q_OMEGA":      [0.3, 0.5, 0.8, 1.5, 3.0],
    "R_DELTA":      [100.0, 130.0, 160.0, 200.0],
    "W_DELTA_RATE": [2.0, 3.0, 4.0, 5.0],
    "V_THETA_MAX":  [8.0, 10.0, 12.0, 15.0],
}

# PHASE5_VALUES removed — was ADMM-only tuning, unused with OSQP solver
PHASE5_VALUES = {}

RANDOM_PROFILES = {
    "racer": {
        "num_perturb_range": (3, 7),
        "default_multipliers": [0.85, 0.95, 1.0, 1.1, 1.2],
        "param_multipliers": {
            "Q_CONTOURING":      [0.7, 0.85, 1.0, 1.2, 1.5],
            "Q_LAG":             [0.7, 0.85, 1.0, 1.2, 1.5],
            "Q_PROGRESS":        [0.8, 0.9, 1.0, 1.15, 1.3, 1.5],
            "Q_CONTOURING_TERM": [1.0, 1.1, 1.2, 1.5, 2.0],
            "Q_LAG_TERM":        [1.0, 1.1, 1.2, 1.5, 2.0],
            "Q_PROGRESS_TERM":   [0.8, 0.9, 1.0, 1.15, 1.3],
            "Q_VX":              [0.0, 0.5, 1.0, 1.5, 2.0, 3.0],
            "Q_VY":              [0.7, 0.9, 1.0, 1.15, 1.3],
            "Q_OMEGA":           [0.7, 0.9, 1.0, 1.15, 1.3],
            "R_DELTA":           [0.8, 0.9, 1.0, 1.1, 1.25],
            "R_AX":              [0.5, 0.8, 1.0, 1.3, 2.0],
            "R_VTHETA":          [0.5, 0.8, 1.0, 1.3, 2.0],
            "W_DELTA_RATE":      [0.8, 0.9, 1.0, 1.15, 1.3],
            "W_AX_RATE":         [0.5, 0.8, 1.0, 1.3, 2.0],
            "W_VTHETA_RATE":     [0.7, 0.85, 1.0, 1.2, 1.4],
            # ADMM_RHO removed — unused with OSQP solver
            "V_THETA_MAX":       [0.8, 0.9, 1.0, 1.1, 1.2],
        },
        "discrete": {
            # HORIZON and DT intentionally omitted — locked
            # ADMM_MAX_ITER removed — unused with OSQP solver
        },
    },
    "tracker": {
        "num_perturb_range": (3, 6),
        "default_multipliers": [0.85, 0.95, 1.0, 1.1, 1.2],
        "param_multipliers": {
            "Q_CONTOURING":      [0.9, 0.97, 1.0, 1.08, 1.18],
            "Q_LAG":             [0.9, 0.97, 1.0, 1.08, 1.18],
            "Q_PROGRESS":        [0.85, 0.95, 1.0, 1.1, 1.2],
            "Q_CONTOURING_TERM": [1.0, 1.1, 1.2, 1.5, 2.0],
            "Q_LAG_TERM":        [1.0, 1.1, 1.2, 1.5, 2.0],
            "Q_PROGRESS_TERM":   [0.8, 0.9, 1.0, 1.15, 1.3],
            "Q_VX":              [0.0, 0.5, 1.0, 1.3, 2.0],
            "Q_VY":              [0.8, 0.9, 1.0, 1.15, 1.3],
            "Q_OMEGA":           [0.8, 0.9, 1.0, 1.15, 1.3],
            "R_DELTA":           [0.85, 0.95, 1.0, 1.1, 1.2],
            "R_AX":              [0.5, 0.8, 1.0, 1.3, 2.0],
            "R_VTHETA":          [0.5, 0.8, 1.0, 1.3, 2.0],
            "W_DELTA_RATE":      [0.85, 0.95, 1.0, 1.1, 1.2],
            "W_AX_RATE":         [0.5, 0.8, 1.0, 1.3, 2.0],
            "W_VTHETA_RATE":     [0.7, 0.85, 1.0, 1.2, 1.4],
            # ADMM_RHO removed — unused with OSQP solver
            "V_THETA_MAX":       [0.85, 0.95, 1.0, 1.1, 1.2],
        },
        "discrete": {
            # HORIZON and DT intentionally omitted — locked
            # ADMM_MAX_ITER removed — unused with OSQP solver
        },
    },
    "racer_exploit": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.96, 0.99, 1.0, 1.03, 1.07],
        "param_multipliers": {
            "Q_CONTOURING":      [0.94, 0.98, 1.0, 1.03, 1.07],
            "Q_LAG":             [0.94, 0.98, 1.0, 1.03, 1.07],
            "Q_PROGRESS":        [0.97, 1.0, 1.03, 1.06, 1.1],
            "Q_CONTOURING_TERM": [1.0, 1.02, 1.05, 1.1, 1.15],
            "Q_LAG_TERM":        [1.0, 1.02, 1.05, 1.1, 1.15],
            "Q_PROGRESS_TERM":   [0.95, 0.98, 1.0, 1.03, 1.06],
            "Q_VX":              [0.0, 0.7, 1.0, 1.2, 1.5],
            "Q_VY":              [0.92, 0.98, 1.0, 1.05, 1.1],
            "Q_OMEGA":           [0.92, 0.98, 1.0, 1.05, 1.1],
            "R_DELTA":           [0.94, 0.99, 1.0, 1.04, 1.08],
            "R_AX":              [0.7, 0.9, 1.0, 1.2, 1.5],
            "R_VTHETA":          [0.7, 0.9, 1.0, 1.2, 1.5],
            "W_DELTA_RATE":      [0.92, 0.98, 1.0, 1.05, 1.1],
            "W_AX_RATE":         [0.7, 0.9, 1.0, 1.2, 1.5],
            "W_VTHETA_RATE":     [0.9, 0.97, 1.0, 1.05, 1.1],
            # ADMM_RHO removed — unused with OSQP solver
            "V_THETA_MAX":       [0.94, 0.99, 1.0, 1.04, 1.08],
        },
        "discrete": {
            # HORIZON and DT intentionally omitted — locked
            # ADMM_MAX_ITER removed — unused with OSQP solver
        },
    },
    "tracker_exploit": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.95, 0.98, 1.0, 1.02, 1.05],
        "param_multipliers": {
            "Q_CONTOURING":      [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_LAG":             [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_PROGRESS":        [0.95, 0.98, 1.0, 1.03, 1.06],
            "Q_CONTOURING_TERM": [1.0, 1.02, 1.05, 1.1, 1.15],
            "Q_LAG_TERM":        [1.0, 1.02, 1.05, 1.1, 1.15],
            "Q_PROGRESS_TERM":   [0.92, 0.97, 1.0, 1.05, 1.1],
            "Q_VX":              [0.0, 0.8, 1.0, 1.15, 1.3],
            "Q_VY":              [0.9, 0.97, 1.0, 1.05, 1.1],
            "Q_OMEGA":           [0.9, 0.97, 1.0, 1.05, 1.1],
            "R_DELTA":           [0.92, 0.98, 1.0, 1.05, 1.1],
            "R_AX":              [0.8, 0.95, 1.0, 1.1, 1.3],
            "R_VTHETA":          [0.8, 0.95, 1.0, 1.1, 1.3],
            "W_DELTA_RATE":      [0.92, 0.98, 1.0, 1.05, 1.1],
            "W_AX_RATE":         [0.8, 0.95, 1.0, 1.1, 1.3],
            "W_VTHETA_RATE":     [0.9, 0.97, 1.0, 1.05, 1.1],
            # ADMM_RHO removed — unused with OSQP solver
            "V_THETA_MAX":       [0.92, 0.98, 1.0, 1.05, 1.1],
        },
        "discrete": {
            # HORIZON and DT intentionally omitted — locked
            # ADMM_MAX_ITER removed — unused with OSQP solver
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

MPCC_PRINT_ORDER = (
    "Q_CONTOURING", "Q_LAG", "Q_PROGRESS",
    "Q_VY", "Q_OMEGA",
    "Q_VX", "VX_REF",
    "R_DELTA", "R_AX", "R_VTHETA",
    "W_DELTA_RATE", "W_AX_RATE", "W_VTHETA_RATE",
    "Q_CONTOURING_TERM", "Q_LAG_TERM", "Q_PROGRESS_TERM",
    "ADMM_RHO", "ADMM_MAX_ITER", "ADMM_TOL",
    "HORIZON", "DT", "V_THETA_MAX", "CROSS_CALL_SCALE",
)

BASE = {}


def infer_raceline_tag(path: str) -> str:
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem.endswith("_raceline"):
        stem = stem[: -len("_raceline")]
    return stem or "unknown"


def resolve_raceline_path(path_arg: str) -> str:
    if os.path.isabs(path_arg):
        return path_arg
    traj_candidate = os.path.join(TRAJ_DIR, path_arg)
    if os.path.exists(traj_candidate):
        return os.path.abspath(traj_candidate)
    return os.path.abspath(os.path.join(MPCC_DIR, path_arg))


def iter_ordered_base_keys():
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
    out = dict(params)
    for k in INT_PARAMS:
        if k in out:
            out[k] = int(float(out[k]))
    return out


def enforce_terminal_weight_floor(params: dict) -> dict:
    """
    Enforce terminal >= running weights (required for Riccati correctness)
    and V_THETA_MAX >= 8 m/s (required for reference to keep up with car).
    """
    out = dict(params)
    qc = float(out.get("Q_CONTOURING", BASE.get("Q_CONTOURING", 1000)))
    ql = float(out.get("Q_LAG",        BASE.get("Q_LAG",        300)))
    qp = float(out.get("Q_PROGRESS",   BASE.get("Q_PROGRESS",   20)))

    if float(out.get("Q_CONTOURING_TERM", 0)) < qc:
        out["Q_CONTOURING_TERM"] = qc * 2.0
    if float(out.get("Q_LAG_TERM", 0)) < ql:
        out["Q_LAG_TERM"] = ql
    if float(out.get("Q_PROGRESS_TERM", 0)) < qp * 0.5:
        out["Q_PROGRESS_TERM"] = qp
    if float(out.get("V_THETA_MAX", BASE.get("V_THETA_MAX", 20.0))) < 8.0:
        out["V_THETA_MAX"] = 8.0

    return out


def is_valid_config(params: dict) -> bool:
    h = int(params.get("HORIZON", BASE.get("HORIZON", 10)))
    if h < 2 or h > 50:
        return False
    dt = float(params.get("DT", BASE.get("DT", 0.02)))
    if dt < 0.01 or dt > 0.2:
        return False
    qc = float(params.get("Q_CONTOURING", BASE.get("Q_CONTOURING", 1000)))
    ql = float(params.get("Q_LAG", BASE.get("Q_LAG", 300)))
    if ql > 0 and (qc / ql) > 10.0:
        return False
    return True


def config_hash(params: dict) -> str:
    eff = canonicalize_params(params)
    key = tuple(sorted((k, round(v, 4) if isinstance(v, float) else v)
                       for k, v in eff.items()))
    return hashlib.md5(str(key).encode()).hexdigest()[:12]


# ==============================================================================
# TEST RUNNER
# ==============================================================================

def run_test(params: dict, binary: str) -> dict:
    env = os.environ.copy()
    env["MPCC_TUNING_CSV"] = "1"
    env["RACELINE_PATH"] = RACELINE_PATH

    effective_params = canonicalize_params(enforce_terminal_weight_floor(params))
    # Always enforce locked prediction window
    effective_params["HORIZON"]         = LOCKED_HORIZON
    effective_params["DT"]              = LOCKED_DT
    effective_params["CROSS_CALL_SCALE"] = LOCKED_CROSS_CALL_SCALE
    for name, value in effective_params.items():
        env[name] = str(value)

    try:
        result = subprocess.run(
            [binary], capture_output=True, text=True, timeout=120, env=env
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
                    "admm_max_iter":     float(effective_params.get("ADMM_MAX_ITER", 300)),
                    "passed": int(parts[1]),
                    "failed": int(parts[2]),
                    "max_contouring_err": float(parts[3]),
                    "avg_contouring_err": float(parts[4]),
                    "max_heading_err":    float(parts[5]),
                    "avg_heading_err":    float(parts[6]),
                    "max_vx":             float(parts[7]),
                    "avg_solve_us":       float(parts[8]),
                    "max_solve_us":       float(parts[9]),
                    "wall_collisions":    int(parts[10]),
                    "time_above_5ms":     float(parts[11]),
                    "max_vel_err":        float(parts[12]) if len(parts) > 12 else 0.0,
                    "avg_vel_err":        float(parts[13]) if len(parts) > 13 else 0.0,
                    "avg_iters":          float(parts[14]) if len(parts) > 14 else 0.0,
                    "avg_rho":            float(parts[15]) if len(parts) > 15 else 0.0,
                    "avg_rho_u":          float(parts[16]) if len(parts) > 16 else 0.0,
                    "avg_adapt_updates":  float(parts[17]) if len(parts) > 17 else 0.0,
                    "avg_clip_events":    float(parts[18]) if len(parts) > 18 else 0.0,
                    "avg_speed":          float(parts[19]) if len(parts) > 19 else 0.0,
                    "lap_count":          int(parts[20])   if len(parts) > 20 else 0,
                    "best_lap_time":      float(parts[21]) if len(parts) > 21 else 9999.0,
                    "s_backward_jumps":   int(parts[22])   if len(parts) > 22 else 0,
                    "s_large_corrections": int(parts[23])  if len(parts) > 23 else 0,
                    "max_s_jump":         float(parts[24]) if len(parts) > 24 else 0.0,
                    "s_prediction_regressions": int(parts[25]) if len(parts) > 25 else 0,
                    "max_predicted_s_span": float(parts[26]) if len(parts) > 26 else 0.0,
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
    """Zero collisions and valid run."""
    return r.get("status") == "OK" and int(r.get("wall_collisions", 999)) == 0


def compute_stability_penalty(r: dict) -> float | None:
    """Return a hard ranking penalty for unstable-but-lucky runs."""
    s_backward_jumps = int(r.get("s_backward_jumps", 0))
    s_prediction_regressions = int(r.get("s_prediction_regressions", 0))
    s_large_corrections = int(r.get("s_large_corrections", 0))
    avg_iters = float(r.get("avg_iters", 0.0))
    admm_max_iter = max(float(r.get("admm_max_iter", 300.0)), 1.0)
    avg_clip_events = float(r.get("avg_clip_events", 0.0))

    if s_backward_jumps > 0 or s_prediction_regressions > 0:
        severity = (2.0 * s_backward_jumps) + (5.0 * s_prediction_regressions) + (0.5 * s_large_corrections)
        return 950.0 + min(40.0, severity)

    if avg_clip_events > 0.05:
        return 940.0 + min(40.0, avg_clip_events * 20.0)

    if avg_iters > 0.8 * admm_max_iter:
        return 930.0 + min(50.0, 50.0 * (avg_iters / admm_max_iter))

    return None


def compute_tracker_score(r: dict) -> float:
    """
    Tracker: minimize contouring + heading errors.
    Any collision = hard penalty of 1000 + collisions (never beats safe config).
    """
    if r.get("status") != "OK":
        return 5000.0

    collisions = int(r.get("wall_collisions", 0))
    if collisions > 0:
        # Hard constraint: unsafe configs are always ranked last
        return 1000.0 + float(collisions)

    stability_penalty = compute_stability_penalty(r)
    if stability_penalty is not None:
        return stability_penalty

    tracking = (
        r["avg_contouring_err"] * 80.0 +
        r["max_contouring_err"] * 15.0 +
        r["avg_heading_err"]    * 35.0 +
        r["max_heading_err"]    *  8.0 +
        r.get("max_vel_err", 0) *  4.0
    )
    solver = r.get("avg_iters", 0) * 0.2 + r["avg_solve_us"] * 0.001
    return round(tracking + solver, 3)


def compute_racer_score(r: dict) -> float:
    """
    Racer: minimize best lap time — zero collisions is a HARD constraint.

    Scoring:
      - Any collision      → 1000 + collisions  (always loses to any safe config)
      - No laps completed  → 999                (worse than any config that completes a lap)
      - Zero collisions    → best_lap_time       (lower = faster = better)

    This means a safe config doing a slow lap beats an unsafe config.
    The sweep will only optimise lap time among collision-free configs.
    """
    if r.get("status") != "OK":
        return 5000.0

    collisions = int(r.get("wall_collisions", 0))
    if collisions > 0:
        # Hard constraint: ANY collision → disqualified
        return 1000.0 + float(collisions)

    lap_count = int(r.get("lap_count", 0))
    if lap_count == 0:
        # No laps completed — worse than any config that finishes a lap
        return 999.0

    stability_penalty = compute_stability_penalty(r)
    if stability_penalty is not None:
        return stability_penalty

    best_lap = float(r.get("best_lap_time", 9999.0))
    return round(best_lap, 6)


def apply_scores(r: dict, objective: str) -> dict:
    r["tracker_score"] = compute_tracker_score(r)
    r["racer_score"]   = compute_racer_score(r)
    r["score"] = r["tracker_score"] if objective == "tracker" else r["racer_score"]
    return r


# ==============================================================================
# CONFIG GENERATORS
# ==============================================================================

def gen_one_at_a_time() -> list:
    combos = [("BASELINE", enforce_terminal_weight_floor(dict(BASE)))]
    for name, values in FULL_SWEEP_VALUES.items():
        for v in values:
            if abs(v - BASE.get(name, -999)) < 1e-6:
                continue
            w = dict(BASE)
            w[name] = v
            w = enforce_terminal_weight_floor(w)
            if is_valid_config(w):
                combos.append((f"{name}={v}", w))
    return combos


def gen_primary_grid() -> list:
    combos = []
    for qc, ql, qp, qct, qlt in itertools.product(
            PHASE2_VALUES["Q_CONTOURING"],
            PHASE2_VALUES["Q_LAG"],
            PHASE2_VALUES["Q_PROGRESS"],
            PHASE2_VALUES["Q_CONTOURING_TERM"],
            PHASE2_VALUES["Q_LAG_TERM"]):

        if qct < qc or qlt < ql:
            continue
        if ql > 0 and (qc / ql) > 8.0:
            continue

        w = dict(BASE)
        w["Q_CONTOURING"]      = qc
        w["Q_LAG"]             = ql
        w["Q_PROGRESS"]        = qp
        w["Q_CONTOURING_TERM"] = qct
        w["Q_LAG_TERM"]        = qlt

        if is_valid_config(w):
            label = (f"QC={qc}+QL={ql}+QP={qp}"
                     f"+QCT={qct}+QLT={qlt}")
            combos.append((label, w))
    return combos


def gen_secondary_grid() -> list:
    combos = []
    for qvy, qom, rd, wdr, vtm in itertools.product(
            PHASE4_VALUES["Q_VY"],
            PHASE4_VALUES["Q_OMEGA"],
            PHASE4_VALUES["R_DELTA"],
            PHASE4_VALUES["W_DELTA_RATE"],
            PHASE4_VALUES["V_THETA_MAX"]):
        w = dict(BASE)
        w["Q_VY"]         = qvy
        w["Q_OMEGA"]      = qom
        w["R_DELTA"]      = rd
        w["W_DELTA_RATE"] = wdr
        w["V_THETA_MAX"]  = vtm
        w = enforce_terminal_weight_floor(w)
        combos.append((f"VY={qvy}+O={qom}+RD={rd}+WDR={wdr}+VT={vtm}", w))
    return combos


def gen_solver_grid() -> list:
    # Solver grid removed — ADMM params unused with OSQP solver
    return []


def gen_fine_tuning(best_weights: dict) -> list:
    combos = []
    pct_range = (0.80, 0.85, 0.90, 0.92, 0.95, 0.97,
                 1.03, 1.05, 1.08, 1.10, 1.15, 1.20)
    skip = {"ADMM_MAX_ITER", "ADMM_RHO", "ADMM_TOL",
            "HORIZON", "DT", "CROSS_CALL_SCALE"}

    for name, base_val in best_weights.items():
        if name in skip:
            continue
        if base_val == 0:
            if name in FULL_SWEEP_VALUES:
                for sv in FULL_SWEEP_VALUES[name]:
                    if sv == 0:
                        continue
                    w = enforce_terminal_weight_floor(dict(best_weights))
                    w[name] = sv
                    combos.append((f"FT:{name}={sv}", w))
            continue
        for mult in pct_range:
            new_val = round(base_val * mult, 6)
            if name in INT_PARAMS:
                new_val = int(new_val)
            w = dict(best_weights)
            w[name] = new_val
            w = enforce_terminal_weight_floor(w)
            if is_valid_config(w):
                pct = int((mult - 1.0) * 100)
                sign = "+" if pct >= 0 else ""
                combos.append((f"FT:{name}{sign}{pct}%", w))

    key_params = [
        "Q_CONTOURING", "Q_LAG", "Q_PROGRESS",
        "Q_CONTOURING_TERM", "Q_LAG_TERM",
        "Q_VX", "R_DELTA", "R_AX", "R_VTHETA",
        "V_THETA_MAX", "W_DELTA_RATE",
    ]
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
            w = enforce_terminal_weight_floor(w)
            if is_valid_config(w):
                p1 = f"+{int((m1-1)*100)}%" if m1 > 1 else f"{int((m1-1)*100)}%"
                p2 = f"+{int((m2-1)*100)}%" if m2 > 1 else f"{int((m2-1)*100)}%"
                combos.append((f"FT:{w1}{p1}+{w2}{p2}", w))

    return combos


def gen_random_neighbors(best_weights: dict, n: int, objective: str,
                         profile_override: str | None = None,
                         seed_offset: int = 0) -> list:
    combos = []
    rng = random.Random(SEED + seed_offset)
    profile_name = profile_override if profile_override else objective
    profile = RANDOM_PROFILES.get(profile_name, RANDOM_PROFILES["racer"])

    discrete          = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_mults     = profile.get("default_multipliers", [0.85, 0.95, 1.0, 1.1, 1.2])
    min_p, max_p      = profile.get("num_perturb_range", (3, 6))

    tune_params = [k for k in best_weights.keys()
                   if k not in ("ADMM_MAX_ITER", "ADMM_RHO", "ADMM_TOL",
                                "HORIZON", "DT", "CROSS_CALL_SCALE")]

    i = 0
    attempts = 0
    max_attempts = max(100, n * 20)

    while i < n and attempts < max_attempts:
        attempts += 1
        w = dict(best_weights)
        num_perturb = rng.randint(min_p, min(max_p, len(tune_params)))
        for name in rng.sample(tune_params, num_perturb):
            if name in discrete:
                w[name] = rng.choice(discrete[name])
            elif w[name] == 0 and name in FULL_SWEEP_VALUES:
                w[name] = rng.choice(FULL_SWEEP_VALUES[name])
            else:
                mult = rng.choice(param_multipliers.get(name, default_mults))
                w[name] = round(w[name] * mult, 6)
            if name in INT_PARAMS:
                w[name] = int(float(w[name]))

        # Always re-lock prediction window
        w["HORIZON"]         = LOCKED_HORIZON
        w["DT"]              = LOCKED_DT
        w["CROSS_CALL_SCALE"] = LOCKED_CROSS_CALL_SCALE

        w = enforce_terminal_weight_floor(w)
        if is_valid_config(w):
            combos.append((f"RND_{i}", w))
            i += 1

    return combos


# ==============================================================================
# DEDUPLICATION
# ==============================================================================

def deduplicate(combos: list) -> list:
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
    label, params, binary, phase_name, objective = args
    r = run_test(params, binary)
    r = apply_scores(r, objective)
    r["label"]    = label
    r["phase"]    = phase_name
    r["raceline"] = RACELINE_TAG
    r.update(canonicalize_params(params))
    return r


# ==============================================================================
# PHASE RUNNER
# ==============================================================================

def run_phase(phase_name: str, combos: list, binary: str, results: list,
              t0: float, num_workers: int, csv_writer, objective: str) -> tuple:
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
            r["label"]    = label
            r["phase"]    = phase_name
            r["raceline"] = RACELINE_TAG
            r.update(canonicalize_params(params))
            results.append(r)
            if csv_writer:
                csv_writer.write_row(r)

            wc = int(r.get("wall_collisions", 0))
            if r["status"] != "OK":
                failed += 1
                print(f"FAIL  (ETA {eta:.0f}s)")
            elif wc > 0:
                failed += 1
                print(f"COLLISION wc={wc} speed={r.get('avg_speed',0):.2f}  (ETA {eta:.0f}s)")
            else:
                passed += 1
                print(f"sc={r['score']:7.4f}  lap={r.get('best_lap_time',0):.3f}s"
                      f"  speed={r.get('avg_speed',0):.3f} m/s"
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
                    wc = int(r.get("wall_collisions", 0))

                    if r["status"] != "OK":
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] "
                              f"{r['label']:55s} FAIL  (ETA {eta:.0f}s)")
                    elif wc > 0:
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] "
                              f"{r['label']:55s} "
                              f"COLLISION wc={wc} speed={r.get('avg_speed',0):.2f}"
                              f"  (ETA {eta:.0f}s)")
                    else:
                        passed += 1
                        print(f"  [{done_count:4d}/{total}] "
                              f"{r['label']:55s} "
                              f"sc={r['score']:7.4f}  "
                              f"lap={r.get('best_lap_time',0):.3f}s  "
                              f"speed={r.get('avg_speed',0):.3f} m/s  "
                              f"ec={r['avg_contouring_err']:.3f}  "
                              f"(ETA {eta:.0f}s)")

    return passed, failed


# ==============================================================================
# RESULT HELPERS
# ==============================================================================

def get_top_n_params(results: list, n: int = CASCADE_TOP_N) -> list:
    """Return list of up to N best params dicts from collision-free runs only."""
    safe = [r for r in results if is_safe_result(r)]
    safe.sort(key=lambda r: r.get("score", 9999))
    seen = set()
    top = []
    for r in safe:
        params = {k: r[k] for k in MPCC_PRINT_ORDER if k in r}
        h = config_hash(params)
        if h not in seen:
            seen.add(h)
            top.append(params)
        if len(top) >= n:
            break
    return top


def get_best_params(results: list) -> dict:
    """Return params dict of best collision-free result."""
    top = get_top_n_params(results, n=1)
    if top:
        return top[0]
    # Fallback: if nothing is safe, return baseline
    return dict(BASE)


def print_best(results: list, objective: str, label: str = ""):
    safe = [r for r in results if is_safe_result(r)]
    if not safe:
        print(f"  [{label}] No collision-free results yet.")
        return
    safe.sort(key=lambda r: r.get("score", 9999))
    b = safe[0]
    print(f"\n  {'='*60}")
    print(f"  Best {label} (collision-free only):")
    print(f"    score={b['score']:.4f}  lap_time={b.get('best_lap_time',0):.3f}s"
          f"  laps={b.get('lap_count',0)}  speed={b.get('avg_speed',0):.3f} m/s"
          f"  ec={b.get('avg_contouring_err',0):.3f}  iters={b.get('avg_iters',0):.1f}")
    print(f"    label={b.get('label','?')}  phase={b.get('phase','?')}")
    for k in iter_ordered_base_keys():
        if k in b:
            print(f"    {k}={b[k]}")
    print(f"  {'='*60}\n")


# ==============================================================================
# SANITY CHECK
# ==============================================================================

def sanity_check_params(binary: str):
    print("\nRunning parameter effect sanity check...")
    print("  [1/5] Running baseline...", end=" ", flush=True)
    baseline = run_test(dict(BASE), binary)
    print(f"status={baseline.get('status')}  "
          f"speed={baseline.get('avg_speed',0):.3f}  "
          f"wc={baseline.get('wall_collisions','?')}")

    probes = [
        ("Q_CONTOURING", BASE.get("Q_CONTOURING", 1000) * 1.5),
        ("Q_LAG",        BASE.get("Q_LAG", 300) * 1.5),
        ("W_DELTA_RATE", BASE.get("W_DELTA_RATE", 3.0) * 1.5),
        ("R_DELTA",      BASE.get("R_DELTA", 130.0) * 1.5),
    ]
    for i, (name, new_val) in enumerate(probes, 2):
        print(f"  [{i}/5] Probing {name}={new_val}...", end=" ", flush=True)
        p = dict(BASE)
        p[name] = new_val
        rr = run_test(p, binary)
        print(f"status={rr.get('status')}  "
              f"speed={rr.get('avg_speed',0):.3f}  "
              f"wc={rr.get('wall_collisions','?')}")
    print("  Sanity check done.")


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    import argparse
    global BASE, RACELINE_PATH, RACELINE_TAG

    parser = argparse.ArgumentParser(
        description="MPCC Weight Tuner — locked HORIZON=20, DT=0.03")
    parser.add_argument("--jobs", "-j", type=int,
                        default=multiprocessing.cpu_count(),
                        help="Number of parallel workers")
    parser.add_argument("--objective", choices=["racer", "tracker"],
                        default="racer",
                        help="Optimization objective")
    parser.add_argument("--raceline", default=DEFAULT_RACELINE_NAME,
                        help="Raceline CSV filename or path")
    parser.add_argument("--binary", default=None,
                        help="Path to test_sim_drive binary")
    parser.add_argument("--sanity-only", action="store_true",
                        help="Only run sanity check, then exit")
    parser.add_argument("--phase1-only", action="store_true",
                        help="Only run Phase 1 (one-at-a-time)")
    args = parser.parse_args()

    RACELINE_PATH = resolve_raceline_path(args.raceline)
    RACELINE_TAG  = infer_raceline_tag(RACELINE_PATH)

    if not os.path.exists(RACELINE_PATH):
        print(f"ERROR: Raceline not found: {RACELINE_PATH}")
        sys.exit(1)

    # Set up BASE
    BASE.update(BASE_CONFIG)
    if args.objective == "racer":
        BASE.update(RACER_BASE_OVERRIDES)
    BASE = enforce_terminal_weight_floor(BASE)

    # Find binary
    binary = args.binary
    if binary is None:
        candidates = [
            os.path.join(MPCC_DIR, "build", "test_sim_drive"),
            os.path.join(MPCC_DIR, "test_sim_drive"),
            os.path.join(PROJECT_DIR, "build", "test_sim_drive"),
        ]
        for c in candidates:
            if os.path.isfile(c):
                binary = c
                break
    if binary is None or not os.path.isfile(binary):
        print("ERROR: Cannot find test_sim_drive binary. Use --binary PATH.")
        sys.exit(1)

    print(f"MPCC Tuner — objective={args.objective}  workers={args.jobs}")
    print(f"  LOCKED: HORIZON={LOCKED_HORIZON}  DT={LOCKED_DT}"
          f"  window={LOCKED_HORIZON * LOCKED_DT:.2f}s"
          f"  CROSS_CALL_SCALE={LOCKED_CROSS_CALL_SCALE:.4f}")
    print(f"  binary:   {binary}")
    print(f"  raceline: {RACELINE_PATH}")
    print(f"  Scoring:  zero-collision HARD constraint — any wc>0 → score=1000+wc")
    print()

    if args.sanity_only:
        sanity_check_params(binary)
        return

    # Output CSV
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = os.path.join(MPCC_DIR, "test",
                           f"tuning_{args.objective}_{ts}.csv")
    all_fields = list(MPCC_PRINT_ORDER) + [
        "label", "phase", "raceline",
        "score", "tracker_score", "racer_score",
        "passed", "failed",
        "max_contouring_err", "avg_contouring_err",
        "max_heading_err", "avg_heading_err",
        "max_vx", "avg_speed",
        "lap_count", "best_lap_time",
        "avg_vel_err", "max_vel_err",
        "avg_solve_us", "max_solve_us",
        "wall_collisions", "time_above_5ms",
        "avg_iters", "avg_rho", "avg_rho_u",
        "avg_adapt_updates", "avg_clip_events",
        "s_backward_jumps", "s_large_corrections",
        "max_s_jump", "s_prediction_regressions",
        "max_predicted_s_span",
        "status", "return_code",
    ]
    csv_writer = IncrementalCSV(out_csv, all_fields)
    print(f"  Output: {out_csv}\n")

    results = []
    t0 = time.time()

    # ── Phase 1 ──────────────────────────────────────────────────────────────
    run_phase("Phase 1: One-at-a-time sensitivity",
              gen_one_at_a_time(), binary, results, t0,
              args.jobs, csv_writer, args.objective)
    print_best(results, args.objective, "after Phase 1")

    if args.phase1_only:
        return

    # ── Phase 2 ──────────────────────────────────────────────────────────────
    run_phase("Phase 2: Primary grid (contouring+lag+progress+terminal)",
              gen_primary_grid(), binary, results, t0,
              args.jobs, csv_writer, args.objective)
    print_best(results, args.objective, "after Phase 2")

    # Cascade: update BASE to best config so far
    top10 = get_top_n_params(results, CASCADE_TOP_N)
    if top10:
        BASE.clear()
        BASE.update(top10[0])
        BASE.update(enforce_terminal_weight_floor(BASE))

    # ── Phase 4 ──────────────────────────────────────────────────────────────
    # Run secondary grid for each of top-10 Phase-2 configs
    p4_combos = []
    for seed_params in top10:
        for qvy, qom, rd, wdr, vtm in itertools.product(
                PHASE4_VALUES["Q_VY"],
                PHASE4_VALUES["Q_OMEGA"],
                PHASE4_VALUES["R_DELTA"],
                PHASE4_VALUES["W_DELTA_RATE"],
                PHASE4_VALUES["V_THETA_MAX"]):
            w = dict(seed_params)
            w["Q_VY"]         = qvy
            w["Q_OMEGA"]      = qom
            w["R_DELTA"]      = rd
            w["W_DELTA_RATE"] = wdr
            w["V_THETA_MAX"]  = vtm
            w = enforce_terminal_weight_floor(w)
            p4_combos.append((f"VY={qvy}+O={qom}+RD={rd}+WDR={wdr}+VT={vtm}", w))

    run_phase("Phase 4: Secondary grid (state-reg + control effort + v_theta)",
              p4_combos, binary, results, t0,
              args.jobs, csv_writer, args.objective)
    print_best(results, args.objective, "after Phase 4")

    best = get_best_params(results)

    # ── Phase 5 (skipped — ADMM params unused with OSQP solver) ────────────
    print("\n  Phase 5: SKIPPED — ADMM solver params unused with OSQP")
    print_best(results, args.objective, "after Phase 4 (Phase 5 skipped)")

    best = get_best_params(results)

    # ── Phases 6-8: iterative optimization ───────────────────────────────────
    for opt_pass in range(GLOBAL_OPTIMIZATION_PASSES):
        best = get_best_params(results)
        print(f"\n  === Optimization pass {opt_pass+1}/{GLOBAL_OPTIMIZATION_PASSES} ===")

        # Phase 6: fine-tuning
        run_phase(f"Phase 6 (pass {opt_pass+1}): Fine-tuning",
                  gen_fine_tuning(best), binary, results, t0,
                  args.jobs, csv_writer, args.objective)

        best = get_best_params(results)

        # Phase 7: random exploration
        n7 = PHASE7_RANDOM_COUNT[args.objective]
        run_phase(f"Phase 7 (pass {opt_pass+1}): Random exploration ({n7})",
                  gen_random_neighbors(best, n7, args.objective,
                                       seed_offset=opt_pass * 10000),
                  binary, results, t0,
                  args.jobs, csv_writer, args.objective)

        best = get_best_params(results)

        # Phase 8: exploitation
        n8 = PHASE8_RANDOM_COUNT[args.objective]
        exploit_profile = (f"{args.objective}_exploit"
                           if f"{args.objective}_exploit" in RANDOM_PROFILES
                           else args.objective)
        run_phase(f"Phase 8 (pass {opt_pass+1}): Exploitation ({n8})",
                  gen_random_neighbors(best, n8, args.objective,
                                       profile_override=exploit_profile,
                                       seed_offset=opt_pass * 10000 + 5000),
                  binary, results, t0,
                  args.jobs, csv_writer, args.objective)

        print_best(results, args.objective, f"pass {opt_pass+1}")

    # ── Final summary ─────────────────────────────────────────────────────────
    print(f"\n{'='*80}")
    print("FINAL RESULT (collision-free configs only, ranked by lap time)")
    print(f"{'='*80}")
    print_best(results, args.objective, "FINAL")

    safe = [r for r in results if is_safe_result(r)]
    print(f"  Total runs: {len(results)}  |  "
          f"Collision-free: {len(safe)}  |  "
          f"Output: {out_csv}")


if __name__ == "__main__":
    main()