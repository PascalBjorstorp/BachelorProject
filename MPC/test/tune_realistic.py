#!/usr/bin/env python3
"""
Comprehensive MPC Weight Tuning for REALISTIC_SIM=1 Mode
=========================================================
Sweeps weights, horizons, wall margins, wall stiffness,
and multiple raceline variants.  Optimized for maximum velocity
while maintaining safety under all realistic effects.

Supports two modes:
  Spielberg  — Large Spielberg track with multiple clearance racelines
  Hardware   — Small SLAM-mapped track (~22m, 0.27-1.4m wide)

Usage:
    python3 test/tune_realistic.py Spielberg              # Full Spielberg sweep
    python3 test/tune_realistic.py Hardware               # Full Hardware sweep
    python3 test/tune_realistic.py Hardware --quick       # Quick Hardware sweep
    python3 test/tune_realistic.py Hardware --phase 2     # Run single phase
    python3 test/tune_realistic.py Hardware --jobs 8      # 8 parallel workers
    python3 test/tune_realistic.py Hardware -j 0          # Auto-detect CPU count
    python3 test/tune_realistic.py Hardware --trials 3    # 3-seed aggregate per config
    python3 test/tune_realistic.py Hardware --seed-base 7 # Deterministic seed offset
    python3 test/tune_realistic.py Hardware --objective fastest
    python3 test/tune_realistic.py Hardware --cascade-top 20  # Cascade top-20
"""

import subprocess, os, sys, csv, itertools, time, random, hashlib
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED
import multiprocessing
from typing import Optional

# ─── Environment ─────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TRAJ_DIR = os.path.join(os.path.dirname(PROJECT_DIR),
                         "f1tenth_planning", "trajectories")

# ═══════════════════════════════════════════════════════════════════════════════
# MODE-SPECIFIC CONFIGURATIONS
# ═══════════════════════════════════════════════════════════════════════════════

# ─── Spielberg: large track, single current raceline ─────────────────────────
SPIELBERG_RACELINES = {
    "spielberg": os.path.join(TRAJ_DIR, "Spielberg_raceline_current.csv"),
}

SPIELBERG_BASE = {
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
    "WALL_MARGIN":  0.15,
    "HORIZON":      20,
    "PRED_DT":      0.03,
}

SPIELBERG_PER_RACELINE_WM = {
    "spielberg": 0.20,
}

# ─── Hardware: small SLAM-mapped track (~22m, 0.27-1.4m wide) ────────────────
# Q_LAT and Q_HDG must be ~5000 (vs 340/1000 for Spielberg) because the track
# has only ~0.10m effective clearance at tight corners (wp 48-58).
# Realistic mode will always clip 1 wall at that corner; scoring tolerates this.
HARDWARE_RACELINES = {
    "hardware": os.path.join(TRAJ_DIR, "hardware_raceline.csv"),
}

HARDWARE_BASE = {
    "Q_LAT":        5000.0,
    "Q_HDG":        1000.0,
    "Q_VEL":        100.0,
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
    "WALL_END":     10,
    "WALL_STRIDE":  4,
    "WALL_MARGIN":  0.02,
    "HORIZON":      10,
    "PRED_DT":      0.06,
}

HARDWARE_PER_RACELINE_WM = {
    "hardware": 0.02,
}

# ─── Active config (set by mode selection in main()) ─────────────────────────
RACELINES = {}
BASE = {}
MODE = "Spielberg"
CASCADE_TOP_N = 1
MAX_ALLOWED_COLLISIONS = 0
TRIALS_PER_CONFIG = 1
SEED_BASE = 42
OBJECTIVE = "tracker"

# ─── Special env vars (not weights, passed as env directly) ──────────────────
SPECIAL_PARAMS = {"WALL_MARGIN", "WALL_STRIDE", "HORIZON", "RACELINE_PATH", "PRED_DT"}

# ─── Sweep ranges (Spielberg) ────────────────────────────────────────────────
SPIELBERG_FULL_VALUES = {
    "Q_LAT":        [150, 200, 250, 300, 320, 340, 350, 360, 380, 400, 450, 500, 550, 600, 650, 700, 750, 800],
    "Q_HDG":        [100, 180, 190, 200, 210, 220, 300, 400, 600, 800, 900, 1000, 1200, 1500],
    "Q_VEL":        [4, 6, 8, 10, 15, 20, 22, 24, 25, 26, 28, 30],
    "Q_LAT_VEL":    [15, 20, 30, 40, 45, 55, 60, 80, 120],
    "Q_YAW":        [5, 10, 15, 18, 22, 30, 40, 60],
    "R_STEER":      [0.04, 0.06, 0.08, 0.09, 0.12, 0.15, 0.18, 0.20, 0.30, 0.5],
    "R_ACCEL":      [0.01, 0.012, 0.015, 0.02, 0.05],
    "W_JERK":       [0.08, 0.1, 0.14, 0.2, 0.3, 0.5, 1.0],
    "W_ACCEL_RATE": [0.05, 0.08, 0.1, 0.15, 0.2, 0.5, 1.0],
    "HORIZON":      [15, 20, 25, 30, 35, 40],
    "WALL_MARGIN":  [0.00, 0.05, 0.10, 0.15, 0.20, 0.30, 0.35, 0.40, 0.45, 0.50, 0.70],
    "WALL_END":     [12, 16, 18, 20, 22, 24, 28],
    "WALL_STRIDE":  [1, 2, 3, 4],
    "RHO":          [10, 20, 30, 40, 50, 60, 70, 80],
    "RHO_U":        [10, 15, 20, 25, 30, 40, 50],
    "ALPHA":        [0.93, 0.95, 1.0, 1.05, 1.15, 1.2, 1.4],
    "PRED_DT":      [0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1],
}

SPIELBERG_QUICK_VALUES = {
    "Q_LAT":        [300, 400, 600],
    "Q_HDG":        [600, 1000, 1200],
    "Q_VEL":        [20, 30],
    "Q_LAT_VEL":    [40, 60, 100],
    "Q_YAW":        [22, 40],
    "HORIZON":      [18, 19, 20],
    "WALL_MARGIN":  [0.10, 0.20, 0.40, 0.70],
    "WALL_END":     [10, 12, 16, 20],
    "WALL_STRIDE":  [1, 2, 3],
    "PRED_DT":      [0.02, 0.04, 0.06, 0.1],
}

# ─── Sweep ranges (Hardware) ────────────────────────────────────────────────
# Tracker and fastest objectives intentionally use different search spaces to
# bias exploration toward their own goals.
HARDWARE_TRACKER_FULL_VALUES = {
    "Q_LAT":        [3500, 4500, 5000, 6000, 7000],
    "Q_HDG":        [700, 900, 1000, 1200, 1500],
    "Q_VEL":        [80, 90, 100, 110, 120],
    "Q_LAT_VEL":    [3, 5, 7, 10],
    "Q_YAW":        [3, 5, 7, 10],
    "R_STEER":      [0.25, 0.35, 0.4, 0.5, 0.6],
    "R_ACCEL":      [0.008, 0.01, 0.012],
    "W_JERK":       [0.3, 0.5, 0.7, 1.0],
    "W_ACCEL_RATE": [0.08, 0.1, 0.12, 0.15],
    "HORIZON":      [9, 10, 11, 12],
    "WALL_MARGIN":  [0.006, 0.008, 0.01, 0.012, 0.015],
    "WALL_END":     [8, 10, 12, 14],
    "WALL_STRIDE":  [1, 2, 3],
    "RHO":          [32, 40, 50, 64],
    "RHO_U":        [6, 8, 10, 12, 16],
    "ALPHA":        [1.1, 1.25, 1.4, 1.6],
    "PRED_DT":      [0.034, 0.036, 0.038, 0.04],
    "TOL":          [4.0, 5.0, 6.0, 7.0],
    "MAX_ITER":     [20, 24, 28],
}

HARDWARE_FASTEST_FULL_VALUES = {
    "Q_LAT":        [5000, 5600, 6200, 6800],
    "Q_HDG":        [650, 720, 800, 900],
    "Q_VEL":        [220, 260, 300, 340],
    "Q_LAT_VEL":    [2.0, 3, 4, 5, 8],
    "Q_YAW":        [2, 3, 5, 7],
    "R_STEER":      [0.2, 0.25, 0.3, 0.35],
    "R_ACCEL":      [0.008, 0.01, 0.012],
    "W_JERK":       [0.2, 0.3, 0.4, 0.5],
    "W_ACCEL_RATE": [0.06, 0.08, 0.1],
    "HORIZON":      [9, 10, 11, 12],
    "WALL_MARGIN":  [0.008, 0.01, 0.012, 0.015],
    "WALL_END":     [8, 10, 12, 14],
    "WALL_STRIDE":  [1, 2, 3],
    "RHO":          [50, 64, 80],
    "RHO_U":        [7, 8, 10, 12],
    "ALPHA":        [1.1, 1.25, 1.4, 1.55],
    "PRED_DT":      [0.036, 0.038, 0.04],
    "TOL":          [4.0, 4.5, 5.0],
    "MAX_ITER":     [20, 24, 28],
}

HARDWARE_TRACKER_QUICK_VALUES = {
    "Q_LAT":        [4500, 5000, 6000],
    "Q_HDG":        [850, 1000, 1200],
    "Q_VEL":        [90, 100, 110, 120],
    "Q_LAT_VEL":    [4, 5, 6],
    "Q_YAW":        [4, 5, 6],
    "R_STEER":      [0.35, 0.4, 0.5],
    "HORIZON":      [10, 11, 12],
    "WALL_MARGIN":  [0.008, 0.01, 0.012],
    "WALL_END":     [10, 12, 14],
    "WALL_STRIDE":  [1, 2],
    "PRED_DT":      [0.034, 0.036, 0.038],
}

HARDWARE_FASTEST_QUICK_VALUES = {
    "Q_LAT":        [5000, 5600, 6200],
    "Q_HDG":        [650, 800, 900],
    "Q_VEL":        [200, 260, 320],
    "Q_LAT_VEL":    [2, 3, 5],
    "Q_YAW":        [3, 5, 7],
    "R_STEER":      [0.2, 0.25, 0.3],
    "HORIZON":      [9, 10, 11],
    "WALL_MARGIN":  [0.008, 0.01, 0.012],
    "WALL_END":     [10, 12, 14],
    "WALL_STRIDE":  [1, 2, 3],
    "PRED_DT":      [0.036, 0.038, 0.04],
}

HARDWARE_TRACKER_GRID_VALUES = {
    "Q_LAT":        [4500, 5000, 6000],
    "Q_HDG":        [850, 1000, 1150],
    "Q_VEL":        [90, 100, 110],
    "HORIZON":      [10, 11],
    "PRED_DT":      [0.034, 0.036, 0.038],
    "WALL_MARGIN":  [0.008, 0.01, 0.012],
    "WALL_END":     [10, 12, 14],
}

HARDWARE_FASTEST_GRID_VALUES = {
    "Q_LAT":        [5000, 5600, 6200],
    "Q_HDG":        [650, 800, 900],
    "Q_VEL":        [220, 260, 300, 340],
    "HORIZON":      [9, 10, 11],
    "PRED_DT":      [0.036, 0.038, 0.04],
    "WALL_MARGIN":  [0.008, 0.01, 0.012],
    "WALL_END":     [10, 12, 14],
}

HARDWARE_FASTEST_BASE_OVERRIDES = {
    "Q_LAT": 5600.0,
    "Q_HDG": 800.0,
    "Q_VEL": 300.0,
    "Q_LAT_VEL": 3.0,
    "Q_YAW": 5.0,
    "R_STEER": 0.25,
    "R_ACCEL": 0.0106,
    "W_JERK": 0.3,
    "W_ACCEL_RATE": 0.1,
    "RHO": 64.0,
    "RHO_U": 8.0,
    "ALPHA": 1.4,
    "TOL": 4.5,
    "MAX_ITER": 20,
    "WALL_END": 10,
    "WALL_STRIDE": 2,
    "WALL_MARGIN": 0.012,
    "HORIZON": 10,
    "PRED_DT": 0.038,
}

HARDWARE_OBJECTIVE_SWEEPS = {
    "tracker": {
        "full": HARDWARE_TRACKER_FULL_VALUES,
        "quick": HARDWARE_TRACKER_QUICK_VALUES,
        "grid": HARDWARE_TRACKER_GRID_VALUES,
    },
    "fastest": {
        "full": HARDWARE_FASTEST_FULL_VALUES,
        "quick": HARDWARE_FASTEST_QUICK_VALUES,
        "grid": HARDWARE_FASTEST_GRID_VALUES,
    },
}

RANDOM_NEIGHBOR_PROFILES = {
    "tracker": {
        "num_perturb_range": (3, 6),
        "default_multipliers": [0.85, 0.95, 1.0, 1.1, 1.2],
        "param_multipliers": {
            "Q_LAT": [0.9, 0.97, 1.0, 1.08, 1.18],
            "Q_HDG": [0.9, 0.97, 1.0, 1.08, 1.18],
            "Q_VEL": [0.9, 0.95, 1.0, 1.05],
            "Q_LAT_VEL": [0.75, 0.9, 1.0, 1.15, 1.3],
            "Q_YAW": [0.75, 0.9, 1.0, 1.15, 1.3],
            "R_STEER": [0.85, 0.95, 1.0, 1.15, 1.3],
            "W_JERK": [0.85, 0.95, 1.0, 1.15, 1.3],
            "RHO": [0.75, 0.9, 1.0, 1.15, 1.35],
            "RHO_U": [0.75, 0.9, 1.0, 1.15, 1.35],
        },
        "discrete": {
            "HORIZON": [9, 10, 11, 12],
            "PRED_DT": [0.034, 0.036, 0.038, 0.04],
            "WALL_STRIDE": [1, 2, 3],
            "WALL_MARGIN": [0.006, 0.008, 0.01, 0.012, 0.015],
            "WALL_END": [8, 10, 12, 14],
            "ALPHA": [1.15, 1.25, 1.4, 1.55],
            "MAX_ITER": [20, 24, 28],
        },
    },
    "fastest": {
        "num_perturb_range": (3, 7),
        "default_multipliers": [0.9, 0.97, 1.0, 1.06, 1.12],
        "param_multipliers": {
            "Q_LAT": [0.9, 0.97, 1.0, 1.06, 1.12],
            "Q_HDG": [0.9, 0.97, 1.0, 1.06, 1.12],
            "Q_VEL": [0.97, 1.0, 1.05, 1.1, 1.15, 1.2],
            "Q_LAT_VEL": [0.85, 0.95, 1.0, 1.1, 1.2],
            "Q_YAW": [0.7, 0.85, 1.0, 1.1, 1.2],
            "R_STEER": [0.9, 0.97, 1.0, 1.08, 1.15],
            "W_JERK": [0.85, 0.95, 1.0, 1.1, 1.2],
            "RHO": [0.85, 0.95, 1.0, 1.1, 1.2],
            "RHO_U": [0.85, 0.95, 1.0, 1.1, 1.2],
        },
        "discrete": {
            "HORIZON": [9, 10, 11, 12],
            "PRED_DT": [0.034, 0.036, 0.038, 0.04, 0.042],
            "WALL_STRIDE": [1, 2, 3],
            "WALL_MARGIN": [0.006, 0.008, 0.01, 0.012, 0.015],
            "WALL_END": [8, 10, 12, 14],
            "ALPHA": [1.1, 1.25, 1.4, 1.55],
            "MAX_ITER": [20, 24, 28],
        },
    },
}

# Backward-compatible aliases used as defaults/fallbacks.
HARDWARE_FULL_VALUES = HARDWARE_TRACKER_FULL_VALUES
HARDWARE_QUICK_VALUES = HARDWARE_TRACKER_QUICK_VALUES
HARDWARE_GRID_VALUES = HARDWARE_TRACKER_GRID_VALUES

SUPPORTED_SWEEP_PARAMS = {
    "Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
    "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE",
    "RHO", "RHO_U", "ALPHA", "TOL", "MAX_ITER",
    "WALL_END", "WALL_STRIDE", "WALL_MARGIN",
    "HORIZON", "PRED_DT"
}

INT_ENV_PARAMS = {"HORIZON", "WALL_END", "WALL_STRIDE", "MAX_ITER"}


def canonicalize_params_for_env(params: dict) -> dict:
    """Normalize params to the values MPC actually receives via env parsing."""
    out = dict(params)

    # MPC code reads these with atoi(), so drop any fractional part.
    for k in INT_ENV_PARAMS:
        if k in out:
            out[k] = int(float(out[k]))

    return out

# Active sweep values (set by mode selection in main())
FULL_VALUES = {}
QUICK_VALUES = {}


def clone_values_dict(values_dict: dict) -> dict:
    """Copy sweep dictionaries so objective selection never mutates shared lists."""
    return {k: list(v) for k, v in values_dict.items()}


# ─── Test runner ─────────────────────────────────────────────────────────────
def run_test(params: dict, binary: str, raceline: Optional[str] = None, seed: Optional[int] = None) -> dict:
    """Run a single REALISTIC_SIM=1 test with given parameters."""
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"
    env["REALISTIC_SIM"] = "1"
    env["WALL_SOFT_K"] = "0"
    if seed is not None:
        env["SIM_SEED"] = str(seed)

    if raceline:
        env["RACELINE_PATH"] = raceline

    for name, value in params.items():
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
                    "avg_vx": float(parts[15]) if len(parts) > 15 else 0.0,
                }
            except (IndexError, ValueError):
                pass

    if result.returncode != 0:
        return {"status": "EXIT_FAIL", "return_code": result.returncode, "passed": 0, "failed": 6}
    return {"status": "NO_CSV", "return_code": result.returncode, "passed": 0, "failed": 6}


def derive_config_seed_base(label: str, params: dict, raceline_label: str = "") -> int:
    """Deterministic per-config seed base so results are reproducible across parallel runs."""
    eff = canonicalize_params_for_env(params)
    key = f"{SEED_BASE}|{raceline_label}|{label}|{sorted(eff.items())}"
    digest = hashlib.sha1(key.encode("utf-8")).hexdigest()
    return int(digest[:8], 16)


def run_test_with_trials(params: dict,
                         binary: str,
                         raceline: Optional[str] = None,
                         label: str = "",
                         raceline_label: str = "",
                         trials: int = 1) -> dict:
    """Run multiple seeds and aggregate conservatively to reduce single-seed overfitting."""
    trials = max(1, int(trials))
    seed0 = derive_config_seed_base(label, params, raceline_label)

    if trials == 1:
        r = run_test(params, binary, raceline, seed=seed0)
        r["trials"] = 1
        r["seed_base"] = seed0
        return r

    out = []
    for i in range(trials):
        rr = run_test(params, binary, raceline, seed=seed0 + i)
        out.append(rr)
        if rr.get("status") != "OK":
            fail = dict(rr)
            fail["status"] = f"TRIAL_{rr.get('status', 'FAIL')}"
            fail["trials"] = trials
            fail["seed_base"] = seed0
            return fail

    agg = {
        "status": "OK",
        "passed": min(r.get("passed", 0) for r in out),
        "failed": max(r.get("failed", 0) for r in out),
        "max_lat_err": max(r.get("max_lat_err", 0.0) for r in out),
        "avg_lat_err": sum(r.get("avg_lat_err", 0.0) for r in out) / trials,
        "max_hdg_err": max(r.get("max_hdg_err", 0.0) for r in out),
        "avg_hdg_err": sum(r.get("avg_hdg_err", 0.0) for r in out) / trials,
        "max_vx": sum(r.get("max_vx", 0.0) for r in out) / trials,
        "avg_solve_us": sum(r.get("avg_solve_us", 0.0) for r in out) / trials,
        "max_solve_us": max(r.get("max_solve_us", 0.0) for r in out),
        "wall_collisions": max(r.get("wall_collisions", 0) for r in out),
        "time_above_5ms": sum(r.get("time_above_5ms", 0.0) for r in out) / trials,
        "max_vel_err": max(r.get("max_vel_err", 0.0) for r in out),
        "avg_vel_err": sum(r.get("avg_vel_err", 0.0) for r in out) / trials,
        "avg_iters": sum(r.get("avg_iters", 0.0) for r in out) / trials,
        "trials": trials,
        "seed_base": seed0,
    }
    return agg


def is_safe_result(r: dict) -> bool:
    """True when run is valid and collision-free enough for objective ranking."""
    if r.get("status") != "OK":
        return False
    if int(r.get("wall_collisions", 999)) > MAX_ALLOWED_COLLISIONS:
        return False
    return True


def compute_tracker_score(r: dict) -> float:
    """Tracker score: minimize trajectory-following errors (lower is better)."""
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return 2000.0 + 100.0 * float(max(0, r.get("wall_collisions", 0) - MAX_ALLOWED_COLLISIONS))

    tracking = (
        r["avg_lat_err"] * 80.0 +
        r["max_lat_err"] * 15.0 +
        r["avg_hdg_err"] * 35.0 +
        r["max_hdg_err"] * 8.0 +
        r["avg_vel_err"] * 40.0 +
        r["max_vel_err"] * 4.0
    )
    solver = r.get("avg_iters", 0) * 0.2 + r["avg_solve_us"] * 0.001
    return round(tracking + solver, 3)


def compute_fastest_score(r: dict) -> float:
    """Fastest score: maximize average speed while staying collision-free."""
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return 2000.0 + 100.0 * float(max(0, r.get("wall_collisions", 0) - MAX_ALLOWED_COLLISIONS))

    # Solely speed-based objective: lower score means higher average speed.
    return round(-r.get("avg_vx", 0.0), 6)


def apply_scores(r: dict) -> dict:
    """Attach both objective scores and active primary score to a result row."""
    r["tracker_score"] = compute_tracker_score(r)
    r["fastest_score"] = compute_fastest_score(r)
    r["score"] = r["tracker_score"] if OBJECTIVE == "tracker" else r["fastest_score"]
    return r


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


def valid_wall_combo(params: dict, horizon: Optional[int] = None) -> bool:
    """Return True when wall settings are valid for the current horizon.

    Adds stricter checks for high-horizon Hardware sweeps so wall constraints
    remain dense enough and cover a meaningful prefix of the prediction.
    """
    h = int(horizon if horizon is not None else params.get("HORIZON", BASE.get("HORIZON", 20)))
    we = int(params.get("WALL_END", BASE.get("WALL_END", h)))
    ws = int(params.get("WALL_STRIDE", BASE.get("WALL_STRIDE", 1)))
    wm = float(params.get("WALL_MARGIN", BASE.get("WALL_MARGIN", 0.0)))

    if ws < 1 or we < 1 or we > h:
        return False

    # Keep hardware margins in a realistic band for the measured map clearances.
    if MODE == "Hardware" and (wm < 0.0 or wm > 0.20):
        return False

    # Ensure we still activate enough constrained nodes.
    constrained_nodes = (we // ws) + 1
    if constrained_nodes < 5:
        return False

    # Extra guardrails for long horizons where sparse/short wall windows under-constrain.
    if MODE == "Hardware" and h > 20:
        min_wall_end = max(12, int(round(0.60 * h)))
        if we < min_wall_end:
            return False
        if ws > 3:
            return False

    return True


def valid_objective_combo(params: dict) -> bool:
    """Objective-specific guardrails to keep tracker/fastest searches distinct."""
    qv = float(params.get("Q_VEL", BASE.get("Q_VEL", 0.0)))
    if OBJECTIVE == "tracker" and qv > 140.0:
        return False
    return True


def assert_sweep_params_supported(values_dict: dict):
    """Fail fast if sweep keys include unknown/unused MPC env variables."""
    unknown = sorted(set(values_dict.keys()) - SUPPORTED_SWEEP_PARAMS)
    if unknown:
        raise RuntimeError("Unsupported sweep parameter(s): " + ", ".join(unknown))


def _result_signature(r: dict) -> tuple:
    """Comparable result signature for sanity-checking parameter impact."""
    return (
        r.get("status"),
        r.get("passed"),
        r.get("failed"),
        round(r.get("max_lat_err", 0.0), 6),
        round(r.get("avg_lat_err", 0.0), 6),
        round(r.get("max_hdg_err", 0.0), 6),
        round(r.get("avg_hdg_err", 0.0), 6),
        round(r.get("max_vx", 0.0), 6),
        round(r.get("avg_vel_err", 0.0), 6),
        round(r.get("avg_iters", 0.0), 6),
        r.get("wall_collisions"),
        round(r.get("time_above_5ms", 0.0), 6),
    )


def sanity_check_parameter_effects(binary: str, raceline: str):
    """Run small A/B checks so key swept params demonstrably affect output."""
    baseline = run_test(dict(BASE), binary, raceline)
    baseline_sig = _result_signature(baseline)

    probes = [
        ("Q_LAT", max(1.0, float(BASE.get("Q_LAT", 1.0)) * 1.5)),
        ("RHO", max(1.0, float(BASE.get("RHO", 1.0)) * 1.5)),
        ("HORIZON", int(max(2, min(40, int(BASE.get("HORIZON", 20)) + 5)))),
    ]
    if MODE == "Hardware":
        probes.append(("WALL_END", int(max(6, min(32, int(BASE.get("WALL_END", 10)) + 6)))))

    ineffective = []
    for name, new_val in probes:
        p = dict(BASE)
        p[name] = new_val
        if not valid_wall_combo(p):
            continue
        rr = run_test(p, binary, raceline)
        if _result_signature(rr) == baseline_sig:
            ineffective.append(name)

    if ineffective:
        print("WARNING: Possible no-effect sweep parameters detected: " + ", ".join(ineffective))
        print("         Verify env-variable plumbing in test_sim_drive/mpc_riccati.")


def gen_primary_grid(values_dict):
    """Grid: Q_LAT × Q_HDG × Q_VEL × HORIZON × PRED_DT.
    In Hardware mode, also sweeps WALL_MARGIN × WALL_END
    so that wall avoidance params are co-optimized with driving weights."""
    combos = []
    ql_vals = values_dict.get("Q_LAT", [BASE["Q_LAT"]])
    qh_vals = values_dict.get("Q_HDG", [BASE["Q_HDG"]])
    qv_vals = values_dict.get("Q_VEL", [BASE["Q_VEL"]])
    h_vals = values_dict.get("HORIZON", [BASE["HORIZON"]])
    pd_vals = values_dict.get("PRED_DT", [BASE["PRED_DT"]])

    if MODE == "Hardware":
        # Hardware: include wall params in primary grid (reduced per-dim)
        wm_vals = values_dict.get("WALL_MARGIN", [BASE["WALL_MARGIN"]])
        we_vals = values_dict.get("WALL_END", [BASE["WALL_END"]])
        for ql, qh, qv, h, pd, wm, we in itertools.product(
                ql_vals, qh_vals, qv_vals, h_vals, pd_vals, wm_vals, we_vals):
            w = dict(BASE)
            w["Q_LAT"] = ql; w["Q_HDG"] = qh; w["Q_VEL"] = qv
            w["HORIZON"] = h; w["PRED_DT"] = pd
            w["WALL_MARGIN"] = wm; w["WALL_END"] = we
            if not valid_wall_combo(w, horizon=h):
                continue
            combos.append((f"L={ql}+H={qh}+V={qv}+N={h}+dt={pd}+WM={wm}+WE={we}", w))
    else:
        for ql, qh, qv, h, pd in itertools.product(ql_vals, qh_vals, qv_vals, h_vals, pd_vals):
            w = dict(BASE)
            w["Q_LAT"] = ql; w["Q_HDG"] = qh; w["Q_VEL"] = qv; w["HORIZON"] = h; w["PRED_DT"] = pd
            combos.append((f"L={ql}+H={qh}+V={qv}+N={h}+dt={pd}", w))
    return combos


def gen_wall_grid(values_dict):
    """Grid: WALL_MARGIN × WALL_END × WALL_STRIDE.
    Skips combos where WALL_END > current HORIZON (from BASE/cascade)."""
    combos = []
    wm_vals = values_dict.get("WALL_MARGIN", [BASE["WALL_MARGIN"]])
    we_vals = values_dict.get("WALL_END", [BASE["WALL_END"]])
    ws_vals = values_dict.get("WALL_STRIDE", [BASE["WALL_STRIDE"]])
    horizon = BASE.get("HORIZON", 20)
    for wm, we, ws in itertools.product(wm_vals, we_vals, ws_vals):
        w = dict(BASE)
        w["WALL_MARGIN"] = wm; w["WALL_END"] = we
        w["WALL_STRIDE"] = ws
        if not valid_wall_combo(w, horizon=horizon):
            continue
        combos.append((f"WM={wm}+WE={we}+WS={ws}", w))
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


def gen_solver_grid(objective="tracker"):
    """Grid: RHO × RHO_U × ALPHA with objective-specific defaults."""
    combos = []
    if objective == "fastest":
        rho_vals = [32, 40, 50, 64]
        rho_u_vals = [8, 10, 12, 16]
        alpha_vals = [1.1, 1.25, 1.4, 1.6]
    else:
        rho_vals = [32, 40, 50, 64]
        rho_u_vals = [6, 8, 10, 12, 16]
        alpha_vals = [1.1, 1.25, 1.4, 1.6]

    for rho in rho_vals:
        for rho_u in rho_u_vals:
            for alpha in alpha_vals:
                w = dict(BASE)
                w["RHO"] = rho; w["RHO_U"] = rho_u; w["ALPHA"] = alpha
                combos.append((f"rho={rho}+ru={rho_u}+a={alpha}", w))
    return combos


def gen_velocity_push(objective="tracker"):
    """Configurations targeting objective-specific speed/handling tradeoffs."""
    combos = []
    if objective == "fastest":
        configs = [
            {"Q_VEL": 260, "Q_LAT": 5600, "Q_HDG": 800, "Q_LAT_VEL": 3, "Q_YAW": 5},
            {"Q_VEL": 300, "Q_LAT": 5600, "Q_HDG": 800, "Q_LAT_VEL": 3, "Q_YAW": 5},
            {"Q_VEL": 320, "Q_LAT": 5600, "Q_HDG": 720, "Q_LAT_VEL": 3, "Q_YAW": 5},
            {"Q_VEL": 340, "Q_LAT": 6200, "Q_HDG": 720, "Q_LAT_VEL": 3, "Q_YAW": 5},
            {"Q_VEL": 300, "HORIZON": 10, "PRED_DT": 0.038},
            {"Q_VEL": 320, "HORIZON": 10, "PRED_DT": 0.036},
            {"Q_VEL": 300, "R_STEER": 0.25, "Q_YAW": 5},
            {"Q_VEL": 320, "R_STEER": 0.25, "Q_YAW": 3},
            {"Q_VEL": 300, "WALL_MARGIN": 0.012, "WALL_END": 10, "WALL_STRIDE": 2},
            {"Q_VEL": 320, "WALL_MARGIN": 0.01, "WALL_END": 10, "WALL_STRIDE": 2},
            {"Q_VEL": 300, "ALPHA": 1.4, "RHO": 64, "RHO_U": 8},
            {"Q_VEL": 320, "ALPHA": 1.4, "RHO": 64, "RHO_U": 8},
        ]
    else:
        configs = [
            {"Q_VEL": 100, "Q_LAT": 5000, "Q_HDG": 1000},
            {"Q_VEL": 110, "Q_LAT": 5000, "Q_HDG": 1000},
            {"Q_VEL": 120, "Q_LAT": 6000, "Q_HDG": 1200},
            {"Q_VEL": 110, "Q_LAT": 7000, "Q_HDG": 1200},
            {"Q_VEL": 110, "WALL_MARGIN": 0.01, "WALL_END": 12, "WALL_STRIDE": 1},
            {"Q_VEL": 120, "WALL_MARGIN": 0.012, "WALL_END": 12, "WALL_STRIDE": 1},
            {"Q_VEL": 110, "R_STEER": 0.4, "W_JERK": 0.5},
            {"Q_VEL": 120, "R_STEER": 0.5, "W_JERK": 0.5},
            {"Q_VEL": 110, "Q_YAW": 5, "Q_LAT_VEL": 5},
            {"Q_VEL": 120, "Q_YAW": 5, "Q_LAT_VEL": 5},
            {"Q_VEL": 110, "ALPHA": 1.4, "RHO": 40, "RHO_U": 8},
            {"Q_VEL": 120, "ALPHA": 1.4, "RHO": 50, "RHO_U": 10},
        ]

    for cfg in configs:
        w = dict(BASE)
        w.update(cfg)
        if not valid_wall_combo(w):
            continue
        label = "+".join(f"{k}={v}" for k, v in cfg.items())
        combos.append((label, w))
    return combos


def gen_fine_tuning(best_weights, pct_range=(0.80, 0.85, 0.90, 0.95, 1.05, 1.10, 1.15, 1.20)):
    """Fine-tuning around best config."""
    combos = []
    skip = {"MAX_ITER", "HORIZON", "WALL_STRIDE"}
    for name, base_val in best_weights.items():
        if base_val == 0 or name in skip:
            continue
        for mult in pct_range:
            new_val = round(base_val * mult, 6)
            if name in ("WALL_END",):
                new_val = max(1, int(new_val))

            # Skip perturbations that do not change the effective value.
            base_eff = canonicalize_params_for_env({name: base_val})[name]
            new_eff = canonicalize_params_for_env({name: new_val})[name]
            if base_eff == new_eff:
                continue

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


def gen_random_neighbors(best_weights, n=150, profile=None, seed=None):
    """Random perturbations around best config with objective-specific profile."""
    combos = []
    rng = random.Random(42 if seed is None else int(seed))
    profile = profile or RANDOM_NEIGHBOR_PROFILES.get(OBJECTIVE, RANDOM_NEIGHBOR_PROFILES["tracker"])
    discrete = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_multipliers = profile.get("default_multipliers", [0.85, 0.95, 1.0, 1.1, 1.2])
    min_perturb, max_perturb = profile.get("num_perturb_range", (2, 6))

    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ITER",) and best_weights[k] != 0]
    i = 0
    attempts = 0
    max_attempts = max(50, n * 20)
    while i < n and attempts < max_attempts:
        attempts += 1
        w = dict(best_weights)
        num_perturb = rng.randint(min_perturb, min(max_perturb, len(tune_params)))
        params_to_perturb = rng.sample(tune_params, num_perturb)
        for name in params_to_perturb:
            if name in discrete:
                w[name] = rng.choice(discrete[name])
            else:
                mult = rng.choice(param_multipliers.get(name, default_multipliers))
                w[name] = round(w[name] * mult, 6)

            if name in INT_ENV_PARAMS:
                w[name] = int(float(w[name]))
                if name == "HORIZON":
                    w[name] = max(2, min(40, w[name]))
                elif name == "WALL_END":
                    w[name] = max(1, w[name])
                elif name == "WALL_STRIDE":
                    w[name] = max(1, w[name])
                elif name == "MAX_ITER":
                    w[name] = max(10, w[name])

        if not valid_wall_combo(w):
            continue

        combos.append((f"RND_{i}", w))
        i += 1

    return combos


# ─── Deduplication ───────────────────────────────────────────────────────────
def deduplicate(combos):
    seen = set()
    unique = []
    for label, params in combos:
        effective = canonicalize_params_for_env(params)
        key = tuple(sorted((k, round(v, 4) if isinstance(v, float) else v)
                           for k, v in effective.items()))
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
    label, params, binary, raceline, raceline_label, phase_name, trials = args
    r = run_test_with_trials(params, binary, raceline, label, raceline_label, trials)
    r = apply_scores(r)
    r["label"] = label
    r["phase"] = phase_name
    r["raceline"] = raceline_label or "default"
    r.update(params)
    return r


# ─── Phase runner ────────────────────────────────────────────────────────────
def run_phase(phase_name, combos, binary, results, t0,
              raceline=None, raceline_label="", num_workers=1,
              csv_writer=None):
    """Run a sweep phase. Returns (passed, failed)."""
    combos = [(label, params) for label, params in combos if valid_objective_combo(params)]
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

            r = run_test_with_trials(params, binary, raceline, label, raceline_label, TRIALS_PER_CONFIG)
            r = apply_scores(r)
            r["label"] = label
            r["phase"] = phase_name
            r["raceline"] = raceline_label or "default"
            r.update(params)
            results.append(r)
            if csv_writer:
                csv_writer.write_row(r)
            score = r["score"]

            if r["status"] != "OK":
                failed += 1
                print(f"FAIL  (ETA {eta:.0f}s)")
            elif not is_safe_result(r):
                failed += 1
                print(f"unsafe wc={r.get('wall_collisions', '?')}  (ETA {eta:.0f}s)")
            else:
                passed += 1
                wc_tag = f"wc={r['wall_collisions']} " if r["wall_collisions"] > 0 else ""
                tr_tag = f"n={r.get('trials', 1)} " if r.get("trials", 1) > 1 else ""
                tf_tag = f"tf={r.get('failed', 0)} " if int(r.get("failed", 0)) > 0 else ""
                print(f"sc={score:7.2f}  avx={r.get('avg_vx', 0.0):.2f}  vx={r['max_vx']:.1f}  "
                      f"t5={r['time_above_5ms']:.0f}s  "
                      f"lat={r['avg_lat_err']:.3f}  "
                      f"ve={r['avg_vel_err']:.2f}  {tf_tag}{wc_tag}{tr_tag}(ETA {eta:.0f}s)")
    else:
        # Parallel execution
        work_items = [
            (label, params, binary, raceline, raceline_label, phase_name, TRIALS_PER_CONFIG)
            for label, params in combos
        ]
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            it = iter(work_items)
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

                    tag = r["label"]
                    if raceline_label:
                        tag = f"{tag}|{raceline_label}"
                    r = apply_scores(r)
                    score = r["score"]

                    elapsed = time.time() - t0
                    rate = max(done_count, 1) / max(elapsed, 0.01)
                    eta = (total - done_count) / max(rate, 0.01)

                    if r["status"] != "OK":
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] {tag:60s} "
                              f"FAIL  (ETA {eta:.0f}s)")
                    elif not is_safe_result(r):
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] {tag:60s} "
                              f"unsafe wc={r.get('wall_collisions', '?')}  (ETA {eta:.0f}s)")
                    else:
                        passed += 1
                        wc_tag = f"wc={r['wall_collisions']} " if r["wall_collisions"] > 0 else ""
                        tr_tag = f"n={r.get('trials', 1)} " if r.get("trials", 1) > 1 else ""
                        tf_tag = f"tf={r.get('failed', 0)} " if int(r.get("failed", 0)) > 0 else ""
                        print(f"  [{done_count:4d}/{total}] {tag:60s} "
                              f"sc={score:7.2f}  avx={r.get('avg_vx', 0.0):.2f}  vx={r['max_vx']:.1f}  "
                              f"t5={r['time_above_5ms']:.0f}s  "
                              f"lat={r['avg_lat_err']:.3f}  "
                              f"ve={r['avg_vel_err']:.2f}  {tf_tag}{wc_tag}{tr_tag}(ETA {eta:.0f}s)")

    return passed, failed


# ─── Main ────────────────────────────────────────────────────────────────────
def main():
    global RACELINES, BASE, FULL_VALUES, QUICK_VALUES, MODE, CASCADE_TOP_N, MAX_ALLOWED_COLLISIONS, TRIALS_PER_CONFIG, SEED_BASE, OBJECTIVE

    # ─── Parse CLI arguments ─────────────────────────────────────────────
    quick = "--quick" in sys.argv
    phase_only = None
    raceline_arg = None
    num_workers = 0
    cascade_top = 1
    mode_arg = None
    trials = 1
    seed_base = 42
    objective = "tracker"
    hardware_grid_values = {}

    # First positional arg (not starting with --) after script name is the mode
    positional_args = [a for a in sys.argv[1:] if not a.startswith("-")
                       and a not in ("--quick",)]
    # Filter out values that follow --phase, --raceline, --jobs, -j, --cascade-top
    skip_next = set()
    for i, arg in enumerate(sys.argv):
        if arg in ("--phase", "--raceline", "--jobs", "-j", "--cascade-top", "--trials", "--seed-base", "--objective") and i + 1 < len(sys.argv):
            skip_next.add(sys.argv[i + 1])
    positional_args = [a for a in positional_args if a not in skip_next]
    if positional_args:
        mode_arg = positional_args[0]

    for i, arg in enumerate(sys.argv):
        if arg == "--phase" and i + 1 < len(sys.argv):
            phase_only = int(sys.argv[i + 1])
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_arg = sys.argv[i + 1]
        if arg == "--jobs" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "-j" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "--cascade-top" and i + 1 < len(sys.argv):
            cascade_top = int(sys.argv[i + 1])
        if arg == "--trials" and i + 1 < len(sys.argv):
            trials = int(sys.argv[i + 1])
        if arg == "--seed-base" and i + 1 < len(sys.argv):
            seed_base = int(sys.argv[i + 1])
        if arg == "--objective" and i + 1 < len(sys.argv):
            objective = sys.argv[i + 1].strip().lower()

    if objective not in ("tracker", "fastest"):
        print("ERROR: --objective must be one of: tracker, fastest")
        sys.exit(1)

    # ─── Mode selection ──────────────────────────────────────────────────
    if mode_arg and mode_arg.lower() in ("hardware", "hw"):
        MODE = "Hardware"
        RACELINES.update(HARDWARE_RACELINES)
        BASE.update(HARDWARE_BASE)
        if objective == "fastest":
            BASE.update(HARDWARE_FASTEST_BASE_OVERRIDES)
        hw_spaces = HARDWARE_OBJECTIVE_SWEEPS.get(objective, HARDWARE_OBJECTIVE_SWEEPS["tracker"])
        FULL_VALUES.update(clone_values_dict(hw_spaces["full"]))
        QUICK_VALUES.update(clone_values_dict(hw_spaces["quick"]))
        hardware_grid_values = clone_values_dict(hw_spaces["grid"])
        MAX_ALLOWED_COLLISIONS = 0  # Tight corner always clips in realistic
        PER_RACELINE_WM = HARDWARE_PER_RACELINE_WM
    elif mode_arg and mode_arg.lower() in ("spielberg", "sp"):
        MODE = "Spielberg"
        RACELINES.update(SPIELBERG_RACELINES)
        BASE.update(SPIELBERG_BASE)
        FULL_VALUES.update(SPIELBERG_FULL_VALUES)
        QUICK_VALUES.update(SPIELBERG_QUICK_VALUES)
        MAX_ALLOWED_COLLISIONS = 0
        PER_RACELINE_WM = SPIELBERG_PER_RACELINE_WM
    else:
        print("ERROR: First argument must be mode: Spielberg or Hardware")
        print("Usage: python3 test/tune_realistic.py <Spielberg|Hardware> [options]")
        sys.exit(1)

    CASCADE_TOP_N = max(1, cascade_top)
    TRIALS_PER_CONFIG = max(1, trials)
    SEED_BASE = int(seed_base)
    OBJECTIVE = objective

    if num_workers <= 0:
        num_workers = multiprocessing.cpu_count()

    print(f"\n  Mode:          {MODE}")
    print(f"  Workers:       {num_workers} "
          f"({'sequential' if num_workers == 1 else 'parallel'})")
    print(f"  Trials/config: {TRIALS_PER_CONFIG}")
    print(f"  Seed base:     {SEED_BASE}")
    print(f"  Objective:     {OBJECTIVE}")
    print(f"  Cascade top-N: {CASCADE_TOP_N}")
    print(f"  Max collisions allowed: {MAX_ALLOWED_COLLISIONS}")
    print(f"  Quick mode:    {quick}")

    os.chdir(PROJECT_DIR)
    binary_name = f"test_sim_drive_{os.getpid()}_{int(time.time())}"
    if os.name == "nt":
        binary_name += ".exe"
    binary = f"./{binary_name}"

    # Build optimized binary
    print("\nBuilding optimized test binary with REALISTIC_SIM support...")
    ret = subprocess.run([
        "gcc", "-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Wno-unknown-pragmas",
        "-Iinclude",
        "test/test_sim_drive.c", "src/mpc_riccati.c", "src/riccati_solver.c",
        "src/vehicle_model.c", "src/util_math.c",
        "-o", binary_name, "-lm"
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

    values = QUICK_VALUES if quick else FULL_VALUES
    assert_sweep_params_supported(values)
    results = []
    t0 = time.time()
    total_p = total_f = 0

    # Incremental CSV writer — results saved after each test
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_{MODE.lower()}_{OBJECTIVE}_{timestamp}.csv"
    fieldnames = (["label", "phase", "raceline", "score", "tracker_score", "fastest_score",
                   "passed", "failed", "max_lat_err", "avg_lat_err",
                   "max_hdg_err", "avg_hdg_err", "max_vx",
                   "avg_vx",
                   "avg_vel_err", "max_vel_err",
                   "avg_solve_us", "max_solve_us",
                   "wall_collisions", "time_above_5ms", "avg_iters", "status",
                   "trials", "seed_base", "return_code"]
                  + list(BASE.keys()))
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results file: {outfile} (incremental)\n")

    def should_run(phase_num):
        return phase_only is None or phase_only == phase_num

    # Helper: get top-N best params for this raceline from results so far
    def get_top_n_params(rl_tag, n=None):
        """Return list of up to N best-so-far params dicts for given raceline."""
        if n is None:
            n = CASCADE_TOP_N
        rl_results = [r for r in results
                      if r.get("raceline") == rl_tag and is_safe_result(r)]
        if not rl_results:
            return []
        rl_results.sort(key=lambda x: x.get("score", 999999.0))
        # Deduplicate by params
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
                        f"(score={r.get('score', 0.0):.2f}, avx={r.get('avg_vx', 0):.2f}, vx={r.get('max_vx', 0):.1f}, "
                      f"wc={r.get('wall_collisions', 0)})")
        return [p for _, p in unique]

    def get_best_params(rl_tag):
        """Return single best-so-far params dict (backward compat)."""
        top = get_top_n_params(rl_tag, n=1)
        return top[0] if top else None

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

        print(f"\n{'#'*80}")
        print(f"# Mode: {MODE}  Raceline: [{rl_tag}]  WALL_MARGIN={BASE['WALL_MARGIN']}")
        print(f"{'#'*80}")

        # Preflight: ensure key swept params change closed-loop outputs.
        sanity_check_parameter_effects(binary, rl_path)

        # ─── Phase 1: One-at-a-time (from original BASE) ────────────
        if should_run(1):
            p, f = run_phase("Phase 1: One-at-a-time",
                             gen_one_at_a_time(values), binary, results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        # ─── Phase 2: Primary grid (from original BASE) ─────────────
        if should_run(2):
            if MODE == "Hardware":
                grid_vals = hardware_grid_values if hardware_grid_values else HARDWARE_GRID_VALUES
                phase2_label = "Phase 2: Primary+Wall grid (Q_LAT×Q_HDG×Q_VEL×HORIZON×PRED_DT×WM×WE)"
            else:
                grid_vals = values
                phase2_label = "Phase 2: Primary grid (Q_LAT×Q_HDG×Q_VEL×HORIZON×PRED_DT)"
            p, f = run_phase(phase2_label,
                             gen_primary_grid(grid_vals), binary, results, t0,
                             rl_path, rl_tag, num_workers, csv_writer)
            total_p += p; total_f += f

        # ─── CASCADE: run phases 3+ for each of top-N from 1+2 ──────
        top_configs = get_top_n_params(rl_tag)
        if not top_configs:
            # Fallback: use original base if nothing passed
            top_configs = [dict(original_base)]
            if rl_tag in PER_RACELINE_WM:
                top_configs[0]["WALL_MARGIN"] = PER_RACELINE_WM[rl_tag]

        for ci, cascade_base in enumerate(top_configs):
            if CASCADE_TOP_N > 1:
                print(f"\n  >>> CASCADE branch {ci+1}/{len(top_configs)} <<<")

            # Set BASE to this cascade config
            for k, v in cascade_base.items():
                BASE[k] = v
            # Tracker keeps per-raceline wall margin defaults; fastest keeps
            # phase-discovered wall margin from top phase-1/2 candidates.
            if rl_tag in PER_RACELINE_WM and OBJECTIVE != "fastest":
                BASE["WALL_MARGIN"] = PER_RACELINE_WM[rl_tag]

            # ─── Phase 3: Wall grid (cascaded) ──────────────────────
            if should_run(3):
                p, f = run_phase(f"Phase 3: Wall grid (WM×WE×WS) [branch {ci+1}]",
                                 gen_wall_grid(values), binary, results, t0,
                                 rl_path, rl_tag, num_workers, csv_writer)
                total_p += p; total_f += f

            if not quick:
                # ─── CASCADE: update to best of 1-3 within this branch
                cascade_params = get_best_params(rl_tag)
                if cascade_params:
                    update_base(cascade_params)

                # ─── Phase 4: Secondary grid ────────────────────────
                if should_run(4):
                    p, f = run_phase(f"Phase 4: Secondary grid [branch {ci+1}]",
                                     gen_secondary_grid(values), binary, results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

                # ─── CASCADE: update to best of 1-4 ────────────────
                cascade_params = get_best_params(rl_tag)
                if cascade_params:
                    update_base(cascade_params)

                # ─── Phase 5: Solver parameters ─────────────────────
                if should_run(5):
                    p, f = run_phase(f"Phase 5: Solver grid [branch {ci+1}]",
                                     gen_solver_grid(OBJECTIVE), binary, results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

                # ─── CASCADE: update to best of 1-5 ────────────────
                cascade_params = get_best_params(rl_tag)
                if cascade_params:
                    update_base(cascade_params)

                # ─── Phase 6: Velocity push ─────────────────────────
                if should_run(6):
                    p, f = run_phase(f"Phase 6: Velocity push [branch {ci+1}]",
                                     gen_velocity_push(OBJECTIVE), binary, results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

            # ─── Phase 7: Fine-tuning around best ───────────────────
            if should_run(7):
                best_params = get_best_params(rl_tag)
                if best_params:
                    p, f = run_phase(f"Phase 7: Fine-tuning [branch {ci+1}]",
                                     gen_fine_tuning(best_params), binary, results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

            # ─── Phase 8: Random neighbors ──────────────────────────
            if should_run(8):
                best_params = get_best_params(rl_tag)
                if best_params:
                    if OBJECTIVE == "fastest":
                        n = 180 if not quick else 60
                    else:
                        n = 150 if not quick else 50
                    phase8_profile = RANDOM_NEIGHBOR_PROFILES.get(
                        OBJECTIVE,
                        RANDOM_NEIGHBOR_PROFILES["tracker"]
                    )
                    phase8_seed = derive_config_seed_base(
                        f"phase8-{OBJECTIVE}-branch{ci+1}",
                        best_params,
                        rl_tag,
                    )
                    p, f = run_phase(f"Phase 8: Random ({n}) [branch {ci+1}]",
                                     gen_random_neighbors(best_params, n,
                                                          profile=phase8_profile,
                                                          seed=phase8_seed), binary, results, t0,
                                     rl_path, rl_tag, num_workers, csv_writer)
                    total_p += p; total_f += f

        # Reset BASE to original for next raceline
        for k, v in original_base.items():
            BASE[k] = v

    # ─── Results ─────────────────────────────────────────────────────────
    results.sort(key=lambda x: x.get("score", 999999.0))
    elapsed = time.time() - t0

    print(f"\n{'='*80}")
    print(f"[{MODE}] COMPLETED {len(results)} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    if MAX_ALLOWED_COLLISIONS > 0:
        print(f"  (Collisions <= {MAX_ALLOWED_COLLISIONS} allowed in {MODE} mode)")
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

        tracker_sorted = outfile.replace(".csv", "_tracker_sorted.csv")
        fastest_sorted = outfile.replace(".csv", "_fastest_sorted.csv")
        safe_rows = [r for r in results if is_safe_result(r)]
        with open(tracker_sorted, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(sorted(safe_rows, key=lambda x: x.get("tracker_score", 999999.0)))
        with open(fastest_sorted, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(sorted(safe_rows, key=lambda x: x.get("fastest_score", 999999.0)))
        print(f"Tracker ranking:    {tracker_sorted}")
        print(f"Fastest ranking:    {fastest_sorted}")

    safe = [r for r in results if is_safe_result(r)]
    if safe:
        print(f"\n{'='*80}")
        print(f"[{MODE}] TOP 30 TRACKER (lowest score = best)")
        print(f"{'='*80}")
        fmt = "{:<4} {:<50} {:>8} {:>6} {:>6} {:>5} {:>5} {:>5} {:>4} {:>4} {:>3} {:>6} {:>6} {:>3}"
        print(fmt.format("Rank", "Label", "TrackSc", "AvgVx", "AvgVE", "MaxVx", "T>5s",
                  "AvgLt", "WM", "WE", "WS", "N", "PdDT", "WC"))
        print("-" * 140)
        tracker_top = sorted(safe, key=lambda x: x.get("tracker_score", 999999.0))
        for i, r in enumerate(tracker_top[:30]):
            print(fmt.format(
                i+1, r['label'][:50], f"{r.get('tracker_score', 0.0):.1f}",
                f"{r.get('avg_vx', 0.0):.2f}",
                f"{r['avg_vel_err']:.2f}", f"{r['max_vx']:.1f}",
                f"{r['time_above_5ms']:.0f}",
                f"{r['avg_lat_err']:.3f}",
                f"{r.get('WALL_MARGIN', '-')}",
                f"{r.get('WALL_END', '-')}",
                f"{r.get('WALL_STRIDE', '-')}",
                f"{r.get('HORIZON', '-')}",
                f"{r.get('PRED_DT', '-')}",
                f"{r.get('wall_collisions', '-')}" ))

        best = tracker_top[0]
        print(f"\n  BEST TRACKER ({MODE}):")
        print(f"    Tracker score: {best.get('tracker_score', 0.0):.2f}")
        print(f"    Avg velocity: {best.get('avg_vx', 0.0):.2f} m/s")
        print(f"    Max velocity: {best['max_vx']:.2f} m/s")
        print(f"    Avg lat err:  {best['avg_lat_err']:.4f} m")
        print(f"    Avg hdg err:  {best['avg_hdg_err']:.4f} rad")
        print(f"    Avg vel err:  {best['avg_vel_err']:.2f} m/s")
        print(f"    Walls:        {best['wall_collisions']}")
        print(f"    ---")
        for k in sorted(BASE.keys()):
            print(f"    {k:15s} = {best.get(k, BASE[k])}")

        print(f"\n{'='*80}")
        print(f"[{MODE}] TOP 30 FASTEST (lowest score = best)")
        print(f"{'='*80}")
        print(fmt.format("Rank", "Label", "FastSc", "AvgVx", "AvgVE", "MaxVx", "T>5s",
                  "AvgLt", "WM", "WE", "WS", "N", "PdDT", "WC"))
        print("-" * 140)
        fastest_top = sorted(safe, key=lambda x: x.get("fastest_score", 999999.0))
        for i, r in enumerate(fastest_top[:30]):
            print(fmt.format(
                i+1, r['label'][:50], f"{r.get('fastest_score', 0.0):.1f}",
                f"{r.get('avg_vx', 0.0):.2f}",
                f"{r['avg_vel_err']:.2f}", f"{r['max_vx']:.1f}",
                f"{r['time_above_5ms']:.0f}",
                f"{r['avg_lat_err']:.3f}",
                f"{r.get('WALL_MARGIN', '-')}",
                f"{r.get('WALL_END', '-')}",
                f"{r.get('WALL_STRIDE', '-')}",
                f"{r.get('HORIZON', '-')}",
                f"{r.get('PRED_DT', '-')}",
                f"{r.get('wall_collisions', '-')}" ))

        fastest = fastest_top[0]
        print(f"\n  FASTEST SAFE CONFIGURATION ({MODE}):")
        print(f"    Fastest score: {fastest.get('fastest_score', 0.0):.2f}")
        print(f"    Avg velocity:  {fastest.get('avg_vx', 0.0):.2f} m/s")
        print(f"    Max velocity: {fastest['max_vx']:.2f} m/s")
        print(f"    Time > 5 m/s: {fastest['time_above_5ms']:.1f} s")
        print(f"    Walls:        {fastest['wall_collisions']}")
        print(f"    ---")
        for k in sorted(BASE.keys()):
            print(f"    {k:15s} = {fastest.get(k, BASE[k])}")

    try:
        os.remove(binary_name)
    except OSError:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
