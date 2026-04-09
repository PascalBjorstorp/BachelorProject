#!/usr/bin/env python3
"""
MPC Weight Tuning for Hardware Map (REALISTIC_SIM=1 Mode)
==========================================================
Sweeps MPC weights, horizons, and solver parameters on the hardware
SLAM-mapped track (~22m, 0.27-1.4m wide). Optimized for maximum velocity
while maintaining safety under all realistic effects.

Usage:
    python3 test/tune_realistic_v2.py                        # Full sweep (all CPUs)
    python3 test/tune_realistic_v2.py --jobs 8               # Use 8 parallel workers
    python3 test/tune_realistic_v2.py -j 4                   # Use 4 workers
    python3 test/tune_realistic_v2.py --objective fastest    # Speed-first + recovery scoring
    python3 test/tune_realistic_v2.py --objective tracker    # Tracking-first scoring
    python3 test/tune_realistic_v2.py --raceline my_track_raceline.csv

The sweep runs 8 phases:
    Phase 1: One-at-a-time parameter sensitivity
    Phase 2: Primary grid (Q_LAT x Q_HDG x Q_VEL x HORIZON x PRED_DT)
    Phase 3: (Skipped for Hardware - wall margin is fixed)
    Phase 4: Secondary grid (Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE)
    Phase 5: Solver parameters (RHO x RHO_U x ALPHA x TOL)
    Phase 6: Fine-tuning around best config
    Phase 7: Random neighbor exploration
    Phase 8: Random exploitation around branch best

Each configuration is evaluated across multiple start scenarios:
    1. A standard raceline launch to estimate lap pace
    2. A left-offset recovery launch
    3. A right-offset recovery launch

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
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TRAJ_DIR = os.path.join(os.path.dirname(PROJECT_DIR), "f1tenth_planning", "trajectories")

HORIZON_SWEEP_VALUES = [8, 10, 12, 14, 16, 18, 20, 22, 26, 30]
HORIZON_LIMIT = 50

# ==============================================================================
# HARDWARE MAP CONFIGURATION
# ==============================================================================

DEFAULT_RACELINE_NAME = "my_track_raceline.csv"
RACELINE_PATH = os.path.join(TRAJ_DIR, DEFAULT_RACELINE_NAME)
RACELINE_TAG = "my_track"
WALL_MARGIN = 0.20

# Base configuration - starting point for all sweeps
BASE_CONFIG = {
    "Q_LAT":        10000.0,
    "Q_HDG":        700.0,
    "Q_VEL":        120.0,
    "Q_LAT_VEL":    24.0,
    "Q_YAW":        22.0,
    "R_STEER":      0.30,
    "R_ACCEL":      0.01,
    "W_JERK":       0.03,
    "W_ACCEL_RATE": 0.10,
    "RHO":          32.0,
    "RHO_U":        20.0,
    "ALPHA":        0.93,
    "TOL":          5.0,
    "MAX_ITER":     20,
    "WALL_END":     30,
    "WALL_STRIDE":  1,
    "WALL_MARGIN":  0.20,
    "HORIZON":      30,
    "PRED_DT":      0.06,
}

# Override base for fastest objective
FASTEST_BASE_OVERRIDES = {
    "Q_LAT": 3500.0,
    "Q_HDG": 180.0,
    "Q_VEL": 260.0,
    "Q_LAT_VEL": 10.0,
    "Q_YAW": 8.0,
    "R_STEER": 0.38,
    "W_JERK": 0.06,
    "W_ACCEL_RATE": 0.08,
}

# ==============================================================================
# SWEEP VALUE RANGES - PHASE 2 (Primary Grid)
# ==============================================================================

PHASE2_VALUES = {
    # Q_LAT: lateral error weight (10 values)
    "Q_LAT": [7000, 8000, 9000, 9500, 10000, 10500, 11000, 12000, 13000, 14000],
    
    # Q_HDG: heading error weight (10 values)
    "Q_HDG": [400, 500, 600, 650, 700, 750, 800, 900, 1000, 1200],
    
    # Q_VEL: velocity tracking weight (10 values)
    "Q_VEL": [80, 100, 110, 120, 130, 140, 150, 160, 180, 200],
    
    # HORIZON: prediction horizon steps
    "HORIZON": HORIZON_SWEEP_VALUES,
    
    # PRED_DT: keep dense coverage around low-latency values that showed best yield.
    "PRED_DT": [0.034, 0.036, 0.038, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065, 0.07],
}

PHASE2_VALUES_FASTEST = {
    "Q_LAT": [1500, 2000, 2500, 3000, 3500, 4000, 5000, 6000, 7000, 8500],
    "Q_HDG": [60, 90, 120, 150, 180, 220, 260, 320, 450, 650],
    "Q_VEL": [120, 150, 180, 210, 240, 280, 320, 360, 420, 500],
    "HORIZON": HORIZON_SWEEP_VALUES,
    "PRED_DT": [0.034, 0.036, 0.038, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065, 0.07],
}

# ==============================================================================
# SWEEP VALUE RANGES - ALL PARAMETERS (for one-at-a-time and fine-tuning)
# ==============================================================================

FULL_SWEEP_VALUES = {
    "Q_LAT":        [6000, 7000, 8000, 8500, 9000, 9500, 10000, 10500, 11000, 11500, 12000, 13000, 14000, 15000],
    "Q_HDG":        [300, 400, 500, 550, 600, 650, 700, 750, 800, 850, 900, 1000, 1100, 1200],
    "Q_VEL":        [60, 80, 100, 110, 120, 130, 140, 150, 160, 170, 180, 200, 220, 250],
    "Q_LAT_VEL":    [8, 10, 12, 15, 18, 20, 24, 28, 32, 36, 40, 48],
    "Q_YAW":        [8, 10, 12, 15, 18, 20, 22, 24, 26, 30, 34, 40],
    "R_STEER":      [0.12, 0.15, 0.18, 0.20, 0.22, 0.25, 0.28, 0.30, 0.33, 0.35, 0.40, 0.45, 0.50],
    "R_ACCEL":      [0.004, 0.006, 0.008, 0.009, 0.01, 0.011, 0.012, 0.014, 0.016, 0.02],
    "W_JERK":       [0.005, 0.01, 0.015, 0.02, 0.03, 0.04, 0.05, 0.07, 0.1, 0.15, 0.2, 0.3, 0.4],
    "W_ACCEL_RATE": [0.04, 0.06, 0.08, 0.09, 0.1, 0.11, 0.12, 0.14, 0.16, 0.2],
    "HORIZON":      HORIZON_SWEEP_VALUES,
    "RHO":          [16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 80],
    "RHO_U":        [6, 8, 10, 12, 14, 16, 18, 20, 24, 28, 32],
    "ALPHA":        [0.80, 0.85, 0.90, 0.93, 0.97, 1.0, 1.05, 1.1, 1.2, 1.3, 1.5, 1.8],
    "PRED_DT":      [0.032, 0.034, 0.036, 0.038, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065, 0.07, 0.075],
    "TOL":          [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0],
}

FULL_SWEEP_VALUES_FASTEST = {
    "Q_LAT":        [1200, 1800, 2400, 3000, 3500, 4000, 5000, 6000, 7000, 8500, 10000, 12000],
    "Q_HDG":        [40, 60, 90, 120, 150, 180, 220, 260, 320, 400, 500, 650, 800, 1000],
    "Q_VEL":        [100, 130, 160, 190, 220, 250, 280, 320, 360, 420, 500],
    "Q_LAT_VEL":    [4, 6, 8, 10, 12, 15, 18, 22, 28],
    "Q_YAW":        [2, 4, 6, 8, 10, 14, 18, 22],
    "R_STEER":      [0.18, 0.22, 0.26, 0.30, 0.35, 0.40, 0.50, 0.60, 0.75],
    "R_ACCEL":      [0.004, 0.006, 0.008, 0.009, 0.01, 0.011, 0.012, 0.014, 0.016, 0.02],
    "W_JERK":       [0.01, 0.02, 0.03, 0.05, 0.08, 0.12, 0.18, 0.25, 0.35, 0.5],
    "W_ACCEL_RATE": [0.04, 0.06, 0.08, 0.09, 0.1, 0.11, 0.12, 0.14, 0.16, 0.2],
    "HORIZON":      HORIZON_SWEEP_VALUES,
    "RHO":          [16, 20, 24, 28, 32, 36, 40, 48, 56, 64, 80],
    "RHO_U":        [6, 8, 10, 12, 14, 16, 18, 20, 24, 28, 32],
    "ALPHA":        [0.80, 0.85, 0.90, 0.93, 0.97, 1.0, 1.05, 1.1, 1.2, 1.3, 1.5, 1.8],
    "PRED_DT":      [0.032, 0.034, 0.036, 0.038, 0.04, 0.045, 0.05, 0.055, 0.06, 0.065, 0.07, 0.075],
    "TOL":          [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 7.0],
}

# ==============================================================================
# PHASE 4: Secondary Grid Values (~2000 configs)
# Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE
# ==============================================================================

PHASE4_VALUES = {
    "Q_LAT_VEL":    [15, 20, 24, 28, 32],
    "Q_YAW":        [16, 20, 22, 26, 30],
    "R_STEER":      [0.22, 0.26, 0.30, 0.34, 0.38],
    "W_JERK":       [0.01, 0.02, 0.03, 0.05, 0.08],
    "R_ACCEL":      [0.008, 0.01, 0.012, 0.015],
    "W_ACCEL_RATE": [0.08, 0.1, 0.12, 0.15],
}

PHASE4_VALUES_FASTEST = {
    "Q_LAT_VEL":    [4, 8, 12, 16, 20],
    "Q_YAW":        [2, 4, 6, 10, 14],
    "R_STEER":      [0.26, 0.34, 0.42, 0.50, 0.62],
    "W_JERK":       [0.02, 0.04, 0.06, 0.10, 0.16],
    "R_ACCEL":      [0.008, 0.01, 0.012, 0.015],
    "W_ACCEL_RATE": [0.06, 0.08, 0.1, 0.14],
}

# ==============================================================================
# PHASE 5: Solver Grid Values (~2000 configs)
# RHO x RHO_U x ALPHA x TOL
# ==============================================================================

PHASE5_VALUES = {
    "RHO":      [20, 28, 32, 40, 50, 64, 80],
    "RHO_U":    [8, 12, 16, 20, 24, 28],
    "ALPHA":    [0.85, 0.93, 1.0, 1.1, 1.25, 1.4],
    "TOL":      [3.5, 4.0, 4.5, 5.0, 5.5, 6.0],
}

# ==============================================================================
# RANDOM NEIGHBOR PROFILES
# ==============================================================================

RANDOM_PROFILES = {
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
            "R_ACCEL": [0.7, 0.85, 1.0, 1.2, 1.4],
            "W_JERK": [0.85, 0.95, 1.0, 1.15, 1.3],
            "W_ACCEL_RATE": [0.7, 0.85, 1.0, 1.2, 1.4],
            "RHO": [0.75, 0.9, 1.0, 1.15, 1.35],
            "RHO_U": [0.75, 0.9, 1.0, 1.15, 1.35],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [0.045, 0.05, 0.055, 0.06, 0.065, 0.07, 0.075],
            "ALPHA": [0.9, 0.93, 1.0, 1.1, 1.2],
        },
    },
    "fastest": {
        "num_perturb_range": (3, 7),
        "default_multipliers": [0.9, 0.97, 1.0, 1.06, 1.12],
        "param_multipliers": {
            "Q_LAT": [0.75, 0.85, 0.95, 1.0, 1.08, 1.18],
            "Q_HDG": [0.7, 0.85, 0.95, 1.0, 1.08, 1.18],
            "Q_VEL": [0.95, 1.0, 1.05, 1.12, 1.2, 1.3, 1.4],
            "Q_LAT_VEL": [0.6, 0.75, 0.9, 1.0, 1.1],
            "Q_YAW": [0.5, 0.7, 0.85, 1.0, 1.1],
            "R_STEER": [0.9, 0.97, 1.0, 1.08, 1.18, 1.3],
            "R_ACCEL": [0.8, 0.9, 1.0, 1.15, 1.3],
            "W_JERK": [0.8, 0.9, 1.0, 1.1, 1.25],
            "W_ACCEL_RATE": [0.8, 0.9, 1.0, 1.15, 1.3],
            "RHO": [0.85, 0.95, 1.0, 1.1, 1.2],
            "RHO_U": [0.85, 0.95, 1.0, 1.1, 1.2],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [0.045, 0.05, 0.055, 0.06, 0.065, 0.07],
            "ALPHA": [0.93, 1.0, 1.1, 1.2, 1.3],
        },
    },
    "tracker_exploit": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.95, 0.98, 1.0, 1.02, 1.05],
        "param_multipliers": {
            "Q_LAT": [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_HDG": [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_VEL": [0.97, 1.0, 1.03, 1.06],
            "Q_LAT_VEL": [0.9, 0.97, 1.0, 1.05, 1.1],
            "Q_YAW": [0.9, 0.97, 1.0, 1.05, 1.1],
            "R_STEER": [0.92, 0.98, 1.0, 1.05, 1.1],
            "R_ACCEL": [0.9, 0.97, 1.0, 1.05, 1.1],
            "W_JERK": [0.9, 0.97, 1.0, 1.05, 1.1],
            "W_ACCEL_RATE": [0.9, 0.97, 1.0, 1.05, 1.1],
            "RHO": [0.9, 0.97, 1.0, 1.06, 1.12],
            "RHO_U": [0.9, 0.97, 1.0, 1.06, 1.12],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [0.034, 0.036, 0.038, 0.04, 0.045, 0.05],
            "ALPHA": [0.9, 0.93, 1.0, 1.1],
        },
    },
    "fastest_exploit": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.96, 0.99, 1.0, 1.03, 1.07],
        "param_multipliers": {
            "Q_LAT": [0.9, 0.96, 0.99, 1.0, 1.03, 1.08],
            "Q_HDG": [0.88, 0.95, 0.99, 1.0, 1.03, 1.08],
            "Q_VEL": [0.98, 1.0, 1.03, 1.06, 1.1, 1.15],
            "Q_LAT_VEL": [0.85, 0.94, 0.98, 1.0, 1.04, 1.1],
            "Q_YAW": [0.82, 0.92, 0.98, 1.0, 1.04, 1.1],
            "R_STEER": [0.92, 0.98, 1.0, 1.04, 1.1, 1.16],
            "R_ACCEL": [0.9, 0.97, 1.0, 1.05, 1.1],
            "W_JERK": [0.9, 0.97, 1.0, 1.05, 1.1],
            "W_ACCEL_RATE": [0.9, 0.97, 1.0, 1.05, 1.1],
            "RHO": [0.9, 0.97, 1.0, 1.06, 1.12],
            "RHO_U": [0.9, 0.97, 1.0, 1.06, 1.12],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [0.034, 0.036, 0.038, 0.04, 0.045],
            "ALPHA": [0.93, 1.0, 1.1, 1.2],
        },
    },
}

# ==============================================================================
# CONSTANTS
# ==============================================================================

INT_PARAMS = {"HORIZON", "WALL_END", "WALL_STRIDE", "MAX_ITER"}

SCENARIO_VEHICLE_HALF_WIDTH = 0.137
SCENARIO_BODY_SAFETY_MARGIN = 0.06
RACE_SCENARIO_DURATION = 75.0
RECOVERY_SCENARIO_DURATION = 18.0
RECOVERY_START_SPEED = 2.0

TRACK_LENGTH_METERS = 0.0
RACELINE_START_LEFT_BOUND = 0.0
RACELINE_START_RIGHT_BOUND = 0.0
EVAL_SCENARIOS = []

CASCADE_TOP_N = 10  # Always cascade top 10 to next phases
SEED = 42           # Fixed seed for reproducibility
GLOBAL_OPTIMIZATION_PASSES = 10
PHASE7_RANDOM_COUNT = {"tracker": 3600, "fastest": 4400}
PHASE8_RANDOM_COUNT = {"tracker": 1800, "fastest": 2400}

# Keep summary print order aligned with swept define order in mpc_types.h.
MPC_TYPES_PRINT_ORDER = (
    "Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
    "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE",
    "HORIZON", "PRED_DT", "MAX_ITER", "WALL_MARGIN",
    "TOL", "RHO", "RHO_U", "ALPHA",
    "WALL_END", "WALL_STRIDE",
)

# Working copy of base config (modified during cascade)
BASE = {}


def infer_raceline_tag(path: str) -> str:
    """Create a compact label for CSV rows/reports from raceline filename."""
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem.endswith("_raceline"):
        stem = stem[: -len("_raceline")]
    return stem or "unknown"


def resolve_raceline_path(path_arg: str) -> str:
    """Resolve raceline path argument to an absolute path."""
    if os.path.isabs(path_arg):
        return path_arg

    # First treat as a trajectory filename under f1tenth_planning/trajectories.
    traj_candidate = os.path.join(TRAJ_DIR, path_arg)
    if os.path.exists(traj_candidate):
        return os.path.abspath(traj_candidate)

    # Fallback: resolve relative to the MPC project directory.
    proj_candidate = os.path.join(PROJECT_DIR, path_arg)
    return os.path.abspath(proj_candidate)


def load_raceline_metadata(path: str) -> dict:
    """Read track length and start corridor bounds from a raceline CSV."""
    first = None
    last = None

    with open(path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            try:
                s = float(row[0])
                left = float(row[7]) if len(row) > 7 else 5.0
                right = float(row[8]) if len(row) > 8 else 5.0
            except (ValueError, IndexError):
                continue

            sample = {"s": s, "left": left, "right": right}
            if first is None:
                first = sample
            last = sample

    if first is None or last is None:
        raise RuntimeError(f"Could not parse raceline CSV: {path}")

    track_length = max(0.0, last["s"] - first["s"])
    return {
        "track_length": track_length,
        "start_left_bound": first["left"],
        "start_right_bound": first["right"],
    }


def compute_recovery_offset(bound: float) -> float:
    """Choose a moderate off-raceline start offset that stays inside the corridor."""
    usable = float(bound) - SCENARIO_VEHICLE_HALF_WIDTH - SCENARIO_BODY_SAFETY_MARGIN
    if usable <= 0.0:
        return 0.0
    return round(min(0.35, 0.45 * usable), 4)


def build_eval_scenarios() -> list:
    """Build deterministic evaluation scenarios for speed and recovery."""
    left_offset = compute_recovery_offset(RACELINE_START_LEFT_BOUND)
    right_offset = compute_recovery_offset(RACELINE_START_RIGHT_BOUND)

    return [
        {
            "name": "race",
            "weight": 0.70,
            "seed_offset": 0,
            "env": {
                "SIM_DURATION": f"{RACE_SCENARIO_DURATION}",
                "START_OFFSET_LAT": "0.0",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": "0.0",
            },
        },
        {
            "name": "recover_left",
            "weight": 0.15,
            "seed_offset": 101,
            "env": {
                "SIM_DURATION": f"{RECOVERY_SCENARIO_DURATION}",
                "START_OFFSET_LAT": f"{left_offset}",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": f"{RECOVERY_START_SPEED}",
            },
        },
        {
            "name": "recover_right",
            "weight": 0.15,
            "seed_offset": 202,
            "env": {
                "SIM_DURATION": f"{RECOVERY_SCENARIO_DURATION}",
                "START_OFFSET_LAT": f"{-right_offset}",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": f"{RECOVERY_START_SPEED}",
            },
        },
    ]


def get_primary_grid_values(objective: str) -> dict:
    """Return the Phase 2 sweep for the active objective."""
    return PHASE2_VALUES_FASTEST if objective == "fastest" else PHASE2_VALUES


def get_full_sweep_values(objective: str) -> dict:
    """Return the broad sweep values for the active objective."""
    return FULL_SWEEP_VALUES_FASTEST if objective == "fastest" else FULL_SWEEP_VALUES


def get_secondary_grid_values(objective: str) -> dict:
    """Return the Phase 4 sweep for the active objective."""
    return PHASE4_VALUES_FASTEST if objective == "fastest" else PHASE4_VALUES


def iter_ordered_base_keys():
    """Yield BASE keys in mpc_types.h-inspired order, then any remaining keys."""
    seen = set()
    for key in MPC_TYPES_PRINT_ORDER:
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
    """Normalize params to the values MPC actually receives."""
    out = dict(params)
    
    # Hardware policy: WALL_END = HORIZON, WALL_STRIDE = 1
    h = int(float(out.get("HORIZON", BASE.get("HORIZON", HORIZON_LIMIT))))
    h = max(2, min(HORIZON_LIMIT, h))
    out["HORIZON"] = h
    out["WALL_END"] = h
    out["WALL_STRIDE"] = 1
    out["WALL_MARGIN"] = float(WALL_MARGIN)
    
    # Integer params
    for k in INT_PARAMS:
        if k in out:
            out[k] = int(float(out[k]))
    
    return out


def is_valid_config(params: dict) -> bool:
    """Check if configuration is valid for hardware map."""
    h = int(params.get("HORIZON", BASE.get("HORIZON", HORIZON_LIMIT)))
    if h < 2 or h > HORIZON_LIMIT:
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

def parse_csv_result(stdout: str, return_code: int) -> dict:
    """Parse the machine-readable summary emitted by test_sim_drive."""
    for line in stdout.splitlines():
        if line.startswith("CSV,"):
            parts = line.split(",")
            try:
                return {
                    "status": "OK",
                    "return_code": return_code,
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
                    "progress_m": float(parts[16]) if len(parts) > 16 else 0.0,
                    "avg_progress_mps": float(parts[17]) if len(parts) > 17 else 0.0,
                    "completed_laps": int(parts[18]) if len(parts) > 18 else 0,
                    "avg_lap_time": float(parts[19]) if len(parts) > 19 else 0.0,
                    "max_steer_change": float(parts[20]) if len(parts) > 20 else 0.0,
                    "steer_reversals": int(parts[21]) if len(parts) > 21 else 0,
                }
            except (IndexError, ValueError):
                break
    return None


def estimate_lap_time(r: dict) -> float:
    """Estimate lap time from completed laps or from progress speed."""
    if int(r.get("completed_laps", 0)) > 0 and float(r.get("avg_lap_time", 0.0)) > 0.0:
        return float(r["avg_lap_time"])

    avg_progress = float(r.get("avg_progress_mps", 0.0))
    if TRACK_LENGTH_METERS > 1e-6 and avg_progress > 0.1:
        return TRACK_LENGTH_METERS / avg_progress

    return 999.0


def is_safe_single_result(r: dict) -> bool:
    """True when one scenario run is valid and collision-free."""
    return r.get("status") == "OK" and int(r.get("wall_collisions", 999)) == 0


def weighted_mean(rows: list, key: str) -> float:
    """Weighted mean over scenario rows."""
    total_weight = sum(float(r.get("scenario_weight", 0.0)) for r in rows)
    if total_weight <= 0.0:
        return 0.0
    return sum(float(r.get(key, 0.0)) * float(r.get("scenario_weight", 0.0)) for r in rows) / total_weight


def aggregate_scenario_results(scenario_results: list) -> dict:
    """Collapse multiple scenario runs into one tuner-facing result row."""
    total_weight = sum(float(r.get("scenario_weight", 0.0)) for r in scenario_results) or 1.0
    first_bad_status = next((r.get("status") for r in scenario_results if r.get("status") != "OK"), "OK")
    completed_rows = [
        r for r in scenario_results
        if int(r.get("completed_laps", 0)) > 0 and float(r.get("avg_lap_time", 0.0)) > 0.0
    ]
    lap_weight = sum(float(r.get("scenario_weight", 0.0)) for r in completed_rows)

    aggregate = {
        "status": first_bad_status,
        "return_code": max(int(r.get("return_code", 0) or 0) for r in scenario_results),
        "scenario_count": len(scenario_results),
        "scenario_failures": sum(1 for r in scenario_results if not is_safe_single_result(r)),
        "recovery_failures": sum(
            1 for r in scenario_results
            if r.get("scenario_name") != "race" and not is_safe_single_result(r)
        ),
        "main_failed": 1 if any(
            r.get("scenario_name") == "race" and not is_safe_single_result(r)
            for r in scenario_results
        ) else 0,
        "passed": sum(int(r.get("passed", 0)) for r in scenario_results),
        "failed": sum(int(r.get("failed", 0)) for r in scenario_results),
        "wall_collisions": sum(int(r.get("wall_collisions", 0)) for r in scenario_results),
        "completed_laps": sum(int(r.get("completed_laps", 0)) for r in scenario_results),
        "time_above_5ms": sum(float(r.get("time_above_5ms", 0.0)) * float(r.get("scenario_weight", 0.0))
                               for r in scenario_results) / total_weight,
        "max_lat_err": max(float(r.get("max_lat_err", 0.0)) for r in scenario_results),
        "avg_lat_err": weighted_mean(scenario_results, "avg_lat_err"),
        "max_hdg_err": max(float(r.get("max_hdg_err", 0.0)) for r in scenario_results),
        "avg_hdg_err": weighted_mean(scenario_results, "avg_hdg_err"),
        "max_vx": max(float(r.get("max_vx", 0.0)) for r in scenario_results),
        "avg_vx": weighted_mean(scenario_results, "avg_vx"),
        "max_vel_err": max(float(r.get("max_vel_err", 0.0)) for r in scenario_results),
        "avg_vel_err": weighted_mean(scenario_results, "avg_vel_err"),
        "avg_solve_us": weighted_mean(scenario_results, "avg_solve_us"),
        "max_solve_us": max(float(r.get("max_solve_us", 0.0)) for r in scenario_results),
        "avg_iters": weighted_mean(scenario_results, "avg_iters"),
        "progress_m": weighted_mean(scenario_results, "progress_m"),
        "avg_progress_mps": weighted_mean(scenario_results, "avg_progress_mps"),
        "avg_lap_time": (
            sum(float(r.get("avg_lap_time", 0.0)) * float(r.get("scenario_weight", 0.0))
                for r in completed_rows) / lap_weight
        ) if lap_weight > 0.0 else 0.0,
        "lap_time_est": weighted_mean(scenario_results, "lap_time_est"),
        "max_steer_change": max(float(r.get("max_steer_change", 0.0)) for r in scenario_results),
        "steer_reversals": weighted_mean(scenario_results, "steer_reversals"),
    }

    for r in scenario_results:
        prefix = f"scenario_{r['scenario_name']}_"
        aggregate[f"{prefix}status"] = r.get("status", "UNKNOWN")
        aggregate[f"{prefix}lap_time_est"] = float(r.get("lap_time_est", 999.0))
        aggregate[f"{prefix}avg_progress_mps"] = float(r.get("avg_progress_mps", 0.0))
        aggregate[f"{prefix}avg_lat_err"] = float(r.get("avg_lat_err", 0.0))
        aggregate[f"{prefix}avg_vx"] = float(r.get("avg_vx", 0.0))
        aggregate[f"{prefix}wall_collisions"] = int(r.get("wall_collisions", 0))

    return aggregate


def run_single_scenario(params: dict, binary: str, scenario: dict, seed: int) -> dict:
    """Run one deterministic scenario for the current MPC configuration."""
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"
    env["REALISTIC_SIM"] = "1"
    env["WALL_SOFT_K"] = "0"
    env["SIM_SEED"] = str(seed + int(scenario.get("seed_offset", 0)))
    env["RACELINE_PATH"] = RACELINE_PATH
    
    effective_params = canonicalize_params(params)
    for name, value in effective_params.items():
        env[name] = str(value)

    for name, value in scenario.get("env", {}).items():
        env[name] = str(value)
    
    try:
        result = subprocess.run(
            [binary], capture_output=True, text=True, timeout=600, env=env
        )
    except subprocess.TimeoutExpired:
        return {
            "status": "TIMEOUT",
            "passed": 0,
            "failed": 6,
            "return_code": -1,
            "scenario_name": scenario["name"],
            "scenario_weight": float(scenario["weight"]),
            "lap_time_est": 999.0,
        }
    except FileNotFoundError:
        print(f"ERROR: Binary '{binary}' not found.")
        sys.exit(1)

    parsed = parse_csv_result(result.stdout, result.returncode)
    if parsed is None:
        if result.returncode != 0:
            parsed = {"status": "EXIT_FAIL", "return_code": result.returncode, "passed": 0, "failed": 6}
        else:
            parsed = {"status": "NO_CSV", "return_code": result.returncode, "passed": 0, "failed": 6}

    parsed["scenario_name"] = scenario["name"]
    parsed["scenario_weight"] = float(scenario["weight"])
    parsed["lap_time_est"] = estimate_lap_time(parsed)
    return parsed


def run_test(params: dict, binary: str, seed: int = SEED) -> dict:
    """Run all evaluation scenarios and return a single aggregate result."""
    scenario_results = [run_single_scenario(params, binary, scenario, seed) for scenario in EVAL_SCENARIOS]
    return aggregate_scenario_results(scenario_results)


# ==============================================================================
# SCORING
# ==============================================================================

def is_safe_result(r: dict) -> bool:
    """True when run is valid and collision-free."""
    return (
        r.get("status") == "OK"
        and int(r.get("wall_collisions", 999)) == 0
        and int(r.get("scenario_failures", 999)) == 0
    )


def compute_tracker_score(r: dict) -> float:
    """Tracker score: minimize trajectory-following errors (lower is better)."""
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return (
            1200.0
            + 250.0 * float(r.get("main_failed", 0))
            + 120.0 * float(r.get("recovery_failures", 0))
            + 40.0 * float(r.get("wall_collisions", 0))
        )
    
    tracking = (
        r["avg_lat_err"] * 70.0 +
        r["max_lat_err"] * 12.0 +
        r["avg_hdg_err"] * 28.0 +
        r["max_hdg_err"] * 6.0 +
        r["avg_vel_err"] * 18.0 +
        r["max_vel_err"] * 2.0 +
        r.get("lap_time_est", 999.0) * 2.5
    )
    solver = r.get("avg_iters", 0) * 0.2 + r["avg_solve_us"] * 0.0008
    return round(tracking + solver, 3)


def compute_fastest_score(r: dict) -> float:
    """Fastest score: minimize lap-time estimate, then lightly break ties by stability."""
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return round(
            400.0
            + 250.0 * float(r.get("main_failed", 0))
            + 120.0 * float(r.get("recovery_failures", 0))
            + 40.0 * float(r.get("wall_collisions", 0))
            + min(float(r.get("lap_time_est", 999.0)), 300.0),
            3,
        )
    
    stability = (
        r.get("avg_lat_err", 0.0) * 1.0 +
        r.get("avg_hdg_err", 0.0) * 0.4 +
        r.get("avg_vel_err", 0.0) * 0.08 +
        r.get("max_steer_change", 0.0) * 0.05 +
        r.get("steer_reversals", 0.0) * 0.015
    )
    solver = r.get("avg_solve_us", 0.0) * 0.0005 + r.get("avg_iters", 0.0) * 0.03
    return round(r.get("lap_time_est", 999.0) + stability + solver, 6)


def apply_scores(r: dict, objective: str) -> dict:
    """Attach scores to a result row."""
    r["tracker_score"] = compute_tracker_score(r)
    r["fastest_score"] = compute_fastest_score(r)
    r["score"] = r["tracker_score"] if objective == "tracker" else r["fastest_score"]
    return r


# ==============================================================================
# CONFIG GENERATORS
# ==============================================================================

def gen_one_at_a_time(objective: str) -> list:
    """Phase 1: Vary each parameter one at a time from baseline."""
    combos = [("BASELINE", dict(BASE))]
    
    for name, values in get_full_sweep_values(objective).items():
        for v in values:
            if abs(v - BASE.get(name, -999)) < 1e-6:
                continue
            w = dict(BASE)
            w[name] = v
            if is_valid_config(w):
                combos.append((f"{name}={v}", w))
    
    return combos


def gen_primary_grid(objective: str) -> list:
    """Phase 2: Primary grid sweep."""
    combos = []
    
    values = get_primary_grid_values(objective)
    ql_vals = values["Q_LAT"]
    qh_vals = values["Q_HDG"]
    qv_vals = values["Q_VEL"]
    h_vals = values["HORIZON"]
    pd_vals = values["PRED_DT"]
    
    for ql, qh, qv, h, pd in itertools.product(ql_vals, qh_vals, qv_vals, h_vals, pd_vals):
        w = dict(BASE)
        w["Q_LAT"] = ql
        w["Q_HDG"] = qh
        w["Q_VEL"] = qv
        w["HORIZON"] = h
        w["PRED_DT"] = pd
        w["WALL_END"] = h
        
        if is_valid_config(w):
            combos.append((f"L={ql}+H={qh}+V={qv}+N={h}+dt={pd}", w))
    
    return combos


def gen_secondary_grid(objective: str) -> list:
    """Phase 4: Secondary parameters grid (includes R_ACCEL and W_ACCEL_RATE)."""
    combos = []
    
    values = get_secondary_grid_values(objective)
    qlv_vals = values["Q_LAT_VEL"]
    qy_vals = values["Q_YAW"]
    rs_vals = values["R_STEER"]
    wj_vals = values["W_JERK"]
    ra_vals = values["R_ACCEL"]
    war_vals = values["W_ACCEL_RATE"]
    
    for qlv, qy, rs, wj, ra, war in itertools.product(
            qlv_vals, qy_vals, rs_vals, wj_vals, ra_vals, war_vals):
        w = dict(BASE)
        w["Q_LAT_VEL"] = qlv
        w["Q_YAW"] = qy
        w["R_STEER"] = rs
        w["W_JERK"] = wj
        w["R_ACCEL"] = ra
        w["W_ACCEL_RATE"] = war
        combos.append((f"LV={qlv}+Y={qy}+RS={rs}+WJ={wj}+RA={ra}+WAR={war}", w))
    
    return combos


def gen_solver_grid() -> list:
    """Phase 5: Solver parameters grid (RHO x RHO_U x ALPHA x TOL)."""
    combos = []
    
    rho_vals = PHASE5_VALUES["RHO"]
    rho_u_vals = PHASE5_VALUES["RHO_U"]
    alpha_vals = PHASE5_VALUES["ALPHA"]
    tol_vals = PHASE5_VALUES["TOL"]
    
    for rho, rho_u, alpha, tol in itertools.product(
            rho_vals, rho_u_vals, alpha_vals, tol_vals):
        w = dict(BASE)
        w["RHO"] = rho
        w["RHO_U"] = rho_u
        w["ALPHA"] = alpha
        w["TOL"] = tol
        combos.append((f"rho={rho}+ru={rho_u}+a={alpha}+tol={tol}", w))
    
    return combos


def gen_fine_tuning(best_weights: dict) -> list:
    """Phase 6: Fine-tuning around best config (~2000 configs)."""
    combos = []
    pct_range = (0.80, 0.85, 0.90, 0.92, 0.95, 0.97, 1.03, 1.05, 1.08, 1.10, 1.15, 1.20)
    skip = {"MAX_ITER", "HORIZON", "WALL_STRIDE", "WALL_END", "WALL_MARGIN"}
    
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
    key_params = ["Q_LAT", "Q_HDG", "Q_VEL", "R_STEER", "HORIZON", "Q_LAT_VEL", "Q_YAW"]
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
                         profile_override: str = None, seed_offset: int = 0) -> list: # type: ignore
    """Generate random perturbations around best config for exploration/exploitation phases."""
    combos = []
    rng = random.Random(SEED + seed_offset)
    profile_name = profile_override if profile_override else objective
    profile = RANDOM_PROFILES.get(profile_name, RANDOM_PROFILES["tracker"])
    
    discrete = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_multipliers = profile.get("default_multipliers", [0.85, 0.95, 1.0, 1.1, 1.2])
    min_perturb, max_perturb = profile.get("num_perturb_range", (3, 6))
    
    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ITER", "WALL_END", "WALL_STRIDE", "WALL_MARGIN") 
                   and best_weights[k] != 0]
    
    i = 0
    attempts = 0
    max_attempts = max(100, n * 20)
    
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
            
            if name in INT_PARAMS:
                w[name] = int(float(w[name]))
                if name == "HORIZON":
                    w[name] = max(2, min(HORIZON_LIMIT, w[name]))
        
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
            writer = csv.DictWriter(f, fieldnames=self.fieldnames, extrasaction="ignore")
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
    print(f"{phase_name} - {total} configurations ({num_workers} workers)")
    print(f"{'='*80}")
    
    passed = failed = 0
    
    if num_workers <= 1:
        # Sequential execution
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
                print(f"unsafe wc={r.get('wall_collisions', '?')}  (ETA {eta:.0f}s)")
            else:
                passed += 1
                print(f"sc={r['score']:7.2f}  avx={r.get('avg_vx', 0.0):.2f}  "
                      f"lap={r.get('lap_time_est', 0.0):.2f}s  "
                      f"prog={r.get('avg_progress_mps', 0.0):.2f}  (ETA {eta:.0f}s)")
    else:
        # Parallel execution
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            it = ((label, params, binary, phase_name, objective) for label, params in combos)
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
                        print(f"  [{done_count:4d}/{total}] {r['label']:55s} FAIL  (ETA {eta:.0f}s)")
                    elif not is_safe_result(r):
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] {r['label']:55s} "
                              f"unsafe wc={r.get('wall_collisions', '?')}  (ETA {eta:.0f}s)")
                    else:
                        passed += 1
                        print(f"  [{done_count:4d}/{total}] {r['label']:55s} "
                              f"sc={r['score']:7.2f}  avx={r.get('avg_vx', 0.0):.2f}  "
                              f"lap={r.get('lap_time_est', 0.0):.2f}s  "
                              f"prog={r.get('avg_progress_mps', 0.0):.2f}  (ETA {eta:.0f}s)")
    
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
        round(baseline.get("avg_lat_err", 0.0), 4),
        round(baseline.get("avg_hdg_err", 0.0), 4),
        round(baseline.get("max_vx", 0.0), 2),
    )
    
    probes = [
        ("Q_LAT", BASE.get("Q_LAT", 10000) * 1.5),
        ("Q_VEL", BASE.get("Q_VEL", 120) * 1.3),
        ("RHO", BASE.get("RHO", 32) * 1.5),
        ("HORIZON", min(HORIZON_LIMIT, int(BASE.get("HORIZON", HORIZON_LIMIT) + 2))),
    ]
    
    ineffective = []
    for name, new_val in probes:
        p = dict(BASE)
        p[name] = new_val
        if name == "HORIZON":
            p["WALL_END"] = int(new_val)
        
        rr = run_test(p, binary)
        sig = (
            rr.get("status"),
            round(rr.get("avg_lat_err", 0.0), 4),
            round(rr.get("avg_hdg_err", 0.0), 4),
            round(rr.get("max_vx", 0.0), 2),
        )
        if sig == baseline_sig:
            ineffective.append(name)
    
    if ineffective:
        print(f"  WARNING: Parameters with no detected effect: {', '.join(ineffective)}")
        print("           Check env-variable plumbing in MPC code.")
    else:
        print("  All tested parameters show effect on output - OK")


# ==============================================================================
# RESULT HELPERS
# ==============================================================================

def get_top_n_params(results: list, n: int = CASCADE_TOP_N) -> list:
    """Return list of up to N best params dicts."""
    safe = [r for r in results if is_safe_result(r)]
    
    if not safe:
        # Fallback to least-bad unsafe results
        unsafe = sorted(results, key=lambda x: (
            0 if x.get("status") == "OK" else 1,
            x.get("wall_collisions", 999),
            x.get("score", 99999)
        ))
        safe = unsafe[:n] if unsafe else []
        if safe:
            print("  WARNING: No safe candidates yet; using least-bad configs for cascade.")
    else:
        safe.sort(key=lambda x: x.get("score", 999999.0))
    
    # Deduplicate by params
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
                  f"lap={r.get('lap_time_est', 0.0):.2f}s, "
                  f"prog={r.get('avg_progress_mps', 0.0):.2f})")
    
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
    global TRACK_LENGTH_METERS, RACELINE_START_LEFT_BOUND, RACELINE_START_RIGHT_BOUND, EVAL_SCENARIOS
    
    # Parse arguments
    num_workers = multiprocessing.cpu_count()  # Default to max workers
    objective = "fastest"
    raceline_override = None
    
    for i, arg in enumerate(sys.argv):
        if arg in ("--jobs", "-j") and i + 1 < len(sys.argv):
            try:
                num_workers = int(sys.argv[i + 1])
            except ValueError:
                print(f"WARNING: invalid --jobs value '{sys.argv[i + 1]}', using CPU count")
                num_workers = multiprocessing.cpu_count()
            if num_workers <= 0:
                num_workers = multiprocessing.cpu_count()
        if arg == "--objective" and i + 1 < len(sys.argv):
            objective = sys.argv[i + 1].strip().lower()
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_override = sys.argv[i + 1].strip()
    
    if objective not in ("tracker", "fastest"):
        print("ERROR: --objective must be 'tracker' or 'fastest'")
        sys.exit(1)

    if raceline_override:
        RACELINE_PATH = resolve_raceline_path(raceline_override)
    else:
        RACELINE_PATH = os.path.abspath(RACELINE_PATH)
    RACELINE_TAG = infer_raceline_tag(RACELINE_PATH)

    if not os.path.exists(RACELINE_PATH):
        print(f"ERROR: Raceline not found: {RACELINE_PATH}")
        sys.exit(1)

    meta = load_raceline_metadata(RACELINE_PATH)
    TRACK_LENGTH_METERS = meta["track_length"]
    RACELINE_START_LEFT_BOUND = meta["start_left_bound"]
    RACELINE_START_RIGHT_BOUND = meta["start_right_bound"]
    EVAL_SCENARIOS = build_eval_scenarios()
    
    # Initialize BASE config
    BASE.update(BASE_CONFIG)
    if objective == "fastest":
        BASE.update(FASTEST_BASE_OVERRIDES)
    
    print(f"\n{'='*80}")
    print("MPC Weight Tuning - Hardware Map")
    print(f"{'='*80}")
    print(f"  Workers:     {num_workers}")
    print(f"  Objective:   {objective}")
    print(f"  Cascade:     top {CASCADE_TOP_N}")
    print(f"  Global passes (P5-P8): {GLOBAL_OPTIMIZATION_PASSES}")
    print(f"  Phase7 random: {PHASE7_RANDOM_COUNT.get(objective, 3600)}")
    print(f"  Phase8 random: {PHASE8_RANDOM_COUNT.get(objective, 1800)}")
    print(f"  Horizon sweep: {HORIZON_SWEEP_VALUES}")
    print(f"  Raceline:    {RACELINE_PATH}")
    print(f"  Raceline tag:{RACELINE_TAG}")
    print(f"  Track length:{TRACK_LENGTH_METERS:.3f} m")
    for scenario in EVAL_SCENARIOS:
        print(f"  Scenario {scenario['name']:<12s}"
              f" weight={scenario['weight']:.2f}"
              f" dur={float(scenario['env'].get('SIM_DURATION', 0.0)):>5.1f}s"
              f" lat={float(scenario['env'].get('START_OFFSET_LAT', 0.0)):>+5.2f}m"
              f" v0={float(scenario['env'].get('START_SPEED', 0.0)):>4.1f}m/s")
    
    os.chdir(PROJECT_DIR)
    
    # Build binary
    binary_name = f"test_sim_drive_{os.getpid()}_{int(time.time())}"
    if os.name == "nt":
        binary_name += ".exe"
    binary = f"./{binary_name}"
    
    print("\nBuilding test binary...")
    ret = subprocess.run([
        "gcc", "-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Wno-unknown-pragmas",
        "-Iinclude",
        "test/test_sim_drive.c", "src/mpc.c", "src/riccati_solver.c",
        "src/vehicle_model.c", "src/util_math.c",
        "-o", binary_name, "-lm"
    ], capture_output=True, text=True)
    
    if ret.returncode != 0:
        print(f"BUILD FAILED:\n{ret.stderr}")
        sys.exit(1)
    print("  Build OK")
    
    # Run sanity check
    sanity_check_params(binary)
    
    # Setup
    results = []
    t0 = time.time()
    total_p = total_f = 0
    
    # CSV writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_hardware_{objective}_{timestamp}.csv"
    scenario_fieldnames = []
    for scenario in EVAL_SCENARIOS:
        prefix = f"scenario_{scenario['name']}_"
        scenario_fieldnames.extend([
            f"{prefix}status",
            f"{prefix}lap_time_est",
            f"{prefix}avg_progress_mps",
            f"{prefix}avg_lat_err",
            f"{prefix}avg_vx",
            f"{prefix}wall_collisions",
        ])
    fieldnames = (
        ["label", "phase", "raceline", "score", "tracker_score", "fastest_score",
         "passed", "failed", "scenario_count", "scenario_failures", "recovery_failures", "main_failed",
         "lap_time_est", "completed_laps", "progress_m", "avg_progress_mps",
         "max_steer_change", "steer_reversals",
         "max_lat_err", "avg_lat_err",
         "max_hdg_err", "avg_hdg_err", "max_vx", "avg_vx",
         "avg_vel_err", "max_vel_err", "avg_solve_us", "max_solve_us",
         "wall_collisions", "time_above_5ms", "avg_iters", "avg_lap_time", "status", "return_code"]
        + scenario_fieldnames
        + list(BASE.keys())
    )
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results: {outfile}\n")
    
    # ========== PHASE 1: One-at-a-time ==========
    p, f = run_phase("Phase 1: One-at-a-time sensitivity",
                     gen_one_at_a_time(objective), binary, results, t0,
                     num_workers, csv_writer, objective)
    total_p += p
    total_f += f
    
    # ========== PHASE 2: Primary grid ==========
    combos = gen_primary_grid(objective)
    print(f"\n  Phase 2 will test {len(combos):,} configurations")
    p, f = run_phase("Phase 2: Primary grid (Q_LAT x Q_HDG x Q_VEL x HORIZON x PRED_DT)",
                     combos, binary, results, t0,
                     num_workers, csv_writer, objective)
    total_p += p
    total_f += f
    
    # Get top N for cascade
    print("\n  Selecting top configs for cascade...")
    top_configs = get_top_n_params(results)
    if not top_configs:
        top_configs = [dict(BASE)]
    
    # ========== PHASES 3-8 ==========
    # Phase 3 is skipped for hardware map mode (fixed wall margin).
    print("\n  Phase 3: Skipped (wall margin is fixed for hardware)")

    # Phase 4: run a reduced seed sweep for each top Phase-2 configuration.
    print(f"\n{'='*80}")
    print(f"Phase 4 seed screening from top {len(top_configs)} Phase-2 configs")
    print(f"{'='*80}")

    for ci, cascade_base in enumerate(top_configs):
        print(f"\n{'#'*80}")
        print(f"# PHASE 4 SEED {ci+1}/{len(top_configs)}")
        print(f"{'#'*80}")

        update_base(cascade_base)
        p, f = run_phase(f"Phase 4: Secondary grid [seed {ci+1}/{len(top_configs)}]",
                         gen_secondary_grid(objective), binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f

    # Promote one global best after seed screening.
    best = get_top_n_params(results, n=1)
    if best:
        update_base(best[0])

    # Global optimization loop: repeatedly refine one global-best candidate.
    for pi in range(GLOBAL_OPTIMIZATION_PASSES):
        print(f"\n{'#'*80}")
        print(f"# GLOBAL OPTIMIZATION PASS {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}")
        print(f"{'#'*80}")

        # Phase 5: solver parameter sweep
        p, f = run_phase(f"Phase 5: Solver parameters [pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
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
            p, f = run_phase(f"Phase 6: Fine-tuning [pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
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
            p, f = run_phase(f"Phase 7: Random neighbors ({n_random}) [pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
                             gen_random_neighbors(best[0], n_random, objective,
                                                  seed_offset=7000 + pi),
                             binary, results, t0,
                             num_workers, csv_writer, objective)
            total_p += p
            total_f += f

            top = get_top_n_params(results, n=1)
            if top:
                update_base(top[0])

        # Phase 8: random exploitation around updated global best
        best = get_top_n_params(results, n=1)
        if best:
            n_random = PHASE8_RANDOM_COUNT.get(objective, 1800)
            p, f = run_phase(f"Phase 8: Random exploitation ({n_random}) [pass {pi+1}/{GLOBAL_OPTIMIZATION_PASSES}]",
                             gen_random_neighbors(best[0], n_random, objective,
                                                  profile_override=f"{objective}_exploit",
                                                  seed_offset=9000 + pi),
                             binary, results, t0,
                             num_workers, csv_writer, objective)
            total_p += p
            total_f += f

            top = get_top_n_params(results, n=1)
            if top:
                update_base(top[0])
    
    # ========== FINAL RESULTS ==========
    results.sort(key=lambda x: x.get("score", 999999.0))
    elapsed = time.time() - t0
    
    print(f"\n{'='*80}")
    print(f"COMPLETED {len(results):,} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    print(f"{'='*80}")
    
    # Write sorted results
    sorted_file = outfile.replace(".csv", "_sorted.csv")
    if results:
        with open(sorted_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(results)
        print(f"Results: {sorted_file}")
    
    # Show top results
    safe = [r for r in results if is_safe_result(r)]
    if safe:
        print(f"\n{'='*80}")
        print(f"TOP 20 RESULTS ({objective} objective)")
        print(f"{'='*80}")
        
        fmt = "{:<4} {:<45} {:>8} {:>7} {:>6} {:>6} {:>3}"
        print(fmt.format("Rank", "Label", "Score", "Lap", "Prog", "AvgLat", "Rec"))
        print("-" * 90)
        
        top = sorted(safe, key=lambda x: x.get("score", 999999.0))[:20]
        for i, r in enumerate(top):
            print(fmt.format(
                i+1,
                r['label'][:45],
                f"{r.get('score', 0.0):.2f}",
                f"{r.get('lap_time_est', 0.0):.2f}",
                f"{r.get('avg_progress_mps', 0.0):.2f}",
                f"{r['avg_lat_err']:.4f}",
                f"{r.get('recovery_failures', 0)}"
            ))
        
        best = top[0]
        print(f"\nBEST CONFIGURATION:")
        print(f"  Score: {best.get('score', 0.0):.2f}")
        print(f"  Lap estimate: {best.get('lap_time_est', 0.0):.3f} s")
        print(f"  Avg progress: {best.get('avg_progress_mps', 0.0):.2f} m/s")
        print(f"  Avg velocity: {best.get('avg_vx', 0.0):.2f} m/s")
        print(f"  Avg lat err: {best['avg_lat_err']:.4f} m")
        print(f"  ---")
        for k in iter_ordered_base_keys():
            print(f"  {k:15s} = {best.get(k, BASE[k])}")
    
    # Cleanup
    try:
        os.remove(binary_name)
    except OSError:
        pass
    
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
