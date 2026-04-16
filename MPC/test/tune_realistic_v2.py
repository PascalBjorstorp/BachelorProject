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

Each configuration is evaluated across deterministic robustness scenarios:
    1. A standard raceline launch to estimate lap pace
    2. A left-offset recovery launch
    3. A right-offset recovery launch
    4. A single planner-style shifted raceline
    5. A double-shift planner-style raceline

Phase 2 promotes an initial seed, then Phases 4-8 repeatedly refine the
current global best.
"""

import subprocess
import os
import sys
import csv
import math
import atexit
import itertools
import time
import random
import hashlib
import shutil
import tempfile
import multiprocessing
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED

# ==============================================================================
# PATHS
# ==============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
TRAJ_DIR = os.path.join(os.path.dirname(PROJECT_DIR), "f1tenth_planning", "trajectories")

HORIZON_SWEEP_VALUES = [10, 12, 14, 16, 18, 20, 24, 28, 32, 36]
HORIZON_LIMIT = 40

# ==============================================================================
# HARDWARE MAP CONFIGURATION
# ==============================================================================

DEFAULT_RACELINE_NAME = "my_track_raceline.csv"
RACELINE_PATH = os.path.join(TRAJ_DIR, DEFAULT_RACELINE_NAME)
RACELINE_TAG = "my_track"
WALL_MARGIN = 0.20

# Base configuration - starting point for all sweeps
BASE_CONFIG = {
    "Q_LAT":        9000.0,
    "Q_HDG":        900.0,
    "Q_VEL":        220.0,
    "Q_LAT_VEL":    10.0,
    "Q_YAW":        4.5,
    "R_STEER":      0.55,
    "R_ACCEL":      0.012,
    "W_JERK":       0.16,
    "W_ACCEL_RATE": 0.14,
    "RHO":          40.0,
    "RHO_U":        24.0,
    "ALPHA":        1.10,
    "TOL":          5.0,
    "MAX_ITER":     20,
    "WALL_END":     12,
    "WALL_STRIDE":  1,
    "WALL_MARGIN":  0.20,
    "HORIZON":      12,
    "PRED_DT":      0.04,
}

# Override base for fastest objective
FASTEST_BASE_OVERRIDES = {
    "Q_LAT": 2200.0,
    "Q_HDG": 480.0,
    "Q_VEL": 620.0,
    "Q_LAT_VEL": 12.0,
    "Q_YAW": 4.5,
    "R_STEER": 0.75,
    "R_ACCEL": 0.012,
    "W_JERK": 0.16,
    "W_ACCEL_RATE": 0.14,
    "HORIZON": 12,
    "PRED_DT": 0.04,
}

# ==============================================================================
# SWEEP VALUE RANGES - PHASE 2 (Primary Grid)
# ==============================================================================

PHASE2_VALUES = {
    "Q_LAT": [1200, 1800, 2600, 3600, 5000, 7000, 9500, 12500, 16000],
    "Q_HDG": [80, 140, 220, 320, 460, 640, 860, 1120, 1500],
    "Q_VEL": [100, 140, 180, 240, 320, 420, 540, 700, 900],
    "HORIZON": [10, 12, 14, 16, 18, 20, 24, 28, 30, 40, 50],
    "PRED_DT": [0.032, 0.034, 0.036, 0.038, 0.04, 0.042, 0.045, 0.05],
}

PHASE2_VALUES_FASTEST = {
    # Feasibility-guided fastest basin from previous runs:
    # higher lateral/heading authority + shorter prediction dt around N=14.
    "Q_LAT": [6000, 8000, 10000, 12000, 14000, 16000],
    "Q_HDG": [500, 700, 850, 950, 1100, 1300],
    "Q_VEL": [420, 520, 620, 760, 900],
    "HORIZON": [12, 14, 16, 18, 20],
    "PRED_DT": [0.032, 0.034, 0.036, 0.038],
}

# ==============================================================================
# SWEEP VALUE RANGES - ALL PARAMETERS (for one-at-a-time and fine-tuning)
# ==============================================================================

FULL_SWEEP_VALUES = {
    "Q_LAT":        [1000, 1400, 2000, 2800, 3800, 5200, 7000, 9200, 12000, 15500],
    "Q_HDG":        [60, 100, 160, 240, 340, 480, 680, 920, 1250, 1600],
    "Q_VEL":        [100, 140, 180, 240, 320, 420, 540, 700, 900],
    "Q_LAT_VEL":    [2, 4, 6, 8, 10, 12, 16, 20],
    "Q_YAW":        [0.5, 1, 2, 3, 4.5, 6, 8],
    "R_STEER":      [0.30, 0.40, 0.52, 0.65, 0.75, 0.90, 1.05],
    "R_ACCEL":      [0.006, 0.008, 0.01, 0.012, 0.014, 0.016],
    "W_JERK":       [0.02, 0.04, 0.06, 0.08, 0.10, 0.14, 0.20],
    "W_ACCEL_RATE": [0.05, 0.07, 0.09, 0.11, 0.14, 0.18],
    "HORIZON":      [10, 12, 14, 16, 18, 20, 24],
    "RHO":          [20, 28, 36, 44, 52, 60],
    "RHO_U":        [10, 14, 18, 24, 30],
    "ALPHA":        [0.9, 1.0, 1.1, 1.2, 1.3],
    "PRED_DT":      [0.032, 0.034, 0.036, 0.038, 0.04, 0.042, 0.045, 0.05],
    "TOL":          [3.5, 4.0, 4.5, 5.0, 5.5],
}

FULL_SWEEP_VALUES_FASTEST = {
    "Q_LAT":        [700, 900, 1200, 1600, 2200, 3000, 4200, 6000, 8500, 12000],
    "Q_HDG":        [60, 90, 140, 200, 280, 380, 520, 720, 980, 1300, 1600],
    "Q_VEL":        [120, 160, 200, 240, 300, 380, 480, 620, 760, 900],
    "Q_LAT_VEL":    [2, 4, 6, 8, 10, 12, 16, 20],
    "Q_YAW":        [0.5, 1, 2, 3, 4.5, 6, 8],
    "R_STEER":      [0.30, 0.40, 0.52, 0.65, 0.75, 0.90, 1.05],
    "R_ACCEL":      [0.006, 0.008, 0.01, 0.012, 0.014, 0.016],
    "W_JERK":       [0.02, 0.04, 0.06, 0.08, 0.10, 0.14, 0.20],
    "W_ACCEL_RATE": [0.05, 0.07, 0.09, 0.11, 0.14, 0.18],
    "HORIZON":      [10, 12, 14, 16, 18, 20, 24],
    "RHO":          [20, 28, 36, 44, 52, 60],
    "RHO_U":        [10, 14, 18, 24, 30],
    "ALPHA":        [0.9, 1.0, 1.1, 1.2, 1.3],
    "PRED_DT":      [0.032, 0.034, 0.036, 0.038, 0.04, 0.042, 0.045, 0.05],
    "TOL":          [3.5, 4.0, 4.5, 5.0, 5.5],
}

# ==============================================================================
# PHASE 4: Secondary Grid Values (~2000 configs)
# Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE
# ==============================================================================

PHASE4_VALUES = {
    "Q_LAT_VEL":    [2, 4, 6, 8, 10, 12, 16, 20],
    "Q_YAW":        [0.5, 1, 2, 3, 4.5, 6, 8],
    "R_STEER":      [0.30, 0.40, 0.52, 0.65, 0.75, 0.90, 1.05],
    "W_JERK":       [0.02, 0.04, 0.06, 0.08, 0.10, 0.14, 0.20],
    "R_ACCEL":      [0.008, 0.01, 0.012, 0.014, 0.016],
    "W_ACCEL_RATE": [0.06, 0.08, 0.10, 0.12, 0.14, 0.18],
}

PHASE4_VALUES_FASTEST = {
    "Q_LAT_VEL":    [2, 4, 6, 8],
    "Q_YAW":        [3, 4.5, 6],
    "R_STEER":      [0.65, 0.75, 0.90, 1.05],
    "W_JERK":       [0.04, 0.08, 0.14, 0.20],
    "R_ACCEL":      [0.008, 0.01, 0.012, 0.014],
    "W_ACCEL_RATE": [0.08, 0.10, 0.12, 0.14],
}

# ==============================================================================
# PHASE 5: Solver Grid Values (~2000 configs)
# RHO x RHO_U x ALPHA x TOL
# ==============================================================================

PHASE5_VALUES = {
    "RHO":      [20, 28, 36, 44, 52, 60],
    "RHO_U":    [10, 14, 18, 24, 30],
    "ALPHA":    [0.9, 1.0, 1.1, 1.2, 1.3],
    "TOL":      [3.5, 4.0, 4.5, 5.0, 5.5],
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
            "PRED_DT": [0.04, 0.045, 0.05, 0.055, 0.06],
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
            "PRED_DT": [0.04, 0.045, 0.05, 0.055, 0.06],
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
            "PRED_DT": [0.04, 0.045, 0.05, 0.055],
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
            "PRED_DT": [0.04, 0.045, 0.05, 0.055],
            "ALPHA": [0.93, 1.0, 1.1, 1.2],
        },
    },
}

# ==============================================================================
# CONSTANTS
# ==============================================================================

INT_PARAMS = {"HORIZON", "WALL_END", "WALL_STRIDE", "MAX_ITER"}

SCENARIO_VEHICLE_HALF_WIDTH = 0.137
SCENARIO_BODY_SAFETY_MARGIN = 0.03
MAX_OFFSET_STEP_M = 0.015
OFFSET_SMOOTHING_PASSES = 4
OFFSET_SMOOTHING_WINDOW = 4
MAX_HEADING_STEP_RAD = 0.30
P99_HEADING_STEP_RAD = 0.18
RACE_SCENARIO_DURATION = 75.0
RECOVERY_SCENARIO_DURATION = 20.0
OBSTACLE_SCENARIO_DURATION = 60.0
RECOVERY_START_SPEED = 0.0

SCENARIO_TARGET_START_X = 5.5
SCENARIO_TARGET_START_Y = 0.0
GLOBAL_START_SHIFT_X_M = 0.5
GLOBAL_START_SHIFT_Y_M = 1.0
MIN_RACE_PROGRESS_MPS = 0.55
MIN_OVERALL_PROGRESS_MPS = 0.45
PLANNER_CAR_WIDTH_M = 0.31
PLANNER_CLEARANCE_TOLERANCE_M = 0.10
PLANNER_PLANNING_TOLERANCE_SCALE = 2.0
PLANNER_MIN_WINDOW_M = 8.0
PLANNER_WINDOW_TIME_S = 2.0
PLANNER_WINDOW_LEAD_RATIO = 0.5
PLANNER_MAX_LATERAL_SHIFT_M = 0.8

DETERMINISTIC_OBSTACLE_PROFILES = {
    "avoid_single": {
        "objects": [
            {"s_fraction": 0.48, "lateral_offset": -0.10},
        ],
    },
    "avoid_double": {
        "objects": [
            {"s_fraction": 0.44, "lateral_offset": 0.10},
            {"s_fraction": 0.84, "lateral_offset": -0.12},
        ],
    },
}

TRACK_LENGTH_METERS = 0.0
RACELINE_START_LEFT_BOUND = 0.0
RACELINE_START_RIGHT_BOUND = 0.0
EVAL_SCENARIOS = []
GENERATED_RACELINE_DIR = None
SCENARIO_RACELINE_PATHS = {}

CASCADE_TOP_N = 4   # Top-N seeds promoted from Phase 2 into Phase 4
SEED = 42           # Fixed seed for reproducibility
GLOBAL_OPTIMIZATION_PASSES = 4  # Repeated refinement passes for Phases 5-8
INCLUDE_OBSTACLE_SCENARIOS = True
PHASE7_RANDOM_COUNT = {"tracker": 900, "fastest": 2000}
PHASE8_RANDOM_COUNT = {"tracker": 450, "fastest": 3000}
STRICT_FASTEST_PROMOTION = True
DIVERSITY_KEYS = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW", "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE", "HORIZON", "PRED_DT"]
DIVERSITY_MIN_DISTANCE = 0.10

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


def load_raceline_samples(path: str) -> list:
    """Load the raceline samples needed for planner-style geometry shifts."""
    samples = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#"):
                continue
            try:
                s = float(row[0])
                x = float(row[1])
                y = float(row[2])
                psi = float(row[3])
                kappa = float(row[4])
                vx = float(row[5])
                ax = float(row[6])
                left = float(row[7]) if len(row) > 7 else 5.0
                right = float(row[8]) if len(row) > 8 else 5.0
            except (ValueError, IndexError):
                continue

            samples.append({
                "s": s,
                "x": x,
                "y": y,
                "psi": psi,
                "kappa": kappa,
                "vx": vx,
                "ax": ax,
                "left": left,
                "right": right,
            })

    if not samples:
        raise RuntimeError(f"Could not parse raceline geometry from {path}")
    return samples


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def wrap_forward_distance(s_from: float, s_to: float, track_length: float) -> float:
    """Forward arc distance on a closed track."""
    if track_length <= 1e-9:
        return max(0.0, s_to - s_from)
    delta = math.fmod(s_to - s_from, track_length)
    if delta < 0.0:
        delta += track_length
    return delta


def closest_waypoint_by_s(samples: list, s_query: float) -> int:
    """Return the waypoint index whose arc-length is nearest to s_query."""
    return min(range(len(samples)), key=lambda idx: abs(samples[idx]["s"] - s_query))


def choose_pass_direction(opp_wp: dict, obstacle_offset: float) -> float:
    """Mirror the lateral planner's passing-side decision for a static obstacle."""
    inflated_tol = PLANNER_CLEARANCE_TOLERANCE_M * max(1.0, PLANNER_PLANNING_TOLERANCE_SCALE)
    clearance = PLANNER_CAR_WIDTH_M + inflated_tol

    left_limit = max(opp_wp["left"] - PLANNER_CAR_WIDTH_M / 2.0 - inflated_tol, 0.05)
    right_limit = max(opp_wp["right"] - PLANNER_CAR_WIDTH_M / 2.0 - inflated_tol, 0.05)

    needed_left = abs(obstacle_offset + clearance)
    needed_right = abs(obstacle_offset - clearance)
    left_feasible = left_limit >= needed_left
    right_feasible = right_limit >= needed_right

    if left_feasible and not right_feasible:
        return 1.0
    if right_feasible and not left_feasible:
        return -1.0
    if left_feasible and right_feasible:
        return -1.0 if obstacle_offset >= 0.0 else 1.0
    return 0.0


def compute_shift_magnitude(opp_wp: dict, obstacle_offset: float, pass_dir: float) -> float:
    """Mirror the planner's shift magnitude calculation for one obstacle."""
    if pass_dir == 0.0:
        return 0.0

    inflated_tol = PLANNER_CLEARANCE_TOLERANCE_M * max(1.0, PLANNER_PLANNING_TOLERANCE_SCALE)
    wall_margin = PLANNER_CLEARANCE_TOLERANCE_M
    clearance = PLANNER_CAR_WIDTH_M + inflated_tol
    target_lateral = obstacle_offset + pass_dir * clearance
    required_shift = abs(target_lateral)

    left_limit = max(opp_wp["left"] - PLANNER_CAR_WIDTH_M / 2.0 - wall_margin, 0.05)
    right_limit = max(opp_wp["right"] - PLANNER_CAR_WIDTH_M / 2.0 - wall_margin, 0.05)
    directional_limit = left_limit if pass_dir >= 0.0 else right_limit

    return min(required_shift, abs(PLANNER_MAX_LATERAL_SHIFT_M), directional_limit)


def smooth_circular_offsets(offsets: list, passes: int = OFFSET_SMOOTHING_PASSES,
                            window: int = OFFSET_SMOOTHING_WINDOW) -> list:
    """Circular moving-average smoothing to prevent rugged heading artifacts."""
    if not offsets:
        return offsets
    n = len(offsets)
    out = list(offsets)
    w = max(1, int(window))
    for _ in range(max(1, int(passes))):
        prev = out[:]
        for i in range(n):
            acc = 0.0
            cnt = 0
            for k in range(-w, w + 1):
                acc += prev[(i + k) % n]
                cnt += 1
            out[i] = acc / max(cnt, 1)
    return out


def limit_offset_step(offsets: list, max_step: float = MAX_OFFSET_STEP_M) -> list:
    """Constrain adjacent offset deltas so heading does not become too rugged."""
    if not offsets:
        return offsets
    out = list(offsets)
    step = max(1e-6, float(max_step))
    for i in range(1, len(out)):
        lo = out[i - 1] - step
        hi = out[i - 1] + step
        out[i] = min(hi, max(lo, out[i]))
    for i in range(len(out) - 2, -1, -1):
        lo = out[i + 1] - step
        hi = out[i + 1] + step
        out[i] = min(hi, max(lo, out[i]))
    return out


def build_shifted_raceline_samples(base_samples: list, objects: list) -> list:
    """Build planner-style shifted raceline while enforcing wall and heading legality."""
    shifted = [dict(sample) for sample in base_samples]
    if not shifted:
        return shifted

    track_length = max(shifted[-1]["s"] - shifted[0]["s"], 0.0)
    accumulated_offsets = [0.0 for _ in shifted]
    min_wall_clearance = SCENARIO_VEHICLE_HALF_WIDTH + SCENARIO_BODY_SAFETY_MARGIN

    def clamp_offsets(offsets: list) -> list:
        out = []
        for idx, sample in enumerate(base_samples):
            max_left = max(sample["left"] - min_wall_clearance, 0.0)
            max_right = max(sample["right"] - min_wall_clearance, 0.0)
            out.append(max(-max_right, min(max_left, offsets[idx])))
        return out

    def materialize_from_offsets(offsets: list) -> list:
        out = [dict(sample) for sample in base_samples]
        for idx, sample in enumerate(base_samples):
            offset = offsets[idx]
            normal = sample["psi"] + math.pi / 2.0
            out[idx]["x"] = sample["x"] + offset * math.cos(normal)
            out[idx]["y"] = sample["y"] + offset * math.sin(normal)
            out[idx]["left"] = max(0.0, sample["left"] - offset)
            out[idx]["right"] = max(0.0, sample["right"] + offset)

        n = len(out)
        if n >= 3:
            for idx in range(n):
                prev_wp = out[(idx - 1) % n]
                curr_wp = out[idx]
                next_wp = out[(idx + 1) % n]

                dx = next_wp["x"] - prev_wp["x"]
                dy = next_wp["y"] - prev_wp["y"]
                if math.hypot(dx, dy) > 1e-9:
                    curr_wp["psi"] = math.atan2(dy, dx)

                psi_prev = math.atan2(curr_wp["y"] - prev_wp["y"], curr_wp["x"] - prev_wp["x"])
                psi_next = math.atan2(next_wp["y"] - curr_wp["y"], next_wp["x"] - curr_wp["x"])
                dpsi = wrap_angle(psi_next - psi_prev)
                ds_prev = math.hypot(curr_wp["x"] - prev_wp["x"], curr_wp["y"] - prev_wp["y"])
                ds_next = math.hypot(next_wp["x"] - curr_wp["x"], next_wp["y"] - curr_wp["y"])
                ds = max(0.5 * (ds_prev + ds_next), 1e-3)
                curr_wp["kappa"] = dpsi / ds
        return out

    def heading_is_legal(samples: list) -> bool:
        if len(samples) < 3:
            return True
        dpsi_vals = []
        for i in range(len(samples)):
            a = float(samples[i - 1]["psi"])
            b = float(samples[i]["psi"])
            dpsi_vals.append(abs(wrap_angle(b - a)))
        dpsi_vals.sort()
        p99_idx = int(0.99 * (len(dpsi_vals) - 1))
        p99 = dpsi_vals[p99_idx]
        return dpsi_vals[-1] <= MAX_HEADING_STEP_RAD and p99 <= P99_HEADING_STEP_RAD

    for obj in objects:
        target_s = shifted[0]["s"] + float(obj["s_fraction"]) * track_length
        obstacle_offset = float(obj["lateral_offset"])
        opp_idx = closest_waypoint_by_s(base_samples, target_s)
        opp_wp = base_samples[opp_idx]

        # Never place synthetic obstacles so close to walls that they force a
        # raceline violation of (half-car-width + margin).
        max_obs_left = max(opp_wp["left"] - min_wall_clearance, 0.0)
        max_obs_right = max(opp_wp["right"] - min_wall_clearance, 0.0)
        obstacle_offset = max(-max_obs_right, min(max_obs_left, obstacle_offset))

        pass_dir = choose_pass_direction(opp_wp, obstacle_offset)
        shift_mag = compute_shift_magnitude(opp_wp, obstacle_offset, pass_dir)
        if pass_dir == 0.0 or shift_mag <= 1e-6:
            continue

        window_dist = max(PLANNER_MIN_WINDOW_M, max(opp_wp["vx"], 1.0) * PLANNER_WINDOW_TIME_S)
        lead_ratio = min(max(PLANNER_WINDOW_LEAD_RATIO, 0.1), 0.9)
        lead_dist = max(0.75, window_dist * lead_ratio)
        trail_dist = max(0.75, window_dist * (1.0 - lead_ratio))
        window_len = lead_dist + trail_dist
        peak_offset = pass_dir * shift_mag
        s_start = opp_wp["s"] - lead_dist

        for idx, sample in enumerate(base_samples):
            s_rel = wrap_forward_distance(s_start, sample["s"], track_length)
            offset = 0.0
            if s_rel <= window_len:
                if s_rel <= lead_dist and lead_dist > 1e-9:
                    offset = peak_offset * 0.5 * (1.0 - math.cos(math.pi * s_rel / lead_dist))
                elif trail_dist > 1e-9:
                    trail_s = s_rel - lead_dist
                    offset = peak_offset * 0.5 * (1.0 + math.cos(math.pi * trail_s / trail_dist))
            accumulated_offsets[idx] += offset

    # Smooth and slope-limit offsets first.
    smoothed_offsets = smooth_circular_offsets(accumulated_offsets)
    smoothed_offsets = limit_offset_step(smoothed_offsets)

    # Enforce heading legality by reducing global shift scale if needed.
    legal_shifted = None
    for scale in (1.0, 0.85, 0.70, 0.55, 0.40, 0.25, 0.10):
        scaled = [v * scale for v in smoothed_offsets]
        scaled = clamp_offsets(scaled)
        candidate = materialize_from_offsets(scaled)
        if heading_is_legal(candidate):
            legal_shifted = candidate
            break

    if legal_shifted is None:
        # If no scaled candidate is smooth enough, keep baseline (legal fallback).
        return [dict(sample) for sample in base_samples]

    return legal_shifted


def write_raceline_samples(path: str, samples: list):
    """Write a minimal raceline CSV understood by the MPC simulator."""
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["# s", "x", "y", "psi", "kappa", "vx", "ax", "d_left", "d_right"])
        for sample in samples:
            writer.writerow([
                f"{sample['s']:.6f}",
                f"{sample['x']:.6f}",
                f"{sample['y']:.6f}",
                f"{sample['psi']:.6f}",
                f"{sample['kappa']:.6f}",
                f"{sample['vx']:.6f}",
                f"{sample['ax']:.6f}",
                f"{sample['left']:.6f}",
                f"{sample['right']:.6f}",
            ])


def cleanup_generated_racelines():
    """Remove the temporary directory holding generated raceline variants."""
    global GENERATED_RACELINE_DIR
    if GENERATED_RACELINE_DIR and os.path.isdir(GENERATED_RACELINE_DIR):
        shutil.rmtree(GENERATED_RACELINE_DIR, ignore_errors=True)
    GENERATED_RACELINE_DIR = None


def rotate_samples_to_start(samples: list, start_idx: int) -> list:
    """Rotate closed-loop samples so start_idx becomes index 0 and recompute arc-length s."""
    if not samples:
        return samples
    n = len(samples)
    start_idx = int(start_idx) % n
    rotated = [dict(samples[(start_idx + i) % n]) for i in range(n)]

    # Recompute s from geometry so first point is exactly s=0.
    rotated[0]["s"] = 0.0
    total = 0.0
    for i in range(1, n):
        dx = rotated[i]["x"] - rotated[i - 1]["x"]
        dy = rotated[i]["y"] - rotated[i - 1]["y"]
        total += math.hypot(dx, dy)
        rotated[i]["s"] = total
    return rotated


def translate_samples(samples: list, dx: float, dy: float) -> list:
    """Translate all waypoint positions by a constant offset in map frame."""
    out = [dict(s) for s in samples]
    for wp in out:
        wp["x"] += dx
        wp["y"] += dy
    return out


def align_samples_to_target_start(samples: list,
                                  target_x: float = 5.5,
                                  target_y: float = 0.0) -> list:
    """Rotate to nearest target point, then translate so first point is exactly at target."""
    if not samples:
        return samples
    idx = min(range(len(samples)), key=lambda i: math.hypot(samples[i]["x"] - target_x, samples[i]["y"] - target_y))
    rotated = rotate_samples_to_start(samples, idx)
    dx = target_x - rotated[0]["x"]
    dy = target_y - rotated[0]["y"]
    return translate_samples(rotated, dx, dy)


def build_scenario_raceline_paths(base_path: str) -> dict:
    """Generate deterministic shifted-raceline CSVs aligned to common start target."""
    global GENERATED_RACELINE_DIR
    cleanup_generated_racelines()
    GENERATED_RACELINE_DIR = tempfile.mkdtemp(prefix="mpc_tuning_racelines_", dir=SCRIPT_DIR)

    base_samples_raw = load_raceline_samples(base_path)
    base_samples = align_samples_to_target_start(base_samples_raw)

    base_out = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_base.csv")
    write_raceline_samples(base_out, base_samples)

    paths = {"base": os.path.abspath(base_out)}
    for profile_name, profile in DETERMINISTIC_OBSTACLE_PROFILES.items():
        shifted_samples = build_shifted_raceline_samples(base_samples, profile["objects"])
        # Keep a common exact start point across all scenario racelines.
        if shifted_samples:
            dx = SCENARIO_TARGET_START_X - shifted_samples[0]["x"]
            dy = SCENARIO_TARGET_START_Y - shifted_samples[0]["y"]
            shifted_samples = translate_samples(shifted_samples, dx, dy)
        out_path = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_{profile_name}.csv")
        write_raceline_samples(out_path, shifted_samples)
        paths[profile_name] = os.path.abspath(out_path)
    return paths


atexit.register(cleanup_generated_racelines)


def compute_recovery_offset(bound: float) -> float:
    """Choose a moderate off-raceline start offset that stays inside the corridor."""
    usable = float(bound) - SCENARIO_VEHICLE_HALF_WIDTH - SCENARIO_BODY_SAFETY_MARGIN
    if usable <= 0.0:
        return 0.0
    return round(min(0.35, 0.45 * usable), 4)


def compute_shifted_spawn_offset(raceline_path: str, preferred_sign: float,
                                 min_shift: float = 0.03, max_shift: float = 0.09) -> float:
    """Return a legal non-zero spawn offset for the given raceline start corridor."""
    meta = load_raceline_metadata(raceline_path)
    left_usable = max(meta["start_left_bound"] - SCENARIO_VEHICLE_HALF_WIDTH - SCENARIO_BODY_SAFETY_MARGIN, 0.0)
    right_usable = max(meta["start_right_bound"] - SCENARIO_VEHICLE_HALF_WIDTH - SCENARIO_BODY_SAFETY_MARGIN, 0.0)
    nominal = min(max_shift, max(min_shift, 0.25 * max(min(left_usable, right_usable), 0.0)))

    if preferred_sign >= 0.0:
        if left_usable >= nominal:
            return nominal
        if right_usable >= nominal:
            return -nominal
    else:
        if right_usable >= nominal:
            return -nominal
        if left_usable >= nominal:
            return nominal

    # Fallback: largest feasible non-zero offset on either side.
    if left_usable > 1e-4 or right_usable > 1e-4:
        if left_usable >= right_usable:
            return min(left_usable, max_shift)
        return -min(right_usable, max_shift)
    return 0.0


def build_eval_scenarios(include_obstacles: bool = INCLUDE_OBSTACLE_SCENARIOS) -> list:
    """Build deterministic evaluation scenarios for race pace and recovery robustness."""
    base_path = SCENARIO_RACELINE_PATHS.get("base", RACELINE_PATH)

    scenarios = [
        {
            "name": "race",
            "weight": 0.70 if not include_obstacles else 0.40,
            "seed_offset": 0,
            "raceline_path": base_path,
            "env": {
                "SIM_DURATION": f"{RACE_SCENARIO_DURATION}",
                "START_OFFSET_LAT": "0.0",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": "0.0",
                "START_OFFSET_X": f"{GLOBAL_START_SHIFT_X_M}",
                "START_OFFSET_Y": f"{GLOBAL_START_SHIFT_Y_M}",
            },
        },
        {
            "name": "recover_left",
            "weight": 0.15,
            "seed_offset": 101,
            "raceline_path": base_path,
            "env": {
                "SIM_DURATION": f"{RECOVERY_SCENARIO_DURATION}",
                "START_OFFSET_LAT": "0.0",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": f"{RECOVERY_START_SPEED}",
                "START_OFFSET_X": f"{GLOBAL_START_SHIFT_X_M}",
                "START_OFFSET_Y": f"{GLOBAL_START_SHIFT_Y_M}",
            },
        },
        {
            "name": "recover_right",
            "weight": 0.15,
            "seed_offset": 202,
            "raceline_path": base_path,
            "env": {
                "SIM_DURATION": f"{RECOVERY_SCENARIO_DURATION}",
                "START_OFFSET_LAT": "0.0",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": f"{RECOVERY_START_SPEED}",
                "START_OFFSET_X": f"{GLOBAL_START_SHIFT_X_M}",
                "START_OFFSET_Y": f"{GLOBAL_START_SHIFT_Y_M}",
            },
        },
    ]

    if include_obstacles:
        avoid_single_path = SCENARIO_RACELINE_PATHS.get("avoid_single", RACELINE_PATH)
        avoid_double_path = SCENARIO_RACELINE_PATHS.get("avoid_double", RACELINE_PATH)
        scenarios.extend([
            {
                "name": "avoid_single",
                "weight": 0.15,
                "seed_offset": 303,
                "raceline_path": avoid_single_path,
                "env": {
                    "SIM_DURATION": f"{OBSTACLE_SCENARIO_DURATION}",
                    "START_OFFSET_LAT": "0.0",
                    "START_HEADING_OFFSET": "0.0",
                    "START_SPEED": "0.0",
                    "START_OFFSET_X": f"{GLOBAL_START_SHIFT_X_M}",
                    "START_OFFSET_Y": f"{GLOBAL_START_SHIFT_Y_M}",
                },
            },
            {
                "name": "avoid_double",
                "weight": 0.15,
                "seed_offset": 404,
                "raceline_path": avoid_double_path,
                "env": {
                    "SIM_DURATION": f"{OBSTACLE_SCENARIO_DURATION}",
                    "START_OFFSET_LAT": "0.0",
                    "START_HEADING_OFFSET": "0.0",
                    "START_SPEED": "0.0",
                    "START_OFFSET_X": f"{GLOBAL_START_SHIFT_X_M}",
                    "START_OFFSET_Y": f"{GLOBAL_START_SHIFT_Y_M}",
                },
            },
        ])

    return scenarios



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
    if not scenario_results:
        return {
            "status": "NO_SCENARIOS",
            "return_code": -1,
            "scenario_count": 0,
            "scenario_failures": 1,
            "recovery_failures": 0,
            "main_failed": 1,
            "passed": 0,
            "failed": 6,
            "wall_collisions": 0,
            "completed_laps": 0,
            "time_above_5ms": 0.0,
            "max_lat_err": 0.0,
            "avg_lat_err": 0.0,
            "max_hdg_err": 0.0,
            "avg_hdg_err": 0.0,
            "max_vx": 0.0,
            "avg_vx": 0.0,
            "max_vel_err": 0.0,
            "avg_vel_err": 0.0,
            "avg_solve_us": 0.0,
            "max_solve_us": 0.0,
            "avg_iters": 0.0,
            "progress_m": 0.0,
            "avg_progress_mps": 0.0,
            "avg_lap_time": 0.0,
            "lap_time_est": 999.0,
            "max_steer_change": 0.0,
            "steer_reversals": 0.0,
        }

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
    env["RACELINE_PATH"] = os.path.abspath(scenario.get("raceline_path", RACELINE_PATH))
    
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


def run_test(params: dict, binary: str, seed: int = SEED, eval_scenarios: list = None) -> dict:
    """Run all evaluation scenarios and return a single aggregate result."""
    scenarios = eval_scenarios if eval_scenarios is not None else EVAL_SCENARIOS
    scenario_results = []
    for scenario in scenarios:
        result = run_single_scenario(params, binary, scenario, seed)
        scenario_results.append(result)
    return aggregate_scenario_results(scenario_results)


# ==============================================================================
# SCORING
# ==============================================================================

def is_safe_result(r: dict) -> bool:
    """True when aggregate run completed and had no wall collisions."""
    return r.get("status") == "OK" and int(r.get("wall_collisions", 999)) == 0


def has_full_scenario_coverage(r: dict) -> bool:
    """Require all configured scenarios to run (no early-stop partial rows)."""
    return int(r.get("scenario_count", 0)) >= len(EVAL_SCENARIOS)


def is_promotable_result(r: dict) -> bool:
    """Safe + full-scenario + sustained forward progress (frame-agnostic)."""
    return len(promotable_fail_reasons(r)) == 0


def promotable_deficit_score(r: dict) -> float:
    """Lower is better; 0 means fully promotable under progress criteria."""
    race_prog = float(r.get("scenario_race_avg_progress_mps", 0.0) or 0.0)
    overall_prog = float(r.get("avg_progress_mps", 0.0) or 0.0)
    race_def = max(0.0, MIN_RACE_PROGRESS_MPS - race_prog)
    overall_def = max(0.0, MIN_OVERALL_PROGRESS_MPS - overall_prog)
    # prioritize race progress first, then aggregate progress
    return race_def * 2.0 + overall_def


def promotable_fail_reasons(r: dict) -> list:
    """Human-readable reasons a result is not promotable."""
    reasons = []
    if r.get("status") != "OK":
        reasons.append("status_not_ok")
    if int(r.get("wall_collisions", 999)) != 0:
        reasons.append("wall_collision")
    if int(r.get("scenario_failures", 999)) != 0:
        reasons.append("scenario_failure")
    if not has_full_scenario_coverage(r):
        reasons.append("partial_scenarios")

    race_status = str(r.get("scenario_race_status", ""))
    race_prog = float(r.get("scenario_race_avg_progress_mps", 0.0) or 0.0)
    overall_prog = float(r.get("avg_progress_mps", 0.0) or 0.0)

    if race_status and race_status != "OK":
        reasons.append("race_not_ok")
    if race_prog < MIN_RACE_PROGRESS_MPS:
        reasons.append("race_progress_low")
    if overall_prog < MIN_OVERALL_PROGRESS_MPS:
        reasons.append("overall_progress_low")

    return reasons

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

    if not is_promotable_result(r):
        race_prog = float(r.get("scenario_race_avg_progress_mps", 0.0) or 0.0)
        overall_prog = float(r.get("avg_progress_mps", 0.0) or 0.0)
        return (
            1300.0
            + 300.0 * max(0.0, MIN_RACE_PROGRESS_MPS - race_prog)
            + 240.0 * max(0.0, MIN_OVERALL_PROGRESS_MPS - overall_prog)
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

    if not is_promotable_result(r):
        race_prog = float(r.get("scenario_race_avg_progress_mps", 0.0) or 0.0)
        overall_prog = float(r.get("avg_progress_mps", 0.0) or 0.0)
        return round(
            1200.0
            + 320.0 * max(0.0, MIN_RACE_PROGRESS_MPS - race_prog)
            + 260.0 * max(0.0, MIN_OVERALL_PROGRESS_MPS - overall_prog),
            6,
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
    """Attach scores and promotability diagnostics to a result row."""
    reasons = promotable_fail_reasons(r)
    r["promotable"] = 1 if not reasons else 0
    r["promotable_deficit"] = round(promotable_deficit_score(r), 6)
    r["promotable_reason"] = "|".join(reasons) if reasons else ""
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
    label, params, binary, phase_name, objective, eval_scenarios, raceline_tag = args
    r = run_test(params, binary, eval_scenarios=eval_scenarios)
    r = apply_scores(r, objective)
    r["label"] = label
    r["phase"] = phase_name
    r["raceline"] = raceline_tag
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
            elif not is_promotable_result(r):
                failed += 1
                print(f"not-promotable reason={r.get('promotable_reason','?')} rprog={r.get('scenario_race_avg_progress_mps', 0.0):.2f} aprog={r.get('avg_progress_mps', 0.0):.2f}  (ETA {eta:.0f}s)")
            else:
                passed += 1
                print(f"sc={r['score']:7.2f}  avx={r.get('avg_vx', 0.0):.2f}  "
                      f"lap={r.get('lap_time_est', 0.0):.2f}s  "
                      f"prog={r.get('avg_progress_mps', 0.0):.2f}  (ETA {eta:.0f}s)")
    else:
        # Parallel execution
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            scenario_bundle = [dict(s) for s in EVAL_SCENARIOS]
            raceline_tag = RACELINE_TAG
            it = ((label, params, binary, phase_name, objective, scenario_bundle, raceline_tag)
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
                        print(f"  [{done_count:4d}/{total}] {r['label']:55s} FAIL  (ETA {eta:.0f}s)")
                    elif not is_promotable_result(r):
                        failed += 1
                        print(f"  [{done_count:4d}/{total}] {r['label']:55s} "
                              f"not-promotable reason={r.get('promotable_reason','?')} rprog={r.get('scenario_race_avg_progress_mps', 0.0):.2f} aprog={r.get('avg_progress_mps', 0.0):.2f}  (ETA {eta:.0f}s)")
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



def normalized_param_distance(a: dict, b: dict) -> float:
    """Distance in normalized parameter space for diversity selection."""
    acc = 0.0
    cnt = 0
    for k in DIVERSITY_KEYS:
        av = float(a.get(k, BASE.get(k, 0.0)) or 0.0)
        bv = float(b.get(k, BASE.get(k, 0.0)) or 0.0)
        if k in ("Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW", "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE"):
            denom = max(abs(av), abs(bv), 1e-6)
            d = abs(av - bv) / denom
        elif k == "HORIZON":
            d = abs(av - bv) / float(max(HORIZON_LIMIT, 1))
        elif k == "PRED_DT":
            d = abs(av - bv) / 0.02
        else:
            denom = max(abs(av), abs(bv), 1e-6)
            d = abs(av - bv) / denom
        acc += d
        cnt += 1
    return (acc / cnt) if cnt > 0 else 0.0


def select_diverse_rows(rows: list, n: int) -> list:
    """Pick up to n rows with minimum pairwise distance threshold."""
    selected = []
    for r in rows:
        if all(normalized_param_distance(r, s) >= DIVERSITY_MIN_DISTANCE for s in selected):
            selected.append(r)
            if len(selected) >= n:
                return selected
    for r in rows:
        if r not in selected:
            selected.append(r)
            if len(selected) >= n:
                break
    return selected


def get_top_n_params(results: list, n: int = CASCADE_TOP_N, objective: str = "fastest") -> list:
    """Return list of up to N best params dicts."""
    promotable = [r for r in results if is_promotable_result(r)]
    pool = promotable

    if not pool:
        # First fallback: safe, full-coverage rows ordered by smallest progress deficit.
        near = [
            r for r in results
            if is_safe_result(r)
            and int(r.get("scenario_failures", 999)) == 0
            and has_full_scenario_coverage(r)
        ]
        if near:
            near.sort(key=lambda x: (
                promotable_deficit_score(x),
                x.get("score", 999999.0),
                -float(x.get("scenario_race_avg_progress_mps", 0.0) or 0.0),
                -float(x.get("avg_progress_mps", 0.0) or 0.0),
            ))
            pool = near
            print("  WARNING: No promotable candidates yet; using nearest-progress-safe configs.")
        else:
            # Last fallback: least-bad global rows.
            unsafe = sorted(results, key=lambda x: (
                0 if x.get("status") == "OK" else 1,
                x.get("wall_collisions", 999),
                x.get("score", 99999)
            ))
            if objective == "fastest" and STRICT_FASTEST_PROMOTION:
                print("  WARNING: No promotable/safe-full candidates; strict fastest promotion returning no seeds.")
                return []
            pool = unsafe[:n] if unsafe else []
            if pool:
                print("  WARNING: No safe candidates yet; using least-bad configs for cascade.")
    else:
        pool.sort(key=lambda x: x.get("score", 999999.0))
    
    # Deduplicate by params
    seen = set()
    unique_rows = []
    for r in pool:
        key = config_hash({k: r.get(k, BASE[k]) for k in BASE.keys()})
        if key not in seen:
            seen.add(key)
            unique_rows.append(r)

    picked = select_diverse_rows(unique_rows, n)

    if picked:
        for i, r in enumerate(picked):
            print(f"  Top-{i+1}: {r['label'][:50]} "
                  f"(score={r.get('score', 0.0):.2f}, "
                  f"lap={r.get('lap_time_est', 0.0):.2f}s, "
                  f"prog={r.get('avg_progress_mps', 0.0):.2f})")

    return [{k: r.get(k, BASE[k]) for k in BASE.keys()} for r in picked]


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
    global SCENARIO_RACELINE_PATHS
    global GLOBAL_START_SHIFT_X_M, GLOBAL_START_SHIFT_Y_M
    
    # Parse arguments
    num_workers = multiprocessing.cpu_count()  # Default to max workers
    objective = "fastest"
    raceline_override = None
    phase2_top_n = CASCADE_TOP_N
    global_passes = GLOBAL_OPTIMIZATION_PASSES
    include_obstacles = INCLUDE_OBSTACLE_SCENARIOS
    
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
        if arg == "--phase2-top" and i + 1 < len(sys.argv):
            try:
                phase2_top_n = max(1, int(sys.argv[i + 1]))
            except ValueError:
                print(f"WARNING: invalid --phase2-top value '{sys.argv[i + 1]}', using {CASCADE_TOP_N}")
                phase2_top_n = CASCADE_TOP_N
        if arg in ("--global-passes", "--refine-passes") and i + 1 < len(sys.argv):
            try:
                global_passes = max(1, int(sys.argv[i + 1]))
            except ValueError:
                print(f"WARNING: invalid {arg} value '{sys.argv[i + 1]}', using {GLOBAL_OPTIMIZATION_PASSES}")
                global_passes = GLOBAL_OPTIMIZATION_PASSES
        if arg == "--start-shift" and i + 1 < len(sys.argv):
            parts = sys.argv[i + 1].split(",")
            if len(parts) == 2:
                try:
                    GLOBAL_START_SHIFT_X_M = float(parts[0])
                    GLOBAL_START_SHIFT_Y_M = float(parts[1])
                except ValueError:
                    print(f"WARNING: invalid --start-shift '{sys.argv[i + 1]}', using defaults")
            else:
                print(f"WARNING: --start-shift expects x,y (e.g. '0.0,0.0'), using defaults")
        if arg == "--with-obstacles":
            include_obstacles = True
        if arg == "--no-obstacles":
            include_obstacles = False
    
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
    SCENARIO_RACELINE_PATHS = build_scenario_raceline_paths(RACELINE_PATH)
    EVAL_SCENARIOS = build_eval_scenarios(include_obstacles=include_obstacles)
    
    # Initialize BASE config
    BASE.update(BASE_CONFIG)
    if objective == "fastest":
        BASE.update(FASTEST_BASE_OVERRIDES)
    
    print(f"\n{'='*80}")
    print("MPC Weight Tuning - Hardware Map")
    print(f"{'='*80}")
    print(f"  Workers:     {num_workers}")
    print(f"  Objective:   {objective}")
    print(f"  Phase2->P4:  top {phase2_top_n}")
    print(f"  Global passes (P5-P8): {global_passes}")
    print(f"  Obstacles:   {'on' if include_obstacles else 'off'}")
    print(f"  Phase7 random: {PHASE7_RANDOM_COUNT.get(objective, 3600)}")
    print(f"  Phase8 random: {PHASE8_RANDOM_COUNT.get(objective, 1800)}")
    print(f"  Horizon sweep: {HORIZON_SWEEP_VALUES}")
    print(f"  Raceline:    {RACELINE_PATH}")
    print(f"  Raceline tag:{RACELINE_TAG}")
    print(f"  Track length:{TRACK_LENGTH_METERS:.3f} m")
    for scenario in EVAL_SCENARIOS:
        scenario_path = os.path.abspath(scenario.get("raceline_path", RACELINE_PATH))
        scenario_line = "base" if scenario_path == os.path.abspath(RACELINE_PATH) else os.path.basename(scenario_path)
        print(f"  Scenario {scenario['name']:<12s}"
              f" weight={scenario['weight']:.2f}"
              f" dur={float(scenario['env'].get('SIM_DURATION', 0.0)):>5.1f}s"
              f" lat={float(scenario['env'].get('START_OFFSET_LAT', 0.0)):>+5.2f}m"
              f" v0={float(scenario['env'].get('START_SPEED', 0.0)):>4.1f}m/s"
              f" line={scenario_line}")
    
    os.chdir(PROJECT_DIR)
    
    # Build binary
    binary_name = f"test_sim_drive_{os.getpid()}_{int(time.time())}"
    if os.name == "nt":
        binary_name += ".exe"
    binary = os.path.abspath(binary_name)
    
    print("\nBuilding test binary...")
    ret = subprocess.run([
        "gcc", "-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall",
        f"-DPREDICTION_HORIZON={HORIZON_LIMIT}",
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
         "promotable", "promotable_deficit", "promotable_reason",
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
    
    # Select top-N Phase 2 seeds, run Phase 4 from each, then refine global best.
    print("\n  Selecting Phase 2 seeds for Phase 4...")
    phase2_results = [r for r in results if str(r.get("phase", "")).startswith("Phase 2:")]
    phase2_seeds = get_top_n_params(phase2_results, n=phase2_top_n, objective=objective)
    if not phase2_seeds:
        phase2_seeds = [dict(BASE)]

    # ========== PHASES 3-8 ==========
    # Phase 3 is skipped for hardware map mode (fixed wall margin).
    print("\n  Phase 3: Skipped (wall margin is fixed for hardware)")

    # Phase 4: branch from top-N seeds from Phase 2.
    print("\n  Phase 4 branching from Phase 2 seeds...")
    for bi, seed_params in enumerate(phase2_seeds, start=1):
        update_base(seed_params)
        p, f = run_phase(f"Phase 4: Secondary grid [seed {bi}/{len(phase2_seeds)}]",
                         gen_secondary_grid(objective), binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f

    # Promote current global best after all branch sweeps.
    top_after_p4 = get_top_n_params(results, n=1, objective=objective)
    if top_after_p4:
        update_base(top_after_p4[0])

    # Global optimization loop: repeatedly refine one promoted-best candidate
    # through Phases 5-8, always handing off the best from each phase.
    current_best = get_top_n_params(results, n=1, objective=objective)
    current_best_params = current_best[0] if current_best else dict(BASE)

    for pi in range(global_passes):
        print(f"\n{'#'*80}")
        print(f"# GLOBAL OPTIMIZATION PASS {pi+1}/{global_passes}")
        print(f"{'#'*80}")

        # Phase 5: solver parameter sweep around current promoted best.
        update_base(current_best_params)
        phase_start = len(results)
        p, f = run_phase(f"Phase 5: Solver parameters [pass {pi+1}/{global_passes}]",
                         gen_solver_grid(), binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        phase_rows = results[phase_start:]
        top = get_top_n_params(phase_rows, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

        # Phase 6: fine tuning around current promoted best.
        update_base(current_best_params)
        phase_start = len(results)
        p, f = run_phase(f"Phase 6: Fine-tuning [pass {pi+1}/{global_passes}]",
                         gen_fine_tuning(current_best_params), binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        phase_rows = results[phase_start:]
        top = get_top_n_params(phase_rows, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

        # Phase 7: random exploration around current promoted best.
        update_base(current_best_params)
        n_random = PHASE7_RANDOM_COUNT.get(objective, 3600)
        phase_start = len(results)
        p, f = run_phase(f"Phase 7: Random neighbors ({n_random}) [pass {pi+1}/{global_passes}]",
                         gen_random_neighbors(current_best_params, n_random, objective,
                                              seed_offset=7000 + pi),
                         binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        phase_rows = results[phase_start:]
        top = get_top_n_params(phase_rows, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

        # Phase 8: random exploitation around promoted best from Phase 7.
        update_base(current_best_params)
        n_random = PHASE8_RANDOM_COUNT.get(objective, 1800)
        phase_start = len(results)
        p, f = run_phase(f"Phase 8: Random exploitation ({n_random}) [pass {pi+1}/{global_passes}]",
                         gen_random_neighbors(current_best_params, n_random, objective,
                                              profile_override=f"{objective}_exploit",
                                              seed_offset=9000 + pi),
                         binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        phase_rows = results[phase_start:]
        top = get_top_n_params(phase_rows, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

    # ========== FINAL RESULTS ==========
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
    safe = [r for r in results if is_promotable_result(r)]
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

