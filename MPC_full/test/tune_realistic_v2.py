#!/usr/bin/env python3
"""
Base MPC tuning for the hardware map.
=====================================
Sweeps MPC weights on the hardware map with fixed horizon and prediction dt
SLAM-mapped track (~22m, 0.27-1.4m wide).

Usage:
    python3 test/tune_realistic_v2.py                        # Full sweep (all CPUs)
    python3 test/tune_realistic_v2.py -j 0                   # Use all workers
    python3 test/tune_realistic_v2.py --seed-csv /path/to/results.csv
    python3 test/tune_realistic_v2.py --hardware-log /path/to/mpc_solver.csv \
        --hardware-meta /path/to/mpc_solver.csv.meta.txt     # Replay exact hardware scenario

The sweep runs 6 phases:
    Phase 1: One-at-a-time parameter sensitivity
    Phase 2: Primary grid (Q_LAT x Q_HDG x Q_VEL x Q_LAT_VEL x Q_YAW x R_STEER x MPC_W_DELTA_ACTUAL)
    Phase 4: Secondary grid (Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE)
    Phase 6: Fine-tuning around best config
    Phase 7: Random neighbor exploration
    Phase 8: Random exploitation around current best

Each configuration is evaluated across deterministic robustness scenarios:
    1. A standard raceline launch with off-raceline recovery start
    2. A single planner-style shifted raceline with off-raceline recovery start
    3. A double-shift planner-style raceline with off-raceline recovery start

Phase 2 promotes an initial seed, then Phases 4-8 repeatedly refine the
current global best.
"""

import subprocess
import os
import sys
import csv
import json
import math
import atexit
import itertools
import time
import random
import hashlib
import shutil
import tempfile
import multiprocessing
from pathlib import Path
from datetime import datetime
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED

try:
    from calibrate_plant_to_hardware import load_hardware_run, load_raceline
except ModuleNotFoundError:
    from MPC.test.calibrate_plant_to_hardware import load_hardware_run, load_raceline

# ==============================================================================
# PATHS
# ==============================================================================
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
REPO_ROOT = os.path.dirname(PROJECT_DIR)
TRAJ_DIR = os.path.join(PROJECT_DIR, "trajectories")

HORIZON_SWEEP_VALUES = [20]
HORIZON_LIMIT = 20
FIXED_PRED_DT = 0.03

# ==============================================================================
# HARDWARE MAP CONFIGURATION
# ==============================================================================

DEFAULT_RACELINE_NAME = "my_track_raceline.csv"
RACELINE_PATH = os.path.join(TRAJ_DIR, DEFAULT_RACELINE_NAME)
RACELINE_TAG = "my_track"
WALL_MARGIN = 0.14
DEFAULT_BASE_SEED_CSV = "/home/akselmo/Downloads/tuning_hardware_base_20260425_161330.csv"

# Base override seed
BASE_OVERRIDES = {
    # Fallback seed for MPC weights only (tuning parameters).
    # Replaced at runtime when --seed-csv is provided or when DEFAULT_BASE_SEED_CSV exists.
    # Vehicle parameters are NOT tuned and use simulator hardcoded defaults.
    "Q_LAT":  800.0,
    "Q_HDG": 28.8,
    "Q_VEL": 25.5,
    "Q_LAT_VEL": 0.9568,
    "Q_YAW": 1.5,
    "R_STEER": 1.5,
    "R_ACCEL": 0.01,
    "W_JERK": 0.04,
    "W_ACCEL_RATE": 0.10,
    "MPC_W_DELTA_ACTUAL": 0.02254,
    "HORIZON": 20,
    "PRED_DT": 0.03,
    "RHO": 7.0,
    "RHO_U": 20.0,   
    "TOL": 0.01,
    "MAX_ITER": 50,
}

HARDWARE_REPLAY_RUN = None
HARDWARE_REPLAY_WINDOW_SEC = 3.0

# ==============================================================================
# SWEEP VALUE RANGES - PHASE 2 (Primary Grid)
# ==============================================================================

PHASE2_VALUES_BASE = {
    # Centered on mpc_types.h current values ±25-30% for exploration
    "Q_LAT": [600.0, 700.0, 800.0, 900.0],
    "Q_HDG": [10, 15, 20, 25, 30],
    "Q_VEL": [20, 25, 30.0, 35, 40],
    "Q_LAT_VEL": [0.5, 0.75, 1.0, 1.25, 1.5],
    "Q_YAW": [1.0, 1.25, 1.50, 1.75, 2.0],
    "R_STEER": [1.0, 1.25, 1.50, 1.75, 2.0],
    "MPC_W_DELTA_ACTUAL": [0.015, 0.02, 0.025, 0.03],
}

# ==============================================================================
# SWEEP VALUE RANGES - ALL PARAMETERS (for one-at-a-time and fine-tuning)
# ==============================================================================

FULL_SWEEP_VALUES_BASE = {
    "Q_LAT": [80.0, 120.0, 160.0, 200.0, 240.0, 300.0, 380.0, 500.0, 700.0, 1000.0],
    "Q_HDG": [10.0, 12.0, 14.0, 16.0, 20.0, 24.0, 28.8, 34.0, 40.0, 48.0],
    "Q_VEL": [20.0, 24.0, 28.0, 30.0, 34.0, 38.0, 44.0],
    "Q_LAT_VEL": [0.50, 0.70, 0.90, 1.04, 1.20, 1.40, 1.70],
    "Q_YAW": [0.90, 1.10, 1.30, 1.50, 1.70, 1.90, 2.20],
    "R_STEER": [0.90, 1.10, 1.30, 1.50, 1.70, 1.90, 2.20],
    "R_ACCEL":      [0.006, 0.008, 0.010, 0.012, 0.015],
    "W_JERK":       [0.020, 0.030, 0.040, 0.050, 0.065],
    "W_ACCEL_RATE": [0.060, 0.080, 0.100, 0.120, 0.150],
    "MPC_W_DELTA_ACTUAL": [0.018, 0.020, 0.023, 0.026, 0.030],
    "HORIZON":      HORIZON_SWEEP_VALUES,
    "PRED_DT":      [0.03],
    "RHO":          [8.0],
    "RHO_U":        [24.0],
    "TOL":          [0.01],
}

# ==============================================================================
# PHASE 4: Secondary Grid Values (~18,000 configs)
# Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE
# ==============================================================================

PHASE4_VALUES_BASE = {
    # Centered on mpc_types.h current values ±25-30% for fine-tuning
    "Q_LAT_VEL":    [0.5, 1.0, 1.5, 2.0, 4.0],
    "Q_YAW":        [0.5, 1.0, 1.5, 2.0, 4.0],
    "R_STEER":      [0.5, 1.0, 1.5, 2.0, 4.0],
    "W_JERK":       [0.01, 0.02, 0.040, 0.06, 0.80],
    "R_ACCEL":      [0.005, 0.0075, 0.010, 0.02, 0.05],
    "W_ACCEL_RATE": [0.01, 0.05, 0.08, 0.120, 0.140],
    "MPC_W_DELTA_ACTUAL": [0.018, 0.020, 0.023, 0.026, 0.030],
}

# Keep these fixed for all sweeps and validations.
FIXED_HORIZON = 20
SOLVER_PAIR_PARAM = "RHO_RHO_U"

# ==============================================================================
# RANDOM NEIGHBOR PROFILES
# ==============================================================================

RANDOM_PROFILES = {
    "base": {
        "num_perturb_range": (4, 8),
        "default_multipliers": [0.85, 0.92, 0.97, 1.0, 1.03, 1.08, 1.15],
        "param_multipliers": {
            "Q_LAT": [0.80, 0.88, 0.94, 1.0, 1.06, 1.12, 1.20],
            "Q_HDG": [0.80, 0.88, 0.94, 1.0, 1.06, 1.12, 1.20],
            "Q_VEL": [0.85, 0.92, 0.97, 1.0, 1.03, 1.08, 1.15],
            "Q_LAT_VEL": [0.85, 0.92, 0.97, 1.0, 1.03, 1.08, 1.15],
            "Q_YAW": [0.85, 0.92, 0.97, 1.0, 1.03, 1.08, 1.15],
            "R_STEER": [0.90, 0.95, 0.98, 1.0, 1.02, 1.06, 1.10],
            "R_ACCEL": [0.90, 0.95, 0.98, 1.0, 1.02, 1.06, 1.10],
            "W_JERK": [0.90, 0.95, 0.98, 1.0, 1.02, 1.06, 1.10],
            "W_ACCEL_RATE": [0.90, 0.95, 0.98, 1.0, 1.02, 1.06, 1.10],
            "MPC_W_DELTA_ACTUAL": [0.90, 0.95, 0.98, 1.0, 1.02, 1.06, 1.10],
            SOLVER_PAIR_PARAM: [0.75, 0.80, 0.84, 0.88, 0.92, 0.95, 0.97, 0.99, 1.0, 1.01, 1.03, 1.05, 1.08, 1.12, 1.16, 1.22, 1.28],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [FIXED_PRED_DT],
        },
    },
    "base_exploit": {
        "num_perturb_range": (3, 5),
        "default_multipliers": [0.92, 0.96, 0.99, 1.0, 1.01, 1.04, 1.08],
        "param_multipliers": {
            "Q_LAT": [0.92, 0.96, 0.99, 1.0, 1.01, 1.04, 1.08],
            "Q_HDG": [0.92, 0.96, 0.99, 1.0, 1.01, 1.04, 1.08],
            "Q_VEL": [0.94, 0.97, 0.99, 1.0, 1.01, 1.03, 1.06],
            "Q_LAT_VEL": [0.92, 0.96, 0.99, 1.0, 1.01, 1.04, 1.08],
            "Q_YAW": [0.92, 0.96, 0.99, 1.0, 1.01, 1.04, 1.08],
            "R_STEER": [0.94, 0.97, 0.99, 1.0, 1.01, 1.03, 1.06],
            "R_ACCEL": [0.94, 0.97, 0.99, 1.0, 1.01, 1.03, 1.06],
            "W_JERK": [0.94, 0.97, 0.99, 1.0, 1.01, 1.03, 1.06],
            "W_ACCEL_RATE": [0.94, 0.97, 0.99, 1.0, 1.01, 1.03, 1.06],
            "MPC_W_DELTA_ACTUAL": [0.94, 0.97, 0.99, 1.0, 1.01, 1.03, 1.06],
            SOLVER_PAIR_PARAM: [0.88, 0.92, 0.95, 0.97, 0.99, 1.0, 1.01, 1.03, 1.05, 1.07, 1.10],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [FIXED_PRED_DT],
        },
    },
}

# ==============================================================================
# CONSTANTS
# ==============================================================================

INT_PARAMS = {"HORIZON", "MAX_ITER"}

SCENARIO_VEHICLE_HALF_WIDTH = 0.137
SCENARIO_BODY_SAFETY_MARGIN = 0.00
FIXED_WALL_MARGIN = 0.14
MAX_OFFSET_STEP_M = 0.015
MAX_HEADING_STEP_RAD = 0.30
P99_HEADING_STEP_RAD = 0.18
RACE_SCENARIO_DURATION = 75.0
OBSTACLE_SCENARIO_DURATION = 60.0

SCENARIO_TARGET_START_X = 0.0
SCENARIO_TARGET_START_Y = 0.0
GLOBAL_START_SHIFT_X_M = 0.0
GLOBAL_START_SHIFT_Y_M = 0.5
MIN_RACE_PROGRESS_MPS = 0.55
MIN_OVERALL_PROGRESS_MPS = 0.45
MIN_AVG_VX_MPS = 1.0
MIN_SOLVER_OPTIMAL_RATE = 0.0
MAX_SOLVER_MAX_ITER_RATE = 1.0
PLANNER_CAR_WIDTH_M = 0.27
PLANNER_CLEARANCE_TOLERANCE_M = 0.10
PLANNER_PLANNING_TOLERANCE_SCALE = 2.0
PLANNER_MIN_WINDOW_M = 8.0
PLANNER_WINDOW_TIME_S = 2.0
PLANNER_WINDOW_LEAD_RATIO = 0.5
PLANNER_MAX_LATERAL_SHIFT_M = 0.8
PLANNER_OPPONENT_LENGTH_M = 0.58
OBSTACLE_BOX_BACKOFF_M = 0.0
OBSTACLE_BOUND_INFLATION_M = 0.0
ENABLE_SCENARIO_AUDIT = True
BOUND_SPIKE_ABS_MAX_M = 0.20
BOUND_SPIKE_NEIGHBOR_MIN_M = 0.40

DETERMINISTIC_OBSTACLE_PROFILES = {
    "avoid_single": {
        "objects": [
            {"s_fraction": 0.90, "lateral_offset": 0.05},
        ],
    },
    "avoid_double": {
        "objects": [
            {"s_fraction": 0.23, "lateral_offset": -0.10},
            {"s_fraction": 0.65, "lateral_offset": 0.00},
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
GLOBAL_OPTIMIZATION_PASSES = 16  # Repeated refinement passes for Phases 5-8
INCLUDE_OBSTACLE_SCENARIOS = True
PHASE7_RANDOM_COUNT = 2000
PHASE8_RANDOM_COUNT = 2000
STRICT_PROMOTION = True
SOLVER_PARAM_KEYS = ("TOL",)
DIVERSITY_KEYS = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW", "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE", "MPC_W_DELTA_ACTUAL", "HORIZON", "PRED_DT"]
DIVERSITY_MIN_DISTANCE = 1.0

# Keep summary print order aligned with swept define order in mpc_types.h.
MPC_TYPES_PRINT_ORDER = (
    "Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
    "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE",
    "HORIZON", "PRED_DT", "MAX_ITER", "WALL_MARGIN",
    "TOL", "RHO", "RHO_U",
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


def resolve_project_path(path_arg: str) -> str:
    """Resolve repo-relative path to existing file, if possible."""
    if not path_arg:
        return path_arg
    if os.path.isabs(path_arg):
        return path_arg

    candidates = [
        os.path.join(REPO_ROOT, path_arg),
        os.path.join(PROJECT_DIR, path_arg),
        os.path.join(SCRIPT_DIR, path_arg),
        os.path.join(os.getcwd(), path_arg),
    ]
    for candidate in candidates:
        if os.path.exists(candidate):
            return os.path.abspath(candidate)

    return os.path.abspath(os.path.join(REPO_ROOT, path_arg))


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


def min_corridor_half_width(samples: list) -> float:
    """Return minimum of min(d_left, d_right) over all waypoints."""
    if not samples:
        return 0.0
    return min(
        min(float(wp.get("left", 0.0)), float(wp.get("right", 0.0)))
        for wp in samples
    )


def despike_wall_bounds(samples: list) -> tuple:
    """Replace isolated one-sample wall-bound dips with local median neighbor value."""
    if len(samples) < 3:
        return [dict(wp) for wp in samples], 0

    out = [dict(wp) for wp in samples]
    spikes_fixed = 0

    for key in ("left", "right"):
        vals = [float(wp.get(key, 0.0)) for wp in out]
        for i in range(1, len(out) - 1):
            prev_v = vals[i - 1]
            cur_v = vals[i]
            next_v = vals[i + 1]
            if (
                cur_v < BOUND_SPIKE_ABS_MAX_M
                and prev_v > BOUND_SPIKE_NEIGHBOR_MIN_M
                and next_v > BOUND_SPIKE_NEIGHBOR_MIN_M
            ):
                repaired = sorted((prev_v, cur_v, next_v))[1]
                out[i][key] = repaired
                vals[i] = repaired
                spikes_fixed += 1

    return out, spikes_fixed


def build_no_obstacle_base_path(base_path: str) -> str:
    """Use original raceline unless isolated wall-bound spikes are repaired into temp copy."""
    global GENERATED_RACELINE_DIR

    base_samples = load_raceline_samples(base_path)
    repaired_samples, spikes_fixed = despike_wall_bounds(base_samples)
    if spikes_fixed <= 0:
        return os.path.abspath(base_path)

    cleanup_generated_racelines()
    GENERATED_RACELINE_DIR = tempfile.mkdtemp(prefix="mpc_tuning_racelines_", dir=SCRIPT_DIR)
    out_path = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_base_despiked.csv")
    write_raceline_samples(out_path, repaired_samples)
    print(f"INFO: Repaired {spikes_fixed} isolated wall-bound spike(s) in temporary base raceline.")
    return os.path.abspath(out_path)


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


def choose_pass_direction(opp_wp: dict, obstacle_offset: float, committed_side: float = 0.0) -> float:
    """Mirror the lateral planner's passing-side decision for a static obstacle."""
    inflated_tol = PLANNER_CLEARANCE_TOLERANCE_M * max(1.0, PLANNER_PLANNING_TOLERANCE_SCALE)
    clearance = PLANNER_CAR_WIDTH_M + inflated_tol

    left_limit = max(opp_wp["left"] - PLANNER_CAR_WIDTH_M / 2.0 - inflated_tol, 0.05)
    right_limit = max(opp_wp["right"] - PLANNER_CAR_WIDTH_M / 2.0 - inflated_tol, 0.05)

    needed_left = abs(obstacle_offset + clearance)
    needed_right = abs(obstacle_offset - clearance)
    left_feasible = left_limit >= needed_left
    right_feasible = right_limit >= needed_right

    # Bias toward committed side when feasible, matching planner hysteresis.
    if committed_side > 0.0:
        if left_feasible:
            return 1.0
        if right_feasible:
            return -1.0
        return 0.0
    if committed_side < 0.0:
        if right_feasible:
            return -1.0
        if left_feasible:
            return 1.0
        return 0.0

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


def build_obstacle_boxes(base_samples: list, objects: list) -> list:
    """Place car-sized oriented obstacle boxes on the unedited baseline raceline."""
    if not base_samples or not objects:
        return []

    track_length = max(base_samples[-1]["s"] - base_samples[0]["s"], 0.0)
    boxes = []
    for obj in objects:
        target_s = base_samples[0]["s"] + float(obj["s_fraction"]) * track_length
        idx = closest_waypoint_by_s(base_samples, target_s)
        wp = base_samples[idx]
        normal = wp["psi"] + math.pi / 2.0
        lateral_offset = float(obj["lateral_offset"])
        boxes.append({
            "x": wp["x"] + lateral_offset * math.cos(normal),
            "y": wp["y"] + lateral_offset * math.sin(normal),
            "yaw": wp["psi"],
            "half_length": max(0.5 * PLANNER_OPPONENT_LENGTH_M, 0.05),
            "half_width": max(0.5 * PLANNER_CAR_WIDTH_M, 0.05),
        })
    return boxes


def min_signed_distance_to_box(samples: list, box: dict) -> float:
    """Minimum signed distance from samples to an oriented obstacle box.

    Negative means at least one sample lies inside the box footprint.
    """
    c = math.cos(box["yaw"])
    s = math.sin(box["yaw"])
    hl = box["half_length"]
    hw = box["half_width"]

    best = float("inf")
    for wp in samples:
        dx = float(wp["x"]) - box["x"]
        dy = float(wp["y"]) - box["y"]
        lx = dx * c + dy * s
        ly = -dx * s + dy * c

        ex = max(abs(lx) - hl, 0.0)
        ey = max(abs(ly) - hw, 0.0)
        outside = math.hypot(ex, ey)
        inside = max(abs(lx) - hl, abs(ly) - hw)
        signed = outside if inside > 0.0 else inside
        if signed < best:
            best = signed
    return best


def validate_obstacle_profile_feasibility(base_samples: list, objects: list, profile_name: str):
    """Reject deterministic obstacle profiles that planner logic cannot pass safely."""
    if not base_samples or not objects:
        return

    track_length = max(base_samples[-1]["s"] - base_samples[0]["s"], 0.0)
    min_clearance = SCENARIO_VEHICLE_HALF_WIDTH + SCENARIO_BODY_SAFETY_MARGIN
    half_obstacle_width = 0.5 * PLANNER_CAR_WIDTH_M
    issues = []

    for obj_idx, obj in enumerate(objects):
        s_fraction = float(obj["s_fraction"])
        obstacle_offset = float(obj["lateral_offset"])
        target_s = base_samples[0]["s"] + s_fraction * track_length
        opp_idx = closest_waypoint_by_s(base_samples, target_s)
        opp_wp = base_samples[opp_idx]

        # Ensure the obstacle box itself fits inside the map corridor.
        if obstacle_offset >= 0.0:
            if obstacle_offset + half_obstacle_width > opp_wp["left"]:
                issues.append(
                    f"obj#{obj_idx}: box exceeds left wall at s_fraction={s_fraction:.3f} "
                    f"(offset={obstacle_offset:.3f}, left={opp_wp['left']:.3f})"
                )
        else:
            if -obstacle_offset + half_obstacle_width > opp_wp["right"]:
                issues.append(
                    f"obj#{obj_idx}: box exceeds right wall at s_fraction={s_fraction:.3f} "
                    f"(offset={obstacle_offset:.3f}, right={opp_wp['right']:.3f})"
                )

        pass_dir = choose_pass_direction(opp_wp, obstacle_offset, 0.0)
        shift_mag = compute_shift_magnitude(opp_wp, obstacle_offset, pass_dir)
        if pass_dir == 0.0 or shift_mag <= 1e-6:
            issues.append(
                f"obj#{obj_idx}: no feasible pass side at s_fraction={s_fraction:.3f} "
                f"(offset={obstacle_offset:.3f})"
            )
            continue

        # Ensure shifted line still leaves at least car+body clearance on the
        # wall-constrained side used by the shift.
        if pass_dir >= 0.0:
            remaining = opp_wp["left"] - shift_mag
            side = "left"
        else:
            remaining = opp_wp["right"] - shift_mag
            side = "right"
        if remaining < min_clearance:
            issues.append(
                f"obj#{obj_idx}: {side} clearance too small after shift at s_fraction={s_fraction:.3f} "
                f"(remaining={remaining:.3f}, required>={min_clearance:.3f})"
            )

    if issues:
        details = "\n  - " + "\n  - ".join(issues)
        raise ValueError(f"Infeasible obstacle profile '{profile_name}':{details}")


def can_realize_obstacle_profile(base_samples: list, objects: list, profile_name: str) -> tuple[bool, list | None, str]:
    """Return whether a deterministic obstacle profile can build a valid shifted raceline."""
    try:
        validate_obstacle_profile_feasibility(base_samples, objects, profile_name)
        global ENABLE_SCENARIO_AUDIT
        prev_audit = ENABLE_SCENARIO_AUDIT
        ENABLE_SCENARIO_AUDIT = False
        try:
            shifted_samples = build_shifted_raceline_samples(base_samples, objects)
        finally:
            ENABLE_SCENARIO_AUDIT = prev_audit
        enforce_min_wall_clearance(shifted_samples, profile_name)
        return True, shifted_samples, ""
    except ValueError as exc:
        return False, None, str(exc)


def generate_obstacle_offset_candidates(wp: dict, preferred_offset: float) -> list:
    """Generate plausible obstacle lateral offsets at one waypoint."""
    half_obstacle_width = 0.5 * PLANNER_CAR_WIDTH_M
    max_left = max(float(wp["left"]) - half_obstacle_width - 1e-3, 0.0)
    max_right = max(float(wp["right"]) - half_obstacle_width - 1e-3, 0.0)

    if max_left <= 1e-6 and max_right <= 1e-6:
        return []

    offsets = []

    def clamp_offset(offset: float) -> float:
        return max(-max_right, min(max_left, float(offset)))

    def add(offset: float):
        clipped = clamp_offset(offset)
        for existing in offsets:
            if abs(existing - clipped) < 0.02:
                return
        offsets.append(clipped)

    preferred = clamp_offset(preferred_offset)
    preferred_sign = 1.0 if preferred_offset >= 0.0 else -1.0
    dominant_wall = max_left if preferred_sign >= 0.0 else max_right

    add(preferred)
    add(0.0)
    if dominant_wall > 1e-6:
        add(preferred_sign * 0.35 * dominant_wall)
        add(preferred_sign * 0.65 * dominant_wall)
        add(preferred_sign * 0.85 * dominant_wall)
    if max_left > 1e-6:
        add(0.50 * max_left)
        add(0.85 * max_left)
    if max_right > 1e-6:
        add(-0.50 * max_right)
        add(-0.85 * max_right)

    viable = []
    min_clearance = SCENARIO_VEHICLE_HALF_WIDTH + SCENARIO_BODY_SAFETY_MARGIN
    for offset in offsets:
        if offset >= 0.0:
            if offset + half_obstacle_width > wp["left"]:
                continue
        else:
            if -offset + half_obstacle_width > wp["right"]:
                continue

        pass_dir = choose_pass_direction(wp, offset, 0.0)
        shift_mag = compute_shift_magnitude(wp, offset, pass_dir)
        if pass_dir == 0.0 or shift_mag <= 1e-6:
            continue

        remaining = (wp["left"] - shift_mag) if pass_dir >= 0.0 else (wp["right"] - shift_mag)
        if remaining < min_clearance:
            continue

        viable.append(offset)

    return viable


def collect_feasible_obstacle_candidates(base_samples: list, preferred_offset: float) -> list:
    """Find waypoint/offset candidates where an obstacle can be placed and still admit a valid planner shift."""
    if not base_samples:
        return []

    track_length = max(base_samples[-1]["s"] - base_samples[0]["s"], 0.0)
    candidates = []

    for idx, wp in enumerate(base_samples):
        offset_candidates = generate_obstacle_offset_candidates(wp, preferred_offset)
        if not offset_candidates:
            continue

        corridor_width = float(wp["left"]) + float(wp["right"])
        curvature_penalty = abs(float(wp.get("kappa", 0.0)))
        s_fraction = ((float(wp["s"]) - float(base_samples[0]["s"])) / track_length) if track_length > 1e-9 else 0.0

        for obstacle_offset in offset_candidates:
            candidates.append({
                "idx": idx,
                "s": float(wp["s"]),
                "s_fraction": s_fraction,
                "lateral_offset": float(obstacle_offset),
                "left": float(wp["left"]),
                "right": float(wp["right"]),
                "corridor_width": corridor_width,
                "curvature_penalty": curvature_penalty,
                "offset_penalty": abs(float(obstacle_offset) - float(preferred_offset)),
            })

    return candidates


def resolve_obstacle_profile_objects(base_samples: list, profile_name: str, objects: list) -> tuple[list, list]:
    """Auto-place deterministic obstacles near requested s-fractions in feasible track regions."""
    if not objects:
        return objects, build_shifted_raceline_samples(base_samples, objects)

    track_length = max(base_samples[-1]["s"] - base_samples[0]["s"], 0.0)
    min_spacing = max(PLANNER_MIN_WINDOW_M * 0.8, 0.12 * track_length)
    candidate_lists = []
    desired_s_values = []
    for obj in objects:
        desired_s = float(base_samples[0]["s"]) + float(obj["s_fraction"]) * track_length
        desired_s_values.append(desired_s)
        preferred_offset = float(obj["lateral_offset"])
        candidates = collect_feasible_obstacle_candidates(base_samples, preferred_offset)
        if not candidates:
            raise ValueError(
                f"Scenario '{profile_name}' has no feasible obstacle placements for lateral_offset={preferred_offset:.3f}"
            )
        candidates.sort(
            key=lambda c: (
                abs(c["s"] - desired_s),
                c["offset_penalty"],
                c["curvature_penalty"],
                -c["corridor_width"],
            )
        )
        max_candidates = 100 if len(objects) <= 1 else 20
        candidate_lists.append(candidates[:max_candidates])

    best_objects = None
    best_shifted = None
    best_cost = None

    def candidate_cost(chosen: list) -> float:
        total = 0.0
        for desired_s, candidate in zip(desired_s_values, chosen):
            total += abs(candidate["s"] - desired_s)
            total += 100.0 * candidate["offset_penalty"]
            total -= 0.05 * candidate["corridor_width"]
            total += 0.1 * candidate["curvature_penalty"]
        return total

    def shifted_geometry_cost(samples: list) -> float:
        min_clear = float("inf")
        dpsi_vals = []
        for wp in samples:
            min_clear = min(min_clear, float(wp["left"]), float(wp["right"]))
        for i in range(1, len(samples)):
            dpsi_vals.append(abs(wrap_angle(float(samples[i]["psi"]) - float(samples[i - 1]["psi"]))))

        max_dpsi = max(dpsi_vals) if dpsi_vals else 0.0
        p99_dpsi = 0.0
        if dpsi_vals:
            dpsi_sorted = sorted(dpsi_vals)
            p99_dpsi = dpsi_sorted[int(0.99 * (len(dpsi_sorted) - 1))]

        required = 0.5 * PLANNER_CAR_WIDTH_M
        clearance_penalty = 20.0 * max(0.0, (required + 0.05) - min_clear)
        max_dpsi_penalty = 25.0 * max(0.0, max_dpsi - MAX_HEADING_STEP_RAD)
        p99_dpsi_penalty = 15.0 * max(0.0, p99_dpsi - P99_HEADING_STEP_RAD)
        return clearance_penalty + max_dpsi_penalty + p99_dpsi_penalty + 2.0 * max_dpsi + p99_dpsi

    def obstruction_cost(placed_objects: list, shifted_samples: list) -> float:
        boxes = build_obstacle_boxes(base_samples, placed_objects)
        if not boxes:
            return 0.0

        signed_distances = [min_signed_distance_to_box(base_samples, box) for box in boxes]
        max_box_gap = max(signed_distances)
        mean_box_gap = sum(signed_distances) / len(signed_distances)

        max_shift = 0.0
        for base_wp, shifted_wp in zip(base_samples, shifted_samples):
            normal = float(base_wp["psi"]) + math.pi / 2.0
            dx = float(shifted_wp["x"]) - float(base_wp["x"])
            dy = float(shifted_wp["y"]) - float(base_wp["y"])
            lat_shift = abs(dx * math.cos(normal) + dy * math.sin(normal))
            if lat_shift > max_shift:
                max_shift = lat_shift

        # Obstacles must actually obstruct the baseline raceline, not merely sit
        # beside it. Allow a small positive gap, but strongly penalize boxes that
        # remain farther away than ~0.15 m from the baseline line.
        target_gap = 0.15
        obstruction_penalty = 12.0 * max(0.0, max_box_gap - target_gap)
        obstruction_penalty += 6.0 * max(0.0, mean_box_gap - target_gap)

        # Also require the resulting shifted raceline to move meaningfully.
        min_meaningful_shift = 0.18
        shift_penalty = 10.0 * max(0.0, min_meaningful_shift - max_shift)
        return obstruction_penalty + shift_penalty

    def spacing_ok(chosen: list) -> bool:
        for i in range(len(chosen)):
            for j in range(i + 1, len(chosen)):
                if abs(chosen[i]["s"] - chosen[j]["s"]) < min_spacing:
                    return False
        return True

    candidate_combos = []
    if len(candidate_lists) == 1:
        candidate_combos = [[candidate] for candidate in candidate_lists[0]]
    else:
        for combo in itertools.product(*candidate_lists):
            combo_list = list(combo)
            if spacing_ok(combo_list):
                candidate_combos.append(combo_list)

    candidate_combos.sort(key=candidate_cost)
    validate_limit = len(candidate_combos)
    for chosen in candidate_combos[:validate_limit]:
        placed_objects = []
        for original, candidate in zip(objects, chosen):
            placed_objects.append({
                "s_fraction": candidate["s_fraction"],
                "lateral_offset": float(candidate["lateral_offset"]),
            })
        ok_local, shifted_local, _ = can_realize_obstacle_profile(base_samples, placed_objects, profile_name)
        if not ok_local:
            continue
        total_cost = candidate_cost(chosen)
        total_cost += shifted_geometry_cost(shifted_local)
        total_cost += obstruction_cost(placed_objects, shifted_local)
        if best_cost is None or total_cost < best_cost:
            best_cost = total_cost
            best_objects = placed_objects
            best_shifted = shifted_local
    if best_objects is None or best_shifted is None:
        ok, _, err = can_realize_obstacle_profile(base_samples, objects, profile_name)
        if ok:
            return objects, build_shifted_raceline_samples(base_samples, objects)
        raise ValueError(
            f"Scenario '{profile_name}' could not auto-place feasible obstacle locations. Last error: {err}"
        )

    return best_objects, best_shifted


def enforce_min_wall_clearance(samples: list, label: str):
    """Require every waypoint to keep at least half-car-width wall clearance."""
    if not samples:
        return

    required = 0.5 * PLANNER_CAR_WIDTH_M
    worst_idx = min(
        range(len(samples)),
        key=lambda i: min(float(samples[i].get("left", 0.0)), float(samples[i].get("right", 0.0))),
    )
    worst = samples[worst_idx]
    min_lr = min(float(worst.get("left", 0.0)), float(worst.get("right", 0.0)))
    if min_lr < required:
        raise ValueError(
            f"Scenario '{label}' violates half-width clearance: min(left,right)={min_lr:.3f} "
            f"at s={float(worst.get('s', 0.0)):.3f}, required>={required:.3f}"
        )


def clip_segment_to_box(start_x: float, start_y: float,
                        end_x: float, end_y: float,
                        box: dict,
                        backoff_m: float = OBSTACLE_BOX_BACKOFF_M) -> tuple:
    """Clip a wall-distance ray to an oriented obstacle box (Liang-Barsky in local frame)."""
    c = math.cos(box["yaw"])
    s = math.sin(box["yaw"])

    def to_local(wx: float, wy: float) -> tuple:
        dx = wx - box["x"]
        dy = wy - box["y"]
        lx = dx * c + dy * s
        ly = -dx * s + dy * c
        return lx, ly

    def to_world(lx: float, ly: float) -> tuple:
        wx = box["x"] + lx * c - ly * s
        wy = box["y"] + lx * s + ly * c
        return wx, wy

    x0, y0 = to_local(start_x, start_y)
    x1, y1 = to_local(end_x, end_y)
    dx = x1 - x0
    dy = y1 - y0

    t_min = 0.0
    t_max = 1.0

    def clip_axis(p0: float, d: float, min_v: float, max_v: float) -> bool:
        nonlocal t_min, t_max
        if abs(d) < 1e-9:
            return min_v <= p0 <= max_v
        t1 = (min_v - p0) / d
        t2 = (max_v - p0) / d
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        return t_min <= t_max

    half_len = box["half_length"]
    half_wid = box["half_width"]
    if not clip_axis(x0, dx, -half_len, half_len):
        return end_x, end_y
    if not clip_axis(y0, dy, -half_wid, half_wid):
        return end_x, end_y
    if t_max < 0.0 or t_min > 1.0:
        return end_x, end_y

    t_hit = min(max(t_min, 0.0), 1.0)
    if t_hit >= 1.0:
        return end_x, end_y

    seg_len = math.hypot(dx, dy)
    backoff_t = (backoff_m / seg_len) if seg_len > 1e-6 else 0.0
    t_clip = max(0.0, t_hit - backoff_t)
    clip_local_x = x0 + t_clip * dx
    clip_local_y = y0 + t_clip * dy
    return to_world(clip_local_x, clip_local_y)


def _cross2(ax: float, ay: float, bx: float, by: float) -> float:
    return ax * by - ay * bx


def ray_polyline_distance(px: float, py: float,
                          dx: float, dy: float,
                          polyline: list) -> float:
    """Distance from ray p+t*d (t>=0) to first hit on a closed polyline."""
    best_t = None
    n = len(polyline)
    if n < 2:
        return None

    for i in range(n):
        ax, ay = polyline[i]
        bx, by = polyline[(i + 1) % n]
        sx = bx - ax
        sy = by - ay
        den = _cross2(dx, dy, sx, sy)
        if abs(den) < 1e-12:
            continue

        qpx = ax - px
        qpy = ay - py
        t = _cross2(qpx, qpy, sx, sy) / den
        u = _cross2(qpx, qpy, dx, dy) / den
        if t >= 0.0 and 0.0 <= u <= 1.0:
            if best_t is None or t < best_t:
                best_t = t
    return best_t


def is_s_in_window(s: float, s_start: float, s_end: float, track_length: float) -> bool:
    """Check whether arc-length s lies inside wrapped window [s_start, s_end]."""
    if track_length <= 1e-9:
        return s_start <= s <= s_end
    window_len = wrap_forward_distance(s_start, s_end, track_length)
    return wrap_forward_distance(s_start, s, track_length) <= window_len


def apply_obstacle_boxes_to_bounds(samples: list,
                                   reference_samples: list,
                                   obstacle_boxes: list,
                                   obstacle_windows: list,
                                   track_length: float):
    """Edit d_left/d_right so obstacle boxes become part of the avoidable corridor."""
    if not samples or not obstacle_boxes:
        return

    for idx, wp in enumerate(samples):
        active_boxes = []
        active_windows_local = []
        for box, window in zip(obstacle_boxes, obstacle_windows):
            if is_s_in_window(wp["s"], window[0], window[1], track_length):
                active_boxes.append(box)
                active_windows_local.append(window)
        if not active_boxes:
            continue

        ref_wp = reference_samples[idx]
        ref_normal = ref_wp["psi"] + math.pi / 2.0
        local_offset = ((wp["x"] - ref_wp["x"]) * math.cos(ref_normal) +
                        (wp["y"] - ref_wp["y"]) * math.sin(ref_normal))
        if abs(local_offset) < 1e-6:
            continue

        normal = wp["psi"] + math.pi / 2.0
        nx = math.cos(normal)
        ny = math.sin(normal)

        left_d = max(float(wp.get("left", 0.0)), 0.0)
        right_d = max(float(wp.get("right", 0.0)), 0.0)

        sx, sy = wp["x"], wp["y"]
        lx, ly = sx + left_d * nx, sy + left_d * ny
        rx, ry = sx - right_d * nx, sy - right_d * ny

        if len(active_boxes) > 1:
            nearest_idx = min(
                range(len(active_boxes)),
                key=lambda j: abs(
                    wp["s"] - (
                        active_windows_local[j][0] +
                        0.5 * wrap_forward_distance(active_windows_local[j][0], active_windows_local[j][1], track_length)
                    )
                ),
            )
            active_boxes = [active_boxes[nearest_idx]]

        if local_offset > 0.0:
            for box in active_boxes:
                rx, ry = clip_segment_to_box(sx, sy, rx, ry, box)
        else:
            for box in active_boxes:
                lx, ly = clip_segment_to_box(sx, sy, lx, ly, box)

        wp["left"] = max(0.0, math.hypot(lx - sx, ly - sy))
        wp["right"] = max(0.0, math.hypot(rx - sx, ry - sy))


def limit_offset_step(offsets: list, max_step: float = MAX_OFFSET_STEP_M) -> list:
    """Constrain adjacent offset deltas so heading changes stay trackable."""
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


def limit_offset_step_masked(offsets: list, active_mask: list, max_step: float = MAX_OFFSET_STEP_M) -> list:
    """Limit offset slope inside active windows only, forcing inactive points to zero."""
    if not offsets:
        return offsets
    out = list(offsets)
    n = len(out)
    step = max(1e-6, float(max_step))

    i = 0
    while i < n:
        if not active_mask[i]:
            out[i] = 0.0
            i += 1
            continue

        start = i
        while i < n and active_mask[i]:
            i += 1
        end = i

        for j in range(start + 1, end):
            lo = out[j - 1] - step
            hi = out[j - 1] + step
            out[j] = min(hi, max(lo, out[j]))
        for j in range(end - 2, start - 1, -1):
            lo = out[j + 1] - step
            hi = out[j + 1] + step
            out[j] = min(hi, max(lo, out[j]))

    for j in range(n):
        if not active_mask[j]:
            out[j] = 0.0
    return out


def audit_shifted_raceline_locality(base_samples: list,
                                    shifted_samples: list,
                                    active_windows: list,
                                    track_length: float,
                                    label: str):
    """Audit that non-obstacle regions remain close to baseline geometry and bounds."""
    if not base_samples or not shifted_samples:
        return

    if len(base_samples) != len(shifted_samples):
        print(f"[AUDIT:{label}] skipped (size mismatch)")
        return

    halo_m = 1.0
    lat_abs = []
    wall_abs = []
    outside_count = 0

    def outside_windows(s_val: float) -> bool:
        for s_start, s_end in active_windows:
            # Expand each active window slightly to avoid edge false positives.
            w_start = s_start - halo_m
            w_end = s_end + halo_m
            if is_s_in_window(s_val, w_start, w_end, track_length):
                return False
        return True

    for base_wp, shift_wp in zip(base_samples, shifted_samples):
        if not outside_windows(shift_wp["s"]):
            continue
        outside_count += 1

        normal = base_wp["psi"] + math.pi / 2.0
        nx = math.cos(normal)
        ny = math.sin(normal)
        dx = shift_wp["x"] - base_wp["x"]
        dy = shift_wp["y"] - base_wp["y"]
        lat_abs.append(abs(dx * nx + dy * ny))

        d_left_delta = abs(float(shift_wp.get("left", 0.0)) - float(base_wp.get("left", 0.0)))
        d_right_delta = abs(float(shift_wp.get("right", 0.0)) - float(base_wp.get("right", 0.0)))
        wall_abs.append(max(d_left_delta, d_right_delta))

    if outside_count == 0:
        print(f"[AUDIT:{label}] skipped (no outside-window samples)")
        return

    lat_sorted = sorted(lat_abs)
    wall_sorted = sorted(wall_abs)
    p99_idx = int(0.99 * (outside_count - 1))
    max_lat = lat_sorted[-1]
    p99_lat = lat_sorted[p99_idx]
    max_wall = wall_sorted[-1]
    p99_wall = wall_sorted[p99_idx]

    pass_ok = (max_lat <= 0.03 and p99_lat <= 0.015 and max_wall <= 0.03 and p99_wall <= 0.015)
    print(
        f"[AUDIT:{label}] outside={outside_count} "
        f"max_lat={max_lat:.4f} p99_lat={p99_lat:.4f} "
        f"max_wall={max_wall:.4f} p99_wall={p99_wall:.4f} "
        f"status={'PASS' if pass_ok else 'WARN'}"
    )


def build_shifted_raceline_samples(base_samples: list, objects: list) -> list:
    """Build planner-style shifted raceline with the same half-cosine offset window."""
    shifted = [dict(sample) for sample in base_samples]
    if not shifted:
        return shifted

    track_length = max(shifted[-1]["s"] - shifted[0]["s"], 0.0)
    obstacle_boxes = build_obstacle_boxes(base_samples, objects)
    accumulated_offsets = [0.0 for _ in shifted]

    def materialize_from_offsets(offsets: list) -> list:
        out = [dict(sample) for sample in base_samples]
        for idx, sample in enumerate(base_samples):
            offset = offsets[idx]
            normal = sample["psi"] + math.pi / 2.0
            out[idx]["x"] = sample["x"] + offset * math.cos(normal)
            out[idx]["y"] = sample["y"] + offset * math.sin(normal)
            # Mirror the planner's corridor semantics in the raceline frame:
            # shifting the centerline by +offset toward the left wall consumes
            # left clearance and increases right clearance by the same amount.
            out[idx]["left"] = max(0.0, float(sample["left"]) - offset)
            out[idx]["right"] = max(0.0, float(sample["right"]) + offset)

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

    wall_limit = abs(PLANNER_MAX_LATERAL_SHIFT_M)

    active_boxes = []
    active_windows = []

    for obj_idx, obj in enumerate(objects):
        target_s = shifted[0]["s"] + float(obj["s_fraction"]) * track_length
        obstacle_offset = float(obj["lateral_offset"])
        opp_idx = closest_waypoint_by_s(base_samples, target_s)
        opp_wp = base_samples[opp_idx]

        window_dist = max(PLANNER_MIN_WINDOW_M, max(opp_wp["vx"], 0.0) * PLANNER_WINDOW_TIME_S)
        lead_ratio = min(max(PLANNER_WINDOW_LEAD_RATIO, 0.1), 0.9)
        lead_dist = max(0.75, window_dist * lead_ratio)
        trail_dist = max(0.75, window_dist * (1.0 - lead_ratio))

        # Mirror planner behavior near the opponent: keep a minimum lead window.
        car_to_opp = wrap_forward_distance(base_samples[0]["s"], opp_wp["s"], track_length)
        if car_to_opp < 0.2:
            lead_dist = max(lead_dist, 0.2)

        # Treat deterministic static boxes as independent local events.
        pass_dir = choose_pass_direction(opp_wp, obstacle_offset, 0.0)
        shift_mag = compute_shift_magnitude(opp_wp, obstacle_offset, pass_dir)
        if pass_dir == 0.0 or shift_mag <= 1e-6:
            continue

        s_start = opp_wp["s"] - lead_dist
        window_len = lead_dist + trail_dist
        s_end = opp_wp["s"] + trail_dist
        active_windows.append((s_start, s_end))
        if obj_idx < len(obstacle_boxes):
            active_boxes.append(obstacle_boxes[obj_idx])

        peak_offset = pass_dir * shift_mag

        for idx, sample in enumerate(base_samples):
            s_rel = wrap_forward_distance(s_start, sample["s"], track_length)
            offset = 0.0
            if s_rel <= window_len:
                if s_rel <= lead_dist and lead_dist > 1e-9:
                    offset = peak_offset * 0.5 * (1.0 - math.cos(math.pi * s_rel / lead_dist))
                elif trail_dist > 1e-9:
                    trail_s = s_rel - lead_dist
                    offset = peak_offset * 0.5 * (1.0 + math.cos(math.pi * trail_s / trail_dist))
                if abs(offset) > wall_limit:
                    offset = math.copysign(wall_limit, offset)
            # Deterministic multi-obstacle scenarios should emulate one active
            # opponent influence at a time (planner runtime behavior), so avoid
            # opposite-window cancellation by keeping the dominant local offset.
            if abs(offset) > abs(accumulated_offsets[idx]):
                accumulated_offsets[idx] = offset

    active_mask = []
    for sample in base_samples:
        is_active = False
        for s_start, s_end in active_windows:
            if is_s_in_window(sample["s"], s_start, s_end, track_length):
                is_active = True
                break
        active_mask.append(is_active)

    final_offsets = []
    min_corridor_clearance = SCENARIO_VEHICLE_HALF_WIDTH + SCENARIO_BODY_SAFETY_MARGIN
    for idx, sample in enumerate(base_samples):
        if not active_mask[idx]:
            final_offsets.append(0.0)
            continue
        max_left = max(float(sample["left"]) - min_corridor_clearance, 0.0)
        max_right = max(float(sample["right"]) - min_corridor_clearance, 0.0)
        final_offsets.append(max(-max_right, min(max_left, accumulated_offsets[idx])))

    candidate = materialize_from_offsets(final_offsets)
    apply_obstacle_boxes_to_bounds(candidate, base_samples, active_boxes, active_windows, track_length)
    if ENABLE_SCENARIO_AUDIT:
        audit_shifted_raceline_locality(base_samples, candidate, active_windows, track_length, "shifted")
    return candidate


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
    base_samples_raw, spikes_fixed = despike_wall_bounds(base_samples_raw)
    if spikes_fixed > 0:
        print(f"INFO: Repaired {spikes_fixed} isolated wall-bound spike(s) before obstacle scenario generation.")
    base_samples = align_samples_to_target_start(base_samples_raw)
    enforce_min_wall_clearance(base_samples, "base")

    base_out = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_base.csv")
    write_raceline_samples(base_out, base_samples)

    paths = {"base": os.path.abspath(base_out)}
    for profile_name, profile in DETERMINISTIC_OBSTACLE_PROFILES.items():
        try:
            resolved_objects, shifted_samples = resolve_obstacle_profile_objects(base_samples, profile_name, profile["objects"])
        except ValueError as exc:
            print(f"[SCENARIO:{profile_name}] skipped: {exc}")
            continue
        # Keep a common exact start point across all scenario racelines.
        if shifted_samples:
            dx = SCENARIO_TARGET_START_X - shifted_samples[0]["x"]
            dy = SCENARIO_TARGET_START_Y - shifted_samples[0]["y"]
            shifted_samples = translate_samples(shifted_samples, dx, dy)
        enforce_min_wall_clearance(shifted_samples, profile_name)
        out_path = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_{profile_name}.csv")
        write_raceline_samples(out_path, shifted_samples)
        paths[profile_name] = os.path.abspath(out_path)
        profile["resolved_objects"] = resolved_objects
    return paths


atexit.register(cleanup_generated_racelines)


def build_eval_scenarios(include_obstacles: bool = INCLUDE_OBSTACLE_SCENARIOS) -> list:
    """Build deterministic scenarios for weight tuning.

    Note: Keep these scenarios *controller-focused* (idealized startup), not
    hardware-debug focused. If you need to model hardware quirks (e.g. delayed
    servo feedback, control-rate limiting), do it in the hardware stack, not
    in this weight sweep.
    """
    if HARDWARE_REPLAY_RUN is not None:
        return build_hardware_replay_scenarios(HARDWARE_REPLAY_RUN)

    base_path = SCENARIO_RACELINE_PATHS.get("base", RACELINE_PATH)

    scenarios = [
        {
            "name": "race",
            "weight": 1.00 if not include_obstacles else 0.50,
            "seed_offset": 0,
            "raceline_path": base_path,
            "env": {
                "SIM_DURATION": f"{RACE_SCENARIO_DURATION}",
                # Idealized start: on-raceline, aligned heading.
                "START_OFFSET_LAT": "0.0",
                "START_HEADING_OFFSET": "0.0",
                "START_SPEED": "0.0",
                "START_INDEX": "0",
            },
        },
    ]

    if include_obstacles:
        if "avoid_single" in SCENARIO_RACELINE_PATHS:
            scenarios.append({
                "name": "avoid_single",
                "weight": 0.25,
                "seed_offset": 303,
                "raceline_path": SCENARIO_RACELINE_PATHS["avoid_single"],
                "env": {
                    "SIM_DURATION": f"{OBSTACLE_SCENARIO_DURATION}",
                    "START_OFFSET_LAT": "0.0",
                    "START_HEADING_OFFSET": "0.0",
                    "START_SPEED": "0.0",
                },
            })
        if "avoid_double" in SCENARIO_RACELINE_PATHS:
            scenarios.append({
                "name": "avoid_double",
                "weight": 0.25,
                "seed_offset": 404,
                "raceline_path": SCENARIO_RACELINE_PATHS["avoid_double"],
                "env": {
                    "SIM_DURATION": f"{OBSTACLE_SCENARIO_DURATION}",
                    "START_OFFSET_LAT": "0.0",
                    "START_HEADING_OFFSET": "0.0",
                    "START_SPEED": "0.0",
                },
            })

    return scenarios


def build_hardware_replay_scenarios(run: dict) -> list:
    """Evaluate against one real hardware segment with exact local-raceline replay."""
    sim_duration = min(run["elapsed_s"][-1], HARDWARE_REPLAY_WINDOW_SEC) if HARDWARE_REPLAY_WINDOW_SEC > 0.0 else run["elapsed_s"][-1]
    return [
        {
            "name": "hardware_replay",
            "weight": 1.0,
            "seed_offset": 0,
            "raceline_path": RACELINE_PATH,
            "env": {
                "SIM_DURATION": f"{sim_duration:.6f}",
                "REPLAY_LOCAL_RACELINE_LOG": str(run["local_raceline_log_path"]),
                "REPLAY_LOCAL_RACELINE_MODE": "progress" if run.get("local_raceline_index_path") else "time",
                "REPLAY_LOCAL_RACELINE_INDEX": str(run["local_raceline_index_path"]) if run.get("local_raceline_index_path") else "",
                "REPLAY_LOCAL_RACELINE_START_NS": str(run["start_local_raceline_ns"]),
                **run["start_env"],
            },
        },
    ]



def get_primary_grid_values(objective: str) -> dict:
    """Return the Phase 2 sweep for the active objective."""
    return PHASE2_VALUES_BASE


def get_full_sweep_values(objective: str) -> dict:
    """Return the broad sweep values for the active objective."""
    return FULL_SWEEP_VALUES_BASE


def get_secondary_grid_values(objective: str) -> dict:
    """Return the Phase 4 sweep for the active objective."""
    return PHASE4_VALUES_BASE


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

    out["HORIZON"] = FIXED_HORIZON
    out["PRED_DT"] = FIXED_PRED_DT
    
    # Integer params
    for k in INT_PARAMS:
        if k in out:
            out[k] = int(float(out[k]))
    
    return out


def is_valid_config(params: dict) -> bool:
    """Check if configuration is valid for hardware map."""
    h = int(float(params.get("HORIZON", FIXED_HORIZON)))
    dt = float(params.get("PRED_DT", FIXED_PRED_DT))
    if h != FIXED_HORIZON:
        return False
    if abs(dt - FIXED_PRED_DT) > 1e-9:
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
                raw_failed = int(parts[2])
                speed_check_pass = int(parts[22]) if len(parts) > 22 else 1
                failed_non_speed = int(parts[23]) if len(parts) > 23 else max(0, raw_failed - (0 if speed_check_pass else 1))
                status = "OK" if failed_non_speed == 0 else "EXIT_FAIL"
                return {
                    "status": status,
                    "return_code": return_code,
                    "passed": int(parts[1]),
                    "failed": raw_failed,
                    "failed_non_speed": failed_non_speed,
                    "speed_check_pass": speed_check_pass,
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
                    "solver_optimal_rate": float(parts[24]) if len(parts) > 24 else 0.0,
                    "solver_max_iter_rate": float(parts[25]) if len(parts) > 25 else 0.0,
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
    return (
        r.get("status") == "OK"
        and int(r.get("wall_collisions", 999)) == 0
        and int(r.get("failed_non_speed", r.get("failed", 999))) == 0
    )


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
            "failed_non_speed": 6,
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
            "solver_optimal_rate": 0.0,
            "solver_max_iter_rate": 0.0,
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
        "failed_non_speed": sum(int(r.get("failed_non_speed", r.get("failed", 0))) for r in scenario_results),
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
        "solver_optimal_rate": weighted_mean(scenario_results, "solver_optimal_rate"),
        "solver_max_iter_rate": weighted_mean(scenario_results, "solver_max_iter_rate"),
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
        aggregate[f"{prefix}failed_non_speed"] = int(r.get("failed_non_speed", r.get("failed", 0)))
        aggregate[f"{prefix}speed_check_pass"] = int(r.get("speed_check_pass", 1))

    return aggregate


def run_single_scenario(params: dict, binary: str, scenario: dict, seed: int) -> dict:
    """Run one deterministic scenario for the current MPC configuration."""
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"
    env["LOCAL_RACELINE_SIM"] = "1"
    env["REALISTIC_SIM"] = "1"
    env["REALISTIC_TIRES"] = "1"
    env["REALISTIC_DRIVE"] = "1"
    env["REALISTIC_NOISE"] = "1"
    env["SIM_SEED"] = str(seed + int(scenario.get("seed_offset", 0)))
    # Align test_sim_drive's internal wall-margin floor (half-width + BODY_SAFETY_MARGIN)
    # with the scenario audit assumptions.
    env["BODY_SAFETY_MARGIN"] = str(SCENARIO_BODY_SAFETY_MARGIN)
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
            "failed_non_speed": 6,
            "speed_check_pass": 0,
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
            parsed = {
                "status": "EXIT_FAIL",
                "return_code": result.returncode,
                "passed": 0,
                "failed": 6,
                "failed_non_speed": 6,
                "speed_check_pass": 0,
            }
        else:
            parsed = {
                "status": "NO_CSV",
                "return_code": result.returncode,
                "passed": 0,
                "failed": 6,
                "failed_non_speed": 6,
                "speed_check_pass": 0,
            }

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
    return (
        r.get("status") == "OK"
        and int(r.get("wall_collisions", 999)) == 0
        and int(r.get("failed_non_speed", r.get("failed", 999))) == 0
    )


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
    avg_vx = float(r.get("avg_vx", 0.0) or 0.0)
    race_def = max(0.0, MIN_RACE_PROGRESS_MPS - race_prog)
    overall_def = max(0.0, MIN_OVERALL_PROGRESS_MPS - overall_prog)
    speed_def = max(0.0, MIN_AVG_VX_MPS - avg_vx)
    # prioritize race progress first, then aggregate progress
    return race_def * 2.0 + overall_def + (0.5 * speed_def)


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
    avg_vx = float(r.get("avg_vx", 0.0) or 0.0)
    solver_opt = float(r.get("solver_optimal_rate", 0.0) or 0.0)
    solver_mx = float(r.get("solver_max_iter_rate", 1.0) or 1.0)

    if race_status and race_status != "OK":
        reasons.append("race_not_ok")
    if race_prog < MIN_RACE_PROGRESS_MPS:
        reasons.append("race_progress_low")
    if overall_prog < MIN_OVERALL_PROGRESS_MPS:
        reasons.append("overall_progress_low")
    if avg_vx < MIN_AVG_VX_MPS:
        reasons.append("avg_vx_low")
    if solver_opt < MIN_SOLVER_OPTIMAL_RATE:
        reasons.append("solver_optimal_rate_low")
    if solver_mx > MAX_SOLVER_MAX_ITER_RATE:
        reasons.append("solver_maxiter_rate_high")

    return reasons

def compute_reference_score(r: dict) -> float:
    """Reference score: minimize trajectory-following errors (lower is better)."""
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


def compute_base_score(r: dict) -> float:
    """Base score: minimize lap-time estimate, then lightly break ties by stability."""
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
    return round(r.get("lap_time_est", 999.0) + stability, 6)


def apply_scores(r: dict, objective: str) -> dict:
    """Attach scores and promotability diagnostics to a result row."""
    reasons = promotable_fail_reasons(r)
    r["promotable"] = 1 if not reasons else 0
    r["promotable_deficit"] = round(promotable_deficit_score(r), 6)
    r["promotable_reason"] = "|".join(reasons) if reasons else ""
    r["base_score"] = compute_base_score(r)
    r["score"] = r["base_score"]
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
    qlv_vals = values["Q_LAT_VEL"]
    qy_vals = values["Q_YAW"]
    rs_vals = values["R_STEER"]
    wda_vals = values["MPC_W_DELTA_ACTUAL"]
    for ql, qh, qv, qlv, qy, rs, wda in itertools.product(
            ql_vals, qh_vals, qv_vals, qlv_vals, qy_vals, rs_vals, wda_vals):
        w = dict(BASE)
        w["Q_LAT"] = ql
        w["Q_HDG"] = qh
        w["Q_VEL"] = qv
        w["Q_LAT_VEL"] = qlv
        w["Q_YAW"] = qy
        w["R_STEER"] = rs
        w["MPC_W_DELTA_ACTUAL"] = wda
        w["HORIZON"] = FIXED_HORIZON
        w["PRED_DT"] = FIXED_PRED_DT
        if is_valid_config(w):
            combos.append((
                f"L={ql}+H={qh}+V={qv}+LV={qlv}+Y={qy}+RS={rs}+WDA={wda}+HZ={FIXED_HORIZON}+DT={FIXED_PRED_DT:.3f}+CFG",
                w,
            ))
    
    return combos


def gen_primary_grid_local() -> list:
    """Focused local sweep around BASE to find a promotable seed region quickly."""
    combos = []

    def around(key: str, mults: tuple, lower: float = 0.0) -> list:
        base_v = float(BASE.get(key, 0.0))
        vals = sorted(set(max(lower, base_v * m) for m in mults))
        return vals

    ql_vals = around("Q_LAT", (0.90, 1.00, 1.10), 100.0)
    qv_vals = around("Q_VEL", (0.92, 1.00, 1.08), 10.0)
    rs_vals = around("R_STEER", (0.92, 1.00, 1.08), 0.1)
    wda_vals = around("MPC_W_DELTA_ACTUAL", (0.67, 1.00, 1.33), 0.005)

    for ql, qv, rs, wda in itertools.product(
            ql_vals, qv_vals, rs_vals, wda_vals):
        w = dict(BASE)
        w["Q_LAT"] = ql
        w["Q_VEL"] = qv
        w["R_STEER"] = rs
        w["MPC_W_DELTA_ACTUAL"] = wda
        w["HORIZON"] = FIXED_HORIZON
        w["PRED_DT"] = FIXED_PRED_DT
        if is_valid_config(w):
            combos.append((
                f"LOCAL+L={ql:.3f}+V={qv:.3f}+RS={rs:.3f}+WDA={wda:.4f}",
                w,
            ))

    return combos


def gen_secondary_grid(objective: str) -> list:
    """Phase 4: Secondary parameters grid."""
    combos = []
    
    values = get_secondary_grid_values(objective)
    qlv_vals = values["Q_LAT_VEL"]
    qy_vals = values["Q_YAW"]
    rs_vals = values["R_STEER"]
    wj_vals = values["W_JERK"]
    ra_vals = values["R_ACCEL"]
    war_vals = values["W_ACCEL_RATE"]
    wda_vals = values["MPC_W_DELTA_ACTUAL"]
    
    for qlv, qy, rs, wj, ra, war, wda in itertools.product(
            qlv_vals, qy_vals, rs_vals, wj_vals, ra_vals, war_vals, wda_vals):
        w = dict(BASE)
        w["Q_LAT_VEL"] = qlv
        w["Q_YAW"] = qy
        w["R_STEER"] = rs
        w["W_JERK"] = wj
        w["R_ACCEL"] = ra
        w["W_ACCEL_RATE"] = war
        w["MPC_W_DELTA_ACTUAL"] = wda
        combos.append((f"LV={qlv}+Y={qy}+RS={rs}+WJ={wj}+RA={ra}+WAR={war}+WDA={wda}", w))

    return combos


def gen_fine_tuning(best_weights: dict) -> list:
    """Phase 6: Fine-tuning around best config (~1000 configs)."""
    combos = []
    pct_range = (
        0.75, 0.78, 0.80, 0.83, 0.85, 0.88, 0.90,
        0.92, 0.95, 0.97, 0.99, 1.01, 1.03, 1.05,
        1.08, 1.10, 1.12, 1.15, 1.17, 1.20, 1.25,
    )
    solver_pair_range = (0.75, 0.80, 0.84, 0.88, 0.92, 0.95, 0.97, 0.99, 1.0, 1.01, 1.03, 1.05, 1.08, 1.12, 1.16, 1.22, 1.28)
    pair_patterns = (
        (0.95, 1.05), (1.05, 0.95), (0.92, 1.08), (1.08, 0.92),
        (0.97, 1.03), (1.03, 0.97), (0.90, 0.90), (1.10, 1.10),
    )
    triple_patterns = (
        (0.95, 0.95, 1.05), (0.95, 1.05, 0.95), (1.05, 0.95, 0.95),
        (1.05, 1.05, 0.95), (1.05, 0.95, 1.05), (0.95, 1.05, 1.05),
        (0.90, 1.00, 1.10), (1.10, 1.00, 0.90), (0.92, 1.00, 1.08),
        (1.08, 1.00, 0.92), (0.97, 1.03, 1.00), (1.03, 0.97, 1.00),
    )
    skip = {"MAX_ITER", "HORIZON", "PRED_DT", "WALL_MARGIN", "TOL", "RHO", "RHO_U"}

    def fmt_pct(mult: float) -> str:
        return f"{int(round((mult - 1.0) * 100)):+d}%"
    
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
            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
            combos.append((f"FT:{name}{fmt_pct(mult)}", w))

    # Coupled solver sweep: RHO and RHO_U always move together.
    if float(best_weights.get("RHO", 0.0) or 0.0) > 0.0 and float(best_weights.get("RHO_U", 0.0) or 0.0) > 0.0:
        for mult in solver_pair_range:
            w = apply_coupled_solver_pair(best_weights, mult)
            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
            combos.append((f"FT:RHO/RHO_U{fmt_pct(mult)}", w))
    
    # Pairwise perturbations across all tunable params.
    pair_params = [
        k for k, v in best_weights.items()
        if k not in skip and k not in ("RHO", "RHO_U") and v != 0
    ]
    for w1, w2 in itertools.combinations(pair_params, 2):
        v1 = best_weights.get(w1, 0)
        v2 = best_weights.get(w2, 0)
        if v1 == 0 or v2 == 0:
            continue
        for m1, m2 in pair_patterns:
            w = dict(best_weights)
            w[w1] = round(v1 * m1, 6)
            w[w2] = round(v2 * m2, 6)
            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
            combos.append((f"FT:{w1}{fmt_pct(m1)}+{w2}{fmt_pct(m2)}", w))

    # Solver pair plus one regular param.
    solver_pair_regular_patterns = (
        (0.95, 1.05), (1.05, 0.95), (0.92, 1.08), (1.08, 0.92),
        (0.97, 1.03), (1.03, 0.97), (0.90, 0.90), (1.10, 1.10),
    )
    for name in pair_params:
        base_val = best_weights.get(name, 0)
        if base_val == 0:
            continue
        for solver_mult, other_mult in solver_pair_regular_patterns:
            w = apply_coupled_solver_pair(best_weights, solver_mult)
            w[name] = round(base_val * other_mult, 6)
            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
            combos.append((f"FT:RHO/RHO_U{fmt_pct(solver_mult)}+{name}{fmt_pct(other_mult)}", w))

    # 3-way local perturbations for core steering/tracking params.
    triple_params = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW", "R_STEER"]
    for w1, w2, w3 in itertools.combinations(triple_params, 3):
        v1 = best_weights.get(w1, 0)
        v2 = best_weights.get(w2, 0)
        v3 = best_weights.get(w3, 0)
        if v1 == 0 or v2 == 0 or v3 == 0:
            continue
        for m1, m2, m3 in triple_patterns:
            w = dict(best_weights)
            w[w1] = round(v1 * m1, 6)
            w[w2] = round(v2 * m2, 6)
            w[w3] = round(v3 * m3, 6)
            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
            combos.append((f"FT:{w1}{fmt_pct(m1)}+{w2}{fmt_pct(m2)}+{w3}{fmt_pct(m3)}", w))

    # Solver pair plus two core params.
    solver_pair_triple_patterns = (
        (0.95, 0.95, 1.05), (0.95, 1.05, 0.95), (1.05, 0.95, 0.95),
        (1.05, 1.05, 0.95), (1.05, 0.95, 1.05), (0.95, 1.05, 1.05),
    )
    for w1, w2 in itertools.combinations(triple_params, 2):
        v1 = best_weights.get(w1, 0)
        v2 = best_weights.get(w2, 0)
        if v1 == 0 or v2 == 0:
            continue
        for solver_mult, m1, m2 in solver_pair_triple_patterns:
            w = apply_coupled_solver_pair(best_weights, solver_mult)
            w[w1] = round(v1 * m1, 6)
            w[w2] = round(v2 * m2, 6)
            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
            combos.append((f"FT:RHO/RHO_U{fmt_pct(solver_mult)}+{w1}{fmt_pct(m1)}+{w2}{fmt_pct(m2)}", w))
    
    return combos


def gen_random_neighbors(best_weights: dict, n: int, objective: str,
                         profile_override: str = None, seed_offset: int = 0) -> list: # type: ignore
    """Generate random perturbations around best config for exploration/exploitation phases."""
    combos = []
    rng = random.Random(SEED + seed_offset)
    profile_name = profile_override if profile_override else "base"
    profile = RANDOM_PROFILES.get(profile_name, RANDOM_PROFILES["base"])
    
    discrete = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_multipliers = profile.get("default_multipliers", [0.85, 0.95, 1.0, 1.1, 1.2])
    min_perturb, max_perturb = profile.get("num_perturb_range", (3, 6))
    
    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ITER", "WALL_MARGIN") 
                   and best_weights[k] != 0]
    tune_params = [k for k in tune_params if k not in ("HORIZON", "PRED_DT", "RHO", "RHO_U", *SOLVER_PARAM_KEYS)]
    if float(best_weights.get("RHO", 0.0) or 0.0) > 0.0 and float(best_weights.get("RHO_U", 0.0) or 0.0) > 0.0:
        tune_params.append(SOLVER_PAIR_PARAM)
    
    i = 0
    attempts = 0
    max_attempts = max(100, n * 20)
    
    while i < n and attempts < max_attempts:
        attempts += 1
        w = dict(best_weights)
        num_perturb = rng.randint(min_perturb, min(max_perturb, len(tune_params)))
        params_to_perturb = rng.sample(tune_params, num_perturb)
        
        for name in params_to_perturb:
            if name == SOLVER_PAIR_PARAM:
                mult = rng.choice(param_multipliers.get(name, default_multipliers))
                w = apply_coupled_solver_pair(w, mult)
            elif name in discrete:
                w[name] = rng.choice(discrete[name])
            else:
                mult = rng.choice(param_multipliers.get(name, default_multipliers))
                w[name] = round(w[name] * mult, 6)
            
            if name in INT_PARAMS:
                w[name] = int(float(w[name]))
                if name == "HORIZON":
                    w[name] = max(2, min(HORIZON_LIMIT, w[name]))

            w["HORIZON"] = FIXED_HORIZON
            w["PRED_DT"] = FIXED_PRED_DT
        
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
        self._header_written = os.path.exists(filepath) and os.path.getsize(filepath) > 0
    
    def write_row(self, row):
        mode = "a" if self._header_written else "w"
        with open(self.filepath, mode, newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames, extrasaction="ignore")
            if not self._header_written:
                writer.writeheader()
                self._header_written = True
            writer.writerow(row)


class ProgressTracker:
    """Writes a lightweight heartbeat file for long remote runs."""

    def __init__(self, filepath: str):
        self.filepath = filepath
        parent = os.path.dirname(os.path.abspath(filepath))
        if parent:
            os.makedirs(parent, exist_ok=True)

    def _write_lines(self, lines: list):
        tmp_path = f"{self.filepath}.tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        os.replace(tmp_path, self.filepath)

    def update(self,
               phase_name: str,
               done: int,
               total: int,
               passed: int,
               failed: int,
               label: str,
               status: str,
               eta_s: float,
               score: float = None):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        pct = (100.0 * float(done) / float(max(total, 1)))
        lines = [
            f"updated_at={now}",
            f"phase={phase_name}",
            f"progress={done}/{total} ({pct:.2f}%)",
            f"passed={passed}",
            f"failed={failed}",
            f"last_status={status}",
            f"eta_s={max(0.0, float(eta_s)):.1f}",
            f"last_label={label}",
        ]
        if score is not None:
            lines.append(f"last_score={float(score):.6f}")
        self._write_lines(lines)

    def finalize(self, total_tests: int, total_passed: int, total_failed: int, elapsed_s: float):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        lines = [
            f"updated_at={now}",
            "phase=COMPLETE",
            f"tests={total_tests}",
            f"passed={total_passed}",
            f"failed={total_failed}",
            f"elapsed_s={float(elapsed_s):.1f}",
        ]
        self._write_lines(lines)


# ==============================================================================
# PARALLEL WORKER
# ==============================================================================

def _run_single(args):
    """Worker: run one test and return scored result."""
    index, total, label, params, binary, phase_name, objective, eval_scenarios, raceline_tag = args
    p = canonicalize_params(dict(params))
    r = run_test(p, binary, eval_scenarios=eval_scenarios)
    r = apply_scores(r, objective)
    r.update(canonicalize_params(p))
    r["global_hdt_count"] = 1
    r["global_hdt_set"] = f"{int(r.get('HORIZON', 0))}x{float(r.get('PRED_DT', 0.0)):.3f}"

    r["label"] = f"{index}/{total} {label}"
    r["phase"] = phase_name
    r["raceline"] = raceline_tag
    return r


# ==============================================================================
# PHASE RUNNER
# ==============================================================================

def run_phase(phase_name: str, combos: list, binary: str, results: list,
              t0: float, num_workers: int, csv_writer, objective: str,
              progress_tracker=None) -> tuple:
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
            
            p = canonicalize_params(dict(params))
            r = run_test(p, binary)
            r = apply_scores(r, objective)
            r["label"] = f"{i+1}/{total} {label}"
            r["phase"] = phase_name
            r["raceline"] = RACELINE_TAG
            r.update(canonicalize_params(p))
            r["global_hdt_count"] = 1
            r["global_hdt_set"] = f"{int(r.get('HORIZON', 0))}x{float(r.get('PRED_DT', 0.0)):.3f}"
            results.append(r)
            
            if csv_writer and r.get("status") == "OK":
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

            if progress_tracker:
                progress_tracker.update(
                    phase_name=phase_name,
                    done=i + 1,
                    total=total,
                    passed=passed,
                    failed=failed,
                    label=r.get("label", label),
                    status=str(r.get("status", "")),
                    eta_s=eta,
                    score=r.get("score"),
                )
    else:
        # Parallel execution
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            scenario_bundle = [dict(s) for s in EVAL_SCENARIOS]
            raceline_tag = RACELINE_TAG
            it = ((idx, total, label, params, binary, phase_name, objective, scenario_bundle, raceline_tag)
                for idx, (label, params) in enumerate(combos, start=1))
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
                    
                    if csv_writer and r.get("status") == "OK":
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

                    if progress_tracker:
                        progress_tracker.update(
                            phase_name=phase_name,
                            done=done_count,
                            total=total,
                            passed=passed,
                            failed=failed,
                            label=r.get("label", ""),
                            status=str(r.get("status", "")),
                            eta_s=eta,
                            score=r.get("score"),
                        )
    
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
        ("RHO", BASE.get("RHO", 8) * 1.5),
        ("RHO_U", BASE.get("RHO_U", 24) * 1.3),
    ]
    
    ineffective = []
    for name, new_val in probes:
        p = dict(BASE)
        p[name] = new_val
        
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


def get_top_n_params(results: list, n: int = CASCADE_TOP_N, objective: str = "base") -> list:
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
            if STRICT_PROMOTION:
                print("  WARNING: No promotable/safe-full candidates; strict promotion returning no seeds.")
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


def apply_coupled_solver_pair(weights: dict, multiplier: float) -> dict:
    """Scale RHO and RHO_U together by same factor."""
    w = dict(weights)
    rho = float(w.get("RHO", BASE.get("RHO", 0.0)) or 0.0)
    rho_u = float(w.get("RHO_U", BASE.get("RHO_U", 0.0)) or 0.0)
    if rho > 0.0 and rho_u > 0.0:
        w["RHO"] = round(rho * multiplier, 6)
        w["RHO_U"] = round(rho_u * multiplier, 6)
    return w


def better_result(lhs: dict | None, rhs: dict | None) -> dict | None:
    """Pick better tuner row. Prefer promotable, then lower score."""
    if lhs is None:
        return rhs
    if rhs is None:
        return lhs

    lhs_prom = is_promotable_result(lhs)
    rhs_prom = is_promotable_result(rhs)
    if lhs_prom != rhs_prom:
        return lhs if lhs_prom else rhs

    try:
        lhs_score = float(lhs.get("score", 999999.0))
    except (TypeError, ValueError):
        lhs_score = 999999.0
    try:
        rhs_score = float(rhs.get("score", 999999.0))
    except (TypeError, ValueError):
        rhs_score = 999999.0

    return lhs if lhs_score <= rhs_score else rhs


def coerce_seed_value(key: str, raw_value: str):
    """Convert CSV text into the type expected by BASE."""
    if raw_value is None:
        return None

    text = str(raw_value).strip()
    if text == "":
        return None

    if key in INT_PARAMS:
        return int(float(text))
    return float(text)


def load_best_seed_from_csv(csv_path: str) -> tuple[dict | None, dict | None]:
    """Load best config row from tuning CSV."""
    if not csv_path or not os.path.exists(csv_path):
        return None, None

    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row:
                continue
            if row.get("status") and row.get("status") != "OK":
                continue
            if "score" not in row:
                continue
            try:
                score = float(row["score"])
            except (TypeError, ValueError):
                continue
            rows.append((score, row))

    if not rows:
        return None, None

    rows.sort(key=lambda item: item[0])
    score, best_row = rows[0]

    seed = dict(BASE_OVERRIDES)
    for key in BASE_OVERRIDES.keys():
        if key in best_row:
            value = coerce_seed_value(key, best_row.get(key))
            if value is not None:
                seed[key] = value

    meta = {
        "label": best_row.get("label", ""),
        "phase": best_row.get("phase", ""),
        "score": score,
        "source": csv_path,
    }
    return seed, meta


def load_progress_state(progress_path: str) -> dict:
    """Load key=value progress metadata if file exists."""
    if not progress_path or not os.path.exists(progress_path):
        return {}

    state = {}
    with open(progress_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            state[key.strip()] = value.strip()
    return state


def phase_priority(phase_name: str) -> int:
    """Order phases by tuning sequence."""
    if not phase_name:
        return -1
    if phase_name.startswith("Phase 1:"):
        return 1
    if phase_name.startswith("Phase 2:"):
        return 2
    if phase_name.startswith("Phase 4:"):
        return 4
    if phase_name.startswith("Phase 6:"):
        return 6
    if phase_name.startswith("Phase 7:"):
        return 7
    if phase_name.startswith("Phase 8:"):
        return 8
    return 0


def load_resume_seed_from_csv(csv_path: str) -> tuple[dict | None, dict | None]:
    """Resume from latest completed phase in CSV, not from global best row."""
    if not csv_path or not os.path.exists(csv_path):
        return None, None

    rows = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if not row:
                continue
            rows.append(row)

    if not rows:
        return None, None

    max_phase = max(phase_priority(r.get("phase", "")) for r in rows)
    phase_rows = [r for r in rows if phase_priority(r.get("phase", "")) == max_phase]
    if not phase_rows:
        phase_rows = rows

    best_row = None
    best_score = None
    for row in phase_rows:
        if row.get("status") not in (None, "", "OK"):
            continue
        try:
            score = float(row.get("score", "inf"))
        except (TypeError, ValueError):
            continue
        if best_row is None or score < best_score:
            best_row = row
            best_score = score

    if best_row is None:
        for row in phase_rows:
            try:
                score = float(row.get("score", "inf"))
            except (TypeError, ValueError):
                continue
            if best_row is None or score < best_score:
                best_row = row
                best_score = score

    if best_row is None:
        return None, None

    seed = dict(BASE_OVERRIDES)
    for key in BASE_OVERRIDES.keys():
        if key in best_row:
            value = coerce_seed_value(key, best_row.get(key))
            if value is not None:
                seed[key] = value

    meta = {
        "label": best_row.get("label", ""),
        "phase": best_row.get("phase", ""),
        "score": best_score if best_score is not None else float(best_row.get("score", 0.0) or 0.0),
        "source": csv_path,
        "resume_phase": max_phase,
    }
    return seed, meta


def make_continued_csv_path(source_csv: str, timestamp: str) -> str:
    """Create new CSV path for resumed run."""
    source_abs = os.path.abspath(source_csv)
    root, ext = os.path.splitext(source_abs)
    if not ext:
        ext = ".csv"
    return f"{root}_continued_{timestamp}{ext}"


# ==============================================================================
# MAIN
# ==============================================================================

def main():
    global BASE, RACELINE_PATH, RACELINE_TAG
    global TRACK_LENGTH_METERS, RACELINE_START_LEFT_BOUND, RACELINE_START_RIGHT_BOUND, EVAL_SCENARIOS
    global SCENARIO_RACELINE_PATHS
    global GLOBAL_START_SHIFT_X_M, GLOBAL_START_SHIFT_Y_M
    global WALL_MARGIN
    global RACE_SCENARIO_DURATION
    global HARDWARE_REPLAY_RUN
    global HARDWARE_REPLAY_WINDOW_SEC
    
    # Parse arguments
    num_workers = multiprocessing.cpu_count()  # Default to max workers
    objective = "base"
    raceline_override = None
    seed_csv_override = None
    resume_csv_override = None
    resume_progress_override = None
    append_resume_csv = False
    phase2_top_n = CASCADE_TOP_N
    global_passes = GLOBAL_OPTIMIZATION_PASSES
    include_obstacles = INCLUDE_OBSTACLE_SCENARIOS
    local_sweep = False
    local_duration = 8.0
    progress_file_override = None
    hardware_log_override = None
    hardware_meta_override = None
    hardware_window_sec = HARDWARE_REPLAY_WINDOW_SEC
    
    for i, arg in enumerate(sys.argv):
        if arg.startswith("--resume-csv "):
            resume_csv_override = arg.split(" ", 1)[1].strip()
        if arg.startswith("--seed-csv "):
            seed_csv_override = arg.split(" ", 1)[1].strip()
        if arg.startswith("--resume-progress "):
            resume_progress_override = arg.split(" ", 1)[1].strip()
        if arg in ("--jobs", "-j") and i + 1 < len(sys.argv):
            try:
                num_workers = int(sys.argv[i + 1])
            except ValueError:
                print(f"WARNING: invalid --jobs value '{sys.argv[i + 1]}', using CPU count")
                num_workers = multiprocessing.cpu_count()
            if num_workers <= 0:
                num_workers = multiprocessing.cpu_count()
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_override = sys.argv[i + 1].strip()
        if arg == "--seed-csv" and i + 1 < len(sys.argv):
            seed_csv_override = sys.argv[i + 1].strip()
        if arg == "--resume-csv" and i + 1 < len(sys.argv):
            resume_csv_override = sys.argv[i + 1].strip()
        if arg == "--resume-progress" and i + 1 < len(sys.argv):
            resume_progress_override = sys.argv[i + 1].strip()
        if arg in ("--append-csv", "--resume-append"):
            append_resume_csv = True
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
        if arg == "--local-sweep":
            local_sweep = True
        if arg == "--local-duration" and i + 1 < len(sys.argv):
            try:
                local_duration = max(2.0, float(sys.argv[i + 1]))
            except ValueError:
                print(f"WARNING: invalid --local-duration '{sys.argv[i + 1]}', using {local_duration}")
        if arg == "--progress-file" and i + 1 < len(sys.argv):
            progress_file_override = sys.argv[i + 1].strip()
        if arg == "--hardware-log" and i + 1 < len(sys.argv):
            hardware_log_override = sys.argv[i + 1].strip()
        if arg == "--hardware-meta" and i + 1 < len(sys.argv):
            hardware_meta_override = sys.argv[i + 1].strip()
        if arg == "--hardware-window-sec" and i + 1 < len(sys.argv):
            try:
                hardware_window_sec = float(sys.argv[i + 1])
            except ValueError:
                print(f"WARNING: invalid --hardware-window-sec '{sys.argv[i + 1]}', using {HARDWARE_REPLAY_WINDOW_SEC}")
        if arg == "--wall-margin" and i + 1 < len(sys.argv):
            try:
                requested = max(0.0, float(sys.argv[i + 1]))
                if abs(requested - FIXED_WALL_MARGIN) > 1e-9:
                    print(
                        f"INFO: --wall-margin={requested:.3f} ignored; "
                        f"using fixed WALL_MARGIN={FIXED_WALL_MARGIN:.3f}."
                    )
            except ValueError:
                print(f"WARNING: invalid --wall-margin '{sys.argv[i + 1]}', using fixed {FIXED_WALL_MARGIN}")

    if local_sweep:
        include_obstacles = True
        phase2_top_n = 1
        global_passes = 1
        RACE_SCENARIO_DURATION = local_duration

    # Keep wall margin fixed slightly above half vehicle width.
    WALL_MARGIN = FIXED_WALL_MARGIN

    # test_sim_drive floors effective margin to (vehicle_half_width + body_safety_margin).
    min_effective_wall_margin = SCENARIO_VEHICLE_HALF_WIDTH + SCENARIO_BODY_SAFETY_MARGIN
    if WALL_MARGIN < min_effective_wall_margin:
        print(
            f"INFO: WALL_MARGIN={WALL_MARGIN:.3f} below effective simulator floor "
            f"({min_effective_wall_margin:.3f}); using floor value."
        )
        WALL_MARGIN = min_effective_wall_margin

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

    if hardware_log_override:
        HARDWARE_REPLAY_WINDOW_SEC = hardware_window_sec
        hardware_log_path = os.path.abspath(hardware_log_override)
        if not os.path.exists(hardware_log_path):
            print(f"ERROR: Hardware log not found: {hardware_log_path}")
            sys.exit(1)
        if hardware_meta_override:
            hardware_meta_path = os.path.abspath(hardware_meta_override)
        else:
            hardware_meta_path = hardware_log_path + ".meta.txt"
        if not os.path.exists(hardware_meta_path):
            print(f"ERROR: Hardware meta not found: {hardware_meta_path}")
            sys.exit(1)
        HARDWARE_REPLAY_RUN = load_hardware_run(
            Path(hardware_log_path),
            Path(hardware_meta_path),
            load_raceline(Path(RACELINE_PATH)),
            window_seconds=HARDWARE_REPLAY_WINDOW_SEC if HARDWARE_REPLAY_WINDOW_SEC > 0.0 else None,
        )
        include_obstacles = False
        SCENARIO_RACELINE_PATHS = {"base": RACELINE_PATH}
    elif include_obstacles:
        SCENARIO_RACELINE_PATHS = build_scenario_raceline_paths(RACELINE_PATH)
    else:
        base_no_obstacle_path = build_no_obstacle_base_path(RACELINE_PATH)
        SCENARIO_RACELINE_PATHS = {"base": base_no_obstacle_path}
    EVAL_SCENARIOS = build_eval_scenarios(include_obstacles=include_obstacles)

    resume_mode = bool(resume_csv_override or resume_progress_override)
    resume_state = load_progress_state(resume_progress_override) if resume_progress_override else {}
    resume_csv_path = resolve_project_path(resume_csv_override) if resume_csv_override else None

    seed_params = None
    seed_meta = None
    if resume_state.get("resume_seed"):
        try:
            seed_params = json.loads(resume_state["resume_seed"])
            seed_meta = {
                "label": resume_state.get("resume_label", ""),
                "phase": resume_state.get("resume_phase_name", ""),
                "score": float(resume_state.get("resume_score", "nan")),
                "source": resume_progress_override,
                "resume_phase": int(resume_state.get("resume_phase", "0")),
                "resume_pass": int(resume_state.get("resume_pass", "1")),
            }
        except Exception:
            seed_params = None
            seed_meta = None

    if seed_params is None and resume_csv_path:
        seed_params, seed_meta = load_resume_seed_from_csv(resume_csv_path)

    if seed_params is None:
        seed_csv_path = resolve_project_path(seed_csv_override) if seed_csv_override else DEFAULT_BASE_SEED_CSV
        seed_params, seed_meta = load_best_seed_from_csv(seed_csv_path)

    # Initialize BASE config from best CSV seed when available.
    if seed_params:
        BASE.update(seed_params)
    else:
        BASE.update(BASE_OVERRIDES)
    BASE["WALL_MARGIN"] = float(WALL_MARGIN)
    
    print(f"\n{'='*80}")
    print("MPC Weight Tuning - Hardware Map")
    print(f"{'='*80}")
    print(f"  Workers:     {num_workers}")
    print("  Mode:        hardware-matched only")
    print(f"  Phase2->P4:  top {phase2_top_n}")
    print(f"  Global passes (P6-P8): {global_passes}")
    print(f"  Obstacles:   {'on' if include_obstacles else 'off'}")
    print(f"  Local sweep: {'on' if local_sweep else 'off'}")
    print(f"  Resume:      {'on' if resume_mode else 'off'}")
    if seed_meta:
        print(
            f"  Base seed:   {seed_meta['label'][:60]} "
            f"(score={seed_meta['score']:.6f}, phase={seed_meta['phase']}, "
            f"src={seed_meta['source']})"
        )
    else:
        print(f"  Base seed:   fallback hardcoded seed")
    if local_sweep:
        print(f"  Local race duration: {RACE_SCENARIO_DURATION:.1f}s")
    print(f"  Solver mode: fixed H={FIXED_HORIZON}, DT={FIXED_PRED_DT:.3f}; RHO/RHO_U tuned only in Phases 6-8")
    print("  Config eval: single (one config must pass all 3 scenarios)")
    print(f"  Base solver tuple: RHO={BASE.get('RHO')} RHO_U={BASE.get('RHO_U')} TOL={BASE.get('TOL')} MAX_ITER={BASE.get('MAX_ITER')}")
    print(f"  Wall margin: {WALL_MARGIN:.3f}")
    print(f"  Phase7 random: {PHASE7_RANDOM_COUNT}")
    print(f"  Phase8 random: {PHASE8_RANDOM_COUNT}")
    print(f"  Horizon sweep: {[FIXED_HORIZON]}")
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

    # Force line-buffered terminal output for remote/non-interactive runs.
    try:
        if hasattr(sys.stdout, "reconfigure"):
            sys.stdout.reconfigure(line_buffering=True)
    except Exception:
        pass
    
    # CSV writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"test/tuning_hardware_base_{timestamp}.csv"
    if resume_mode and resume_csv_path:
        if append_resume_csv:
            outfile = resume_csv_path
        else:
            outfile = make_continued_csv_path(resume_csv_path, timestamp)
            shutil.copy2(resume_csv_path, outfile)
    progress_file = progress_file_override if progress_file_override else f"test/tuning_hardware_base_{timestamp}_progress.txt"
    progress_tracker = ProgressTracker(progress_file)
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
            f"{prefix}failed_non_speed",
            f"{prefix}speed_check_pass",
        ])
    fieldnames = (
        ["label", "phase", "raceline", "score", "base_score",
         "promotable", "promotable_deficit", "promotable_reason",
         "global_hdt_count", "global_hdt_set",
         "passed", "failed", "failed_non_speed", "scenario_count", "scenario_failures", "recovery_failures", "main_failed",
         "lap_time_est", "completed_laps", "progress_m", "avg_progress_mps",
         "max_steer_change", "steer_reversals",
         "max_lat_err", "avg_lat_err",
         "max_hdg_err", "avg_hdg_err", "max_vx", "avg_vx",
         "avg_vel_err", "max_vel_err", "avg_solve_us", "max_solve_us",
         "wall_collisions", "time_above_5ms", "avg_iters", "solver_optimal_rate", "solver_max_iter_rate",
         "avg_lap_time", "status", "return_code"]
        + scenario_fieldnames
        + list(BASE.keys())
    )
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results: {outfile}\n")
    if resume_mode and resume_csv_path:
        if append_resume_csv:
            print(f"  Resume CSV: {resume_csv_path} (appending in-place)\n")
        else:
            print(f"  Resume CSV: {resume_csv_path} (copied to results)\n")
    print(f"  Progress: {progress_file}\n")
    
    # ========== PHASE 1: One-at-a-time ==========
    if local_sweep or resume_mode:
        print("\n  Skipping Phase 1 (resume/local sweep mode)")
    else:
        p, f = run_phase("Phase 1: One-at-a-time sensitivity",
                         gen_one_at_a_time(objective), binary, results, t0,
                         num_workers, csv_writer, objective, progress_tracker)
        total_p += p
        total_f += f

    # ========== PHASE 2: Primary grid ==========
    if local_sweep or resume_mode:
        print("\n  Skipping Phase 2 (resume/local sweep mode)")
    else:
        combos = gen_primary_grid_local() if local_sweep else gen_primary_grid(objective)
        print(f"\n  Phase 2 will test {len(combos):,} configurations")
        p, f = run_phase("Phase 2: Primary grid (Q_LAT x Q_HDG x Q_VEL x Q_LAT_VEL x Q_YAW x R_STEER x MPC_W_DELTA_ACTUAL)",
                         combos, binary, results, t0,
                         num_workers, csv_writer, objective, progress_tracker)
        total_p += p
        total_f += f

    if local_sweep:
        print("\nLocal sweep mode: skipping Phases 4-8 (focused seed-region search only).")
    elif resume_mode:
        print("\nResume mode: skipping Phases 4-8 setup, continuing Phase 5-8 from resume seed.")
    else:
        # Select top-N Phase 2 seeds, run Phase 4 from each, then refine global best.
        print("\n  Selecting Phase 2 seeds for Phase 4...")
        phase2_results = [r for r in results if str(r.get("phase", "")).startswith("Phase 2:")]
        phase2_seeds = get_top_n_params(phase2_results, n=phase2_top_n, objective=objective)
        if not phase2_seeds:
            phase2_seeds = [dict(BASE)]
    
    if not local_sweep and not resume_mode:
        # ========== PHASES 4-8 ==========

        # Phase 4: branch from top-N seeds from Phase 2.
        print("\n  Phase 4 branching from Phase 2 seeds...")
        for bi, seed_params in enumerate(phase2_seeds, start=1):
            update_base(seed_params)
            p, f = run_phase(f"Phase 4: Secondary grid [seed {bi}/{len(phase2_seeds)}]",
                             gen_secondary_grid(objective), binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f

        # Promote current global best after all branch sweeps.
        top_after_p4 = get_top_n_params(results, n=1, objective=objective)
        if top_after_p4:
            update_base(top_after_p4[0])

        # Global optimization loop: start each pass from pass-best seed.
        # Phase-local best only advances seed inside current pass.
        global_best = get_top_n_params(results, n=1, objective=objective)
        global_best_row = global_best[0] if global_best else dict(BASE)

        for pi in range(global_passes):
            print(f"\n{'#'*80}")
            print(f"# GLOBAL OPTIMIZATION PASS {pi+1}/{global_passes} (Phase 5-8)")
            print(f"{'#'*80}")

            pass_best_row = dict(global_best_row)
            pass_improved = False

            # Phase 5: lock in pass-best seed.
            update_base(pass_best_row)

            # Phase 6: fine tuning around pass-best.
            phase_start = len(results)
            update_base(pass_best_row)
            p, f = run_phase(f"Phase 6: Fine-tuning [pass {pi+1}/{global_passes}]",
                             gen_fine_tuning(pass_best_row), binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f
            phase_top = get_top_n_params(results[phase_start:], n=1, objective=objective)
            if phase_top:
                new_best = better_result(pass_best_row, phase_top[0])
                if new_best is not pass_best_row:
                    pass_best_row = new_best
                    pass_improved = True

            # Phase 7: random exploration around pass-best.
            phase_start = len(results)
            update_base(pass_best_row)
            n_random = PHASE7_RANDOM_COUNT
            p, f = run_phase(f"Phase 7: Random neighbors ({n_random}) [pass {pi+1}/{global_passes}]",
                             gen_random_neighbors(pass_best_row, n_random, objective,
                                                  seed_offset=7000 + pi),
                             binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f
            phase_top = get_top_n_params(results[phase_start:], n=1, objective=objective)
            if phase_top:
                new_best = better_result(pass_best_row, phase_top[0])
                if new_best is not pass_best_row:
                    pass_best_row = new_best
                    pass_improved = True

            # Phase 8: random exploitation around pass-best.
            phase_start = len(results)
            update_base(pass_best_row)
            n_random = PHASE8_RANDOM_COUNT
            p, f = run_phase(f"Phase 8: Random exploitation ({n_random}) [pass {pi+1}/{global_passes}]",
                             gen_random_neighbors(pass_best_row, n_random, objective,
                                                  profile_override="base_exploit",
                                                  seed_offset=9000 + pi),
                             binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f
            phase_top = get_top_n_params(results[phase_start:], n=1, objective=objective)
            if phase_top:
                new_best = better_result(pass_best_row, phase_top[0])
                if new_best is not pass_best_row:
                    pass_best_row = new_best
                    pass_improved = True

            if not pass_improved:
                print("  No improvement in this pass. Stop early.")
                break

            global_best_row = pass_best_row
    elif resume_mode:
        global_best_row = dict(BASE)
        if seed_meta and seed_meta.get("resume_phase"):
            print(f"\n  Resuming from phase {seed_meta['resume_phase']} seed.")
        else:
            print("\n  Resuming from loaded seed.")
        for pi in range(global_passes):
            print(f"\n{'#'*80}")
            print(f"# GLOBAL OPTIMIZATION PASS {pi+1}/{global_passes} (Phase 5-8 resume)")
            print(f"{'#'*80}")
            pass_best_row = dict(global_best_row)
            pass_improved = False

            phase_start = len(results)
            update_base(pass_best_row)
            p, f = run_phase(f"Phase 6: Fine-tuning [pass {pi+1}/{global_passes}]",
                             gen_fine_tuning(pass_best_row), binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f
            phase_top = get_top_n_params(results[phase_start:], n=1, objective=objective)
            if phase_top:
                new_best = better_result(pass_best_row, phase_top[0])
                if new_best is not pass_best_row:
                    pass_best_row = new_best
                    pass_improved = True

            phase_start = len(results)
            update_base(pass_best_row)
            n_random = PHASE7_RANDOM_COUNT
            p, f = run_phase(f"Phase 7: Random neighbors ({n_random}) [pass {pi+1}/{global_passes}]",
                             gen_random_neighbors(pass_best_row, n_random, objective,
                                                  seed_offset=7000 + pi),
                             binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f
            phase_top = get_top_n_params(results[phase_start:], n=1, objective=objective)
            if phase_top:
                new_best = better_result(pass_best_row, phase_top[0])
                if new_best is not pass_best_row:
                    pass_best_row = new_best
                    pass_improved = True

            phase_start = len(results)
            update_base(pass_best_row)
            n_random = PHASE8_RANDOM_COUNT
            p, f = run_phase(f"Phase 8: Random exploitation ({n_random}) [pass {pi+1}/{global_passes}]",
                             gen_random_neighbors(pass_best_row, n_random, objective,
                                                  profile_override="base_exploit",
                                                  seed_offset=9000 + pi),
                             binary, results, t0,
                             num_workers, csv_writer, objective, progress_tracker)
            total_p += p
            total_f += f
            phase_top = get_top_n_params(results[phase_start:], n=1, objective=objective)
            if phase_top:
                new_best = better_result(pass_best_row, phase_top[0])
                if new_best is not pass_best_row:
                    pass_best_row = new_best
                    pass_improved = True

            if not pass_improved:
                print("  No improvement in this pass. Stop early.")
                break

            global_best_row = pass_best_row

    # ========== FINAL RESULTS ==========
    results.sort(key=lambda x: x.get("score", 999999.0))
    elapsed = time.time() - t0
    
    print(f"\n{'='*80}")
    print(f"COMPLETED {len(results):,} tests in {elapsed:.1f}s ({elapsed/60:.1f} min)")
    print(f"  Passed: {total_p}  Failed: {total_f}")
    print(f"{'='*80}")
    progress_tracker.finalize(len(results), total_p, total_f, elapsed)
    
    # Write sorted results
    sorted_file = outfile.replace(".csv", "_sorted.csv")
    ok_results = [r for r in results if r.get("status") == "OK"]
    if ok_results:
        with open(sorted_file, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            writer.writerows(ok_results)
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
