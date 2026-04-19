#!/usr/bin/env python3
"""
Base MPC tuning for the hardware map.
=====================================
Sweeps MPC weights, horizons, and solver timing on the hardware
SLAM-mapped track (~22m, 0.27-1.4m wide).

Usage:
    python3 test/tune_realistic_v2.py                        # Full sweep (all CPUs)
    python3 test/tune_realistic_v2.py -j 0                   # Use all workers

The sweep runs 6 phases:
    Phase 1: One-at-a-time parameter sensitivity
    Phase 2: Primary grid (Q_LAT x Q_HDG x Q_VEL x HORIZON x PRED_DT)
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
TRAJ_DIR = os.path.join(PROJECT_DIR, "trajectories")

HORIZON_SWEEP_VALUES = [20]
HORIZON_LIMIT = 20

# ==============================================================================
# HARDWARE MAP CONFIGURATION
# ==============================================================================

DEFAULT_RACELINE_NAME = "my_track_raceline.csv"
RACELINE_PATH = os.path.join(TRAJ_DIR, DEFAULT_RACELINE_NAME)
RACELINE_TAG = "my_track"
WALL_MARGIN = 0.20

# Base override seed
BASE_OVERRIDES = {
    # Seeded from top row in tuning_hardware_fastest_20260410_231743_sorted.csv
    "Q_LAT": 9660.42,
    "Q_HDG": 1400.0,
    "Q_VEL": 132.192,
    "Q_LAT_VEL": 4.59,
    "Q_YAW": 2.112,
    "R_STEER": 2.244,
    "R_ACCEL": 0.0065,
    "W_JERK": 0.063,
    "W_ACCEL_RATE": 0.17,
    "MPC_W_DELTA_ACTUAL": 0.03,
    "HORIZON": 10,
    "PRED_DT": 0.032,
    "RHO": 18.0,
    "RHO_U": 24.0,
    "ALPHA": 1.05,
    "TOL": 0.05,
    "MAX_ITER": 100,
    "WALL_END": 10,
    "WALL_STRIDE": 1,
    "WALL_MARGIN": 0.2,
}

# ==============================================================================
# SWEEP VALUE RANGES - PHASE 2 (Primary Grid)
# ==============================================================================

PHASE2_VALUES_BASE = {
    # Large primary grid centered around the attached best CSV seed.
    # Sweeps grouped solver bucket profile as one config value
    # (HORIZON+PRED_DT+RHO+RHO_U+TOL tied together).
    # 6*5*5*4*4*4*3*10 = 288,000 configs.
    "Q_LAT": [2000.0, 4000.0, 6000.0, 8000.0, 9660.42],
    "Q_HDG": [400, 800, 1100.0, 1250.0, 1400.0],
    "Q_VEL": [60, 95.0, 110.0, 122.4, 132.192, 145.0],
    "Q_LAT_VEL": [2.0, 3.8, 4.59, 5.6, 7.0],
    "Q_YAW": [1.8, 2.112, 2.6, 3.2, 3.8],
    "R_STEER": [1.8, 2.1, 2.244, 2.5, 2.8],
    "MPC_W_DELTA_ACTUAL": [0.02, 0.03, 0.05, 0.08, 0.2],
    "SOLVER_BUCKET": ["t01"],
}

# ==============================================================================
# SWEEP VALUE RANGES - ALL PARAMETERS (for one-at-a-time and fine-tuning)
# ==============================================================================

FULL_SWEEP_VALUES_BASE = {
    "Q_LAT":        [2000.0, 4000.0, 6000.0, 8000.0, 9660.42, 10300.0, 11000.0],
    "Q_HDG":        [400, 800, 1100.0, 1250.0, 1400.0, 1550.0, 1700.0],
    "Q_VEL":        [60, 95.0, 110.0, 122.4, 132.192, 145.0],
    "Q_LAT_VEL":    [2.0, 3.8, 4.59, 5.6, 7.0, 8.5],
    "Q_YAW":        [1.8, 2.112, 2.6, 3.2, 3.8],
    "R_STEER":      [1.8, 2.1, 2.244, 2.5, 2.8],
    "R_ACCEL":      [0.004, 0.005, 0.0065, 0.0075, 0.0085, 0.0095],
    "W_JERK":       [0.05, 0.063, 0.08, 0.10, 0.12],
    "W_ACCEL_RATE": [0.14, 0.16, 0.17, 0.19, 0.22],
    "MPC_W_DELTA_ACTUAL": [0.02, 0.03, 0.05, 0.08],
    "HORIZON":      HORIZON_SWEEP_VALUES,
    "RHO":          [14, 18, 22],
    "RHO_U":        [20, 24, 28],
    "PRED_DT":      [0.03],
    "TOL":          [0.03, 0.05, 0.08, 0.10],
}

# ==============================================================================
# PHASE 4: Secondary Grid Values (~18,000 configs)
# Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE
# ==============================================================================

PHASE4_VALUES_BASE = {
    "Q_LAT_VEL":    [2.0, 3.8, 4.59, 5.6, 7.0, 8.5],
    "Q_YAW":        [1.8, 2.112, 2.6, 3.2, 3.8],
    "R_STEER":      [1.8, 2.1, 2.244, 2.5, 2.8],
    "W_JERK":       [0.05, 0.063, 0.08, 0.10, 0.12],
    "R_ACCEL":      [0.004, 0.005, 0.0065, 0.0075, 0.0085, 0.0095],
    "W_ACCEL_RATE": [0.14, 0.16, 0.17, 0.19, 0.22],
    "MPC_W_DELTA_ACTUAL": [0.02, 0.03, 0.05, 0.08, 0.10],
}

# ==============================================================================
# SOLVER BUCKETS (lookahead-based)
# T = HORIZON * PRED_DT -> (RHO, RHO_U, TOL)
# ==============================================================================

SOLVER_BUCKETS = [
    {
        "name": "t01",
        "t_max": 1.15,
        "horizon": 20,
        "pred_dt": 0.03,
        "rho": 28.0,
        "rho_u": 42.0,
        "tol": 0.05,
    },
]

SOLVER_BUCKETS_BY_NAME = {b["name"]: b for b in SOLVER_BUCKETS}

# Evaluate each weight candidate across multiple lookahead anchors so
# promotion does not overfit to one short-horizon operating point.
GLOBAL_WEIGHT_REGION_HDT = [
    (20, 0.03)
]

# Candidate is globally passable if enough anchors are promotable.
GLOBAL_HDT_MIN_PASS_RATIO = 0.50

# ==============================================================================
# RANDOM NEIGHBOR PROFILES
# ==============================================================================

RANDOM_PROFILES = {
    "base": {
        "num_perturb_range": (3, 6),
        "default_multipliers": [0.96, 0.99, 1.0, 1.03, 1.06],
        "param_multipliers": {
            "Q_LAT": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "Q_HDG": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "Q_VEL": [0.8, 0.9, 0.96, 1.0, 1.05, 1.1, 1.2],
            "Q_LAT_VEL": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "Q_YAW": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "R_STEER": [0.85, 0.92, 0.97, 1.0, 1.04, 1.08],
            "R_ACCEL": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "W_JERK": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "W_ACCEL_RATE": [0.8, 0.9, 0.96, 1.0, 1.04, 1.1],
            "RHO": [0.85, 0.92, 0.97, 1.0, 1.04, 1.1],
            "RHO_U": [0.85, 0.92, 0.97, 1.0, 1.04, 1.1],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [0.03],
        },
    },
    "base_exploit": {
        "num_perturb_range": (2, 3),
        "default_multipliers": [0.97, 0.99, 1.0, 1.02, 1.05],
        "param_multipliers": {
            "Q_LAT": [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_HDG": [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_VEL": [0.97, 1.0, 1.02, 1.05, 1.08],
            "Q_LAT_VEL": [0.96, 0.99, 1.0, 1.02, 1.05],
            "Q_YAW": [0.96, 0.99, 1.0, 1.02, 1.05],
            "R_STEER": [0.96, 0.99, 1.0, 1.02, 1.05],
            "R_ACCEL": [0.96, 0.99, 1.0, 1.02, 1.05],
            "W_JERK": [0.96, 0.99, 1.0, 1.02, 1.05],
            "W_ACCEL_RATE": [0.96, 0.99, 1.0, 1.02, 1.05],
            "RHO": [0.96, 0.99, 1.0, 1.02, 1.05],
            "RHO_U": [0.96, 0.99, 1.0, 1.02, 1.05],
        },
        "discrete": {
            "HORIZON": HORIZON_SWEEP_VALUES,
            "PRED_DT": [0.03],
        },
    },
}

# ==============================================================================
# CONSTANTS
# ==============================================================================

INT_PARAMS = {"HORIZON", "MAX_ITER"}

SCENARIO_VEHICLE_HALF_WIDTH = 0.137
SCENARIO_BODY_SAFETY_MARGIN = 0.06
MAX_OFFSET_STEP_M = 0.015
MAX_HEADING_STEP_RAD = 0.30
P99_HEADING_STEP_RAD = 0.18
RACE_SCENARIO_DURATION = 75.0
OBSTACLE_SCENARIO_DURATION = 60.0

SCENARIO_TARGET_START_X = 0.0
SCENARIO_TARGET_START_Y = 0.0
GLOBAL_START_SHIFT_X_M = 0.0
GLOBAL_START_SHIFT_Y_M = 0.0
MIN_RACE_PROGRESS_MPS = 0.55
MIN_OVERALL_PROGRESS_MPS = 0.45
MIN_AVG_VX_MPS = 1.0
MIN_SOLVER_OPTIMAL_RATE = 0.0
MAX_SOLVER_MAX_ITER_RATE = 1.0
PLANNER_CAR_WIDTH_M = 0.31
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

DETERMINISTIC_OBSTACLE_PROFILES = {
    "avoid_single": {
        "objects": [
            #{"s_fraction": 0.62, "lateral_offset": -0.2},
        ],
    },
    "avoid_double": {
        "objects": [
            #{"s_fraction": 0.58, "lateral_offset": 0.3},
            #{"s_fraction": 0.9, "lateral_offset": -0.3},
        ],
    },
}

TRACK_LENGTH_METERS = 0.0
RACELINE_START_LEFT_BOUND = 0.0
RACELINE_START_RIGHT_BOUND = 0.0
EVAL_SCENARIOS = []
GENERATED_RACELINE_DIR = None
SCENARIO_RACELINE_PATHS = {}

CASCADE_TOP_N = 1   # Top-N seeds promoted from Phase 2 into Phase 4
SEED = 42           # Fixed seed for reproducibility
GLOBAL_OPTIMIZATION_PASSES = 4  # Repeated refinement passes for Phases 5-8
INCLUDE_OBSTACLE_SCENARIOS = False
PHASE7_RANDOM_COUNT = 5000
PHASE8_RANDOM_COUNT = 5000
STRICT_PROMOTION = True
SOLVER_PARAM_KEYS = ("RHO", "RHO_U", "TOL")
DIVERSITY_KEYS = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW", "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE", "MPC_W_DELTA_ACTUAL", "HORIZON", "PRED_DT"]
DIVERSITY_MIN_DISTANCE = 0.10

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
        for box, window in zip(obstacle_boxes, obstacle_windows):
            if is_s_in_window(wp["s"], window[0], window[1], track_length):
                active_boxes.append(box)
        if not active_boxes:
            continue

        normal = wp["psi"] + math.pi / 2.0
        nx = math.cos(normal)
        ny = math.sin(normal)

        left_d = max(float(wp.get("left", 0.0)), 0.0)
        right_d = max(float(wp.get("right", 0.0)), 0.0)

        sx, sy = wp["x"], wp["y"]
        lx, ly = sx + left_d * nx, sy + left_d * ny
        rx, ry = sx - right_d * nx, sy - right_d * ny

        for box in active_boxes:
            lx, ly = clip_segment_to_box(sx, sy, lx, ly, box)
            rx, ry = clip_segment_to_box(sx, sy, rx, ry, box)

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

    baseline_left_wall = []
    baseline_right_wall = []
    for sample in base_samples:
        normal = sample["psi"] + math.pi / 2.0
        nx = math.cos(normal)
        ny = math.sin(normal)
        baseline_left_wall.append((
            sample["x"] + sample["left"] * nx,
            sample["y"] + sample["left"] * ny,
        ))
        baseline_right_wall.append((
            sample["x"] - sample["right"] * nx,
            sample["y"] - sample["right"] * ny,
        ))

    def materialize_from_offsets(offsets: list) -> list:
        out = [dict(sample) for sample in base_samples]
        for idx, sample in enumerate(base_samples):
            offset = offsets[idx]
            normal = sample["psi"] + math.pi / 2.0
            out[idx]["x"] = sample["x"] + offset * math.cos(normal)
            out[idx]["y"] = sample["y"] + offset * math.sin(normal)
            out[idx]["left"] = max(0.0, sample["left"])
            out[idx]["right"] = max(0.0, sample["right"])

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

        # Re-project to fixed baseline wall world-points so the map wall
        # geometry remains unchanged outside obstacle edits.
        for idx, curr_wp in enumerate(out):
            normal = curr_wp["psi"] + math.pi / 2.0
            nx = math.cos(normal)
            ny = math.sin(normal)

            left_hits = [
                ray_polyline_distance(curr_wp["x"], curr_wp["y"], nx, ny, baseline_left_wall),
                ray_polyline_distance(curr_wp["x"], curr_wp["y"], nx, ny, baseline_right_wall),
            ]
            right_hits = [
                ray_polyline_distance(curr_wp["x"], curr_wp["y"], -nx, -ny, baseline_left_wall),
                ray_polyline_distance(curr_wp["x"], curr_wp["y"], -nx, -ny, baseline_right_wall),
            ]

            left_valid = [d for d in left_hits if d is not None]
            right_valid = [d for d in right_hits if d is not None]

            left_hit = min(left_valid) if left_valid else None
            right_hit = min(right_valid) if right_valid else None

            if left_hit is None:
                left_hit = max(0.0, base_samples[idx]["left"])
            if right_hit is None:
                right_hit = max(0.0, base_samples[idx]["right"])

            curr_wp["left"] = max(0.0, left_hit)
            curr_wp["right"] = max(0.0, right_hit)
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
    for idx, sample in enumerate(base_samples):
        if not active_mask[idx]:
            final_offsets.append(0.0)
            continue
        max_left = max(sample["left"] - 1e-6, 0.0)
        max_right = max(sample["right"] - 1e-6, 0.0)
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
    base_samples = align_samples_to_target_start(base_samples_raw)
    enforce_min_wall_clearance(base_samples, "base")

    base_out = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_base.csv")
    write_raceline_samples(base_out, base_samples)

    paths = {"base": os.path.abspath(base_out)}
    for profile_name, profile in DETERMINISTIC_OBSTACLE_PROFILES.items():
        validate_obstacle_profile_feasibility(base_samples, profile["objects"], profile_name)
        shifted_samples = build_shifted_raceline_samples(base_samples, profile["objects"])
        # Keep a common exact start point across all scenario racelines.
        if shifted_samples:
            dx = SCENARIO_TARGET_START_X - shifted_samples[0]["x"]
            dy = SCENARIO_TARGET_START_Y - shifted_samples[0]["y"]
            shifted_samples = translate_samples(shifted_samples, dx, dy)
        enforce_min_wall_clearance(shifted_samples, profile_name)
        out_path = os.path.join(GENERATED_RACELINE_DIR, f"{RACELINE_TAG}_{profile_name}.csv")
        write_raceline_samples(out_path, shifted_samples)
        paths[profile_name] = os.path.abspath(out_path)
    return paths


atexit.register(cleanup_generated_racelines)


def build_eval_scenarios(include_obstacles: bool = INCLUDE_OBSTACLE_SCENARIOS) -> list:
    """Build deterministic scenarios where every run starts off-raceline."""
    base_path = SCENARIO_RACELINE_PATHS.get("base", RACELINE_PATH)

    scenarios = [
        {
            "name": "race",
            "weight": 1.00 if not include_obstacles else 0.50,
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
    ]

    if include_obstacles:
        #avoid_single_path = SCENARIO_RACELINE_PATHS.get("avoid_single", RACELINE_PATH)
        #avoid_double_path = SCENARIO_RACELINE_PATHS.get("avoid_double", RACELINE_PATH)
        """scenarios.extend([
            {
                "name": "avoid_single",
                "weight": 0.25,
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
                "weight": 0.25,
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
        """

    return scenarios



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

    h = int(float(out.get("HORIZON", BASE.get("HORIZON", HORIZON_LIMIT))))
    h = max(2, min(HORIZON_LIMIT, h))
    out["HORIZON"] = h
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
    sb_vals = values["SOLVER_BUCKET"]
    for ql, qh, qv, qlv, qy, rs, wda, sb in itertools.product(
            ql_vals, qh_vals, qv_vals, qlv_vals, qy_vals, rs_vals, wda_vals, sb_vals):
        w = dict(BASE)
        w["Q_LAT"] = ql
        w["Q_HDG"] = qh
        w["Q_VEL"] = qv
        w["Q_LAT_VEL"] = qlv
        w["Q_YAW"] = qy
        w["R_STEER"] = rs
        w["MPC_W_DELTA_ACTUAL"] = wda
        w["SOLVER_BUCKET"] = sb
        w = apply_solver_bucket(w)
        if is_valid_config(w):
            combos.append((
                f"L={ql}+H={qh}+V={qv}+LV={qlv}+Y={qy}+RS={rs}+WDA={wda}+SB={sb}+HZ={int(w['HORIZON'])}+DT={float(w['PRED_DT']):.3f}+CFG",
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
    """Phase 6: Fine-tuning around best config (~2000 configs)."""
    combos = []
    pct_range = (0.80, 0.85, 0.90, 0.92, 0.95, 0.97, 1.03, 1.05, 1.08, 1.10, 1.15, 1.20)
    skip = {"MAX_ITER", "HORIZON", "WALL_MARGIN"}
    
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
    profile_name = profile_override if profile_override else "base"
    profile = RANDOM_PROFILES.get(profile_name, RANDOM_PROFILES["base"])
    
    discrete = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_multipliers = profile.get("default_multipliers", [0.85, 0.95, 1.0, 1.1, 1.2])
    min_perturb, max_perturb = profile.get("num_perturb_range", (3, 6))
    
    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ITER", "WALL_MARGIN") 
                   and best_weights[k] != 0]
    tune_params = [k for k in tune_params if k not in ("HORIZON", "PRED_DT")]
    
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


def select_solver_bucket(params: dict) -> dict:
    """Select grouped HORIZON/PRED_DT/RHO/RHO_U/TOL solver bucket."""
    explicit_name = str(params.get("SOLVER_BUCKET", "") or "").strip()
    if explicit_name:
        bucket = SOLVER_BUCKETS_BY_NAME.get(explicit_name)
        if bucket is None:
            bucket = SOLVER_BUCKETS[0]
        return {
            "HORIZON": int(bucket["horizon"]),
            "PRED_DT": float(bucket["pred_dt"]),
            "RHO": float(bucket["rho"]),
            "RHO_U": float(bucket["rho_u"]),
            "TOL": float(bucket["tol"]),
        }

    # Fallback path: infer bucket from current HORIZON/PRED_DT by T=H*dt.
    h = int(float(params.get("HORIZON", BASE.get("HORIZON", 10)) or 10))
    dt = float(params.get("PRED_DT", BASE.get("PRED_DT", 0.034)) or 0.034)
    lookahead_t = max(0.01, float(h) * dt)

    selected = SOLVER_BUCKETS[-1]
    for bucket in SOLVER_BUCKETS:
        if lookahead_t <= float(bucket["t_max"]):
            selected = bucket
            break

    return {
        "HORIZON": int(h),
        "PRED_DT": float(dt),
        "RHO": float(selected["rho"]),
        "RHO_U": float(selected["rho_u"]),
        "TOL": float(selected["tol"]),
    }


def apply_solver_bucket(params: dict) -> dict:
    """Return params with solver tuple overridden by T=H*dt bucket schedule."""
    out = dict(params)
    out.update(select_solver_bucket(out))
    return out


def aggregate_global_hdt_rows(rows: list) -> dict:
    """Aggregate per-anchor results into a single global-region score row."""
    if not rows:
        return {
            "status": "EXIT_FAIL",
            "score": 999999.0,
            "base_score": 999999.0,
            "promotable": 0,
            "promotable_reason": "no_rows",
        }

    first = rows[0]
    out = dict(first)

    count = len(rows)
    avg = lambda key: sum(float(r.get(key, 0.0) or 0.0) for r in rows) / float(count)
    minv = lambda key: min(float(r.get(key, 0.0) or 0.0) for r in rows)
    maxv = lambda key: max(float(r.get(key, 0.0) or 0.0) for r in rows)

    non_promotable = [r for r in rows if not is_promotable_result(r)]
    fail_count = len(non_promotable)
    promotable_count = count - fail_count
    min_promotable = max(1, int(math.ceil(GLOBAL_HDT_MIN_PASS_RATIO * float(count))))
    pass_global = promotable_count >= min_promotable

    mean_base = avg("base_score")
    worst_base = maxv("base_score")
    failure_penalty = 2000.0 * float(fail_count)
    global_score = mean_base + (0.35 * worst_base) + failure_penalty

    out["status"] = "OK" if pass_global else "EXIT_FAIL"
    out["scenario_count"] = int(sum(int(r.get("scenario_count", 0) or 0) for r in rows))
    out["scenario_failures"] = int(sum(int(r.get("scenario_failures", 0) or 0) for r in rows))
    out["failed"] = int(sum(int(r.get("failed", 0) or 0) for r in rows))
    out["failed_non_speed"] = int(sum(int(r.get("failed_non_speed", 0) or 0) for r in rows))
    out["wall_collisions"] = int(sum(int(r.get("wall_collisions", 0) or 0) for r in rows))

    out["avg_lat_err"] = avg("avg_lat_err")
    out["avg_hdg_err"] = avg("avg_hdg_err")
    out["avg_vx"] = minv("avg_vx")
    out["avg_progress_mps"] = minv("avg_progress_mps")
    out["scenario_race_avg_progress_mps"] = minv("scenario_race_avg_progress_mps")
    out["scenario_race_status"] = "OK" if all(str(r.get("scenario_race_status", "")) == "OK" for r in rows) else "EXIT_FAIL"
    out["solver_optimal_rate"] = minv("solver_optimal_rate")
    out["solver_max_iter_rate"] = maxv("solver_max_iter_rate")
    out["lap_time_est"] = avg("lap_time_est")

    out["base_score"] = round(global_score, 6)
    out["score"] = out["base_score"]
    out["promotable"] = 1 if pass_global else 0
    out["promotable_deficit"] = round(promotable_deficit_score(out), 6)
    if not pass_global:
        reasons = sorted(set(r.get("promotable_reason", "") for r in non_promotable if r.get("promotable_reason")))
        out["promotable_reason"] = f"global_hdt_fail:{promotable_count}/{count}:" + ";".join(reasons[:4])
    else:
        out["promotable_reason"] = ""

    out["global_hdt_count"] = count
    out["global_hdt_promotable"] = promotable_count
    out["global_hdt_set"] = "|".join(f"{int(r.get('HORIZON', 0))}x{float(r.get('PRED_DT', 0.0)):.3f}" for r in rows)
    return out


def build_global_hdt_region(center_h: int, center_dt: float) -> list:
    """Build sweepable anchor set around candidate horizon/dt."""
    anchors = set(GLOBAL_WEIGHT_REGION_HDT)
    h = int(center_h)
    dt = float(center_dt)
    anchors.add((h, dt))

    # Add local neighbors around candidate to sweep anchor region itself.
    for dh, ddt in [(-4, -0.004), (-2, -0.002), (2, 0.002), (4, 0.004)]:
        nh = int(max(min(h + dh, max(HORIZON_SWEEP_VALUES)), min(HORIZON_SWEEP_VALUES)))
        ndt = max(0.032, min(0.050, dt + ddt))
        anchors.add((nh, round(ndt, 3)))

    return sorted(anchors, key=lambda x: (int(x[0]), float(x[1])))


# ==============================================================================
# PARALLEL WORKER
# ==============================================================================

def _run_single(args):
    """Worker: run one test and return scored result."""
    label, params, binary, phase_name, objective, eval_scenarios, raceline_tag = args
    p = apply_solver_bucket(dict(params))
    r = run_test(p, binary, eval_scenarios=eval_scenarios)
    r = apply_scores(r, objective)
    r.update(canonicalize_params(p))
    r["global_hdt_count"] = 1
    r["global_hdt_set"] = f"{int(r.get('HORIZON', 0))}x{float(r.get('PRED_DT', 0.0)):.3f}"

    r["label"] = label
    r["phase"] = phase_name
    r["raceline"] = raceline_tag
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
            
            p = apply_solver_bucket(dict(params))
            r = run_test(p, binary)
            r = apply_scores(r, objective)
            r["label"] = label
            r["phase"] = phase_name
            r["raceline"] = RACELINE_TAG
            r.update(canonicalize_params(p))
            r["global_hdt_count"] = 1
            r["global_hdt_set"] = f"{int(r.get('HORIZON', 0))}x{float(r.get('PRED_DT', 0.0)):.3f}"
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
    objective = "base"
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
    BASE.update(BASE_OVERRIDES)
    
    print(f"\n{'='*80}")
    print("MPC Weight Tuning - Hardware Map")
    print(f"{'='*80}")
    print(f"  Workers:     {num_workers}")
    print("  Mode:        base")
    print(f"  Phase2->P4:  top {phase2_top_n}")
    print(f"  Global passes (P6-P8): {global_passes}")
    print(f"  Obstacles:   {'on' if include_obstacles else 'off'}")
    print("  Solver mode: T=H*dt bucketed RHO/RHO_U/TOL")
    print("  Config eval: single (one config must pass all 3 scenarios)")
    print(f"  Base solver tuple: RHO={BASE.get('RHO')} RHO_U={BASE.get('RHO_U')} TOL={BASE.get('TOL')} MAX_ITER={BASE.get('MAX_ITER')}")
    print(f"  Phase7 random: {PHASE7_RANDOM_COUNT}")
    print(f"  Phase8 random: {PHASE8_RANDOM_COUNT}")
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
    outfile = f"test/tuning_hardware_base_{timestamp}.csv"
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

    # ========== PHASES 4-8 ==========

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
    # through Phases 5-8, always handing off the global best so phase-local
    # regressions are never promoted forward.
    current_best = get_top_n_params(results, n=1, objective=objective)
    current_best_params = current_best[0] if current_best else dict(BASE)

    for pi in range(global_passes):
        print(f"\n{'#'*80}")
        print(f"# GLOBAL OPTIMIZATION PASS {pi+1}/{global_passes}")
        print(f"{'#'*80}")

        # Phase 6: fine tuning around current promoted best.
        update_base(current_best_params)
        p, f = run_phase(f"Phase 6: Fine-tuning [pass {pi+1}/{global_passes}]",
                         gen_fine_tuning(current_best_params), binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        top = get_top_n_params(results, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

        # Phase 7: random exploration around current promoted best.
        update_base(current_best_params)
        n_random = PHASE7_RANDOM_COUNT
        p, f = run_phase(f"Phase 7: Random neighbors ({n_random}) [pass {pi+1}/{global_passes}]",
                         gen_random_neighbors(current_best_params, n_random, objective,
                                              seed_offset=7000 + pi),
                         binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        top = get_top_n_params(results, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

        # Phase 8: random exploitation around promoted best from Phase 7.
        update_base(current_best_params)
        n_random = PHASE8_RANDOM_COUNT
        p, f = run_phase(f"Phase 8: Random exploitation ({n_random}) [pass {pi+1}/{global_passes}]",
                         gen_random_neighbors(current_best_params, n_random, objective,
                                              profile_override="base_exploit",
                                              seed_offset=9000 + pi),
                         binary, results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f
        top = get_top_n_params(results, n=1, objective=objective)
        if top:
            current_best_params = top[0]
            update_base(current_best_params)

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

