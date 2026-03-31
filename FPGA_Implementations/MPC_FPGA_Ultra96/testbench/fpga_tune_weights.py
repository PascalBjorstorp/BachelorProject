#!/usr/bin/env python3
"""
FPGA MPC Weight Tuning for Hardware Map (Realistic FPGA Sweep)
==============================================================
Systematically sweeps MPC weights, horizons, and solver parameters on the FPGA
using a hardware-oriented workflow aligned with the CPU realistic tuner
(tune_realistic_v2.py).

Since FPGA uses compile-time constants, this script modifies the centralized
constants header (mpc_fpga_constants.h), recompiles, runs the test, and collects results.

The sweep runs 7 phases:
    Phase 1: One-at-a-time parameter sensitivity
    Phase 2: Primary grid (Q_LAT x Q_HDG x Q_VEL x N x dt)
    Phase 3: (Skipped for Hardware - wall margin is fixed)
    Phase 4: Secondary grid (Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE)
    Phase 5: Solver parameters (ADMM_RHO x ADMM_RHO_U x ADMM_TOL)
    Phase 6: Fine-tuning around best config
    Phase 7: Random neighbor exploration

Top 1 configuration cascades to subsequent phases (CASCADE_TOP_N=1).

Usage:
    python3 fpga_tune_weights.py                # Full Hardware sweep (default)
    python3 fpga_tune_weights.py --jobs 8       # Use 8 parallel workers
    python3 fpga_tune_weights.py --objective tracker  # Optimize for tracking (default)
    python3 fpga_tune_weights.py --raceline my_track_raceline.csv
"""

import subprocess
import os
import sys
import csv
import re
import shutil
import time
import random
import hashlib
import itertools
import multiprocessing
from datetime import datetime
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, wait, FIRST_COMPLETED

# ==============================================================================
# PROJECT LAYOUT
# ==============================================================================
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
HEADER = PROJECT_DIR / "include" / "mpc_fpga_constants.h"
HEADER_BAK = HEADER.with_suffix(".h.tuning_backup")

C_SRC_FILES = [
    "testbench/test_fpga_sim_drive.c",
    "src/riccati_solver_hls.c",
    "src/mpc_riccati_hls.c",
    "src/vehicle_model_hls.c",
    "src/fp_math_hls.c",
]
CPP_SRC_FILES = ["src/mpc_fpga_top.cpp"]
BINARY = PROJECT_DIR / "test_fpga_tune"
C_FLAGS = ["-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall", "-Wno-unknown-pragmas"]
CXX_FLAGS = ["-D_GNU_SOURCE", "-O2", "-Wall", "-Wno-unknown-pragmas"]

# Parallel worker scratch area
WORKER_ROOT = PROJECT_DIR / ".fpga_tune_workers"
_WORKER_CTX = None

# ==============================================================================
# HARDWARE MAP CONFIGURATION
# ==============================================================================
WORKSPACE_ROOT = PROJECT_DIR.parent.parent  # BachelorProject/
TRAJ_DIR = WORKSPACE_ROOT / "f1tenth_planning" / "trajectories"

DEFAULT_RACELINE_NAME = "my_track_raceline.csv"
RACELINE_PATH = TRAJ_DIR / DEFAULT_RACELINE_NAME
RACELINE_TAG = "my_track"
HARDWARE_MAP_YAML = str(WORKSPACE_ROOT / "f1tenth_sim" / "maps" / "my_track_map.yaml")
BODY_SAFETY_MARGIN_DEFAULT = "0.00"
WALL_MARGIN = 0.20

# ==============================================================================
# BEST CONFIGURATION — Starting point for all sweeps (USER'S FPGA-TUNED BASELINE)
# ==============================================================================
BASE_CONFIG = {
    # Structural / timing (user's best FPGA config)
    "dt":               0.04,
    "N":                10,
    "MAX_ADMM_ITER":    20,
    # Wall constraints (canonical: WALL_END = HORIZON, WALL_START = 1, WALL_STRIDE = 1)
    "WALL_START":       1,
    "WALL_STRIDE":      1,
    "WALL_END":         10,  # Will be derived as N during apply_params
    "WALL_MARGIN":      0.20,
    # State weights (user's best FPGA values)
    "Q_LAT":            9624.862036,
    "Q_HDG":            401.74426,
    "Q_VEL":            160.0,
    "Q_LAT_VEL":        24.0,
    "Q_YAW":            19.238172,
    # Control weights
    "R_STEER":          0.05,
    "R_ACCEL":          0.01,
    "W_JERK":           0.005,
    "W_ACCEL_RATE":     0.08,
    "W_DELTA_ACT":      0.1,
    # Solver (user's best FPGA)
    "ADMM_RHO":         32.0,
    "ADMM_RHO_U":       24.800802,
    "ADMM_TOL":         5.0,
    # Model/constraint limits
    "MIN_LIN_VEL":      1.0,
    "WP_ADVANCE_MAX":   10,
    "STABILITY_LIMIT":  0.95,
    "C_SHAPE":          1.9,
}

# ==============================================================================
# SWEEP VALUE RANGES — PHASE 2 (Primary Grid)
# ==============================================================================
PHASE2_VALUES = {
    # dt: prediction timestep (FPGA horizons: narrowed for FPGA hardware)
    "dt": [0.035, 0.04, 0.045],
    # N: prediction horizon steps
    "N": [8, 10, 12],
    # Q_LAT: lateral error weight (centered on user's best 9624.86)
    "Q_LAT": [8000, 9000, 9624, 10500, 11500],
    # Q_HDG: heading error weight (centered on user's best 401.74)
    "Q_HDG": [300, 350, 401, 450, 500],
    # Q_VEL: velocity tracking weight (user's best 160)
    "Q_VEL": [120, 140, 160, 180, 200],
}

# ==============================================================================
# SWEEP VALUE RANGES — ALL PARAMETERS (for one-at-a-time and fine-tuning)
# ==============================================================================
FULL_SWEEP_VALUES = {
    "Q_LAT":        [7000, 8000, 9000, 9624, 10500, 11500, 12500, 14000],
    "Q_HDG":        [250, 300, 350, 401, 450, 500, 550, 600],
    "Q_VEL":        [100, 120, 140, 160, 180, 200, 220, 250],
    "Q_LAT_VEL":    [12, 16, 20, 24, 28, 32],
    "Q_YAW":        [12, 16, 19, 22, 26, 30],
    "R_STEER":      [0.02, 0.03, 0.05, 0.08, 0.10, 0.15],
    "R_ACCEL":      [0.005, 0.008, 0.01, 0.012, 0.015],
    "W_JERK":       [0.002, 0.005, 0.01, 0.02, 0.05],
    "W_ACCEL_RATE": [0.04, 0.06, 0.08, 0.10, 0.12],
    "dt":           [0.03, 0.035, 0.04, 0.045, 0.05],
    "N":            [6, 8, 10, 12, 14, 16],
    "ADMM_RHO":     [16, 20, 24, 28, 32, 40, 50],
    "ADMM_RHO_U":   [8, 12, 16, 20, 24, 28],
    "ADMM_TOL":     [3.5, 4.0, 4.5, 5.0, 5.5, 6.0],
}

# ==============================================================================
# PHASE 4: Secondary Grid Values (~1600 configs)
# Q_LAT_VEL x Q_YAW x R_STEER x W_JERK x R_ACCEL x W_ACCEL_RATE
# ==============================================================================
PHASE4_VALUES = {
    "Q_LAT_VEL":    [16, 20, 24, 28],
    "Q_YAW":        [16, 19, 22, 26],
    "R_STEER":      [0.02, 0.05, 0.08, 0.10],
    "W_JERK":       [0.005, 0.01, 0.02, 0.05],
    "R_ACCEL":      [0.008, 0.01, 0.012],
    "W_ACCEL_RATE": [0.06, 0.08, 0.10],
}

# ==============================================================================
# PHASE 5: Solver Grid Values (~1000 configs)
# ADMM_RHO x ADMM_RHO_U x ADMM_TOL
# ==============================================================================
PHASE5_VALUES = {
    "ADMM_RHO":     [20, 28, 32, 40],
    "ADMM_RHO_U":   [12, 16, 20, 24, 28],
    "ADMM_TOL":     [4.0, 4.5, 5.0, 5.5],
}

# ==============================================================================
# RANDOM NEIGHBOR PROFILES
# ==============================================================================
RANDOM_PROFILES = {
    "tracker": {
        "num_perturb_range": (2, 4),
        "default_multipliers": [0.90, 0.95, 1.0, 1.1, 1.2],
        "param_multipliers": {
            "Q_LAT": [0.95, 0.98, 1.0, 1.05, 1.1],
            "Q_HDG": [0.95, 0.98, 1.0, 1.05, 1.1],
            "Q_VEL": [0.9, 0.95, 1.0, 1.05, 1.1],
            "Q_LAT_VEL": [0.85, 0.95, 1.0, 1.1, 1.2],
            "Q_YAW": [0.85, 0.95, 1.0, 1.1, 1.2],
            "R_STEER": [0.85, 0.95, 1.0, 1.15, 1.3],
            "R_ACCEL": [0.8, 0.9, 1.0, 1.2, 1.4],
            "W_JERK": [0.85, 0.95, 1.0, 1.15, 1.3],
            "W_ACCEL_RATE": [0.8, 0.9, 1.0, 1.2, 1.4],
            "ADMM_RHO": [0.85, 0.95, 1.0, 1.1, 1.2],
            "ADMM_RHO_U": [0.85, 0.95, 1.0, 1.1, 1.2],
        },
        "discrete": {
            "N": [6, 8, 10, 12, 14],
            "dt": [0.035, 0.04, 0.045, 0.05],
        },
    },
}

# ==============================================================================
# CONSTANTS
# ==============================================================================
INT_PARAMS = {"N", "WALL_END", "WALL_STRIDE", "WALL_START", "MAX_ADMM_ITER", "WP_ADVANCE_MAX"}

CASCADE_TOP_N = 1  # User specified: follow 1 best config through phases
SEED = 42           # Reproducible randomness

# Parameter → header #define mapping
PARAM_TO_DEFINE = {
    "dt":               "MPC_FPGA_PREDICTION_DT_S",
    "N":                "MPC_FPGA_HORIZON_STEPS",
    "MAX_ADMM_ITER":    "MPC_FPGA_MAX_ADMM_ITER",
    "WALL_START":       "MPC_FPGA_WALL_START",
    "WALL_STRIDE":      "MPC_FPGA_WALL_STRIDE",
    "WALL_END":         "MPC_FPGA_WALL_END",
    "WALL_MARGIN":      "MPC_FPGA_WALL_MARGIN_M",
    "Q_LAT":            "MPC_FPGA_W_LAT_ERROR",
    "Q_HDG":            "MPC_FPGA_W_HEADING",
    "Q_VEL":            "MPC_FPGA_W_VELOCITY",
    "Q_LAT_VEL":        "MPC_FPGA_W_LAT_VEL",
    "Q_YAW":            "MPC_FPGA_W_YAW_RATE",
    "R_STEER":          "MPC_FPGA_W_STEER_EFF",
    "R_ACCEL":          "MPC_FPGA_W_ACCEL_EFF",
    "W_JERK":           "MPC_FPGA_W_STEER_JERK",
    "W_ACCEL_RATE":     "MPC_FPGA_W_ACCEL_RATE",
    "W_DELTA_ACT":      "MPC_FPGA_W_DELTA_ACT",
    "ADMM_RHO":         "MPC_FPGA_ADMM_RHO",
    "ADMM_RHO_U":       "MPC_FPGA_ADMM_RHO_U",
    "ADMM_TOL":         "MPC_FPGA_ADMM_TOL",
    "MIN_LIN_VEL":      "MPC_FPGA_MIN_LIN_VEL_MPS",
    "WP_ADVANCE_MAX":   "MPC_FPGA_WP_ADVANCE_MAX",
    "STABILITY_LIMIT":  "MPC_FPGA_STABILITY_LIMIT",
}

# Ordered printing (match mpc_types.h order for consistency)
MPC_TYPES_PRINT_ORDER = (
    "Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
    "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE",
    "dt", "N", "MAX_ADMM_ITER", "WALL_MARGIN",
    "ADMM_TOL", "ADMM_RHO", "ADMM_RHO_U",
    "WALL_END", "WALL_STRIDE", "WALL_START",
)

# Working copy of base config (modified during cascade)
BASE = {}

# ==============================================================================
# UTILITY FUNCTIONS
# ==============================================================================

def infer_raceline_tag(path: str) -> str:
    """Create a compact label for CSV rows."""
    stem = os.path.splitext(os.path.basename(path))[0]
    if stem.endswith("_raceline"):
        stem = stem[: -len("_raceline")]
    return stem or "unknown"


def resolve_raceline_path(path_arg: str) -> str:
    """Resolve raceline path argument to absolute path."""
    if os.path.isabs(path_arg):
        return path_arg

    traj_candidate = TRAJ_DIR / path_arg
    if traj_candidate.exists():
        return str(traj_candidate.resolve())

    proj_candidate = PROJECT_DIR / path_arg
    return str(proj_candidate.resolve())


def iter_ordered_base_keys():
    """Yield BASE keys in mpc_types.h-inspired order."""
    seen = set()
    for key in MPC_TYPES_PRINT_ORDER:
        if key in BASE:
            seen.add(key)
            yield key
    for key in BASE.keys():
        if key not in seen:
            yield key


def canonicalize_params(params: dict) -> dict:
    """Normalize params to the values MPC actually receives."""
    out = dict(params)
    
    # Hardware policy: WALL_END = HORIZON (N), WALL_START = 1, WALL_STRIDE = 1
    n = int(float(out.get("N", BASE.get("N", 10))))
    n = max(2, n)
    out["N"] = n
    out["WALL_END"] = n
    out["WALL_START"] = 1
    out["WALL_STRIDE"] = 1
    out["WALL_MARGIN"] = float(WALL_MARGIN)
    
    # Integer params
    for k in INT_PARAMS:
        if k in out:
            out[k] = int(float(out[k]))
    
    return out


def is_valid_config(params: dict) -> bool:
    """Check if configuration is valid."""
    n = int(params.get("N", BASE.get("N", 10)))
    if n < 2 or n > 50:
        return False
    return True


def config_hash(params: dict) -> str:
    """Create unique hash for config to avoid duplicates."""
    eff = canonicalize_params(params)
    key = tuple(sorted((k, round(v, 4) if isinstance(v, float) else v)
                       for k, v in eff.items()))
    return hashlib.md5(str(key).encode()).hexdigest()[:12]


# ==============================================================================
# HEADER FILE MODIFICATION
# ==============================================================================

def set_header_param(header_text: str, param: str, value) -> str:
    """Modify a #define in header text."""
    define_name = PARAM_TO_DEFINE[param]
    
    def as_float_literal(v) -> str:
        txt = f"{float(v):.12g}"
        if "e" not in txt and "E" not in txt and "." not in txt:
            txt += ".0"
        return txt + "f"

    if param in INT_PARAMS:
        pattern = rf"#define\s+{define_name}\s+\d+"
        replacement = f"#define {define_name:<24s} {int(value)}"
        return re.sub(pattern, replacement, header_text)
    else:
        pattern = rf"#define\s+{define_name}\s+[-+0-9.eEfF]+"
        replacement = f"#define {define_name:<32s} {as_float_literal(value)}"
        return re.sub(pattern, replacement, header_text)


def apply_params(params: dict, header_path = HEADER, header_bak_path = HEADER_BAK, 
                 base_params: dict = None):
    """Write modified header with given parameters."""
    if base_params is None:
        base_params = BASE_CONFIG

    header_text = header_bak_path.read_text()

    for param, value in params.items():
        if param in PARAM_TO_DEFINE:
            header_text = set_header_param(header_text, param, value)

    # Hardware policy: fixed wall geometry derived from horizon
    n = max(2, int(params.get("N", base_params["N"])))
    header_text = set_header_param(header_text, "WALL_START", 1)
    header_text = set_header_param(header_text, "WALL_STRIDE", 1)
    header_text = set_header_param(header_text, "WALL_END", n)

    header_path.write_text(header_text)


# ==============================================================================
# COMPILE & RUN
# ==============================================================================

def compile_test(project_dir = PROJECT_DIR, binary_path = BINARY) -> bool:
    """Compile test binary."""
    include_flag = f"-I{project_dir / 'include'}"
    build_dir = project_dir / ".fpga_tune_build"
    build_dir.mkdir(parents=True, exist_ok=True)
    obj_files = []

    for rel in C_SRC_FILES:
        src = project_dir / rel
        obj = build_dir / f"{Path(rel).stem}.o"
        cmd = ["gcc"] + C_FLAGS + [include_flag, "-c", str(src), "-o", str(obj)]
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(project_dir))
        if result.returncode != 0:
            print(f"Compile failed: {src}")
            if result.stderr:
                print(result.stderr.splitlines()[-1])
            return False
        obj_files.append(str(obj))

    for rel in CPP_SRC_FILES:
        src = project_dir / rel
        obj = build_dir / f"{Path(rel).stem}.o"
        cmd = ["g++"] + CXX_FLAGS + [include_flag, "-x", "c++", "-c", str(src), "-o", str(obj)]
        result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(project_dir))
        if result.returncode != 0:
            print(f"Compile failed: {src}")
            if result.stderr:
                print(result.stderr.splitlines()[-1])
            return False
        obj_files.append(str(obj))

    link_cmd = ["g++"] + obj_files + ["-o", str(binary_path), "-lm"]
    link_result = subprocess.run(link_cmd, capture_output=True, text=True, cwd=str(project_dir))
    
    # Set execute permission on binary after linking
    if link_result.returncode == 0:
        import stat
        binary_path.chmod(binary_path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    
    return link_result.returncode == 0


def run_test(params: dict, raceline: str = None,
             project_dir = PROJECT_DIR,
             header_path = HEADER,
             header_bak_path = HEADER_BAK,
             binary_path = BINARY,
             base_params: dict = None) -> dict:
    """Apply params, compile, run, return parsed results."""
    if base_params is None:
        base_params = BASE_CONFIG

    apply_params(params, header_path=header_path, header_bak_path=header_bak_path,
                 base_params=base_params)

    if not compile_test(project_dir=project_dir, binary_path=binary_path):
        return {"status": "COMPILE_FAIL", "passed": 0, "failed": 6}

    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"
    env["REALISTIC_SIM"] = "1"
    env["REALISTIC_TIRES"] = "1"
    env["REALISTIC_DRIVE"] = "1"
    env["REALISTIC_DELAY"] = "1"
    env["REALISTIC_NOISE"] = "1"
    env["MAP_YAML"] = HARDWARE_MAP_YAML
    env["BODY_SAFETY_MARGIN"] = BODY_SAFETY_MARGIN_DEFAULT
    env["BODY_SAFETY_MARGIN_M"] = BODY_SAFETY_MARGIN_DEFAULT

    if raceline:
        env["RACELINE"] = str(raceline)
        env["RACELINE_PATH"] = str(raceline)

    try:
        result = subprocess.run(
            [str(binary_path)],
            capture_output=True, text=True, timeout=180, env=env,
            cwd=str(project_dir)
        )
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "passed": 0, "failed": 6}

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
                    "avg_vx": float(parts[15]) if len(parts) > 15 else 0.0,
                }
            except (IndexError, ValueError):
                pass

    if result.returncode != 0:
        return {"status": "EXIT_FAIL", "return_code": result.returncode, "passed": 0, "failed": 6}
    return {"status": "NO_CSV", "return_code": result.returncode, "passed": 0, "failed": 6}


# ==============================================================================
# SCORING
# ==============================================================================

def is_safe_result(r: dict) -> bool:
    """True when run is valid and collision-free."""
    return r.get("status") == "OK" and int(r.get("wall_collisions", 999)) == 0


def compute_tracker_score(r: dict) -> float:
    """Tracker score: minimize trajectory-following errors (lower is better)."""
    if not is_safe_result(r):
        if r.get("status") != "OK":
            return 5000.0
        return 2000.0 + 100.0 * float(r.get("wall_collisions", 0))

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
        return 2000.0 + 100.0 * float(r.get("wall_collisions", 0))

    return round(-r.get("avg_vx", 0.0), 6)


def apply_scores(r: dict, objective: str) -> dict:
    """Attach scores to result row."""
    r["tracker_score"] = compute_tracker_score(r)
    r["fastest_score"] = compute_fastest_score(r)
    r["score"] = r["tracker_score"] if objective == "tracker" else r["fastest_score"]
    return r


# ==============================================================================
# WORKER SETUP (for parallel)
# ==============================================================================

def _setup_worker_context() -> dict:
    """Create one isolated workspace per process for safe parallel compilation."""
    global _WORKER_CTX
    if _WORKER_CTX is not None:
        return _WORKER_CTX

    pid = os.getpid()
    worker_dir = WORKER_ROOT / f"worker_{pid}"
    if worker_dir.exists():
        shutil.rmtree(worker_dir)

    worker_dir.mkdir(parents=True, exist_ok=True)
    for rel in ("include", "src", "testbench"):
        shutil.copytree(PROJECT_DIR / rel, worker_dir / rel)

    header = worker_dir / "include" / "mpc_fpga_constants.h"
    header_bak = header.with_suffix(".h.tuning_backup")
    shutil.copy2(header, header_bak)
    binary = worker_dir / "test_fpga_tune"

    _WORKER_CTX = {
        "project_dir": worker_dir,
        "header": header,
        "header_bak": header_bak,
        "binary": binary,
    }
    return _WORKER_CTX


def _run_single(args):
    """Worker: run one test and return scored result."""
    label, params, raceline, objective, base_params = args
    ctx = _setup_worker_context()
    r = run_test(params, raceline=raceline, 
                 project_dir=ctx["project_dir"],
                 header_path=ctx["header"],
                 header_bak_path=ctx["header_bak"],
                 binary_path=ctx["binary"],
                 base_params=base_params)
    r = apply_scores(r, objective)
    r["label"] = label
    r.update(canonicalize_params(params))
    return r


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
    """Phase 2: Primary grid sweep."""
    combos = []

    for dt in PHASE2_VALUES["dt"]:
        for n in PHASE2_VALUES["N"]:
            for ql in PHASE2_VALUES["Q_LAT"]:
                for qh in PHASE2_VALUES["Q_HDG"]:
                    for qv in PHASE2_VALUES["Q_VEL"]:
                        w = dict(BASE)
                        w["dt"] = dt
                        w["N"] = n
                        w["Q_LAT"] = ql
                        w["Q_HDG"] = qh
                        w["Q_VEL"] = qv

                        if is_valid_config(w):
                            combos.append((f"dt={dt}+N={n}+QL={ql}+QH={qh}+QV={qv}", w))

    return combos


def gen_secondary_grid() -> list:
    """Phase 4: Secondary parameters grid."""
    combos = []

    for qlv in PHASE4_VALUES["Q_LAT_VEL"]:
        for qy in PHASE4_VALUES["Q_YAW"]:
            for rs in PHASE4_VALUES["R_STEER"]:
                for wj in PHASE4_VALUES["W_JERK"]:
                    for ra in PHASE4_VALUES["R_ACCEL"]:
                        for war in PHASE4_VALUES["W_ACCEL_RATE"]:
                            w = dict(BASE)
                            w["Q_LAT_VEL"] = qlv
                            w["Q_YAW"] = qy
                            w["R_STEER"] = rs
                            w["W_JERK"] = wj
                            w["R_ACCEL"] = ra
                            w["W_ACCEL_RATE"] = war
                            combos.append((f"QLV={qlv}+QY={qy}+RS={rs}+WJ={wj}+RA={ra}+WAR={war}", w))

    return combos


def gen_solver_grid() -> list:
    """Phase 5: Solver parameters grid."""
    combos = []

    for rho in PHASE5_VALUES["ADMM_RHO"]:
        for rho_u in PHASE5_VALUES["ADMM_RHO_U"]:
            for tol in PHASE5_VALUES["ADMM_TOL"]:
                w = dict(BASE)
                w["ADMM_RHO"] = rho
                w["ADMM_RHO_U"] = rho_u
                w["ADMM_TOL"] = tol
                combos.append((f"rho={rho}+ru={rho_u}+tol={tol}", w))

    return combos


def gen_fine_tuning(best_weights: dict) -> list:
    """Phase 6: Fine-tuning around best config."""
    combos = []
    pct_range = (0.80, 0.85, 0.90, 0.92, 0.95, 0.97, 1.03, 1.05, 1.08, 1.10, 1.15, 1.20)
    skip = {"MAX_ADMM_ITER", "N", "WALL_STRIDE", "WALL_END", "WALL_MARGIN"}

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

    return combos


def gen_random_neighbors(best_weights: dict, n: int, objective: str) -> list:
    """Phase 7: Random perturbations around best config."""
    combos = []
    rng = random.Random(SEED)
    profile = RANDOM_PROFILES.get(objective, RANDOM_PROFILES["tracker"])

    discrete = profile.get("discrete", {})
    param_multipliers = profile.get("param_multipliers", {})
    default_multipliers = profile.get("default_multipliers", [0.85, 0.95, 1.0, 1.1, 1.2])
    min_perturb, max_perturb = profile.get("num_perturb_range", (2, 4))

    tune_params = [k for k in best_weights.keys()
                   if k not in ("MAX_ADMM_ITER", "WALL_END", "WALL_STRIDE", "WALL_MARGIN")
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
                if name == "N":
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
# PHASE RUNNER
# ==============================================================================

def run_phase(phase_name: str, combos: list, results: list,
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
        # Sequential
        for i, (label, params) in enumerate(combos):
            elapsed = time.time() - t0
            rate = max(len(results), 1) / max(elapsed, 0.01)
            eta = (total - i - 1) / max(rate, 0.01)
            print(f"  [{i+1:4d}/{total}] {label:55s} ", end="", flush=True)

            r = run_test(params, raceline=str(RACELINE_PATH), base_params=BASE)
            r = apply_scores(r, objective)
            r["label"] = label
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
                      f"lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")
    else:
        # Parallel
        done_count = 0
        with ProcessPoolExecutor(max_workers=num_workers) as executor:
            it = ((label, params, str(RACELINE_PATH), objective, BASE) for label, params in combos)
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
                              f"lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")

    return passed, failed


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
            print("  WARNING: No safe candidates yet; using least-bad configs for cascade.")
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
                  f"(score={r.get('score', 0.0):.2f}, avx={r.get('avg_vx', 0):.2f})")

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
    objective = "tracker"
    raceline_override = None

    for i, arg in enumerate(sys.argv[1:]):
        if arg in ("--jobs", "-j") and i + 1 < len(sys.argv) - 1:
            try:
                num_workers = int(sys.argv[i + 2])
            except ValueError:
                num_workers = multiprocessing.cpu_count()
            if num_workers <= 0:
                num_workers = multiprocessing.cpu_count()
        if arg == "--objective" and i + 1 < len(sys.argv) - 1:
            objective = sys.argv[i + 2].strip().lower()
        if arg == "--raceline" and i + 1 < len(sys.argv) - 1:
            raceline_override = sys.argv[i + 2].strip()

    if objective not in ("tracker", "fastest"):
        print("ERROR: --objective must be 'tracker' or 'fastest'")
        sys.exit(1)

    if raceline_override:
        RACELINE_PATH = resolve_raceline_path(raceline_override)
    else:
        RACELINE_PATH = str(RACELINE_PATH.resolve())
    RACELINE_TAG = infer_raceline_tag(RACELINE_PATH)

    # Initialize BASE with user's best config
    BASE.update(BASE_CONFIG)

    print(f"\n{'='*80}")
    print("FPGA MPC Weight Tuning - Hardware Map (7-Phase Cascade)")
    print(f"{'='*80}")
    print(f"  Workers:     {num_workers}")
    print(f"  Objective:   {objective}")
    print(f"  Cascade:     top {CASCADE_TOP_N}")
    print(f"  Raceline:    {RACELINE_PATH}")
    print(f"  Raceline tag:{RACELINE_TAG}")

    os.chdir(PROJECT_DIR)

    # Backup and build
    if not HEADER_BAK.exists():
        shutil.copy2(HEADER, HEADER_BAK)
        print("  Created header backup")

    print("\nBuilding test binary...")
    if not compile_test():
        print("BUILD FAILED")
        sys.exit(1)
    print("  Build OK")

    if not Path(RACELINE_PATH).exists():
        print(f"ERROR: Raceline not found: {RACELINE_PATH}")
        sys.exit(1)

    # Setup
    results = []
    t0 = time.time()
    total_p = total_f = 0

    # CSV writer
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"tuning_results/fpga_tuning_hardware_{objective}_{timestamp}.csv"
    Path("tuning_results").mkdir(exist_ok=True)
    fieldnames = (
        ["label", "score", "tracker_score", "fastest_score",
         "passed", "failed", "max_lat_err", "avg_lat_err",
         "max_hdg_err", "avg_hdg_err", "max_vx", "avg_vx",
         "avg_vel_err", "max_vel_err", "avg_solve_us", "max_solve_us",
         "wall_collisions", "time_above_5ms", "avg_iters", "status"]
        + list(BASE.keys())
    )
    csv_writer = IncrementalCSV(outfile, fieldnames)
    print(f"  Results: {outfile}\n")

    # ========== PHASE 1: One-at-a-time ==========
    p, f = run_phase("Phase 1: One-at-a-time sensitivity",
                     gen_one_at_a_time(), results, t0,
                     num_workers, csv_writer, objective)
    total_p += p
    total_f += f

    # ========== PHASE 2: Primary grid ==========
    combos = gen_primary_grid()
    print(f"\n  Phase 2 will test {len(combos):,} configurations")
    p, f = run_phase("Phase 2: Primary grid (dt x N x Q_LAT x Q_HDG x Q_VEL)",
                     combos, results, t0,
                     num_workers, csv_writer, objective)
    total_p += p
    total_f += f

    # Get top N for cascade
    print("\n  Selecting top configs for cascade...")
    top_configs = get_top_n_params(results)
    if not top_configs:
        top_configs = [dict(BASE)]

    # ========== PHASES 3-7: Cascade from top configs ==========
    for ci, cascade_base in enumerate(top_configs):
        print(f"\n{'#'*80}")
        print(f"# CASCADE BRANCH {ci+1}/{len(top_configs)}")
        print(f"{'#'*80}")

        update_base(cascade_base)

        # Phase 3: Skipped (wall margin fixed)
        print("\n  Phase 3: Skipped (wall margin is fixed for hardware)")

        # Phase 4: Secondary grid
        p, f = run_phase(f"Phase 4: Secondary grid [branch {ci+1}]",
                         gen_secondary_grid(), results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f

        # Update to best
        top = get_top_n_params(results, n=1)
        if top:
            update_base(top[0])

        # Phase 5: Solver grid
        p, f = run_phase(f"Phase 5: Solver parameters [branch {ci+1}]",
                         gen_solver_grid(), results, t0,
                         num_workers, csv_writer, objective)
        total_p += p
        total_f += f

        # Update to best
        top = get_top_n_params(results, n=1)
        if top:
            update_base(top[0])

        # Phase 6: Fine-tuning
        best = get_top_n_params(results, n=1)
        if best:
            p, f = run_phase(f"Phase 6: Fine-tuning [branch {ci+1}]",
                             gen_fine_tuning(best[0]), results, t0,
                             num_workers, csv_writer, objective)
            total_p += p
            total_f += f

        # Phase 7: Random neighbors
        best = get_top_n_params(results, n=1)
        if best:
            n_random = 2000
            p, f = run_phase(f"Phase 7: Random neighbors ({n_random}) [branch {ci+1}]",
                             gen_random_neighbors(best[0], n_random, objective),
                             results, t0,
                             num_workers, csv_writer, objective)
            total_p += p
            total_f += f

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

        fmt = "{:<4} {:<45} {:>8} {:>6} {:>6} {:>6} {:>3}"
        print(fmt.format("Rank", "Label", "Score", "AvgVx", "AvgLat", "AvgVE", "WC"))
        print("-" * 90)

        top = sorted(safe, key=lambda x: x.get("score", 999999.0))[:20]
        for i, r in enumerate(top):
            print(fmt.format(
                i+1,
                r['label'][:45],
                f"{r.get('score', 0.0):.2f}",
                f"{r.get('avg_vx', 0.0):.2f}",
                f"{r['avg_lat_err']:.4f}",
                f"{r['avg_vel_err']:.2f}",
                f"{r.get('wall_collisions', '-')}"
            ))

        best = top[0]
        print(f"\nBEST CONFIGURATION:")
        print(f"  Score: {best.get('score', 0.0):.2f}")
        print(f"  Avg velocity: {best.get('avg_vx', 0.0):.2f} m/s")
        print(f"  Avg lat err: {best['avg_lat_err']:.4f} m")
        print(f"  ---")
        for k in iter_ordered_base_keys():
            print(f"  {k:15s} = {best.get(k, BASE[k])}")

    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
