#!/usr/bin/env python3
"""
FPGA MPC Weight Tuning Script — Hardware Realistic Sweep
========================================================
Systematically tests parameter combinations for the FPGA Riccati-ADMM MPC
using a hardware-oriented workflow aligned with the CPU realistic tuner.
Since FPGA uses compile-time constants, this script modifies the centralized
constants header (mpc_fpga_constants.h), recompiles, runs the test, and
collects results.

Sweep parameters:
    - dt (prediction timestep): affects horizon spacing and model discretization
  - N  (horizon steps): affects how many prediction steps (array sizes)
  - WALL_END, WALL_MARGIN: wall constraint parameters
  - Q_LAT, Q_HDG, Q_VEL, Q_LAT_VEL, Q_YAW: state tracking weights
  - R_STEER, W_JERK, W_ACCEL_RATE, W_DELTA_ACT: control weights
  - ADMM_RHO, ADMM_RHO_U: solver parameters

Usage:
    python3 fpga_tune_weights.py Hardware          # Full Hardware sweep (default)
    python3 fpga_tune_weights.py --quick             # Quick sweep
    python3 fpga_tune_weights.py --raceline hardware  # Sweep specific hardware raceline
    python3 fpga_tune_weights.py --all-racelines     # Sweep all racelines sequentially
    python3 fpga_tune_weights.py --phase 1           # Phase 1 only (dt × N × WALL)
    python3 fpga_tune_weights.py --single dt=0.05 N=20 Q_VEL=80   # Single test
"""

import subprocess
import os
import sys
import csv
import re
import shutil
import time
import itertools
import random
from datetime import datetime
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing

# ─── Project layout ──────────────────────────────────────────────────────────
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
CPP_SRC_FILES = [
    "src/mpc_fpga_top.cpp",
]
BINARY = PROJECT_DIR / "test_fpga_tune"
C_FLAGS = ["-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall", "-Wno-unknown-pragmas"]
CXX_FLAGS = ["-D_GNU_SOURCE", "-O2", "-Wall", "-Wno-unknown-pragmas"]

# Parallel worker scratch area (created only when --jobs > 1)
WORKER_ROOT = PROJECT_DIR / ".fpga_tune_workers"
_WORKER_CTX = None

# ─── Mode-specific racelines (aligned with CPU realistic tuner) ─────────────
WORKSPACE_ROOT = PROJECT_DIR.parent.parent  # BachelorProject/
TRAJ_DIR = WORKSPACE_ROOT / "f1tenth_planning" / "trajectories"
HARDWARE_RACELINES = {
    "hardware": str(TRAJ_DIR / "my_track_raceline.csv"),
}
HARDWARE_MAP_YAML = str(WORKSPACE_ROOT / "f1tenth_sim" / "maps" / "my_track_map.yaml")
BODY_SAFETY_MARGIN_DEFAULT = "0.00"

# Per-raceline WALL_MARGIN defaults (matching CPU realistic sweeps)
HARDWARE_PER_RACELINE_WM = {
    "hardware": 0.20,
}

# ─── Current baseline (synced with CPU realistic sweep results) ─────────────
HARDWARE_BASE_PARAMS = {
    # Structural / timing
    "dt":               0.03,
    "N":                14,
    "MAX_ADMM_ITER":    20,
    # Wall constraints
    "WALL_START":       1,
    "WALL_STRIDE":      1,
    "WALL_END":         14,
    "WALL_MARGIN":      0.20,
    # State weights
    "Q_LAT":            6500.0,
    "Q_HDG":            1200.0,
    "Q_VEL":            24.0,
    "Q_LAT_VEL":        24.0,
    "Q_YAW":            22.0,
    # Control weights
    "R_STEER":          0.05,
    "R_ACCEL":          0.01,
    "W_JERK":           0.005,
    "W_ACCEL_RATE":     0.08,
    "W_DELTA_ACT":      0.1,
    # Solver
    "ADMM_RHO":         32.0,
    "ADMM_RHO_U":       20.0,
    "ADMM_TOL":         5.0,
    # Model/constraint limits
    "MIN_LIN_VEL":      1.0,
    "WP_ADVANCE_MAX":   10,
    "STABILITY_LIMIT":  0.95,
    "C_SHAPE":         1.9,
}

# Map parameter names → header #define names
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

# ─── Sweep ranges (THOROUGH) ─────────────────────────────────────────────────
HARDWARE_FULL_VALUES = {
    # Structural / timing
    "dt":               [0.03, 0.035, 0.04],
    "N":                [12, 14, 16],
    "MAX_ADMM_ITER":    [20],
    # Wall constraints
    "WALL_START":       [1],
    "WALL_STRIDE":      [1],
    "WALL_END":         [14],
    "WALL_MARGIN":      [0.20],
    # State weights
    "Q_LAT":            [5000, 6500, 8000, 9500, 11000],
    "Q_HDG":            [800, 1000, 1200, 1400],
    "Q_VEL":            [16, 18, 20, 22, 24],
    "Q_LAT_VEL":        [20, 24, 28],
    "Q_YAW":            [20, 22, 24],
    # Control weights
    "R_STEER":          [0.05, 0.10, 0.20],
    "W_JERK":           [0.005, 0.01, 0.02],
    "W_ACCEL_RATE":     [0.05, 0.08, 0.10],
    "W_DELTA_ACT":      [0.05, 0.10, 0.20],
    # Solver
    "ADMM_RHO":         [24, 32, 40],
    "ADMM_RHO_U":       [12, 20, 28],
    "ADMM_TOL":         [4.5, 5.0, 5.5],
    # Model limits (kept constant for all tests)
    "MIN_LIN_VEL":      [1.0],
    "WP_ADVANCE_MAX":   [10],
    "STABILITY_LIMIT":  [0.95],
}

HARDWARE_QUICK_VALUES = {
    "dt":               [0.03],
    "N":                [14],
    "MAX_ADMM_ITER":    [20],
    "WALL_START":       [1],
    "WALL_STRIDE":      [1],
    "WALL_END":         [14],
    "WALL_MARGIN":      [0.20],
    "Q_LAT":            [6500, 8000],
    "Q_HDG":            [1000, 1200],
    "Q_VEL":            [18, 20, 22, 24],
    "Q_LAT_VEL":        [24],
    "Q_YAW":            [22],
    "R_STEER":          [0.05, 0.10],
    "W_JERK":           [0.005, 0.01],
    "W_ACCEL_RATE":     [0.08],
    "W_DELTA_ACT":      [0.10, 0.20],
    "ADMM_RHO":         [32],
    "ADMM_RHO_U":       [20],
    "ADMM_TOL":         [5.0],
    "MIN_LIN_VEL":      [1.0],
    "WP_ADVANCE_MAX":   [10],
    "STABILITY_LIMIT":  [0.95],
}

CASCADE_TOP_N = 1
ACTIVE_SWEEP_KEYS = set()


# ═══════════════════════════════════════════════════════════════════════════════
#  Header file modification
# ═══════════════════════════════════════════════════════════════════════════════

def set_header_param(header_text: str, param: str, value) -> str:
    """Modify a #define in the header text. Returns modified text."""
    define_name = PARAM_TO_DEFINE[param]

    def as_float_literal(v) -> str:
        txt = f"{float(v):.12g}"
        if "e" not in txt and "E" not in txt and "." not in txt:
            txt += ".0"
        return txt + "f"

    if param in ("N", "MAX_ADMM_ITER", "WALL_END", "WALL_START",
                   "WALL_STRIDE", "WP_ADVANCE_MAX"):
        # Integer defines
        pattern = rf"#define\s+{define_name}\s+\d+"
        replacement = f"#define {define_name:<24s} {int(value)}"
        return re.sub(pattern, replacement, header_text)

    else:
        # Float constants in centralized header use plain literals with f suffix.
        pattern = rf"#define\s+{define_name}\s+[-+0-9.eEfF]+"
        replacement = f"#define {define_name:<32s} {as_float_literal(value)}"
        return re.sub(pattern, replacement, header_text)


def apply_params(params: dict, header_path: Path = HEADER, header_bak_path: Path = HEADER_BAK,
                 base_params: dict = None):
    """Write modified header with given parameters."""
    if base_params is None:
        base_params = BASE_PARAMS

    header_text = header_bak_path.read_text()

    for param, value in params.items():
        if param in PARAM_TO_DEFINE:
            header_text = set_header_param(header_text, param, value)

    # Hardware policy: fixed wall geometry, derived from the horizon.
    n = max(2, int(params.get("N", base_params["N"])))
    header_text = set_header_param(header_text, "WALL_START", 1)
    header_text = set_header_param(header_text, "WALL_STRIDE", 1)
    header_text = set_header_param(header_text, "WALL_END", n)

    header_path.write_text(header_text)


# ═══════════════════════════════════════════════════════════════════════════════
#  Compile & Run
# ═══════════════════════════════════════════════════════════════════════════════

def compile_test(project_dir: Path = PROJECT_DIR, binary_path: Path = BINARY) -> bool:
    """Compile the test binary. Returns True on success."""
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
            print(f"Compile failed for {src}")
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
            print(f"Compile failed for {src}")
            if result.stderr:
                print(result.stderr.splitlines()[-1])
            return False
        obj_files.append(str(obj))

    link_cmd = ["g++"] + obj_files + ["-o", str(binary_path), "-lm"]
    link_result = subprocess.run(link_cmd, capture_output=True, text=True, cwd=str(project_dir))
    if link_result.returncode != 0:
        print("Link failed for test binary")
        if link_result.stderr:
            print(link_result.stderr.splitlines()[-1])
    return link_result.returncode == 0


def run_test(params: dict, raceline: str = None,
             project_dir: Path = PROJECT_DIR,
             header_path: Path = HEADER,
             header_bak_path: Path = HEADER_BAK,
             binary_path: Path = BINARY,
             base_params: dict = None) -> dict:
    """Apply params, compile, run, return parsed results."""
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

    if raceline:
        # Keep both names for compatibility with different test binaries.
        env["RACELINE"] = str(raceline)
        env["RACELINE_PATH"] = str(raceline)

    if ACTIVE_MODE == "Hardware":
        env["MAP_YAML"] = HARDWARE_MAP_YAML
        # With occupancy-map body checks enabled, avoid extra inflation by default.
        body_margin = os.environ.get("FPGA_TUNE_BODY_SAFETY_MARGIN", BODY_SAFETY_MARGIN_DEFAULT)
        env["BODY_SAFETY_MARGIN"] = body_margin
        env["BODY_SAFETY_MARGIN_M"] = env["BODY_SAFETY_MARGIN"]

    cmd = [str(binary_path)]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True, text=True, timeout=180, env=env,
            cwd=str(project_dir)
        )
    except subprocess.TimeoutExpired:
        return {"status": "TIMEOUT", "passed": 0, "failed": 6}

    # Parse CSV line
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


# ═══════════════════════════════════════════════════════════════════════════════
#  Scoring
# ═══════════════════════════════════════════════════════════════════════════════

def compute_score(r: dict) -> float:
    """Composite score (lower = better)."""
    if r["status"] != "OK":
        return 999.0

    if r["wall_collisions"] > 0:
        if ACTIVE_MODE == "Hardware":
            # Differentiate crashing runs by severity/progress so cascade
            # doesn't treat all crashes as equally good.
            crash_score = (
                400.0
                + 120.0 * float(r.get("wall_collisions", 0))
                + max(0.0, 8.0 - float(r.get("time_above_5ms", 0.0))) * 8.0
                + float(r.get("avg_lat_err", 0.0)) * 120.0
                + float(r.get("max_lat_err", 0.0)) * 40.0
                + float(r.get("avg_hdg_err", 0.0)) * 30.0
                + max(0.0, 5.0 - float(r.get("max_vx", 0.0))) * 6.0
                + float(r.get("failed", 0)) * 5.0
                + float(r.get("avg_solve_us", 0.0)) * 0.002
            )
            return round(crash_score, 3)

        return 500.0 + r["wall_collisions"] * 100.0

    if ACTIVE_MODE == "Hardware":
        tracking = (
            r["avg_lat_err"] * 50.0 +
            r["avg_vel_err"] * 20.0 +
            r["max_lat_err"] * 10.0 +
            r["avg_hdg_err"] * 15.0
        )
        # Match tracker intent: sub-5 m/s is not a hard fail, only a soft penalty.
        velocity_penalty = max(0, 5.0 - r["max_vx"]) * 4.0
        failure_penalty = r.get("failed", 0) * 5.0
        solver = r.get("avg_iters", 0) * 0.2 + r["avg_solve_us"] * 0.001
        return round(tracking + velocity_penalty + failure_penalty + solver, 3)

    score = (
        max(0, 12.0 - r["max_vx"]) * 15.0 +
        max(0, 60 - r["time_above_5ms"]) * 2.0 +
        r["avg_lat_err"] * 5.0 +
        r["max_lat_err"] * 1.0 +
        r["avg_vel_err"] * 5.0 +
        r["avg_hdg_err"] * 2.0 +
        r.get("avg_iters", 0) * 0.3 +
        r["avg_solve_us"] * 0.002
    )
    return round(score, 3)


def is_safe_result(r: dict) -> bool:
    """True when run succeeded without wall collision."""
    return r.get("status") == "OK" and int(r.get("wall_collisions", 999)) == 0


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


def _run_single_parallel(job):
    """Worker entrypoint: run one config in an isolated per-process copy."""
    label, params, raceline = job
    ctx = _setup_worker_context()
    r = run_test(
        params,
        raceline=raceline,
        project_dir=ctx["project_dir"],
        header_path=ctx["header"],
        header_bak_path=ctx["header_bak"],
        binary_path=ctx["binary"],
        base_params=BASE_PARAMS,
    )
    r["label"] = label
    r["score"] = compute_score(r)
    r["params"] = dict(params)
    return r


class IncrementalCSV:
    """Appends results after each test for crash/interruption safety."""

    def __init__(self, filepath: Path, fieldnames: list[str]):
        self.filepath = filepath
        self.fieldnames = fieldnames
        self.header_written = False

    def write_row(self, row: dict):
        mode = "a" if self.header_written else "w"
        with self.filepath.open(mode, newline="") as f:
            writer = csv.DictWriter(f, fieldnames=self.fieldnames, extrasaction="ignore")
            if not self.header_written:
                writer.writeheader()
                self.header_written = True
            writer.writerow(row)


# ═══════════════════════════════════════════════════════════════════════════════
#  Combination generators
# ═══════════════════════════════════════════════════════════════════════════════

def gen_structural_sweep(values, base_params=None):
    """Phase 1a: dt × N × WALL_MARGIN (core structural).
    Wall geometry is fixed to start=1, end=horizon, stride=1."""
    if base_params is None:
        base_params = BASE_PARAMS
    combos = []
    for dt in values.get("dt", [base_params["dt"]]):
        for n in values.get("N", [base_params["N"]]):
            for wm in values.get("WALL_MARGIN", [base_params["WALL_MARGIN"]]):
                p = dict(base_params)
                p["dt"] = dt
                p["N"] = n
                p["WALL_START"] = 1
                p["WALL_STRIDE"] = 1
                p["WALL_END"] = int(n)
                p["WALL_MARGIN"] = wm
                label = f"dt={dt}+N={n}+WE={n}+WM={wm}"
                combos.append((label, p))
    return combos


def gen_wall_detail_sweep(best_structural, values):
    """Wall geometry is fixed; no separate wall-detail sweep."""
    return []


def gen_primary_sweep(best_structural, values):
    """Phase 2: Q_LAT × Q_HDG × Q_VEL grid on best structural."""
    combos = []
    for ql in values.get("Q_LAT", [BASE_PARAMS["Q_LAT"]]):
        for qh in values.get("Q_HDG", [BASE_PARAMS["Q_HDG"]]):
            for qv in values.get("Q_VEL", [BASE_PARAMS["Q_VEL"]]):
                p = dict(best_structural)
                p["Q_LAT"] = ql
                p["Q_HDG"] = qh
                p["Q_VEL"] = qv
                label = f"QL={ql}+QH={qh}+QV={qv}"
                combos.append((label, p))
    return combos


def gen_secondary_sweep(best_primary, values):
    """Phase 3: One-at-a-time sweep of secondary weights."""
    combos = []
    secondary = ["Q_LAT_VEL", "Q_YAW", "R_STEER", "R_ACCEL", "W_JERK",
                  "W_ACCEL_RATE", "W_DELTA_ACT", "STABILITY_LIMIT"]

    for wname in secondary:
        for v in values.get(wname, [best_primary.get(wname, BASE_PARAMS[wname])]):
            p = dict(best_primary)
            p[wname] = v
            label = f"best+{wname}={v}"
            combos.append((label, p))

    # Pairwise combos for R_STEER × W_JERK (most coupled)
    for rs in values.get("R_STEER", [BASE_PARAMS["R_STEER"]]):
        for wj in values.get("W_JERK", [BASE_PARAMS["W_JERK"]]):
            p = dict(best_primary)
            p["R_STEER"] = rs
            p["W_JERK"] = wj
            label = f"best+RS={rs}+WJ={wj}"
            combos.append((label, p))

    return combos


def gen_solver_sweep(best_so_far, values):
    """Phase 4: ADMM solver parameters."""
    combos = []
    # One-at-a-time for all solver params
    solver_params = ["ADMM_RHO", "ADMM_RHO_U", "ADMM_TOL", "MAX_ADMM_ITER"]
    for sp in solver_params:
        for v in values.get(sp, [best_so_far.get(sp, BASE_PARAMS[sp])]):
            p = dict(best_so_far)
            p[sp] = v
            label = f"best+{sp}={v}"
            combos.append((label, p))

    # Pairwise RHO × RHO_U
    for rho in values.get("ADMM_RHO", [BASE_PARAMS["ADMM_RHO"]]):
        for rho_u in values.get("ADMM_RHO_U", [BASE_PARAMS["ADMM_RHO_U"]]):
            p = dict(best_so_far)
            p["ADMM_RHO"] = rho
            p["ADMM_RHO_U"] = rho_u
            label = f"best+rho={rho}+rho_u={rho_u}"
            combos.append((label, p))

    return combos


def gen_fine_tune(best_params):
    """Phase 5: ±5%, ±10%, ±25%, ±50% around best."""
    combos = []
    perturbations = [0.50, 0.75, 0.90, 0.95, 1.05, 1.10, 1.25, 1.50]
    tunable = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
               "R_STEER", "W_JERK", "W_DELTA_ACT", "WALL_MARGIN"]

    for wname in tunable:
        base_val = best_params.get(wname, BASE_PARAMS[wname])
        if base_val == 0:
            continue
        for mult in perturbations:
            new_val = round(base_val * mult, 6)
            p = dict(best_params)
            p[wname] = new_val
            pct = int((mult - 1.0) * 100)
            sign = "+" if pct >= 0 else ""
            label = f"FT:{wname}{sign}{pct}%={new_val}"
            combos.append((label, p))

    # Pairwise ±10% for primary weights
    primary = ["Q_LAT", "Q_HDG", "Q_VEL"]
    for w1, w2 in itertools.combinations(primary, 2):
        for m1 in [0.90, 1.10]:
            for m2 in [0.90, 1.10]:
                p = dict(best_params)
                p[w1] = round(best_params[w1] * m1, 2)
                p[w2] = round(best_params[w2] * m2, 2)
                p1 = "+10%" if m1 > 1 else "-10%"
                p2 = "+10%" if m2 > 1 else "-10%"
                label = f"FT:{w1}{p1}+{w2}{p2}"
                combos.append((label, p))

    return combos


def deduplicate(combos):
    """Remove duplicate parameter combinations."""
    seen = set()
    unique = []
    for label, params in combos:
        key = tuple(sorted((k, round(v, 6) if isinstance(v, float) else v)
                           for k, v in params.items()))
        if key not in seen:
            seen.add(key)
            unique.append((label, params))
    return unique


def top_n_param_sets(results, n, base_params):
    """Return up to N unique best parameter dictionaries."""
    if n <= 1:
        return [find_best_params(results, base_params)]

    ranked = sorted(results, key=lambda r: r.get("score", 999))
    safe_ranked = [r for r in ranked if is_safe_result(r)]
    ranked_pool = safe_ranked if safe_ranked else ranked

    out = []
    seen = set()
    for r in ranked_pool:
        p = r.get("params", {})
        if not p:
            continue
        key = param_key(p)
        if key in seen:
            continue
        seen.add(key)
        out.append(dict(p))
        if len(out) >= n:
            break

    if not out:
        out.append(dict(base_params))
    return out


def filter_supported_sweep_values(values: dict, header_text: str) -> dict:
    """Drop sweep keys that are not patchable by the active header."""
    filtered = {}
    dropped = []

    for name, vals in values.items():
        define_name = PARAM_TO_DEFINE.get(name)
        if define_name is None:
            filtered[name] = vals
            continue

        if re.search(rf"#define\s+{re.escape(define_name)}\b", header_text):
            filtered[name] = vals
        else:
            dropped.append(name)

    if dropped:
        print("WARNING: Ignoring unsupported sweep parameters:", ", ".join(sorted(dropped)))
    return filtered


def preflight_validate(mode: str, racelines_to_test: list[tuple[str, str]]):
    """Validate sweep wiring before starting a long run."""
    required_files = [HEADER] + [PROJECT_DIR / s for s in C_SRC_FILES + CPP_SRC_FILES]
    missing = [str(p) for p in required_files if not p.exists()]
    if missing:
        raise RuntimeError("Missing required source/header files:\n  " + "\n  ".join(missing))

    for rl_name, rl_path in racelines_to_test:
        if not Path(rl_path).exists():
            raise RuntimeError(f"Raceline '{rl_name}' not found: {rl_path}")

    if mode == "Hardware" and not Path(HARDWARE_MAP_YAML).exists():
        raise RuntimeError(f"Hardware map yaml not found: {HARDWARE_MAP_YAML}")

    # Fast compile + one baseline run to catch bad wiring early.
    probe_raceline = racelines_to_test[0][1] if racelines_to_test else None
    baseline = run_test(dict(BASE_PARAMS), raceline=probe_raceline)
    if baseline.get("status") in ("COMPILE_FAIL", "NO_CSV", "TIMEOUT"):
        raise RuntimeError(
            "Preflight baseline failed "
            f"(status={baseline.get('status')}). Fix sweep/sim wiring before long runs."
        )


def gen_random_neighbors(best_params, n_samples=150):
    """Phase 6: Random perturbations around the best config.
    Each sample randomly perturbs 2-5 parameters by ±5-40%."""
    random.seed(42)
    combos = []
    tunable = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
               "R_STEER", "W_JERK", "W_ACCEL_RATE", "W_DELTA_ACT",
               "WALL_MARGIN", "ADMM_RHO", "ADMM_RHO_U"]

    for i in range(n_samples):
        p = dict(best_params)
        n_perturb = random.randint(2, 5)
        chosen = random.sample(tunable, min(n_perturb, len(tunable)))
        parts = []
        for wname in chosen:
            base_val = best_params.get(wname, BASE_PARAMS[wname])
            if base_val == 0:
                continue
            mult = random.uniform(0.60, 1.40)
            new_val = round(base_val * mult, 6)
            if new_val <= 0:
                new_val = round(base_val * 0.5, 6)
            p[wname] = new_val
            parts.append(f"{wname}={new_val:.3g}")
        label = f"RND{i+1}:" + "+".join(parts[:3])
        combos.append((label, p))

    return combos


# ═══════════════════════════════════════════════════════════════════════════════
#  Main
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    global ACTIVE_MODE, BASE_PARAMS, CASCADE_TOP_N, ACTIVE_SWEEP_KEYS

    mode = "Hardware"
    for arg in sys.argv[1:]:
        if arg in ("Hardware", "hardware"):
            mode = "Hardware"

    quick = "--quick" in sys.argv
    phase_only = None
    raceline_name = None
    num_workers = 1
    cascade_top = 1
    for i, arg in enumerate(sys.argv):
        if arg == "--phase" and i + 1 < len(sys.argv):
            phase_only = int(sys.argv[i + 1])
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_name = sys.argv[i + 1]
        if arg == "--jobs" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "-j" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "--cascade-top" and i + 1 < len(sys.argv):
            cascade_top = int(sys.argv[i + 1])
    single = "--single" in sys.argv
    all_racelines = "--all-racelines" in sys.argv

    if num_workers <= 0:
        num_workers = multiprocessing.cpu_count()
    CASCADE_TOP_N = max(1, cascade_top)

    ACTIVE_MODE = mode
    if mode == "Hardware":
        BASE_PARAMS = dict(HARDWARE_BASE_PARAMS)
        full_values_mode = HARDWARE_FULL_VALUES
        quick_values_mode = HARDWARE_QUICK_VALUES
        racelines_mode = HARDWARE_RACELINES
        per_raceline_wm_mode = HARDWARE_PER_RACELINE_WM

    os.chdir(str(PROJECT_DIR))

    # Resolve raceline path
    raceline_path = None
    if raceline_name:
        if raceline_name in racelines_mode:
            raceline_path = racelines_mode[raceline_name]
        elif os.path.isfile(raceline_name):
            raceline_path = raceline_name
            raceline_name = Path(raceline_name).stem
        else:
            print(f"ERROR: Unknown raceline '{raceline_name}'. "
                  f"Available: {', '.join(racelines_mode.keys())}")
            return 1

    # Always refresh backup from the current header to avoid stale snapshots
    # carrying over between interrupted runs.
    shutil.copy2(HEADER, HEADER_BAK)

    try:
        header_for_filter = HEADER_BAK.read_text() if HEADER_BAK.exists() else HEADER.read_text()
        raw_values = quick_values_mode if quick else full_values_mode
        values = filter_supported_sweep_values(raw_values, header_for_filter)
        ACTIVE_SWEEP_KEYS = set(values.keys())

        print(f"Workers: {num_workers} ({'sequential' if num_workers == 1 else 'parallel'})")
        print(f"Mode: {mode}")
        print(f"Objective: tracker")
        print(f"Cascade top-N: {CASCADE_TOP_N}")

        if single:
            params = dict(BASE_PARAMS)
            for arg in sys.argv:
                if "=" in arg and not arg.startswith("--"):
                    k, v = arg.split("=", 1)
                    if k in BASE_PARAMS:
                        params[k] = float(v) if "." in v else int(v)
            print(f"Testing: { {k: v for k, v in params.items() if v != BASE_PARAMS.get(k)} }")
            r = run_test(params, raceline=raceline_path)
            score = compute_score(r)
            print(f"Result: {r}")
            print(f"Score:  {score}")
            return

        available_racelines = {}
        for tag, path in racelines_mode.items():
            if os.path.exists(path):
                available_racelines[tag] = path
                print(f"Raceline [{tag}]: {path}")
            else:
                print(f"Raceline [{tag}]: NOT FOUND ({path})")

        if not available_racelines:
            raise RuntimeError("No racelines found for selected mode")

        # If --all-racelines, iterate over all available racelines
        if all_racelines:
            racelines_to_test = list(available_racelines.items())
        elif raceline_name:
            if raceline_name in available_racelines:
                racelines_to_test = [(raceline_name, available_racelines[raceline_name])]
            elif raceline_path and os.path.exists(raceline_path):
                racelines_to_test = [(raceline_name, raceline_path)]
            else:
                raise RuntimeError(f"Requested raceline not found: {raceline_name}")
        else:
            if mode == "Hardware":
                racelines_to_test = [("hardware", available_racelines["hardware"])]

        preflight_validate(mode, racelines_to_test)

        for rl_name, rl_path in racelines_to_test:
            print(f"\n{'#'*80}")
            print(f"# RACELINE: {rl_name}")
            print(f"{'#'*80}")

            # Override WALL_MARGIN baseline for this raceline
            sweep_base = dict(BASE_PARAMS)
            if rl_name in per_raceline_wm_mode:
                sweep_base["WALL_MARGIN"] = per_raceline_wm_mode[rl_name]

            run_sweep(values, phase_only, quick, rl_path, rl_name, sweep_base, num_workers, CASCADE_TOP_N)

    finally:
        # Restore original header
        if HEADER_BAK.exists():
            shutil.copy2(HEADER_BAK, HEADER)
            HEADER_BAK.unlink()
        if BINARY.exists():
            BINARY.unlink()
        if WORKER_ROOT.exists():
            shutil.rmtree(WORKER_ROOT, ignore_errors=True)


def run_sweep(values, phase_only, quick, raceline_path, raceline_name, base_params,
              num_workers=1, cascade_top=1):
    """Run a full multi-phase sweep for one raceline."""
    all_results = []
    t0 = time.time()

    mode = "QUICK" if quick else "EXHAUSTIVE"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_dir = PROJECT_DIR / "testbench" / "tuning_results"
    results_dir.mkdir(parents=True, exist_ok=True)
    outfile = results_dir / f"fpga_tuning_{ACTIVE_MODE.lower()}_{raceline_name}_{timestamp}.csv"
    sorted_outfile = results_dir / f"fpga_tuning_{ACTIVE_MODE.lower()}_{raceline_name}_{timestamp}_sorted.csv"

    fieldnames = ["label", "phase", "raceline", "score", "passed", "failed",
                  "max_lat_err", "avg_lat_err", "max_hdg_err", "avg_hdg_err",
                  "max_vx", "avg_vel_err", "max_vel_err", "avg_solve_us",
                  "max_solve_us", "wall_collisions", "time_above_5ms",
                  "avg_iters", "status"] + sorted(PARAM_TO_DEFINE.keys())
    csv_writer = IncrementalCSV(outfile, fieldnames)

    print(f"\n{'='*80}")
    print(f"FPGA MPC Parameter Tuning — {mode} — raceline={raceline_name}")
    print(f"Results (incremental): {outfile}")
    print(f"{'='*80}")

    rl = raceline_path

    # ─── Phase 1a: Structural (dt × N × WALL_END × WALL_MARGIN) ────
    if phase_only is None or phase_only == 1:
        combos = gen_structural_sweep(values, base_params)
        combos = deduplicate(combos)
        print(f"\n--- Phase 1a: Structural sweep ({len(combos)} configs) ---")
        phase_results = run_phase("Phase 1a", combos, all_results, t0, rl,
                                  num_workers=num_workers, raceline_label=raceline_name,
                                  csv_writer=csv_writer)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 1a")

        # Phase 1b: WALL_START × WALL_STRIDE on best structural
        best_struct = find_best_params(all_results, base_params)
        combos_b = gen_wall_detail_sweep(best_struct, values)
        combos_b = deduplicate(combos_b)
        tested = get_tested_keys(all_results)
        combos_b = [(l, p) for l, p in combos_b if param_key(p) not in tested]
        if combos_b:
            print(f"\n--- Phase 1b: Wall detail ({len(combos_b)} configs) ---")
            phase_results_b = run_phase("Phase 1b", combos_b, all_results, t0, rl,
                                        num_workers=num_workers, raceline_label=raceline_name,
                                        csv_writer=csv_writer)
            all_results.extend(phase_results_b)
            print_top(phase_results_b, "Phase 1b")

    # Find best structural
    best_struct = find_best_params(all_results, base_params)
    print(f"\nBest structural: dt={best_struct.get('dt')}, "
          f"N={best_struct.get('N')}, "
          f"WE={best_struct.get('WALL_END')}, "
          f"WM={best_struct.get('WALL_MARGIN')}, "
          f"WS={best_struct.get('WALL_START')}, "
          f"WST={best_struct.get('WALL_STRIDE')}")

    # ─── Phase 2: Primary weights ───────────────────────────────────
    if phase_only is None or phase_only == 2:
        combos = gen_primary_sweep(best_struct, values)
        combos = deduplicate(combos)
        print(f"\n--- Phase 2: Primary weights ({len(combos)} configs) ---")
        phase_results = run_phase("Phase 2", combos, all_results, t0, rl,
                                  num_workers=num_workers, raceline_label=raceline_name,
                                  csv_writer=csv_writer)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 2")

    best_primary = find_best_params(all_results, base_params)
    branches = [best_primary]
    if phase_only is None and cascade_top > 1:
        branches = top_n_param_sets(all_results, cascade_top, base_params)

    for branch_idx, branch_base in enumerate(branches, start=1):
        branch_tag = f"[branch {branch_idx}/{len(branches)}]" if len(branches) > 1 else ""
        current_best = dict(branch_base)

        if len(branches) > 1:
            print(f"\n>>> CASCADE {branch_tag} <<<")

        # ─── Phase 3: Secondary weights ─────────────────────────────
        if phase_only is None or phase_only == 3:
            combos = gen_secondary_sweep(current_best, values)
            combos = deduplicate(combos)
            print(f"\n--- Phase 3 {branch_tag}: Secondary weights ({len(combos)} configs) ---")
            phase_results = run_phase(f"Phase 3 {branch_tag}".strip(), combos, all_results, t0, rl,
                                      num_workers=num_workers, raceline_label=raceline_name,
                                      csv_writer=csv_writer)
            all_results.extend(phase_results)
            print_top(phase_results, f"Phase 3 {branch_tag}".strip())
            current_best = find_best_params(all_results, base_params)

        # ─── Phase 4: ADMM solver ───────────────────────────────────
        if phase_only is None or phase_only == 4:
            combos = gen_solver_sweep(current_best, values)
            combos = deduplicate(combos)
            print(f"\n--- Phase 4 {branch_tag}: ADMM parameters ({len(combos)} configs) ---")
            phase_results = run_phase(f"Phase 4 {branch_tag}".strip(), combos, all_results, t0, rl,
                                      num_workers=num_workers, raceline_label=raceline_name,
                                      csv_writer=csv_writer)
            all_results.extend(phase_results)
            print_top(phase_results, f"Phase 4 {branch_tag}".strip())
            current_best = find_best_params(all_results, base_params)

        # ─── Phase 5: Fine-tuning ───────────────────────────────────
        if phase_only is None or phase_only == 5:
            combos = gen_fine_tune(current_best)
            combos = deduplicate(combos)
            tested = get_tested_keys(all_results)
            combos = [(l, p) for l, p in combos if param_key(p) not in tested]
            print(f"\n--- Phase 5 {branch_tag}: Fine-tuning ({len(combos)} configs) ---")
            phase_results = run_phase(f"Phase 5 {branch_tag}".strip(), combos, all_results, t0, rl,
                                      num_workers=num_workers, raceline_label=raceline_name,
                                      csv_writer=csv_writer)
            all_results.extend(phase_results)
            print_top(phase_results, f"Phase 5 {branch_tag}".strip())
            current_best = find_best_params(all_results, base_params)

        # ─── Phase 6: Random neighbors ──────────────────────────────
        if phase_only is None or phase_only == 6:
            n_random = 600 if not quick else 60
            combos = gen_random_neighbors(current_best, n_random)
            combos = deduplicate(combos)
            tested = get_tested_keys(all_results)
            combos = [(l, p) for l, p in combos if param_key(p) not in tested]
            print(f"\n--- Phase 6 {branch_tag}: Random neighbors ({len(combos)} configs) ---")
            phase_results = run_phase(f"Phase 6 {branch_tag}".strip(), combos, all_results, t0, rl,
                                      num_workers=num_workers, raceline_label=raceline_name,
                                      csv_writer=csv_writer)
            all_results.extend(phase_results)
            print_top(phase_results, f"Phase 6 {branch_tag}".strip())

    # ─── Final report ───────────────────────────────────────────────
    elapsed = time.time() - t0
    save_results(all_results, sorted_outfile)

    print(f"\n{'='*80}")
    print(f"[{raceline_name}] Completed {len(all_results)} tests in {elapsed:.1f}s")
    print(f"Incremental results: {outfile}")
    print(f"Sorted results:      {sorted_outfile}")
    print(f"{'='*80}")

    print_top(all_results, f"OVERALL ({raceline_name})", n=30)

    best_final = find_best_params(all_results, base_params)
    print(f"\n--- Best Configuration ({raceline_name}) ---")
    for k, v in sorted(best_final.items()):
        if k in PARAM_TO_DEFINE:
            base = base_params.get(k)
            marker = " ←CHANGED" if base is not None and abs(v - base) > 1e-6 else ""
            print(f"  {k:16s} = {v}{marker}")


def _flatten_result_row(result: dict) -> dict:
    row = dict(result)
    for k, v in result.get("params", {}).items():
        row[k] = v
    return row


def run_phase(phase_name, combos, previous_results, t0,
              raceline=None, num_workers=1, raceline_label="", csv_writer=None):
    """Run all combos, print progress, return results."""
    results = []
    tested = get_tested_keys(previous_results)
    pending = [(l, p) for l, p in combos if param_key(p) not in tested]
    total = len(pending)

    if total == 0:
        return results

    if num_workers > 1 and not WORKER_ROOT.exists():
        WORKER_ROOT.mkdir(parents=True, exist_ok=True)

    if num_workers <= 1:
        for i, (label, params) in enumerate(pending):
            elapsed = time.time() - t0
            rate = max((len(previous_results) + len(results) + 1) / max(elapsed, 0.01), 0.1)
            eta = (total - i - 1) / rate
            print(f"[{i+1:4d}/{total}] {label:55s} ", end="", flush=True)

            r = run_test(params, raceline=raceline, base_params=BASE_PARAMS)
            score = compute_score(r)
            r["label"] = label
            r["score"] = score
            r["params"] = dict(params)
            r["phase"] = phase_name
            r["raceline"] = raceline_label or "default"
            results.append(r)
            if csv_writer:
                csv_writer.write_row(_flatten_result_row(r))

            if r["status"] != "OK":
                print(f"  → {r['status']}  (ETA {eta:.0f}s)")
            elif r["wall_collisions"] > 0:
                print(f"  → wc={r['wall_collisions']} sc={score:.2f}  t5={r['time_above_5ms']:.0f}s  "
                      f"lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")
            else:
                tf = f"  tf={r.get('failed', 0)}" if r.get("failed", 0) > 0 else ""
                print(f"  → PASS sc={score:.2f}  t5={r['time_above_5ms']:.0f}s  "
                      f"vmax={r['max_vx']:.1f}  lat={r['avg_lat_err']:.3f}{tf}  (ETA {eta:.0f}s)")
        return results

    jobs = [(label, params, raceline) for label, params in pending]
    done = 0
    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        futures = [executor.submit(_run_single_parallel, job) for job in jobs]
        for fut in as_completed(futures):
            done += 1
            r = fut.result()
            r["phase"] = phase_name
            r["raceline"] = raceline_label or "default"
            results.append(r)
            if csv_writer:
                csv_writer.write_row(_flatten_result_row(r))

            elapsed = time.time() - t0
            rate = max((len(previous_results) + done) / max(elapsed, 0.01), 0.1)
            eta = (total - done) / rate

            if r["status"] != "OK":
                print(f"[{done:4d}/{total}] {r['label']:55s}   → {r['status']}  (ETA {eta:.0f}s)")
            elif r["wall_collisions"] > 0:
                print(f"[{done:4d}/{total}] {r['label']:55s}   → wc={r['wall_collisions']} sc={r['score']:.2f}  "
                      f"t5={r['time_above_5ms']:.0f}s  lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")
            else:
                tf = f"  tf={r.get('failed', 0)}" if r.get("failed", 0) > 0 else ""
                print(f"[{done:4d}/{total}] {r['label']:55s}   → PASS sc={r['score']:.2f}  "
                      f"t5={r['time_above_5ms']:.0f}s  vmax={r['max_vx']:.1f}  "
                      f"lat={r['avg_lat_err']:.3f}{tf}  (ETA {eta:.0f}s)")

    return results


def find_best_params(results, base_params=None):
    """Return the params of the best result."""
    if base_params is None:
        base_params = BASE_PARAMS
    if not results:
        return dict(base_params)
    safe = [r for r in results if is_safe_result(r)]
    ranked_pool = safe if safe else results
    best = min(ranked_pool, key=lambda r: r.get("score", 999))
    return best.get("params", dict(base_params))


def param_key(params):
    """Create a hashable key for a parameter set."""
    return tuple(sorted((k, round(v, 6) if isinstance(v, float) else v)
                        for k, v in params.items() if k in PARAM_TO_DEFINE))


def get_tested_keys(results):
    """Get set of already-tested parameter keys."""
    keys = set()
    for r in results:
        p = r.get("params", {})
        if p:
            keys.add(param_key(p))
    return keys


def print_top(results, phase_name, n=10):
    """Print top N results from this phase."""
    passing = [r for r in results if is_safe_result(r)]
    best = sorted(results, key=lambda r: (r.get("score", 999),
                                          r.get("wall_collisions", 99),
                                          -r.get("time_above_5ms", 0)))

    print(f"\n  Top {min(n, len(best))} — {phase_name}:")
    fmt = "  {:<4} {:<55} {:>7} {:>5} {:>5} {:>6} {:>6}"
    print(fmt.format("Rank", "Label", "Score", "T>5s", "WC", "AvgLat", "MaxVx"))
    for i, r in enumerate(best[:n]):
        print(fmt.format(
            i+1, r['label'][:55],
            f"{r.get('score', 999):.1f}",
            f"{r.get('time_above_5ms', 0):.0f}",
            str(r.get('wall_collisions', '?')),
            f"{r.get('avg_lat_err', 0):.3f}",
            f"{r.get('max_vx', 0):.1f}"))

    if passing:
        print(f"\n  *** {len(passing)} configs passed without wall collisions ***")


def save_results(results, outfile):
    """Save all results to CSV."""
    if not results:
        return

    fieldnames = ["label", "phase", "raceline", "score", "passed", "failed",
                  "max_lat_err", "avg_lat_err", "max_hdg_err", "avg_hdg_err",
                  "max_vx", "avg_vel_err", "max_vel_err",
                  "avg_solve_us", "max_solve_us",
                  "wall_collisions", "time_above_5ms", "avg_iters", "status"]
    # Add all tunable params
    for k in sorted(PARAM_TO_DEFINE.keys()):
        fieldnames.append(k)

    with open(outfile, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        writer.writeheader()
        for r in sorted(results, key=lambda x: x.get("score", 999)):
            writer.writerow(_flatten_result_row(r))


if __name__ == "__main__":
    sys.exit(main() or 0)
