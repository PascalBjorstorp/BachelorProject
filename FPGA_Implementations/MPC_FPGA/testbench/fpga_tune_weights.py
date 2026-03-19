#!/usr/bin/env python3
"""
FPGA MPC Weight Tuning Script — Exhaustive Search with dt/N sweep
=================================================================
Systematically tests parameter combinations for the FPGA Riccati-ADMM MPC.
Since FPGA uses compile-time constants, this script modifies the header
file (mpc_fpga_types.h), recompiles, runs the test, and collects results.

Sweep parameters:
    - dt (prediction timestep): affects horizon spacing and model discretization
  - N  (horizon steps): affects how many prediction steps (array sizes)
  - WALL_END, WALL_MARGIN: wall constraint parameters
  - Q_LAT, Q_HDG, Q_VEL, Q_LAT_VEL, Q_YAW: state tracking weights
  - R_STEER, W_JERK, W_ACCEL_RATE, W_DELTA_ACT: control weights
  - ADMM_RHO, ADMM_RHO_U: solver parameters

Usage:
    python3 fpga_tune_weights.py                    # Full exhaustive sweep (default raceline)
    python3 fpga_tune_weights.py --quick             # Quick sweep
    python3 fpga_tune_weights.py --raceline cl050    # Sweep specific raceline
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
HEADER = PROJECT_DIR / "include" / "mpc_fpga_types.h"
HEADER_BAK = HEADER.with_suffix(".h.tuning_backup")

SRC_FILES = [
    "testbench/test_fpga_sim_drive.c",
    "src/riccati_solver_hls.c",
    "src/mpc_riccati_hls.c",
    "src/mpc_fpga_top.c",
    "src/vehicle_model_hls.c",
    "src/fp_math_hls.c",
]
BINARY = PROJECT_DIR / "test_fpga_tune"
CC_FLAGS = ["-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall", "-Wno-unknown-pragmas",
            f"-I{PROJECT_DIR / 'include'}"]

# Parallel worker scratch area (created only when --jobs > 1)
WORKER_ROOT = PROJECT_DIR / ".fpga_tune_workers"
_WORKER_CTX = None

# ─── Raceline variants ───────────────────────────────────────────────────────
WORKSPACE_ROOT = PROJECT_DIR.parent.parent  # BachelorProject/
TRAJ_DIR = WORKSPACE_ROOT / "f1tenth_planning" / "trajectories"
RACELINES = {
    "cl020": str(TRAJ_DIR / "Spielberg_raceline_pipeline_cl020.csv"),
    "cl030": str(TRAJ_DIR / "Spielberg_raceline_pipeline_cl030.csv"),
    "cl045": str(TRAJ_DIR / "Spielberg_raceline_pipeline_cl045.csv"),
    "cl050": str(TRAJ_DIR / "Spielberg_raceline_pipeline_cl050.csv"),
}

# Per-raceline WALL_MARGIN defaults (matching CPU optimal results)
PER_RACELINE_WM = {
    "cl020": 0.10,
    "cl030": 0.18,
    "cl045": 0.15,
    "cl050": 0.15,
}

# ─── Current baseline (synced with CPU optimal sweep results) ──────────────
BASE_PARAMS = {
    # Structural / timing
    "dt":               0.04,
    "N":                20,
    "MAX_ADMM_ITER":    20,
    # Wall constraints
    "WALL_START":       1,
    "WALL_STRIDE":      1,
    "WALL_END":         18,
    "WALL_MARGIN":      0.15,
    # State weights
    "Q_LAT":            340.0,
    "Q_HDG":            1000.0,
    "Q_VEL":            26.0,
    "Q_LAT_VEL":        69.0,
    "Q_YAW":            22.0,
    # Control weights
    "R_STEER":          0.15,
    "R_ACCEL":          0.01,
    "W_JERK":           0.3,
    "W_ACCEL_RATE":     0.1,
    "W_DELTA_ACT":      0.53,
    # Solver
    "ADMM_RHO":         50.0,
    "ADMM_RHO_U":       26.6,
    "ADMM_TOL":         5.0,
    "ADMM_ALPHA":       1.2,
    # Model/constraint limits
    "MIN_LIN_VEL":      2.0,
    "WP_ADVANCE_MAX":   10,
    "STABILITY_LIMIT":  0.95,
    # Fixed physical tire model constant for F1TENTH rubber tires.
    # Kept as a baseline parameter for single-run overrides, but not swept.
    "C_SHAPE":         1.9,
}

# Map parameter names → header #define names
PARAM_TO_DEFINE = {
    "dt":               "MPC_DT",
    "N":                "MPC_HORIZON",
    "MAX_ADMM_ITER":    "MPC_MAX_ADMM_ITER",
    "WALL_START":       "WALL_START",
    "WALL_STRIDE":      "WALL_STRIDE",
    "WALL_END":         "WALL_END",
    "WALL_MARGIN":      "WALL_MARGIN",
    "Q_LAT":            "MPC_W_LAT_ERROR",
    "Q_HDG":            "MPC_W_HEADING",
    "Q_VEL":            "MPC_W_VELOCITY",
    "Q_LAT_VEL":        "MPC_W_LAT_VEL",
    "Q_YAW":            "MPC_W_YAW_RATE",
    "R_STEER":          "MPC_W_STEER_EFF",
    "R_ACCEL":          "MPC_W_ACCEL_EFF",
    "W_JERK":           "MPC_W_STEER_JERK",
    "W_ACCEL_RATE":     "MPC_W_ACCEL_RATE",
    "W_DELTA_ACT":      "MPC_W_DELTA_ACT",
    "ADMM_RHO":         "ADMM_RHO_DEFAULT",
    "ADMM_RHO_U":       "ADMM_RHO_U_DEFAULT",
    "ADMM_TOL":         "ADMM_TOL_DEFAULT",
    "ADMM_ALPHA":       "ADMM_OVER_RELAX",
    "MIN_LIN_VEL":      "MIN_LIN_VEL",
    "WP_ADVANCE_MAX":   "WP_ADVANCE_MAX",
    "STABILITY_LIMIT":  "STABILITY_LIMIT_VAL",
}

# ─── Sweep ranges (THOROUGH) ─────────────────────────────────────────────────
FULL_VALUES = {
    # Structural / timing
    "dt":               [0.02, 0.025, 0.030, 0.035, 0.040, 0.050, 0.060, 0.075, 0.08, 0.1],
    "N":                [16, 17, 18, 19, 20],
    "MAX_ADMM_ITER":    [5, 8, 10, 15, 20, 30],
    # Wall constraints
    "WALL_START":       [0, 1, 2, 3],
    "WALL_STRIDE":      [1, 2, 3, 4],
    "WALL_END":         [5, 8, 10, 12, 15, 16, 18, 20, 22, 24, 28],
    "WALL_MARGIN":      [0.00, 0.05, 0.10, 0.15, 0.20, 0.30, 0.35, 0.40, 0.45, 0.50, 0.70],
    # State weights (denser around new best: Q_LAT=340, Q_VEL=26)
    "Q_LAT":            [50, 100, 150, 200, 250, 300, 320, 340, 350, 360, 380, 400, 450, 500, 550, 600, 700, 800, 1000],
    "Q_HDG":            [100, 180, 200, 300, 400, 600, 800, 900, 1000, 1200, 1500, 2000, 3000],
    "Q_VEL":            [4, 6, 8, 10, 15, 20, 22, 24, 25, 26, 28, 30, 35, 50, 80, 100, 150],
    "Q_LAT_VEL":        [10, 15, 20, 30, 40, 45, 55, 60, 80, 100, 120, 150],
    "Q_YAW":            [1, 5, 10, 15, 18, 20, 22, 30, 40, 60, 100],
    # Control weights
    "R_STEER":          [0.04, 0.06, 0.08, 0.09, 0.10, 0.12, 0.15, 0.18, 0.20, 0.30, 0.50, 1.0],
    "W_JERK":           [0.05, 0.08, 0.1, 0.14, 0.2, 0.3, 0.5, 1.0, 2.0, 5.0],
    "W_ACCEL_RATE":     [0.01, 0.05, 0.08, 0.1, 0.15, 0.2, 0.3, 0.5, 1.0],
    "W_DELTA_ACT":      [0.05, 0.1, 0.2, 0.5, 1.0, 2.0],
    # Solver (more granular around current best)
    "ADMM_RHO":         [10, 15, 20, 25, 30, 40, 50, 60, 70, 80],
    "ADMM_RHO_U":       [5, 10, 15, 20, 25, 30, 40, 50],
    "ADMM_TOL":         [1.0, 3.0, 5.0, 10.0, 20.0],
    "ADMM_ALPHA":       [0.93, 0.95, 1.0, 1.05, 1.1, 1.15, 1.2, 1.3, 1.4, 1.5, 1.6, 1.8],
    # Model limits
    "MIN_LIN_VEL":      [1.0, 1.5, 2.0, 3.0],
    "WP_ADVANCE_MAX":   [5, 8, 10, 15, 20],
    "STABILITY_LIMIT":  [0.85, 0.90, 0.95, 1.0],
}

QUICK_VALUES = {
    "dt":               [0.03, 0.035, 0.040, 0.050],
    "N":                [18, 19, 20],
    "MAX_ADMM_ITER":    [10, 20],
    "WALL_START":       [1],
    "WALL_STRIDE":      [1, 2],
    "WALL_END":         [12, 16, 18, 22],
    "WALL_MARGIN":      [0.10, 0.15, 0.20, 0.40],
    "Q_LAT":            [200, 300, 340, 400, 500],
    "Q_HDG":            [400, 700, 1000, 1500],
    "Q_VEL":            [15, 22, 26, 30, 50],
    "Q_LAT_VEL":        [30, 60, 69, 100],
    "Q_YAW":            [10, 22, 40],
    "R_STEER":          [0.10, 0.15, 0.25],
    "W_JERK":           [0.1, 0.3, 0.5],
    "W_ACCEL_RATE":     [0.05, 0.1, 0.3],
    "W_DELTA_ACT":      [0.1, 0.5, 1.0],
    "ADMM_RHO":         [20, 30, 50],
    "ADMM_RHO_U":       [10, 20, 30],
    "ADMM_TOL":         [3.0, 5.0],
    "ADMM_ALPHA":       [0.93, 1.0, 1.2, 1.4],
    "MIN_LIN_VEL":      [2.0],
    "WP_ADVANCE_MAX":   [10],
    "STABILITY_LIMIT":  [0.95],
}


# ═══════════════════════════════════════════════════════════════════════════════
#  Header file modification
# ═══════════════════════════════════════════════════════════════════════════════

def set_header_param(header_text: str, param: str, value) -> str:
    """Modify a #define in the header text. Returns modified text."""
    define_name = PARAM_TO_DEFINE[param]

    if param == "dt":
        # MPC_DT is in Q16.16 fixed-point
        fp_val = round(value * 65536)
        pattern = r"#define\s+MPC_DT\s+.*"
        replacement = f"#define MPC_DT              ((fixed_point_t){fp_val})   /* {value}s in Q16.16 */"
        return re.sub(pattern, replacement, header_text)

    elif param == "ADMM_ALPHA":
        # Update ADMM_OVER_RELAX and its derived constants
        alpha = float(value)
        pattern = r"#define\s+ADMM_OVER_RELAX\s+FP_CONST\([^)]+\)"
        replacement = f"#define ADMM_OVER_RELAX             FP_CONST({alpha})"
        header_text = re.sub(pattern, replacement, header_text)

        complement = round(1.0 - alpha, 4)
        pattern2 = r"#define\s+ADMM_OVER_RELAX_COMPLEMENT\s+.*"
        replacement2 = f"#define ADMM_OVER_RELAX_COMPLEMENT  FP_CONST({complement})  /* 1 - alpha */"
        header_text = re.sub(pattern2, replacement2, header_text)

        minus1_fp = round((alpha - 1.0) * 65536)
        pattern3 = r"#define\s+ADMM_OVER_RELAX_MINUS1\s+.*"
        replacement3 = f"#define ADMM_OVER_RELAX_MINUS1      {minus1_fp}           /* alpha - 1 = {alpha-1.0:.1f}  in Q16.16 */"
        header_text = re.sub(pattern3, replacement3, header_text)
        return header_text

    elif param in ("N", "MAX_ADMM_ITER", "WALL_END", "WALL_START",
                   "WALL_STRIDE", "WP_ADVANCE_MAX"):
        # Integer defines
        pattern = rf"#define\s+{define_name}\s+\d+"
        replacement = f"#define {define_name:<24s} {int(value)}"
        return re.sub(pattern, replacement, header_text)

    else:
        # Float value → FP_CONST(x)
        pattern = rf"#define\s+{define_name}\s+FP_CONST\([^)]+\)"
        replacement = f"#define {define_name:<24s} FP_CONST({value})"
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

    # Ensure WALL_END doesn't exceed N-1
    n = int(params.get("N", base_params["N"]))
    we = int(params.get("WALL_END", base_params["WALL_END"]))
    if we >= n:
        we = n - 2
        header_text = set_header_param(header_text, "WALL_END", we)

    # Ensure WALL_START < WALL_END
    ws = int(params.get("WALL_START", base_params["WALL_START"]))
    if ws >= we:
        ws = max(0, we - 1)
        header_text = set_header_param(header_text, "WALL_START", ws)

    header_path.write_text(header_text)


# ═══════════════════════════════════════════════════════════════════════════════
#  Compile & Run
# ═══════════════════════════════════════════════════════════════════════════════

def compile_test(project_dir: Path = PROJECT_DIR, binary_path: Path = BINARY) -> bool:
    """Compile the test binary. Returns True on success."""
    src_paths = [str(project_dir / s) for s in SRC_FILES]
    cc_flags = ["-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall", "-Wno-unknown-pragmas",
                f"-I{project_dir / 'include'}"]
    cmd = ["gcc"] + cc_flags + src_paths + ["-o", str(binary_path), "-lm"]
    result = subprocess.run(cmd, capture_output=True, text=True, cwd=str(project_dir))
    return result.returncode == 0


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

    cmd = [str(binary_path)]
    if raceline:
        # test_fpga_sim_drive.c reads raceline override from argv[1]
        cmd.append(str(raceline))

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
        return 500.0 + r["wall_collisions"] * 100.0

    # Match tune_realistic Spielberg scoring.
    # This is a weighted composite, not an automatic normalization pass.
    # Metric scaling is encoded in the constants below to keep scoring stable
    # and comparable across sweeps:
    # - speed shortfall uses max(0, 12.0 - max_vx)
    # - latency uses max(0, 60 - time_above_5ms)
    # - errors/iters/runtime use fixed coefficients.
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

    header = worker_dir / "include" / "mpc_fpga_types.h"
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


# ═══════════════════════════════════════════════════════════════════════════════
#  Combination generators
# ═══════════════════════════════════════════════════════════════════════════════

def gen_structural_sweep(values, base_params=None):
    """Phase 1a: dt × N × WALL_END × WALL_MARGIN (core structural).
    WALL_START and WALL_STRIDE deferred to Phase 1b."""
    if base_params is None:
        base_params = BASE_PARAMS
    combos = []
    for dt in values.get("dt", [base_params["dt"]]):
        for n in values.get("N", [base_params["N"]]):
            for we in values.get("WALL_END", [base_params["WALL_END"]]):
                if we >= n:
                    continue
                for wm in values.get("WALL_MARGIN", [base_params["WALL_MARGIN"]]):
                    p = dict(base_params)
                    p["dt"] = dt
                    p["N"] = n
                    p["WALL_END"] = we
                    p["WALL_MARGIN"] = wm
                    label = f"dt={dt}+N={n}+WE={we}+WM={wm}"
                    combos.append((label, p))
    return combos


def gen_wall_detail_sweep(best_structural, values):
    """Phase 1b: WALL_START × WALL_STRIDE on best structural."""
    combos = []
    n = int(best_structural.get("N", BASE_PARAMS["N"]))
    we = int(best_structural.get("WALL_END", BASE_PARAMS["WALL_END"]))
    for ws in values.get("WALL_START", [BASE_PARAMS["WALL_START"]]):
        if ws >= we:
            continue
        for wst in values.get("WALL_STRIDE", [BASE_PARAMS["WALL_STRIDE"]]):
            p = dict(best_structural)
            p["WALL_START"] = ws
            p["WALL_STRIDE"] = wst
            label = f"bestStruct+WS={ws}+WST={wst}"
            combos.append((label, p))
    return combos


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
                  "W_ACCEL_RATE", "W_DELTA_ACT", "MIN_LIN_VEL",
                  "WP_ADVANCE_MAX", "STABILITY_LIMIT"]

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
    solver_params = ["ADMM_RHO", "ADMM_RHO_U", "ADMM_TOL",
                     "ADMM_ALPHA", "MAX_ADMM_ITER"]
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

    # RHO × ALPHA
    for rho in values.get("ADMM_RHO", [BASE_PARAMS["ADMM_RHO"]]):
        for alpha in values.get("ADMM_ALPHA", [BASE_PARAMS["ADMM_ALPHA"]]):
            p = dict(best_so_far)
            p["ADMM_RHO"] = rho
            p["ADMM_ALPHA"] = alpha
            label = f"best+rho={rho}+alpha={alpha}"
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


def gen_random_neighbors(best_params, n_samples=150):
    """Phase 6: Random perturbations around the best config.
    Each sample randomly perturbs 2-5 parameters by ±5-40%."""
    combos = []
    tunable = ["Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
               "R_STEER", "W_JERK", "W_ACCEL_RATE", "W_DELTA_ACT",
               "WALL_MARGIN", "ADMM_RHO", "ADMM_RHO_U", "ADMM_ALPHA"]

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
    quick = "--quick" in sys.argv
    phase_only = None
    raceline_name = None
    num_workers = 1
    for i, arg in enumerate(sys.argv):
        if arg == "--phase" and i + 1 < len(sys.argv):
            phase_only = int(sys.argv[i + 1])
        if arg == "--raceline" and i + 1 < len(sys.argv):
            raceline_name = sys.argv[i + 1]
        if arg == "--jobs" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
        if arg == "-j" and i + 1 < len(sys.argv):
            num_workers = int(sys.argv[i + 1])
    single = "--single" in sys.argv
    all_racelines = "--all-racelines" in sys.argv

    if num_workers <= 0:
        num_workers = multiprocessing.cpu_count()

    os.chdir(str(PROJECT_DIR))

    # Resolve raceline path
    raceline_path = None
    if raceline_name:
        if raceline_name in RACELINES:
            raceline_path = RACELINES[raceline_name]
        elif os.path.isfile(raceline_name):
            raceline_path = raceline_name
        else:
            print(f"ERROR: Unknown raceline '{raceline_name}'. "
                  f"Available: {', '.join(RACELINES.keys())}")
            return 1

    # Backup header
    if not HEADER_BAK.exists():
        shutil.copy2(HEADER, HEADER_BAK)
    else:
        # Ensure we have a clean backup
        pass

    try:
        values = QUICK_VALUES if quick else FULL_VALUES
        print(f"Workers: {num_workers} ({'sequential' if num_workers == 1 else 'parallel'})")

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

        # If --all-racelines, iterate over all racelines
        if all_racelines:
            racelines_to_test = list(RACELINES.items())
        elif raceline_name:
            racelines_to_test = [(raceline_name, raceline_path)]
        else:
            racelines_to_test = [("default", None)]

        for rl_name, rl_path in racelines_to_test:
            print(f"\n{'#'*80}")
            print(f"# RACELINE: {rl_name}")
            print(f"{'#'*80}")

            # Override WALL_MARGIN baseline for this raceline
            sweep_base = dict(BASE_PARAMS)
            if rl_name in PER_RACELINE_WM:
                sweep_base["WALL_MARGIN"] = PER_RACELINE_WM[rl_name]

            run_sweep(values, phase_only, quick, rl_path, rl_name, sweep_base, num_workers)

    finally:
        # Restore original header
        if HEADER_BAK.exists():
            shutil.copy2(HEADER_BAK, HEADER)
            HEADER_BAK.unlink()
        if BINARY.exists():
            BINARY.unlink()
        if WORKER_ROOT.exists():
            shutil.rmtree(WORKER_ROOT, ignore_errors=True)


def run_sweep(values, phase_only, quick, raceline_path, raceline_name, base_params, num_workers=1):
    """Run a full multi-phase sweep for one raceline."""
    all_results = []
    t0 = time.time()

    mode = "QUICK" if quick else "EXHAUSTIVE"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outfile = f"testbench/fpga_tuning_{raceline_name}_{timestamp}.csv"

    print(f"\n{'='*80}")
    print(f"FPGA MPC Parameter Tuning — {mode} — raceline={raceline_name}")
    print(f"{'='*80}")

    rl = raceline_path

    # ─── Phase 1a: Structural (dt × N × WALL_END × WALL_MARGIN) ────
    if phase_only is None or phase_only == 1:
        combos = gen_structural_sweep(values, base_params)
        combos = deduplicate(combos)
        print(f"\n--- Phase 1a: Structural sweep ({len(combos)} configs) ---")
        phase_results = run_phase(combos, all_results, t0, rl, num_workers=num_workers)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 1a")

        # Phase 1b: WALL_START × WALL_STRIDE on best structural
        best_struct = find_best_params(all_results, base_params)
        combos_b = gen_wall_detail_sweep(best_struct, values)
        combos_b = deduplicate(combos_b)
        tested = get_tested_keys(all_results)
        combos_b = [(l, p) for l, p in combos_b
                    if param_key(p) not in tested]
        if combos_b:
            print(f"\n--- Phase 1b: Wall detail ({len(combos_b)} configs) ---")
            phase_results_b = run_phase(combos_b, all_results, t0, rl, num_workers=num_workers)
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
        phase_results = run_phase(combos, all_results, t0, rl, num_workers=num_workers)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 2")

    best_primary = find_best_params(all_results, base_params)

    # ─── Phase 3: Secondary weights ─────────────────────────────────
    if phase_only is None or phase_only == 3:
        combos = gen_secondary_sweep(best_primary, values)
        combos = deduplicate(combos)
        print(f"\n--- Phase 3: Secondary weights ({len(combos)} configs) ---")
        phase_results = run_phase(combos, all_results, t0, rl, num_workers=num_workers)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 3")

    best_secondary = find_best_params(all_results, base_params)

    # ─── Phase 4: ADMM solver ───────────────────────────────────────
    if phase_only is None or phase_only == 4:
        combos = gen_solver_sweep(best_secondary, values)
        combos = deduplicate(combos)
        print(f"\n--- Phase 4: ADMM parameters ({len(combos)} configs) ---")
        phase_results = run_phase(combos, all_results, t0, rl, num_workers=num_workers)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 4")

    best_solver = find_best_params(all_results, base_params)

    # ─── Phase 5: Fine-tuning ───────────────────────────────────────
    if phase_only is None or phase_only == 5:
        combos = gen_fine_tune(best_solver)
        combos = deduplicate(combos)
        # Remove already-tested
        tested = get_tested_keys(all_results)
        combos = [(l, p) for l, p in combos
                  if param_key(p) not in tested]
        print(f"\n--- Phase 5: Fine-tuning ({len(combos)} configs) ---")
        phase_results = run_phase(combos, all_results, t0, rl, num_workers=num_workers)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 5")

    # ─── Phase 6: Random neighbors ──────────────────────────────────
    if phase_only is None or phase_only == 6:
        best_so_far = find_best_params(all_results, base_params)
        n_random = 150 if not quick else 60
        combos = gen_random_neighbors(best_so_far, n_random)
        combos = deduplicate(combos)
        tested = get_tested_keys(all_results)
        combos = [(l, p) for l, p in combos
                  if param_key(p) not in tested]
        print(f"\n--- Phase 6: Random neighbors ({len(combos)} configs) ---")
        phase_results = run_phase(combos, all_results, t0, rl, num_workers=num_workers)
        all_results.extend(phase_results)
        print_top(phase_results, "Phase 6")

    # ─── Final report ───────────────────────────────────────────────
    elapsed = time.time() - t0
    save_results(all_results, outfile)

    print(f"\n{'='*80}")
    print(f"[{raceline_name}] Completed {len(all_results)} tests in {elapsed:.1f}s")
    print(f"Results saved to: {outfile}")
    print(f"{'='*80}")

    print_top(all_results, f"OVERALL ({raceline_name})", n=30)

    best_final = find_best_params(all_results, base_params)
    print(f"\n--- Best Configuration ({raceline_name}) ---")
    for k, v in sorted(best_final.items()):
        if k in PARAM_TO_DEFINE:
            base = base_params.get(k)
            marker = " ←CHANGED" if base is not None and abs(v - base) > 1e-6 else ""
            print(f"  {k:16s} = {v}{marker}")


def run_phase(combos, previous_results, t0, raceline=None, num_workers=1):
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
            results.append(r)

            if r["status"] != "OK":
                print(f"  → {r['status']}  (ETA {eta:.0f}s)")
            elif r["wall_collisions"] > 0:
                print(f"  → wc={r['wall_collisions']}  t5={r['time_above_5ms']:.0f}s  "
                      f"lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")
            elif r["failed"] > 0:
                print(f"  → FAIL {r['failed']}  (ETA {eta:.0f}s)")
            else:
                print(f"  → PASS sc={score:.2f}  t5={r['time_above_5ms']:.0f}s  "
                      f"vmax={r['max_vx']:.1f}  lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")
        return results

    jobs = [(label, params, raceline) for label, params in pending]
    done = 0
    with ProcessPoolExecutor(max_workers=num_workers) as executor:
        futures = [executor.submit(_run_single_parallel, job) for job in jobs]
        for fut in as_completed(futures):
            done += 1
            r = fut.result()
            results.append(r)

            elapsed = time.time() - t0
            rate = max((len(previous_results) + done) / max(elapsed, 0.01), 0.1)
            eta = (total - done) / rate

            if r["status"] != "OK":
                print(f"[{done:4d}/{total}] {r['label']:55s}   → {r['status']}  (ETA {eta:.0f}s)")
            elif r["wall_collisions"] > 0:
                print(f"[{done:4d}/{total}] {r['label']:55s}   → wc={r['wall_collisions']}  "
                      f"t5={r['time_above_5ms']:.0f}s  lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")
            elif r["failed"] > 0:
                print(f"[{done:4d}/{total}] {r['label']:55s}   → FAIL {r['failed']}  (ETA {eta:.0f}s)")
            else:
                print(f"[{done:4d}/{total}] {r['label']:55s}   → PASS sc={r['score']:.2f}  "
                      f"t5={r['time_above_5ms']:.0f}s  vmax={r['max_vx']:.1f}  "
                      f"lat={r['avg_lat_err']:.3f}  (ETA {eta:.0f}s)")

    return results


def find_best_params(results, base_params=None):
    """Return the params of the best result."""
    if base_params is None:
        base_params = BASE_PARAMS
    if not results:
        return dict(base_params)
    best = min(results, key=lambda r: r.get("score", 999))
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
    passing = [r for r in results if r.get("score", 999) < 500]
    best = sorted(results, key=lambda r: (r.get("wall_collisions", 99),
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

    fieldnames = ["label", "score", "passed", "failed",
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
            row = dict(r)
            # Flatten params into row
            for k, v in r.get("params", {}).items():
                row[k] = v
            writer.writerow(row)


if __name__ == "__main__":
    sys.exit(main() or 0)
