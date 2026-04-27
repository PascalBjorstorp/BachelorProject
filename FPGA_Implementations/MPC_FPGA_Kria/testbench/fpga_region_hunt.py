#!/usr/bin/env python3
"""
Autonomous FPGA tuning search focused on:
1) zero wall crashes
2) lower avg solver iterations

Two-phase process:
- Phase A: no-obstacle scenario only (race)
- Phase B: all deterministic scenarios (race + avoid_single + avoid_double)
- Phase C: solver tuple sweep (RHO/RHO_U/TOL) on best Phase B weight configs
"""

import csv
import os
import sys
import time
import math
import itertools
import random
import subprocess
from datetime import datetime

import fpga_tune_weights as tune

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
OUT_DIR = os.path.join(SCRIPT_DIR, "data")
os.makedirs(OUT_DIR, exist_ok=True)

PHASE_A_MULTS = [0.90, 0.95, 1.00, 1.05, 1.10]
PHASE_A_KEYS = [
    "Q_LAT", "Q_HDG", "Q_VEL", "Q_LAT_VEL", "Q_YAW",
    "R_STEER", "R_ACCEL", "W_JERK", "W_ACCEL_RATE",
]

SOLVER_RHO_VALUES = [8.0, 12.0, 18.0, 24.0, 28.0, 32.0, 36.0, 42.0]
SOLVER_RHO_U_VALUES = [12.0, 18.0, 24.0, 32.0, 42.0, 48.0, 56.0, 64.0]
SOLVER_TOL_VALUES = [0.03, 0.05, 0.08]
RANDOM_SAFE_SEARCH_COUNT = 1200

TOP_PHASEA_FOR_PHASEB = 48
TOP_PHASEB_FOR_SOLVER = 4


def setup_globals(include_obstacles: bool) -> None:
    """Initialize tuning module globals needed by run_test()."""
    tune.BASE.clear()
    tune.BASE.update(tune.BASE_CONFIG)
    tune.BASE["HORIZON"] = tune.FIXED_HORIZON_STEPS
    tune.BASE["PRED_DT"] = tune.FIXED_PRED_DT

    tune.RACELINE_PATH = os.path.abspath(tune.RACELINE_PATH)
    tune.RACELINE_TAG = tune.infer_raceline_tag(tune.RACELINE_PATH)

    meta = tune.load_raceline_metadata(tune.RACELINE_PATH)
    tune.TRACK_LENGTH_METERS = meta["track_length"]
    tune.RACELINE_START_LEFT_BOUND = meta["start_left_bound"]
    tune.RACELINE_START_RIGHT_BOUND = meta["start_right_bound"]

    tune.SCENARIO_RACELINE_PATHS = tune.build_scenario_raceline_paths(tune.RACELINE_PATH)
    tune.EVAL_SCENARIOS = tune.build_eval_scenarios(include_obstacles=include_obstacles)


def build_binary() -> str:
    """Build dedicated test binary once for this search."""
    binary_name = f"test_sim_drive_hunt_{os.getpid()}_{int(time.time())}"
    if os.name == "nt":
        binary_name += ".exe"
    binary = os.path.join(PROJECT_DIR, binary_name)

    xilinx_include_dir = tune.detect_xilinx_include_dir()
    if not xilinx_include_dir:
        raise RuntimeError("Could not locate ap_fixed.h include directory")

    build_cmd = [
        "g++", "-D_GNU_SOURCE", "-O3", "-DNDEBUG", "-std=c++17", "-Wall",
        "-march=native", "-mtune=native", "-flto",
        "-DMPC_HLS_TARGET", "-DMPC_RUNTIME_TUNE",
        f"-DMPC_FPGA_HORIZON_STEPS={tune.HORIZON_LIMIT}",
        f"-DPREDICTION_HORIZON={tune.HORIZON_LIMIT}",
        "-Wno-unknown-pragmas",
        f"-I{os.path.join(PROJECT_DIR, 'include')}",
        f"-I{xilinx_include_dir}",
        "-x", "c++",
    ]
    build_cmd.extend([os.path.join(PROJECT_DIR, rel_path) for rel_path in tune.KRIA_BUILD_SOURCES])
    build_cmd.extend(["-o", binary_name, "-lm"])

    ret = subprocess.run(build_cmd, cwd=PROJECT_DIR, capture_output=True, text=True)
    if ret.returncode != 0:
        raise RuntimeError(f"Build failed:\n{ret.stderr}")
    return binary


def canonical(cfg: dict) -> dict:
    p = dict(tune.BASE)
    p.update(cfg)
    p = tune.canonicalize_params(p)
    p["MAX_ITER"] = int(p.get("MAX_ITER", tune.BASE_CONFIG["MAX_ITER"]))
    return p


def safe_for_goal(r: dict) -> bool:
    # Primary requirement for this hunt: no wall crashes.
    return int(r.get("wall_collisions", 999)) == 0


def make_phase_a_candidates() -> list:
    """Generate weight region around known bootstrap seeds."""
    raw = []

    raw.append(canonical(dict(tune.BASE_CONFIG)))
    for seed in tune.PASS_REGION_BOOTSTRAP:
        s = dict(tune.BASE_CONFIG)
        s.update(seed)
        raw.append(canonical(s))

    bootstrap_pool = [dict(tune.BASE_CONFIG)] + [dict({**tune.BASE_CONFIG, **seed}) for seed in tune.PASS_REGION_BOOTSTRAP]

    for base in bootstrap_pool:
        base = canonical(base)
        for key in PHASE_A_KEYS:
            base_val = float(base[key])
            for m in PHASE_A_MULTS:
                w = dict(base)
                w[key] = round(base_val * m, 6)
                raw.append(canonical(w))

    # Wider conservative exploration: prioritize stability over speed to locate
    # zero-wall regions, then later reduce iterations.
    rng = random.Random(42)
    for _ in range(RANDOM_SAFE_SEARCH_COUNT):
        w = canonical({
            "Q_LAT": rng.uniform(9000.0, 16384.0),
            "Q_HDG": rng.uniform(220.0, 900.0),
            "Q_VEL": rng.uniform(12.0, 52.0),
            "Q_LAT_VEL": rng.uniform(3.5, 8.0),
            "Q_YAW": rng.uniform(1.0, 2.6),
            "R_STEER": rng.uniform(1.6, 4.5),
            "R_ACCEL": rng.uniform(0.0045, 0.0120),
            "W_JERK": rng.uniform(0.020, 0.080),
            "W_ACCEL_RATE": rng.uniform(0.080, 0.250),
            "RHO": rng.choice([6.0, 8.0, 12.0, 18.0, 24.0, 28.0, 32.0, 42.0]),
            "RHO_U": rng.choice([8.0, 12.0, 18.0, 24.0, 32.0, 42.0, 48.0, 64.0]),
            "TOL": rng.choice([0.02, 0.03, 0.05, 0.08, 0.10]),
            "MAX_ITER": 20,
        })
        raw.append(w)

    seen = set()
    uniq = []
    for w in raw:
        if not tune.is_valid_config(w):
            continue
        h = tune.config_hash(w)
        if h in seen:
            continue
        seen.add(h)
        uniq.append(w)
    return uniq


def eval_configs(configs: list, binary: str, phase: str) -> list:
    out = []
    total = len(configs)
    for idx, cfg in enumerate(configs, start=1):
        r = tune.run_test(cfg, binary, eval_scenarios=tune.EVAL_SCENARIOS)
        r = tune.apply_scores(r, "base")
        row = {
            "phase": phase,
            "idx": idx,
            "total": total,
            "safe_goal": int(safe_for_goal(r)),
            "status": r.get("status", ""),
            "wall_collisions": int(r.get("wall_collisions", 0)),
            "failed_non_speed": int(r.get("failed_non_speed", r.get("failed", 0))),
            "scenario_failures": int(r.get("scenario_failures", 0)),
            "avg_iters": float(r.get("avg_iters", 0.0)),
            "avg_vx": float(r.get("avg_vx", 0.0)),
            "avg_progress_mps": float(r.get("avg_progress_mps", 0.0)),
            "lap_time_est": float(r.get("lap_time_est", 999.0)),
            "solver_optimal_rate": float(r.get("solver_optimal_rate", 0.0)),
            "solver_max_iter_rate": float(r.get("solver_max_iter_rate", 0.0)),
        }
        for k in tune.BASE.keys():
            row[k] = cfg.get(k, tune.BASE[k])
        out.append(row)

        if idx % 20 == 0 or idx == total:
            safe_count = sum(int(x["safe_goal"]) for x in out)
            print(f"[{phase}] {idx}/{total} done | safe={safe_count}")
    return out


def write_rows(path: str, rows: list) -> None:
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        w.writerows(rows)


def pick_best_safe(rows: list, top_n: int) -> list:
    safe = [r for r in rows if int(r["safe_goal"]) == 1]
    safe.sort(key=lambda r: (
        float(r["wall_collisions"]),
        float(r["avg_iters"]),
        float(r["lap_time_est"]),
        -float(r["avg_progress_mps"]),
        float(r["failed_non_speed"]),
    ))
    return safe[:top_n]


def configs_from_rows(rows: list) -> list:
    out = []
    for r in rows:
        cfg = {}
        for k in tune.BASE.keys():
            v = r.get(k)
            if k in tune.INT_PARAMS:
                cfg[k] = int(float(v))
            else:
                cfg[k] = float(v)
        out.append(canonical(cfg))
    return out


def make_solver_candidates(weight_cfg: dict) -> list:
    out = []
    for rho, rho_u, tol in itertools.product(SOLVER_RHO_VALUES, SOLVER_RHO_U_VALUES, SOLVER_TOL_VALUES):
        w = dict(weight_cfg)
        w["RHO"] = rho
        w["RHO_U"] = rho_u
        w["TOL"] = tol
        w = canonical(w)
        if tune.is_valid_config(w):
            out.append(w)

    seen = set()
    uniq = []
    for w in out:
        h = tune.config_hash(w)
        if h in seen:
            continue
        seen.add(h)
        uniq.append(w)
    return uniq


def summarize_best(rows: list, title: str) -> None:
    safe = [r for r in rows if int(r["safe_goal"]) == 1]
    print(f"\n{title}")
    print(f"  total={len(rows)} safe={len(safe)}")
    if not safe:
        return
    safe.sort(key=lambda r: (float(r["avg_iters"]), float(r["lap_time_est"])))
    b = safe[0]
    print(
        "  best-safe "
        f"iters={b['avg_iters']:.2f} lap={b['lap_time_est']:.2f} "
        f"prog={b['avg_progress_mps']:.2f} "
        f"rho={b['RHO']:.3f} rho_u={b['RHO_U']:.3f} tol={b['TOL']:.5f}"
    )


def main() -> int:
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    setup_globals(include_obstacles=False)
    print("Building binary...")
    binary = build_binary()

    try:
        print("Phase A: no-obstacle region hunt...")
        phase_a_cfgs = make_phase_a_candidates()
        phase_a_rows = eval_configs(phase_a_cfgs, binary, "phaseA_no_obstacles")
        phase_a_file = os.path.join(OUT_DIR, f"fpga_region_hunt_phaseA_{timestamp}.csv")
        write_rows(phase_a_file, phase_a_rows)
        summarize_best(phase_a_rows, "Phase A summary")

        phase_a_best = pick_best_safe(phase_a_rows, TOP_PHASEA_FOR_PHASEB)
        if not phase_a_best:
            print("No safe config in Phase A. Expanding with raw bootstrap seeds only fallback.")
            fallback = []
            for seed in tune.PASS_REGION_BOOTSTRAP:
                w = canonical({**tune.BASE_CONFIG, **seed})
                if tune.is_valid_config(w):
                    fallback.append(w)
            phase_a_best_cfgs = fallback
        else:
            phase_a_best_cfgs = configs_from_rows(phase_a_best)

        print("Phase B: all 3 scenarios validation...")
        setup_globals(include_obstacles=True)
        phase_b_rows = eval_configs(phase_a_best_cfgs, binary, "phaseB_all_scenarios")
        phase_b_file = os.path.join(OUT_DIR, f"fpga_region_hunt_phaseB_{timestamp}.csv")
        write_rows(phase_b_file, phase_b_rows)
        summarize_best(phase_b_rows, "Phase B summary")

        phase_b_best = pick_best_safe(phase_b_rows, TOP_PHASEB_FOR_SOLVER)
        if not phase_b_best:
            print("No safe-all-scenarios config found. Exiting with artifacts for inspection.")
            print(f"PhaseA CSV: {phase_a_file}")
            print(f"PhaseB CSV: {phase_b_file}")
            return 2

        print("Phase C: solver sweep for lower avg_iters...")
        solver_rows = []
        for i, rr in enumerate(phase_b_best, start=1):
            base_cfg = configs_from_rows([rr])[0]
            solver_cfgs = make_solver_candidates(base_cfg)
            print(f"  Branch {i}/{len(phase_b_best)} solver candidates={len(solver_cfgs)}")
            branch_rows = eval_configs(solver_cfgs, binary, f"phaseC_solver_branch{i}")
            solver_rows.extend(branch_rows)

        solver_file = os.path.join(OUT_DIR, f"fpga_region_hunt_phaseC_{timestamp}.csv")
        write_rows(solver_file, solver_rows)
        summarize_best(solver_rows, "Phase C summary")

        all_rows = phase_a_rows + phase_b_rows + solver_rows
        all_file = os.path.join(OUT_DIR, f"fpga_region_hunt_all_{timestamp}.csv")
        write_rows(all_file, all_rows)

        print("\nArtifacts")
        print(f"  {phase_a_file}")
        print(f"  {phase_b_file}")
        print(f"  {solver_file}")
        print(f"  {all_file}")

        safe_solver = [r for r in solver_rows if int(r["safe_goal"]) == 1]
        if safe_solver:
            safe_solver.sort(key=lambda r: (float(r["avg_iters"]), float(r["lap_time_est"])))
            best = safe_solver[0]
            print("\nBEST_OVERALL_SAFE")
            print(
                f"iters={best['avg_iters']:.3f}, lap={best['lap_time_est']:.3f}, "
                f"progress={best['avg_progress_mps']:.3f}, "
                f"wall={best['wall_collisions']}, failed_non_speed={best['failed_non_speed']}"
            )
            for k in tune.MPC_TYPES_PRINT_ORDER:
                if k in best:
                    print(f"{k}={best[k]}")
        else:
            print("\nNo safe solver-sweep config. Check CSV for near-safe rows.")
            return 3

        return 0
    finally:
        try:
            os.remove(binary)
        except OSError:
            pass


if __name__ == "__main__":
    sys.exit(main())
