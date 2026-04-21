#!/usr/bin/env python3
"""
Pure Pursuit realistic tuning sweep.
===================================
Runs single-scenario tuning for:
  ./build/f1tenth_control/test_sim_drive_pure_pursuit

This script mirrors the intent of MPC/test/tune_realistic_v2.py but targets
Pure Pursuit environment variables and parses PP_TUNING_CSV output.

Usage:
  python3 f1tenth_control/test/tune_realistic_v2.py
  python3 f1tenth_control/test/tune_realistic_v2.py -j 0
    python3 f1tenth_control/test/tune_realistic_v2.py --phase1 2000 --phase2 2000 --phase3 2000
"""

from __future__ import annotations

import argparse
import csv
import math
import multiprocessing
import os
import random
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional, Tuple

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
WORKSPACE_DIR = os.path.abspath(os.path.join(SCRIPT_DIR, "..", ".."))
DEFAULT_EXECUTABLE = os.path.join(
    WORKSPACE_DIR, "build", "f1tenth_control", "test_sim_drive_pure_pursuit"
)
DEFAULT_RACELINE = os.path.join(
    WORKSPACE_DIR, "f1tenth_planning", "trajectories", "my_track_raceline.csv"
)
RESULTS_DIR = os.path.join(SCRIPT_DIR, "results")

CSV_KEYS = [
    "tests_passed",
    "tests_failed",
    "max_lat_err",
    "avg_lat",
    "max_hdg_err",
    "avg_hdg",
    "max_vx",
    "avg_compute_us",
    "max_compute_us",
    "wall_collisions",
    "time_above_5ms",
    "max_vel_err",
    "avg_vel",
    "avg_vx",
    "progress_m",
    "avg_progress_mps",
    "completed_laps",
    "avg_lap_time",
    "max_abs_steer_change",
    "steer_reversals",
    "speed_check_pass",
    "failed_non_speed",
    "controller_success_rate",
]

INT_CSV_KEYS = {
    "tests_passed",
    "tests_failed",
    "wall_collisions",
    "completed_laps",
    "steer_reversals",
    "speed_check_pass",
    "failed_non_speed",
}

PARAM_ORDER = [
    "PP_MIN_LOOKAHEAD",
    "PP_MAX_LOOKAHEAD",
    "PP_LOOKAHEAD_GAIN",
    "PP_CTE_LOOKAHEAD_GAIN",
    "PP_CURV_LOOKAHEAD_GAIN",
    "PP_CURV_SPEED_FACTOR",
    "PP_CURV_SPEED_FLOOR",
    "PP_CTE_SPEED_FACTOR",
    "PP_CTE_SPEED_FLOOR",
    "PP_CURV_PREVIEW_FACTOR",
    "PP_MAX_SPEED",
    "PP_SPEED_TAU",
]

PARAM_BOUNDS = {
    "PP_MIN_LOOKAHEAD": (0.20, 0.55),
    "PP_MAX_LOOKAHEAD": (0.55, 1.50),
    "PP_LOOKAHEAD_GAIN": (0.03, 0.20),
    "PP_CTE_LOOKAHEAD_GAIN": (0.00, 0.15),
    "PP_CURV_LOOKAHEAD_GAIN": (0.60, 2.00),
    "PP_CURV_SPEED_FACTOR": (0.10, 1.20),
    "PP_CURV_SPEED_FLOOR": (0.20, 0.75),
    "PP_CTE_SPEED_FACTOR": (0.10, 1.80),
    "PP_CTE_SPEED_FLOOR": (0.20, 0.80),
    "PP_CURV_PREVIEW_FACTOR": (1.00, 3.50),
    "PP_MAX_SPEED": (4.0, 15.0),
    "PP_SPEED_TAU": (0.15, 0.65),
}

BASELINE_CANDIDATE = {
    "PP_MIN_LOOKAHEAD": 0.35,
    "PP_MAX_LOOKAHEAD": 1.20,
    "PP_LOOKAHEAD_GAIN": 0.10,
    "PP_CTE_LOOKAHEAD_GAIN": 0.08,
    "PP_CURV_LOOKAHEAD_GAIN": 1.34,
    "PP_CURV_SPEED_FACTOR": 0.30,
    "PP_CURV_SPEED_FLOOR": 0.43,
    "PP_CTE_SPEED_FACTOR": 0.50,
    "PP_CTE_SPEED_FLOOR": 0.50,
    "PP_CURV_PREVIEW_FACTOR": 1.20,
    "PP_MAX_SPEED": 12.0,
    "PP_SPEED_TAU": 0.30,
}


@dataclass
class Scenario:
    name: str
    overrides: Dict[str, str]


@dataclass
class RunResult:
    returncode: int
    csv_metrics: Optional[Dict[str, float]]
    stdout_tail: str
    stderr_tail: str


@dataclass
class CandidateEval:
    candidate: Dict[str, float]
    score: float
    feasible: bool
    wall_collisions: int
    failed_runs: int
    min_progress_m: float
    min_completed_laps: int
    avg_lap_time_s: float
    avg_progress_mps: float
    avg_vx_mps: float
    max_lat_err: float
    min_controller_success: float
    scenario_count: int


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def parse_csv_metrics(stdout: str) -> Optional[Dict[str, float]]:
    line = None
    for raw in stdout.splitlines():
        if raw.startswith("CSV,"):
            line = raw.strip()
    if line is None:
        return None

    parts = line.split(",")
    if len(parts) != len(CSV_KEYS) + 1:
        return None

    metrics: Dict[str, float] = {}
    for key, value in zip(CSV_KEYS, parts[1:]):
        if key in INT_CSV_KEYS:
            metrics[key] = int(float(value))
        else:
            metrics[key] = float(value)
    return metrics


def format_env_value(value: float) -> str:
    return f"{value:.8g}"


def sanitize_candidate(candidate: Dict[str, float]) -> Dict[str, float]:
    out = dict(candidate)
    for key, (lo, hi) in PARAM_BOUNDS.items():
        out[key] = clamp(float(out[key]), lo, hi)

    # Keep lookahead ordering valid.
    if out["PP_MAX_LOOKAHEAD"] < out["PP_MIN_LOOKAHEAD"] + 0.15:
        out["PP_MAX_LOOKAHEAD"] = out["PP_MIN_LOOKAHEAD"] + 0.15
        out["PP_MAX_LOOKAHEAD"] = clamp(
            out["PP_MAX_LOOKAHEAD"],
            PARAM_BOUNDS["PP_MAX_LOOKAHEAD"][0],
            PARAM_BOUNDS["PP_MAX_LOOKAHEAD"][1],
        )

    if out["PP_MAX_LOOKAHEAD"] < out["PP_MIN_LOOKAHEAD"] + 0.05:
        out["PP_MIN_LOOKAHEAD"] = out["PP_MAX_LOOKAHEAD"] - 0.05
        out["PP_MIN_LOOKAHEAD"] = clamp(
            out["PP_MIN_LOOKAHEAD"],
            PARAM_BOUNDS["PP_MIN_LOOKAHEAD"][0],
            PARAM_BOUNDS["PP_MIN_LOOKAHEAD"][1],
        )

    return out


def random_candidate(rng: random.Random) -> Dict[str, float]:
    c = {}
    for key, (lo, hi) in PARAM_BOUNDS.items():
        c[key] = rng.uniform(lo, hi)
    return sanitize_candidate(c)


def mutate_candidate(base: Dict[str, float], rng: random.Random, sigma_scale: float) -> Dict[str, float]:
    c = dict(base)
    for key, (lo, hi) in PARAM_BOUNDS.items():
        sigma = (hi - lo) * sigma_scale
        c[key] = base[key] + rng.gauss(0.0, sigma)
    return sanitize_candidate(c)


def candidate_key(candidate: Dict[str, float]) -> Tuple[float, ...]:
    return tuple(round(candidate[k], 6) for k in PARAM_ORDER)


def read_raceline_start_pose(raceline_path: str) -> Tuple[float, float, float]:
    with open(raceline_path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue

            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 4:
                continue

            try:
                x = float(parts[1])
                y = float(parts[2])
                heading = float(parts[3])
            except ValueError:
                continue

            return x, y, heading

    raise RuntimeError(f"could not parse first waypoint from raceline: {raceline_path}")


def run_once(
    executable: str,
    cwd: str,
    env_base: Dict[str, str],
    scenario_overrides: Dict[str, str],
    candidate: Dict[str, float],
    timeout_sec: float,
) -> RunResult:
    env = os.environ.copy()
    env.update(env_base)
    env.update(scenario_overrides)
    for key in PARAM_ORDER:
        env[key] = format_env_value(candidate[key])
    env["PP_TUNING_CSV"] = "1"

    proc = subprocess.run(
        [executable],
        cwd=cwd,
        env=env,
        capture_output=True,
        text=True,
        timeout=timeout_sec,
    )
    csv_metrics = parse_csv_metrics(proc.stdout)

    stdout_tail = "\n".join(proc.stdout.splitlines()[-20:])
    stderr_tail = "\n".join(proc.stderr.splitlines()[-20:])

    return RunResult(
        returncode=proc.returncode,
        csv_metrics=csv_metrics,
        stdout_tail=stdout_tail,
        stderr_tail=stderr_tail,
    )


def evaluate_candidate_worker(payload: dict) -> dict:
    executable = payload["executable"]
    cwd = payload["cwd"]
    env_base = payload["env_base"]
    scenarios = payload["scenarios"]
    candidate = payload["candidate"]
    timeout_sec = payload["timeout_sec"]

    scenario_results = []
    for sc in scenarios:
        run = run_once(
            executable=executable,
            cwd=cwd,
            env_base=env_base,
            scenario_overrides=sc["overrides"],
            candidate=candidate,
            timeout_sec=timeout_sec,
        )
        scenario_results.append((sc["name"], run))

    metrics_rows = []
    failed_runs = 0
    soft_fail_runs = 0
    wall_collisions = 0
    parse_failures = 0

    for _, run in scenario_results:
        if run.csv_metrics is None:
            parse_failures += 1
            failed_runs += 1
            continue
        metrics_rows.append(run.csv_metrics)
        wall_collisions += int(run.csv_metrics["wall_collisions"])
        if run.returncode != 0 or int(run.csv_metrics["tests_failed"]) > 0:
            soft_fail_runs += 1

    if not metrics_rows:
        return {
            "candidate": candidate,
            "score": -1e12,
            "feasible": False,
            "wall_collisions": wall_collisions,
            "failed_runs": failed_runs,
            "min_progress_m": 0.0,
            "min_completed_laps": 0,
            "avg_lap_time_s": 1e9,
            "avg_progress_mps": 0.0,
            "avg_vx_mps": 0.0,
            "max_lat_err": 999.0,
            "min_controller_success": 0.0,
            "scenario_count": len(scenarios),
            "parse_failures": parse_failures,
            "scenario_logs": [
                {
                    "name": name,
                    "returncode": run.returncode,
                    "stdout_tail": run.stdout_tail,
                    "stderr_tail": run.stderr_tail,
                }
                for name, run in scenario_results
            ],
        }

    min_progress_m = min(float(m["progress_m"]) for m in metrics_rows)
    min_completed_laps = min(int(m["completed_laps"]) for m in metrics_rows)
    lap_times = [
        float(m["avg_lap_time"])
        for m in metrics_rows
        if int(m["completed_laps"]) > 0 and float(m["avg_lap_time"]) > 1e-9
    ]
    avg_lap_time_s = (sum(lap_times) / len(lap_times)) if lap_times else 1e9
    avg_progress_mps = sum(float(m["avg_progress_mps"]) for m in metrics_rows) / len(metrics_rows)
    avg_vx_mps = sum(float(m["avg_vx"]) for m in metrics_rows) / len(metrics_rows)
    max_lat_err = max(float(m["max_lat_err"]) for m in metrics_rows)
    min_controller_success = min(float(m["controller_success_rate"]) for m in metrics_rows)

    feasible = (
        failed_runs == 0
        and parse_failures == 0
        and wall_collisions == 0
        and min_completed_laps >= 1
    )

    score = 0.0
    if feasible:
        score += 20000.0
    if wall_collisions == 0:
        score += 3000.0
    score += 1200.0 * min_completed_laps
    if avg_lap_time_s < 1e8:
        score += 12000.0 / avg_lap_time_s
    else:
        score -= 1000.0
    score += 0.8 * min_progress_m
    score += 8.0 * avg_progress_mps
    score += 25.0 * avg_vx_mps
    score += 250.0 * min_controller_success
    score -= 400.0 * wall_collisions
    score -= 200.0 * max_lat_err
    score -= 1200.0 * parse_failures
    score -= 50.0 * soft_fail_runs

    return {
        "candidate": candidate,
        "score": score,
        "feasible": feasible,
        "wall_collisions": wall_collisions,
        "failed_runs": failed_runs,
        "min_progress_m": min_progress_m,
        "min_completed_laps": min_completed_laps,
        "avg_lap_time_s": avg_lap_time_s,
        "avg_progress_mps": avg_progress_mps,
        "avg_vx_mps": avg_vx_mps,
        "max_lat_err": max_lat_err,
        "min_controller_success": min_controller_success,
        "scenario_count": len(scenarios),
        "parse_failures": parse_failures,
        "scenario_logs": [
            {
                "name": name,
                "returncode": run.returncode,
                "stdout_tail": run.stdout_tail,
                "stderr_tail": run.stderr_tail,
            }
            for name, run in scenario_results
        ],
    }


def evaluate_candidates(
    candidates: List[Dict[str, float]],
    executable: str,
    cwd: str,
    env_base: Dict[str, str],
    scenarios: List[Scenario],
    jobs: int,
    timeout_sec: float,
) -> List[CandidateEval]:
    if jobs == 0:
        max_workers = multiprocessing.cpu_count()
    else:
        max_workers = max(1, jobs)

    scenario_payload = [{"name": s.name, "overrides": s.overrides} for s in scenarios]

    payloads = [
        {
            "executable": executable,
            "cwd": cwd,
            "env_base": env_base,
            "scenarios": scenario_payload,
            "candidate": candidate,
            "timeout_sec": timeout_sec,
        }
        for candidate in candidates
    ]

    results: List[CandidateEval] = []

    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        fut_map = {ex.submit(evaluate_candidate_worker, payload): payload for payload in payloads}
        completed = 0
        total = len(payloads)
        t0 = time.time()

        for fut in as_completed(fut_map):
            completed += 1
            data = fut.result()
            results.append(
                CandidateEval(
                    candidate=data["candidate"],
                    score=float(data["score"]),
                    feasible=bool(data["feasible"]),
                    wall_collisions=int(data["wall_collisions"]),
                    failed_runs=int(data["failed_runs"]),
                    min_progress_m=float(data["min_progress_m"]),
                    min_completed_laps=int(data["min_completed_laps"]),
                    avg_lap_time_s=float(data["avg_lap_time_s"]),
                    avg_progress_mps=float(data["avg_progress_mps"]),
                    avg_vx_mps=float(data["avg_vx_mps"]),
                    max_lat_err=float(data["max_lat_err"]),
                    min_controller_success=float(data["min_controller_success"]),
                    scenario_count=int(data["scenario_count"]),
                )
            )

            if completed % max(1, total // 20) == 0 or completed == total:
                dt = time.time() - t0
                rate = completed / dt if dt > 1e-9 else 0.0
                print(
                    f"  progress: {completed:4d}/{total:4d} ({100.0*completed/total:5.1f}%)"
                    f"  rate={rate:.2f} cand/s"
                )

    results.sort(key=lambda r: r.score, reverse=True)
    return results


def make_scenarios(start_x: float, start_y: float, start_heading: float) -> List[Scenario]:
    return [
        Scenario(
            name="raceline_only_origin_start",
            overrides={
                "START_INDEX": "0",
                "START_OFFSET_LAT": "0",
                "START_OFFSET_X": format_env_value(-start_x),
                "START_OFFSET_Y": format_env_value(-start_y),
                "START_HEADING_OFFSET": format_env_value(-start_heading),
                "START_SPEED": "0",
            },
        ),
    ]


def save_results_csv(path: str, rows: List[CandidateEval]) -> None:
    fieldnames = [
        "rank",
        "score",
        "feasible",
        "wall_collisions",
        "failed_runs",
        "min_progress_m",
        "min_completed_laps",
        "avg_lap_time_s",
        "avg_progress_mps",
        "avg_vx_mps",
        "max_lat_err",
        "min_controller_success",
        "scenario_count",
    ] + PARAM_ORDER

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for idx, row in enumerate(rows, start=1):
            out = {
                "rank": idx,
                "score": row.score,
                "feasible": int(row.feasible),
                "wall_collisions": row.wall_collisions,
                "failed_runs": row.failed_runs,
                "min_progress_m": row.min_progress_m,
                "min_completed_laps": row.min_completed_laps,
                "avg_lap_time_s": row.avg_lap_time_s,
                "avg_progress_mps": row.avg_progress_mps,
                "avg_vx_mps": row.avg_vx_mps,
                "max_lat_err": row.max_lat_err,
                "min_controller_success": row.min_controller_success,
                "scenario_count": row.scenario_count,
            }
            for k in PARAM_ORDER:
                out[k] = row.candidate[k]
            writer.writerow(out)


def print_candidate(prefix: str, row: CandidateEval) -> None:
    lap_time_display = row.avg_lap_time_s if row.avg_lap_time_s < 1e8 else float("nan")
    print(
        f"{prefix} score={row.score:.2f} feasible={int(row.feasible)}"
        f" collisions={row.wall_collisions} failed_runs={row.failed_runs}"
        f" laps_min={row.min_completed_laps}"
        f" avg_lap={lap_time_display:.3f}s"
        f" progress_min={row.min_progress_m:.2f}m avg_vx={row.avg_vx_mps:.2f}"
        f" max_lat={row.max_lat_err:.3f} ctrl_ok_min={row.min_controller_success:.3f}"
    )
    for k in PARAM_ORDER:
        print(f"    {k:24s}= {row.candidate[k]:.8g}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Tune Pure Pursuit realistic simulator")
    parser.add_argument("-j", "--jobs", type=int, default=max(1, multiprocessing.cpu_count() // 2))
    parser.add_argument("--phase1", type=int, default=2000, help="Random candidates in coarse phase")
    parser.add_argument("--phase2", type=int, default=2000, help="Local refinement candidates")
    parser.add_argument("--phase3", type=int, default=2000, help="Exploit/explore mixed random phase")
    parser.add_argument("--topk", type=int, default=12, help="Seeds promoted between phases")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--timeout", type=float, default=25.0, help="Timeout per simulator run (seconds)")
    parser.add_argument("--sim-duration", type=float, default=75.0)
    parser.add_argument("--sim-dt", type=float, default=0.005)
    parser.add_argument("--pp-dt", type=float, default=0.005)
    parser.add_argument("--body-safety-margin", type=float, default=0.06)
    parser.add_argument("--raceline", type=str, default=DEFAULT_RACELINE)
    parser.add_argument("--executable", type=str, default=DEFAULT_EXECUTABLE)
    parser.add_argument("--realistic", action="store_true", default=True)
    parser.add_argument("--no-realistic", action="store_false", dest="realistic")
    parser.add_argument("--verbose-sim", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    executable = os.path.abspath(args.executable)
    if not os.path.exists(executable):
        print(f"ERROR: pure pursuit executable not found: {executable}")
        print("Build first: colcon build --packages-select f1tenth_control --symlink-install")
        return 2

    raceline_path = os.path.abspath(args.raceline)
    if not os.path.exists(raceline_path):
        print(f"ERROR: raceline file not found: {raceline_path}")
        return 2

    try:
        start_x, start_y, start_heading = read_raceline_start_pose(raceline_path)
    except RuntimeError as exc:
        print(f"ERROR: {exc}")
        return 2

    random.seed(args.seed)
    rng = random.Random(args.seed)

    scenarios = make_scenarios(start_x=start_x, start_y=start_y, start_heading=start_heading)
    env_base = {
        "RACELINE_PATH": raceline_path,
        "SIM_DURATION": f"{args.sim_duration:.8g}",
        "SIM_DT": f"{args.sim_dt:.8g}",
        "PP_DT": f"{args.pp_dt:.8g}",
        "BODY_SAFETY_MARGIN": f"{args.body_safety_margin:.8g}",
    }
    if args.verbose_sim:
        env_base["VERBOSE"] = "1"
    if args.realistic:
        env_base["REALISTIC_SIM"] = "1"
    else:
        env_base["REALISTIC_SIM"] = "0"

    print("=== Pure Pursuit Tuning (realistic v2 style) ===")
    print(f"Executable : {executable}")
    print(f"Raceline   : {raceline_path}")
    print(f"Scenarios  : {len(scenarios)}")
    for sc in scenarios:
        print(f"  - {sc.name}: {sc.overrides}")
    print("Start pose : forced x=0 y=0 heading=0 speed=0")
    print(
        f"Phase sizes: phase1={args.phase1}, phase2={args.phase2}, phase3={args.phase3}, topk={args.topk}, "
        f"jobs={args.jobs}"
    )
    print("")

    # Phase 1: baseline + random global exploration.
    phase1_candidates = [sanitize_candidate(dict(BASELINE_CANDIDATE))]
    seen = {candidate_key(phase1_candidates[0])}
    while len(phase1_candidates) < max(1, args.phase1):
        c = random_candidate(rng)
        ck = candidate_key(c)
        if ck in seen:
            continue
        seen.add(ck)
        phase1_candidates.append(c)

    print("[Phase 1] Coarse random exploration")
    phase1_results = evaluate_candidates(
        candidates=phase1_candidates,
        executable=executable,
        cwd=WORKSPACE_DIR,
        env_base=env_base,
        scenarios=scenarios,
        jobs=args.jobs,
        timeout_sec=args.timeout,
    )

    print_candidate("Best phase-1:", phase1_results[0])

    promoted = phase1_results[: max(1, min(args.topk, len(phase1_results)))]

    # Phase 2: local mutation around promoted seeds.
    phase2_candidates: List[Dict[str, float]] = []
    seen2 = set()
    phase2_candidates.append(dict(promoted[0].candidate))
    seen2.add(candidate_key(phase2_candidates[0]))

    while len(phase2_candidates) < max(1, args.phase2):
        base = promoted[len(phase2_candidates) % len(promoted)].candidate
        sigma_scale = 0.06 if len(phase2_candidates) < args.phase2 // 2 else 0.035
        c = mutate_candidate(base, rng, sigma_scale=sigma_scale)
        ck = candidate_key(c)
        if ck in seen2:
            continue
        seen2.add(ck)
        phase2_candidates.append(c)

    print("\n[Phase 2] Local refinement around top phase-1 seeds")
    phase2_results = evaluate_candidates(
        candidates=phase2_candidates,
        executable=executable,
        cwd=WORKSPACE_DIR,
        env_base=env_base,
        scenarios=scenarios,
        jobs=args.jobs,
        timeout_sec=args.timeout,
    )

    print_candidate("Best phase-2:", phase2_results[0])

    promoted_phase2 = phase2_results[: max(1, min(args.topk, len(phase2_results)))]

    # Phase 3: random exploitation + fresh exploration.
    phase3_candidates: List[Dict[str, float]] = []
    seen3 = set()
    while len(phase3_candidates) < max(1, args.phase3):
        if rng.random() < 0.6:
            base = promoted_phase2[len(phase3_candidates) % len(promoted_phase2)].candidate
            sigma_scale = 0.03 if len(phase3_candidates) < args.phase3 // 2 else 0.02
            c = mutate_candidate(base, rng, sigma_scale=sigma_scale)
        else:
            c = random_candidate(rng)

        ck = candidate_key(c)
        if ck in seen3:
            continue
        seen3.add(ck)
        phase3_candidates.append(c)

    print("\n[Phase 3] Mixed random search (exploit + explore)")
    phase3_results = evaluate_candidates(
        candidates=phase3_candidates,
        executable=executable,
        cwd=WORKSPACE_DIR,
        env_base=env_base,
        scenarios=scenarios,
        jobs=args.jobs,
        timeout_sec=args.timeout,
    )

    print_candidate("Best phase-3:", phase3_results[0])

    all_results = phase1_results + phase2_results + phase3_results
    all_results.sort(key=lambda r: r.score, reverse=True)

    best = all_results[0]

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = os.path.join(RESULTS_DIR, f"pure_pursuit_tuning_{timestamp}.csv")
    save_results_csv(out_csv, all_results)

    print("\n=== Final Best Candidate ===")
    print_candidate("Best:", best)
    print(f"\nSaved ranked candidates: {out_csv}")

    print("\nExport block (copy to environment):")
    for k in PARAM_ORDER:
        print(f"export {k}={best.candidate[k]:.8g}")

    if best.feasible:
        print("\nStatus: feasible across all scenarios (no crashes / no test failures).")
        return 0

    print("\nStatus: no fully feasible candidate found; inspect top rows in CSV.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
