#!/usr/bin/env python3
import csv
import hashlib
import itertools
import math
import os
import random
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor, as_completed

PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BINARY_NAME = f"test_sim_drive_local_{os.getpid()}_{int(time.time())}"
if os.name == "nt":
    BINARY_NAME += ".exe"
BINARY_PATH = os.path.join(PROJECT_DIR, BINARY_NAME)
RACELINE = os.path.join(os.path.dirname(PROJECT_DIR), "f1tenth_planning", "trajectories", "hardware_raceline.csv")

BASE = {
    "ALPHA": 1.25,
    "HORIZON": 10,
    "MAX_ITER": 20,
    "PRED_DT": 0.04,
    "Q_HDG": 1000.0,
    "Q_LAT": 4500.0,
    "Q_LAT_VEL": 3.0,
    "Q_VEL": 142.56,
    "Q_YAW": 3.45,
    "RHO": 45.0,
    "RHO_U": 21.6,
    "R_ACCEL": 0.011,
    "R_STEER": 0.5,
    "TOL": 5.0,
    "WALL_END": 10,
    "WALL_MARGIN": 0.01,
    "WALL_STRIDE": 2,
    "W_ACCEL_RATE": 0.1,
    "W_JERK": 0.5,
}

SPACE = {
    "Q_LAT": [4200.0, 4400.0, 4500.0, 4700.0, 5000.0],
    "Q_HDG": [900.0, 1000.0, 1100.0],
    "Q_VEL": [140.0, 142.56, 150.0, 155.0, 160.0],
    "Q_LAT_VEL": [2.5, 3.0, 4.0, 5.0],
    "Q_YAW": [2.5, 3.45, 5.0, 8.0],
    "R_STEER": [0.4, 0.5, 0.6, 0.7],
    "R_ACCEL": [0.01, 0.011, 0.012],
    "W_JERK": [0.4, 0.5, 0.7],
    "RHO": [40.0, 45.0, 50.0],
    "RHO_U": [16.0, 20.0, 21.6, 24.0],
    "ALPHA": [1.2, 1.25, 1.4, 1.55],
    "TOL": [4.0, 5.0, 6.0],
    "HORIZON": [9, 10, 11],
    "PRED_DT": [0.036, 0.04, 0.042],
    "WALL_MARGIN": [0.008, 0.01, 0.012],
    "WALL_END": [10, 12],
    "WALL_STRIDE": [1, 2],
}

INT_PARAMS = {"HORIZON", "MAX_ITER", "WALL_END", "WALL_STRIDE"}


def build_binary():
    cmd = [
        "gcc", "-D_GNU_SOURCE", "-O2", "-std=c99", "-Wall",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable", "-Wno-unknown-pragmas",
        "-Iinclude",
        "test/test_sim_drive.c", "src/mpc_riccati.c", "src/riccati_solver.c",
        "src/vehicle_model.c", "src/util_math.c",
        "-o", BINARY_NAME, "-lm",
    ]
    ret = subprocess.run(cmd, cwd=PROJECT_DIR, capture_output=True, text=True)
    if ret.returncode != 0:
        raise RuntimeError(ret.stderr)


def deterministic_seed(params: dict, seed_base: int = 104) -> int:
    key = f"{seed_base}|{sorted(params.items())}"
    digest = hashlib.sha1(key.encode("utf-8")).hexdigest()
    return int(digest[:8], 16) % 100000


def run_one(item):
    idx, params = item
    env = os.environ.copy()
    env.update({
        "MPC_TUNING_CSV": "1",
        "REALISTIC_SIM": "1",
        "WALL_SOFT_K": "0",
        "RACELINE_PATH": RACELINE,
        "SIM_SEED": str(deterministic_seed(params)),
    })
    for k, v in params.items():
        env[k] = str(int(v) if k in INT_PARAMS else v)

    try:
        p = subprocess.run([BINARY_PATH], capture_output=True, text=True, timeout=20, env=env)
    except Exception:
        return None

    line = None
    for ln in p.stdout.splitlines():
        if ln.startswith("CSV,"):
            line = ln
            break
    if not line:
        return None

    parts = line.split(",")
    if len(parts) < 16:
        return None

    passed = int(parts[1])
    failed = int(parts[2])
    max_vx = float(parts[7])
    avg_solve = float(parts[8])
    wall_collisions = int(parts[10])
    t5 = float(parts[11])
    avg_lat = float(parts[4])
    avg_hdg = float(parts[6])
    avg_vx = float(parts[15])

    if wall_collisions > 0:
        score = 9999.0 + wall_collisions * 100.0
    else:
        score = -((avg_vx * 120.0) + (max_vx * 10.0) + (t5 * 0.5)) + (avg_solve * 0.0002)

    row = {
        "idx": idx,
        "score": round(score, 3),
        "avg_vx": avg_vx,
        "max_vx": max_vx,
        "avg_lat_err": avg_lat,
        "avg_hdg_err": avg_hdg,
        "passed": passed,
        "failed": failed,
        "wall_collisions": wall_collisions,
    }
    row.update(params)
    return row


def sample_candidates(n: int = 2500):
    rng = random.Random(1337)
    out = [dict(BASE)]
    keys = list(SPACE.keys())
    while len(out) < n:
        p = dict(BASE)
        perturb = rng.randint(4, 9)
        for k in rng.sample(keys, perturb):
            p[k] = rng.choice(SPACE[k])
        out.append(p)
    return out


def main():
    os.chdir(PROJECT_DIR)
    build_binary()

    candidates = sample_candidates(3000)
    best = []

    with ProcessPoolExecutor(max_workers=os.cpu_count() or 8) as ex:
        futures = [ex.submit(run_one, item) for item in enumerate(candidates)]
        for fut in as_completed(futures):
            r = fut.result()
            if r is None:
                continue
            best.append(r)

    best.sort(key=lambda x: x["score"])
    safe = [r for r in best if r["wall_collisions"] == 0]

    out_csv = os.path.join(PROJECT_DIR, "test", f"local_fast_search_{int(time.time())}.csv")
    if safe:
        fieldnames = list(safe[0].keys())
        with open(out_csv, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fieldnames)
            w.writeheader()
            w.writerows(safe)

        top = safe[0]
        print("OUTFILE", out_csv)
        print("BEST_SCORE", top["score"])
        print("BEST_AVG_VX", top["avg_vx"])
        print("BEST_MAX_VX", top["max_vx"])
        print("BEST_AVG_LAT", top["avg_lat_err"])
        print("BEST_AVG_HDG", top["avg_hdg_err"])
        print("BEST_IDX", top["idx"])
    else:
        print("NO_SAFE_RESULTS")

    try:
        os.remove(BINARY_PATH)
    except OSError:
        pass


if __name__ == "__main__":
    main()
