#!/usr/bin/env python3
"""Closed-loop plant fit: make the simulated car COMPLETE the lap and match the
real ICP position and speed as closely as possible.

The open-loop identification gives a physically faithful plant, but what the user
ultimately needs is that the closed-loop simulation (the real controller, with the
sanctioned `*1.4` max-accel removed) reproduces the recorded run: same speed and
same line, all the way around, without crashing. This searches plant parameters to
minimise the closed-loop error against the ICP truth (odom is NOT used -- it reads
high in corners), with a large penalty for any lap it fails to finish.

Each evaluation runs the actual C simulator (~0.4 s), so a few thousand evaluations
(a global differential-evolution search) is entirely affordable.
"""
from __future__ import annotations

import argparse
import multiprocessing as mp
import os
import subprocess
import tempfile
from pathlib import Path

import numpy as np
import pandas as pd
from scipy.optimize import differential_evolution

from plant_model import PlantParameters
from export_sim_env import MAPPING

# Plant parameters searched in closed loop, with physical bounds. Centred on the
# open-loop fit / nominal car; the closed-loop objective selects within these.
SEARCH = {
    "accel_gain_pos": (0.60, 1.05), "accel_gain_neg": (0.50, 1.05),
    "accel_tau_pos_s": (0.01, 0.20), "accel_tau_neg_s": (0.01, 0.30),
    "roll_resistance_n": (2.0, 6.0), "drag_c2": (0.0, 0.15),
    "csf": (3.0, 8.0), "csr": (3.0, 9.0),
    "csf_high_slip": (1.0, 8.0), "csr_high_slip": (1.0, 9.0),
    "mu_front": (0.60, 1.05), "mu_rear": (0.50, 0.95),
    "iz_kgm2": (0.035, 0.070), "rel_len_front_m": (0.02, 0.40),
}


def build_targets(observed: pd.DataFrame, track_len: float, n_bins: int) -> dict:
    s = observed.path_s.to_numpy() % track_len
    edges = np.linspace(0.0, track_len, n_bins + 1)
    idx = np.clip(np.digitize(s, edges) - 1, 0, n_bins - 1)
    speed = observed.icp_speed.to_numpy()
    ey = observed.ey.to_numpy()
    tgt_speed = np.full(n_bins, np.nan)
    tgt_ey = np.full(n_bins, np.nan)
    for b in range(n_bins):
        m = idx == b
        if m.sum() >= 3:
            tgt_speed[b] = np.median(speed[m])
            tgt_ey[b] = np.median(ey[m])
    return {"edges": edges, "speed": tgt_speed, "ey": tgt_ey, "n_bins": n_bins}


def env_for(params: PlantParameters, sim_dt: float, base_env: dict) -> dict:
    env = dict(base_env)
    for field, var in MAPPING.items():
        value = getattr(params, field)
        if field == "steer_delay_s":
            env["SIM_STEER_DELAY_STEPS"] = str(int(round(value / sim_dt)))
        elif field == "accel_delay_s":
            env["SIM_ACCEL_DELAY_STEPS"] = str(int(round(value / sim_dt)))
        else:
            env[var] = f"{value:.10g}"
    return env


def run_sim(params: PlantParameters, binary: str, sim_dt: float, duration: float,
            base_env: dict) -> pd.DataFrame | None:
    with tempfile.NamedTemporaryFile(suffix=".csv", delete=False) as tf:
        trace_path = tf.name
    env = env_for(params, sim_dt, base_env)
    env["SIM_TRACE_LOG"] = trace_path
    env["SIM_DURATION"] = str(duration)
    env["SIM_DT"] = str(sim_dt)
    try:
        subprocess.run([binary], env={**os.environ, **env}, stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, timeout=60)
        df = pd.read_csv(trace_path)
    except Exception:
        return None
    finally:
        try:
            os.unlink(trace_path)
        except OSError:
            pass
    return df if len(df) > 10 else None


def score(df: pd.DataFrame | None, tgt: dict, track_len: float, duration: float,
          target_laps: float, w_ey: float) -> tuple[float, dict]:
    if df is None:
        return 1e6, {"laps": 0.0, "reason": "no-trace"}
    laps = float(df.completed_laps.max())
    partial = float(df.true_path_s.iloc[-1]) / track_len
    progress = laps + partial
    finished = float(df.sim_time_s.iloc[-1]) >= duration - 2.0 and int(df.wall_hit.iloc[-1]) == 0
    if (not finished) or progress < target_laps:
        # Climb toward completion: smaller penalty the further it got.
        return 1e4 * (target_laps - progress + 1.0), {"laps": laps, "progress": progress, "reason": "incomplete"}
    # Bin the sim trajectory by track position and compare to the real medians.
    s = df.true_path_s.to_numpy() % track_len
    idx = np.clip(np.digitize(s, tgt["edges"]) - 1, 0, tgt["n_bins"] - 1)
    sim_speed = np.hypot(df.vx.to_numpy(), df.vy.to_numpy())
    sim_ey = df.e_y.to_numpy()
    sp_err, ey_err, used = 0.0, 0.0, 0
    for b in range(tgt["n_bins"]):
        if not np.isfinite(tgt["speed"][b]):
            continue
        m = idx == b
        if m.sum() < 2:
            continue
        sp_err += (np.median(sim_speed[m]) - tgt["speed"][b]) ** 2
        ey_err += (np.median(sim_ey[m]) - tgt["ey"][b]) ** 2
        used += 1
    if used == 0:
        return 1e5, {"laps": laps, "reason": "no-bins"}
    speed_rmse = float(np.sqrt(sp_err / used))
    ey_rmse = float(np.sqrt(ey_err / used))
    cost = speed_rmse + w_ey * ey_rmse
    return cost, {"laps": laps, "speed_rmse": speed_rmse, "ey_rmse_m": ey_rmse, "cost": cost}


# Context shared with forked worker processes (Linux fork inherits module globals).
_CTX: dict = {}


def _make_params(x: np.ndarray) -> PlantParameters:
    import copy
    p = copy.deepcopy(_CTX["base"])
    for n, v in zip(_CTX["names"], x):
        setattr(p, n, float(v))
    return p


def _objective(x: np.ndarray) -> float:
    df = run_sim(_make_params(x), _CTX["binary"], _CTX["sim_dt"], _CTX["duration"], _CTX["base_env"])
    cost, _ = score(df, _CTX["tgt"], _CTX["track_len"], _CTX["duration"], _CTX["target_laps"], _CTX["w_ey"])
    return cost


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--observed", type=Path, required=True)
    ap.add_argument("--base-params", type=Path, required=True)
    ap.add_argument("--raceline", type=Path, required=True)
    ap.add_argument("--binary", type=str, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--sim-dt", type=float, default=0.002)
    ap.add_argument("--duration", type=float, default=50.0)
    ap.add_argument("--target-laps", type=float, default=4.0)
    ap.add_argument("--n-bins", type=int, default=60)
    ap.add_argument("--w-ey", type=float, default=4.0)
    ap.add_argument("--maxiter", type=int, default=40)
    ap.add_argument("--popsize", type=int, default=15)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--workers", type=int, default=-1)
    args = ap.parse_args()

    observed = pd.read_csv(args.observed)
    raceline = np.loadtxt(args.raceline, delimiter=",", comments="#")
    track_len = float(raceline[-1, 0])
    tgt = build_targets(observed, track_len, args.n_bins)
    base = PlantParameters.from_json(args.base_params)
    base_env = {"MPC_DT": "0.005", "PRED_DT": "0.03",
                "REALISTIC_TIRES": "1", "REALISTIC_ACTUATION": "1", "REALISTIC_DRIVE": "1"}
    names = list(SEARCH.keys())
    bounds = [SEARCH[n] for n in names]
    _CTX.update(base=base, names=names, binary=args.binary, sim_dt=args.sim_dt,
                duration=args.duration, base_env=base_env, tgt=tgt, track_len=track_len,
                target_laps=args.target_laps, w_ey=args.w_ey)

    x0 = np.array([float(np.clip(getattr(base, n), lo, hi)) for n, (lo, hi) in zip(names, bounds)])
    c0, info0 = score(run_sim(_make_params(x0), args.binary, args.sim_dt, args.duration, base_env),
                      tgt, track_len, args.duration, args.target_laps, args.w_ey)
    print(f"[init] cost={c0:.4f} {info0}")

    result = differential_evolution(
        _objective, bounds, x0=x0, maxiter=args.maxiter, popsize=args.popsize,
        seed=args.seed, workers=args.workers, polish=True, tol=1e-4,
        mutation=(0.5, 1.0), recombination=0.7, init="sobol", disp=True,
    )
    best = _make_params(result.x)
    df = run_sim(best, args.binary, args.sim_dt, args.duration, base_env)
    cost, info = score(df, tgt, track_len, args.duration, args.target_laps, args.w_ey)
    best.save_json(args.output, stage="closed_loop",
                   closed_loop_metrics={k: (float(v) if isinstance(v, (int, float)) else v) for k, v in info.items()},
                   searched={n: float(v) for n, v in zip(names, result.x)})
    print(f"[best] cost={cost:.4f} {info}")
    print("Wrote", args.output)
    return 0


if __name__ == "__main__":
    # Forked workers inherit _CTX (set before the optimiser builds its pool);
    # spawn would re-import the module with an empty context.
    try:
        mp.set_start_method("fork")
    except RuntimeError:
        pass
    raise SystemExit(main())
