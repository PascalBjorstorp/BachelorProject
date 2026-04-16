#!/usr/bin/env python3
"""
Analyze sweep run stability — compare best results across multiple sweep runs.

Reads the persistent log (tuning_results/sweep_best_log.jsonl) written by
tune_mpcc.py and shows:
  - Per-run best config & metrics
  - Mean / std / min / max for each weight and metric
  - Coefficient of variation (CV%) to flag unstable parameters

Usage:
    python3 test/analyze_sweep_stability.py                      # All runs
    python3 test/analyze_sweep_stability.py --objective racer    # Filter by objective
    python3 test/analyze_sweep_stability.py --last 5             # Last 5 runs only
"""

import json
import math
import os
import sys

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MPCC_DIR = os.path.dirname(SCRIPT_DIR)
LOG_PATH = os.path.join(MPCC_DIR, "tuning_results", "sweep_best_log.jsonl")

METRIC_KEYS = [
    "score", "avg_speed", "max_vx",
    "avg_contouring_err", "max_contouring_err",
    "avg_heading_err", "max_heading_err",
    "wall_collisions", "avg_solve_us", "avg_iters",
]

PARAM_PRINT_ORDER = [
    "Q_CONTOURING", "Q_LAG", "Q_PROGRESS",
    "Q_VY", "Q_OMEGA",
    "R_DELTA", "R_VTHETA",
    "W_DELTA_RATE", "W_VTHETA_RATE",
    "Q_CONTOURING_TERM", "Q_LAG_TERM", "Q_PROGRESS_TERM",
    "ADMM_RHO", "ADMM_MAX_ITER", "ADMM_TOL",
    "HORIZON", "DT", "V_THETA_MAX",
]


def load_entries(objective=None, last_n=None):
    if not os.path.exists(LOG_PATH):
        print(f"No log file found: {LOG_PATH}")
        print("Run tune_mpcc.py at least once to generate it.")
        sys.exit(1)

    entries = []
    with open(LOG_PATH) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            entry = json.loads(line)
            if objective and entry.get("objective") != objective:
                continue
            entries.append(entry)

    if last_n and last_n < len(entries):
        entries = entries[-last_n:]

    return entries


def stats(values):
    """Return (mean, std, min, max) for a list of numbers."""
    n = len(values)
    if n == 0:
        return (0.0, 0.0, 0.0, 0.0)
    mean = sum(values) / n
    if n == 1:
        return (mean, 0.0, mean, mean)
    variance = sum((v - mean) ** 2 for v in values) / (n - 1)
    return (mean, math.sqrt(variance), min(values), max(values))


def cv_pct(mean, std):
    """Coefficient of variation as percentage."""
    if abs(mean) < 1e-12:
        return 0.0 if std < 1e-12 else float("inf")
    return abs(std / mean) * 100.0


def main():
    objective = None
    last_n = None

    i = 1
    while i < len(sys.argv):
        if sys.argv[i] == "--objective" and i + 1 < len(sys.argv):
            objective = sys.argv[i + 1].strip().lower()
            i += 2
        elif sys.argv[i] == "--last" and i + 1 < len(sys.argv):
            last_n = int(sys.argv[i + 1])
            i += 2
        else:
            i += 1

    entries = load_entries(objective=objective, last_n=last_n)
    if not entries:
        print("No matching sweep runs found.")
        sys.exit(1)

    n_runs = len(entries)
    filter_desc = ""
    if objective:
        filter_desc += f" objective={objective}"
    if last_n:
        filter_desc += f" last {last_n}"

    print(f"\n{'='*90}")
    print(f"SWEEP STABILITY ANALYSIS — {n_runs} runs{filter_desc}")
    print(f"{'='*90}")

    # ---- Per-run summary table ----
    print(f"\n{'─'*90}")
    print("Per-run best results:")
    print(f"{'─'*90}")
    hdr = f"{'Run':>4}  {'Timestamp':<20} {'Obj':<8} {'Score':>9} {'AvgSpd':>7} "
    hdr += f"{'AvgEc':>8} {'MaxVx':>7} {'WC':>3} {'Tests':>7}"
    print(hdr)
    print("-" * 90)

    for i, e in enumerate(entries, 1):
        ts = e.get("timestamp", "?")[:19]
        print(f"{i:4d}  {ts:<20} {e.get('objective', '?'):<8} "
              f"{e.get('score', 0):9.3f} "
              f"{e.get('avg_speed', 0):7.2f} "
              f"{e.get('avg_contouring_err', 0):8.4f} "
              f"{e.get('max_vx', 0):7.2f} "
              f"{e.get('wall_collisions', '?'):>3} "
              f"{e.get('total_tests', '?'):>7}")

    # ---- Metric statistics ----
    print(f"\n{'─'*90}")
    print("Metric statistics across runs:")
    print(f"{'─'*90}")
    print(f"{'Metric':<25} {'Mean':>10} {'Std':>10} {'Min':>10} {'Max':>10} {'CV%':>8}")
    print("-" * 78)

    for key in METRIC_KEYS:
        vals = [e.get(key) for e in entries if e.get(key) is not None]
        vals = [float(v) for v in vals]
        if not vals:
            continue
        m, s, lo, hi = stats(vals)
        c = cv_pct(m, s)
        cv_flag = " <<<" if c > 15 and len(vals) > 1 else ""
        print(f"{key:<25} {m:10.4f} {s:10.4f} {lo:10.4f} {hi:10.4f} {c:7.1f}%{cv_flag}")

    # ---- Parameter statistics ----
    print(f"\n{'─'*90}")
    print("Weight/parameter statistics across runs:")
    print(f"{'─'*90}")
    print(f"{'Parameter':<25} {'Mean':>10} {'Std':>10} {'Min':>10} {'Max':>10} {'CV%':>8}")
    print("-" * 78)

    # Collect all parameter keys from the first entry's params
    all_param_keys = set()
    for e in entries:
        if "params" in e:
            all_param_keys.update(e["params"].keys())

    ordered_keys = [k for k in PARAM_PRINT_ORDER if k in all_param_keys]
    ordered_keys += sorted(all_param_keys - set(ordered_keys))

    for key in ordered_keys:
        vals = []
        for e in entries:
            v = e.get("params", {}).get(key)
            if v is not None:
                vals.append(float(v))
        if not vals:
            continue
        m, s, lo, hi = stats(vals)
        c = cv_pct(m, s)
        cv_flag = " <<<" if c > 15 and len(vals) > 1 else ""
        print(f"{key:<25} {m:10.4f} {s:10.4f} {lo:10.4f} {hi:10.4f} {c:7.1f}%{cv_flag}")

    # ---- Stability verdict ----
    if n_runs >= 2:
        print(f"\n{'─'*90}")
        print("Stability summary:")
        print(f"{'─'*90}")

        unstable_params = []
        for key in ordered_keys:
            vals = [float(e["params"][key]) for e in entries
                    if "params" in e and key in e["params"]]
            if len(vals) < 2:
                continue
            m, s, _, _ = stats(vals)
            c = cv_pct(m, s)
            if c > 15:
                unstable_params.append((key, c))

        score_vals = [float(e["score"]) for e in entries if e.get("score") is not None]
        if score_vals:
            sm, ss, slo, shi = stats(score_vals)
            sc = cv_pct(sm, ss)
            print(f"  Score CV:  {sc:.1f}%  (mean={sm:.4f}, std={ss:.4f})")
            if sc < 5:
                print("  -> Score is STABLE across runs (< 5% CV)")
            elif sc < 15:
                print("  -> Score is MODERATELY stable (5-15% CV)")
            else:
                print("  -> Score is UNSTABLE across runs (> 15% CV)")

        if unstable_params:
            print(f"\n  Parameters with high variance (CV > 15%):")
            for name, c in sorted(unstable_params, key=lambda x: -x[1]):
                print(f"    {name:<25s} CV = {c:.1f}%")
            print("  These weights may not be well-determined by the sweep.")
        else:
            print("  All parameters are stable across runs (CV <= 15%)")
    else:
        print(f"\nRun the sweep at least 2 times to see stability analysis.")

    print()


if __name__ == "__main__":
    main()
