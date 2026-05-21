#!/usr/bin/env python3
"""Compare live Jetson CPU MPC iter count against PC CPU replay.

The hypothesis under test: was the Jetson stressed enough during the live
drive that the CPU MPC hit the iteration cap more often than the same code
does on an unloaded host?

Inputs
------
  --replay  replay_cpu_out.csv (cols: idx, stamp_ns, status, iters, ...)
  --bag     <bag>.mcap     (source of /mpc/timing/iteration_count and
                            /mpc/timing/solve_us)
  --out-dir output dir     (gets iters_compare.csv + summary.txt)

Alignment is by `stamp_ns` nearest-neighbour within 50 ms.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import os
import sys
from typing import Dict, List, Tuple


def extract_timing_from_bag(bag_path: str, out_csv: str) -> int:
    """Dump /mpc/timing/iteration_count + /mpc/timing/solve_us with timestamps."""
    try:
        import rclpy  # noqa: F401
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        print(f"ROS2 Python not available: {e}", file=sys.stderr)
        return 0

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=bag_path, storage_id="mcap"),
        ConverterOptions(input_serialization_format="cdr",
                         output_serialization_format="cdr"),
    )
    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    iter_topic = "/mpc/timing/iteration_count"
    solve_topic = "/mpc/timing/solve_us"
    seq_topic = "/mpc/timing/solver_enter_seq"
    if iter_topic not in type_map:
        print(f"Topic {iter_topic} not found in bag", file=sys.stderr)
        return 0
    f64_type = get_message(type_map[iter_topic])

    # The /mpc/timing/* messages are std_msgs/Float64 (no timestamp inside) —
    # use the bag's per-message receive timestamp instead.
    rows = []
    iter_count = 0
    solve_count = 0
    seq_count = 0
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic == iter_topic:
            msg = deserialize_message(data, f64_type)
            rows.append((int(t_ns), "iter", float(msg.data)))
            iter_count += 1
        elif topic == solve_topic:
            msg = deserialize_message(data, f64_type)
            rows.append((int(t_ns), "solve_us", float(msg.data)))
            solve_count += 1
        elif topic == seq_topic:
            msg = deserialize_message(data, f64_type)
            rows.append((int(t_ns), "seq", float(msg.data)))
            seq_count += 1

    rows.sort()
    # Merge into per-tick records keyed by stamp_ns. The publisher emits the
    # three timing topics back-to-back per cycle, so messages with the same
    # solver_enter_seq value belong to the same tick. We pair by the order
    # they arrive in the bag.
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_ns", "iter_count", "solve_us", "seq"])
        # Walk and collect triples. Use a tiny time-window grouping: any
        # messages within 5 ms of each other are the same tick.
        i = 0
        while i < len(rows):
            base_t = rows[i][0]
            iter_v = None
            solve_v = None
            seq_v = None
            j = i
            while j < len(rows) and rows[j][0] - base_t <= 5_000_000:
                _, kind, val = rows[j]
                if kind == "iter" and iter_v is None:
                    iter_v = val
                elif kind == "solve_us" and solve_v is None:
                    solve_v = val
                elif kind == "seq" and seq_v is None:
                    seq_v = val
                j += 1
            if iter_v is not None:
                w.writerow([base_t, int(iter_v),
                            f"{solve_v:.3f}" if solve_v is not None else "",
                            int(seq_v) if seq_v is not None else ""])
            i = j if j > i else i + 1

    print(f"Extracted iter={iter_count}, solve_us={solve_count}, seq={seq_count}")
    return iter_count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--max-dt-ms", type=float, default=50.0,
                    help="Max pairing dt in milliseconds (default 50)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    timing_csv = os.path.join(args.out_dir, "jetson_timing.csv")

    n_iter = extract_timing_from_bag(args.bag, timing_csv)
    if n_iter == 0:
        print("No iteration_count messages extracted; aborting", file=sys.stderr)
        return 1

    # Load Jetson timing
    jet = []
    with open(timing_csv) as f:
        for r in csv.DictReader(f):
            jet.append((int(r["stamp_ns"]),
                        int(r["iter_count"]),
                        float(r["solve_us"]) if r["solve_us"] else float("nan")))
    jet.sort(key=lambda x: x[0])
    j_stamps = [x[0] for x in jet]

    # Load PC replay
    replay = []
    with open(args.replay) as f:
        for r in csv.DictReader(f):
            replay.append((int(r["stamp_ns"]),
                           int(r["status"]),
                           int(r["iters"])))

    max_dt_ns = int(args.max_dt_ms * 1_000_000)
    pairs = []
    n_unpaired = 0
    for stamp, status, iters in replay:
        if stamp <= 0:
            continue
        i = bisect.bisect_left(j_stamps, stamp)
        best = i
        if i == len(j_stamps):
            best = i - 1
        elif i > 0 and abs(j_stamps[i - 1] - stamp) < abs(j_stamps[i] - stamp):
            best = i - 1
        dt = j_stamps[best] - stamp
        if abs(dt) > max_dt_ns:
            n_unpaired += 1
            continue
        j_stamp, j_iter, j_solve_us = jet[best]
        pairs.append((stamp, dt, status, iters, j_iter, j_solve_us))

    out_csv = os.path.join(args.out_dir, "iters_compare.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["stamp_ns", "dt_ns",
                    "replay_status", "replay_iters",
                    "jetson_iters", "jetson_solve_us",
                    "iter_diff"])
        for stamp, dt, status, riters, jiters, js in pairs:
            w.writerow([stamp, dt, status, riters, jiters, js, riters - jiters])

    # Stats
    def stats(vals):
        if not vals:
            return None
        n = len(vals)
        return {
            "n": n,
            "min": min(vals),
            "mean": sum(vals) / n,
            "max": max(vals),
        }

    r_iters = [p[3] for p in pairs]
    j_iters = [p[4] for p in pairs]
    diffs = [p[3] - p[4] for p in pairs]  # replay - jetson (negative => Jetson higher)

    # Max-iter (assume MAX_ITER = 50 — matches MPC_FPGA_MAX_ADMM_ITER default)
    MAX_ITER = 50
    r_max_iter = sum(1 for v in r_iters if v >= MAX_ITER)
    j_max_iter = sum(1 for v in j_iters if v >= MAX_ITER)
    both_max = sum(1 for r, j in zip(r_iters, j_iters) if r >= MAX_ITER and j >= MAX_ITER)
    only_jetson = sum(1 for r, j in zip(r_iters, j_iters) if j >= MAX_ITER and r < MAX_ITER)
    only_replay = sum(1 for r, j in zip(r_iters, j_iters) if r >= MAX_ITER and j < MAX_ITER)
    jetson_higher = sum(1 for d in diffs if d < 0)
    replay_higher = sum(1 for d in diffs if d > 0)
    equal = sum(1 for d in diffs if d == 0)
    abs_diffs = [abs(d) for d in diffs]

    # Histogram of jetson_iter - replay_iter (positive = Jetson harder)
    hist: Dict[int, int] = {}
    for d in diffs:
        k = -d  # invert so positive = Jetson higher
        hist[k] = hist.get(k, 0) + 1
    hist_sorted = sorted(hist.items(), key=lambda x: -x[1])[:15]

    n_p = len(pairs)
    summary_lines = [
        "=== Jetson live MPC vs PC CPU replay ===",
        f"replay rows : {len(replay)}",
        f"jetson rows : {len(jet)}",
        f"paired      : {n_p}  (unpaired {n_unpaired} replay rows > {args.max_dt_ms} ms gap)",
        "",
        f"replay iters: min={min(r_iters)} mean={sum(r_iters)/n_p:.2f} max={max(r_iters)}",
        f"jetson iters: min={min(j_iters)} mean={sum(j_iters)/n_p:.2f} max={max(j_iters)}",
        "",
        f"iter (jetson - replay) histogram (top 15, positive = Jetson harder):",
    ]
    for k, c in hist_sorted:
        summary_lines.append(f"  +{k:+d}: {c} ({c/n_p*100:.2f}%)")
    summary_lines.extend([
        "",
        f"exact match               : {equal} ({equal/n_p*100:.2f}%)",
        f"jetson higher than replay : {jetson_higher} ({jetson_higher/n_p*100:.2f}%)",
        f"replay higher than jetson : {replay_higher} ({replay_higher/n_p*100:.2f}%)",
        f"mean |diff|               : {sum(abs_diffs)/n_p:.2f} iters",
        "",
        f"MAX_ITER (>=50) on replay : {r_max_iter} ({r_max_iter/n_p*100:.2f}%)",
        f"MAX_ITER (>=50) on jetson : {j_max_iter} ({j_max_iter/n_p*100:.2f}%)",
        f"  both maxed              : {both_max}",
        f"  only jetson maxed       : {only_jetson}",
        f"  only replay maxed       : {only_replay}",
    ])

    # Stress proxy: per-row Jetson solve_us where finite
    finite_solve = [p[5] for p in pairs if p[5] == p[5]]  # not NaN
    if finite_solve:
        n = len(finite_solve)
        finite_solve.sort()
        p50 = finite_solve[n // 2]
        p90 = finite_solve[int(n * 0.9)]
        p99 = finite_solve[int(n * 0.99)]
        summary_lines.extend([
            "",
            f"jetson solve_us p50/p90/p99/max : {p50:.0f} / {p90:.0f} / {p99:.0f} / {max(finite_solve):.0f}",
        ])
        # When Jetson maxed out: what was solve_us?
        maxed_solve = [p[5] for p in pairs
                       if p[5] == p[5] and p[4] >= MAX_ITER]
        if maxed_solve:
            summary_lines.append(
                f"jetson solve_us when iters>=50  : "
                f"n={len(maxed_solve)} mean={sum(maxed_solve)/len(maxed_solve):.0f} "
                f"max={max(maxed_solve):.0f}")

    summary = "\n".join(summary_lines)
    print(summary)
    with open(os.path.join(args.out_dir, "summary_cpu_vs_jetson.txt"), "w") as f:
        f.write(summary + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
