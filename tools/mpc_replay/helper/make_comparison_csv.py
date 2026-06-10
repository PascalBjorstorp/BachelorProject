#!/usr/bin/env python3
"""
Produce a single per-solve comparison CSV for either the CPU MPC baseline
or the FPGA UDP run.

Both CSVs share the same columns so the user can diff them directly:

  idx, stamp_ns, lap,
  pos_x, pos_y, theta,
  vx, vx_ref,
  e_y, e_psi, e_vx,
  iteration_count, compute_time_us,
  pipeline_latency_ms

Per-row meanings:
  pos_x / pos_y / theta   vehicle pose in the map frame [m, m, rad]
  vx / vx_ref             actual / reference longitudinal velocity [m/s]
  e_y / e_psi             lateral / heading tracking error (signed) [m, rad]
  e_vx                    vx - vx_ref (signed) [m/s]
  iteration_count         ADMM iterations the solver took for this solve
  compute_time_us         CPU /solve_us (baseline) OR FPGA kernel_us (fpga-udp)
                          — i.e. the pure compute time, comparable across both
  pipeline_latency_ms     CPU /ekf_to_control_ms (baseline) OR
                          Monitor/ekf_to_drive_ms (fpga-udp)
  lap                     race-lap number (2..max_lap) — warmup is dropped

Source-of-timing differences:
  baseline   /mpc/timing/iteration_count, /mpc/timing/solve_us,
             /mpc/timing/ekf_to_control_ms   (all in the bag)
  fpga-udp   ActualOutput/*.stats.csv  (iterations + kernel_us)
             Monitor/pipeline_latency_*.csv  (ekf_to_drive_ms)

Both modes share the lap-filter rule used elsewhere: discard lap 1 (warmup),
keep race laps 2..MAX_LAP.
"""

import argparse
import bisect
import csv
import subprocess
import sys
import tempfile
from pathlib import Path

import numpy as np


QP_SCALE = 262144.0
HORIZON = 20


# ---------- Frenet error helpers (same as plot_baseline_tracking.py) ----------

def wrap_pi(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def compute_frenet_errors(x, y, theta, ax, ay, bx, by, h0, h1):
    abx = bx - ax; aby = by - ay
    apx = x - ax;  apy = y - ay
    ab_len2 = abx * abx + aby * aby
    t = np.where(ab_len2 > 1e-12,
                 (apx * abx + apy * aby) / np.maximum(ab_len2, 1e-12), 0.0)
    t = np.clip(t, 0.0, 1.0)
    wx = ax + t * abx
    wy = ay + t * aby
    wpsi = h0 + t * wrap_pi(h1 - h0)
    e_y = -np.sin(wpsi) * (x - wx) + np.cos(wpsi) * (y - wy)
    e_psi = wrap_pi(theta - wpsi)
    return e_y, e_psi


# ---------- Lap detection ----------

def detect_laps(pos_x, pos_y, min_radius=2.0, min_lap_length=25.0):
    n = len(pos_x)
    if n < 2:
        return [(0, n)]
    sx, sy = pos_x[0], pos_y[0]
    dist = np.sqrt((pos_x - sx) ** 2 + (pos_y - sy) ** 2)
    seg = np.sqrt(np.diff(pos_x) ** 2 + np.diff(pos_y) ** 2)
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    starts = [0]; last_path = 0.0; in_zone = True
    for i in range(1, n):
        inside = dist[i] < min_radius
        if inside and not in_zone and (cum[i] - last_path) > min_lap_length:
            starts.append(i); last_path = cum[i]
        in_zone = inside
    starts.append(n)
    return [(starts[i], starts[i + 1]) for i in range(len(starts) - 1)]


# ---------- State CSV loader ----------

def fp_array(rows, key):
    return np.asarray([int(r[key]) for r in rows], dtype=np.int64) / QP_SCALE


def load_state_replay(path):
    rows = list(csv.DictReader(open(path)))
    if not rows:
        raise SystemExit(f"empty state CSV: {path}")
    return {
        "idx":      np.asarray([int(r["idx"]) for r in rows], dtype=np.int64),
        "stamp_ns": np.asarray([int(r["stamp_ns"]) for r in rows], dtype=np.int64),
        "x":        fp_array(rows, "x_fp"),
        "y":        fp_array(rows, "y_fp"),
        "theta":    fp_array(rows, "theta_fp"),
        "vx":       fp_array(rows, "velocity_fp"),
        "ref_x0":   fp_array(rows, "ref_x_fp_0"),
        "ref_y0":   fp_array(rows, "ref_y_fp_0"),
        "ref_x1":   fp_array(rows, "ref_x_fp_1"),
        "ref_y1":   fp_array(rows, "ref_y_fp_1"),
        "ref_psi0": fp_array(rows, "ref_psi_fp_0"),
        "ref_psi1": fp_array(rows, "ref_psi_fp_1"),
        "ref_vx0":  fp_array(rows, "ref_vx_fp_0"),
    }


def drop_warmup(d):
    mask = (d["x"] ** 2 + d["y"] ** 2) > 1e-4
    return {k: v[mask] for k, v in d.items()}


# ---------- Bag-topic timing extractor (baseline) ----------

def extract_baseline_timing(bag_path):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message
    from rclpy.serialization import deserialize_message

    iter_topic    = "/mpc/timing/iteration_count"
    solve_topic   = "/mpc/timing/solve_us"
    ekf2ctl_topic = "/mpc/timing/ekf_to_control_ms"

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_path), storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_types = {}
    for t in (iter_topic, solve_topic, ekf2ctl_topic):
        if t not in tmap:
            print(f"WARN: topic {t} missing from baseline bag", file=sys.stderr)
            continue
        msg_types[t] = get_message(tmap[t])

    events = {iter_topic: [], solve_topic: [], ekf2ctl_topic: []}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic in msg_types:
            msg = deserialize_message(data, msg_types[topic])
            events[topic].append((int(stamp_ns), float(msg.data)))
    for v in events.values():
        v.sort(key=lambda e: e[0])
    return events


def nearest_join(target_stamps, events, max_dt_ns=int(50e6)):
    """For each target stamp, find nearest event value within max_dt_ns; else NaN."""
    if not events:
        return np.full(len(target_stamps), np.nan)
    stamps = np.asarray([e[0] for e in events])
    values = np.asarray([e[1] for e in events])
    out = np.full(len(target_stamps), np.nan)
    for i, t in enumerate(target_stamps):
        j = int(np.searchsorted(stamps, t))
        cands = []
        if j < len(stamps): cands.append(j)
        if j > 0: cands.append(j - 1)
        if not cands: continue
        best = min(cands, key=lambda k: abs(stamps[k] - t))
        if abs(stamps[best] - t) <= max_dt_ns:
            out[i] = values[best]
    return out


# ---------- FPGA-side helpers ----------

def load_fpga_stats(stats_csv):
    rows = list(csv.DictReader(open(stats_csv)))
    return {
        "iterations":     np.asarray([float(r["iterations"])     for r in rows]),
        "kernel_us":      np.asarray([float(r["kernel_us"])      for r in rows]),
        "total_call_us":  np.asarray([float(r["total_call_us"])  for r in rows]),
        "status":         np.asarray([int(r["status"])           for r in rows]),
    }


def load_monitor(monitor_csv, latency_col="ekf_to_drive_ms"):
    rows = list(csv.DictReader(open(monitor_csv)))
    if latency_col not in rows[0]:
        # fall back to scan_to_ackermann_ms if requested column is missing
        latency_col = "scan_to_ackermann_ms"
    return [
        (int(r["wall_time_ns"]), float(r[latency_col]))
        for r in rows
    ]


# ---------- Main builders ----------

def common_postprocess(d, max_lap):
    """Compute Frenet errors + lap numbers, drop everything outside laps 2..max_lap."""
    e_y, e_psi = compute_frenet_errors(
        d["x"], d["y"], d["theta"],
        d["ref_x0"], d["ref_y0"],
        d["ref_x1"], d["ref_y1"],
        d["ref_psi0"], d["ref_psi1"],
    )
    d["e_y"] = e_y
    d["e_psi"] = e_psi
    d["e_vx"] = d["vx"] - d["ref_vx0"]

    laps = detect_laps(d["x"], d["y"])
    print(f"detected {len(laps)} laps (post-warmup)")
    for i, (s, e) in enumerate(laps, start=1):
        kept = "[keep]" if 2 <= i <= max_lap else "[drop]"
        print(f"  lap {i:>2}: rows [{s:>6}, {e:>6})  length={e-s}  {kept}")

    # Build per-row lap number; -1 for dropped rows
    lap_num = np.full(len(d["x"]), -1, dtype=np.int64)
    for i, (s, e) in enumerate(laps, start=1):
        if 2 <= i <= max_lap:
            lap_num[s:e] = i
    d["lap"] = lap_num
    return d


def write_csv(d, out_path, extra_cols=None):
    """Write the comparison CSV, keeping only rows with valid lap number."""
    keep = d["lap"] > 0
    cols = ["idx", "stamp_ns", "lap",
            "pos_x", "pos_y", "theta",
            "vx", "vx_ref",
            "e_y", "e_psi", "e_vx",
            "iteration_count", "compute_time_us", "pipeline_latency_ms"]
    rename = {
        "pos_x": "x", "pos_y": "y", "theta": "theta",
        "vx": "vx", "vx_ref": "ref_vx0",
    }
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(cols)
        for i in np.where(keep)[0]:
            row = [
                int(d["idx"][i]),
                int(d["stamp_ns"][i]),
                int(d["lap"][i]),
                f"{d['x'][i]:.6f}",
                f"{d['y'][i]:.6f}",
                f"{d['theta'][i]:.6f}",
                f"{d['vx'][i]:.6f}",
                f"{d['ref_vx0'][i]:.6f}",
                f"{d['e_y'][i]:.6f}",
                f"{d['e_psi'][i]:.6f}",
                f"{d['e_vx'][i]:.6f}",
                f"{d['iteration_count'][i]:.0f}"
                    if np.isfinite(d['iteration_count'][i]) else "",
                f"{d['compute_time_us'][i]:.3f}"
                    if np.isfinite(d['compute_time_us'][i]) else "",
                f"{d['pipeline_latency_ms'][i]:.4f}"
                    if np.isfinite(d['pipeline_latency_ms'][i]) else "",
            ]
            w.writerow(row)
    print(f"wrote {out_path} ({int(keep.sum())} rows for laps 2..{int(d['lap'][keep].max())})")


def synthesize_state_csv(bag_path, helper_dir, horizon=20):
    """Run export_mpc_state_csv_from_cpu_bag.py; return the path to the produced CSV."""
    tmp = tempfile.NamedTemporaryFile(suffix=".csv", delete=False)
    tmp.close()
    cmd = ["python3",
           str(helper_dir / "export_mpc_state_csv_from_cpu_bag.py"),
           "--bag", str(bag_path),
           "--out", tmp.name,
           "--horizon", str(horizon),
           "--steering-source", "auto"]
    print(f"running: {' '.join(cmd[:3])} ...")
    subprocess.run(cmd, check=True)
    return Path(tmp.name)


def build_baseline(bag_path, helper_dir, out_csv, max_lap):
    state_csv = synthesize_state_csv(bag_path, helper_dir)
    d = drop_warmup(load_state_replay(state_csv))
    print(f"state_replay synthesized: {len(d['idx'])} rows (post-warmup)")

    timing = extract_baseline_timing(bag_path)
    d["iteration_count"]      = nearest_join(d["stamp_ns"], timing["/mpc/timing/iteration_count"])
    d["compute_time_us"]      = nearest_join(d["stamp_ns"], timing["/mpc/timing/solve_us"])
    d["pipeline_latency_ms"]  = nearest_join(d["stamp_ns"], timing["/mpc/timing/ekf_to_control_ms"])

    d = common_postprocess(d, max_lap)
    write_csv(d, out_csv)


def build_fpga_udp(bag_path, stats_csv, monitor_csv, helper_dir, out_csv, max_lap):
    state_csv = synthesize_state_csv(bag_path, helper_dir)
    d = drop_warmup(load_state_replay(state_csv))
    print(f"state_replay synthesized: {len(d['idx'])} rows (post-warmup)")

    stats = load_fpga_stats(stats_csv)
    n_stats = len(stats["iterations"])
    n_state = len(d["idx"])
    print(f"FPGA stats rows: {n_stats}; state_replay rows: {n_state}")

    # The synthesized state_replay is keyed off /drive (one row per command).
    # The FPGA stats CSV logs one row per FPGA solve, which produced one
    # /drive. Align by truncating to min(n_state, n_stats). The drop_warmup
    # filter may have removed a leading region of state_replay; that region
    # was published before any FPGA solve happened, so dropping it is fine.
    n = min(n_state, n_stats)
    iter_full   = np.full(n_state, np.nan)
    kernel_full = np.full(n_state, np.nan)
    # If state_replay was trimmed at the start, line up the END of both.
    # (Both finish at run-end so this is the safer alignment.)
    iter_full[n_state - n:n_state]   = stats["iterations"][n_stats - n:n_stats]
    kernel_full[n_state - n:n_state] = stats["kernel_us"][n_stats - n:n_stats]
    d["iteration_count"] = iter_full
    d["compute_time_us"] = kernel_full
    # status != 0 (max-iter reached / infeasible) is real solver behavior, not
    # a failure to mask. Keep all rows so the user can analyze the full
    # iteration-count and compute-time distributions including the cap.
    n_max_iter = int(np.sum(stats["status"][n_stats - n:n_stats] == 1))
    if n_max_iter:
        print(f"note: {n_max_iter} FPGA solves hit MAX_ITERATIONS (status=1) "
              f"and are kept in the CSV with their real iteration/kernel values")

    monitor_events = load_monitor(monitor_csv)
    d["pipeline_latency_ms"] = nearest_join(d["stamp_ns"], monitor_events,
                                            max_dt_ns=int(200e6))  # 200 ms slack
    n_lat_ok = int(np.isfinite(d["pipeline_latency_ms"]).sum())
    print(f"matched {n_lat_ok}/{n_state} state rows to a Monitor latency sample within 200 ms")

    d = common_postprocess(d, max_lap)
    write_csv(d, out_csv)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["baseline", "fpga-udp"], required=True)
    ap.add_argument("--bag", required=True, help="bag .mcap")
    ap.add_argument("--out", required=True, help="output comparison CSV")
    ap.add_argument("--max-lap", type=int, default=11,
                    help="Keep race laps 2..MAX_LAP (default 11)")
    ap.add_argument("--stats-csv", default=None,
                    help="ActualOutput/*.stats.csv (required for --mode fpga-udp)")
    ap.add_argument("--monitor-csv", default=None,
                    help="Monitor/pipeline_latency_*.csv (required for --mode fpga-udp)")
    args = ap.parse_args()

    repo_root = Path(__file__).resolve().parents[3]
    helper_dir = Path(__file__).resolve().parent

    if args.mode == "baseline":
        build_baseline(Path(args.bag), helper_dir, Path(args.out), args.max_lap)
    else:
        if not args.stats_csv or not args.monitor_csv:
            raise SystemExit("--stats-csv and --monitor-csv are required for --mode fpga-udp")
        build_fpga_udp(Path(args.bag), Path(args.stats_csv), Path(args.monitor_csv),
                       helper_dir, Path(args.out), args.max_lap)


if __name__ == "__main__":
    main()
