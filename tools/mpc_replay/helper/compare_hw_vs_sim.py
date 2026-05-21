#!/usr/bin/env python3
"""Compare real hardware FPGA stats + bag /drive vs PC FPGA replay output.

Inputs:
  --stats   <hardware>.stats.csv  (cols: idx, iterations, status, ...)
  --replay  replay_fpga_out.csv   (cols: idx, stamp_ns, status, status_api, iters,
                                          out_steer_fp, out_accel_fp, ...)
  --bag     <bag>.mcap            (used to extract /drive messages)
  --out-dir output directory      (gets aligned.csv, drive_compare.csv, summary.txt)

Stats.csv carries no timestamp, so we lock its row offset to the replay output by
cross-correlating the iters/iterations sequence. /drive (which has timestamps) is
aligned to replay rows via the replay's stamp_ns column.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
from typing import Dict, List, Tuple

# Replay output writes fp values scaled by QP scaling factor; match the constant
# used by replay_fpga_scalar / dump_fpga_frenet. We read it from the header file.
def _read_qp_scale() -> int:
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, "..", "..", "..",
                                      "FPGA_Implementations", "MPC_FPGA_Kria",
                                      "include", "mpc_fpga_constants.h")),
    ]
    for path in candidates:
        if not os.path.exists(path):
            continue
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line.startswith("#define") and "MPC_FPGA_QP_SCALE" in line:
                    parts = line.split()
                    try:
                        return int(parts[2].rstrip("uUlL"))
                    except (IndexError, ValueError):
                        pass
    return 65536  # safe-ish default; will be overridden via --qp-scale if wrong


def read_csv_rows(path: str) -> List[Dict[str, str]]:
    with open(path) as f:
        return list(csv.DictReader(f))


def cross_correlate_iters(stats_iters: List[int],
                          replay_iters: List[int],
                          max_offset: int = 500) -> Tuple[int, float, int]:
    """Find integer offset k that maximizes per-row agreement.

    Returns (best_offset, best_agreement_rate, best_overlap_n).
    Offset semantics: replay row i corresponds to stats row (i + offset).
    A positive offset means stats started later than replay (replay had warm-up rows
    before the receiver started logging).
    """
    n_stats = len(stats_iters)
    n_replay = len(replay_iters)
    best = (-1, -1.0, 0)
    for off in range(-max_offset, max_offset + 1):
        # replay[i] vs stats[i + off]
        lo = max(0, -off)
        hi = min(n_replay, n_stats - off)
        n = hi - lo
        if n < 50:
            continue
        match = 0
        for i in range(lo, hi):
            if replay_iters[i] == stats_iters[i + off]:
                match += 1
        rate = match / n
        if rate > best[1]:
            best = (off, rate, n)
    return best


def extract_drive_from_bag(bag_path: str, out_csv: str) -> int:
    """Use ROS2 Python API to dump /drive messages: stamp_ns, steering, speed/accel."""
    try:
        from rosbags.highlevel import AnyReader  # type: ignore
        from pathlib import Path
        with AnyReader([Path(bag_path)]) as reader:
            conns = [c for c in reader.connections if c.topic == "/drive"]
            if not conns:
                print("No /drive topic in bag", file=sys.stderr)
                return 0
            with open(out_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["stamp_ns", "steering_angle", "speed",
                            "acceleration", "steering_angle_velocity", "jerk"])
                count = 0
                for conn, t, raw in reader.messages(connections=conns):
                    msg = reader.deserialize(raw, conn.msgtype)
                    hdr = msg.header
                    stamp_ns = hdr.stamp.sec * 1_000_000_000 + hdr.stamp.nanosec
                    d = msg.drive
                    w.writerow([stamp_ns, d.steering_angle, d.speed,
                                d.acceleration, d.steering_angle_velocity, d.jerk])
                    count += 1
                return count
    except ImportError:
        pass
    # Fallback: native rclpy/rosbag2_py
    try:
        import rclpy  # noqa: F401
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
        reader = SequentialReader()
        reader.open(
            StorageOptions(uri=bag_path, storage_id="mcap"),
            ConverterOptions(input_serialization_format="cdr",
                             output_serialization_format="cdr"),
        )
        topics = reader.get_all_topics_and_types()
        type_map = {t.name: t.type for t in topics}
        if "/drive" not in type_map:
            print("No /drive topic in bag", file=sys.stderr)
            return 0
        drive_type = get_message(type_map["/drive"])
        with open(out_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["stamp_ns", "steering_angle", "speed",
                        "acceleration", "steering_angle_velocity", "jerk"])
            count = 0
            while reader.has_next():
                topic, data, _ = reader.read_next()
                if topic != "/drive":
                    continue
                msg = deserialize_message(data, drive_type)
                stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                d = msg.drive
                w.writerow([stamp_ns, d.steering_angle, d.speed,
                            d.acceleration, d.steering_angle_velocity, d.jerk])
                count += 1
            return count
    except Exception as e:
        print(f"Failed to extract /drive: {e}", file=sys.stderr)
        return 0


def align_drive_to_replay(drive_csv: str,
                          replay_rows: List[Dict[str, str]],
                          qp_scale: int,
                          out_csv: str) -> Tuple[int, Dict[str, float]]:
    """For each replay row, find the /drive message with the closest stamp_ns.

    Writes one row per replay sample with both sides side-by-side.
    Returns (n_paired, summary_stats).
    """
    drive = []
    with open(drive_csv) as f:
        for r in csv.DictReader(f):
            drive.append((int(r["stamp_ns"]),
                          float(r["steering_angle"]),
                          float(r["speed"]),
                          float(r["acceleration"])))
    if not drive:
        return 0, {}
    drive.sort(key=lambda x: x[0])
    stamps = [d[0] for d in drive]

    import bisect
    n_paired = 0
    steer_errs = []
    accel_errs = []
    with open(out_csv, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["replay_idx", "stamp_ns", "dt_to_drive_ns",
                    "replay_steer", "drive_steer", "steer_err",
                    "replay_accel", "drive_accel", "accel_err",
                    "drive_speed"])
        for r in replay_rows:
            stamp_ns = int(r["stamp_ns"])
            if stamp_ns <= 0:
                continue
            i = bisect.bisect_left(stamps, stamp_ns)
            # nearest neighbour
            best_i = i
            if i == len(stamps):
                best_i = i - 1
            elif i > 0 and abs(stamps[i - 1] - stamp_ns) < abs(stamps[i] - stamp_ns):
                best_i = i - 1
            d_stamp, d_steer, d_speed, d_accel = drive[best_i]
            dt = d_stamp - stamp_ns
            # Skip nearest-neighbour matches farther than 50 ms — those aren't a real pair.
            if abs(dt) > 50_000_000:
                continue
            # Prefer the published (clamped + override-applied) values when
            # the replay emitted them — those are what /drive carries on the
            # car. Fall back to dividing the raw kernel output for older
            # replay CSVs.
            if "pub_steer_rad" in r and r["pub_steer_rad"] != "":
                r_steer = float(r["pub_steer_rad"])
                r_accel = float(r["pub_accel_mps2"])
            else:
                r_steer = float(r["out_steer_fp"]) / qp_scale
                r_accel = float(r["out_accel_fp"]) / qp_scale
            steer_err = r_steer - d_steer
            accel_err = r_accel - d_accel
            steer_errs.append(steer_err)
            accel_errs.append(accel_err)
            w.writerow([r["idx"], stamp_ns, dt,
                        f"{r_steer:.6f}", f"{d_steer:.6f}", f"{steer_err:.6f}",
                        f"{r_accel:.6f}", f"{d_accel:.6f}", f"{accel_err:.6f}",
                        f"{d_speed:.6f}"])
            n_paired += 1

    def _stats(vals):
        if not vals:
            return {"n": 0, "mean": 0.0, "abs_mean": 0.0, "rms": 0.0, "max_abs": 0.0}
        n = len(vals)
        mean = sum(vals) / n
        abs_mean = sum(abs(v) for v in vals) / n
        rms = (sum(v * v for v in vals) / n) ** 0.5
        max_abs = max(abs(v) for v in vals)
        return {"n": n, "mean": mean, "abs_mean": abs_mean,
                "rms": rms, "max_abs": max_abs}

    return n_paired, {"steer": _stats(steer_errs), "accel": _stats(accel_errs)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stats", required=True)
    ap.add_argument("--replay", required=True)
    ap.add_argument("--bag", required=True)
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--qp-scale", type=int, default=None,
                    help="QP fixed-point scale. Default reads from mpc_fpga_constants.h")
    ap.add_argument("--max-offset", type=int, default=500)
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    qp_scale = args.qp_scale or _read_qp_scale()

    stats_rows = read_csv_rows(args.stats)
    replay_rows = read_csv_rows(args.replay)
    print(f"Loaded {len(stats_rows)} hardware stats rows and "
          f"{len(replay_rows)} replay rows (QP_SCALE={qp_scale})")

    stats_iters = [int(r["iterations"]) for r in stats_rows]
    stats_status = [int(r["status"]) for r in stats_rows]
    replay_iters = [int(r["iters"]) for r in replay_rows]
    replay_status = [int(r["status"]) for r in replay_rows]

    off, rate, n = cross_correlate_iters(stats_iters, replay_iters,
                                          max_offset=args.max_offset)
    print(f"Best alignment: replay[i] <-> stats[i+{off}], "
          f"agreement={rate*100:.2f}% over n={n} rows")

    # write the aligned per-row file with both iters/status
    aligned_path = os.path.join(args.out_dir, "aligned_iters_status.csv")
    iter_match = 0
    status_match = 0
    iter_diff_hist: Dict[int, int] = {}
    status_pairs: Dict[Tuple[int, int], int] = {}
    n_overlap = 0
    with open(aligned_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["replay_idx", "stats_idx",
                    "replay_iters", "stats_iters", "iter_match",
                    "replay_status", "stats_status", "status_match"])
        lo = max(0, -off)
        hi = min(len(replay_iters), len(stats_iters) - off)
        for i in range(lo, hi):
            ri = replay_iters[i]
            si = stats_iters[i + off]
            rs = replay_status[i]
            ss = stats_status[i + off]
            im = int(ri == si)
            sm = int(rs == ss)
            iter_match += im
            status_match += sm
            d = ri - si
            iter_diff_hist[d] = iter_diff_hist.get(d, 0) + 1
            status_pairs[(rs, ss)] = status_pairs.get((rs, ss), 0) + 1
            n_overlap += 1
            w.writerow([i + 1, i + off + 1, ri, si, im, rs, ss, sm])

    # /drive comparison
    drive_csv = os.path.join(args.out_dir, "drive_extracted.csv")
    drive_n = extract_drive_from_bag(args.bag, drive_csv)
    print(f"Extracted {drive_n} /drive messages")
    paired = 0
    drive_stats = {}
    if drive_n:
        drive_compare_csv = os.path.join(args.out_dir, "drive_compare.csv")
        paired, drive_stats = align_drive_to_replay(drive_csv, replay_rows,
                                                     qp_scale, drive_compare_csv)
        print(f"Paired {paired} replay rows with /drive within 50 ms")

    summary_path = os.path.join(args.out_dir, "summary_hw_vs_sim.txt")
    with open(summary_path, "w") as f:
        f.write("=== Hardware (stats.csv) vs PC FPGA Replay ===\n")
        f.write(f"stats rows  : {len(stats_rows)}\n")
        f.write(f"replay rows : {len(replay_rows)}\n")
        f.write(f"best offset : replay[i] <-> stats[i+{off}]\n")
        f.write(f"overlap     : n={n_overlap}\n")
        f.write(f"iters match : {iter_match}/{n_overlap} "
                f"({iter_match/max(1,n_overlap)*100:.2f}%)\n")
        f.write(f"status match: {status_match}/{n_overlap} "
                f"({status_match/max(1,n_overlap)*100:.2f}%)\n")
        # show top iter diffs
        diffs_sorted = sorted(iter_diff_hist.items(), key=lambda x: -x[1])[:10]
        f.write("iter (replay - stats) diff histogram (top 10):\n")
        for d, c in diffs_sorted:
            f.write(f"  diff={d:+d}: {c} ({c/n_overlap*100:.2f}%)\n")
        f.write("status pairs (replay,stats) -> count:\n")
        for k, c in sorted(status_pairs.items(), key=lambda x: -x[1]):
            f.write(f"  {k}: {c}\n")
        if drive_stats:
            f.write("\n=== /drive vs replay control output ===\n")
            f.write(f"paired samples : {paired}\n")
            for name, s in drive_stats.items():
                f.write(f"{name}: n={s['n']} mean={s['mean']:+.4f} "
                        f"|mean|={s['abs_mean']:.4f} rms={s['rms']:.4f} "
                        f"max|err|={s['max_abs']:.4f}\n")
    print(f"\nWrote {summary_path}")
    with open(summary_path) as f:
        print(f.read())


if __name__ == "__main__":
    sys.exit(main())
