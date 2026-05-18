#!/usr/bin/env python3
"""Compare replay outputs and print top-N discrepancies.

Usage:
  compare_replays_top.py --cpu cpu.csv --fpga fpga.csv [--opencl opencl.csv] [--top 10]
"""
import argparse
import csv
import math

NUM_FIELDS = ["out_steer_fp", "out_accel_fp", "ey_fp", "epsi_fp", "prev_accel_in_fp"]


def load_by_idx(path):
    out = {}
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            out[int(row["idx"])]=row
    return out


def field_diff(a,b,field):
    va = a.get(field)
    vb = b.get(field)
    try:
        fa = float(va) if va is not None else 0.0
        fb = float(vb) if vb is not None else 0.0
        return abs(fa-fb)
    except Exception:
        return 0.0 if va==vb else float('inf')


def metric_for_pair(a,b):
    # combined L2 over numeric fields; status mismatch adds large penalty
    m=0.0
    for f in NUM_FIELDS:
        d = field_diff(a,b,f)
        if math.isinf(d):
            return float('inf')
        m += d*d
    m = math.sqrt(m)
    # status mismatch penalty
    if a.get('status')!=b.get('status'):
        m += 1e6
    return m


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cpu', required=True)
    p.add_argument('--fpga', required=True)
    p.add_argument('--opencl')
    p.add_argument('--top', type=int, default=10)
    args = p.parse_args()

    cpu = load_by_idx(args.cpu)
    fpga = load_by_idx(args.fpga)
    common = sorted(set(cpu.keys()) & set(fpga.keys()))
    if not common:
        print('No common indices between cpu and fpga')
        return

    diffs = []
    for idx in common:
        a = cpu[idx]
        b = fpga[idx]
        m = metric_for_pair(a,b)
        diffs.append((m, idx, a, b))

    diffs.sort(key=lambda x: (math.isinf(x[0]), -x[0]), reverse=False)
    # Put inf (non-numeric mismatches) at top
    # Now pick top N by descending metric
    topn = sorted(diffs, key=lambda x: x[0], reverse=True)[:args.top]

    print(f"Top {len(topn)} discrepancies (metric, idx):")
    for m, idx, a, b in topn:
        if math.isinf(m):
            print(f"IDX={idx} metric=INF (non-numeric mismatch e.g. string mismatch)")
        else:
            print(f"IDX={idx} metric={m:.6g}")
        # print per-field deltas
        parts = []
        for f in NUM_FIELDS:
            try:
                va = float(a.get(f, '0'))
                vb = float(b.get(f, '0'))
                parts.append(f"{f}: {va:.6g}/{vb:.6g} d={abs(va-vb):.6g}")
            except Exception:
                parts.append(f"{f}: {a.get(f)}/{b.get(f)}")
        # status and iters
        parts.append(f"status: {a.get('status')}/{b.get('status')}")
        parts.append(f"iters: {a.get('iters')}/{b.get('iters')}")
        print("  " + "; ".join(parts))

    # If opencl provided, also report top mismatches vs opencl
    if args.opencl:
        opencl = load_by_idx(args.opencl)
        common2 = sorted(set(fpga.keys()) & set(opencl.keys()))
        if common2:
            diffs2 = []
            for idx in common2:
                a = fpga[idx]
                b = opencl[idx]
                m = metric_for_pair(a,b)
                diffs2.append((m, idx, a, b))
            top2 = sorted(diffs2, key=lambda x: x[0], reverse=True)[:args.top]
            print('\nTop mismatches fpga vs opencl:')
            for m, idx, a, b in top2:
                print(f"IDX={idx} metric={m:.6g}")
                parts=[]
                for f in NUM_FIELDS:
                    try:
                        va=float(a.get(f,'0'))
                        vb=float(b.get(f,'0'))
                        parts.append(f"{f}: {va:.6g}/{vb:.6g} d={abs(va-vb):.6g}")
                    except:
                        parts.append(f"{f}: {a.get(f)}/{b.get(f)}")
                parts.append(f"status: {a.get('status')}/{b.get('status')}")
                print("  " + "; ".join(parts))

if __name__=='__main__':
    main()
