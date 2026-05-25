#!/usr/bin/env python3
import argparse
import csv
import math


def load(path):
    rows = {}
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        fields = r.fieldnames or []
        for row in r:
            idx = int(row["idx"])
            rows[idx] = row
    return rows, fields


def numeric_fields(fields):
    skip = {"idx", "stamp_ns"}
    return [f for f in fields if f not in skip]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cpu", required=True)
    ap.add_argument("--fpga", required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    cpu, cpu_fields = load(args.cpu)
    fpga, fpga_fields = load(args.fpga)

    common = sorted(set(cpu.keys()) & set(fpga.keys()))
    if not common:
        raise SystemExit("No common idx rows")

    fields = [f for f in numeric_fields(cpu_fields) if f in fpga_fields]

    stats = {f: {"max_abs": -1.0, "idx": None, "cpu": None, "fpga": None} for f in fields}
    worst_rows = []

    out_writer = None
    out_file = None
    if args.out:
        out_file = open(args.out, "w", newline="")
        out_cols = ["idx"] + [f"d_{f}" for f in fields]
        out_writer = csv.writer(out_file)
        out_writer.writerow(out_cols)

    for idx in common:
        c = cpu[idx]
        f = fpga[idx]
        row_abs_sum = 0.0
        diffs = []
        for name in fields:
            try:
                cv = float(c[name])
                fv = float(f[name])
            except Exception:
                continue
            d = fv - cv
            ad = abs(d)
            row_abs_sum += ad
            diffs.append(d)
            st = stats[name]
            if ad > st["max_abs"]:
                st["max_abs"] = ad
                st["idx"] = idx
                st["cpu"] = cv
                st["fpga"] = fv

        worst_rows.append((row_abs_sum, idx))
        if out_writer is not None:
            out_writer.writerow([idx] + diffs)

    if out_file is not None:
        out_file.close()

    print(f"common_rows={len(common)}")
    print("\nPer-field max abs diff:")
    for name in fields:
        st = stats[name]
        print(f"  {name}: max_abs={st['max_abs']:.9g} at idx={st['idx']} cpu={st['cpu']:.9g} fpga={st['fpga']:.9g}")

    worst_rows.sort(reverse=True)
    print(f"\nTop {args.top} rows by total |diff| sum:")
    for s, idx in worst_rows[:args.top]:
        print(f"  idx={idx} sum_abs_diff={s:.9g}")


if __name__ == "__main__":
    main()
