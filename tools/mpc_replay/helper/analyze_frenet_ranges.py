#!/usr/bin/env python3

# python3 analyze_frenet_ranges.py --in frenet_parity_eval_smoke2/cpu_frenet.csv
import argparse
import csv
import math


def required_int_bits_signed(absmax: float) -> int:
    if absmax <= 0.0:
        return 1
    return max(1, int(math.ceil(math.log2(absmax))) + 1)


def trunc_quantize(x: float, frac_bits: int) -> float:
    s = float(1 << frac_bits)
    y = x * s
    if y >= 0.0:
        q = math.floor(y)
    else:
        q = math.ceil(y)
    return q / s


def min_frac_bits_for_tol(values, tol: float, max_frac: int = 40):
    if tol <= 0.0:
        return None
    for fb in range(0, max_frac + 1):
        max_err = 0.0
        for v in values:
            q = trunc_quantize(v, fb)
            e = abs(v - q)
            if e > max_err:
                max_err = e
            if max_err > tol:
                break
        if max_err <= tol:
            return fb
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--in", dest="in_csv", required=True)
    ap.add_argument("--out", default="")
    ap.add_argument("--tols", default="1e-2,1e-3,1e-4,1e-5")
    args = ap.parse_args()

    tols = [float(x.strip()) for x in args.tols.split(",") if x.strip()]

    with open(args.in_csv, newline="") as f:
        r = csv.DictReader(f)
        fields = [c for c in (r.fieldnames or []) if c not in {"idx", "stamp_ns"}]
        vals = {c: [] for c in fields}
        for row in r:
            for c in fields:
                try:
                    vals[c].append(float(row[c]))
                except Exception:
                    pass

    rows = []
    for c in fields:
        v = vals[c]
        if not v:
            continue
        vmin = min(v)
        vmax = max(v)
        absmax = max(abs(vmin), abs(vmax))
        nonzero = [abs(x) for x in v if x != 0.0]
        min_nonzero = min(nonzero) if nonzero else 0.0
        int_bits = required_int_bits_signed(absmax)

        rec = {
            "field": c,
            "min": vmin,
            "max": vmax,
            "absmax": absmax,
            "min_nonzero_abs": min_nonzero,
            "int_bits_signed_min": int_bits,
        }

        if min_nonzero > 0.0:
            rec["frac_bits_min_nonzero"] = max(
                0, int(math.ceil(-math.log2(min_nonzero)))
            )
        else:
            rec["frac_bits_min_nonzero"] = 0

        for t in tols:
            fb = min_frac_bits_for_tol(v, t)
            key = f"frac_bits_tol_{t:g}"
            rec[key] = "NA" if fb is None else fb

        rows.append(rec)

    rows.sort(key=lambda x: x["field"])

    print(f"fields={len(rows)}")
    for r in rows:
        s = [
            f"{r['field']}",
            f"min={r['min']:.9g}",
            f"max={r['max']:.9g}",
            f"absmax={r['absmax']:.9g}",
            f"int_bits>={r['int_bits_signed_min']}",
            f"min_nonzero={r['min_nonzero_abs']:.9g}",
            f"frac_nonzero~{r['frac_bits_min_nonzero']}",
        ]
        for t in tols:
            s.append(f"fb@{t:g}={r[f'frac_bits_tol_{t:g}']}")
        print(" | ".join(s))

    if args.out:
        out_cols = [
            "field",
            "min",
            "max",
            "absmax",
            "min_nonzero_abs",
            "int_bits_signed_min",
            "frac_bits_min_nonzero",
        ] + [f"frac_bits_tol_{t:g}" for t in tols]

        with open(args.out, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=out_cols)
            w.writeheader()
            for r in rows:
                w.writerow(r)


if __name__ == "__main__":
    main()
