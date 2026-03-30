#!/usr/bin/env python3
import csv
import sys
from pathlib import Path


def parse_time_to_ns(text: str) -> float:
    s = text.strip()
    if not s:
        return float("inf")
    parts = s.split()
    if len(parts) != 2:
        return float("inf")
    value = float(parts[0])
    unit = parts[1].lower()
    if unit == "ns":
        return value
    if unit == "us":
        return value * 1_000.0
    if unit == "ms":
        return value * 1_000_000.0
    return float("inf")


def parse_clock_ns(text: str) -> float:
    return parse_time_to_ns(text)


def main() -> int:
    if len(sys.argv) > 2:
        print("Usage: rank_hls_sweep.py [summary_csv_path]")
        return 2

    default_csv = Path("logs/hls_sweep/hls_sweep_summary.csv")
    csv_path = Path(sys.argv[1]) if len(sys.argv) == 2 else default_csv

    if not csv_path.exists():
        print(f"ERROR: Summary CSV not found: {csv_path}")
        return 1

    rows = []
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            latency_abs = row.get("max_latency_abs", "")
            est_clock = row.get("estimated_clock_ns", "")
            dsp = int((row.get("dsp", "0") or "0").strip())
            lut = int((row.get("lut", "0") or "0").strip())
            ff = int((row.get("ff", "0") or "0").strip())
            bram = int((row.get("bram18k", "0") or "0").strip())

            row["_max_latency_ns"] = parse_time_to_ns(latency_abs)
            row["_est_clock_ns"] = parse_clock_ns(est_clock)
            row["_dsp"] = dsp
            row["_lut"] = lut
            row["_ff"] = ff
            row["_bram"] = bram
            rows.append(row)

    if not rows:
        print("No rows found in summary CSV.")
        return 0

    rows.sort(
        key=lambda r: (
            r["_max_latency_ns"],
            r["_est_clock_ns"],
            r["_dsp"],
            r["_lut"],
            r["_ff"],
            r["_bram"],
        )
    )

    print("Ranked profiles (best real-time speed first):")
    print("rank,profile,max_latency_abs,estimated_clock_ns,dsp,lut,ff,bram18k")
    for i, r in enumerate(rows, start=1):
        print(
            f"{i},{r.get('profile','')},{r.get('max_latency_abs','')},"
            f"{r.get('estimated_clock_ns','')},{r.get('dsp','')},"
            f"{r.get('lut','')},{r.get('ff','')},{r.get('bram18k','')}"
        )

    ranked_csv = csv_path.with_name("hls_sweep_ranked.csv")
    fieldnames = list(rows[0].keys())
    # Remove internal sort helper fields from saved CSV.
    fieldnames = [f for f in fieldnames if not f.startswith("_")]

    with ranked_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for r in rows:
            out = {k: v for k, v in r.items() if not k.startswith("_")}
            writer.writerow(out)

    print(f"\nSaved ranked CSV: {ranked_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
