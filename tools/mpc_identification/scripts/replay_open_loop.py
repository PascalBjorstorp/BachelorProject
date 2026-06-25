#!/usr/bin/env python3
"""Run one selected multiple-shooting split and export all prediction traces."""
from __future__ import annotations

import argparse
from pathlib import Path
import json
import pandas as pd

from fit_plant import evaluate
from plant_model import PlantParameters


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--observed", type=Path, required=True)
    parser.add_argument("--segments", type=Path, required=True)
    parser.add_argument("--drive", type=Path, required=True)
    parser.add_argument("--parameters", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--metrics", type=Path, required=True)
    parser.add_argument("--split", choices=["train", "validation", "test"], default="test")
    parser.add_argument("--sim-dt", type=float, default=0.002)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    observed = pd.read_csv(args.observed)
    segments = pd.read_csv(args.segments)
    selected = segments[segments.split == args.split]
    if len(selected) == 0:
        raise RuntimeError(f"No {args.split} segments")
    predictions, metrics = evaluate(PlantParameters.from_json(args.parameters), observed, selected,
                                    pd.read_csv(args.drive).sort_values("t_s"), args.sim_dt)
    predictions.to_csv(args.output, index=False)
    args.metrics.write_text(json.dumps({"split": args.split, **metrics}, indent=2) + "\n")
    print(json.dumps({"split": args.split, **metrics}, indent=2))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
