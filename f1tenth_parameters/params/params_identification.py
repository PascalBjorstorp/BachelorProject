#!/usr/bin/env python3
"""Inspect the lateral-dynamics identification test-suite design."""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT))

from params_identification.config import load_yaml
from params_identification.design import design_summary, validate_design


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=ROOT / "config" / "params_identification.yaml")
    parser.add_argument("--check", action="store_true", help="Validate the configured test matrix.")
    args = parser.parse_args()
    config = load_yaml(args.config)
    summary = design_summary(config)
    print(json.dumps(summary, indent=2))
    errors = validate_design(config)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        return 2
    if args.check:
        print("params identification design checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
