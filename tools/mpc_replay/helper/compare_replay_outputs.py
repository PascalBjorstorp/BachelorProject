#!/usr/bin/env python3
"""Compare two deterministic CPU-MPC replay outputs row by row.

The replay harness evaluates both controller revisions against the same recorded
state/reference sequence.  This script treats convergence regressions as test
failures and reports command changes separately; an offline replay cannot prove
that a changed command improves closed-loop tracking.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import sys
from pathlib import Path


QP_SCALE = 262144.0  # MPC_FPGA_QP_SCALE_F32
SUCCESS_STATUS = 0
MAX_ITERATION_STATUS = 1
ALIGNMENT_COLUMNS = (
    "stamp_ns",
    "ey_fp",
    "epsi_fp",
    "vx_fp",
    "vy_fp",
    "omega_fp",
    "steer_meas_fp",
)
REQUIRED_COLUMNS = {
    "idx",
    "status",
    "iters",
    "out_steer_fp",
    "out_accel_fp",
    *ALIGNMENT_COLUMNS,
}


def load_rows(path: Path) -> dict[int, dict[str, str]]:
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        missing_columns = REQUIRED_COLUMNS - set(reader.fieldnames or ())
        if missing_columns:
            raise ValueError(
                f"missing columns in {path}: {sorted(missing_columns)}"
            )
        rows = list(reader)
    if not rows:
        raise ValueError(f"empty replay CSV: {path}")

    indexed: dict[int, dict[str, str]] = {}
    for row in rows:
        idx = int(row["idx"])
        if idx in indexed:
            raise ValueError(f"duplicate idx {idx} in {path}")
        indexed[idx] = row
    return indexed


def percentile(values: list[float], fraction: float) -> float:
    """Return the nearest-rank percentile."""
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = max(0, math.ceil(len(ordered) * fraction) - 1)
    return ordered[position]


def distribution(values: list[float]) -> dict[str, float]:
    return {
        "mean": statistics.fmean(values),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "p99": percentile(values, 0.99),
        "max": max(values),
    }


def command_delta(
    baseline: dict[int, dict[str, str]],
    candidate: dict[int, dict[str, str]],
    indices: list[int],
    column: str,
) -> dict[str, float]:
    deltas = [
        (int(candidate[idx][column]) - int(baseline[idx][column])) / QP_SCALE
        for idx in indices
    ]
    absolute = [abs(value) for value in deltas]
    return {
        "mean_signed": statistics.fmean(deltas),
        "mean_absolute": statistics.fmean(absolute),
        "rms": math.sqrt(statistics.fmean(value * value for value in deltas)),
        "p95_absolute": percentile(absolute, 0.95),
        "p99_absolute": percentile(absolute, 0.99),
        "max_absolute": max(absolute),
        "exact_match_fraction": sum(value == 0.0 for value in deltas) / len(deltas),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("baseline", type=Path)
    parser.add_argument("candidate", type=Path)
    parser.add_argument(
        "--max-mean-iteration-increase",
        type=float,
        default=0.05,
        help="Allowed candidate minus baseline mean iterations (default: 0.05)",
    )
    parser.add_argument(
        "--max-p95-iteration-increase",
        type=float,
        default=1.0,
        help="Allowed candidate minus baseline p95 iterations (default: 1)",
    )
    parser.add_argument(
        "--max-p99-iteration-increase",
        type=float,
        default=1.0,
        help="Allowed candidate minus baseline p99 iterations (default: 1)",
    )
    parser.add_argument(
        "--max-non-success-increase",
        type=int,
        default=0,
        help="Allowed increase in all non-success statuses (default: 0)",
    )
    parser.add_argument(
        "--max-error-status-increase",
        type=int,
        default=0,
        help="Allowed increase in status values other than success/max-iter",
    )
    parser.add_argument(
        "--max-p95-steering-delta-rad",
        type=float,
        default=0.02,
        help="Maximum p95 absolute steering-command delta (default: 0.02 rad)",
    )
    parser.add_argument(
        "--max-p95-acceleration-delta-mps2",
        type=float,
        default=0.5,
        help="Maximum p95 absolute acceleration-command delta (default: 0.5 m/s^2)",
    )
    parser.add_argument("--json", type=Path, default=None)
    args = parser.parse_args()

    try:
        baseline = load_rows(args.baseline)
        candidate = load_rows(args.candidate)
    except (OSError, ValueError) as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 2
    baseline_indices = set(baseline)
    candidate_indices = set(candidate)
    if baseline_indices != candidate_indices:
        missing = sorted(baseline_indices - candidate_indices)[:5]
        extra = sorted(candidate_indices - baseline_indices)[:5]
        print(
            f"FAIL: row alignment differs: baseline={len(baseline_indices)} "
            f"candidate={len(candidate_indices)} missing={missing} extra={extra}",
            file=sys.stderr,
        )
        return 2

    indices = sorted(baseline_indices)
    input_mismatches = [
        (idx, column, baseline[idx][column], candidate[idx][column])
        for idx in indices
        for column in ALIGNMENT_COLUMNS
        if baseline[idx][column] != candidate[idx][column]
    ]
    if input_mismatches:
        print(
            "FAIL: replay inputs differ: "
            + "; ".join(
                f"idx={idx} {column}={before}->{after}"
                for idx, column, before, after in input_mismatches[:5]
            ),
            file=sys.stderr,
        )
        return 2

    baseline_iterations = [int(baseline[idx]["iters"]) for idx in indices]
    candidate_iterations = [int(candidate[idx]["iters"]) for idx in indices]
    baseline_status = [int(baseline[idx]["status"]) for idx in indices]
    candidate_status = [int(candidate[idx]["status"]) for idx in indices]

    baseline_maxiter = sum(value == MAX_ITERATION_STATUS for value in baseline_status)
    candidate_maxiter = sum(value == MAX_ITERATION_STATUS for value in candidate_status)
    baseline_non_success = sum(value != SUCCESS_STATUS for value in baseline_status)
    candidate_non_success = sum(value != SUCCESS_STATUS for value in candidate_status)
    baseline_errors = sum(
        value not in (SUCCESS_STATUS, MAX_ITERATION_STATUS)
        for value in baseline_status
    )
    candidate_errors = sum(
        value not in (SUCCESS_STATUS, MAX_ITERATION_STATUS)
        for value in candidate_status
    )
    baseline_iteration_distribution = distribution(
        [float(value) for value in baseline_iterations]
    )
    candidate_iteration_distribution = distribution(
        [float(value) for value in candidate_iterations]
    )
    steering_delta = command_delta(
        baseline, candidate, indices, "out_steer_fp"
    )
    acceleration_delta = command_delta(
        baseline, candidate, indices, "out_accel_fp"
    )

    report = {
        "rows": len(indices),
        "iterations": {
            "baseline": baseline_iteration_distribution,
            "candidate": candidate_iteration_distribution,
            "mean_delta": statistics.fmean(candidate_iterations)
            - statistics.fmean(baseline_iterations),
            "p95_delta": candidate_iteration_distribution["p95"]
            - baseline_iteration_distribution["p95"],
            "p99_delta": candidate_iteration_distribution["p99"]
            - baseline_iteration_distribution["p99"],
            "maxiter_baseline": baseline_maxiter,
            "maxiter_candidate": candidate_maxiter,
            "maxiter_delta": candidate_maxiter - baseline_maxiter,
        },
        "status": {
            "error_baseline": baseline_errors,
            "error_candidate": candidate_errors,
            "error_delta": candidate_errors - baseline_errors,
            "non_success_baseline": baseline_non_success,
            "non_success_candidate": candidate_non_success,
            "non_success_delta": candidate_non_success - baseline_non_success,
            "new_non_success_rows": sum(
                baseline_status[pos] == SUCCESS_STATUS
                and candidate_status[pos] != SUCCESS_STATUS
                for pos in range(len(indices))
            ),
            "new_success_rows": sum(
                baseline_status[pos] != SUCCESS_STATUS
                and candidate_status[pos] == SUCCESS_STATUS
                for pos in range(len(indices))
            ),
            "changed_rows": sum(
                baseline[idx]["status"] != candidate[idx]["status"] for idx in indices
            ),
        },
        "steering_delta_rad": steering_delta,
        "acceleration_delta_mps2": acceleration_delta,
    }

    failures: list[str] = []
    if report["iterations"]["mean_delta"] > args.max_mean_iteration_increase:
        failures.append(
            "mean iterations increased by "
            f"{report['iterations']['mean_delta']:.6g} "
            f"(limit {args.max_mean_iteration_increase:.6g})"
        )
    if (
        report["iterations"]["p95_delta"]
        > args.max_p95_iteration_increase
    ):
        failures.append(
            "p95 iterations increased by "
            f"{report['iterations']['p95_delta']:.6g} "
            f"(limit {args.max_p95_iteration_increase:.6g})"
        )
    if (
        report["iterations"]["p99_delta"]
        > args.max_p99_iteration_increase
    ):
        failures.append(
            "p99 iterations increased by "
            f"{report['iterations']['p99_delta']:.6g} "
            f"(limit {args.max_p99_iteration_increase:.6g})"
        )
    if report["status"]["non_success_delta"] > args.max_non_success_increase:
        failures.append(
            "non-success rows increased by "
            f"{report['status']['non_success_delta']} "
            f"(limit {args.max_non_success_increase})"
        )
    if report["status"]["error_delta"] > args.max_error_status_increase:
        failures.append(
            "error-status rows increased by "
            f"{report['status']['error_delta']} "
            f"(limit {args.max_error_status_increase})"
        )
    if (
        steering_delta["p95_absolute"]
        > args.max_p95_steering_delta_rad
    ):
        failures.append(
            "p95 steering delta is "
            f"{steering_delta['p95_absolute']:.6g} rad "
            f"(limit {args.max_p95_steering_delta_rad:.6g})"
        )
    if (
        acceleration_delta["p95_absolute"]
        > args.max_p95_acceleration_delta_mps2
    ):
        failures.append(
            "p95 acceleration delta is "
            f"{acceleration_delta['p95_absolute']:.6g} m/s^2 "
            f"(limit {args.max_p95_acceleration_delta_mps2:.6g})"
        )

    report["passed"] = not failures
    report["failures"] = failures
    rendered = json.dumps(report, indent=2, sort_keys=True)
    print(rendered)
    if args.json is not None:
        args.json.write_text(rendered + "\n", encoding="utf-8")

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
