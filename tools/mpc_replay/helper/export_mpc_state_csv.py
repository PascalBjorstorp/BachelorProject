#!/usr/bin/env python3
"""
Export /mpc_state messages from a rosbag2 MCAP into fixed-width CSV rows.

Usage:
  python3 export_mpc_state_csv.py \
    --bag /path/to/file.mcap \
    --out /path/to/mpc_state_replay.csv \
    --horizon 20
"""

import argparse
import csv
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


ARRAY_PREFIXES = [
    "ref_ey_fp",
    "ref_epsi_fp",
    "ref_x_fp",
    "ref_y_fp",
    "ref_psi_fp",
    "ref_vx_fp",
    "ref_vy_fp",
    "ref_omega_ref_fp",
    "ref_kappa_fp",
    "ref_left_bound_fp",
    "ref_right_bound_fp",
]


def fixed_len(arr, n):
    out = list(arr[:n])
    if len(out) < n:
        out.extend([0] * (n - len(out)))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True, help="Path to .mcap bag file")
    parser.add_argument("--out", required=True, help="Output CSV path")
    parser.add_argument("--horizon", type=int, default=20, help="Fixed horizon to export")
    args = parser.parse_args()

    if args.horizon < 2:
        print("horizon must be >= 2", file=sys.stderr)
        return 2

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=args.bag, storage_id="mcap"),
        ConverterOptions("cdr", "cdr"),
    )

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    if "/mpc_state" not in type_map:
        print("Topic /mpc_state not found in bag", file=sys.stderr)
        return 3

    mpc_state_type = get_message(type_map["/mpc_state"])

    header = [
        "idx",
        "stamp_sec",
        "stamp_nsec",
        "stamp_ns",
        "x_fp",
        "y_fp",
        "theta_fp",
        "velocity_fp",
        "vy_fp",
        "omega_fp",
        "steering_angle_fp",
        "horizon_length_msg",
    ]
    for prefix in ARRAY_PREFIXES:
        for i in range(args.horizon):
            header.append(f"{prefix}_{i}")
    header.extend(["e_y_fp", "e_psi_fp"])

    count = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)

        while reader.has_next():
            topic, data, _ = reader.read_next()
            if topic != "/mpc_state":
                continue
            msg = deserialize_message(data, mpc_state_type)
            count += 1

            stamp_sec = int(msg.header.stamp.sec)
            stamp_nsec = int(msg.header.stamp.nanosec)
            stamp_ns = stamp_sec * 1_000_000_000 + stamp_nsec

            row = [
                count,
                stamp_sec,
                stamp_nsec,
                stamp_ns,
                int(getattr(msg, "x_fp", 0)),
                int(getattr(msg, "y_fp", 0)),
                int(getattr(msg, "theta_fp", 0)),
                int(msg.velocity_fp),
                int(msg.vy_fp),
                int(msg.omega_fp),
                int(msg.steering_angle_fp),
                int(msg.horizon_length),
            ]

            arrays = {
                "ref_ey_fp": getattr(msg, "ref_ey_fp", []),
                "ref_epsi_fp": getattr(msg, "ref_epsi_fp", []),
                "ref_x_fp": getattr(msg, "ref_x_fp", []),
                "ref_y_fp": getattr(msg, "ref_y_fp", []),
                "ref_psi_fp": getattr(msg, "ref_psi_fp", []),
                "ref_vx_fp": getattr(msg, "ref_vx_fp", []),
                "ref_vy_fp": getattr(msg, "ref_vy_fp", []),
                "ref_omega_ref_fp": getattr(msg, "ref_omega_ref_fp", []),
                "ref_kappa_fp": getattr(msg, "ref_kappa_fp", []),
                "ref_left_bound_fp": getattr(msg, "ref_left_bound_fp", []),
                "ref_right_bound_fp": getattr(msg, "ref_right_bound_fp", []),
            }

            for prefix in ARRAY_PREFIXES:
                row.extend(fixed_len(arrays[prefix], args.horizon))

            # Newer /mpc_state messages carry Frenet errors directly.
            row.append(int(getattr(msg, "e_y_fp", 0)))
            row.append(int(getattr(msg, "e_psi_fp", 0)))

            w.writerow(row)

    print(f"Exported {count} /mpc_state rows to {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
