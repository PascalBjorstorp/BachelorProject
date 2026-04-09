#!/usr/bin/env python3
"""
Test a single MPCC sweep configuration visually.

Usage:
    # Run standalone sim with verbose output (quick check):
    python3 test/test_sweep_config.py "QC=200+QL=1000+QP=5.0+N=12+dt=0.06"

    # Print env export commands (for pasting into terminal before ROS2 sim):
    python3 test/test_sweep_config.py --export "QC=200+QL=1000+QP=5.0+N=12+dt=0.06"

    # Override extra params on top of the label:
    python3 test/test_sweep_config.py "QC=200+QL=1000+QP=5.0+N=12+dt=0.06" --set Q_VY=5.0 R_DELTA=3.0

    # Use a different raceline:
    python3 test/test_sweep_config.py "QC=200+QL=1000+QP=5.0+N=12+dt=0.06" --raceline Spielberg_raceline.csv

Parses the sweep label string from tune_mpcc.py output, merges with
BASE_CONFIG, and either runs test_sim_drive with VERBOSE=1 or prints
export commands for the ROS2 simulation.
"""

import os
import re
import sys
import subprocess

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MPCC_DIR = os.path.dirname(SCRIPT_DIR)
PROJECT_DIR = os.path.dirname(MPCC_DIR)
TRAJ_DIR = os.path.join(PROJECT_DIR, "f1tenth_planning", "trajectories")

# Must match tune_mpcc.py BASE_CONFIG
BASE_CONFIG = {
    "Q_CONTOURING":      1000.0,
    "Q_LAG":             700.0,
    "Q_PROGRESS":        5.0,
    "Q_VX":              0.0,
    "VX_REF":            5.0,
    "Q_VY":              3.5,
    "Q_OMEGA":           0.7,
    "R_DELTA":           6.5,
    "R_AX":              0.014149,
    "R_VTHETA":          1.0,
    "W_DELTA_RATE":      2.0,
    "W_AX_RATE":         0.1,
    "W_VTHETA_RATE":     0.1,
    "Q_CONTOURING_TERM": 450.0,
    "Q_LAG_TERM":        950.0,
    "Q_PROGRESS_TERM":   5.0,
    "ADMM_RHO":          17.0,
    "ADMM_MAX_ITER":     50,
    "ADMM_TOL":          0.05,
    "HORIZON":           7,
    "DT":                0.035,
    "V_THETA_MAX":       8.0,
}

# Label abbreviation → env var name
LABEL_MAP = {
    "QC":   "Q_CONTOURING",
    "QL":   "Q_LAG",
    "QP":   "Q_PROGRESS",
    "N":    "HORIZON",
    "dt":   "DT",
    "QVY":  "Q_VY",
    "QOM":  "Q_OMEGA",
    "RD":   "R_DELTA",
    "WDR":  "W_DELTA_RATE",
    "VTM":  "V_THETA_MAX",
    "RHO":  "ADMM_RHO",
    "ITER": "ADMM_MAX_ITER",
    "TOL":  "ADMM_TOL",
    "RVT":  "R_VTHETA",
    "WVR":  "W_VTHETA_RATE",
    "QCT":  "Q_CONTOURING_TERM",
    "QLT":  "Q_LAG_TERM",
    "QPT":  "Q_PROGRESS_TERM",
}

INT_PARAMS = {"HORIZON", "ADMM_MAX_ITER"}


def parse_label(label: str) -> dict:
    """Parse a sweep label like 'QC=200+QL=1000+QP=5.0+N=12+dt=0.06' into env vars."""
    params = {}
    for token in label.split("+"):
        if "=" not in token:
            continue
        abbrev, val = token.split("=", 1)
        env_name = LABEL_MAP.get(abbrev, abbrev)
        if env_name in INT_PARAMS:
            params[env_name] = int(float(val))
        else:
            try:
                params[env_name] = float(val)
            except ValueError:
                params[env_name] = val
    return params


def build_binary():
    """Build the test_sim_drive binary, return path."""
    binary = os.path.join(MPCC_DIR, "test_sim_drive_visual")
    print("Building test binary...")
    ret = subprocess.run([
        "gcc",
        "-D_GNU_SOURCE", "-O3", "-std=c99", "-Wall", "-ffast-math",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-Wno-unused-function", "-Wno-unknown-pragmas",
        f"-I{MPCC_DIR}/include",
        f"{MPCC_DIR}/test/test_sim_drive.c",
        f"{MPCC_DIR}/src/mpcc.c",
        f"{MPCC_DIR}/src/mpcc_vehicle_model.c",
        f"{MPCC_DIR}/src/qp_solver_mpcc.c",
        "-lm", "-o", binary,
    ], capture_output=True, text=True)
    if ret.returncode != 0:
        print(f"Build failed:\n{ret.stderr}")
        sys.exit(1)
    print("Build OK.")
    return binary


def main():
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help"):
        print(__doc__)
        sys.exit(0)

    export_mode = False
    label = None
    extra = {}
    raceline = os.path.join(TRAJ_DIR, "my_track_raceline.csv")

    i = 0
    while i < len(args):
        if args[i] == "--export":
            export_mode = True
        elif args[i] == "--set" and i + 1 < len(args):
            for kv in args[i + 1:]:
                if kv.startswith("--"):
                    break
                if "=" in kv:
                    k, v = kv.split("=", 1)
                    extra[k] = v
                i += 1
        elif args[i] == "--raceline" and i + 1 < len(args):
            i += 1
            rl = args[i]
            if os.path.isabs(rl):
                raceline = rl
            elif os.path.exists(os.path.join(TRAJ_DIR, rl)):
                raceline = os.path.join(TRAJ_DIR, rl)
            else:
                raceline = os.path.abspath(rl)
        elif label is None and "=" in args[i]:
            label = args[i].strip().strip('"').strip("'")
        i += 1

    if label is None:
        print("ERROR: No sweep label provided.")
        print("Example: python3 test/test_sweep_config.py \"QC=200+QL=1000+QP=5.0+N=12+dt=0.06\"")
        sys.exit(1)

    # Merge: BASE_CONFIG ← label overrides ← extra --set overrides
    params = dict(BASE_CONFIG)
    label_params = parse_label(label)
    params.update(label_params)
    for k, v in extra.items():
        if k in INT_PARAMS:
            params[k] = int(float(v))
        else:
            try:
                params[k] = float(v)
            except ValueError:
                params[k] = v

    print(f"\n{'='*60}")
    print(f"Config: {label}")
    if extra:
        print(f"Extra:  {extra}")
    print(f"Raceline: {raceline}")
    print(f"{'='*60}")
    print("\nParameters:")
    for k, v in sorted(params.items()):
        marker = " <--" if k in label_params or k in extra else ""
        print(f"  {k:25s} = {v}{marker}")

    if export_mode:
        print(f"\n# Paste these into your terminal before launching the ROS2 sim:")
        print(f"# Then run: ros2 launch f1tenth_gym_ros gym_bridge_launch.py ground_truth:=true")
        print(f"# And:      ros2 launch mpcc_f1_10th mpcc_launch.py trajectory_file:={raceline}")
        print()
        for k, v in sorted(params.items()):
            print(f"export {k}={v}")
        print(f"export RACELINE_PATH={raceline}")
        return

    # Run standalone test_sim_drive with verbose output
    binary = build_binary()

    env = os.environ.copy()
    env["VERBOSE"] = "1"
    env["RACELINE_PATH"] = raceline
    for k, v in params.items():
        env[k] = str(v)

    print(f"\nRunning test_sim_drive (VERBOSE=1)...\n{'='*60}\n")
    result = subprocess.run([binary], env=env)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
