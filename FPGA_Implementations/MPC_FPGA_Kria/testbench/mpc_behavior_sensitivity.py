#!/usr/bin/env python3
import argparse
import csv
import os
import subprocess
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List

PROJECT_DIR = Path(__file__).resolve().parents[1]
SRC_FILES = [
    "testbench/test_fpga_sim_drive.c",
    "src/mpc_fpga_top.cpp",
    "src/riccati_solver_hls.cpp",
    "src/mpc_riccati_hls.cpp",
    "src/vehicle_model_hls.cpp",
    "src/fp_math_hls.cpp",
]

CSV_FIELDS = [
    "tests_passed",
    "tests_failed",
    "max_lat_err",
    "avg_lat_err",
    "max_hdg_err",
    "avg_hdg_err",
    "max_vx",
    "avg_solve_us",
    "max_solve_us",
    "wall_collisions",
    "time_above_5ms",
    "max_vel_err",
    "avg_vel_err",
    "avg_iters",
    "avg_primal_res",
    "max_primal_res",
    "avg_dual_res",
    "max_dual_res",
    "avg_rho",
    "avg_rho_u",
    "avg_abs_delta_rate",
    "avg_abs_delta_cmd_raw",
    "avg_abs_accel_raw",
    "avg_delta_rate_signed",
    "avg_delta_cmd_raw_signed",
    "avg_accel_raw_signed",
    "avg_ref_v0",
    "max_ref_v0",
    "avg_ref_vmean",
    "max_ref_vmean",
    "steer_sat_hits",
    "accel_sat_hits",
    "prev_conv_drops",
    "prev_conv_recovers",
]

DEFINE_MAP = {
    "acc_width": "MPC_HLS_ACC_WIDTH",
    "acc_int_bits": "MPC_HLS_ACC_INT_BITS",
    "diag_p_approx": "MPC_HLS_DIAG_P_APPROX",
    "max_admm_iter": "MPC_FPGA_MAX_ADMM_ITER",
    "ap_core_arith": "MPC_HLS_AP_CORE_ARITH",
    "admm_rho": "MPC_FPGA_ADMM_RHO",
    "admm_rho_u": "MPC_FPGA_ADMM_RHO_U",
    "admm_tol": "MPC_FPGA_ADMM_TOL",
    "w_velocity": "MPC_FPGA_W_VELOCITY",
    "w_lat_error": "MPC_FPGA_W_LAT_ERROR",
    "w_heading": "MPC_FPGA_W_HEADING",
    "w_lat_vel": "MPC_FPGA_W_LAT_VEL",
    "w_yaw_rate": "MPC_FPGA_W_YAW_RATE",
    "w_steer_eff": "MPC_FPGA_W_STEER_EFF",
    "w_accel_eff": "MPC_FPGA_W_ACCEL_EFF",
    "w_steer_jerk": "MPC_FPGA_W_STEER_JERK",
    "w_accel_rate": "MPC_FPGA_W_ACCEL_RATE",
    "w_delta_act": "MPC_FPGA_W_DELTA_ACT",
    "launch_lb": "MPC_FPGA_LAUNCH_ASSIST_LB",
    "launch_steps": "MPC_FPGA_LAUNCH_ASSIST_STEPS",
    "launch_vx_thresh": "MPC_FPGA_LAUNCH_ASSIST_VX_THRESH",
    "launch_ref_thresh": "MPC_FPGA_LAUNCH_ASSIST_REF_THRESH",
}


@dataclass
class RunResult:
    label: str
    cfg: Dict[str, str]
    metrics: Dict[str, float]


def parse_factor_spec(spec: str) -> Dict[str, List[str]]:
    factors: Dict[str, List[str]] = {}
    if not spec.strip():
        return factors
    for chunk in spec.split(";"):
        chunk = chunk.strip()
        if not chunk:
            continue
        if "=" not in chunk:
            raise ValueError(f"Invalid factor chunk: {chunk}")
        key, vals = chunk.split("=", 1)
        key = key.strip()
        values = [v.strip() for v in vals.split(",") if v.strip()]
        if not values:
            raise ValueError(f"No values for factor: {key}")
        factors[key] = values
    return factors


def to_define_args(cfg: Dict[str, str]) -> List[str]:
    out = ["-DMPC_HLS_TARGET"]
    for k, v in cfg.items():
        macro = DEFINE_MAP.get(k)
        if macro is None:
            raise ValueError(f"Unknown factor key: {k}")
        out.append(f"-D{macro}={v}")
    return out


def compile_binary(cfg: Dict[str, str], out_bin: Path) -> None:
    xilinx_hls = os.environ.get("XILINX_HLS")
    if not xilinx_hls:
        raise RuntimeError("XILINX_HLS is not set. Source Vitis settings first.")

    cmd = [
        "g++",
        "-O2",
        "-std=c++17",
        "-D_GNU_SOURCE",
        "-Iinclude",
        f"-I{xilinx_hls}/include",
        "-x",
        "c++",
    ]
    cmd.extend(to_define_args(cfg))
    cmd.extend(SRC_FILES)
    cmd.extend(["-lm", "-o", str(out_bin)])

    subprocess.run(cmd, cwd=PROJECT_DIR, check=True)


def run_binary(out_bin: Path) -> Dict[str, float]:
    env = os.environ.copy()
    env["MPC_TUNING_CSV"] = "1"
    p = subprocess.run([str(out_bin)], cwd=PROJECT_DIR, env=env, check=False, capture_output=True, text=True)

    csv_line = None
    for line in p.stdout.splitlines():
        if line.startswith("CSV,"):
            csv_line = line
    if csv_line is None:
        raise RuntimeError("CSV line not found in simulation output")

    parts = csv_line.split(",")
    if len(parts) < 1 + len(CSV_FIELDS):
        raise RuntimeError(f"Unexpected CSV column count: {len(parts)}")

    values = parts[1:1 + len(CSV_FIELDS)]
    metrics: Dict[str, float] = {}
    for key, raw in zip(CSV_FIELDS, values):
        if key in {"tests_passed", "tests_failed", "wall_collisions", "steer_sat_hits", "accel_sat_hits", "prev_conv_drops", "prev_conv_recovers"}:
            metrics[key] = float(int(float(raw)))
        else:
            metrics[key] = float(raw)
    return metrics


def write_raw(results: List[RunResult], out_csv: Path) -> None:
    keys = sorted(results[0].cfg.keys()) if results else []
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", *keys, *CSV_FIELDS])
        for r in results:
            w.writerow([r.label, *[r.cfg[k] for k in keys], *[r.metrics[k] for k in CSV_FIELDS]])


def write_effects(results: List[RunResult], baseline: RunResult, out_csv: Path) -> None:
    with out_csv.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "factor", "value", "metric", "delta_vs_baseline"])
        for r in results:
            if r.label == baseline.label:
                continue
            changed = [k for k in r.cfg.keys() if r.cfg[k] != baseline.cfg[k]]
            factor = changed[0] if len(changed) == 1 else "multi"
            value = r.cfg[factor] if len(changed) == 1 else "mixed"
            for m in CSV_FIELDS:
                w.writerow([r.label, factor, value, m, r.metrics[m] - baseline.metrics[m]])


def main() -> None:
    parser = argparse.ArgumentParser(description="Deterministic MPC behavior sensitivity study (simulation-only).")
    parser.add_argument("--name", default="behavior_sensitivity", help="Output name prefix")
    parser.add_argument(
        "--base",
        default="acc_width=28;acc_int_bits=16;diag_p_approx=1;max_admm_iter=6;ap_core_arith=1",
        help="Baseline config spec: key=value;key=value",
    )
    parser.add_argument(
        "--factors",
        default="acc_width=28,32,36,40;diag_p_approx=0,1;max_admm_iter=6,8,12,20",
        help="Deterministic OFAT factors: key=v1,v2;key=v1,v2",
    )
    args = parser.parse_args()

    base_cfg = {k: v[0] for k, v in parse_factor_spec(args.base).items()}
    factors = parse_factor_spec(args.factors)

    out_dir = PROJECT_DIR / "tuning_results"
    out_dir.mkdir(parents=True, exist_ok=True)

    run_cfgs: List[RunResult] = []

    def execute(label: str, cfg: Dict[str, str]) -> None:
        out_bin = PROJECT_DIR / "build" / f"{args.name}_{label}"
        print(f"[run] {label}: {cfg}")
        compile_binary(cfg, out_bin)
        metrics = run_binary(out_bin)
        run_cfgs.append(RunResult(label=label, cfg=dict(cfg), metrics=metrics))
        print(
            f"      max_vx={metrics['max_vx']:.2f} avg_vel_err={metrics['avg_vel_err']:.3f} "
            f"avg_lat={metrics['avg_lat_err']:.3f} walls={int(metrics['wall_collisions'])} "
            f"avg_iters={metrics['avg_iters']:.2f}"
        )

    execute("baseline", base_cfg)

    for factor, levels in factors.items():
        if factor not in base_cfg:
            # Add factor if missing in baseline using first level as default.
            base_cfg[factor] = levels[0]
        for lv in levels:
            if base_cfg[factor] == lv:
                continue
            cfg = dict(base_cfg)
            cfg[factor] = lv
            execute(f"{factor}_{lv}", cfg)

    raw_csv = out_dir / f"{args.name}_raw.csv"
    eff_csv = out_dir / f"{args.name}_effects.csv"
    write_raw(run_cfgs, raw_csv)
    write_effects(run_cfgs, run_cfgs[0], eff_csv)

    print(f"\nWrote: {raw_csv}")
    print(f"Wrote: {eff_csv}")


if __name__ == "__main__":
    main()
