#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import glob
import math
import random
import statistics
import tempfile
from pathlib import Path

try:
    from calibrate_plant_to_hardware import (
        build_sim_binary,
        evaluate_params,
        load_hardware_run,
        load_raceline,
        nearest_raceline_index,
    )
    from fit_turn_in_transient import compute_error as compute_transient_error
    from fit_turn_in_transient import load_turn_runs
except ModuleNotFoundError:
    from MPC.test.calibrate_plant_to_hardware import (
        build_sim_binary,
        evaluate_params,
        load_hardware_run,
        load_raceline,
        nearest_raceline_index,
    )
    from MPC.test.fit_turn_in_transient import compute_error as compute_transient_error
    from MPC.test.fit_turn_in_transient import load_turn_runs


def median(values: list[float]) -> float:
    if not values:
        raise RuntimeError("Cannot take median of empty sequence")
    return float(statistics.median(values))


def load_friction_prior(repo_root: Path) -> tuple[float, float]:
    mu_values: list[float] = []
    for path_str in sorted(glob.glob(str(repo_root / "f1tenth_parameters/data/friction_test_*.csv"))):
        with open(path_str) as handle:
            reader = csv.DictReader(handle)
            grouped: dict[str, list[float]] = {}
            for row in reader:
                grouped.setdefault(row["phase"], []).append(abs(float(row["imu_ay"])))
        phase_means = [sum(samples) / len(samples) for samples in grouped.values() if len(samples) >= 10]
        if phase_means:
            phase_means.sort()
            mu_values.append(sum(phase_means[-2:]) / min(2, len(phase_means)) / 9.81)
    return median(mu_values), max(0.02, statistics.pstdev(mu_values))


def load_servo_gain_prior(repo_root: Path, wheelbase_m: float) -> tuple[float, float]:
    ratios: list[float] = []
    for path_str in sorted(glob.glob(str(repo_root / "f1tenth_parameters/data/servo_calibration_test_*.csv"))):
        rows = list(csv.DictReader(open(path_str)))
        grouped: dict[str, list[dict[str, str]]] = {}
        for row in rows:
            grouped.setdefault(row["phase"], []).append(row)
        for phase, phase_rows in grouped.items():
            if not phase.startswith("circle_d"):
                continue
            cmd = abs(float(phase_rows[0]["cmd_steering"]))
            v_samples = []
            w_samples = []
            for row in phase_rows:
                vx = abs(float(row["odom_vx"]))
                wz = abs(float(row["imu_gz"]))
                if vx > 0.05 and wz > 0.03:
                    v_samples.append(vx)
                    w_samples.append(wz)
            if len(v_samples) < 20 or cmd < 1e-4:
                continue
            radii = [vx / wz for vx, wz in zip(v_samples, w_samples)]
            radius = median(radii)
            actual = math.atan(wheelbase_m / radius)
            ratios.append(actual / cmd)
    return median(ratios), max(0.03, statistics.pstdev(ratios))


def load_cornering_ratio_prior(repo_root: Path, mass_kg: float, lf_m: float, lr_m: float) -> tuple[float, float]:
    wheelbase_m = lf_m + lr_m
    ratios: list[float] = []
    for path_str in sorted(glob.glob(str(repo_root / "f1tenth_parameters/data/cornering_stiffness_*.csv"))):
        rows = list(csv.DictReader(open(path_str)))
        grouped: dict[tuple[float, float], list[dict[str, str]]] = {}
        for row in rows:
            if row["phase"] != "record":
                continue
            key = (float(row["cmd_speed"]), float(row["cmd_steering"]))
            grouped.setdefault(key, []).append(row)
        for (_speed, steer), samples in grouped.items():
            vx = [float(row["odom_vx"]) for row in samples]
            vy_odom = [float(row["odom_vy"]) for row in samples]
            ay = [float(row["imu_ay"]) for row in samples]
            wz = [float(row["imu_gz"]) for row in samples]
            vy_lidar = [float(row["v_lidar_vy"]) for row in samples]
            consistency = [abs(abs(a) - abs(v * w)) for a, v, w in zip(ay, vx, wz)]
            keep = [value < 0.8 for value in consistency]
            if sum(keep) < 20:
                keep = [True] * len(consistency)
            vx_used = [value for value, mask in zip(vx, keep) if mask]
            if not vx_used:
                continue
            vx_avg = sum(vx_used) / len(vx_used)
            wz_avg = sum(abs(value) for value, mask in zip(wz, keep) if mask) / len(vx_used)
            ay_avg = sum(abs(value) for value, mask in zip(ay, keep) if mask) / len(vx_used)
            if vx_avg <= 1.5 or wz_avg <= 0.01:
                continue
            vy_odom_used = [value for value, mask in zip(vy_odom, keep) if mask]
            vy_lidar_used = [value for value, mask in zip(vy_lidar, keep) if mask]
            vy_odom_std = statistics.pstdev(vy_odom_used) if len(vy_odom_used) > 1 else 0.0
            vy_used = (sum(vy_odom_used) / len(vy_odom_used)) if vy_odom_std > 1e-6 else (sum(vy_lidar_used) / len(vy_lidar_used))
            beta = vy_used / vx_avg
            alpha_f = abs(abs(steer) - beta - lf_m * wz_avg / vx_avg)
            alpha_r = abs(lr_m * wz_avg / vx_avg - beta)
            if alpha_f <= 0.005 or alpha_r <= 0.005:
                continue
            fyf = mass_kg * ay_avg * lr_m / wheelbase_m
            fyr = mass_kg * ay_avg * lf_m / wheelbase_m
            c_af = fyf / alpha_f
            c_ar = fyr / alpha_r
            if c_af > 1e-6:
                ratios.append(c_ar / c_af)
    return median(ratios), max(0.08, statistics.pstdev(ratios))


def build_corner_focus_run(run: dict, raceline: list[dict], corner_window: tuple[float, float], pre_margin_m: float = 2.5, post_margin_m: float = 4.0) -> dict:
    lower = corner_window[0] - pre_margin_m
    upper = corner_window[1] + post_margin_m
    start = None
    end = None
    for idx, s_mod in enumerate(run["s_mod"]):
        if lower <= s_mod <= upper:
            if start is None:
                start = idx
            end = idx
        elif start is not None and s_mod > upper:
            break
    if start is None or end is None or end - start < 50:
        raise RuntimeError("Could not isolate corner segment from hardware run")

    rows = run["rows"][start:end + 1]
    progress = run["progress"][start:end + 1]
    s_mod = run["s_mod"][start:end + 1]
    elapsed_s = run["elapsed_s"][start:end + 1]
    first_row = rows[0]
    start_idx = nearest_raceline_index(raceline, float(first_row["pos_x"]), float(first_row["pos_y"]))
    start_wp = raceline[start_idx]
    start_speed = math.hypot(float(first_row["vx"]), float(first_row["vy"]))
    progress0 = progress[0]
    elapsed0 = elapsed_s[0]
    subset_index_path = Path(tempfile.gettempdir()) / f"{Path(run['log_path']).name}.corner.local_raceline_index.csv"
    with subset_index_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["local_raceline_seq", "s_global_m", "pose_ros_time_ns"])
        seen = set()
        for row, prog in zip(rows, progress):
            seq = row.get("local_raceline_seq", "")
            if not seq or seq in seen:
                continue
            seen.add(seq)
            writer.writerow([seq, f"{prog - progress0:.9f}", row.get("pose_ros_time_ns", "")])
    return {
        "log_path": Path(str(run["log_path"]) + "::corner"),
        "meta_path": run["meta_path"],
        "meta": run["meta"],
        "local_raceline_log_path": run["local_raceline_log_path"],
        "local_raceline_index_path": subset_index_path,
        "rows": rows,
        "track_length": run["track_length"],
        "progress": [value - progress0 for value in progress],
        "s_mod": s_mod,
        "elapsed_s": [value - elapsed0 for value in elapsed_s],
        "start_env": {
            "START_INDEX": str(start_idx),
            "START_OFFSET_X": f"{float(first_row['pos_x']) - start_wp['x']:.9f}",
            "START_OFFSET_Y": f"{float(first_row['pos_y']) - start_wp['y']:.9f}",
            "START_HEADING_OFFSET": f"{math.atan2(math.sin(float(first_row['heading']) - start_wp['psi']), math.cos(float(first_row['heading']) - start_wp['psi'])):.9f}",
            "START_SPEED": f"{start_speed:.9f}",
            "START_LAT_SPEED": f"{float(first_row['vy']):.9f}",
            "START_YAW_RATE": f"{float(first_row['omega']):.9f}",
            "START_STEER": f"{float(first_row['actual_steer']):.9f}",
        },
        "start_local_raceline_ns": int(first_row["local_raceline_ros_time_ns"]),
        "laps_complete": 0,
    }


def clip(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def prior_penalty(params: dict, priors: dict) -> float:
    total = 0.0
    mu_sigma = max(priors["mu_sigma"], 0.02)
    total += 0.5 * ((params["SIM_MU"] - priors["mu"]) / mu_sigma) ** 2
    total += 0.6 * ((params["SIM_MU_FRONT"] - priors["mu"]) / mu_sigma) ** 2
    total += 0.3 * ((params["SIM_MU_REAR"] - priors["mu"]) / mu_sigma) ** 2

    steer_sigma = max(priors["steer_gain_sigma"], 0.03)
    total += 1.0 * ((params["SIM_STEER_GAIN"] - priors["steer_gain"]) / steer_sigma) ** 2

    ratio = params["SIM_C_SR"] / max(params["SIM_C_SF"], 1e-6)
    ratio_sigma = max(priors["corner_ratio_sigma"], 0.08)
    total += 0.4 * ((ratio - priors["corner_ratio"]) / ratio_sigma) ** 2

    if params["SIM_C_SF_HIGH_SLIP"] > params["SIM_C_SF"]:
        total += 20.0 * (params["SIM_C_SF_HIGH_SLIP"] - params["SIM_C_SF"]) ** 2
    if params["SIM_C_SR_HIGH_SLIP"] > params["SIM_C_SR"]:
        total += 10.0 * (params["SIM_C_SR_HIGH_SLIP"] - params["SIM_C_SR"]) ** 2
    if params["SIM_STEER_GAIN_HIGH_SLIP"] > params["SIM_STEER_GAIN"]:
        total += 15.0 * (params["SIM_STEER_GAIN_HIGH_SLIP"] - params["SIM_STEER_GAIN"]) ** 2
    if params["SIM_MU_FRONT"] > params["SIM_MU_REAR"] + 0.02:
        total += 8.0 * (params["SIM_MU_FRONT"] - params["SIM_MU_REAR"] - 0.02) ** 2
    return total


def evaluate_behavioral_params(
    binary_path: Path,
    repo_root: Path,
    raceline_path: Path,
    full_run: dict,
    corner_run: dict,
    corner_window: tuple[float, float],
    transient_runs: list,
    params: dict,
    priors: dict,
) -> tuple[float, dict]:
    full_score, full_details = evaluate_params(binary_path, repo_root, raceline_path, [full_run], params, corner_window)
    corner_score, corner_details = evaluate_params(binary_path, repo_root, raceline_path, [corner_run], params, corner_window)
    transient_params = (
        0.5 * (params["SIM_MU_FRONT"] + params["SIM_MU_REAR"]),
        params["SIM_C_SF"],
        params["SIM_C_SR"],
        0.5 * (params["SIM_PACEJKA_C_FRONT"] + params["SIM_PACEJKA_C_REAR"]),
        params["SIM_STEER_GAIN"],
    )
    transient_score, transient_metrics = compute_transient_error(transient_runs, transient_params)
    reg = prior_penalty(params, priors)
    full_metrics = full_details["runs"][0]["metrics"]
    corner_metrics = corner_details["runs"][0]["metrics"]
    lap_corner_cost = (
        90.0 * full_metrics.get("corner_rmse_e_y", full_metrics["rmse_e_y"])
        + 70.0 * full_metrics.get("corner_rmse_e_psi", full_metrics["rmse_e_psi"])
        + 8.0 * full_metrics.get("corner_rmse_omega", full_metrics["rmse_omega"])
        + 4.0 * full_metrics.get("corner_rmse_actual_steer", full_metrics["rmse_actual_steer"])
    )
    corner_behavior_cost = (
        70.0 * corner_metrics.get("corner_rmse_e_y", corner_metrics["rmse_e_y"])
        + 55.0 * corner_metrics.get("corner_rmse_e_psi", corner_metrics["rmse_e_psi"])
        + 4.0 * corner_metrics.get("corner_rmse_omega", corner_metrics["rmse_omega"])
        + 3.0 * corner_metrics.get("corner_rmse_actual_steer", corner_metrics["rmse_actual_steer"])
        + 0.8 * corner_metrics.get("progress_shortfall_m", 0.0)
    )
    total = full_score + lap_corner_cost + 0.7 * corner_behavior_cost + 8.0 * transient_score + reg
    return total, {
        "full_score": full_score,
        "corner_score": corner_score,
        "lap_corner_cost": lap_corner_cost,
        "corner_behavior_cost": corner_behavior_cost,
        "transient_score": transient_score,
        "prior_penalty": reg,
        "full": full_details["runs"][0],
        "corner": corner_details["runs"][0],
        "transient_metrics": transient_metrics,
    }


def propose_candidate(rng: random.Random, best: dict, priors: dict, iteration: int) -> dict:
    candidate = dict(best)
    if iteration == 0:
        candidate["SIM_MU"] = priors["mu"]
        candidate["SIM_MU_FRONT"] = priors["mu"]
        candidate["SIM_MU_REAR"] = priors["mu"]
        candidate["SIM_STEER_GAIN"] = priors["steer_gain"]
        candidate["SIM_C_SR"] = candidate["SIM_C_SF"] * priors["corner_ratio"]
        return candidate

    candidate["SIM_MU"] = clip(rng.gauss(best["SIM_MU"], 0.03), 0.64, 0.84)
    candidate["SIM_MU_FRONT"] = clip(rng.gauss(best["SIM_MU_FRONT"], 0.03), 0.55, 0.84)
    candidate["SIM_MU_REAR"] = clip(rng.gauss(best["SIM_MU_REAR"], 0.03), 0.55, 0.84)
    candidate["SIM_C_SF"] = clip(rng.gauss(best["SIM_C_SF"], 0.30), 3.2, 5.2)
    candidate["SIM_C_SR"] = clip(rng.gauss(best["SIM_C_SR"], 0.35), 3.0, 5.4)

    front_soft = max(0.6, abs(rng.gauss(best["SIM_C_SF"] - best["SIM_C_SF_HIGH_SLIP"], 0.35)))
    rear_soft = max(0.2, abs(rng.gauss(best["SIM_C_SR"] - best["SIM_C_SR_HIGH_SLIP"], 0.30)))
    candidate["SIM_C_SF_HIGH_SLIP"] = clip(candidate["SIM_C_SF"] - front_soft, 1.2, candidate["SIM_C_SF"])
    candidate["SIM_C_SR_HIGH_SLIP"] = clip(candidate["SIM_C_SR"] - rear_soft, 1.8, candidate["SIM_C_SR"])

    candidate["SIM_PACEJKA_C"] = clip(rng.gauss(best["SIM_PACEJKA_C"], 0.12), 1.45, 2.35)
    candidate["SIM_PACEJKA_C_FRONT"] = clip(rng.gauss(best["SIM_PACEJKA_C_FRONT"], 0.12), 1.25, 2.35)
    candidate["SIM_PACEJKA_C_REAR"] = clip(rng.gauss(best["SIM_PACEJKA_C_REAR"], 0.12), 1.25, 2.35)
    candidate["SIM_STEER_GAIN"] = clip(rng.gauss(best["SIM_STEER_GAIN"], 0.03), 0.90, 1.08)

    gain_drop = max(0.0, abs(rng.gauss(best["SIM_STEER_GAIN"] - best["SIM_STEER_GAIN_HIGH_SLIP"], 0.05)))
    candidate["SIM_STEER_GAIN_HIGH_SLIP"] = clip(candidate["SIM_STEER_GAIN"] - gain_drop, 0.70, candidate["SIM_STEER_GAIN"])

    candidate["SIM_SLIP_BLEND_START_FRONT"] = clip(rng.gauss(best["SIM_SLIP_BLEND_START_FRONT"], 0.03), 0.18, 0.42)
    candidate["SIM_SLIP_BLEND_END_FRONT"] = clip(rng.gauss(best["SIM_SLIP_BLEND_END_FRONT"], 0.04), candidate["SIM_SLIP_BLEND_START_FRONT"] + 0.05, 0.62)
    candidate["SIM_SLIP_BLEND_START_REAR"] = clip(rng.gauss(best["SIM_SLIP_BLEND_START_REAR"], 0.03), 0.18, 0.42)
    candidate["SIM_SLIP_BLEND_END_REAR"] = clip(rng.gauss(best["SIM_SLIP_BLEND_END_REAR"], 0.04), candidate["SIM_SLIP_BLEND_START_REAR"] + 0.05, 0.62)
    candidate["SIM_SLIP_BLEND_START"] = candidate["SIM_SLIP_BLEND_START_FRONT"]
    candidate["SIM_SLIP_BLEND_END"] = candidate["SIM_SLIP_BLEND_END_FRONT"]
    candidate["SIM_COMBINED_SLIP_GAIN"] = clip(rng.gauss(best["SIM_COMBINED_SLIP_GAIN"], 0.05), 0.0, 0.35)
    candidate["SIM_FRONT_PEAK_DROP"] = clip(rng.gauss(best["SIM_FRONT_PEAK_DROP"], 0.10), 0.0, 0.70)
    candidate["SIM_FRONT_PEAK_DROP_START"] = clip(rng.gauss(best["SIM_FRONT_PEAK_DROP_START"], 0.03), 0.10, 0.40)
    candidate["SIM_FRONT_PEAK_DROP_END"] = clip(rng.gauss(best["SIM_FRONT_PEAK_DROP_END"], 0.04), candidate["SIM_FRONT_PEAK_DROP_START"] + 0.04, 0.60)
    candidate["SIM_FRONT_PEAK_DROP_POW"] = clip(rng.gauss(best["SIM_FRONT_PEAK_DROP_POW"], 0.20), 1.0, 3.0)
    candidate["SIM_FRONT_COMBINED_GAIN"] = clip(rng.gauss(best["SIM_FRONT_COMBINED_GAIN"], 0.10), 0.0, 0.80)
    candidate["SIM_FRONT_PEAK_FLOOR"] = clip(rng.gauss(best["SIM_FRONT_PEAK_FLOOR"], 0.05), 0.10, 0.50)

    if rng.random() < 0.35:
        candidate["SIM_MU"] = clip(rng.gauss(priors["mu"], 0.02), 0.68, 0.82)
        candidate["SIM_MU_FRONT"] = clip(rng.gauss(priors["mu"], 0.03), 0.55, 0.82)
        candidate["SIM_MU_REAR"] = clip(rng.gauss(priors["mu"], 0.03), 0.55, 0.82)
    if rng.random() < 0.35:
        candidate["SIM_STEER_GAIN"] = clip(rng.gauss(priors["steer_gain"], 0.02), 0.92, 1.04)
    return candidate


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit a behavioral plant that matches the hardware lap and washout behavior.")
    parser.add_argument("--hardware-log", required=True)
    parser.add_argument("--hardware-meta", default="")
    parser.add_argument("--turn-in-csv", action="append", required=True)
    parser.add_argument("--raceline", default="MPC/trajectories/my_track_raceline.csv")
    parser.add_argument("--binary", default="/tmp/test_sim_drive_behavioral")
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--iterations", type=int, default=80)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    raceline_path = (repo_root / args.raceline).resolve()
    raceline = load_raceline(raceline_path)
    corner_window = (raceline[1366]["s"], raceline[1379]["s"])

    hardware_log = Path(args.hardware_log)
    hardware_meta = Path(args.hardware_meta) if args.hardware_meta else Path(str(hardware_log) + ".meta.txt")
    full_run = load_hardware_run(hardware_log, hardware_meta, raceline, window_seconds=None)
    corner_run = build_corner_focus_run(full_run, raceline, corner_window)
    transient_runs = load_turn_runs([Path(path) for path in args.turn_in_csv])

    priors = {
        "mu": 0.70,
        "mu_sigma": 0.03,
        "steer_gain": 1.0,
        "steer_gain_sigma": 0.05,
        "corner_ratio": 1.0,
        "corner_ratio_sigma": 0.15,
    }
    priors["mu"], priors["mu_sigma"] = load_friction_prior(repo_root)
    priors["steer_gain"], priors["steer_gain_sigma"] = load_servo_gain_prior(repo_root, wheelbase_m=0.324)
    priors["corner_ratio"], priors["corner_ratio_sigma"] = load_cornering_ratio_prior(repo_root, mass_kg=3.314, lf_m=0.166, lr_m=0.160)

    binary_path = Path(args.binary)
    if args.build or not binary_path.exists():
        build_sim_binary(repo_root, binary_path)

    best = {
        "SIM_MU": 0.6652002785524997,
        "SIM_MU_FRONT": 0.6745101974282083,
        "SIM_MU_REAR": 0.6565520426481404,
        "SIM_C_SF": 4.78281642069513,
        "SIM_C_SR": 2.73123678240426,
        "SIM_C_SF_HIGH_SLIP": 2.4199490875105907,
        "SIM_C_SR_HIGH_SLIP": 2.73123678240426,
        "SIM_SLIP_BLEND_START": 0.1643527788471148,
        "SIM_SLIP_BLEND_END": 0.5319307735091576,
        "SIM_SLIP_BLEND_START_FRONT": 0.1643527788471148,
        "SIM_SLIP_BLEND_END_FRONT": 0.5319307735091576,
        "SIM_SLIP_BLEND_START_REAR": 0.2502122916247753,
        "SIM_SLIP_BLEND_END_REAR": 0.47793678552502167,
        "SIM_PACEJKA_C": 1.6041121492252324,
        "SIM_PACEJKA_C_FRONT": 1.8031639754063644,
        "SIM_PACEJKA_C_REAR": 1.7681655069132207,
        "SIM_STEER_GAIN": 1.0085301459687404,
        "SIM_STEER_GAIN_HIGH_SLIP": 0.6541720766809247,
        "SIM_COMBINED_SLIP_GAIN": 0.10359393575265835,
        "SIM_FRONT_PEAK_DROP": 0.11804981810838257,
        "SIM_FRONT_PEAK_DROP_START": 0.13813810031946996,
        "SIM_FRONT_PEAK_DROP_END": 0.48938120479012814,
        "SIM_FRONT_PEAK_DROP_POW": 1.0,
        "SIM_FRONT_COMBINED_GAIN": 0.13366870620631957,
        "SIM_FRONT_PEAK_FLOOR": 0.2708096984131235,
        "DRAG_C1": 0.05,
        "DRAG_C2": 0.04,
        "ACCEL_TAU_POS": 0.05,
        "ACCEL_TAU_NEG": 0.12,
        "ACCEL_GAIN_POS": 0.5,
        "ACCEL_GAIN_NEG": 1.0,
    }

    rng = random.Random(args.seed)
    best_score, best_details = evaluate_behavioral_params(
        binary_path, repo_root, raceline_path, full_run, corner_run, corner_window, transient_runs, best, priors
    )
    print(f"priors mu={priors['mu']:.4f}±{priors['mu_sigma']:.4f} steer_gain={priors['steer_gain']:.4f}±{priors['steer_gain_sigma']:.4f} rear/front={priors['corner_ratio']:.4f}±{priors['corner_ratio_sigma']:.4f}")
    print(f"baseline joint_score={best_score:.6f}")

    for iteration in range(args.iterations):
        trial = propose_candidate(rng, best, priors, iteration)
        score, details = evaluate_behavioral_params(
            binary_path, repo_root, raceline_path, full_run, corner_run, corner_window, transient_runs, trial, priors
        )
        print(
            f"iter={iteration:03d} joint={score:.6f} full={details['full_score']:.3f} "
            f"lap_corner={details['lap_corner_cost']:.3f} corner={details['corner_behavior_cost']:.3f} "
            f"transient={details['transient_score']:.3f} reg={details['prior_penalty']:.3f}"
        )
        if score < best_score:
            best = trial
            best_score = score
            best_details = details
            print(f"  -> keep iter={iteration:03d}")

    print("\nBEST")
    print(f"joint_score={best_score:.6f}")
    for key in sorted(best):
        print(f"{key}={best[key]}")
    print(
        f"full_score={best_details['full_score']:.6f} corner_score={best_details['corner_score']:.6f} "
        f"lap_corner_cost={best_details['lap_corner_cost']:.6f} corner_behavior_cost={best_details['corner_behavior_cost']:.6f} "
        f"transient_score={best_details['transient_score']:.6f} prior_penalty={best_details['prior_penalty']:.6f}"
    )
    for label in ("full", "corner"):
        metrics = best_details[label]["metrics"]
        summary = best_details[label]["summary"]
        print(
            f"{label}_summary laps={summary.get('completed_laps', 'n/a')} "
            f"wall={summary.get('wall_collisions', 'n/a')} avg_vx={summary.get('avg_vx', 'n/a')}"
        )
        for metric_key in ("rmse_e_y", "rmse_e_psi", "rmse_vx", "rmse_omega", "corner_rmse_e_y", "corner_rmse_e_psi"):
            if metric_key in metrics:
                print(f"  {label}_{metric_key}={metrics[metric_key]:.6f}")
    print(
        f"transient_rmse x={best_details['transient_metrics']['rmse_x']:.6f} "
        f"y={best_details['transient_metrics']['rmse_y']:.6f} "
        f"yaw={best_details['transient_metrics']['rmse_yaw']:.6f} "
        f"omega={best_details['transient_metrics']['rmse_omega']:.6f}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
