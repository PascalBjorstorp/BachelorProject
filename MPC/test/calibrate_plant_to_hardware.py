#!/usr/bin/env python3
import argparse
import csv
import math
import os
import subprocess
import sys
import tempfile
from bisect import bisect_left
from pathlib import Path


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_meta(meta_path: Path) -> dict:
    meta = {}
    with meta_path.open() as handle:
        for raw in handle:
            line = raw.strip()
            if not line or "=" not in line:
                continue
            key, value = line.split("=", 1)
            meta[key] = value
    return meta


def load_raceline(raceline_path: Path) -> list[dict]:
    points = []
    with raceline_path.open() as handle:
        for raw in handle:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m = map(float, line.split(","))
            points.append({
                "s": s_m,
                "x": x_m,
                "y": y_m,
                "psi": psi_rad,
                "kappa": kappa_radpm,
                "vx": vx_mps,
            })
    if len(points) < 2:
        raise RuntimeError(f"Raceline too short: {raceline_path}")
    return points


def nearest_raceline_index(raceline: list[dict], x: float, y: float) -> int:
    best_idx = 0
    best_dist = None
    for idx, point in enumerate(raceline):
        dx = x - point["x"]
        dy = y - point["y"]
        dist = dx * dx + dy * dy
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx


def nearest_raceline_index_windowed(raceline: list[dict], x: float, y: float, center_idx: int, back: int, forward: int) -> tuple[int, float]:
    count = len(raceline)
    best_idx = center_idx
    best_dist = None
    for offset in range(-back, forward + 1):
        idx = (center_idx + offset) % count
        dx = x - raceline[idx]["x"]
        dy = y - raceline[idx]["y"]
        dist = dx * dx + dy * dy
        if best_dist is None or dist < best_dist:
            best_dist = dist
            best_idx = idx
    return best_idx, math.sqrt(best_dist if best_dist is not None else 0.0)


def project_to_raceline(raceline: list[dict], track_length: float, x: float, y: float, heading: float, hint_idx: int) -> tuple[float, float, float]:
    count = len(raceline)
    idx0 = max(0, min(hint_idx, count - 1))
    idx1 = (idx0 + 1) % count
    ax = raceline[idx0]["x"]
    ay = raceline[idx0]["y"]
    bx = raceline[idx1]["x"]
    by = raceline[idx1]["y"]
    abx = bx - ax
    aby = by - ay
    apx = x - ax
    apy = y - ay
    ab_len2 = abx * abx + aby * aby
    t = 0.0
    if ab_len2 > 1e-12:
        t = (apx * abx + apy * aby) / ab_len2
    t = max(0.0, min(1.0, t))
    path_x = ax + t * abx
    path_y = ay + t * aby
    psi0 = raceline[idx0]["psi"]
    psi1 = raceline[idx1]["psi"]
    dpsi = wrap_angle(psi1 - psi0)
    path_heading = wrap_angle(psi0 + t * dpsi)
    dx = x - path_x
    dy = y - path_y
    ey = -dx * math.sin(path_heading) + dy * math.cos(path_heading)
    epsi = wrap_angle(heading - path_heading)
    s0 = raceline[idx0]["s"]
    s1 = raceline[idx1]["s"]
    if idx0 == count - 1:
        s1 += track_length
    s = s0 + t * (s1 - s0)
    while s < 0.0:
        s += track_length
    while s >= track_length:
        s -= track_length
    return s, ey, epsi


def unwrap_progress(s_values: list[float], track_length: float) -> list[float]:
    if not s_values:
        return []
    unwrapped = [s_values[0]]
    laps = 0
    for idx in range(1, len(s_values)):
        cur = s_values[idx]
        prev = s_values[idx - 1]
        if cur < prev - 0.5 * track_length:
            laps += 1
        elif cur > prev + 0.5 * track_length:
            laps -= 1
        unwrapped.append(cur + laps * track_length)
    offset = unwrapped[0]
    return [value - offset for value in unwrapped]


def rmse(reference: list[float], estimate: list[float]) -> float:
    if not reference:
        return 0.0
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(reference, estimate)) / len(reference))


def interp_linear(xs: list[float], ys: list[float], xq: float) -> float | None:
    if not xs:
        return None
    if xq < xs[0] or xq > xs[-1]:
        return None
    idx = bisect_left(xs, xq)
    if idx == 0:
        return ys[0]
    if idx >= len(xs):
        return ys[-1]
    x0, x1 = xs[idx - 1], xs[idx]
    y0, y1 = ys[idx - 1], ys[idx]
    if abs(x1 - x0) < 1e-12:
        return y0
    alpha = (xq - x0) / (x1 - x0)
    return y0 + alpha * (y1 - y0)


def infer_local_raceline_log_path(log_path: Path, meta: dict) -> Path:
    meta_value = meta.get("local_raceline_log_path", "").strip()
    if meta_value:
        return log_path.parent / Path(meta_value).name
    return Path(str(log_path) + ".local_raceline.csv")


def write_local_raceline_index(log_path: Path, rows: list[dict], s_wrapped: list[float]) -> Path | None:
    if not rows or "local_raceline_seq" not in rows[0]:
        return None

    index_path = Path(str(log_path) + ".local_raceline_index.csv")
    seen = set()
    with index_path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["local_raceline_seq", "s_global_m", "pose_ros_time_ns"])
        for row, s_global in zip(rows, s_wrapped):
            seq_raw = row.get("local_raceline_seq", "")
            if not seq_raw:
                continue
            try:
                seq = int(seq_raw)
            except ValueError:
                continue
            if seq in seen:
                continue
            seen.add(seq)
            writer.writerow([seq, f"{s_global:.9f}", row.get("pose_ros_time_ns", "")])
    return index_path


def load_hardware_run(log_path: Path, meta_path: Path, raceline: list[dict], window_seconds: float | None = None) -> dict:
    rows = []
    with log_path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
    if not rows:
        raise RuntimeError(f"Empty hardware log: {log_path}")
    meta = load_meta(meta_path)

    track_length = raceline[-1]["s"]
    hints = []
    s_wrapped = []
    for row in rows:
        x = float(row["pos_x"])
        y = float(row["pos_y"])
        if hints:
            best_idx, best_dist = nearest_raceline_index_windowed(raceline, x, y, hints[-1], 80, 260)
            if best_dist > 1.0:
                best_idx = nearest_raceline_index(raceline, x, y)
        else:
            best_idx = nearest_raceline_index(raceline, x, y)
        hints.append(best_idx)
        s_val, _, _ = project_to_raceline(raceline, track_length, x, y, float(row["heading"]), best_idx)
        s_wrapped.append(s_val)

    progress = unwrap_progress(s_wrapped, track_length)

    start_sample_idx = 0
    stable_window = 200
    for idx in range(0, max(0, len(rows) - stable_window)):
        stable = True
        for j in range(idx, idx + stable_window):
            ey = abs(float(rows[j]["e_y"]))
            epsi = abs(float(rows[j]["e_psi"]))
            vx = float(rows[j]["vx"])
            if ey >= 0.35 or epsi >= 0.20 or vx <= 2.5:
                stable = False
                break
        if stable:
            start_sample_idx = idx
            break

    if start_sample_idx == 0 and progress[-1] >= track_length:
        for idx, value in enumerate(progress):
            if value >= track_length:
                start_sample_idx = idx
                break

    progress_offset = progress[start_sample_idx]
    rows = rows[start_sample_idx:]
    hints = hints[start_sample_idx:]
    s_wrapped = s_wrapped[start_sample_idx:]
    progress = [value - progress_offset for value in progress[start_sample_idx:]]
    if window_seconds is not None and window_seconds > 0.0:
        limited_count = 0
        base_pose_ns = float(rows[0]["pose_ros_time_ns"])
        for row in rows:
            elapsed = (float(row["pose_ros_time_ns"]) - base_pose_ns) / 1e9
            if elapsed > window_seconds:
                break
            limited_count += 1
        limited_count = max(2, limited_count)
        rows = rows[:limited_count]
        hints = hints[:limited_count]
        s_wrapped = s_wrapped[:limited_count]
        progress = progress[:limited_count]
    first_row = rows[0]
    start_idx = hints[0]
    start_wp = raceline[start_idx]
    start_speed = math.hypot(float(first_row["vx"]), float(first_row["vy"]))
    start_state = {
        "START_INDEX": str(start_idx),
        "START_OFFSET_X": f"{float(first_row['pos_x']) - start_wp['x']:.9f}",
        "START_OFFSET_Y": f"{float(first_row['pos_y']) - start_wp['y']:.9f}",
        "START_HEADING_OFFSET": f"{wrap_angle(float(first_row['heading']) - start_wp['psi']):.9f}",
        "START_SPEED": f"{start_speed:.9f}",
        "START_LAT_SPEED": f"{float(first_row['vy']):.9f}",
        "START_YAW_RATE": f"{float(first_row['omega']):.9f}",
        "START_STEER": f"{float(first_row['actual_steer']):.9f}",
    }

    elapsed_s = [(float(row["pose_ros_time_ns"]) - float(rows[0]["pose_ros_time_ns"])) / 1e9 for row in rows]
    laps_complete = int(progress[-1] / track_length)
    return {
        "log_path": log_path,
        "meta_path": meta_path,
        "meta": meta,
        "local_raceline_log_path": infer_local_raceline_log_path(log_path, meta),
        "local_raceline_index_path": write_local_raceline_index(log_path, rows, s_wrapped),
        "rows": rows,
        "track_length": track_length,
        "progress": progress,
        "s_mod": s_wrapped,
        "elapsed_s": elapsed_s,
        "start_env": start_state,
        "start_local_raceline_ns": int(first_row["local_raceline_ros_time_ns"]),
        "laps_complete": laps_complete,
    }


def parse_sim_summary(stdout: str) -> dict:
    for raw in stdout.splitlines():
        line = raw.strip()
        if not line.startswith("CSV,"):
            continue
        parts = line.split(",")
        return {
            "passed": int(parts[1]),
            "failed": int(parts[2]),
            "max_lat_err": float(parts[3]),
            "avg_lat_err": float(parts[4]),
            "avg_hdg_err": float(parts[6]),
            "wall_collisions": int(parts[10]),
            "avg_vx": float(parts[15]),
            "progress_m": float(parts[16]),
            "avg_progress_mps": float(parts[17]),
            "completed_laps": int(parts[18]),
            "avg_lap_time": float(parts[19]),
        }
    return {}


def load_sim_trace(trace_path: Path, track_length: float) -> dict:
    rows = []
    with trace_path.open() as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            rows.append(row)
    if not rows:
        raise RuntimeError(f"Empty sim trace: {trace_path}")
    s_wrapped = [float(row["true_path_s"]) for row in rows]
    progress = unwrap_progress(s_wrapped, track_length)
    return {
        "rows": rows,
        "progress": progress,
        "s_mod": s_wrapped,
    }


def compute_run_score(hw: dict, sim: dict, corner_window: tuple[float, float]) -> tuple[float, dict]:
    sim_progress = sim["progress"]
    hw_progress = hw["progress"]
    overlap_end = min(hw_progress[-1], sim_progress[-1])
    if overlap_end <= 1.0:
        return 1e9, {"reason": "insufficient_overlap"}

    fields = ["e_y", "e_psi", "vx", "omega", "actual_steer", "cmd_steer", "cmd_accel"]
    weights = {
        "e_y": 4.0,
        "e_psi": 3.0,
        "vx": 1.5,
        "omega": 2.0,
        "actual_steer": 2.0,
        "cmd_steer": 1.0,
        "cmd_accel": 0.8,
    }
    aligned = {field: ([], []) for field in fields}
    corner = {field: ([], []) for field in fields}
    sim_series = {field: [float(row[field]) for row in sim["rows"]] for field in fields}

    for idx, progress in enumerate(hw_progress):
        if progress > overlap_end:
            break
        hw_row = hw["rows"][idx]
        s_mod = hw["s_mod"][idx]
        for field in fields:
            sim_val = interp_linear(sim_progress, sim_series[field], progress)
            if sim_val is None:
                continue
            aligned[field][0].append(float(hw_row[field]))
            aligned[field][1].append(sim_val)
            if corner_window[0] <= s_mod <= corner_window[1]:
                corner[field][0].append(float(hw_row[field]))
                corner[field][1].append(sim_val)

    total = 0.0
    metrics = {}
    for field in fields:
        score = rmse(*aligned[field])
        metrics[f"rmse_{field}"] = score
        total += weights[field] * score
        if corner[field][0]:
            c_score = rmse(*corner[field])
            metrics[f"corner_rmse_{field}"] = c_score
            total += 2.0 * weights[field] * c_score

    progress_shortfall = max(0.0, hw_progress[-1] - sim_progress[-1])
    terminal_progress_error = abs(hw_progress[-1] - sim_progress[-1])
    total += 10.0 * (progress_shortfall / hw["track_length"])
    total += 12.0 * (terminal_progress_error / hw["track_length"])
    metrics["progress_shortfall_m"] = progress_shortfall
    metrics["terminal_progress_error_m"] = terminal_progress_error
    metrics["overlap_end_m"] = overlap_end
    return total, metrics


def build_sim_binary(repo_root: Path, output_path: Path) -> None:
    cmd = [
        "gcc", "-D_GNU_SOURCE", "-O3", "-std=c99", "-Wall", "-ffast-math",
        "-Wno-unused-variable", "-Wno-unused-but-set-variable",
        "-IMPC/include",
        "MPC/test/test_sim_drive.c",
        "MPC/src/mpc.c",
        "MPC/src/riccati_solver.c",
        "MPC/src/vehicle_model.c",
        "MPC/src/util_math.c",
        "-o", str(output_path),
        "-lm",
    ]
    subprocess.run(cmd, cwd=repo_root, check=True)


def meta_to_sim_env(meta: dict) -> dict:
    mapping = {
        "weight_lat": ("Q_LAT", float),
        "weight_heading": ("Q_HDG", float),
        "weight_velocity": ("Q_VEL", float),
        "weight_lat_vel": ("Q_LAT_VEL", float),
        "weight_yaw_rate": ("Q_YAW", float),
        "weight_steer_effort": ("R_STEER", float),
        "weight_accel_effort": ("R_ACCEL", float),
        "weight_steer_rate": ("W_JERK", float),
        "weight_accel_rate": ("W_ACCEL_RATE", float),
        "weight_delta_actual": ("MPC_W_DELTA_ACTUAL", float),
        "prediction_horizon": ("HORIZON", int),
        "prediction_dt_s": ("PRED_DT", float),
        "solver_max_iter": ("MAX_ITER", int),
        "solver_tol": ("TOL", float),
        "vehicle_mass_kg": ("SIM_MASS", float),
        "vehicle_iz_kgm2": ("SIM_IZ", float),
        "vehicle_lf_m": ("SIM_LF", float),
        "vehicle_lr_m": ("SIM_LR", float),
        "vehicle_hcg_m": ("SIM_H_CG", float),
        "vehicle_steer_max_rad": ("SIM_STEER_ANGLE_MAX", float),
        "vehicle_min_speed_mps": ("SIM_V_MIN", float),
        "vehicle_max_speed_mps": ("SIM_V_MAX", float),
        "steering_rate_limit": ("SIM_STEER_RATE_MAX", float),
    }
    env = {}
    for meta_key, (env_key, caster) in mapping.items():
        if meta_key in meta:
            env[env_key] = str(caster(meta[meta_key]))
    return env


def evaluate_params(binary_path: Path, repo_root: Path, raceline_path: Path, runs: list[dict], params: dict, corner_window: tuple[float, float]) -> tuple[float, dict]:
    total_score = 0.0
    details = {"runs": []}
    for run in runs:
        with tempfile.TemporaryDirectory(prefix="sim_cal_") as tmpdir:
            trace_path = Path(tmpdir) / "sim_trace.csv"
            if not Path(run["local_raceline_log_path"]).exists():
                return 1e9, {"error": f"missing local raceline log: {run['local_raceline_log_path']}"}
            env = os.environ.copy()
            env.update(meta_to_sim_env(run["meta"]))
            env.update(run["start_env"])
            env.update({
                "SIM_DT": "0.005",
                "MPC_DT": run["meta"].get("control_dt_s", "0.005"),
                "SIM_DURATION": f"{run['elapsed_s'][-1]:.6f}",
                "LOCAL_RACELINE_SIM": "1",
                "BODY_SAFETY_MARGIN": "0.0",
                "RACELINE_PATH": str(raceline_path.resolve()),
                "SIM_TRACE_LOG": str(trace_path),
                "MPC_TUNING_CSV": "1",
                "REPLAY_LOCAL_RACELINE_LOG": str(run["local_raceline_log_path"]),
            })
            if run.get("local_raceline_index_path"):
                env["REPLAY_LOCAL_RACELINE_MODE"] = "progress"
                env["REPLAY_LOCAL_RACELINE_INDEX"] = str(run["local_raceline_index_path"])
            else:
                env["REPLAY_LOCAL_RACELINE_MODE"] = "time"
                env["REPLAY_LOCAL_RACELINE_START_NS"] = str(run["start_local_raceline_ns"])
            for key, value in params.items():
                env[key] = str(value)

            proc = subprocess.run(
                [str(binary_path)],
                cwd=repo_root,
                env=env,
                capture_output=True,
                text=True,
            )
            summary = parse_sim_summary(proc.stdout)
            if not trace_path.exists():
                return 1e9, {"error": proc.stderr[-500:] if proc.stderr else "sim_failed"}
            sim = load_sim_trace(trace_path, run["track_length"])
            score, metrics = compute_run_score(run, sim, corner_window)
            if proc.returncode != 0:
                score += 50.0
            if summary.get("wall_collisions", 0) > 0:
                score += 100.0 + 20.0 * summary["wall_collisions"]
            if summary.get("completed_laps", 0) < run["laps_complete"]:
                score += 30.0 * (run["laps_complete"] - summary.get("completed_laps", 0))
            total_score += score
            details["runs"].append({
                "log": str(run["log_path"]),
                "score": score,
                "summary": summary,
                "metrics": metrics,
            })
    return total_score / max(len(runs), 1), details


def main() -> int:
    parser = argparse.ArgumentParser(description="Calibrate sim plant parameters against hardware MPC logs.")
    parser.add_argument("--hardware-log", action="append", required=True, help="Hardware MPC CSV log. Repeat for multiple runs.")
    parser.add_argument("--hardware-meta", action="append", default=[], help="Optional metadata file matching each hardware log.")
    parser.add_argument("--raceline", default="MPC/trajectories/my_track_raceline.csv")
    parser.add_argument("--binary", default="/tmp/test_sim_drive_cal")
    parser.add_argument("--build", action="store_true")
    parser.add_argument("--passes", type=int, default=2)
    parser.add_argument("--window-sec", type=float, default=3.0, help="Length of the replay/calibration window after the stable start sample. Use <=0 for full remaining run.")
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[2]
    raceline_path = (repo_root / args.raceline).resolve()
    raceline = load_raceline(raceline_path)
    corner_window = (raceline[1366]["s"], raceline[1379]["s"])

    meta_paths = list(args.hardware_meta)
    while len(meta_paths) < len(args.hardware_log):
        inferred = Path(args.hardware_log[len(meta_paths)] + ".meta.txt")
        meta_paths.append(str(inferred))

    runs = []
    for log_str, meta_str in zip(args.hardware_log, meta_paths):
        window_seconds = args.window_sec if args.window_sec > 0.0 else None
        runs.append(load_hardware_run(Path(log_str), Path(meta_str), raceline, window_seconds=window_seconds))

    binary_path = Path(args.binary)
    if args.build or not binary_path.exists():
        build_sim_binary(repo_root, binary_path)

    search_space = {
        "SIM_MU": [0.68, 0.70, 0.72, 0.75],
        "SIM_MU_FRONT": [0.60, 0.64, 0.68, 0.72],
        "SIM_MU_REAR": [0.64, 0.68, 0.72, 0.76],
        "SIM_C_SF": [3.5, 4.0, 4.5, 5.0],
        "SIM_C_SR": [3.0, 3.5, 4.0, 4.5],
        "SIM_C_SF_HIGH_SLIP": [2.5, 3.0, 3.5, 4.0],
        "SIM_C_SR_HIGH_SLIP": [3.0, 3.5, 4.0],
        "SIM_SLIP_BLEND_START": [0.30, 0.35, 0.40],
        "SIM_SLIP_BLEND_END": [0.45, 0.50, 0.55],
        "SIM_SLIP_BLEND_START_FRONT": [0.28, 0.35, 0.42],
        "SIM_SLIP_BLEND_END_FRONT": [0.42, 0.50, 0.58],
        "SIM_SLIP_BLEND_START_REAR": [0.20, 0.28, 0.35],
        "SIM_SLIP_BLEND_END_REAR": [0.32, 0.40, 0.48],
        "SIM_PACEJKA_C": [1.30, 1.60, 1.90, 2.20],
        "SIM_PACEJKA_C_FRONT": [1.30, 1.60, 1.90, 2.20],
        "SIM_PACEJKA_C_REAR": [1.30, 1.60, 1.90, 2.20],
        "SIM_STEER_GAIN": [1.00, 1.05, 1.10, 1.15],
        "SIM_STEER_GAIN_HIGH_SLIP": [0.85, 0.92, 1.00, 1.08],
        "SIM_COMBINED_SLIP_GAIN": [0.0, 0.10, 0.20, 0.30],
        "SIM_FRONT_PEAK_DROP": [0.0, 0.15, 0.30, 0.45],
        "SIM_FRONT_PEAK_DROP_START": [0.18, 0.24, 0.30],
        "SIM_FRONT_PEAK_DROP_END": [0.30, 0.40, 0.50],
        "SIM_FRONT_PEAK_DROP_POW": [1.0, 1.5, 2.0],
        "SIM_FRONT_COMBINED_GAIN": [0.0, 0.20, 0.40, 0.60],
        "DRAG_C1": [0.01, 0.03, 0.05, 0.07],
        "DRAG_C2": [0.01, 0.02, 0.03, 0.04],
        "ACCEL_TAU_POS": [0.02, 0.05, 0.08, 0.12],
        "ACCEL_TAU_NEG": [0.05, 0.08, 0.12],
        "ACCEL_GAIN_POS": [0.40, 0.50, 0.60, 0.70, 0.85, 1.00],
        "ACCEL_GAIN_NEG": [0.90, 1.00, 1.05],
    }
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

    best_score, best_details = evaluate_params(binary_path, repo_root, raceline_path, runs, best, corner_window)
    print(f"baseline_score={best_score:.6f}")
    print(f"baseline_params={best}")

    for sweep_pass in range(args.passes):
        improved = False
        print(f"\npass={sweep_pass + 1}")
        for key, candidates in search_space.items():
            local_best_score = best_score
            local_best_value = best[key]
            for candidate in candidates:
                trial = dict(best)
                trial[key] = candidate
                score, _ = evaluate_params(binary_path, repo_root, raceline_path, runs, trial, corner_window)
                print(f"  {key}={candidate:.6f} score={score:.6f}")
                if score < local_best_score - 1e-9:
                    local_best_score = score
                    local_best_value = candidate
            if local_best_value != best[key]:
                best[key] = local_best_value
                best_score = local_best_score
                best_score, best_details = evaluate_params(binary_path, repo_root, raceline_path, runs, best, corner_window)
                improved = True
                print(f"  -> keep {key}={best[key]:.6f} best_score={best_score:.6f}")
        if not improved:
            break

    print("\nBEST")
    print(f"score={best_score:.6f}")
    for key in sorted(best):
        print(f"{key}={best[key]}")
    for run_detail in best_details.get("runs", []):
        print(
            f"run={run_detail['log']} score={run_detail['score']:.6f} "
            f"laps={run_detail['summary'].get('completed_laps', 'n/a')} "
            f"avg_vx={run_detail['summary'].get('avg_vx', 'n/a')}"
        )
        for metric_key in [
            "rmse_e_y", "rmse_e_psi", "rmse_vx", "rmse_omega",
            "corner_rmse_e_y", "corner_rmse_e_psi", "corner_rmse_vx",
        ]:
            if metric_key in run_detail["metrics"]:
                print(f"  {metric_key}={run_detail['metrics'][metric_key]:.6f}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
