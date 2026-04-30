#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
import random
from dataclasses import dataclass
from pathlib import Path


MASS_KG = 3.314
IZ_KGM2 = 0.035
LF_M = 0.166
LR_M = 0.160
H_CG_M = 0.0703
STEER_RATE_MAX_RADPS = 2.8492


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def pacejka_force(mu: float, cornering_stiffness: float, shape: float, normal_load: float, slip_angle: float) -> float:
    return mu * normal_load * math.sin(shape * math.atan((cornering_stiffness / shape) * slip_angle))


@dataclass
class RunData:
    source: str
    run_id: int
    speed_mps: float
    direction: str
    dt_s: list[float]
    cmd_steer_rad: list[float]
    vx_mps: list[float]
    omega_radps: list[float]
    x_local_m: list[float]
    y_local_m: list[float]
    yaw_local_rad: list[float]


def load_turn_runs(csv_paths: list[Path]) -> list[RunData]:
    runs: list[RunData] = []
    for csv_path in csv_paths:
        with csv_path.open() as handle:
            reader = csv.DictReader(handle)
            rows = list(reader)

        grouped: dict[int, list[dict[str, str]]] = {}
        for row in rows:
            if row.get("phase") != "turn":
                continue
            run_id = int(row["run_id"])
            if run_id <= 0:
                continue
            grouped.setdefault(run_id, []).append(row)

        for run_id, run_rows in grouped.items():
            run_rows.sort(key=lambda row: float(row["timestamp_s"]))
            if len(run_rows) < 200:
                continue

            x0 = float(run_rows[0]["odom_x"])
            y0 = float(run_rows[0]["odom_y"])
            yaw0 = float(run_rows[0]["odom_yaw"])
            cos0 = math.cos(-yaw0)
            sin0 = math.sin(-yaw0)

            dt_s: list[float] = []
            cmd_steer_rad: list[float] = []
            vx_mps: list[float] = []
            omega_radps: list[float] = []
            x_local_m: list[float] = []
            y_local_m: list[float] = []
            yaw_local_rad: list[float] = []
            prev_t = float(run_rows[0]["timestamp_s"])
            for idx, row in enumerate(run_rows):
                t = float(row["timestamp_s"])
                dt_s.append(0.0 if idx == 0 else t - prev_t)
                prev_t = t
                cmd_steer_rad.append(float(row["cmd_steering"]))
                vx_mps.append(float(row["odom_vx_corr"]))
                omega_radps.append(float(row["imu_gz_corr"]))

                dx = float(row["odom_x"]) - x0
                dy = float(row["odom_y"]) - y0
                x_local_m.append(cos0 * dx - sin0 * dy)
                y_local_m.append(sin0 * dx + cos0 * dy)
                yaw_local_rad.append(wrap_angle(float(row["odom_yaw"]) - yaw0))

            direction = "L" if float(run_rows[0]["direction_sign"]) > 0.0 else "R"
            speed_mps = float(run_rows[0]["speed_setpoint"])
            runs.append(
                RunData(
                    source=csv_path.name,
                    run_id=run_id,
                    speed_mps=speed_mps,
                    direction=direction,
                    dt_s=dt_s,
                    cmd_steer_rad=cmd_steer_rad,
                    vx_mps=vx_mps,
                    omega_radps=omega_radps,
                    x_local_m=x_local_m,
                    y_local_m=y_local_m,
                    yaw_local_rad=yaw_local_rad,
                )
            )
    return runs


def simulate_run(run: RunData, params: tuple[float, float, float, float, float]) -> tuple[list[float], list[float], list[float], list[float]]:
    mu, c_sf, c_sr, pacejka_c, steer_gain = params
    wheelbase_m = LF_M + LR_M
    x_m = 0.0
    y_m = 0.0
    yaw_rad = 0.0
    vy_mps = 0.0
    omega_radps = run.omega_radps[0]
    steer_state_rad = run.cmd_steer_rad[0]

    pred_x = [x_m]
    pred_y = [y_m]
    pred_yaw = [yaw_rad]
    pred_omega = [omega_radps]

    for idx in range(1, len(run.dt_s)):
        dt = max(1e-3, run.dt_s[idx])
        target_steer = steer_gain * run.cmd_steer_rad[idx]
        steer_error = target_steer - steer_state_rad
        steer_state_rad += max(-STEER_RATE_MAX_RADPS * dt, min(STEER_RATE_MAX_RADPS * dt, steer_error))

        vx_mps = max(0.5, run.vx_mps[idx])
        ax_mps2 = (run.vx_mps[idx] - run.vx_mps[idx - 1]) / dt
        fzf_n = MASS_KG * (9.81 * LR_M - ax_mps2 * H_CG_M) / wheelbase_m
        fzr_n = MASS_KG * (9.81 * LF_M + ax_mps2 * H_CG_M) / wheelbase_m

        alpha_f = steer_state_rad - math.atan2(vy_mps + LF_M * omega_radps, vx_mps)
        alpha_r = -math.atan2(vy_mps - LR_M * omega_radps, vx_mps)
        fyf_n = pacejka_force(mu, c_sf, pacejka_c, fzf_n, alpha_f)
        fyr_n = pacejka_force(mu, c_sr, pacejka_c, fzr_n, alpha_r)

        dvy_mps2 = (fyf_n * math.cos(steer_state_rad) + fyr_n - MASS_KG * vx_mps * omega_radps) / MASS_KG
        domega_radps2 = (LF_M * fyf_n * math.cos(steer_state_rad) - LR_M * fyr_n) / IZ_KGM2

        vy_mps += dt * dvy_mps2
        omega_radps += dt * domega_radps2
        yaw_rad = wrap_angle(yaw_rad + dt * omega_radps)
        x_m += dt * (vx_mps * math.cos(yaw_rad) - vy_mps * math.sin(yaw_rad))
        y_m += dt * (vx_mps * math.sin(yaw_rad) + vy_mps * math.cos(yaw_rad))

        pred_x.append(x_m)
        pred_y.append(y_m)
        pred_yaw.append(yaw_rad)
        pred_omega.append(omega_radps)

    return pred_x, pred_y, pred_yaw, pred_omega


def compute_error(runs: list[RunData], params: tuple[float, float, float, float, float]) -> tuple[float, dict[str, float]]:
    err_x_sq = 0.0
    err_y_sq = 0.0
    err_yaw_sq = 0.0
    err_omega_sq = 0.0
    count = 0
    for run in runs:
        pred_x, pred_y, pred_yaw, pred_omega = simulate_run(run, params)
        for idx in range(len(pred_x)):
            dx = pred_x[idx] - run.x_local_m[idx]
            dy = pred_y[idx] - run.y_local_m[idx]
            dyaw = wrap_angle(pred_yaw[idx] - run.yaw_local_rad[idx])
            domega = pred_omega[idx] - run.omega_radps[idx]
            err_x_sq += dx * dx
            err_y_sq += dy * dy
            err_yaw_sq += dyaw * dyaw
            err_omega_sq += domega * domega
            count += 1

    rmse_x = math.sqrt(err_x_sq / count)
    rmse_y = math.sqrt(err_y_sq / count)
    rmse_yaw = math.sqrt(err_yaw_sq / count)
    rmse_omega = math.sqrt(err_omega_sq / count)
    weighted = math.sqrt(
        (1.5 * err_x_sq + 2.0 * err_y_sq + 2.5 * err_yaw_sq + 4.0 * err_omega_sq) / count
    )
    return weighted, {
        "rmse_x": rmse_x,
        "rmse_y": rmse_y,
        "rmse_yaw": rmse_yaw,
        "rmse_omega": rmse_omega,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", nargs="+", required=True, help="One or more turn_in_transient CSV files.")
    parser.add_argument("--iterations", type=int, default=800, help="Random-search samples.")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    runs = load_turn_runs([Path(path) for path in args.csv])
    if not runs:
        raise SystemExit("No usable turn-phase runs found.")

    rng = random.Random(args.seed)
    best_params = (0.7342, 1.8165, 2.7135, 2.1800, 1.1191)
    best_score, best_metrics = compute_error(runs, best_params)
    print(f"seed fit score={best_score:.6f} params={best_params} metrics={best_metrics}")

    for candidate in (
        (0.66, 5.0, 4.0, 1.9, 1.0),
        (0.745, 4.297, 3.473, 1.9, 1.0),
        best_params,
    ):
        score, metrics = compute_error(runs, candidate)
        if score < best_score:
            best_score, best_params, best_metrics = score, candidate, metrics
        print(f"candidate score={score:.6f} params={candidate} metrics={metrics}")

    explore_count = max(80, args.iterations // 4)
    for iteration in range(args.iterations):
        if iteration < explore_count:
            candidate = (
                rng.uniform(0.45, 0.95),
                rng.uniform(1.0, 4.0),
                rng.uniform(1.2, 4.5),
                rng.uniform(1.4, 2.5),
                rng.uniform(0.9, 1.25),
            )
        else:
            candidate = (
                min(1.10, max(0.45, rng.gauss(best_params[0], 0.05))),
                min(4.5, max(0.8, rng.gauss(best_params[1], 0.20))),
                min(5.0, max(1.0, rng.gauss(best_params[2], 0.25))),
                min(2.8, max(1.2, rng.gauss(best_params[3], 0.12))),
                min(1.35, max(0.85, rng.gauss(best_params[4], 0.04))),
            )

        score, metrics = compute_error(runs, candidate)
        if score < best_score:
            best_score = score
            best_params = candidate
            best_metrics = metrics
            print(f"best iter={iteration} score={score:.6f} params={candidate} metrics={metrics}")

    print("\nRecommended overrides:")
    print(f"  SIM_MU={best_params[0]:.6f}")
    print(f"  SIM_C_SF={best_params[1]:.6f}")
    print(f"  SIM_C_SR={best_params[2]:.6f}")
    print(f"  SIM_PACEJKA_C={best_params[3]:.6f}")
    print(f"  SIM_STEER_GAIN={best_params[4]:.6f}")
    print("Fit RMSE:")
    print(f"  x={best_metrics['rmse_x']:.4f} m")
    print(f"  y={best_metrics['rmse_y']:.4f} m")
    print(f"  yaw={best_metrics['rmse_yaw']:.4f} rad")
    print(f"  omega={best_metrics['rmse_omega']:.4f} rad/s")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
