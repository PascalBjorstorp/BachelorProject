"""Small per-stage plots and human-readable result pages."""
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


def _read(path: Path) -> Any:
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None


def _read_yaml(path: Path) -> Any:
    if not path.exists():
        return None
    try:
        import yaml
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None


def _frozen_config(session: Path) -> dict[str, Any]:
    for name in ("calibration_config_snapshot.yaml", "steering_calibration_config_snapshot.yaml"):
        value = _read_yaml(session / name)
        if isinstance(value, dict):
            return value
    return {}


def _scalar_lines(value: Any, prefix: str = "", limit: int = 32) -> list[str]:
    lines: list[str] = []
    if isinstance(value, dict):
        for key, child in value.items():
            if key in {"records", "candidate_records", "confirmation_records", "candidate_estimates", "epochs", "failures"}:
                continue
            lines.extend(_scalar_lines(child, f"{prefix}.{key}" if prefix else str(key), limit - len(lines)))
            if len(lines) >= limit:
                break
    elif isinstance(value, (str, bool, int, float)):
        if isinstance(value, float) and (math.isnan(value) or math.isinf(value)):
            return lines
        lines.append(f"{prefix} = {value}")
    return lines[:limit]


def _runtime_result(session: Path, stage: Any) -> dict[str, Any]:
    value = _read(session / stage.directory / "runtime_result.json")
    return value if isinstance(value, dict) else {}


def _save_figure(figure: Any, session: Path, plots: Path, key: str) -> str:
    """Save and close one stage figure using the GUI/LaTeX naming contract."""
    import matplotlib.pyplot as plt
    from .statistics import statistics_footer

    footer = statistics_footer(session, key)
    if footer:
        figure.text(0.5, 0.008, footer, ha="center", va="bottom", fontsize=6.5, color="#334155")
        figure.tight_layout(rect=(0.0, 0.055, 1.0, 1.0))
    else:
        figure.tight_layout()
    output = plots / f"{key}.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return str(output.relative_to(session))


def _finite_limits(*values: Any) -> tuple[float, float] | None:
    """Return finite plotting bounds with enough padding for an identity line."""
    import numpy as np

    arrays = [np.asarray(value, dtype=float).reshape(-1) for value in values if value is not None]
    finite = np.concatenate(arrays) if arrays else np.empty(0, dtype=float)
    finite = finite[np.isfinite(finite)]
    if not len(finite):
        return None
    low, high = float(finite.min()), float(finite.max())
    span = high - low
    padding = 0.05 * span if span > 1.0e-12 else max(0.05 * abs(high), 0.01)
    return low - padding, high + padding


def _plot_stage(session: Path, stage: Any, plots: Path) -> str | None:
    """Create a compact plot even for stages without a fitted parameter."""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
        import pandas as pd
    except ImportError:
        return None
    plots.mkdir(parents=True, exist_ok=True)
    key = stage.key
    if key == "steering_command_audit":
        samples = _runtime_result(session, stage).get("samples", [])
        if not isinstance(samples, list) or not samples:
            return None
        frame = pd.DataFrame(samples)
        required = {"label", "raw_servo_target"}
        if not required.issubset(frame.columns):
            return None
        outputs = (
            ("servo_selected_mean", "selector", "tab:blue", "o"),
            ("servo_bus_mean", "servo bus", "tab:orange", "s"),
            ("servo_echo_mean", "driver echo", "tab:green", "^"),
        )
        x = np.arange(len(frame))
        figure, axes = plt.subplots(2, 1, figsize=(7.4, 5.5), sharex=True)
        target = frame.raw_servo_target.to_numpy(float)
        axes[0].plot(x, target, color="black", marker="x", linestyle="--", label="requested raw servo")
        plotted = False
        for column, label, color, marker in outputs:
            if column not in frame:
                continue
            values = frame[column].to_numpy(float)
            axes[0].plot(x, values, color=color, marker=marker, label=label)
            axes[1].plot(x, values - target, color=color, marker=marker, label=label)
            plotted = True
        if not plotted:
            plt.close(figure)
            return None
        axes[0].set_ylabel("raw servo value")
        axes[0].set_title("Steering command-chain delivery")
        axes[0].legend(fontsize=7, ncol=2)
        axes[1].axhline(0.0, color="black", linewidth=0.7)
        axes[1].set(ylabel="delivered − requested", xlabel="stationary audit command")
        axes[1].set_xticks(x, frame.label.astype(str), rotation=25, ha="right")
        return _save_figure(figure, session, plots, key)

    if key == "steering_endstops":
        result = _runtime_result(session, stage)
        needed = {"centre_servo_raw", "raw_low_safe", "raw_high_safe"}
        if not needed.issubset(result):
            return None
        low_safe = float(result["raw_low_safe"])
        high_safe = float(result["raw_high_safe"])
        centre = float(result["centre_servo_raw"])
        low_free = float(result.get("raw_low_last_free", low_safe))
        high_free = float(result.get("raw_high_last_free", high_safe))
        figure, axes = plt.subplots(2, 1, figsize=(7.2, 4.8))
        axes[0].hlines(0.0, low_free, high_free, color="tab:red", linewidth=10, label="human last-free range")
        axes[0].hlines(0.0, low_safe, high_safe, color="tab:green", linewidth=4, label="stored safe range")
        axes[0].scatter([centre], [0.0], marker="|", s=260, color="black", label=f"centre {centre:.4f}")
        axes[0].set(yticks=[], xlabel="raw servo value", title="Mechanical steering envelope and safety margin")
        axes[0].legend(fontsize=8, ncol=2)
        angle_fields = ("observed_low_wheel_angle_deg", "observed_high_wheel_angle_deg")
        if all(field in result for field in angle_fields):
            angles = [float(result[field]) for field in angle_fields]
            axes[1].bar(["low endpoint", "high endpoint"], angles, color=["tab:blue", "tab:orange"])
            axes[1].axhline(0.0, color="black", linewidth=0.7)
            axes[1].set_ylabel("human-observed wheel angle [deg]")
        else:
            axes[1].axis("off")
            margin = result.get("safety_margin_servo", "unknown")
            axes[1].text(0.02, 0.65, f"Safe low: {low_safe:.4f}\nCentre: {centre:.4f}\nSafe high: {high_safe:.4f}\nServo safety margin: {margin}",
                         family="monospace", va="top")
        return _save_figure(figure, session, plots, key)

    if key == "steering_observability":
        path = session / stage.directory / "derived" / "lidar_window_motion.parquet"
        if not path.exists():
            path = session / stage.directory / "derived" / "lidar_velocity.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"vx", "vy", "valid"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        valid = frame.valid.fillna(False).astype(bool).to_numpy()
        x = np.arange(len(frame))
        speed = np.hypot(frame.vx.to_numpy(float), frame.vy.to_numpy(float))
        figure, axes = plt.subplots(3, 1, figsize=(7.4, 6.6), sharex=True)
        for mask, label, color in ((valid, "accepted window", "tab:blue"), (~valid, "rejected window", "tab:red")):
            if mask.any():
                axes[0].scatter(x[mask], speed[mask], s=16, color=color, label=label)
                if "yaw_rate_icp" in frame:
                    axes[1].scatter(x[mask], frame.yaw_rate_icp.to_numpy(float)[mask], s=16, color=color)
                if "icp_rmse_m" in frame:
                    axes[2].scatter(x[mask], frame.icp_rmse_m.to_numpy(float)[mask] * 1000.0, s=16, color=color)
        axes[0].set(ylabel="LiDAR speed [m/s]", title="Robust LiDAR-window observability")
        axes[0].legend(fontsize=8)
        axes[1].axhline(0.0, color="black", linewidth=0.6)
        axes[1].set_ylabel("ICP yaw rate [rad/s]")
        axes[2].set(ylabel="ICP RMSE [mm]", xlabel="time-window index")
        return _save_figure(figure, session, plots, key)

    if key == "steering_centre":
        path = session / "analysis" / "centre_trim_points.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty:
            return None
        figure, axis = plt.subplots(figsize=(6.5, 4.0))
        x = frame["raw_servo_echo"]
        if "yaw_rate_icp_rad_s" in frame:
            axis.scatter(x, frame["yaw_rate_icp_rad_s"], label="LiDAR ICP", color="tab:blue")
        if "yaw_rate_imu_rad_s" in frame:
            axis.scatter(x, frame["yaw_rate_imu_rad_s"], label="IMU cross-check", color="tab:orange", marker="x")
        if "yaw_rate_odom_rad_s" in frame:
            axis.scatter(x, frame["yaw_rate_odom_rad_s"], label="odometry cross-check", color="tab:green", marker="+")
        result = _read(session / "analysis" / "centre_trim_offline.json") or {}
        centre = result.get("centre_servo_raw")
        if isinstance(centre, (float, int)) and math.isfinite(float(centre)):
            axis.axvline(float(centre), color="tab:red", linestyle="--", label=f"centre {float(centre):.4f}")
            slope = result.get("yaw_vs_servo_slope_rad_s_per_servo")
            if isinstance(slope, (float, int)) and math.isfinite(float(slope)):
                fit_x = np.linspace(float(x.min()), float(x.max()), 100)
                axis.plot(fit_x, float(slope) * (fit_x - float(centre)), color="tab:red", linewidth=1.2,
                          label="LiDAR linear fit")
        bootstrap = result.get("bootstrap", {}) if isinstance(result, dict) else {}
        centre_ci = bootstrap.get("centre_servo_raw_95pct", []) if isinstance(bootstrap, dict) else []
        if isinstance(centre_ci, list) and len(centre_ci) == 2 and all(math.isfinite(float(v)) for v in centre_ci):
            axis.axvspan(float(centre_ci[0]), float(centre_ci[1]), color="tab:red", alpha=0.12,
                         label="centre bootstrap 95%")
        axis.axhline(0.0, color="black", linewidth=0.8)
        axis.set(xlabel="raw servo command", ylabel="yaw rate [rad/s]", title="Steering centre: LiDAR fine fit; IMU/odom guidance")
        axis.legend()
        return _save_figure(figure, session, plots, key)
    if key == "steering_centre_validation":
        path = session / "analysis" / "centre_validation_trials.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty:
            return None
        figure, axes = plt.subplots(2, 1, figsize=(7.0, 5.5), sharex=True)
        labels = [str(value).split("__", 1)[0].replace("centre_validation_rep_", "rep ") for value in frame.trial_id]
        colors = ["tab:green" if bool(value) else "tab:red" for value in frame.accepted_for_validation]
        axes[0].bar(labels, frame.yaw_rate_icp_rad_s, color=colors)
        limits = _frozen_config(session).get("centre_trim", {})
        yaw_limit = float(limits.get("max_abs_validation_icp_yaw_rate_rad_s", 0.015))
        lateral_limit = float(limits.get("max_abs_validation_lateral_velocity_mps", 0.05))
        axes[0].axhline(yaw_limit, color="black", linestyle="--", linewidth=0.8)
        axes[0].axhline(-yaw_limit, color="black", linestyle="--", linewidth=0.8)
        axes[0].set_ylabel("ICP yaw [rad/s]")
        axes[1].bar(labels, frame.lidar_vy_mps, color=colors)
        axes[1].axhline(lateral_limit, color="black", linestyle="--", linewidth=0.8)
        axes[1].axhline(-lateral_limit, color="black", linestyle="--", linewidth=0.8)
        axes[1].set_ylabel("LiDAR lateral [m/s]")
        axes[1].set_xlabel("validation pass")
        figure.suptitle("Physical straightness validation (green = accepted)")
        return _save_figure(figure, session, plots, key)

    if key in {"steering_static_training", "steering_static_holdout"}:
        name = "static_map_training_segments.parquet" if key.endswith("training") else "static_map_holdout_segments.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty or not {"raw_servo_echo", "delta_eq_rad"}.issubset(frame.columns):
            return None
        figure, axis = plt.subplots(figsize=(6.5, 4.0))
        accepted = frame[frame.accepted.astype(bool)] if "accepted" in frame else frame
        rejected = frame[~frame.accepted.astype(bool)] if "accepted" in frame else frame.iloc[0:0]
        if len(rejected):
            axis.scatter(rejected.raw_servo_echo, rejected.delta_eq_rad, color="lightcoral",
                         alpha=0.65, label="rejected segment")
        if len(accepted):
            axis.scatter(accepted.raw_servo_echo, accepted.delta_eq_rad, color="tab:blue", label="accepted")
        candidate = _read(session / "analysis" / "candidate_static_steering_map.json") or {}
        x, y = candidate.get("raw_servo", []), candidate.get("delta_eq_rad", [])
        if len(x) >= 2 and len(x) == len(y):
            axis.plot(x, y, color="tab:red", label=str(candidate.get("plot_label", "training candidate")))
        axis.set(xlabel="raw servo echo", ylabel="effective steering [rad]",
                 title="Static servo-to-steering map" if key.endswith("training") else "Static-map hold-out")
        axis.legend()
        return _save_figure(figure, session, plots, key)

    if key in {"steering_response", "steering_response_validation"}:
        prefix = "validation_" if key.endswith("validation") else ""
        path = session / "analysis" / f"{prefix}command_to_effective_steering_response_metrics.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty:
            return None
        usable = frame[frame.effective_response_valid.fillna(False).astype(bool)] if "effective_response_valid" in frame else frame
        if usable.empty:
            usable = frame
        metric_panels = (
            ("effective_delay_10pct_s", "effective 10% delay [s]"),
            ("effective_rise_10_90_s", "effective rise 10–90% [s]"),
            ("fopdt_tau_s", "FOPDT time constant [s]"),
            ("fopdt_rmse_normalized", "FOPDT normalized RMSE"),
        )
        if not any(column in usable for column, _ in metric_panels):
            return None
        figure, axes = plt.subplots(2, 2, figsize=(8.0, 6.0), sharex=True)
        x = usable.speed_mps.to_numpy(float) if "speed_mps" in usable else np.arange(len(usable), dtype=float)
        side = usable.side.astype(str) if "side" in usable else pd.Series(["all"] * len(usable), index=usable.index)
        markers = {"left": "o", "right": "s", "all": "o"}
        colors = {"left": "tab:blue", "right": "tab:orange", "all": "tab:blue"}
        for axis, (column, label) in zip(axes.flat, metric_panels):
            if column not in usable:
                axis.axis("off")
                continue
            for group in side.unique():
                mask = side == group
                axis.scatter(x[mask.to_numpy()], usable.loc[mask, column].to_numpy(float), s=30,
                             marker=markers.get(str(group), "o"), color=colors.get(str(group), "tab:green"),
                             label=str(group), alpha=0.85)
            values = usable[column].to_numpy(float)
            values = values[np.isfinite(values)]
            if len(values):
                axis.axhline(float(np.median(values)), color="black", linestyle="--", linewidth=0.8,
                             label="median")
            axis.set_ylabel(label)
            axis.grid(alpha=0.18)
        for axis in axes[-1]:
            axis.set_xlabel("test speed [m/s]" if "speed_mps" in usable else "trial index")
        handles, labels = axes[0, 0].get_legend_handles_labels()
        if handles:
            axes[0, 0].legend(fontsize=7)
        figure.suptitle("Steering dynamics on independent hold-out" if key.endswith("validation")
                       else "Steering command-to-effective-angle dynamics")
        return _save_figure(figure, session, plots, key)

    if key == "motor_command_audit":
        samples = _runtime_result(session, stage).get("samples", [])
        if not isinstance(samples, list) or not samples:
            return None
        frame = pd.DataFrame(samples)
        if not {"label", "command_kind", "target"}.issubset(frame.columns):
            return None
        channels = (
            ("raw_erpm", "selected_speed_erpm_mean", "ERPM"),
            ("raw_current", "selected_current_a_mean", "current [A]"),
            ("raw_brake", "selected_brake_a_mean", "brake current [A]"),
        )
        figure, axes = plt.subplots(1, 3, figsize=(9.0, 3.9))
        any_panel = False
        for axis, (kind, column, unit) in zip(axes, channels):
            part = frame[frame.command_kind.astype(str) == kind]
            if part.empty or column not in part:
                axis.axis("off")
                continue
            any_panel = True
            x = np.arange(len(part))
            target = part.target.to_numpy(float)
            delivered = part[column].to_numpy(float)
            axis.plot(x, target, color="black", marker="x", linestyle="--", label="requested")
            axis.plot(x, delivered, color="tab:blue", marker="o", label="selector output")
            axis.set_xticks(x, part.label.astype(str), rotation=30, ha="right", fontsize=7)
            axis.set_ylabel(unit)
            axis.set_title(kind.replace("raw_", ""))
            axis.legend(fontsize=7)
        if not any_panel:
            plt.close(figure)
            return None
        figure.suptitle("Motor selector command-chain delivery")
        return _save_figure(figure, session, plots, key)

    if key == "longitudinal_observability":
        path = session / "analysis" / "longitudinal_observability_trials.parquet"
        all_path = session / "analysis" / "longitudinal_observability_all_trials.parquet"
        if not path.exists():
            return None
        accepted = pd.read_parquet(path)
        all_trials = pd.read_parquet(all_path) if all_path.exists() else accepted
        required = {"speed_command_mps", "vx_lidar_mps"}
        if all_trials.empty or not required.issubset(all_trials.columns):
            return None
        figure, axes = plt.subplots(1, 2, figsize=(8.2, 4.0))
        axes[0].scatter(all_trials.speed_command_mps, all_trials.vx_lidar_mps,
                        color="lightgray", label="all accepted captures")
        if len(accepted):
            axes[0].scatter(accepted.speed_command_mps, accepted.vx_lidar_mps,
                            color="tab:blue", label="passed straight/quality gates")
        bounds = _finite_limits(all_trials.speed_command_mps, all_trials.vx_lidar_mps)
        if bounds:
            axes[0].plot(bounds, bounds, color="black", linestyle="--", linewidth=0.8, label="ideal tracking")
            axes[0].set(xlim=bounds, ylim=bounds)
        axes[0].set(xlabel="commanded speed [m/s]", ylabel="LiDAR ground speed [m/s]",
                    title="Moving observability")
        axes[0].legend(fontsize=7)
        x = np.arange(len(all_trials))
        if "lidar_valid_fraction" in all_trials:
            axes[1].bar(x, all_trials.lidar_valid_fraction.to_numpy(float), color="tab:blue", alpha=0.75,
                        label="valid LiDAR-window fraction")
            axes[1].set_ylim(0.0, 1.05)
        if "imu_gz_rad_s" in all_trials:
            twin = axes[1].twinx()
            twin.plot(x, np.abs(all_trials.imu_gz_rad_s.to_numpy(float)), color="tab:orange", marker="o",
                      linewidth=1.0, label="|IMU yaw rate|")
            twin.set_ylabel("|IMU yaw rate| [rad/s]", color="tab:orange")
        axes[1].set(xlabel="probe trial", ylabel="valid fraction", title="Measurement quality and straightness")
        return _save_figure(figure, session, plots, key)

    if key == "low_speed_launch":
        path = session / "analysis" / "low_speed_launch_trials.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"nominal_speed_mps", "vx_lidar_mps"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        figure, axes = plt.subplots(1, 2, figsize=(8.2, 4.0))
        axes[0].scatter(frame.nominal_speed_mps, frame.vx_lidar_mps, color="tab:blue", alpha=0.85)
        bounds = _finite_limits(frame.nominal_speed_mps, frame.vx_lidar_mps)
        if bounds:
            axes[0].plot(bounds, bounds, color="black", linestyle="--", linewidth=0.8, label="ideal")
            axes[0].set(xlim=bounds, ylim=bounds)
        report_path = session / "analysis" / "low_speed_launch_report.yaml"
        if report_path.exists():
            import yaml
            report = yaml.safe_load(report_path.read_text(encoding="utf-8")) or {}
            first = report.get("first_repeatable_launch", {})
            if isinstance(first, dict):
                speed = first.get("nominal_speed_mps", first.get("ground_speed_mps"))
                if isinstance(speed, (float, int)) and math.isfinite(float(speed)):
                    axes[0].axvline(float(speed), color="tab:green", linestyle=":", label=f"first repeatable {float(speed):.2f} m/s")
        axes[0].set(xlabel="nominal raw-ERPM speed [m/s]", ylabel="LiDAR ground speed [m/s]",
                    title="Low-speed launch repeatability")
        axes[0].legend(fontsize=7)
        erpm_column = "erpm_measured" if "erpm_measured" in frame else "selected_speed_erpm"
        if erpm_column in frame:
            axes[1].scatter(frame[erpm_column], frame.vx_lidar_mps, color="tab:orange")
            axes[1].set(xlabel="measured ERPM" if erpm_column == "erpm_measured" else "selected ERPM",
                        ylabel="LiDAR ground speed [m/s]", title="Launch dead-band evidence")
        else:
            axes[1].axis("off")
        return _save_figure(figure, session, plots, key)

    if key == "vel_to_erpm_audit":
        path = session / "analysis" / "vel_to_erpm_pipeline_audit_trials.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"speed_command_mps", "vx_lidar_mps"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        figure, axes = plt.subplots(1, 2, figsize=(8.2, 4.0))
        axes[0].scatter(frame.speed_command_mps, frame.vx_lidar_mps, color="tab:blue", label="new pipeline data")
        bounds = _finite_limits(frame.speed_command_mps, frame.vx_lidar_mps)
        if bounds:
            axes[0].plot(bounds, bounds, color="black", linestyle="--", linewidth=0.8, label="ideal")
            axes[0].set(xlim=bounds, ylim=bounds)
        axes[0].set(xlabel="Ackermann speed command [m/s]", ylabel="LiDAR ground speed [m/s]",
                    title="Rebuilt VEL_TO_ERPM end-to-end check")
        axes[0].legend(fontsize=7)
        if {"selected_speed_erpm", "erpm_measured"}.issubset(frame.columns):
            axes[1].scatter(frame.selected_speed_erpm, frame.erpm_measured, color="tab:orange")
            erpm_bounds = _finite_limits(frame.selected_speed_erpm, frame.erpm_measured)
            if erpm_bounds:
                axes[1].plot(erpm_bounds, erpm_bounds, color="black", linestyle="--", linewidth=0.8)
                axes[1].set(xlim=erpm_bounds, ylim=erpm_bounds)
            axes[1].set(xlabel="selected ERPM", ylabel="VESC measured ERPM", title="Selector-to-motor tracking")
        else:
            axes[1].axis("off")
        return _save_figure(figure, session, plots, key)

    if key in {"erpm_response", "erpm_response_validation"}:
        name = "erpm_response_validation_trials.parquet" if key.endswith("validation") else "erpm_response_trials.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"baseline_speed_mps", "target_speed_mps"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        labels = [f"{base:.1f}→{target:.1f}" for base, target in
                  zip(frame.baseline_speed_mps.to_numpy(float), frame.target_speed_mps.to_numpy(float))]
        x = np.arange(len(frame))
        figure, axes = plt.subplots(3, 1, figsize=(8.0, 6.6), sharex=True)
        panels = (
            ("erpm_delay_10pct_s", "command→ERPM delay [s]", "tab:blue"),
            ("ground_speed_delay_10pct_s", "command→ground-speed delay [s]", "tab:orange"),
            ("ground_speed_tau_s", "ground-speed FOPDT τ [s]", "tab:green"),
        )
        any_panel = False
        for axis, (column, ylabel, color) in zip(axes, panels):
            if column not in frame:
                axis.axis("off")
                continue
            any_panel = True
            values = frame[column].to_numpy(float)
            axis.scatter(x, values, color=color, s=32)
            finite = values[np.isfinite(values)]
            if len(finite):
                axis.axhline(float(np.median(finite)), color="black", linestyle="--", linewidth=0.8,
                             label=f"median {float(np.median(finite)):.3f} s")
                axis.legend(fontsize=7)
            axis.set_ylabel(ylabel)
            axis.grid(alpha=0.18)
        if not any_panel:
            plt.close(figure)
            return None
        axes[-1].set_xticks(x, labels, rotation=35, ha="right", fontsize=7)
        axes[-1].set_xlabel("speed step [m/s]")
        figure.suptitle("ERPM/ground-speed dynamics on independent hold-out" if key.endswith("validation")
                       else "ERPM and ground-speed step dynamics")
        return _save_figure(figure, session, plots, key)

    if key in {"erpm_map_training", "erpm_map_holdout"}:
        name = "erpm_map_training_trials.parquet" if key.endswith("training") else "erpm_map_holdout_trials.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty or not {"vx_lidar_mps", "erpm_measured"}.issubset(frame.columns):
            return None
        figure, axis = plt.subplots(figsize=(6.5, 4.0))
        axis.scatter(frame.vx_lidar_mps, frame.erpm_measured, color="tab:blue", label="independent LiDAR trials")
        report_path = session / "analysis" / "erpm_speed_map_training_report.yaml"
        if report_path.exists():
            report = _read_yaml(report_path) or {}
            gain = report.get("candidate_speed_to_erpm_gain")
            if isinstance(gain, (float, int)) and math.isfinite(float(gain)):
                x = np.linspace(0.0, max(0.1, float(frame.vx_lidar_mps.max())), 80)
                axis.plot(x, float(gain) * x, color="tab:red", label=f"candidate {float(gain):.0f} ERPM/(m/s)")
                bootstrap = report.get("candidate_speed_to_erpm_gain_bootstrap", {})
                interval = bootstrap.get("gain_erpm_per_mps_95pct", []) if isinstance(bootstrap, dict) else []
                if isinstance(interval, list) and len(interval) == 2:
                    axis.fill_between(x, float(interval[0]) * x, float(interval[1]) * x,
                                      color="tab:red", alpha=0.12, label="gain bootstrap 95%")
        axis.set(xlabel="LiDAR ground speed [m/s]", ylabel="measured ERPM", title="ERPM-to-ground-speed map")
        axis.legend()
        return _save_figure(figure, session, plots, key)

    if key in {"current_training", "current_holdout"}:
        name = "current_model_training_trials.parquet" if key.endswith("training") else "current_model_holdout_trials.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        current_column = "selected_actuation_a" if "selected_actuation_a" in frame else "current_command_a"
        if frame.empty or current_column not in frame or "net_accel_mps2" not in frame:
            return None
        figure, axis = plt.subplots(figsize=(6.5, 4.0))
        colors = frame.polarity.astype(str) if "polarity" in frame else "tab:blue"
        palette = {"drive": "tab:blue", "brake": "tab:orange"}
        if isinstance(colors, str):
            axis.scatter(frame[current_column], frame.net_accel_mps2, color=colors)
        else:
            for polarity in sorted(colors.unique()):
                part = frame[colors == polarity]
                axis.scatter(part[current_column], part.net_accel_mps2,
                             color=palette.get(str(polarity)), label=f"{polarity} trials")
        report = _read_yaml(session / "analysis" / "current_acceleration_training_report.yaml") or {}
        scalar = report.get("low_slip_scalar_fit", {}) if isinstance(report, dict) else {}
        for polarity, fit in scalar.items() if isinstance(scalar, dict) else ():
            part = frame[frame.polarity.astype(str) == str(polarity)] if "polarity" in frame else frame
            slope = fit.get("accel_per_amp") if isinstance(fit, dict) else None
            if len(part) and isinstance(slope, (float, int)) and math.isfinite(float(slope)):
                fit_x = np.linspace(0.0, max(0.1, float(part[current_column].max())), 80)
                axis.plot(fit_x, float(slope) * fit_x, color=palette.get(str(polarity)), linewidth=1.4,
                          linestyle="--", label=f"frozen {polarity} scalar")
        axis.legend(fontsize=7)
        axis.set(xlabel="selected drive/brake current [A]", ylabel="drag-corrected LiDAR acceleration [m/s²]",
                 title="Current-to-acceleration training" if key.endswith("training") else "Current model hold-out")
        return _save_figure(figure, session, plots, key)

    if key in {"coastdown", "coastdown_validation"}:
        name = "coastdown_samples.parquet" if key == "coastdown" else "coastdown_validation_samples.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty or not {"trial_id", "t_s", "vx_mps"}.issubset(frame.columns):
            return None
        figure, axis = plt.subplots(figsize=(6.5, 4.0))
        for trial_id, part in frame.groupby("trial_id", sort=False):
            axis.plot(part.t_s, part.vx_mps, alpha=0.65, label=str(trial_id).split("__", 1)[0])
            if "vx_model_mps" in part:
                axis.plot(part.t_s, part.vx_model_mps, color="black", linewidth=0.7, alpha=0.4)
        axis.set(xlabel="time since zero-current command [s]", ylabel="LiDAR speed [m/s]",
                 title="Coast-down training trajectories" if key == "coastdown" else "Frozen drag model on hold-out trajectories")
        if frame.trial_id.nunique() <= 10:
            axis.legend(fontsize=6, ncol=2)
        return _save_figure(figure, session, plots, key)

    if key in {"accel_interface", "accel_interface_validation"}:
        name = "accel_to_current_interface_trials.parquet" if key == "accel_interface" else "accel_to_current_interface_validation_trials.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        if frame.empty or not {"expected_ground_accel_mps2", "observed_ground_accel_mps2"}.issubset(frame.columns):
            return None
        figure, axis = plt.subplots(figsize=(5.8, 4.4))
        colors = frame.route.astype(str) if "route" in frame else None
        if colors is not None:
            for route in sorted(colors.unique()):
                part = frame[colors == route]
                axis.scatter(part.expected_ground_accel_mps2, part.observed_ground_accel_mps2, label=route)
            axis.legend()
        else:
            axis.scatter(frame.expected_ground_accel_mps2, frame.observed_ground_accel_mps2)
        finite = np.r_[frame.expected_ground_accel_mps2.to_numpy(float), frame.observed_ground_accel_mps2.to_numpy(float)]
        finite = finite[np.isfinite(finite)]
        if len(finite):
            lo, hi = float(finite.min()), float(finite.max())
            axis.plot([lo, hi], [lo, hi], color="black", linestyle="--", linewidth=0.8, label="ideal")
        axis.set(xlabel="expected ground acceleration [m/s²]", ylabel="LiDAR ground acceleration [m/s²]",
                 title="ACCEL_TO_CURRENT training" if key == "accel_interface" else "ACCEL_TO_CURRENT independent hold-out")
        return _save_figure(figure, session, plots, key)

    if key == "odometry_candidate_velocity_validation":
        path = session / "analysis" / "odometry_candidate_velocity_trials.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"vx_lidar_mps", "speed_command_mps", "candidate_odom_vx_mps"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        figure, axis = plt.subplots(figsize=(6.2, 4.5))
        axis.scatter(frame.vx_lidar_mps, frame.speed_command_mps, marker="x", label="commanded speed")
        axis.scatter(frame.vx_lidar_mps, frame.candidate_odom_vx_mps, marker="o", label="shadow odometry")
        limits = _finite_limits(frame.vx_lidar_mps, frame.speed_command_mps, frame.candidate_odom_vx_mps)
        if limits:
            axis.plot(limits, limits, color="black", linestyle="--", linewidth=0.8, label="ideal")
            axis.set(xlim=limits, ylim=limits)
        axis.set(xlabel="LiDAR ground speed [m/s]", ylabel="command / estimate [m/s]",
                 title="Selected odometry: fresh speed hold-outs")
        axis.legend(fontsize=8)
        return _save_figure(figure, session, plots, key)

    if key == "odometry_candidate_accel_validation":
        path = session / "analysis" / "candidate_dynamic_speed_samples.parquet"
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"vx", "candidate_vx_mps", "imu_ax"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        measured = frame.vx.to_numpy(float)
        predicted = frame.candidate_vx_mps.to_numpy(float)
        accel = frame.imu_ax.to_numpy(float)
        residual = predicted - measured
        figure, axes = plt.subplots(1, 2, figsize=(8.0, 3.8))
        scatter = axes[0].scatter(measured, predicted, c=accel, cmap="coolwarm", s=18)
        limits = _finite_limits(measured, predicted)
        if limits:
            axes[0].plot(limits, limits, color="black", linestyle="--", linewidth=0.8)
            axes[0].set(xlim=limits, ylim=limits)
        axes[0].set(xlabel="LiDAR speed [m/s]", ylabel="shadow odometry [m/s]", title="Transient speed")
        figure.colorbar(scatter, ax=axes[0], label="IMU longitudinal acceleration [m/s²]")
        axes[1].scatter(accel, residual, s=18, color="tab:purple")
        axes[1].axhline(0.0, color="black", linewidth=0.8)
        axes[1].set(xlabel="IMU longitudinal acceleration [m/s²]", ylabel="odometry − LiDAR [m/s]",
                    title="Residual by dynamic regime")
        return _save_figure(figure, session, plots, key)

    if key in {"lateral_stiffness_training", "lateral_stiffness_validation"}:
        name = "lateral_stiffness_training_trials.parquet" if key == "lateral_stiffness_training" else "lateral_stiffness_validation_trials.parquet"
        path = session / "analysis" / name
        if not path.exists():
            return None
        frame = pd.read_parquet(path)
        required = {"alpha_front_rad", "fy_front_N", "alpha_rear_rad", "fy_rear_N"}
        if frame.empty or not required.issubset(frame.columns):
            return None
        turn_columns = {
            "turn_slip_lateral_accel_regressor_mps2", "turn_slip_fraction",
            "turn_slip_measurement_valid",
        }
        has_turn_slip = turn_columns.issubset(frame.columns)
        figure, axes = plt.subplots(1, 3 if has_turn_slip else 2,
                                    figsize=(10.8 if has_turn_slip else 7.4, 3.8), sharey=False)
        usable = frame[frame.measurement_valid.astype(bool)] if "measurement_valid" in frame else frame
        training = _read_yaml(session / "analysis" / "lateral_stiffness_training_report.yaml") or {}
        intervals = training.get("bootstrap_linear_cornering_stiffness_95pct_N_per_rad", {}) if isinstance(training, dict) else {}
        for axis, alpha, force, label, model_key in (
            (axes[0], "alpha_front_rad", "fy_front_N", "front", "front_tyre"),
            (axes[1], "alpha_rear_rad", "fy_rear_N", "rear", "rear_tyre"),
        ):
            axis.scatter(frame[alpha], frame[force], color="lightgray", label="rejected/all")
            axis.scatter(usable[alpha], usable[force], color="tab:blue", label="accepted")
            model = training.get(model_key, {}) if isinstance(training, dict) else {}
            linear = model.get("linear", {}) if isinstance(model, dict) else {}
            nonlinear = model.get("nonlinear", {}) if isinstance(model, dict) else {}
            runtime = model.get("runtime_bounded", {}) if isinstance(model, dict) else {}
            stiffness = linear.get("cornering_stiffness_N_per_rad") if isinstance(linear, dict) else None
            if len(usable) and isinstance(stiffness, (float, int)) and math.isfinite(float(stiffness)):
                limit = max(0.01, float(np.nanmax(np.abs(usable[alpha].to_numpy(float)))))
                fit_x = np.linspace(-limit, limit, 100)
                axis.plot(fit_x, float(stiffness) * fit_x, color="gray", linestyle=":",
                          label="small-angle tangent")
                shape = runtime.get("pacejka_shape_factor") if isinstance(runtime, dict) else None
                if isinstance(shape, (float, int)) and math.isfinite(float(shape)) and float(shape) > 0.0:
                    basis = np.sin(float(shape) * np.arctan(fit_x / float(shape)))
                    axis.plot(fit_x, float(stiffness) * basis, color="tab:red",
                              label="frozen runtime bounded")
                else:
                    basis = fit_x
                    axis.plot(fit_x, float(stiffness) * basis, color="tab:red",
                              label="frozen runtime")
                interval = intervals.get(label, []) if isinstance(intervals, dict) else []
                if isinstance(interval, list) and len(interval) == 2:
                    low = np.minimum(float(interval[0]) * basis, float(interval[1]) * basis)
                    high = np.maximum(float(interval[0]) * basis, float(interval[1]) * basis)
                    axis.fill_between(fit_x, low, high, color="tab:red", alpha=0.12, label="stiffness bootstrap 95%")
                q = nonlinear.get("quadratic_saturation_N_per_rad2") if isinstance(nonlinear, dict) else None
                nonlinear_c = nonlinear.get("cornering_stiffness_N_per_rad") if isinstance(nonlinear, dict) else None
                if all(isinstance(value, (float, int)) and math.isfinite(float(value)) for value in (q, nonlinear_c)):
                    fit_y = float(nonlinear_c) * fit_x + float(q) * fit_x * np.abs(fit_x)
                    axis.plot(fit_x, fit_y, color="tab:purple", linestyle="--", label="diagnostic nonlinear")
            axis.axhline(0.0, color="black", linewidth=0.5)
            axis.axvline(0.0, color="black", linewidth=0.5)
            axis.set(xlabel=f"{label} slip angle [rad]", ylabel=f"{label} lateral force [N]")
            axis.legend(fontsize=6)
        if has_turn_slip:
            axis = axes[2]
            turn_usable = frame[frame.turn_slip_measurement_valid.astype(bool)]
            axis.scatter(
                frame.turn_slip_lateral_accel_regressor_mps2,
                frame.turn_slip_fraction,
                color="lightgray", label="rejected/all",
            )
            axis.scatter(
                turn_usable.turn_slip_lateral_accel_regressor_mps2,
                turn_usable.turn_slip_fraction,
                color="tab:blue", label="accepted trial",
            )
            candidate = training.get("cornering_longitudinal_slip", {}) if isinstance(training, dict) else {}
            coefficient = candidate.get("selected_coefficient_per_mps2") if isinstance(candidate, dict) else None
            clip_fraction = candidate.get("clip_fraction", 0.25) if isinstance(candidate, dict) else 0.25
            if len(turn_usable) and isinstance(coefficient, (float, int)) and math.isfinite(float(coefficient)):
                maximum = max(0.1, float(np.nanmax(turn_usable.turn_slip_lateral_accel_regressor_mps2)))
                fit_x = np.linspace(0.0, maximum, 100)
                fit_y = np.clip(float(coefficient) * fit_x, 0.0, float(clip_fraction))
                axis.plot(fit_x, fit_y, color="tab:red", label="frozen causal model")
                interval = candidate.get("coefficient_bootstrap_95pct_per_mps2", [])
                if isinstance(interval, list) and len(interval) == 2:
                    low = np.clip(float(interval[0]) * fit_x, 0.0, float(clip_fraction))
                    high = np.clip(float(interval[1]) * fit_x, 0.0, float(clip_fraction))
                    axis.fill_between(fit_x, low, high, color="tab:red", alpha=0.12,
                                      label="coefficient bootstrap 95%")
            axis.axhline(0.0, color="black", linewidth=0.5)
            axis.set(
                xlabel=r"$|v_{wheel}|\,|r|$ [m/s²]",
                ylabel=r"$1-v_{LiDAR}/v_{wheel}$",
                title="Cornering wheel over-read",
            )
            axis.legend(fontsize=6)
        figure.suptitle("Effective tyre-force data" if key == "lateral_stiffness_training" else "Frozen tyre-model hold-out data")
        return _save_figure(figure, session, plots, key)

    if key == "physical_metrology":
        path = session / "analysis" / "physical_vehicle_parameters.yaml"
        if not path.exists():
            return None
        import yaml
        data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
        lidar = data.get("lidar_to_base", {}) if isinstance(data, dict) else {}
        imu = data.get("imu_to_base", {}) if isinstance(data, dict) else {}
        rear = data.get("rear_axle_in_base_link", {}) if isinstance(data, dict) else {}
        fields = (
            ("mass [kg]", data.get("mass_kg")),
            ("wheelbase [m]", data.get("wheelbase_m")),
            ("l_f [m]", data.get("cg_to_front_axle_lf_m")),
            ("l_r [m]", data.get("cg_to_rear_axle_lr_m")),
            ("LiDAR x [m]", lidar.get("x_m") if isinstance(lidar, dict) else None),
            ("LiDAR y [m]", lidar.get("y_m") if isinstance(lidar, dict) else None),
            ("rear axle x [m]", rear.get("x_m") if isinstance(rear, dict) else None),
            ("rear axle y [m]", rear.get("y_m") if isinstance(rear, dict) else None),
            ("IMU x [m]", imu.get("x_m") if isinstance(imu, dict) else None),
            ("IMU y [m]", imu.get("y_m") if isinstance(imu, dict) else None),
            ("IMU yaw [rad]", imu.get("yaw_rad") if isinstance(imu, dict) else None),
            ("CG x in base [m]", data.get("cg_in_base_link", {}).get("x_m") if isinstance(data.get("cg_in_base_link"), dict) else None),
            ("I_z [kg m²]", data.get("selected_yaw_inertia_kg_m2")),
        )
        figure, axis = plt.subplots(figsize=(6.5, 4.4))
        axis.axis("off")
        text = "\n".join(f"{name}: {value if value is not None else 'missing'}" for name, value in fields)
        axis.text(0.05, 0.92, "Direct physical measurements", fontsize=13, weight="bold", va="top")
        axis.text(0.05, 0.76, text, family="monospace", fontsize=10, va="top")
        axis.text(0.05, 0.08, "Axle-load closure and geometry provenance are recorded in YAML; no uncertainty is fabricated.", fontsize=9, va="bottom")
        return _save_figure(figure, session, plots, key)

    path = session / stage.directory / "derived" / "events.parquet"
    if not path.exists():
        return None
    frame = pd.read_parquet(path)
    if frame.empty or "event" not in frame:
        return None
    decisions = frame[frame.event == "trial_decision"] if "trial_decision" in set(frame.event) else frame.iloc[0:0]
    counts = decisions.decision.value_counts() if len(decisions) and "decision" in decisions else None
    figure, axis = plt.subplots(figsize=(6.5, 3.8))
    if counts is not None and len(counts):
        axis.bar(counts.index.astype(str), counts.to_numpy(), color="tab:blue")
        axis.set_ylabel("trial decisions")
    else:
        axis.bar(["events"], [len(frame)], color="tab:blue")
        axis.set_ylabel("decoded events")
    axis.set_title(f"{key}: capture coverage")
    return _save_figure(figure, session, plots, key)


def write_campaign_log(runner: Any) -> Path:
    """Regenerate the campaign index from the manifest.

    It is regenerated rather than append-only so an explicit recalibration does
    not leave an obsolete failed-candidate link pretending to be current.
    """
    session = Path(runner.session)
    manifest = runner.manifest
    lines = ["# Vehicle calibration log", ""]
    for stage in runner.STAGES:
        entry = manifest.get("stages", {}).get(stage.key)
        if not isinstance(entry, dict):
            continue
        lines += [f"## {stage.key}", "", f"- Status: `{entry.get('status', 'unknown')}`"]
        report = entry.get("stage_report")
        if report:
            lines.append(f"- Stage report: `{report}`")
        plot = session / "plots" / "stages" / f"{stage.key}.png"
        if plot.exists():
            lines.append(f"- Plot: `../{plot.relative_to(session)}`")
        scalars = _scalar_lines(entry.get("analysis", {}), limit=1)
        if scalars:
            lines.append(f"- Headline: `{scalars[0]}`")
        lines.append("")
    output = session / "analysis" / "vehicle_calibration_log.md"
    output.write_text("\n".join(lines), encoding="utf-8")
    return output


def write_stage_report(runner: Any, stage: Any, analysis: dict[str, Any]) -> str:
    """Write one stage page; the manifest-backed campaign log is rebuilt later."""
    session = Path(runner.session)
    report_dir = session / "analysis" / "stage_reports"
    report_dir.mkdir(parents=True, exist_ok=True)
    plot = None
    try:
        plot = _plot_stage(session, stage, session / "plots" / "stages")
    except Exception as exc:  # plotting must never invalidate a valid capture
        (report_dir / f"{stage.key}.plot_error.txt").write_text(repr(exc) + "\n", encoding="utf-8")
    lines = [
        f"# {stage.key}",
        "",
        "Workflow completed: capture → offline analysis/gate → recorded update or validation; any temporary VESC patch was restored after the stage.",
        "",
        "## Numbers",
        "",
    ]
    scalars = _scalar_lines(analysis)
    lines.extend(f"- `{line}`" for line in scalars) if scalars else lines.append("- No scalar analysis values were emitted; inspect the raw and Parquet artifacts.")
    statistics = analysis.get("statistical_evidence", {}) if isinstance(analysis, dict) else {}
    if isinstance(statistics, dict) and statistics:
        lines += ["", "## Statistical evidence", ""]
        for name in (
            "total_rows", "accepted_rows", "independent_units", "condition_count",
            "minimum_per_condition", "median_per_condition", "maximum_per_condition",
            "reported_accuracy_metric_count", "sampling",
        ):
            if statistics.get(name) is not None:
                lines.append(f"- `{name} = {statistics[name]}`")
        headline = statistics.get("headline_distribution")
        if isinstance(headline, dict) and headline.get("field"):
            lines.append(
                f"- `{headline['field']} median = {headline.get('median')}; "
                f"bootstrap 95% = {headline.get('bootstrap_median_95pct')}`"
            )
    lines += ["", "## Artifacts", "", f"- Raw stage data: `../../{stage.directory}/`"]
    if plot:
        lines.append(f"- Plot: `../../{plot}`")
    if isinstance(statistics, dict) and statistics.get("artifact"):
        lines.append(f"- Full trial-level statistics: `../../{statistics['artifact']}`")
    lines += ["- Analysis logs and gated JSON/YAML: `../`", ""]
    output = report_dir / f"{stage.key}.md"
    output.write_text("\n".join(lines), encoding="utf-8")

    return str(output.relative_to(session))
