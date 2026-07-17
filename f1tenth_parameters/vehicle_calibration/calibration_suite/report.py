"""Assemble a reviewable parameter document from a unified session."""
from __future__ import annotations

import json
import shutil
import subprocess
from pathlib import Path
from typing import Any

import yaml


def _read(path: Path) -> Any:
    if not path.exists():
        return None
    if path.suffix in {".yaml", ".yml"}:
        return yaml.safe_load(path.read_text(encoding="utf-8"))
    return json.loads(path.read_text(encoding="utf-8"))


def _finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number == number and abs(number) != float("inf") else None


def _nested(mapping: Any, *keys: str) -> Any:
    value = mapping
    for key in keys:
        if not isinstance(value, dict):
            return None
        value = value.get(key)
    return value


def _parameter_inventory(physical: dict[str, Any], centre: dict[str, Any],
                         steering_map: dict[str, Any], steering_response: dict[str, Any],
                         speed: dict[str, Any], coast: dict[str, Any],
                         current: dict[str, Any], lateral: dict[str, Any],
                         speed_validation: dict[str, Any],
                         lateral_validation: dict[str, Any],
                         low_speed_launch: dict[str, Any]) -> dict[str, Any]:
    """State exactly what the session has identified and what it cannot claim.

    Calling every number a "vehicle parameter" is tempting but scientifically
    misleading.  This inventory separates direct metrology, effective
    data-identified values, and quantities which need a different sensor or
    deliberately unsafe manoeuvre.  It is part of the final hand-off rather
    than an implicit limitation hidden in code comments.
    """
    recommended = physical.get("recommended_measurements", {}) if isinstance(physical, dict) else {}
    mass = _finite(physical.get("mass_kg"))
    drive_gain = _finite(current.get("candidate_accel_to_current_gain"))
    brake_gain = _finite(current.get("candidate_accel_to_brake_gain"))
    force_per_drive_amp = mass / drive_gain if mass is not None and drive_gain not in {None, 0.0} else None
    force_per_brake_amp = mass / brake_gain if mass is not None and brake_gain not in {None, 0.0} else None
    static_patch = steering_map.get("deployable_linear_patch", {}) if isinstance(steering_map, dict) else {}
    static_global = static_patch.get("global", {}) if isinstance(static_patch, dict) else {}
    return {
        "scope": (
            "Values marked effective are valid for the measured tyre state, load, battery and indoor surface. "
            "They are not claimed as universal component constants."
        ),
        "direct_metrology": {
            "mass_kg": mass,
            "wheelbase_m": physical.get("wheelbase_m"),
            "cg_to_front_axle_lf_m": physical.get("cg_to_front_axle_lf_m"),
            "cg_to_rear_axle_lr_m": physical.get("cg_to_rear_axle_lr_m"),
            "lidar_to_base": physical.get("lidar_to_base"),
            "imu_to_base": physical.get("imu_to_base"),
            "rear_axle_in_base_link": physical.get("rear_axle_in_base_link"),
            "cg_in_base_link": physical.get("cg_in_base_link"),
            "yaw_inertia_kg_m2": {
                "value": physical.get("selected_yaw_inertia_kg_m2"),
                "source": physical.get("selected_yaw_inertia_source"),
            },
            "other_direct_measurements": recommended,
        },
        "identified_and_validated": {
            "steering_offset_raw_servo": centre.get("centre_servo_raw"),
            "steering_linear_gain_servo_per_rad": static_global.get("steering_angle_to_servo_gain"),
            "steering_effective_response": {
                "median_fopdt_tau_s": steering_response.get("median_fopdt_tau_s"),
                "median_fopdt_delay_s": steering_response.get("median_fopdt_delay_s"),
                "median_effective_delay_10pct_s": steering_response.get("median_effective_delay_10pct_s"),
                "median_effective_rise_10_90_s": steering_response.get("median_effective_rise_10_90_s"),
                "median_effective_settling_5pct_s": steering_response.get("median_effective_settling_5pct_s"),
            },
            "erpm_speed_map": {
                "speed_to_erpm_gain": speed.get("candidate_speed_to_erpm_gain"),
                "odom_speed_scale": speed.get("candidate_odom_speed_scale"),
                "first_repeatable_low_speed_launch": low_speed_launch.get("first_repeatable_launch"),
                "holdout_accepted": speed_validation.get("accepted_for_validation"),
            },
            "effective_longitudinal_resistance": {
                "coulomb_mps2": coast.get("accel_drag_coulomb_mps2"),
                "viscous_per_s": coast.get("accel_drag_viscous_per_s"),
                "quadratic_per_m": coast.get("accel_drag_quadratic_per_m"),
                "coulomb_force_N": mass * _finite(coast.get("accel_drag_coulomb_mps2")) if mass is not None and _finite(coast.get("accel_drag_coulomb_mps2")) is not None else None,
                "viscous_force_N_per_mps": mass * _finite(coast.get("accel_drag_viscous_per_s")) if mass is not None and _finite(coast.get("accel_drag_viscous_per_s")) is not None else None,
                "quadratic_force_N_per_mps2": mass * _finite(coast.get("accel_drag_quadratic_per_m")) if mass is not None and _finite(coast.get("accel_drag_quadratic_per_m")) is not None else None,
            },
            "effective_longitudinal_actuation": {
                "accel_to_current_gain_A_per_mps2": drive_gain,
                "accel_to_brake_gain_A_per_mps2": brake_gain,
                "effective_drive_force_N_per_A": force_per_drive_amp,
                "effective_brake_force_N_per_A": force_per_brake_amp,
            },
            "effective_low_slip_tyre_stiffness": {
                "front_N_per_rad": _nested(lateral, "front_tyre", "linear", "cornering_stiffness_N_per_rad"),
                "rear_N_per_rad": _nested(lateral, "rear_tyre", "linear", "cornering_stiffness_N_per_rad"),
                "dynamic_model_steering_scale": lateral.get("steering_model_scale_candidate"),
                "dynamic_model_steering_scale_95pct": lateral.get("bootstrap_steering_model_scale_95pct"),
                "understeer_gradient_rad_per_mps2": lateral.get("linear_understeer_gradient_rad_per_mps2"),
                "validated_runtime_bounded_model": lateral.get("runtime_lateral_tyre_model"),
                "holdout_accepted": lateral_validation.get("accepted_for_validation"),
            },
            "cornering_longitudinal_wheel_slip": {
                "coefficient_per_mps2": _nested(
                    lateral, "cornering_longitudinal_slip", "selected_coefficient_per_mps2"
                ),
                "coefficient_95pct": _nested(
                    lateral, "cornering_longitudinal_slip", "coefficient_bootstrap_95pct_per_mps2"
                ),
                "correction_active": _nested(
                    lateral, "cornering_longitudinal_slip", "correction_active"
                ),
                "holdout": _nested(
                    lateral_validation, "cornering_longitudinal_slip_validation", "frozen_candidate_holdout"
                ),
                "runtime_equation_check": _nested(
                    lateral_validation, "cornering_longitudinal_slip_validation",
                    "runtime_odom_against_frozen_equation",
                ),
            },
        },
        "model_selection_flags": {
            "requires_nonlinear_static_speed_map": speed_validation.get("requires_full_stack_upgrade_for_selected_static_map", False),
            "requires_nonlinear_tyre_model": lateral_validation.get("requires_nonlinear_tyre_model", False),
            "dynamic_bicycle_transient_ready": physical.get("dynamic_bicycle_transient_ready", False),
        },
        "not_identifiable_from_this_safe_room_campaign": {
            "peak_tyre_friction_and_full_pacejka_shape": (
                "Not estimated: the low-slip arc grid deliberately avoids tyre saturation. "
                "The runtime shape factor remains a fixed bounded extrapolation whose adequacy is validated only over the safe arc envelope. "
                "Estimate peak/shape only with a separately approved near-limit test or tyre-force measurement."
            ),
            "motor_electrical_R_L_back_emf_and_gear_efficiency": (
                "Not separated by ground-current pulses. They require synchronized duty-cycle/voltage/current telemetry "
                "with a motor/roller test or a controlled electrical identification experiment."
            ),
            "separate_servo_linkage_and_tyre_relaxation_dynamics": (
                "The suite identifies the combined command-to-vehicle response. Splitting components requires a measured "
                "wheel-angle/servo-shaft sensor and a dedicated transient model."
            ),
            "component_level_tyre_stiffness_without_direct_road_wheel_angle": (
                "The reported C_f/C_r values are effective values conditional on the LiDAR-derived steering map and fitted "
                "dynamic steering scale. A multi-point physical road-wheel-angle measurement is required to separate steering "
                "geometry error from a universal tyre cornering-stiffness constant."
            ),
            "aerodynamic_CdA_vs_rolling_resistance": (
                "Coast-down identifies an effective constant/linear/quadratic resistance curve over 0.7–2.2 m/s, not a "
                "unique CdA and rolling-resistance decomposition."
            ),
        },
    }


def _mpc_simulation_bundle(
    session: Path,
    *,
    accepted: bool,
    physical: dict[str, Any],
    centre: dict[str, Any],
    steering_map: dict[str, Any],
    steering_response: dict[str, Any],
    speed: dict[str, Any],
    erpm_response: dict[str, Any],
    coast: dict[str, Any],
    current: dict[str, Any],
    lateral: dict[str, Any],
    odometry: dict[str, Any],
    odometry_patch: dict[str, Any],
    odometry_validation: dict[str, Any],
    voltage_temperature: dict[str, Any],
    room: dict[str, Any],
) -> dict[str, Any]:
    """Emit a compact, controller-oriented view of accepted evidence."""
    recommended = physical.get("recommended_measurements", {}) if isinstance(physical, dict) else {}

    def measured(name: str) -> Any:
        item = recommended.get(name, {}) if isinstance(recommended, dict) else {}
        return item.get("value") if isinstance(item, dict) else None

    static_patch = steering_map.get("deployable_linear_patch", {}) if isinstance(steering_map, dict) else {}
    steering_global = static_patch.get("global", {}) if isinstance(static_patch, dict) else {}
    odom_params = _nested(odometry_patch, "vesc_to_odom_node", "ros__parameters") or {}
    command_params = _nested(odometry_patch, "ackermann_to_vesc_node", "ros__parameters") or {}
    selected_family = odometry.get("selected_family")
    production_port_required = not bool(
        selected_family in {"legacy_scalar", "static_linear"}
        and odometry.get("command_map_selected") == "linear"
        and odom_params.get("odom_wheel_model") == "linear"
        and (session / "analysis" / "odometry_runtime_vesc_patch.yaml").is_file()
    )
    return {
        "provenance": {
            "session": str(session),
            "all_campaign_gates_passed": accepted,
            "units": "SI unless explicitly named otherwise",
            "safe_test_envelope": (
                f"indoor low-slip data at <=3.0 m/s in a "
                f"{room.get('physical_room_length_m', 14.0):g} x "
                f"{room.get('physical_room_width_m', 14.0):g} m room; "
                f"{room.get('wall_clearance_m', 1.0):g} m wall clearance; "
                f"straight lane at {room.get('planned_straight_lane_heading_deg', 45.0):g} degrees"
            ),
            "room_preflight": room,
        },
        "recommended_state_models": {
            "kinematic_mpc": ["x_m", "y_m", "yaw_rad", "vx_mps", "steering_effective_rad"],
            "dynamic_bicycle_mpc": ["x_m", "y_m", "yaw_rad", "vx_mps", "vy_mps", "yaw_rate_rad_s", "steering_effective_rad"],
            "optional_longitudinal_actuator_states": ["longitudinal_acceleration_mps2", "wheel_speed_mps", "longitudinal_slip_mps"],
            "note": "State lists are implementation targets; include only states whose required parameters below are populated and validated.",
        },
        "physical": {
            "mass_kg": physical.get("mass_kg"),
            "yaw_inertia_kg_m2": physical.get("selected_yaw_inertia_kg_m2"),
            "wheelbase_m": physical.get("wheelbase_m"),
            "cg_to_front_axle_lf_m": physical.get("cg_to_front_axle_lf_m"),
            "cg_to_rear_axle_lr_m": physical.get("cg_to_rear_axle_lr_m"),
            "cg_height_m": measured("cg_height_m"),
            "track_front_m": measured("track_front_m"),
            "track_rear_m": measured("track_rear_m"),
            "loaded_wheel_radius_m": measured("loaded_wheel_radius_m"),
        },
        "steering": {
            "servo_offset_raw": centre.get("centre_servo_raw"),
            "servo_per_road_wheel_rad": steering_global.get("steering_angle_to_servo_gain"),
            "servo_min": steering_global.get("servo_min"),
            "servo_max": steering_global.get("servo_max"),
            "dynamic_steering_scale": lateral.get("steering_model_scale_candidate"),
            "combined_fopdt_delay_s": steering_response.get("median_fopdt_delay_s"),
            "combined_fopdt_tau_s": steering_response.get("median_fopdt_tau_s"),
            "rise_10_90_s": steering_response.get("median_effective_rise_10_90_s"),
            "settling_5pct_s": steering_response.get("median_effective_settling_5pct_s"),
        },
        "longitudinal": {
            "speed_to_erpm_gain": speed.get("candidate_speed_to_erpm_gain"),
            "speed_to_erpm_quadratic": speed.get("candidate_speed_to_erpm_quadratic"),
            "erpm_response_delay_s": erpm_response.get("median_fopdt_delay_s"),
            "erpm_response_tau_s": erpm_response.get("median_fopdt_tau_s"),
            "drag_coulomb_mps2": coast.get("accel_drag_coulomb_mps2"),
            "drag_viscous_per_s": coast.get("accel_drag_viscous_per_s"),
            "drag_quadratic_per_m": coast.get("accel_drag_quadratic_per_m"),
            "accel_to_current_gain_A_per_mps2": current.get("candidate_accel_to_current_gain"),
            "accel_to_brake_gain_A_per_mps2": current.get("candidate_accel_to_brake_gain"),
            "selected_odometry_family": selected_family,
            "selected_odometry_parameters": odom_params,
            "selected_command_parameters": command_params,
            "fresh_shadow_validation_passed": odometry_validation.get("accepted_for_permanent_review"),
        },
        "lateral_low_slip": {
            "front_cornering_stiffness_N_per_rad": _nested(lateral, "front_tyre", "linear", "cornering_stiffness_N_per_rad"),
            "rear_cornering_stiffness_N_per_rad": _nested(lateral, "rear_tyre", "linear", "cornering_stiffness_N_per_rad"),
            "understeer_gradient_rad_per_mps2": lateral.get("linear_understeer_gradient_rad_per_mps2"),
            "runtime_bounded_tyre_model": lateral.get("runtime_lateral_tyre_model"),
            "cornering_longitudinal_slip": lateral.get("cornering_longitudinal_slip"),
        },
        "sensor_reference": {
            "lidar_to_base": physical.get("lidar_to_base"),
            "imu_to_base": physical.get("imu_to_base"),
            "rear_axle_in_base_link": physical.get("rear_axle_in_base_link"),
        },
        "validity_and_implementation": {
            "voltage_temperature_context": voltage_temperature,
            "dynamic_bicycle_transient_ready": physical.get("dynamic_bicycle_transient_ready", False),
            "selected_odometry_requires_production_cpp_port": production_port_required,
            "production_port_contract": "f1tenth_parameters/ERPM/full_stack/PRODUCTION_PORT_CONTRACT.md" if production_port_required else None,
            "full_pacejka_or_peak_friction_identified": False,
            "component_motor_electrical_constants_identified": False,
        },
    }


def _promote(runner: Any, candidate: dict[str, Any]) -> None:
    source = runner.workspace / str(runner.campaign["config_relpath"])
    if not source.is_file():
        raise FileNotFoundError(source)
    if not candidate.get("accepted_for_promotion"):
        raise RuntimeError("final candidate is not accepted; promotion is blocked")
    # Promotion is deliberately separate from run.  The source file is backed
    # up beside the session and the same build command is run immediately.
    backup = runner.session / "promotion" / "vesc.yaml.before_promotion"
    backup.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, backup)
    document = yaml.safe_load(source.read_text(encoding="utf-8")) or {}
    for node, values in candidate["vesc_patch"].items():
        section = document.setdefault("/**" if node == "global" else node, {})
        params = section.setdefault("ros__parameters", {})
        params.update(values)
    temporary = source.with_suffix(".promotion.tmp")
    temporary.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    temporary.replace(source)
    log = runner.session / "promotion" / "build_promotion.log"
    with log.open("w", encoding="utf-8") as handle:
        result = subprocess.run(list(runner.campaign["build_command"]), cwd=runner.workspace,
                                stdout=handle, stderr=subprocess.STDOUT, text=True, check=False)
    if result.returncode:
        shutil.copy2(backup, source)
        restore_log = runner.session / "promotion" / "build_restore_after_failed_promotion.log"
        with restore_log.open("w", encoding="utf-8") as handle:
            restore_result = subprocess.run(list(runner.campaign["build_command"]), cwd=runner.workspace,
                                             stdout=handle, stderr=subprocess.STDOUT, text=True, check=False)
        if restore_result.returncode:
            raise RuntimeError(
                f"promotion build failed and restored-source build also failed; "
                f"source bytes restored from {backup}; inspect {log} and {restore_log}"
            )
        raise RuntimeError(f"promotion build failed; source restored and rebuilt from {backup}; inspect {log}")


def _plot_centre(session: Path, plots: Path) -> str | None:
    path = session / "analysis" / "centre_trim_points.parquet"
    if not path.exists():
        return None
    try:
        import matplotlib.pyplot as plt
        import pandas as pd
    except ImportError:
        return None
    frame = pd.read_parquet(path)
    if frame.empty:
        return None
    plots.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(6.5, 4.0))
    yaw_column = "yaw_rate_icp_rad_s" if "yaw_rate_icp_rad_s" in frame else "yaw_rate_rad_s"
    axis.scatter(frame["raw_servo_echo"], frame[yaw_column], label="LiDAR ICP accepted capture")
    centre = _read(session / "analysis" / "centre_trim_offline.json") or {}
    x = _finite(centre.get("centre_servo_raw"))
    if x is not None:
        axis.axvline(x, color="tab:red", linestyle="--", label=f"candidate centre {x:.4f}")
    axis.axhline(0.0, color="black", linewidth=0.8)
    axis.set(xlabel="raw servo command", ylabel="LiDAR ICP yaw rate [rad/s]", title="Steering centre fit")
    axis.legend()
    figure.tight_layout()
    output = plots / "steering_centre_fit.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return str(output.relative_to(session))


def _plot_centre_validation(session: Path, plots: Path) -> str | None:
    path = session / "analysis" / "centre_validation_trials.parquet"
    if not path.exists():
        return None
    try:
        import matplotlib.pyplot as plt
        import pandas as pd
    except ImportError:
        return None
    frame = pd.read_parquet(path)
    if frame.empty:
        return None
    plots.mkdir(parents=True, exist_ok=True)
    figure, axes = plt.subplots(2, 1, figsize=(7.0, 5.5), sharex=True)
    labels = [f"{index + 1}" for index in range(len(frame))]
    colors = ["tab:green" if bool(value) else "tab:red" for value in frame.accepted_for_validation]
    axes[0].bar(labels, frame.yaw_rate_icp_rad_s, color=colors)
    axes[0].axhline(0.015, color="black", linestyle="--", linewidth=0.8)
    axes[0].axhline(-0.015, color="black", linestyle="--", linewidth=0.8)
    axes[0].set_ylabel("ICP yaw [rad/s]")
    axes[1].bar(labels, frame.lidar_vy_mps, color=colors)
    axes[1].axhline(0.05, color="black", linestyle="--", linewidth=0.8)
    axes[1].axhline(-0.05, color="black", linestyle="--", linewidth=0.8)
    axes[1].set_ylabel("LiDAR lateral [m/s]")
    axes[1].set_xlabel("validation pass")
    figure.suptitle("Centre physical straightness validation")
    figure.tight_layout()
    output = plots / "steering_centre_validation.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return str(output.relative_to(session))


def _plot_speed_map(session: Path, plots: Path) -> str | None:
    path = session / "analysis" / "erpm_map_training_trials.parquet"
    if not path.exists():
        return None
    try:
        import matplotlib.pyplot as plt
        import pandas as pd
    except ImportError:
        return None
    frame = pd.read_parquet(path)
    if frame.empty or "erpm_measured" not in frame or "vx_lidar_mps" not in frame:
        return None
    plots.mkdir(parents=True, exist_ok=True)
    figure, axis = plt.subplots(figsize=(6.5, 4.0))
    axis.scatter(frame["vx_lidar_mps"], frame["erpm_measured"], label="accepted training")
    report = _read(session / "analysis" / "erpm_speed_map_report.yaml") or {}
    gain = _finite(report.get("candidate_speed_to_erpm_gain"))
    if gain is not None:
        import numpy as np
        x = np.linspace(0.0, max(0.1, float(frame["vx_lidar_mps"].max())), 80)
        axis.plot(x, gain * x, color="tab:red", label=f"through-origin fit {gain:.1f} ERPM/(m/s)")
    axis.set(xlabel="LiDAR ground speed [m/s]", ylabel="measured ERPM", title="Longitudinal speed map")
    axis.legend()
    figure.tight_layout()
    output = plots / "erpm_speed_map.png"
    figure.savefig(output, dpi=160)
    plt.close(figure)
    return str(output.relative_to(session))


def build_report(runner: Any, *, promote: bool = False) -> None:
    session = runner.session
    analysis = session / "analysis"
    analysis.mkdir(parents=True, exist_ok=True)

    centre = _read(analysis / "centre_trim_offline.json") or {}
    centre_validation = _read(analysis / "centre_validation_report.json") or {}
    icp = _read(analysis / "icp_observability_report.json") or {}
    steering_map = _read(analysis / "candidate_static_steering_map.json") or {}
    steering_response = _read(analysis / "command_to_effective_steering_response_summary.json") or {}
    steering_response_validation = _read(analysis / "steering_response_validation_report.json") or {}
    speed = _read(analysis / "erpm_speed_map_report.yaml") or {}
    speed_validation = _read(analysis / "erpm_speed_map_validation_report.yaml") or {}
    erpm_response = _read(analysis / "erpm_response_report.yaml") or {}
    erpm_response_validation = _read(analysis / "erpm_response_validation_report.yaml") or {}
    coast = _read(analysis / "coastdown_drag_report.yaml") or {}
    coast_validation = _read(analysis / "coastdown_validation_report.yaml") or {}
    current = _read(analysis / "current_acceleration_report.yaml") or {}
    traction = _read(analysis / "traction_transient_report.yaml") or {}
    interface = _read(analysis / "accel_to_current_interface_report.yaml") or {}
    interface_validation = _read(analysis / "accel_to_current_interface_validation_report.yaml") or {}
    physical = _read(analysis / "physical_vehicle_parameters.yaml") or {}
    longitudinal_observability = _read(analysis / "longitudinal_observability_report.yaml") or {}
    low_speed_launch = _read(analysis / "low_speed_launch_report.yaml") or {}
    lateral = _read(analysis / "lateral_stiffness_training_report.yaml") or {}
    lateral_validation = _read(analysis / "lateral_stiffness_validation_report.yaml") or {}
    odometry = _read(analysis / "odometry_model_selection_report.yaml") or {}
    odometry_patch = _read(analysis / "selected_odometry_candidate_patch.yaml") or {}
    odometry_velocity_validation = _read(analysis / "odometry_candidate_velocity_validation_report.yaml") or {}
    odometry_validation = _read(analysis / "candidate_deployment_verification_report.yaml") or {}
    voltage_temperature = _read(analysis / "voltage_temperature_report.yaml") or {}
    state = runner.state
    room = runner.manifest.get("room_preflight", {})

    patch: dict[str, Any] = {}
    for file_name in (
        "steering_centre_vesc_patch.yaml", "steering_map_vesc_patch.yaml",
        "speed_map_vesc_patch.yaml", "coastdown_vesc_patch.yaml", "current_vesc_patch.yaml",
        "physical_vehicle_vesc_patch.yaml", "odometry_runtime_vesc_patch.yaml",
        "lateral_stiffness_vesc_patch.yaml",
    ):
        value = _read(analysis / file_name)
        if isinstance(value, dict):
            for node, values in value.items():
                if isinstance(values, dict):
                    patch.setdefault(node, {}).update(values)

    completed = [key for key, value in runner.manifest.get("stages", {}).items()
                 if value.get("status") == "completed"]
    observability_gates = runner.steering_cfg.get("analysis", {}).get("icp_observability", {})
    traction_scalar_ok = bool(traction.get("scalar_dynamic_traction_adequate_over_envelope", False))
    if not traction:
        traction_scalar_ok = bool(current.get("scalar_accel_to_current_adequate_over_envelope", False))
    gates = {
        "steering_centre": bool(centre.get("accepted_for_update", False)),
        "steering_centre_validation": bool(centre_validation.get("accepted_for_validation", False)),
        "steering_observability": bool(
            int(icp.get("moving_valid_windows", 0)) >= int(observability_gates.get("min_moving_valid_windows", 12))
            and float(icp.get("moving_window_valid_fraction") or 0.0) >= float(observability_gates.get("min_moving_window_valid_fraction", 0.70))
            and float(icp.get("moving_window_speed_median_mps") or 0.0) >= float(observability_gates.get("min_moving_window_speed_mps", 0.20))
            and _finite(icp.get("moving_window_point_to_line_rmse_p95_m")) is not None
            and float(icp["moving_window_point_to_line_rmse_p95_m"]) <= float(observability_gates.get("max_moving_point_to_line_rmse_p95_m", 0.03))
        ),
        "steering_static_map": bool(steering_map.get("accepted_for_deployment", False)),
        "steering_response_validation": bool(steering_response_validation.get("accepted_for_validation", False)),
        "longitudinal_observability": bool(longitudinal_observability.get("accepted_for_downstream", False)),
        "low_speed_launch": bool(low_speed_launch.get("accepted_for_downstream", False)),
        "erpm_speed_map": bool(speed.get("accepted_for_candidate", False)),
        "erpm_speed_map_validation": bool(
            speed_validation.get("accepted_for_validation", False)
            and not speed_validation.get("requires_full_stack_upgrade_for_selected_static_map", False)
        ),
        "erpm_response": bool(erpm_response.get("accepted_for_candidate", False)),
        "erpm_response_validation": bool(erpm_response_validation.get("accepted_for_validation", False)),
        "coastdown": bool(coast.get("accepted_for_candidate", False)),
        "coastdown_validation": bool(coast_validation.get("accepted_for_validation", False)),
        "current_model": bool(current.get("accepted_for_candidate", False)),
        "traction_scalar_candidate": traction_scalar_ok,
        "accel_interface": bool(interface.get("accepted_for_candidate", False)),
        "accel_interface_validation": bool(interface_validation.get("accepted_for_validation", False)),
        "odometry_model_selection": bool(odometry.get("accepted_for_shadow_deployment_verification", False)),
        "odometry_velocity_validation": bool(odometry_velocity_validation.get("accepted_for_validation", False)),
        "odometry_accel_validation": bool(odometry_validation.get("accepted_for_permanent_review", False)),
        "odometry_runtime_implementation_available": bool(
            odometry.get("selected_family") in {"legacy_scalar", "static_linear"}
            and odometry.get("command_map_selected") == "linear"
            and _nested(odometry_patch, "vesc_to_odom_node", "ros__parameters", "odom_wheel_model") == "linear"
            and (analysis / "odometry_runtime_vesc_patch.yaml").is_file()
        ),
        "physical_metrology": bool(physical.get("accepted_for_lateral_identification", False)),
        "lateral_stiffness": bool(lateral.get("accepted_for_candidate", False)),
        "lateral_stiffness_validation": bool(
            lateral_validation.get("accepted_for_validation", False)
            and not lateral_validation.get("requires_nonlinear_tyre_model", False)
        ),
        "all_stages_complete": len(completed) == len(runner.STAGES) if hasattr(runner, "STAGES") else len(completed) == 19,
    }
    accepted = bool(gates["all_stages_complete"] and gates["steering_centre"] and
                    gates["steering_centre_validation"] and gates["steering_observability"] and
                    gates["steering_static_map"] and gates["steering_response_validation"] and
                    gates["longitudinal_observability"] and gates["low_speed_launch"] and
                    gates["erpm_speed_map"] and gates["erpm_speed_map_validation"] and
                    gates["erpm_response"] and gates["erpm_response_validation"] and
                    gates["coastdown"] and gates["coastdown_validation"] and gates["current_model"] and
                    gates["traction_scalar_candidate"] and gates["accel_interface"] and gates["accel_interface_validation"] and
                    gates["odometry_model_selection"] and gates["odometry_velocity_validation"] and
                    gates["odometry_accel_validation"] and gates["odometry_runtime_implementation_available"] and
                    gates["physical_metrology"] and gates["lateral_stiffness"] and gates["lateral_stiffness_validation"])
    candidate = {
        "accepted_for_promotion": accepted,
        "vesc_patch": patch,
        "gates": gates,
        "source_session": str(session),
    }
    _dump = lambda path, value: path.write_text(yaml.safe_dump(value, sort_keys=False), encoding="utf-8")
    inventory = _parameter_inventory(
        physical, centre, steering_map, steering_response, speed, coast,
        current, lateral, speed_validation, lateral_validation, low_speed_launch,
    )
    _dump(analysis / "vehicle_parameter_inventory.yaml", inventory)
    mpc_bundle = _mpc_simulation_bundle(
        session,
        accepted=accepted,
        physical=physical,
        centre=centre,
        steering_map=steering_map,
        steering_response=steering_response,
        speed=speed,
        erpm_response=erpm_response,
        coast=coast,
        current=current,
        lateral=lateral,
        odometry=odometry,
        odometry_patch=odometry_patch,
        odometry_validation=odometry_validation,
        voltage_temperature=voltage_temperature,
        room=room,
    )
    _dump(analysis / "mpc_simulation_parameter_bundle.yaml", mpc_bundle)
    _dump(analysis / "vehicle_parameters_candidate.yaml", {
        "provenance": {"session": str(session), "gated": accepted},
        "steering": {
            "servo_offset_raw": centre.get("centre_servo_raw"),
            "centre_validation": centre_validation,
            "map": steering_map,
            "response": steering_response,
            "response_validation": steering_response_validation,
        },
        "longitudinal": {
            "speed_map": speed,
            "speed_map_validation": speed_validation,
            "coastdown": coast,
            "coastdown_validation": coast_validation,
            "erpm_response": erpm_response,
            "erpm_response_validation": erpm_response_validation,
            "current_model": current,
            "traction": traction,
            "acceleration_interface": interface,
            "acceleration_interface_validation": interface_validation,
            "odometry_model_selection": odometry,
            "odometry_velocity_validation": odometry_velocity_validation,
            "odometry_acceleration_validation": odometry_validation,
        },
        "physical_vehicle": physical,
        "lateral": {
            "effective_tyre_stiffness_training": lateral,
            "effective_tyre_stiffness_validation": lateral_validation,
        },
        "existing_runtime_state": state,
    })
    _dump(analysis / "vehicle_calibration_promotion_candidate.yaml", candidate)
    plots = session / "plots"
    plot_paths = [_plot_centre(session, plots), _plot_centre_validation(session, plots), _plot_speed_map(session, plots)]
    plot_paths = [path for path in plot_paths if path]

    lines = [
        f"# Unified vehicle-calibration report — `{session.name}`",
        "",
        "This report is generated from accepted stage records only. A candidate is not promoted unless every required stage and hold-out gate passes.",
        "",
        "## Campaign status",
        "",
        f"- Completed stages: {len(completed)} / {len(runner.STAGES)}",
        f"- Final candidate accepted for promotion: **{accepted}**",
        f"- Room profile: {room.get('physical_room_length_m', 'n/a')} × {room.get('physical_room_width_m', 'n/a')} m, "
        f"{room.get('wall_clearance_m', 'n/a')} m wall clearance, "
        f"{room.get('planned_straight_lane_heading_deg', 'n/a')}° diagonal lane "
        f"({float(room.get('usable_straight_m', 0.0)):.2f} m footprint-aware motion capacity), "
        f"maximum test speed {room.get('max_test_speed_mps', 'n/a')} m/s",
        f"- Nominal accepted driving passes: {runner.manifest.get('campaign_budget', {}).get('nominal_driving_trials_min', 'n/a')}–{runner.manifest.get('campaign_budget', {}).get('nominal_driving_trials_max', 'n/a')}",
        "",
        "## Gated outputs",
        "",
        f"- Steering centre: `{centre.get('centre_servo_raw', 'not available')}`",
        f"- Steering centre physical validation: `{gates['steering_centre_validation']}`",
        f"- Steering static map hold-out: `{gates['steering_static_map']}`",
        f"- Steering response hold-out: `{gates['steering_response_validation']}`",
        f"- Longitudinal moving-sensor preflight / low-speed launch: `{gates['longitudinal_observability']}` / `{gates['low_speed_launch']}`",
        f"- ERPM/speed map hold-out: `{gates['erpm_speed_map']}`",
        f"- ERPM/speed map validation after update: `{gates['erpm_speed_map_validation']}`",
        f"- ERPM response hold-out: `{gates['erpm_response_validation']}`",
        f"- Coast-down fit / hold-out: `{gates['coastdown']}` / `{gates['coastdown_validation']}`",
        f"- Current model hold-out: `{gates['current_model']}`",
        f"- ACCEL_TO_CURRENT physical hold-out: `{gates['accel_interface_validation']}`",
        f"- Selected causal odometry speed/transient hold-outs: `{gates['odometry_velocity_validation']}` / `{gates['odometry_accel_validation']}`",
        f"- Selected odometry production implementation available: `{gates['odometry_runtime_implementation_available']}`",
        f"- Physical metrology: `{gates['physical_metrology']}`",
        f"- Effective tyre stiffness hold-out: `{gates['lateral_stiffness_validation']}`",
        f"- Cornering wheel-slip correction active: `{_nested(lateral, 'cornering_longitudinal_slip', 'correction_active')}` "
        f"(zero remains a valid, independently tested result)",
        "",
        "## Artifacts",
        "",
        "- `analysis/vehicle_parameters_candidate.yaml` — assembled parameter document.",
        "- `analysis/vehicle_parameter_inventory.yaml` — direct, effective identified, and explicitly non-identifiable parameter inventory.",
        "- `analysis/mpc_simulation_parameter_bundle.yaml` — compact MPC/simulation state and parameter hand-off.",
        "- `analysis/vehicle_calibration_promotion_candidate.yaml` — only patch eligible for promotion.",
        "- Every stage directory contains the raw MCAP, export, runtime result, and bag verification.",
    ]
    if plot_paths:
        lines += ["", "## Plots", ""] + [f"- `{path}`" for path in plot_paths]
    (analysis / "vehicle_calibration_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    from .latex_report import write_latex_document
    latex_source = write_latex_document(runner, compile_pdf=True)
    if promote:
        _promote(runner, candidate)
        print("Promotion completed. The promotion backup and build log are in:", session / "promotion")
    print("Report:", analysis / "vehicle_calibration_report.md")
    print("LaTeX:", latex_source)
    pdf = latex_source.with_suffix(".pdf")
    if pdf.exists():
        print("PDF:", pdf)
