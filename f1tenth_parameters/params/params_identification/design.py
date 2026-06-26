from __future__ import annotations

import math
from typing import Any

G = 9.80665


def steady_lateral_conditions(config: dict[str, Any]) -> list[dict[str, Any]]:
    section = config["steady_lateral"]
    wheelbase = float(config["hardware"]["wheelbase_m"])
    conditions: list[dict[str, Any]] = []
    for radius in section["radii_m"]:
        radius = float(radius)
        steering = math.atan(wheelbase / radius)
        for direction in section["directions"]:
            sign = -1.0 if direction == "right" else 1.0
            for ay_g in section["ay_ladder_g_initial"]:
                ay = float(ay_g) * G
                speed = math.sqrt(ay * radius)
                for rep in range(1, int(section["repetitions"]) + 1):
                    conditions.append({
                        "stage": "steady_lateral",
                        "radius_m": radius,
                        "direction": direction,
                        "target_ay_g": float(ay_g),
                        "target_ay_mps2": ay,
                        "target_speed_mps": speed,
                        "effective_steering_angle_rad": sign * steering,
                        "repetition": rep,
                    })
    return conditions


def steering_transient_conditions(config: dict[str, Any]) -> list[dict[str, Any]]:
    section = config["steering_transients"]
    site = config["site"]
    vmax = float(config["session"]["max_race_speed_mps"])
    wheelbase = float(config["hardware"]["wheelbase_m"])
    conditions: list[dict[str, Any]] = []
    for speed_fraction in section["speed_fractions_of_vmax"]:
        speed = vmax * float(speed_fraction)
        for response_g in section["lateral_response_targets_g"]:
            ay = float(response_g) * G
            steering = math.atan((ay * wheelbase) / max(speed * speed, 1e-9))
            for order in section["doublet_orders"]:
                for rep in range(1, int(section["repetitions"]) + 1):
                    conditions.append({
                        "stage": "steering_transients",
                        "speed_mps": speed,
                        "speed_fraction_of_vmax": float(speed_fraction),
                        "target_response_g": float(response_g),
                        "target_response_mps2": ay,
                        "effective_steering_amplitude_rad": steering,
                        "doublet_order": order,
                        "repetition": rep,
                        "post_stable_required_m": post_stable_required_m(config, speed),
                        "post_stable_available_m": float(site["post_stable_corridor_available_m"]),
                    })
    return conditions


def post_stable_required_m(config: dict[str, Any], speed_mps: float) -> float:
    section = config["steering_transients"]
    capture_s = float(section["capture_s"])
    braking = float(config["site"]["verified_braking_mps2"])
    margin = float(section["post_capture_terminal_margin_m"])
    speed = float(speed_mps)
    return speed * capture_s + speed * speed / (2.0 * braking) + margin


def combined_slip_conditions(config: dict[str, Any]) -> list[dict[str, Any]]:
    section = config["combined_slip"]
    conditions: list[dict[str, Any]] = []
    for eta_y in section["lateral_utilisation"]:
        for eta_x in section["longitudinal_utilisation"]:
            for direction in section["directions"]:
                for rep in range(1, int(section["repetitions"]) + 1):
                    conditions.append({
                        "stage": "combined_slip",
                        "lateral_utilisation": float(eta_y),
                        "longitudinal_utilisation": float(eta_x),
                        "direction": direction,
                        "repetition": rep,
                    })
    return conditions


def limit_lap_blocks(config: dict[str, Any]) -> dict[str, int]:
    return {str(name): int(count) for name, count in config["limit_laps"]["blocks"].items()}


def design_summary(config: dict[str, Any]) -> dict[str, Any]:
    steady = steady_lateral_conditions(config)
    transients = steering_transient_conditions(config)
    combined = combined_slip_conditions(config)
    laps = limit_lap_blocks(config)
    transient_lengths = [c["post_stable_required_m"] for c in transients]
    steady_speeds = [c["target_speed_mps"] for c in steady]
    return {
        "steady_lateral_core_captures": len(steady),
        "steady_lateral_speed_range_mps": [min(steady_speeds), max(steady_speeds)],
        "steering_transient_captures": len(transients),
        "steering_transient_max_post_stable_required_m": max(transient_lengths),
        "combined_slip_captures": len(combined),
        "limit_lap_blocks": laps,
        "limit_lap_total": sum(laps.values()),
        "clear_pad_min_m": config["site"]["clear_pad_min_m"],
        "clear_pad_preferred_m": config["site"]["clear_pad_preferred_m"],
        "straight_runway_available_m": float(config["site"]["straight_runway_available_m"]),
    }


def validate_design(config: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    steady = config["steady_lateral"]
    site = config["site"]
    transients = config["steering_transients"]
    combined = config["combined_slip"]
    if steady_lateral_conditions(config) and len(steady_lateral_conditions(config)) != 32:
        errors.append("steady_lateral initial matrix must be 32 captures")
    if len(steering_transient_conditions(config)) != 36:
        errors.append("steering_transients matrix must be 36 captures")
    if len(combined_slip_conditions(config)) != 48:
        errors.append("combined_slip matrix must be 48 captures")
    if sum(limit_lap_blocks(config).values()) != 24:
        errors.append("limit_laps must total 24 diversified laps")
    if max(float(r) for r in steady["radii_m"]) < 3.0:
        errors.append("steady_lateral must include a 3.0 m radius")
    if list(site["clear_pad_min_m"]) != [9.0, 9.0]:
        errors.append("minimum clear pad must be 9 m x 9 m")
    if list(site["clear_pad_preferred_m"]) != [10.0, 10.0]:
        errors.append("preferred clear pad must be 10 m x 10 m")
    capture_sum = (
        float(transients["baseline_s"])
        + float(transients["first_step_s"])
        + float(transients["reverse_step_s"])
        + float(transients["recovery_s"])
    )
    if abs(capture_sum - float(transients["capture_s"])) > 1e-9:
        errors.append("steering_transients capture_s must equal baseline + first + reverse + recovery")
    max_required = max(c["post_stable_required_m"] for c in steering_transient_conditions(config))
    if max_required > float(site["post_stable_corridor_available_m"]) + 1e-9:
        errors.append(
            f"steering_transients need {max_required:.2f} m post-stable, "
            f"available {float(site['post_stable_corridor_available_m']):.2f} m"
        )
    if not bool(combined["pure_lateral_parameters_fixed_during_fit"]):
        errors.append("combined_slip must fit reduction while pure lateral parameters remain fixed")
    if not bool(config["limit_laps"]["hold_out_by_whole_lap"]):
        errors.append("limit_laps must hold out by whole lap")
    return errors
