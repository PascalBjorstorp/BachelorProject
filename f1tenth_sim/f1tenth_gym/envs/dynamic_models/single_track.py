import numpy as np
from numba import njit
from numba.typed import Dict

from .utils import steering_constraint, accl_constraints


def _param(params: dict, key: str, fallback: float) -> float:
    value = params.get(key, fallback)
    return fallback if value is None else value


def _smoothstep_abs(value: float, start: float, end: float) -> float:
    mag = np.fabs(value)
    if end <= start:
        return 1.0 if mag >= end else 0.0
    if mag <= start:
        return 0.0
    if mag >= end:
        return 1.0
    t = (mag - start) / (end - start)
    return t * t * (3.0 - 2.0 * t)


def vehicle_dynamics_st(x: np.ndarray, u_init: np.ndarray, params: dict):
    """
    Single Track Vehicle Dynamics.
    From https://gitlab.lrz.de/tum-cps/commonroad-vehicle-models/-/blob/master/vehicleModels_commonRoad.pdf, section 7

        Args:
            x (numpy.ndarray (7, )): vehicle state vector (x0, x1, x2, x3, x4, x5, x6)
                x0: x position in global coordinates
                x1: y position in global coordinates
                x2: steering angle of front wheels
                x3: velocity in x direction
                x4:yaw angle
                x5: yaw rate
                x6: slip angle at vehicle center
            u (numpy.ndarray (2, )): control input vector (u1, u2)
                u1: steering angle velocity of front wheels
                u2: longitudinal acceleration
            params (dict): dictionary containing the following parameters:
                mu (float): friction coefficient
                C_Sf (float): cornering stiffness of front wheels
                C_Sr (float): cornering stiffness of rear wheels
                lf (float): distance from center of gravity to front axle
                lr (float): distance from center of gravity to rear axle
                h (float): height of center of gravity
                m (float): mass of vehicle
                I (float): moment of inertia of vehicle, about Z axis
                s_min (float): minimum steering angle
                s_max (float): maximum steering angle
                sv_min (float): minimum steering velocity
                sv_max (float): maximum steering velocity
                v_switch (float): velocity above which the acceleration is no longer able to create wheel spin
                a_max (float): maximum allowed acceleration
                v_min (float): minimum allowed velocity
                v_max (float): maximum allowed velocity

        Returns:
            f (numpy.ndarray): right hand side of differential equations
    """
    # States
    X = x[0]
    Y = x[1]
    DELTA = x[2]
    V = x[3]
    PSI = x[4]
    PSI_DOT = x[5]
    BETA = x[6]
    # We have to wrap the slip angle to [-pi, pi]
    # BETA = np.arctan2(np.sin(BETA), np.cos(BETA))

    # gravity constant m/s^2
    g = 9.81
    realistic_plant = bool(params.get("realistic_plant_enabled", False))

    # constraints
    u = np.array(
        [
            steering_constraint(
                DELTA,
                u_init[0],
                params["s_min"],
                params["s_max"],
                params["sv_min"],
                params["sv_max"],
            ),
            accl_constraints(
                V,
                u_init[1],
                params["v_switch"],
                params["a_max"],
                params["v_min"],
                params["v_max"],
            ),
        ]
    )
    # Controls
    STEER_VEL = u[0]
    ACCL = u[1]

    # switch to kinematic model for small velocities
    if V < 0.5:
        # wheelbase
        lwb = params["lf"] + params["lr"]
        steer_gain = _param(params, "steer_gain", 1.0) if realistic_plant else 1.0
        delta_eff = steer_gain * DELTA
        steer_vel_eff = steer_gain * STEER_VEL
        BETA_HAT = np.arctan(np.tan(delta_eff) * params["lr"] / lwb)
        BETA_DOT = (
            (1 / (1 + (np.tan(delta_eff) * (params["lr"] / lwb)) ** 2))
            * (params["lr"] / (lwb * np.cos(delta_eff) ** 2))
            * steer_vel_eff
        )
        f = np.array(
            [
                V * np.cos(PSI + BETA_HAT),  # X_DOT
                V * np.sin(PSI + BETA_HAT),  # Y_DOT
                STEER_VEL,  # DELTA_DOT
                ACCL,  # V_DOT
                V * np.cos(BETA_HAT) * np.tan(delta_eff) / lwb,  # PSI_DOT
                (1 / lwb)
                * (
                    ACCL * np.cos(BETA) * np.tan(delta_eff)
                    - V * np.sin(BETA) * np.tan(delta_eff) * BETA_DOT
                    + ((V * np.cos(BETA) * steer_vel_eff) / (np.cos(delta_eff) ** 2))
                ),  # PSI_DOT_DOT
                BETA_DOT,  # BETA_DOT
            ]
        )
    else:
        # Full dynamic model with atan-based slip angles and
        # cos(δ)/sin(δ) force resolution for real-world accuracy.
        #
        # Changes from the original CommonRoad model:
        #   1. Slip angles use atan instead of small-angle approximation
        #   2. Front tire force resolved with cos(δ)/sin(δ) for longitudinal/lateral coupling
        #   3. V_DOT computed from full body dynamics instead of just ACCL

        lwb = params["lf"] + params["lr"]

        # Decompose velocity into body-frame components
        vx = V * np.cos(BETA)
        vy = V * np.sin(BETA)
        vx_safe = max(vx, 0.5)  # numerical floor for division

        # Normal forces with longitudinal load transfer
        Fzf = params["m"] * (g * params["lr"] - ACCL * params["h"]) / lwb
        Fzr = params["m"] * (g * params["lf"] + ACCL * params["h"]) / lwb

        steer_gain = _param(params, "steer_gain", 1.0) if realistic_plant else 1.0
        steer_gain_high_slip = _param(params, "steer_gain_high_slip", steer_gain)
        slip_blend_start_front = _param(params, "slip_blend_start_front", 1.0e9)
        slip_blend_end_front = _param(params, "slip_blend_end_front", 1.0e9)
        alpha_f_raw = steer_gain * DELTA - np.arctan2(
            vy + params["lf"] * PSI_DOT, vx_safe
        )
        steer_blend = _smoothstep_abs(
            alpha_f_raw, slip_blend_start_front, slip_blend_end_front
        ) if realistic_plant else 0.0
        steer_gain_eff = steer_gain + (steer_gain_high_slip - steer_gain) * steer_blend
        delta_eff = steer_gain_eff * DELTA

        # Full atan-based slip angles
        alpha_f = delta_eff - np.arctan2(vy + params["lf"] * PSI_DOT, vx_safe)
        alpha_r = -np.arctan2(vy - params["lr"] * PSI_DOT, vx_safe)

        mu_front = _param(params, "mu_front", params["mu"])
        mu_rear = _param(params, "mu_rear", params["mu"])
        C_Sf_eff = params["C_Sf"]
        C_Sr_eff = params["C_Sr"]
        combined_scale = 1.0
        front_peak_scale = 1.0
        accel_usage = min(np.fabs(ACCL) / max(params["a_max"], 1.0e-6), 1.0)

        if realistic_plant:
            C_Sf_high = _param(params, "C_Sf_high_slip", C_Sf_eff)
            C_Sr_high = _param(params, "C_Sr_high_slip", C_Sr_eff)
            slip_blend_start_rear = _param(
                params, "slip_blend_start_rear", slip_blend_start_front
            )
            slip_blend_end_rear = _param(
                params, "slip_blend_end_rear", slip_blend_end_front
            )
            C_Sf_eff = C_Sf_eff + (C_Sf_high - C_Sf_eff) * _smoothstep_abs(
                alpha_f, slip_blend_start_front, slip_blend_end_front
            )
            C_Sr_eff = C_Sr_eff + (C_Sr_high - C_Sr_eff) * _smoothstep_abs(
                alpha_r, slip_blend_start_rear, slip_blend_end_rear
            )

            combined_gain = _param(params, "combined_slip_gain", 0.0)
            combined_scale = max(1.0 - combined_gain * accel_usage, 0.30)

            front_drop = _param(params, "front_peak_drop", 0.0)
            front_drop_start = _param(params, "front_peak_drop_start", 1.0e9)
            front_drop_end = _param(params, "front_peak_drop_end", 1.0e9)
            front_drop_pow = _param(params, "front_peak_drop_pow", 1.0)
            front_peak_floor = _param(params, "front_peak_floor", 0.30)
            front_drop_blend = _smoothstep_abs(alpha_f, front_drop_start, front_drop_end)
            if front_drop_pow > 1.0:
                front_drop_blend = front_drop_blend ** front_drop_pow
            front_peak_scale = max(1.0 - front_drop * front_drop_blend, front_peak_floor)

            front_combined_gain = _param(params, "front_combined_gain", 0.0)
            front_combined_scale = max(
                1.0 - front_combined_gain * accel_usage * front_drop_blend,
                front_peak_floor,
            )
            front_peak_scale *= front_combined_scale

        # Lateral tire forces
        if realistic_plant and bool(params.get("pacejka_tires_enabled", True)):
            pacejka_c_front = _param(
                params, "pacejka_c_front", _param(params, "pacejka_c", 1.9)
            )
            pacejka_c_rear = _param(
                params, "pacejka_c_rear", _param(params, "pacejka_c", 1.9)
            )
            B_f = C_Sf_eff / max(pacejka_c_front, 1.0e-6)
            B_r = C_Sr_eff / max(pacejka_c_rear, 1.0e-6)
            D_f = mu_front * Fzf * front_peak_scale
            D_r = mu_rear * Fzr
            Fyf = combined_scale * D_f * np.sin(
                pacejka_c_front * np.arctan(B_f * alpha_f)
            )
            Fyr = combined_scale * D_r * np.sin(
                pacejka_c_rear * np.arctan(B_r * alpha_r)
            )
        else:
            Fyf = (
                combined_scale * mu_front * front_peak_scale
                * C_Sf_eff * alpha_f * Fzf
            )
            Fyr = combined_scale * mu_rear * C_Sr_eff * alpha_r * Fzr

        # Longitudinal force from acceleration command
        Fx = params["m"] * ACCL
        if realistic_plant and bool(params.get("realistic_drive_enabled", True)):
            Fx -= _param(params, "roll_resistance_n", 0.0)
            v_abs = np.fabs(vx)
            a_drag = (
                _param(params, "drag_c0", 0.0)
                + _param(params, "drag_c1", 0.0) * v_abs
                + _param(params, "drag_c2", 0.0) * v_abs * v_abs
            )
            if a_drag > 0.0:
                Fx -= params["m"] * a_drag

        # cos/sin of steering angle for force resolution
        cos_delta = np.cos(delta_eff)
        sin_delta = np.sin(delta_eff)

        # Body dynamics with cos(δ)/sin(δ) force resolution
        # dvx/dt = (Fx - Fyf*sin(δ) + m*vy*ω) / m
        dvx_dt = (Fx - Fyf * sin_delta + params["m"] * vy * PSI_DOT) / params["m"]
        # dvy/dt = (Fyf*cos(δ) + Fyr - m*vx*ω) / m
        dvy_dt = (Fyf * cos_delta + Fyr - params["m"] * vx * PSI_DOT) / params["m"]
        # dω/dt = (lf*Fyf*cos(δ) - lr*Fyr) / Iz
        PSI_DOT_DOT = (params["lf"] * Fyf * cos_delta - params["lr"] * Fyr) / params["I"]

        # Convert body-frame accelerations back to (V, β) derivatives
        V_sq = max(V * V, 0.001)
        V_DOT = (vx * dvx_dt + vy * dvy_dt) / max(V, 0.001)
        BETA_DOT = (vx * dvy_dt - vy * dvx_dt) / V_sq

        f = np.array(
            [
                V * np.cos(PSI + BETA),  # X_DOT
                V * np.sin(PSI + BETA),  # Y_DOT
                STEER_VEL,  # DELTA_DOT
                V_DOT,  # V_DOT (full dynamics, not just ACCL)
                PSI_DOT,  # PSI_DOT
                PSI_DOT_DOT,  # PSI_DOT_DOT
                BETA_DOT,  # BETA_DOT
            ]
        )

    return f


@njit(cache=True)
def get_standardized_state_st(x: np.ndarray) -> dict:
    """[X,Y,DELTA,V_X, V_Y,YAW,YAW_RATE,SLIP]"""
    d = dict()
    d["x"] = x[0]
    d["y"] = x[1]
    d["delta"] = x[2]
    d["v_x"] = x[3] * np.cos(x[6])
    d["v_y"] = x[3] * np.sin(x[6])
    d["yaw"] = x[4]
    d["yaw_rate"] = x[5]
    d["slip"] = x[6]
    return d
