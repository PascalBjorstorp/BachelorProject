import numpy as np
from numba import njit
from numba.typed import Dict

from .utils import steering_constraint, accl_constraints


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
        BETA_HAT = np.arctan(np.tan(DELTA) * params["lr"] / lwb)
        BETA_DOT = (
            (1 / (1 + (np.tan(DELTA) * (params["lr"] / lwb)) ** 2))
            * (params["lr"] / (lwb * np.cos(DELTA) ** 2))
            * STEER_VEL
        )
        f = np.array(
            [
                V * np.cos(PSI + BETA_HAT),  # X_DOT
                V * np.sin(PSI + BETA_HAT),  # Y_DOT
                STEER_VEL,  # DELTA_DOT
                ACCL,  # V_DOT
                V * np.cos(BETA_HAT) * np.tan(DELTA) / lwb,  # PSI_DOT
                (1 / lwb)
                * (
                    ACCL * np.cos(BETA) * np.tan(DELTA)
                    - V * np.sin(BETA) * np.tan(DELTA) * BETA_DOT
                    + ((V * np.cos(BETA) * STEER_VEL) / (np.cos(DELTA) ** 2))
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

        # Full atan-based slip angles
        alpha_f = DELTA - np.arctan2(vy + params["lf"] * PSI_DOT, vx_safe)
        alpha_r = -np.arctan2(vy - params["lr"] * PSI_DOT, vx_safe)

        # Lateral tire forces (linear cornering stiffness × atan slip angles)
        Fyf = params["mu"] * params["C_Sf"] * alpha_f * Fzf
        Fyr = params["mu"] * params["C_Sr"] * alpha_r * Fzr

        # Longitudinal force from acceleration command
        Fx = params["m"] * ACCL

        # cos/sin of steering angle for force resolution
        cos_delta = np.cos(DELTA)
        sin_delta = np.sin(DELTA)

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
