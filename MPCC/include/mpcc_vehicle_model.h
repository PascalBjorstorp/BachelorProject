/**
 * @file mpcc_vehicle_model.h
 * @brief MPCC Vehicle Dynamics Linearization
 *
 * Linearizes the Lifted ODE (Frenet + linear tire + Cartesian) vehicle model
 * for use in the MPCC QP problem construction.
 *
 * Separated from mpcc.c for modularity and easier HLS porting.
 */

#ifndef MPCC_VEHICLE_MODEL_H
#define MPCC_VEHICLE_MODEL_H

#include "mpcc_types.h"

/**
 * Linearize the Lifted ODE dynamics around (state, control).
 *
 * Produces discrete-time A, B, d such that:
 *   x_{k+1} = A * x_k + B * u_k + d
 *
 * @param state    Operating-point state [s, n, alpha, vx, vy, omega, X, Y, psi]
 * @param control  Operating-point control [delta, a_x]
 * @param kappa    Path curvature at current s
 * @param dt       Time step (Q16.16)
 * @param cfg      MPCC configuration (tire params: C_Sf, C_Sr, mu)
 * @param sys      Output: linearized system (A, B, d)
 */
void mpcc_linearize_dynamics(
    const MPCCState_t *state,
    const MPCCControl_t *control,
    fixed_point_t kappa,
    fixed_point_t dt,
    const MPCCConfiguration_t *cfg,
    MPCCLinearSystem_t *sys);

#endif /* MPCC_VEHICLE_MODEL_H */
