/**
 * @file mpcc_vehicle_model.h
 * @brief MPCC Vehicle Dynamics Linearization
 *
 * Linearized vehicle model
 * for use in the MPCC QP problem construction.
 */

#ifndef MPCC_VEHICLE_MODEL_H
#define MPCC_VEHICLE_MODEL_H

#include "mpcc_types.h"
#include <math.h>

/**
 * Linearize the dynamics around (state, control).
 *
 * Produces discrete-time A, B, d such that:
 *   x_{k+1} = A * x_k + B * u_k + d
 *
 * @param state    Operating-point state [s, vx, vy, omega, X, Y, psi]
 * @param control  Operating-point control [delta, a_x, v_theta]
 * @param dt       Time step
 * @param cfg      MPCC configuration (tire params: C_Sf, C_Sr, mu)
 * @param sys      Output: linearized system (A, B, d)
 */
void mpcc_linearize_dynamics(
    const MPCCState_t *state,
    const MPCCControl_t *control,
    float dt,
    const MPCCConfiguration_t *cfg,
    MPCCLinearSystem_t *sys);

#endif /* MPCC_VEHICLE_MODEL_H */
