/**
 * @file vehicle_model.h
 * @brief Kinematic Bicycle Model for F1/10th Vehicle
 *
 * Model Equations (velocity is a direct control input):
 *   dx/dt       = v_cmd × cos(heading)
 *   dy/dt       = v_cmd × sin(heading)
 *   dheading/dt = (v_cmd / wheelbase) × tan(steer)
 *   v           = v_cmd
 *
 * All calculations use Q16.16 fixed-point for FPGA compatibility.
 */

#ifndef VEHICLE_MODEL_H
#define VEHICLE_MODEL_H

#include "mpc_types.h"
#include "fp_math.h"

/*===========================================================================
 * F1/10th Default Vehicle Parameters
 *===========================================================================*/

#define F110_WHEELBASE      FP_CONST(0.33)      /**< Wheelbase [m] */
#define F110_MAX_STEER      FP_CONST(0.4189)    /**< Max steering [rad] (~24°) */
#define F110_MAX_VEL        FP_CONST(6.0)       /**< Max velocity [m/s] */
#define F110_MIN_VEL        FP_CONST(0.0)       /**< Min velocity [m/s] */

/*===========================================================================
 * Model Initialization
 *===========================================================================*/

/** Initialize with default F1/10th parameters */
void vehicle_model_init(void);

/** Initialize with custom parameters */
void vehicle_model_init_params(const VehicleParams_t *params);

/** Get current parameters */
VehicleParams_t vehicle_model_get_params(void);

/*===========================================================================
 * Control Saturation
 *===========================================================================*/

/** Clamp control to physical limits */
ControlInput_t vehicle_model_saturate(const ControlInput_t *raw);

/*===========================================================================
 * State Prediction
 *===========================================================================*/

/** Predict next state (single step) */
VehicleState_t vehicle_model_predict(
    const VehicleState_t *state,
    const ControlInput_t *control,
    fixed_point_t dt);

/** Predict trajectory (multiple steps) */
void vehicle_model_predict_traj(
    const VehicleState_t *initial,
    const ControlInput_t *controls,
    fixed_point_t dt,
    uint16_t steps,
    VehicleState_t *trajectory);

/*===========================================================================
 * Model Linearization
 *===========================================================================*/

/** Compute A (4x4) and B (4x2) matrices at operating point */
void vehicle_model_linearize(
    const VehicleState_t *state,
    const ControlInput_t *control,
    fixed_point_t dt,
    fixed_point_t A[4][4],
    fixed_point_t B[4][2]);

#endif /* VEHICLE_MODEL_H */
