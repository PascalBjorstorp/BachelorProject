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
 * F1/10th Default Vehicle Parameters (from f1tenth_gym)
 *===========================================================================*/

/* Geometry */
#define F110_WHEELBASE      FP_CONST(0.3302)    /**< lf + lr = 0.15875 + 0.17145 [m] */
#define F110_LF             FP_CONST(0.15875)   /**< CG to front axle [m] */
#define F110_LR             FP_CONST(0.17145)   /**< CG to rear axle [m] */
#define F110_WIDTH          FP_CONST(0.31)      /**< Vehicle width [m] */
#define F110_LENGTH         FP_CONST(0.58)      /**< Vehicle length [m] */

/* Steering constraints */
#define F110_MAX_STEER      FP_CONST(0.4189)    /**< s_max [rad] (~24°) */
#define F110_MIN_STEER      FP_CONST(-0.4189)   /**< s_min [rad] (~-24°) */
#define F110_MAX_STEER_VEL  FP_CONST(3.2)       /**< sv_max [rad/s] */
#define F110_MIN_STEER_VEL  FP_CONST(-3.2)      /**< sv_min [rad/s] */

/* Velocity constraints */
#define F110_MAX_VEL        FP_CONST(20.0)      /**< v_max [m/s] */
#define F110_MIN_VEL        FP_CONST(-5.0)      /**< v_min [m/s] */
#define F110_V_SWITCH       FP_CONST(7.319)     /**< Velocity for accel limit change [m/s] */

/* Acceleration constraints */
#define F110_MAX_ACCEL      FP_CONST(9.51)      /**< a_max [m/s²] */

/* Mass/Inertia (for dynamic model) */
#define F110_MASS           FP_CONST(3.74)      /**< m [kg] */
#define F110_INERTIA        FP_CONST(0.04712)   /**< I_z [kg·m²] */
#define F110_CG_HEIGHT      FP_CONST(0.074)     /**< h [m] */

/* Tire parameters (for dynamic model) */
#define F110_MU             FP_CONST(1.0489)    /**< Friction coefficient */
#define F110_C_SF           FP_CONST(4.718)     /**< Front cornering stiffness */
#define F110_C_SR           FP_CONST(5.4562)    /**< Rear cornering stiffness */

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
