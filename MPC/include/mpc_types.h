/**
 * @file mpc_types.h
 * @brief Type Definitions for MPC System
 *
 * All numerical values use Q16.16 fixed-point for FPGA compatibility.
 * Units: SI (meters, radians, seconds)
 */

#ifndef MPC_TYPES_H
#define MPC_TYPES_H

#include "fp_math.h"
#include <stdint.h>

/*===========================================================================
 * Shared Solver Status (used by QP and MPC)
 *===========================================================================*/

typedef enum
{
    SOLVER_SUCCESS = 0,          /**< Optimal solution found */
    SOLVER_MAX_ITER = 1,         /**< Max iterations reached */
    SOLVER_INFEASIBLE = 2,       /**< No feasible solution */
    SOLVER_ERROR = 3             /**< Solver error */
} SolverStatus_t;

/*===========================================================================
 * Vehicle State [INPUT to MPC]
 *===========================================================================*/

typedef struct
{
    fixed_point_t x;             /**< X position [m] */
    fixed_point_t y;             /**< Y position [m] */
    fixed_point_t heading;       /**< Heading angle [rad] */
    fixed_point_t vel;           /**< Velocity [m/s] */
} VehicleState_t;

/*===========================================================================
 * Control Input [OUTPUT from MPC]
 *===========================================================================*/

typedef struct
{
    fixed_point_t steer;         /**< Steering angle [rad] */
    fixed_point_t vel;           /**< Commanded velocity [m/s] */
} ControlInput_t;

/*===========================================================================
 * Vehicle Parameters
 *===========================================================================*/

typedef struct
{
    /* Geometry */
    fixed_point_t wheelbase;     /**< Wheelbase [m] */
    
    /* Steering constraints */
    fixed_point_t max_steer;     /**< Max steering angle [rad] */
    fixed_point_t max_steer_vel; /**< Max steering velocity [rad/s] */
    
    /* Velocity constraints */
    fixed_point_t max_vel;       /**< Max velocity [m/s] */
    fixed_point_t min_vel;       /**< Min velocity [m/s] */
    
    /* Acceleration constraints */
    fixed_point_t max_accel;     /**< Max acceleration [m/s²] */
} VehicleParams_t;

/*===========================================================================
 * MPC Configuration
 *===========================================================================*/

typedef struct
{
    uint16_t horizon;            /**< Prediction horizon steps */
    fixed_point_t dt;            /**< Time step [s] */

    /* State tracking weights */
    fixed_point_t w_x;           /**< Weight: X position */
    fixed_point_t w_y;           /**< Weight: Y position */
    fixed_point_t w_heading;     /**< Weight: heading */
    fixed_point_t w_vel;         /**< Weight: velocity */

    /* Control effort weights */
    fixed_point_t w_steer;       /**< Weight: steering effort */
    fixed_point_t w_vel_cmd;     /**< Weight: velocity command */

    /* Control rate weights */
    fixed_point_t w_steer_rate;  /**< Weight: steering rate */
    fixed_point_t w_vel_rate;    /**< Weight: velocity rate */

    /* Solver params */
    uint16_t max_iter;           /**< Max solver iterations */
    fixed_point_t tolerance;     /**< Convergence tolerance */
} MpcConfig_t;

/*===========================================================================
 * Trajectory Reference Point
 *===========================================================================*/

typedef struct
{
    fixed_point_t x;             /**< Reference X [m] */
    fixed_point_t y;             /**< Reference Y [m] */
    fixed_point_t heading;       /**< Reference heading [rad] */
    fixed_point_t vel;           /**< Reference velocity [m/s] */
    fixed_point_t left_bound;    /**< Max distance to left (outer) wall [m] */
    fixed_point_t right_bound;   /**< Max distance to right (inner) wall [m] */
} TrajectoryPoint_t;

/*===========================================================================
 * MPC Solver Result
 *===========================================================================*/

typedef struct
{
    SolverStatus_t status;       /**< Solver termination status */
    ControlInput_t control;      /**< Optimal control */
    uint16_t iterations;         /**< Iterations used */
    fixed_point_t cost;          /**< Final cost value */
} MpcResult_t;

/*===========================================================================
 * Default MPC Configuration Constants
 *===========================================================================*/

#define MPC_DEFAULT_HORIZON     20
#define MPC_DEFAULT_DT          FP_CONST(0.005)
#define MPC_DEFAULT_MAX_ITER    1000
#define MPC_DEFAULT_TOLERANCE   FP_CONST(0.001)

#endif /* MPC_TYPES_H */
