/**
 * @file mpc_riccati.c
 * @brief MPC Implementation using Riccati-ADMM (Non-Condensed Formulation)
 *
 * Alternative MPC controller that uses the Riccati-ADMM sparse solver
 * instead of the condensed QP approach. Key differences:
 *
 * 1. No Hessian condensing — state variables remain explicit decision
 *    variables, enabling O(N) per-iteration cost via Riccati recursion.
 *
 * 2. State augmentation: the Frenet state [e_y, e_psi, vx, vy, omega]
 *    is augmented with previous control [delta_prev, accel_prev] to
 *    handle rate penalty natively in the LQR cost structure.
 *
 * 3. Wall constraints are direct box constraints on x_k[0] (=e_y),
 *    handled naturally by ADMM's projection step.
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#include "mpc.h"
#include "fp_math.h"
#include "riccati_solver.h"
#include "vehicle_model.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/** Frenet state dimension (e_y, e_psi, vx, vy, omega) */
#define NX_FRENET 5

/** Augmented state dimension:
 *  [e_y, e_psi, vx, vy, omega, delta_actual, delta_rate_prev, accel_prev]
 *  States 0-5 form the "dense block" in the Riccati pass (6x6).
 *  States 6-7 are the "previous control" states (zero in A). */
#define NX_AUG 8

/** Index of delta_actual in the augmented state vector */
#define IDX_DELTA_ACTUAL 5

/** Index of delta_rate_prev in the augmented state vector */
#define IDX_DRATE_PREV 6

/** Index of accel_prev in the augmented state vector */
#define IDX_ACCEL_PREV 7

/** Dense block size in A matrix (Frenet + delta_actual) */
#define NX_DENSE 6

/** Control dimension (delta_rate, acceleration) */
#define NU 2

/** Maximum steering rate (rad/s) — servo physical limit */
#define MAX_STEERING_RATE FP_CONST(2.849)

/** Maximum horizon steps */
#define MAX_HORIZON 20

/** Big number for unconstrained states */
#define BIG_BOUND FP_CONST(100.0)

/** Minimum linearization velocity (same as condensed approach) */
#define MIN_LINEARIZATION_VELOCITY FP_CONST(2.0)

/** A-row stability limit (same as condensed approach) */
#define STABILITY_LIMIT FP_CONST(0.95)

/** Wall constraint margin (same as condensed approach) */
#define WALL_MARGIN FP_CONST(0.1)

/** Match PG's sparse wall constraints: start at step 2, every 3 steps */
#define WALL_CONSTRAINT_START  2
#define WALL_CONSTRAINT_STRIDE 3

/** v_switch: above this velocity, max acceleration = a_max * v_switch / v.
 *  From f1tenth gym STDynamicsModel: v_switch = 7.319 m/s.
 *  Models constant-power regime: P = F*v = const → a_max(v) ∝ 1/v. */
#define V_SWITCH FP_CONST(7.319)

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MpcConfiguration_t config;
static int initialized = 0;
static ControlInput_t prev_control;
static fixed_point_t actual_steering_angle = 0;  /* Servo physical position */
static RiccatiAdmmState_t admm_state;
static fixed_point_t warm_start_prev_curvature = 0;
static int prev_solver_converged = 0;  /* Track if previous solve converged */
static fixed_point_t prev_delta_cmd = 0;  /* Previous steer output for rate-limiting */

/** Maximum steering-command change per solve when ADMM didn't converge.
 *  At 200 Hz control rate, 0.02 rad/solve ≈ 4 rad/s — well above servo limit
 *  (2.849 rad/s) but prevents erratic jumps from non-converged solves.
 *  Data shows all high-speed wild turns are from status=1 solves. */
#define MAX_NONCONV_STEER_DELTA FP_CONST(0.005)

/*===========================================================================
 * Default Configuration
 *===========================================================================*/

MpcConfiguration_t get_default_configuration(void)
{
    MpcConfiguration_t cfg;

    cfg.prediction_horizon_steps = MPC_DEFAULT_PREDICTION_HORIZON;
    cfg.time_step_seconds = MPC_DEFAULT_TIME_STEP_SECONDS;

    /* State tracking weights (Frenet frame)
     * Tuning v3 — cornering-speed optimized via steering responsiveness:
     *   Q_hdg reduced 1500→800: diminishes anticipatory braking while still
     *         driving proactive steering into corners. The 47% cut changes the
     *         Q_hdg/Q_vel ratio from 500:1 to 53:1.
     *   Q_vel increased 3→15: creates strong incentive to maintain speed and
     *         re-accelerate after corners. 5× increase.
     *   w_vy reduced 30→25: allows slightly more sideslip in dynamic cornering.
     *   Q_lat, w_yaw_rate unchanged: these were already working for safety.
     *
     * Performance vs original (Q_hdg=1500, Q_vel=3):
     *   Q_hdg=1000 is the safe lower bound (Q_hdg<1000 crashes at hairpin).
     *   Q_vel=10: improved speed tracking vs original (3).
     *   w_vy=28, R_steer=0.35, w_jerk=3.0: stable agile steering.
     *   Tested: 0 collisions in 60s ROS2 sim, 67→68% time above 5 m/s.
     */
    cfg.weight_lateral_error    = FP_CONST(75.0);
    cfg.weight_heading_error    = FP_CONST(100.0);
    cfg.weight_velocity         = FP_CONST(6.0);
    cfg.weight_lateral_velocity = FP_CONST(60.0);
    cfg.weight_yaw_rate         = FP_CONST(5.0);

    /* Control effort weights */
    cfg.weight_steering_effort      = FP_CONST(0.35);
    cfg.weight_acceleration_effort  = FP_CONST(0.01);

    /* Control rate weights */
    cfg.weight_steering_rate        = FP_CONST(2.5);
    cfg.weight_acceleration_rate    = FP_CONST(0.01);

    /* Cross-call rate scale: ratio of control interval to prediction dt. */
    cfg.cross_call_rate_scale = FP_CONST(0.1);

    /* Solver parameters */
    cfg.maximum_solver_iterations = MPC_DEFAULT_MAXIMUM_ITERATIONS;
    cfg.solver_convergence_tolerance = MPC_DEFAULT_CONVERGENCE_TOLERANCE;

    /* Environment variable overrides for runtime tuning */
    {
        const char *env_val;
        if ((env_val = getenv("MPC_W_LAT_ERROR")) != NULL)
            cfg.weight_lateral_error = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_HEADING")) != NULL)
            cfg.weight_heading_error = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_VELOCITY")) != NULL)
            cfg.weight_velocity = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_LAT_VEL")) != NULL)
            cfg.weight_lateral_velocity = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_YAW_RATE")) != NULL)
            cfg.weight_yaw_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_STEER_RATE")) != NULL)
            cfg.weight_steering_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_STEER_EFFORT")) != NULL)
            cfg.weight_steering_effort = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_TORQUE_RATE")) != NULL)
            cfg.weight_acceleration_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_CROSS_CALL_SCALE")) != NULL)
            cfg.cross_call_rate_scale = DOUBLE_TO_FP(atof(env_val));
    }

    return cfg;
}

/*===========================================================================
 * Forward declarations for Riccati-ADMM specific functions
 *===========================================================================*/

static void mpc_riccati_initialize(void);
static void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg);
static MpcSolverStatus_t mpc_riccati_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result);
static void mpc_riccati_reset(void);

/*===========================================================================
 * Public API (mpc.h interface — delegates to Riccati-ADMM)
 *===========================================================================*/

void mpc_initialize(void)
{
    mpc_riccati_initialize();
}

void mpc_initialize_with_configuration(const MpcConfiguration_t *configuration)
{
    mpc_riccati_initialize_with_configuration(configuration);
}

MpcSolverStatus_t mpc_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result)
{
    return mpc_riccati_compute_optimal_control(
        current_frenet_state, reference_trajectory, result);
}

MpcConfiguration_t mpc_get_configuration(void)
{
    return config;
}

void mpc_set_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration) config = *configuration;
}

void mpc_reset(void)
{
    mpc_riccati_reset();
}

/*===========================================================================
 * Riccati-ADMM specific implementation
 *===========================================================================*/

static void mpc_riccati_initialize(void)
{
    config = get_default_configuration();
    vehicle_model_initialize();
    prev_control.steering_angle_radians = 0;
    prev_control.acceleration_meters_per_second_squared = 0;
    actual_steering_angle = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
    prev_solver_converged = 0;
    initialized = 1;
}

static void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg)
{
    config = cfg ? *cfg : get_default_configuration();
    vehicle_model_initialize();
    prev_control.steering_angle_radians = 0;
    prev_control.acceleration_meters_per_second_squared = 0;
    actual_steering_angle = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
    prev_solver_converged = 0;
    initialized = 1;
}

static void mpc_riccati_reset(void)
{
    prev_control.steering_angle_radians = 0;
    prev_control.acceleration_meters_per_second_squared = 0;
    actual_steering_angle = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
    prev_solver_converged = 0;
}

void mpc_set_actual_previous_control(const ControlInput_t *actual)
{
    if (actual) {
        /* For the 8-state formulation:
         * - prev_control stores the previous δ̇ (steering rate) and acceleration
         *   for the rate-of-rate penalty.
         * - actual_steering_angle stores the physical servo position for x0[5]. */
        actual_steering_angle = actual->steering_angle_radians;
        prev_control.acceleration_meters_per_second_squared =
            actual->acceleration_meters_per_second_squared;
    }
}

static MpcSolverStatus_t mpc_riccati_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result)
{
    if (!current_frenet_state || !reference_trajectory || !result) {
        if (result) result->solver_status = MPC_STATUS_ERROR;
        return MPC_STATUS_ERROR;
    }

    if (!initialized) mpc_riccati_initialize();

    int N = config.prediction_horizon_steps;
    if (N > MAX_HORIZON) N = MAX_HORIZON;

    /* ---------------------------------------------------------------
     * Step 1: Linearize vehicle model
     *
     * Linearize at δ=atan(L·κ): the kinematic feedforward steering.
     * This gives the most accurate model for corners (κ up to 0.72),
     * because the Pacejka tire forces and B matrix entries are evaluated
     * at the actual operating point rather than at δ=0.
     * At 200Hz control, the Riccati gains change slowly between calls,
     * so the B matrix sensitivity is not an issue.
     * --------------------------------------------------------------- */
    /* Set up linearization point */
    VehicleParameters_t vp = vehicle_model_get_parameters();
    fixed_point_t path_curvature0 = reference_trajectory[0].path_curvature_radians_per_meter;

    /* Feedforward steering: atan(L * κ), clamped to ±δ_max/2 */
    fixed_point_t delta_ff = fp_atan(fp_mul(vp.wheelbase_meters, path_curvature0));
    fixed_point_t delta_clamp = vp.maximum_steering_angle_radians >> 1;
    if (delta_ff > delta_clamp) delta_ff = delta_clamp;
    if (delta_ff < fp_neg(delta_clamp)) delta_ff = fp_neg(delta_clamp);

    ControlInput_t lin_control;
    lin_control.steering_angle_radians = delta_ff;
    lin_control.acceleration_meters_per_second_squared = 0;

    FrenetState_t lin_state = *current_frenet_state;
    if (lin_state.longitudinal_velocity_meters_per_second < MIN_LINEARIZATION_VELOCITY)
        lin_state.longitudinal_velocity_meters_per_second = MIN_LINEARIZATION_VELOCITY;

    /* Single-step Forward Euler linearization */
    fixed_point_t A_base[5][5];
    fixed_point_t B_base[5][2];

    vehicle_model_compute_frenet_linearization(
        &lin_state, &lin_control,
        config.time_step_seconds,
        path_curvature0,
        A_base, B_base);

    /* Stabilize fast dynamics (row 4 = omega) */
    {
        int row = 4;
        fixed_point_t abs_aii = fp_abs(A_base[row][row]);
        if (abs_aii > STABILITY_LIMIT) {
            fixed_point_t target = (A_base[row][row] < 0)
                ? fp_neg(STABILITY_LIMIT) : STABILITY_LIMIT;
            fixed_point_t num = fp_sub(target, FP_ONE);
            fixed_point_t den = fp_sub(A_base[row][row], FP_ONE);
            if (den != 0) {
                fixed_point_t scale = fp_div(num, den);
                for (int j = 0; j < 5; j++) {
                    if (j != row) A_base[row][j] = fp_mul(A_base[row][j], scale);
                }
                B_base[row][0] = fp_mul(B_base[row][0], scale);
                B_base[row][1] = fp_mul(B_base[row][1], scale);
            }
            A_base[row][row] = target;
        }
    }

    /* ---------------------------------------------------------------
     * Step 2: Build augmented per-step data (8-state servo formulation)
     *
     * Augmented state:
     *   x_aug = [e_y, e_psi, vx, vy, omega, δ_actual, δ̇_prev, a_prev]
     *
     * Control: u = [δ̇, a] (steering RATE and acceleration)
     *
     * The key insight: by making steering RATE the control input,
     * the servo rate limit (±2.849 rad/s) becomes a simple box
     * constraint on u[0], which ADMM handles natively.
     *
     * Augmented dynamics (8×8 A, 8×2 B):
     *   Rows 0-4: Frenet dynamics, using δ_actual (x[5]) for steering
     *             A[i][5] = B_frenet[i][0]  (steering effect through state)
     *             B[i][0] = 0               (δ̇ doesn't affect dynamics directly)
     *   Row 5:    δ_actual_{k+1} = δ_actual_k + dt * δ̇_k
     *             A[5][5] = 1, B[5][0] = dt
     *   Row 6:    δ̇_prev_{k+1} = δ̇_k
     *             B[6][0] = 1
     *   Row 7:    a_prev_{k+1} = a_k
     *             B[7][1] = 1
     *
     * Sparsity: rows/cols 6-7 of A are zero (same pattern as before,
     * just the dense block is 6×6 instead of 5×5).
     *
     * Cost structure:
     *   Q[0-4]:  Frenet tracking weights
     *   Q[5]:    Small weight on |δ_actual|² to prefer centered steering
     *   Q[6-7]:  Rate-of-rate weights (delta jerk + accel rate)
     *   R[0]:    Weight on |δ̇|² (steering rate effort)
     *   R[1]:    Weight on |a|² (acceleration effort + rate)
     *   N[6][0]: Steering jerk cross-cost (couples δ̇_prev with δ̇)
     *   N[7][1]: Acceleration rate cross-cost
     * --------------------------------------------------------------- */

    /* Precompute cost weights.
     *
     * In the 8-state formulation, the weight interpretation changes:
     * - w_steer_eff → effort on |δ̇|², penalizes fast steering
     * - w_steer_rate → jerk penalty on |δ̇_k - δ̇_{k-1}|²
     * - w_accel_eff → effort on |a|²
     * - w_accel_rate → rate penalty on |a_k - a_{k-1}|²
     */
    fixed_point_t w_steer_rate_eff = config.weight_steering_effort;   /* Penalizes δ̇ magnitude */
    fixed_point_t w_accel_eff = config.weight_acceleration_effort;
    fixed_point_t w_steer_jerk = config.weight_steering_rate;         /* Penalizes δ̇ change (jerk) */
    fixed_point_t w_accel_rate = config.weight_acceleration_rate;

    /* Small weight on δ_actual² to prefer centered steering */
    fixed_point_t w_delta_actual = FP_CONST(0.1);

    /* Build per-step data array */
    RiccatiStepData_t step_data[MAX_HORIZON];
    memset(step_data, 0, sizeof(step_data));

    for (int k = 0; k < N; k++) {
        RiccatiStepData_t *sd = &step_data[k];

        /* === Augmented A matrix (8×8) === */

        /* Top-left 5×5: Frenet A (dynamics of e_y, e_psi, vx, vy, omega) */
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
                sd->A[i][j] = A_base[i][j];

        /* A[1][2] varies with per-step curvature */
        sd->A[1][2] = fp_neg(fp_mul(config.time_step_seconds,
            reference_trajectory[k].path_curvature_radians_per_meter));

        /* Column 5 of A (rows 0-4): steering effect via δ_actual.
         * This is the old B_frenet[:,0] — steering no longer comes
         * through the control, it comes through the δ_actual state. */
        for (int i = 0; i < 5; i++)
            sd->A[i][IDX_DELTA_ACTUAL] = B_base[i][0];

        /* A[5][5] = 1: δ_actual integrator (δ_{k+1} = δ_k + dt*δ̇) */
        sd->A[IDX_DELTA_ACTUAL][IDX_DELTA_ACTUAL] = FP_ONE;

        /* Rows 6-7 of A: all zero (prev controls overwritten by B) */
        /* Cols 6-7 of A: all zero (prev controls don't affect dynamics) */
        /* Already zero from memset */

        /* === Augmented B matrix (8×2) === */

        /* Rows 0-4, col 0: ZERO — δ̇ doesn't directly affect Frenet dynamics */
        /* (steering effect goes through A[i][5] via δ_actual state) */
        /* Already zero from memset */

        /* Rows 0-4, col 1: acceleration effect on dynamics (unchanged) */
        for (int i = 0; i < 5; i++)
            sd->B[i][1] = B_base[i][1];

        /* Row 5: δ_actual integrator — B[5][0] = dt */
        sd->B[IDX_DELTA_ACTUAL][0] = config.time_step_seconds;

        /* Row 6: δ̇_prev_{k+1} = u[0] = δ̇ */
        sd->B[IDX_DRATE_PREV][0] = FP_ONE;

        /* Row 7: a_prev_{k+1} = u[1] = a */
        sd->B[IDX_ACCEL_PREV][1] = FP_ONE;

        /* === Q_diag (8 elements): state tracking weights === */
        sd->Q_diag[0] = fp_mul(FP_TWO, config.weight_lateral_error);
        sd->Q_diag[1] = fp_mul(FP_TWO, config.weight_heading_error);
        sd->Q_diag[2] = fp_mul(FP_TWO, config.weight_velocity);
        sd->Q_diag[3] = fp_mul(FP_TWO, config.weight_lateral_velocity);
        sd->Q_diag[4] = fp_mul(FP_TWO, config.weight_yaw_rate);
        sd->Q_diag[IDX_DELTA_ACTUAL] = fp_mul(FP_TWO, w_delta_actual);
        sd->Q_diag[IDX_DRATE_PREV] = fp_mul(FP_TWO, w_steer_jerk);
        sd->Q_diag[IDX_ACCEL_PREV] = fp_mul(FP_TWO, w_accel_rate);

        /* Apply cross-call scaling for step 0 (jerk/rate penalties) */
        if (k == 0) {
            sd->Q_diag[IDX_DRATE_PREV] = fp_mul(FP_TWO, fp_mul(w_steer_jerk, config.cross_call_rate_scale));
            sd->Q_diag[IDX_ACCEL_PREV] = fp_mul(FP_TWO, fp_mul(w_accel_rate, config.cross_call_rate_scale));
        }

        /* === q (8 elements): linear state cost (tracking references) === */
        sd->q[0] = fp_neg(fp_mul(sd->Q_diag[0], reference_trajectory[k].reference_lateral_error_meters));
        sd->q[1] = fp_neg(fp_mul(sd->Q_diag[1], reference_trajectory[k].reference_heading_error_radians));
        sd->q[2] = fp_neg(fp_mul(sd->Q_diag[2], reference_trajectory[k].reference_velocity_meters_per_second));
        sd->q[3] = fp_neg(fp_mul(sd->Q_diag[3], reference_trajectory[k].reference_lateral_velocity_meters_per_second));
        sd->q[4] = fp_neg(fp_mul(sd->Q_diag[4], reference_trajectory[k].reference_yaw_rate_radians_per_second));

        /* δ_actual reference: feedforward steering δ_ff = atan(L*κ) */
        {
            fixed_point_t kappa_k = reference_trajectory[k].path_curvature_radians_per_meter;
            fixed_point_t delta_ff_k = fp_atan(fp_mul(vp.wheelbase_meters, kappa_k));
            sd->q[IDX_DELTA_ACTUAL] = fp_neg(fp_mul(sd->Q_diag[IDX_DELTA_ACTUAL], delta_ff_k));
        }
        sd->q[IDX_DRATE_PREV] = 0;  /* No tracking ref for δ̇_prev */
        sd->q[IDX_ACCEL_PREV] = 0;  /* No tracking ref for a_prev */

        /* === R_diag (2 elements): control cost === */
        /* R[0]: weight on |δ̇|² = effort + jerk penalty */
        sd->R_diag[0] = fp_mul(FP_TWO, fp_add(w_steer_rate_eff, w_steer_jerk));
        /* R[1]: weight on |a|² = effort + rate penalty */
        sd->R_diag[1] = fp_mul(FP_TWO, fp_add(w_accel_eff, w_accel_rate));

        if (k == 0) {
            sd->R_diag[0] = fp_mul(FP_TWO, fp_add(w_steer_rate_eff,
                fp_mul(w_steer_jerk, config.cross_call_rate_scale)));
            sd->R_diag[1] = fp_mul(FP_TWO, fp_add(w_accel_eff,
                fp_mul(w_accel_rate, config.cross_call_rate_scale)));
        }

        /* r: no constant control bias */
        sd->r[0] = 0;
        sd->r[1] = 0;

        /* === Cross-cost N (8×2) === */
        /* N[6][0]: couples δ̇_prev (x[6]) with δ̇ (u[0]) — steering jerk */
        sd->N[IDX_DRATE_PREV][0] = fp_neg(fp_mul(FP_TWO, w_steer_jerk));
        /* N[7][1]: couples a_prev (x[7]) with a (u[1]) — accel rate */
        sd->N[IDX_ACCEL_PREV][1] = fp_neg(fp_mul(FP_TWO, w_accel_rate));

        if (k == 0) {
            sd->N[IDX_DRATE_PREV][0] = fp_neg(fp_mul(FP_TWO, fp_mul(w_steer_jerk, config.cross_call_rate_scale)));
            sd->N[IDX_ACCEL_PREV][1] = fp_neg(fp_mul(FP_TWO, fp_mul(w_accel_rate, config.cross_call_rate_scale)));
        }

        /* === State bounds (8 elements) === */

        /* e_y: wall constraints (sparse: every 3rd step) */
        int wall_active = (k >= WALL_CONSTRAINT_START) &&
                          ((k - WALL_CONSTRAINT_START) % WALL_CONSTRAINT_STRIDE == 0);

        if (wall_active &&
            reference_trajectory[k].left_wall_bound_meters < FP_CONST(4.0) &&
            reference_trajectory[k].right_wall_bound_meters < FP_CONST(4.0)) {
            sd->x_lb[0] = fp_neg(fp_sub(reference_trajectory[k].right_wall_bound_meters, WALL_MARGIN));
            sd->x_ub[0] = fp_sub(reference_trajectory[k].left_wall_bound_meters, WALL_MARGIN);
        } else {
            sd->x_lb[0] = -BIG_BOUND;
            sd->x_ub[0] = BIG_BOUND;
        }

        /* States 1-4 (e_psi, vx, vy, omega): unconstrained */
        for (int s = 1; s < 5; s++) {
            sd->x_lb[s] = -BIG_BOUND;
            sd->x_ub[s] = BIG_BOUND;
        }

        /* State 5 (δ_actual): physical steering angle limit */
        sd->x_lb[IDX_DELTA_ACTUAL] = fp_neg(vp.maximum_steering_angle_radians);
        sd->x_ub[IDX_DELTA_ACTUAL] = vp.maximum_steering_angle_radians;

        /* States 6-7 (prev controls): unconstrained */
        sd->x_lb[IDX_DRATE_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_DRATE_PREV] = BIG_BOUND;
        sd->x_lb[IDX_ACCEL_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_ACCEL_PREV] = BIG_BOUND;

        /* === Control bounds === */
        /* u[0] = δ̇: steering RATE limit (the key benefit of 8-state!) */
        sd->u_lb[0] = fp_neg(MAX_STEERING_RATE);
        sd->u_ub[0] = MAX_STEERING_RATE;

        /* u[1] = acceleration: speed-dependent power limit (v_switch model).
         * Above v_switch=7.319 m/s, the motor power is constant, so:
         *   a_max_eff = a_max * v_switch / v
         * This matches the f1tenth gym's STDynamicsModel exactly. */
        {
            fixed_point_t v_ref_k = reference_trajectory[k].reference_velocity_meters_per_second;
            fixed_point_t a_max = vp.maximum_acceleration_meters_per_second_squared;
            fixed_point_t a_min = vp.minimum_acceleration_meters_per_second_squared;

            if (v_ref_k > V_SWITCH) {
                /* a_max_eff = a_max * v_switch / v_ref */
                fixed_point_t scale = fp_div(V_SWITCH, v_ref_k);
                sd->u_ub[1] = fp_mul(a_max, scale);
                sd->u_lb[1] = fp_mul(a_min, scale);  /* a_min is negative */
            } else {
                sd->u_ub[1] = a_max;
                sd->u_lb[1] = a_min;
            }
        }
    }

    /* Terminal state cost */
    fixed_point_t terminal_Q[RICCATI_MAX_NX];
    fixed_point_t terminal_q[RICCATI_MAX_NX];
    memset(terminal_Q, 0, sizeof(terminal_Q));
    memset(terminal_q, 0, sizeof(terminal_q));

    terminal_Q[0] = fp_mul(FP_TWO, config.weight_lateral_error);
    terminal_Q[1] = fp_mul(FP_TWO, config.weight_heading_error);
    terminal_Q[2] = fp_mul(FP_TWO, config.weight_velocity);
    terminal_Q[3] = fp_mul(FP_TWO, config.weight_lateral_velocity);
    terminal_Q[4] = fp_mul(FP_TWO, config.weight_yaw_rate);
    terminal_Q[IDX_DELTA_ACTUAL] = fp_mul(FP_TWO, w_delta_actual);
    /* No jerk/rate penalty on terminal prev controls (Q[6:7] = 0) */

    /* Terminal q: tracking at last reference */
    if (N > 0) {
        terminal_q[0] = -fp_mul(terminal_Q[0], reference_trajectory[N-1].reference_lateral_error_meters);
        terminal_q[1] = -fp_mul(terminal_Q[1], reference_trajectory[N-1].reference_heading_error_radians);
        terminal_q[2] = -fp_mul(terminal_Q[2], reference_trajectory[N-1].reference_velocity_meters_per_second);
        terminal_q[3] = -fp_mul(terminal_Q[3], reference_trajectory[N-1].reference_lateral_velocity_meters_per_second);
        terminal_q[4] = -fp_mul(terminal_Q[4], reference_trajectory[N-1].reference_yaw_rate_radians_per_second);
        /* δ_actual terminal: track feedforward */
        {
            fixed_point_t kappa_N = reference_trajectory[N-1].path_curvature_radians_per_meter;
            fixed_point_t delta_ff_N = fp_atan(fp_mul(vp.wheelbase_meters, kappa_N));
            terminal_q[IDX_DELTA_ACTUAL] = -fp_mul(terminal_Q[IDX_DELTA_ACTUAL], delta_ff_N);
        }
    }

    /* ---------------------------------------------------------------
     * Step 3: Build augmented initial state (8 elements)
     * --------------------------------------------------------------- */
    fixed_point_t x0[RICCATI_MAX_NX];
    memset(x0, 0, sizeof(x0));
    x0[0] = current_frenet_state->lateral_error_meters;
    x0[1] = current_frenet_state->heading_error_radians;
    x0[2] = current_frenet_state->longitudinal_velocity_meters_per_second;
    x0[3] = current_frenet_state->lateral_velocity_meters_per_second;
    x0[4] = current_frenet_state->yaw_rate_radians_per_second;
    x0[IDX_DELTA_ACTUAL] = actual_steering_angle;      /* Physical servo position */
    x0[IDX_DRATE_PREV] = prev_control.steering_angle_radians;  /* Previous δ̇ command */
    x0[IDX_ACCEL_PREV] = prev_control.acceleration_meters_per_second_squared;

    /* ---------------------------------------------------------------
     * Step 4: Warm-start management
     * --------------------------------------------------------------- */
    fixed_point_t cur_curvature = reference_trajectory[0].path_curvature_radians_per_meter;
    /* Convergence-conditioned warm-start: reuse ADMM state only when the
     * previous call converged AND curvature hasn't changed drastically.
     * Non-converged dual variables cause cascading failures (steering
     * ramps to zero while accelerating into walls). */
    {
        fixed_point_t kappa_diff = fp_abs(fp_sub(cur_curvature, warm_start_prev_curvature));
        if (!prev_solver_converged || !admm_state.initialized || kappa_diff > FP_CONST(0.3)) {
            riccati_admm_state_init(&admm_state);
        }
    }
    warm_start_prev_curvature = cur_curvature;

    /* ---------------------------------------------------------------
     * Step 5: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */
    RiccatiAdmmConfig_t solver_config;
    riccati_admm_config_init(&solver_config);
    solver_config.max_iterations = (int)config.maximum_solver_iterations;

    RiccatiSolution_t riccati_sol;
    memset(&riccati_sol, 0, sizeof(riccati_sol));

    RiccatiStatus_t rstatus = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, x0,
        NX_AUG, NU, N,
        &solver_config, &admm_state, &riccati_sol);

    /* ---------------------------------------------------------------
     * Step 6: Extract control output
     *
     * The ADMM z_u gives [δ̇, a] (steering rate, acceleration).
     * We need to convert δ̇ back to a steering angle command:
     *   δ_cmd = δ_actual + dt_call × δ̇
     * where dt_call is the control interval (5ms at 200Hz).
     *
     * For the test (which calls at prediction dt = 50ms), we use:
     *   δ_cmd = δ_actual + dt_pred × δ̇  (= predicted δ_actual at step 1)
     *
     * In practice, the caller should decide dt_call based on their
     * actual call rate. For maximum generality, we output the predicted
     * δ_actual at the end of the first prediction step:
     *   δ_cmd = x_predicted[1][5] = z_x[1][5]
     *
     * This is equivalent to δ_actual + dt_pred * δ̇ (from the integrator).
     * --------------------------------------------------------------- */
    fixed_point_t delta_rate = admm_state.z_u[0][0];
    fixed_point_t accel = admm_state.z_u[0][1];

    /* Compute steering angle command: δ_actual + dt * δ̇ */
    fixed_point_t delta_cmd = fp_add(actual_steering_angle,
        fp_mul(config.time_step_seconds, delta_rate));

    /* Clamp to physical steering limits */
    if (delta_cmd > vp.maximum_steering_angle_radians)
        delta_cmd = vp.maximum_steering_angle_radians;
    if (delta_cmd < fp_neg(vp.maximum_steering_angle_radians))
        delta_cmd = fp_neg(vp.maximum_steering_angle_radians);

    /* Rate-limit steering output for non-converged solves.
     * Log analysis shows ALL high-speed steering jumps correlate with
     * status=1 (non-converged) solves. The solver produces erratic δ̇
     * when it doesn't converge, causing visible jerks at high speed.
     * Clamping the output change to ±0.02 rad prevents wild turns
     * while preserving normal cornering behavior. */
    if (rstatus != RICCATI_STATUS_OPTIMAL) {
        fixed_point_t delta_change = fp_sub(delta_cmd, prev_delta_cmd);
        if (delta_change > MAX_NONCONV_STEER_DELTA)
            delta_cmd = fp_add(prev_delta_cmd, MAX_NONCONV_STEER_DELTA);
        else if (delta_change < fp_neg(MAX_NONCONV_STEER_DELTA))
            delta_cmd = fp_sub(prev_delta_cmd, MAX_NONCONV_STEER_DELTA);
    }
    prev_delta_cmd = delta_cmd;

    ControlInput_t raw_control;
    raw_control.steering_angle_radians = delta_cmd;
    raw_control.acceleration_meters_per_second_squared = accel;

    ControlInput_t saturated = vehicle_model_saturate_control(&raw_control);

    result->optimal_control = saturated;
    result->iterations_used = (uint16_t)riccati_sol.iterations;
    result->final_cost = riccati_sol.primal_residual;

    switch (rstatus) {
    case RICCATI_STATUS_OPTIMAL:
        result->solver_status = MPC_STATUS_SUCCESS;
        prev_solver_converged = 1;
        break;
    case RICCATI_STATUS_MAX_ITERATIONS:
        result->solver_status = MPC_STATUS_MAXIMUM_ITERATIONS_REACHED;
        prev_solver_converged = 0;
        break;
    default:
        result->solver_status = MPC_STATUS_ERROR;
        prev_solver_converged = 0;
        break;
    }

    /* Save previous control for rate-of-rate penalty.
     * Store the steering rate (δ̇) as the "steering" value, and accel as-is. */
    prev_control.steering_angle_radians = delta_rate;
    prev_control.acceleration_meters_per_second_squared = saturated.acceleration_meters_per_second_squared;
    return result->solver_status;
}
