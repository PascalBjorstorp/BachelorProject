/**
 * @file mpc.c
 * @brief MPC implementation using Riccati-ADMM (non-condensed formulation).
 * @details Alternative MPC controller that uses the Riccati-ADMM sparse solver
 *          instead of the condensed QP approach. Key differences:
 *
 * 1. No Hessian condensing — state variables remain explicit decision
 *    variables, enabling O(N) per-iteration cost via Riccati recursion.
 *
 * 2. State augmentation: the Frenet state [e_y, e_psi, vx, vy, omega]
 *    is augmented with commanded/effective steering and previous controls.
 *
 * 3. Wall constraints are direct box constraints on x_k[0] (=e_y),
 *    handled naturally by ADMM's projection step.
 *
 * All arithmetic uses native float operations on CPU.
 * @dependencies mpc.h, util_math.h, riccati_solver.h, vehicle_model.h,
 *               mpc_types.h, <string.h>, <stdlib.h>
 */

#include "mpc.h"
#include "util_math.h"
#include "riccati_solver.h"
#include "vehicle_model.h"
#include "mpc_types.h"
#include "steering_dynamics.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

/*===========================================================================
 * Helper functions
 *===========================================================================*/

float get_env_float(const char *name, float default_val){
    const char *env = getenv(name);
    return env ? strtof(env, NULL) : default_val;
}

int get_env_int(const char *name, int default_val){
    const char *env = getenv(name);
    return env ? (int)strtol(env, NULL, 10) : default_val;
}

static float get_wall_bias_clearance_m(void)
{
    /* Extra clearance beyond config.wall_margin; applied by biasing the lateral
     * reference within the corridor. Set to 0 to disable. */
    return get_env_float("MPC_WALL_BIAS_CLEAR_M", 0.05f);
}

static float get_wall_bias_max_shift_m(void)
{
    /* Clamp max correction magnitude applied to base reference (safety valve). */
    return get_env_float("MPC_WALL_BIAS_MAX_M", 0.2f);
}

static float get_wall_ref_clearance_m(void)
{
    /* Desired minimum standoff of the *tracked reference* from each corridor
     * edge. Distinct from MPC_WALL_BIAS_CLEAR_M (which tightens the hard
     * constraint): this pulls the e_y reference inward so the car does not
     * aim to ride a raceline that hugs a wall. Critical where the planned
     * line sits ~0.3-0.5 m off the right wall. Set 0 to disable. */
    return get_env_float("MPC_WALL_REF_CLEAR_M", 0.10f);
}

static void compute_wall_ey_bounds(
    float left_wall_bound,
    float right_wall_bound,
    float wall_margin,
    float *out_x_lb,
    float *out_x_ub)
{
    /* Feasible e_y interval after subtracting wall_margin from both sides:
     *   e_y <= left_wall_bound  - wall_margin
     *   e_y >= -(right_wall_bound - wall_margin)
     */
    float x_lb = wall_margin - right_wall_bound;
    float x_ub = left_wall_bound - wall_margin;

    if (!isfinite(x_lb) || !isfinite(x_ub)) {
        x_lb = -BIG_BOUND;
        x_ub = BIG_BOUND;
    }

    /* If corridor is infeasible, collapse to the midpoint for solver stability. */
    if (x_lb > x_ub) {
        float mid = 0.5f * (x_lb + x_ub);
        x_lb = mid;
        x_ub = mid;
    }

    *out_x_lb = x_lb;
    *out_x_ub = x_ub;
}

static float compute_wall_biased_ey_ref(
    float base_ref,
    float x_lb,
    float x_ub,
    float clearance_m,
    float max_shift_m)
{
    if (!(max_shift_m > 0.0f) || !isfinite(max_shift_m))
        max_shift_m = BIG_BOUND;

    float target_lb = x_lb;
    float target_ub = x_ub;

    if (clearance_m > 0.0f && isfinite(clearance_m)) {
        /* When possible, keep at least clearance_m from both corridor edges. */
        const float desired_lb = x_lb + clearance_m;
        const float desired_ub = x_ub - clearance_m;
        if (desired_lb <= desired_ub) {
            target_lb = desired_lb;
            target_ub = desired_ub;
        }
    }

    /* Project reference to nearest feasible point, then limit correction size. */
    float target_ref = util_clamp(base_ref, target_lb, target_ub);
    float corr = target_ref - base_ref;
    corr = util_clamp(corr, -max_shift_m, max_shift_m);
    float ref = base_ref + corr;

    ref = util_clamp(ref, x_lb, x_ub);
    return ref;
}

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MpcConfiguration_t config;
static int initialized = 0;
static ControlInput_t prev_control;
static float commanded_steering_angle = 0.0f;
static float effective_steering_angle = 0.0f;
static int steering_command_initialized = 0;
static SteeringDynamicsCoefficients_t control_steering_dynamics;
static SteeringDynamicsCoefficients_t prediction_steering_dynamics;
static RiccatiAdmmState_t admm_state;
static float warm_start_prev_curvature = 0;
static int warm_start_prev_model_signature = MPC_MODEL_SIGNATURE;

static void refresh_steering_dynamics(void)
{
    control_steering_dynamics =
        steering_dynamics_coefficients(CONTROL_DT_SECONDS);
    prediction_steering_dynamics =
        steering_dynamics_coefficients(config.time_step);
}

static void reset_steering_state(void)
{
    commanded_steering_angle = 0.0f;
    effective_steering_angle = 0.0f;
    steering_command_initialized = 0;
}

static FrenetState_t mpc_predict_frenet_next_state(
    const FrenetState_t *state,
    const ControlInput_t *control,
    float dt,
    float path_curvature,
    float v_ref)
{
    FrenetState_t next = *state;
    const float vx = state->flong_vel;
    const float vy = state->flat_vel;
    const float omega = state->fyaw_rate;
    const float delta = control->steer_ang;
    const float a_cmd = control->long_acc;

    const float cos_delta = cosf(delta);
    const float sin_delta = sinf(delta);

    SlipTerms_t slip_terms;
    vehicle_model_compute_slip_terms(vx, vy, omega, delta, &slip_terms);

    const float Fx = VP_MASS_KG * a_cmd;

    float F_zf;
    float F_zr;
    vehicle_model_compute_normal_loads(Fx, &F_zf, &F_zr);

    float C_Sf_Fzf;
    float C_Sr_Fzr;
    float F_yf;
    float F_yr;
    vehicle_model_compute_effective_lateral_stiffness(
        1u,
        F_zf,
        slip_terms.alpha_front,
        &C_Sf_Fzf,
        &F_yf);
    vehicle_model_compute_effective_lateral_stiffness(
        0u,
        F_zr,
        slip_terms.alpha_rear,
        &C_Sr_Fzr,
        &F_yr);

    const float dvx_dt = (Fx - F_yf * sin_delta) * VP_INV_MASS_1_PER_KG + vy * omega;
    const float dvy_dt = (F_yf * cos_delta + F_yr) * VP_INV_MASS_1_PER_KG - vx * omega;
    const float domega_dt =
        (VP_CG_TO_FRONT_AXLE_M * F_yf * cos_delta - VP_CG_TO_REAR_AXLE_M * F_yr) *
        VP_INV_YAW_INERTIA_1_PER_KGM2;

    float ey_denom = 1.0f - path_curvature * state->flat_error;
    if (fabsf(ey_denom) < 1e-3f)
        ey_denom = (ey_denom >= 0.0f) ? 1e-3f : -1e-3f;

    /* Frenet error kinematics use the ACTUAL longitudinal speed, not the
     * reference speed: e_y_dot / e_psi_dot are geometric identities in the
     * true vehicle speed, so substituting v_ref injects a model-plant
     * mismatch on every accel/decel transient.
     *
     * Loss-of-rank safeguard: as vx -> 0 the (e_y, e_psi) subsystem becomes
     * uncontrollable (steering produces no lateral motion) and the Riccati
     * recursion degenerates. Flooring the speed at VP_MIN_VELOCITY_MPS keeps
     * the subsystem controllable at standstill — this is the regularization
     * the old v_ref substitution was really standing in for. */
    (void)v_ref;
    const float v_eff =
        (vx > VP_MIN_VELOCITY_MPS) ? vx : VP_MIN_VELOCITY_MPS;
    const float e_y_dot = v_eff * sinf(state->fhead_error) + vy * cosf(state->fhead_error);
    const float e_psi_dot = omega -
        path_curvature * v_eff * cosf(state->fhead_error) / ey_denom;

    next.flat_error = state->flat_error + dt * e_y_dot;
    next.fhead_error = util_normalize_angle(state->fhead_error + dt * e_psi_dot);
    next.flong_vel = util_clamp(vx + dt * dvx_dt, VP_MIN_VELOCITY_MPS, VP_MAX_VELOCITY_MPS);
    next.flat_vel = vy + dt * dvy_dt;
    next.fyaw_rate = omega + dt * domega_dt;

    return next;
}


/*===========================================================================
 * Default Configuration
 *===========================================================================*/

MpcConfiguration_t get_default_configuration(void)
{
    MpcConfiguration_t cfg = {
    .time_step = TIME_STEP_SECONDS,

    /* State tracking weights (Frenet frame). */
    .weight_lateral_error    = WEIGHT_LAT_ERROR,
    .weight_heading_error    = WEIGHT_HEADING,
    .weight_velocity         = WEIGHT_VELOCITY,
    .weight_lateral_velocity = WEIGHT_LAT_VEL,
    .weight_yaw_rate         = WEIGHT_YAW_RATE,

    /* Control effort weights */
    .weight_steering_effort      = WEIGHT_STEER_EFFORT,
    .weight_acceleration_effort  = WEIGHT_ACCEL_EFFORT,

    /* Control rate weights (W_JERK, W_ACCEL_RATE) */
    .weight_steering_rate        = WEIGHT_STEER_RATE,
    .weight_acceleration_rate    = WEIGHT_ACCEL_RATE,
    .weight_effective_steering   = WEIGHT_EFFECTIVE_STEERING,

    /* Cross-call rate scale computed from control and prediction periods. */
    .cross_call_rate_scale = CROSS_CALL_RATE_SCALE,
    .wall_margin = WALL_MARGIN,

    /* Solver parameters */
    .max_solver_iterations = MAX_ITERATIONS,
    .solver_convergence_tolerance = CONVERGENCE_TOLERANCE
    };


    /* Environment variable overrides */
    cfg.weight_lateral_error    = get_env_float("MPC_W_LAT_ERROR", cfg.weight_lateral_error);
    cfg.weight_heading_error    = get_env_float("MPC_W_HEADING", cfg.weight_heading_error);
    cfg.weight_velocity         = get_env_float("MPC_W_VELOCITY", cfg.weight_velocity);
    cfg.weight_lateral_velocity = get_env_float("MPC_W_LAT_VEL", cfg.weight_lateral_velocity);
    cfg.weight_yaw_rate         = get_env_float("MPC_W_YAW_RATE", cfg.weight_yaw_rate);

    cfg.weight_steering_effort     = get_env_float("MPC_W_STEER_EFFORT", cfg.weight_steering_effort);
    cfg.weight_acceleration_effort = get_env_float("MPC_W_ACCEL_EFFORT", cfg.weight_acceleration_effort);

    cfg.weight_steering_rate     = get_env_float("MPC_W_STEER_RATE", cfg.weight_steering_rate);
    cfg.weight_acceleration_rate = get_env_float("MPC_W_ACCEL_RATE", cfg.weight_acceleration_rate);
    cfg.weight_effective_steering = get_env_float(
        "MPC_W_EFFECTIVE_STEERING", cfg.weight_effective_steering);

    cfg.cross_call_rate_scale = get_env_float("MPC_CROSS_CALL_SCALE", cfg.cross_call_rate_scale);
    cfg.wall_margin = get_env_float("WALL_MARGIN", cfg.wall_margin);
    cfg.wall_margin = get_env_float("MPC_WALL_MARGIN", cfg.wall_margin);
    if (!(cfg.wall_margin >= 0.0f) || !isfinite(cfg.wall_margin))
        cfg.wall_margin = WALL_MARGIN;

    /* Horizon is fixed at compile-time to PREDICTION_HORIZON; env override removed. */

    float dt = get_env_float("PRED_DT", cfg.time_step);
    if (!(dt > 0.0f) || !isfinite(dt))
        dt = TIME_STEP_SECONDS;
    cfg.time_step = dt;

    /* Auto-update cross-call scaling if not explicitly set */
    if (getenv("MPC_CROSS_CALL_SCALE") == NULL)
        cfg.cross_call_rate_scale = CONTROL_DT_SECONDS / dt;

    return cfg;

}

/*===========================================================================
 * Public Riccati-ADMM API implementation
 *===========================================================================*/

/* Initialize MPC module state to defaults and prepare the vehicle model
 * and ADMM warm-start buffers for the first solve cycle. */
void mpc_initialize(void)
{
    config = get_default_configuration();
    prev_control.steer_ang = 0.0f;
    prev_control.long_acc = 0.0f;
    reset_steering_state();
    refresh_steering_dynamics();
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0.0f;
    warm_start_prev_model_signature = MPC_MODEL_SIGNATURE;
    initialized = 1;
}

/* Initialize MPC module with a caller-supplied configuration.
 * Falls back to default configuration when cfg is NULL. */
void mpc_initialize_with_configuration(const MpcConfiguration_t *cfg)
{
    config = cfg ? *cfg : get_default_configuration();
    if (!(config.time_step > 0.0f) || !isfinite(config.time_step))
        config.time_step = TIME_STEP_SECONDS;
    if (!(config.wall_margin >= 0.0f) || !isfinite(config.wall_margin))
        config.wall_margin = WALL_MARGIN;
    prev_control.steer_ang = 0.0f;
    prev_control.long_acc = 0.0f;
    reset_steering_state();
    refresh_steering_dynamics();
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0.0f;
    warm_start_prev_model_signature = MPC_MODEL_SIGNATURE;
    initialized = 1;
}

/* Reset all inter-cycle state (previous control, steering pole, warm-start)
 * without altering the active configuration or vehicle model parameters. */
void mpc_reset(void)
{
    prev_control.steer_ang = 0.0f;
    prev_control.long_acc = 0.0f;
    reset_steering_state();
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0.0f;
    warm_start_prev_model_signature = MPC_MODEL_SIGNATURE;
}

MpcConfiguration_t mpc_get_configuration(void)
{
    return config;
}

void mpc_set_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration) {
        config = *configuration;
        if (config.time_step <= 0.0f)
            config.time_step = TIME_STEP_SECONDS;
        if (!(config.wall_margin >= 0.0f) || !isfinite(config.wall_margin))
            config.wall_margin = WALL_MARGIN;
        refresh_steering_dynamics();
    }
}

void mpc_set_previous_command(const ControlInput_t *command)
{
    if (command) {
        float new_command = command->steer_ang;
        if (!isfinite(new_command))
            new_command = commanded_steering_angle;
        new_command = util_clamp(
            new_command, -VP_MAX_STEERING_RAD, VP_MAX_STEERING_RAD);

        if (!steering_command_initialized) {
            commanded_steering_angle = new_command;
            effective_steering_angle = new_command;
            prev_control.steer_ang = 0.0f;
            steering_command_initialized = 1;
        } else {
            float steer_rate =
                (new_command - commanded_steering_angle) / CONTROL_DT_SECONDS;
            if (steer_rate > STEERING_RATE_LIMIT)
                steer_rate = STEERING_RATE_LIMIT;
            if (steer_rate < -STEERING_RATE_LIMIT)
                steer_rate = -STEERING_RATE_LIMIT;
            prev_control.steer_ang = steer_rate;
            commanded_steering_angle = new_command;
        }

        if (isfinite(command->long_acc)) {
            prev_control.long_acc = util_clamp(
                command->long_acc, VP_MIN_ACCEL_MPS2, VP_MAX_ACCEL_MPS2);
        }
    }
}

int mpc_debug_copy_last_plan(
    float x_out[PREDICTION_HORIZON + 1][RICCATI_MAX_NX],
    float u_out[PREDICTION_HORIZON][RICCATI_MAX_NU])
{
    if (!admm_state.initialized) {
        return 0;
    }

    if (x_out != NULL) {
        memcpy(x_out, admm_state.z_x, sizeof(admm_state.z_x));
    }
    if (u_out != NULL) {
        memcpy(u_out, admm_state.z_u, sizeof(admm_state.z_u));
    }
    return 1;
}

MpcSolverStatus_t mpc_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result)
{
    // Validate inputs and module state.
    if (!current_frenet_state || !reference_trajectory || !result) {
        if (result) result->solver_status = MPC_STATUS_ERROR;
        return MPC_STATUS_ERROR;
    }

    // Auto-initialize on first use if not already initialized.
    if (!initialized) mpc_initialize();

    /* The command supplied before this call acted during the preceding 5 ms
     * control interval. Advance the effective-steering estimate exactly once
     * per MPC solve, independent of callback or simulation update frequency. */
    if (steering_command_initialized) {
        effective_steering_angle = steering_dynamics_next_effective(
            effective_steering_angle,
            commanded_steering_angle,
            0.0f,
            &control_steering_dynamics);
    }

    const FrenetState_t *frenet = current_frenet_state;

    // Horizon is fixed at compile-time.
    int N = PREDICTION_HORIZON;

    /* ---------------------------------------------------------------
     * Step 1: Prepare model constants for per-step linearization.
     *
     * Full Frenet linearization is computed per horizon point in the
     * loop below (A_step/B_step), using each step's curvature and
     * feedforward steering operating point.
     * --------------------------------------------------------------- */

    float delta_clamp = VP_MAX_STEERING_RAD * STEERING_FEEDFORWARD_CLAMP_FACTOR;

    const float wall_bias_clear_m = get_wall_bias_clearance_m();
    const float wall_bias_max_m = get_wall_bias_max_shift_m();
    /* Affine-bias ablation: 1.0 = baseline, 0.0 = no affine term. */
    float affine_scale = get_env_float("MPC_AFFINE_SCALE", 1.0f);
    if (!isfinite(affine_scale)) affine_scale = 1.0f;
    const float wall_ref_clear_m = get_wall_ref_clearance_m();
    int wall_bound_window = get_env_int("MPC_WALL_BOUND_WINDOW", 3);
    if (wall_bound_window < 0) wall_bound_window = 0;
    if (wall_bound_window > 25) wall_bound_window = 25;

    /* Warm-start / cold-start policy is evaluated after the horizon bounds are
     * built because it needs the first and terminal lateral-error boxes. */

    FrenetState_t lin_state = *frenet;
    if (lin_state.flong_vel < MIN_LINEARIZATION_VELOCITY)
        lin_state.flong_vel = MIN_LINEARIZATION_VELOCITY;

    /* Step 2: Build augmented dynamics, costs, and bounds over the horizon. */

    /* Build per-step data array */
    RiccatiStepData_t step_data[PREDICTION_HORIZON];
    /* Zero only sparse blocks that are not explicitly written later. */

    for (int k = 0; k < N; k++) {
        RiccatiStepData_t *sd = &step_data[k];

        /* --- Sparse zeroing (replaces memset) --- */
        for (int i = 0; i < NX_AUG; i++) {
            sd->A[IDX_DELTA_COMMAND][i] = 0;
            sd->A[IDX_DELTA_EFFECTIVE][i] = 0;
            sd->A[IDX_DRATE_PREV][i] = 0;
            sd->A[IDX_ACCEL_PREV][i] = 0;

            sd->N[i][0] = 0;
            sd->N[i][1] = 0;

            if (i < NX_DENSE) {
                sd->A[i][IDX_DRATE_PREV] = 0;
                sd->A[i][IDX_ACCEL_PREV] = 0;
            }
        }
        sd->B[IDX_DELTA_COMMAND][1] = 0;  /* Steering command integrator is not affected by accel. */
        sd->B[IDX_DELTA_EFFECTIVE][1] = 0; /* Effective-steering pole is not affected by accel. */
        sd->B[IDX_DRATE_PREV][1] = 0;    /* δ̇_prev not affected by accel */
        sd->B[IDX_ACCEL_PREV][0] = 0;    /* a_prev not affected by δ̇ */

        /* --- End sparse zeroing --- */

        float kappa_k = reference_trajectory[k].path_curvature;
        float v_state_for_limits = lin_state.flong_vel;

        ControlInput_t lin_control;
        lin_control.steer_ang = atanf(VP_WHEELBASE_M * kappa_k);
        if (lin_control.steer_ang > delta_clamp)
            lin_control.steer_ang = delta_clamp;
        if (lin_control.steer_ang < -delta_clamp)
            lin_control.steer_ang = -delta_clamp;
        lin_control.long_acc = prev_control.long_acc;

        float A_step[5][5];
        float B_step[5][2];

        vehicle_model_compute_frenet_linearization(
            &lin_state, &lin_control,
            config.time_step,
            kappa_k,
            reference_trajectory[k].reference_velocity,
            A_step, B_step);

        /* Stabilize fast dynamics (omega row = 4) per stage. */
        {
            int row = 4;  /* Yaw-rate row (omega) in the 5-state Frenet model */
            float abs_aii = fabsf(A_step[row][row]);
            if (abs_aii > STABILITY_LIMIT) {
                A_step[row][row] = (A_step[row][row] < 0)
                    ? -STABILITY_LIMIT : STABILITY_LIMIT;
            }
        }

        FrenetState_t lin_state_next = mpc_predict_frenet_next_state(
            &lin_state, &lin_control, config.time_step, kappa_k,
            reference_trajectory[k].reference_velocity);

        /* === Augmented A matrix (9x9) === */

        /* Top-left 5×5: per-step Frenet A (e_y, e_psi, vx, vy, omega) */
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
            sd->A[i][j] = A_step[i][j];

        /* The discrete vehicle Jacobian assumes constant steering over the
         * stage. Split that steering effect according to the exact average
         * effective angle generated by the command ramp and 25 ms pole. */
        for (int i = 0; i < NX_FRENET; i++) {
            const float steering_jacobian = B_step[i][0];
            sd->A[i][IDX_DELTA_COMMAND] =
                prediction_steering_dynamics.average_command_gain *
                steering_jacobian;
            sd->A[i][IDX_DELTA_EFFECTIVE] =
                prediction_steering_dynamics.average_effective_gain *
                steering_jacobian;
            sd->B[i][0] =
                prediction_steering_dynamics.average_rate_gain_seconds *
                steering_jacobian;
        }

        for (int i = 0; i < NX_AUG; i++)
            sd->d[i] = 0.0f;

        const float xbar[NX_FRENET] = {
            lin_state.flat_error,
            lin_state.fhead_error,
            lin_state.flong_vel,
            lin_state.flat_vel,
            lin_state.fyaw_rate,
        };
        const float xbar_next[NX_FRENET] = {
            lin_state_next.flat_error,
            lin_state_next.fhead_error,
            lin_state_next.flong_vel,
            lin_state_next.flat_vel,
            lin_state_next.fyaw_rate,
        };

        for (int i = 0; i < NX_FRENET; i++) {
            float lin_pred = 0.0f;
            for (int j = 0; j < NX_FRENET; j++) {
                lin_pred += A_step[i][j] * xbar[j];
            }
            lin_pred += B_step[i][0] * lin_control.steer_ang;
            lin_pred += B_step[i][1] * lin_control.long_acc;
            sd->d[i] = affine_scale * (xbar_next[i] - lin_pred);
        }

        /* Exact command-ramp/effective-steering transition. */
        sd->A[IDX_DELTA_COMMAND][IDX_DELTA_COMMAND] = 1.0f;
        sd->A[IDX_DELTA_EFFECTIVE][IDX_DELTA_COMMAND] =
            prediction_steering_dynamics.command_gain;
        sd->A[IDX_DELTA_EFFECTIVE][IDX_DELTA_EFFECTIVE] =
            prediction_steering_dynamics.retention;

        /* Previous-control tail rows and columns were zeroed above. */

        /* === Augmented B matrix (9x2) === */

        /* Rows 0-4, col 1: acceleration effect on dynamics */
        for (int i = 0; i < 5; i++)
            sd->B[i][1] = B_step[i][1];

        /* Command angle integrates the optimized steering rate. */
        sd->B[IDX_DELTA_COMMAND][0] = config.time_step;

        /* Effective steering responds to the command ramp within the stage. */
        sd->B[IDX_DELTA_EFFECTIVE][0] =
            prediction_steering_dynamics.rate_gain_seconds;

        /* Previous steering-rate state. */
        sd->B[IDX_DRATE_PREV][0] = 1.0f;

        /* Previous acceleration state. */
        sd->B[IDX_ACCEL_PREV][1] = 1.0f;

        /* === Q_diag (9 elements): state tracking weights === */
        sd->Q_diag[0] = RICCATI_COST_FACTOR * config.weight_lateral_error;
        sd->Q_diag[1] = RICCATI_COST_FACTOR * config.weight_heading_error;
        sd->Q_diag[2] = RICCATI_COST_FACTOR * config.weight_velocity;
        sd->Q_diag[3] = RICCATI_COST_FACTOR * config.weight_lateral_velocity;
        sd->Q_diag[4] = RICCATI_COST_FACTOR * config.weight_yaw_rate;
        sd->Q_diag[IDX_DELTA_COMMAND] = 0.0f;
        sd->Q_diag[IDX_DELTA_EFFECTIVE] =
            RICCATI_COST_FACTOR * config.weight_effective_steering;
        sd->Q_diag[IDX_DRATE_PREV] = RICCATI_COST_FACTOR * config.weight_steering_rate;
        sd->Q_diag[IDX_ACCEL_PREV] = RICCATI_COST_FACTOR * config.weight_acceleration_rate;

        /* Apply cross-call scaling for step 0 (jerk/rate penalties) */
        if (k == 0) {
            sd->Q_diag[IDX_DRATE_PREV] = RICCATI_COST_FACTOR * (config.weight_steering_rate * config.cross_call_rate_scale);
            sd->Q_diag[IDX_ACCEL_PREV] = RICCATI_COST_FACTOR * (config.weight_acceleration_rate * config.cross_call_rate_scale);
        }

        float wall_x_lb = -BIG_BOUND;
        float wall_x_ub = BIG_BOUND;
        float left_bound_k = reference_trajectory[k].left_wall_bound;
        float right_bound_k = reference_trajectory[k].right_wall_bound;
        if (wall_bound_window > 0) {
            int j0 = k - wall_bound_window;
            int j1 = k + wall_bound_window;
            if (j0 < 0) j0 = 0;
            if (j1 > (N - 1)) j1 = (N - 1);
            for (int j = j0; j <= j1; j++) {
                if (reference_trajectory[j].left_wall_bound < left_bound_k)
                    left_bound_k = reference_trajectory[j].left_wall_bound;
                if (reference_trajectory[j].right_wall_bound < right_bound_k)
                    right_bound_k = reference_trajectory[j].right_wall_bound;
            }
        }

        compute_wall_ey_bounds(
            left_bound_k,
            right_bound_k,
            config.wall_margin,
            &wall_x_lb,
            &wall_x_ub);

        /* Tighten the corridor by an extra buffer (in addition to wall_margin)
         * to improve robustness against model mismatch and discretization. */
        float wall_x_lb_con = wall_x_lb;
        float wall_x_ub_con = wall_x_ub;
        if (wall_bias_clear_m > 0.0f && isfinite(wall_bias_clear_m)) {
            wall_x_lb_con = wall_x_lb + wall_bias_clear_m;
            wall_x_ub_con = wall_x_ub - wall_bias_clear_m;
            if (wall_x_lb_con > wall_x_ub_con) {
                wall_x_lb_con = wall_x_lb;
                wall_x_ub_con = wall_x_ub;
            }
        }

        /* === q (9 elements): linear state cost (tracking references) === */
        {
            float ey_ref_k = reference_trajectory[k].reference_lateral_error;
            ey_ref_k = compute_wall_biased_ey_ref(ey_ref_k, wall_x_lb_con, wall_x_ub_con,
                                                  wall_ref_clear_m, wall_bias_max_m);
            sd->q[0] = -(sd->Q_diag[0] * ey_ref_k);
        }
        sd->q[1] = -(sd->Q_diag[1] * reference_trajectory[k].reference_heading_error);

        {
            float v_ref_track = reference_trajectory[k].reference_velocity;
            sd->q[2] = -(sd->Q_diag[2] * v_ref_track);
        }

        sd->q[3] = -(sd->Q_diag[3] * reference_trajectory[k].reference_lateral_velocity);
        sd->q[4] = -(sd->Q_diag[4] * reference_trajectory[k].reference_yaw_rate);

        sd->q[IDX_DELTA_COMMAND] = 0.0f;

        /* Effective-steering reference: delta_ff = atan(L*kappa). */
        {
            float delta_ff_k = atanf(VP_WHEELBASE_M * kappa_k);
            if (delta_ff_k > VP_MAX_STEERING_RAD) delta_ff_k = VP_MAX_STEERING_RAD;
            if (delta_ff_k < -VP_MAX_STEERING_RAD) delta_ff_k = -VP_MAX_STEERING_RAD;
            sd->q[IDX_DELTA_EFFECTIVE] =
                -(sd->Q_diag[IDX_DELTA_EFFECTIVE] * delta_ff_k);
        }
        sd->q[IDX_DRATE_PREV] = 0;  /* No tracking ref for δ̇_prev */
        sd->q[IDX_ACCEL_PREV] = 0;  /* No tracking ref for a_prev */

        /* === R_diag (2 elements): control cost === */
        /* R[0]: weight on |δ̇|² = effort + jerk penalty */
        sd->R_diag[0] = RICCATI_COST_FACTOR * (config.weight_steering_effort + config.weight_steering_rate);
        /* R[1]: weight on |a|² = effort + rate penalty */
        sd->R_diag[1] = RICCATI_COST_FACTOR * (config.weight_acceleration_effort + config.weight_acceleration_rate);

        if (k == 0) {
            sd->R_diag[0] = RICCATI_COST_FACTOR * (config.weight_steering_effort + (config.weight_steering_rate * config.cross_call_rate_scale));
            sd->R_diag[1] = RICCATI_COST_FACTOR * (config.weight_acceleration_effort + (config.weight_acceleration_rate * config.cross_call_rate_scale));
        }

        /* r: no constant control bias */
        sd->r[0] = 0;
        sd->r[1] = 0;

        /* === Cross-cost N (9x2) === */
        /* Couple previous and current steering rates for the jerk cost. */
        sd->N[IDX_DRATE_PREV][0] = -(RICCATI_COST_FACTOR * config.weight_steering_rate);
        /* Couple previous and current acceleration for the rate cost. */
        sd->N[IDX_ACCEL_PREV][1] = -(RICCATI_COST_FACTOR * config.weight_acceleration_rate);

        if (k == 0) {
            sd->N[IDX_DRATE_PREV][0] = -(RICCATI_COST_FACTOR * (config.weight_steering_rate * config.cross_call_rate_scale));
            sd->N[IDX_ACCEL_PREV][1] = -(RICCATI_COST_FACTOR * (config.weight_acceleration_rate * config.cross_call_rate_scale));
        }

        /* === State bounds (9 elements) === */

        /* e_y wall bounds active from the first stage. */
        sd->x_lb[0] = wall_x_lb_con;
        sd->x_ub[0] = wall_x_ub_con;

        /* States 1-4 (e_psi, vx, vy, omega): unconstrained */
        for (int s = 1; s < 5; s++) {
            sd->x_lb[s] = -BIG_BOUND;
            sd->x_ub[s] = BIG_BOUND;
        }

        /* The issued command carries the hard steering-angle constraint. */
        sd->x_lb[IDX_DELTA_COMMAND] = -VP_MAX_STEERING_RAD;
        sd->x_ub[IDX_DELTA_COMMAND] = VP_MAX_STEERING_RAD;

        /* The stable pole stays inside bounded commands without an extra ADMM
         * projection channel. */
        sd->x_lb[IDX_DELTA_EFFECTIVE] = -BIG_BOUND;
        sd->x_ub[IDX_DELTA_EFFECTIVE] = BIG_BOUND;

        /* Previous controls are unconstrained states. */
        sd->x_lb[IDX_DRATE_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_DRATE_PREV] = BIG_BOUND;
        sd->x_lb[IDX_ACCEL_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_ACCEL_PREV] = BIG_BOUND;

        /* === Control bounds === */
        /* u[0] = δ̇: steering RATE limit */
        sd->u_lb[0] = -STEERING_RATE_LIMIT;
        sd->u_ub[0] = STEERING_RATE_LIMIT;

        /* u[1] acceleration bound follows a constant-power region model. */
        {
            float v_ref_k = reference_trajectory[k].reference_velocity;
            float v_model_k = v_state_for_limits;
            float v_for_limit;
            float a_max = VP_MAX_ACCEL_MPS2;
            float a_min = VP_MIN_ACCEL_MPS2;

            if (v_model_k < MIN_LINEARIZATION_VELOCITY)
                v_model_k = MIN_LINEARIZATION_VELOCITY;

            /* Blend model speed with reference speed so under-speed states are not
             * over-limited by an aggressive reference profile. */
            v_for_limit = 0.7f * v_model_k + 0.3f * v_ref_k;
            if (v_for_limit < MIN_LINEARIZATION_VELOCITY)
                v_for_limit = MIN_LINEARIZATION_VELOCITY;

            if (v_for_limit > V_SWITCH) {
                float scale = V_SWITCH / v_for_limit;
                sd->u_ub[1] = a_max * scale;
                sd->u_lb[1] = a_min;
            } else {
                sd->u_ub[1] = a_max;
                sd->u_lb[1] = a_min;
            }
        }

        lin_state = lin_state_next;
        if (lin_state.flong_vel < MIN_LINEARIZATION_VELOCITY)
            lin_state.flong_vel = MIN_LINEARIZATION_VELOCITY;
    }

    /* Terminal state cost */
    float terminal_Q[RICCATI_MAX_NX];
    float terminal_q[RICCATI_MAX_NX];
    float terminal_x_lb[RICCATI_MAX_NX];
    float terminal_x_ub[RICCATI_MAX_NX];
    memset(terminal_Q, 0, sizeof(terminal_Q));
    memset(terminal_q, 0, sizeof(terminal_q));
    memset(terminal_x_lb, 0, sizeof(terminal_x_lb));
    memset(terminal_x_ub, 0, sizeof(terminal_x_ub));

    terminal_Q[0] = RICCATI_COST_FACTOR * config.weight_lateral_error;
    terminal_Q[1] = RICCATI_COST_FACTOR * config.weight_heading_error;
    terminal_Q[2] = RICCATI_COST_FACTOR * config.weight_velocity;
    terminal_Q[3] = RICCATI_COST_FACTOR * config.weight_lateral_velocity;
    terminal_Q[4] = RICCATI_COST_FACTOR * config.weight_yaw_rate;
    terminal_Q[IDX_DELTA_EFFECTIVE] =
        RICCATI_COST_FACTOR * config.weight_effective_steering;
    /* Command and previous-control terminal weights remain zero. */


    /* Terminal q: tracking at last reference */
    if (N > 0) {
        terminal_q[0] = -(terminal_Q[0] * reference_trajectory[N-1].reference_lateral_error);
        terminal_q[1] = -(terminal_Q[1] * reference_trajectory[N-1].reference_heading_error);
        {
            float v_ref_terminal = reference_trajectory[N-1].reference_velocity;
            terminal_q[2] = -(terminal_Q[2] * v_ref_terminal);
        }
        terminal_q[3] = -(terminal_Q[3] * reference_trajectory[N-1].reference_lateral_velocity);
        terminal_q[4] = -(terminal_Q[4] * reference_trajectory[N-1].reference_yaw_rate);
        /* Effective steering tracks terminal curvature feedforward. */
        {
            float kappa_N = reference_trajectory[N-1].path_curvature;
            float delta_ff_N = atanf(VP_WHEELBASE_M * kappa_N);
            if (delta_ff_N > VP_MAX_STEERING_RAD) delta_ff_N = VP_MAX_STEERING_RAD;
            if (delta_ff_N < -VP_MAX_STEERING_RAD) delta_ff_N = -VP_MAX_STEERING_RAD;
            terminal_q[IDX_DELTA_EFFECTIVE] =
                -(terminal_Q[IDX_DELTA_EFFECTIVE] * delta_ff_N);
        }

        {
            float wall_x_lb = -BIG_BOUND;
            float wall_x_ub = BIG_BOUND;
            float left_bound_N = reference_trajectory[N-1].left_wall_bound;
            float right_bound_N = reference_trajectory[N-1].right_wall_bound;
            if (wall_bound_window > 0) {
                int j0 = (N - 1) - wall_bound_window;
                int j1 = (N - 1) + wall_bound_window;
                if (j0 < 0) j0 = 0;
                if (j1 > (N - 1)) j1 = (N - 1);
                for (int j = j0; j <= j1; j++) {
                    if (reference_trajectory[j].left_wall_bound < left_bound_N)
                        left_bound_N = reference_trajectory[j].left_wall_bound;
                    if (reference_trajectory[j].right_wall_bound < right_bound_N)
                        right_bound_N = reference_trajectory[j].right_wall_bound;
                }
            }
            compute_wall_ey_bounds(
                left_bound_N,
                right_bound_N,
                config.wall_margin,
                &wall_x_lb,
                &wall_x_ub);

            float wall_x_lb_con = wall_x_lb;
            float wall_x_ub_con = wall_x_ub;
            if (wall_bias_clear_m > 0.0f && isfinite(wall_bias_clear_m)) {
                wall_x_lb_con = wall_x_lb + wall_bias_clear_m;
                wall_x_ub_con = wall_x_ub - wall_bias_clear_m;
                if (wall_x_lb_con > wall_x_ub_con) {
                    wall_x_lb_con = wall_x_lb;
                    wall_x_ub_con = wall_x_ub;
                }
            }

            terminal_x_lb[0] = wall_x_lb_con;
            terminal_x_ub[0] = wall_x_ub_con;

            /* Apply same wall-bias to the terminal lateral reference. */
            {
                float ey_ref_N = reference_trajectory[N-1].reference_lateral_error;
                ey_ref_N = compute_wall_biased_ey_ref(ey_ref_N, wall_x_lb_con, wall_x_ub_con, 0.0f, wall_bias_max_m);
                terminal_q[0] = -(terminal_Q[0] * ey_ref_N);
            }
        }

        for (int s = 1; s < 5; s++) {
            terminal_x_lb[s] = -BIG_BOUND;
            terminal_x_ub[s] = BIG_BOUND;
        }
        terminal_x_lb[IDX_DELTA_COMMAND] = -VP_MAX_STEERING_RAD;
        terminal_x_ub[IDX_DELTA_COMMAND] = VP_MAX_STEERING_RAD;
        terminal_x_lb[IDX_DELTA_EFFECTIVE] = -BIG_BOUND;
        terminal_x_ub[IDX_DELTA_EFFECTIVE] = BIG_BOUND;
        terminal_x_lb[IDX_DRATE_PREV] = -BIG_BOUND;
        terminal_x_ub[IDX_DRATE_PREV] = BIG_BOUND;
        terminal_x_lb[IDX_ACCEL_PREV] = -BIG_BOUND;
        terminal_x_ub[IDX_ACCEL_PREV] = BIG_BOUND;
    }

    /* ---------------------------------------------------------------
     * Step 3: Build augmented initial state (9 elements)
     * --------------------------------------------------------------- */
    float x0[RICCATI_MAX_NX];
    memset(x0, 0, sizeof(x0));
    x0[0] = frenet->flat_error;
    x0[1] = frenet->fhead_error;
    x0[2] = frenet->flong_vel;
    x0[3] = frenet->flat_vel;
    x0[4] = frenet->fyaw_rate;
    x0[IDX_DELTA_COMMAND] = commanded_steering_angle;
    x0[IDX_DELTA_EFFECTIVE] = effective_steering_angle;
    x0[IDX_DRATE_PREV] = prev_control.steer_ang;  /* Previous delta-rate command */
    x0[IDX_ACCEL_PREV] = prev_control.long_acc;

    /* ---------------------------------------------------------------
     * Step 4: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */

    /* ---------------------------------------------------------------
     * Step 5: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */
    const float solver_rho = get_env_float("RHO", ADMM_RHO);
    float solver_rho_u = ADMM_RHO_U;
    int shared_rho = 0;
    {
        const char *rho_u_env = getenv("RHO_U");
        if (rho_u_env != NULL && rho_u_env[0] != '\0') {
            solver_rho_u = strtof(rho_u_env, NULL);
            if (!(solver_rho_u > 0.0f)) {
                shared_rho = 1;
                solver_rho_u = (solver_rho > 0.0f) ? solver_rho : ADMM_RHO;
            }
        }
    }
    if (get_env_int("MPC_SHARED_RHO", get_env_int("SHARED_RHO", shared_rho)) != 0) {
        shared_rho = 1;
        solver_rho_u = solver_rho;
    }
    const int adaptive_rho =
        get_env_int("MPC_ADAPTIVE_RHO", get_env_int("ADAPTIVE_RHO", 1)) != 0;

    RiccatiAdmmConfig_t solver_config = {
        .rho = solver_rho,
        .rho_u = solver_rho_u,
        .tolerance = get_env_float("TOL", config.solver_convergence_tolerance),
        .max_iterations = get_env_int("MAX_ITER", (int)config.max_solver_iterations),
        .adaptive_rho = adaptive_rho,
        .shared_rho = shared_rho,
    };

    RiccatiSolution_t riccati_sol;
    memset(&riccati_sol, 0, sizeof(riccati_sol));

    /* Cold start on first call, model-signature change, abrupt curvature
     * change, or when the previous primal trajectory is incompatible with the
     * current lateral-error box (start and terminal). */
    {
        const float cur_curvature = reference_trajectory[0].path_curvature;
        const float kappa_diff = fabsf(cur_curvature - warm_start_prev_curvature);
        const int curvature_jump = (kappa_diff >= MPC_WS_CURVATURE_THRESH);
        const int signature_jump =
            (warm_start_prev_model_signature != MPC_MODEL_SIGNATURE);

        int bound_incompatible = 0;
        if (admm_state.initialized) {
            const float ey0 = admm_state.z_x[0][IDX_EY];
            const float eyN = admm_state.z_x[PREDICTION_HORIZON][IDX_EY];
            const float tol = MPC_WS_BOUND_THRESH;

            if (ey0 < (step_data[0].x_lb[IDX_EY] - tol) ||
                ey0 > (step_data[0].x_ub[IDX_EY] + tol) ||
                eyN < (terminal_x_lb[IDX_EY] - tol) ||
                eyN > (terminal_x_ub[IDX_EY] + tol)) {
                bound_incompatible = 1;
            }
        }

        if (!admm_state.initialized || signature_jump || curvature_jump ||
            bound_incompatible) {
            riccati_admm_state_init(&admm_state);
        }

        warm_start_prev_curvature = cur_curvature;
        warm_start_prev_model_signature = MPC_MODEL_SIGNATURE;
    }

    RiccatiStatus_t rstatus = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, terminal_x_lb, terminal_x_ub, x0,
        NX_AUG, NU, N,
        &solver_config, &admm_state, &riccati_sol);

    if (rstatus != RICCATI_STATUS_OPTIMAL && rstatus != RICCATI_STATUS_MAX_ITERATIONS) {
        result->optimal_control.steer_ang = commanded_steering_angle;
        /* Avoid "dead stop" behavior on transient solver failures by holding the
         * previous acceleration command. */
        result->optimal_control.long_acc = prev_control.long_acc;
        result->iterations_used = (uint16_t)riccati_sol.iterations;
        result->final_cost = riccati_sol.primal_residual;
        result->dual_residual = riccati_sol.dual_residual;
        result->solver_status = MPC_STATUS_ERROR;
        return result->solver_status;
    }

    /* Step 6: map solver output [delta_rate, accel] to actuator command. */
    float delta_rate = admm_state.z_u[0][0];
    float accel = admm_state.z_u[0][1];

    /* Preserve the existing 30 ms rate-to-target semantics for this isolated
     * model change; the CPU controller still replans every 5 ms. */
    float delta_cmd =
        commanded_steering_angle + config.time_step * delta_rate;

    /* Clamp to physical steering limits */
    if (delta_cmd > VP_MAX_STEERING_RAD)
        delta_cmd = VP_MAX_STEERING_RAD;
    if (delta_cmd < -VP_MAX_STEERING_RAD)
        delta_cmd = -VP_MAX_STEERING_RAD;

    ControlInput_t raw_control;
    raw_control.steer_ang = delta_cmd;
    raw_control.long_acc = accel;

    ControlInput_t saturated = vehicle_model_saturate_control(&raw_control);

    result->optimal_control = saturated;
    result->iterations_used = (uint16_t)riccati_sol.iterations;
    result->final_cost = riccati_sol.primal_residual;
    result->dual_residual = riccati_sol.dual_residual;

    switch (rstatus) {
    case RICCATI_STATUS_OPTIMAL:
        result->solver_status = MPC_STATUS_SUCCESS;
        break;
    case RICCATI_STATUS_MAX_ITERATIONS:
        result->solver_status = MPC_STATUS_MAXIMUM_ITERATIONS_REACHED;
        break;
    default:
        result->solver_status = MPC_STATUS_ERROR;
        break;
    }

    /* Save previous control for rate-of-rate penalty.
     * Store the steering rate (δ̇) as the "steering" value, and accel as-is. */
    prev_control.steer_ang = delta_rate;
    prev_control.long_acc = saturated.long_acc;
    return result->solver_status;
}
