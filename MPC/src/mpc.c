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
 *    is augmented with previous control [delta_prev, accel_prev] to
 *    handle rate penalty natively in the LQR cost structure.
 *
 * 3. Wall constraints are direct box constraints on x_k[0] (=e_y),
 *    handled naturally by ADMM's projection step.
 *
 * All arithmetic uses native float operations on CPU.
 * @dependencies mpc.h, util_math.h, riccati_solver.h, vehicle_model.h,
 *               mpc_types.h, <string.h>, <stdio.h>, <stdlib.h>
 */

#include "mpc.h"
#include "util_math.h"
#include "riccati_solver.h"
#include "vehicle_model.h"
#include "mpc_types.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MpcConfiguration_t config;
static int initialized = 0;
static ControlInput_t prev_control;
static float actual_steering_angle = 0;  /* Servo physical position */
static RiccatiAdmmState_t admm_state;
static float warm_start_prev_curvature = 0;


/*===========================================================================
 * Default Configuration
 *===========================================================================*/

MpcConfiguration_t get_default_configuration(void)
{
    MpcConfiguration_t cfg = {
    .prediction_horizon_steps = PREDICTION_HORIZON,
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
    .weight_delta_actual         = WEIGHT_DELTA_ACTUAL,

    /* Cross-call rate scale computed from control and prediction periods. */
    .cross_call_rate_scale = CROSS_CALL_RATE_SCALE,

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
    cfg.weight_acceleration_rate = get_env_float("MPC_W_TORQUE_RATE", cfg.weight_acceleration_rate);
    cfg.weight_delta_actual      = get_env_float("MPC_W_DELTA_ACTUAL", cfg.weight_delta_actual);

    cfg.cross_call_rate_scale = get_env_float("MPC_CROSS_CALL_SCALE", cfg.cross_call_rate_scale);

    int horizon = get_env_int("HORIZON", cfg.prediction_horizon_steps);
    if (horizon >= 1 && horizon <= PREDICTION_HORIZON)
        cfg.prediction_horizon_steps = horizon;

    float dt = get_env_float("PRED_DT", cfg.time_step);
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
    actual_steering_angle = 0.0f;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0.0f;
    initialized = 1;
}

/* Initialize MPC module with a caller-supplied configuration.
 * Falls back to default configuration when cfg is NULL. */
void mpc_initialize_with_configuration(const MpcConfiguration_t *cfg)
{
    config = cfg ? *cfg : get_default_configuration();
    prev_control.steer_ang = 0.0f;
    prev_control.long_acc = 0.0f;
    actual_steering_angle = 0.0f;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0.0f;
    initialized = 1;
}

/* Reset all inter-cycle state (previous control, servo position, warm-start)
 * without altering the active configuration or vehicle model parameters. */
void mpc_reset(void)
{
    prev_control.steer_ang = 0.0f;
    prev_control.long_acc = 0.0f;
    actual_steering_angle = 0.0f;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0.0f;
}

MpcConfiguration_t mpc_get_configuration(void)
{
    return config;
}

void mpc_set_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration) config = *configuration;
}

void mpc_set_actual_previous_control(const ControlInput_t *actual)
{
    if (actual) {
        /* For the 8-state formulation:
         * - prev_control stores the previous δ̇ (steering rate) and acceleration
         *   for the rate-of-rate penalty.
         * - actual_steering_angle stores the physical servo position for x0[5]. */
        actual_steering_angle = actual->steer_ang;
        prev_control.long_acc =
            actual->long_acc;
    }
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

    /* EMA filter for measurement smoothing. */
    FrenetState_t filtered_state = *current_frenet_state;
    {
        static float ema_alpha = EMA_ALPHA;
        static int ema_initialized = 0;
        static int alpha_cached = 0;
        static FrenetState_t prev_filtered;

        if (!alpha_cached) {
            ema_alpha = get_env_float("MPC_EMA_ALPHA", ema_alpha);
            if (ema_alpha < 0.0f) ema_alpha = 0.0f;
            if (ema_alpha > 1.0f) ema_alpha = 1.0f;
            alpha_cached = 1;
        }

        if (!ema_initialized) {
            prev_filtered = *current_frenet_state;
            ema_initialized = 1;
        }

        if (ema_alpha < 1.0f) {
            float a = ema_alpha;
            float b = 1.0f - a;

            filtered_state.flat_error =
                a * current_frenet_state->flat_error +
                b * prev_filtered.flat_error;

            filtered_state.fhead_error =
                a * current_frenet_state->fhead_error +
                b * prev_filtered.fhead_error;

            filtered_state.flong_vel =
                a * current_frenet_state->flong_vel +
                b * prev_filtered.flong_vel;
            
            filtered_state.flat_vel =
                a * current_frenet_state->flat_vel +
                b * prev_filtered.flat_vel;

            filtered_state.fyaw_rate =
                a * current_frenet_state->fyaw_rate +
                b * prev_filtered.fyaw_rate;
        }
        prev_filtered = filtered_state;
    }
    const FrenetState_t *frenet = &filtered_state;

    // Extract horizon length and clamp to compile-time maximum.
    int N = config.prediction_horizon_steps;
    if (N > PREDICTION_HORIZON) N = PREDICTION_HORIZON;

    /* ---------------------------------------------------------------
     * Step 1: Prepare model constants for per-step linearization.
     *
     * Full Frenet linearization is computed per horizon point in the
     * loop below (A_step/B_step), using each step's curvature and
     * feedforward steering operating point.
     * --------------------------------------------------------------- */

    float delta_clamp = VP_MAX_STEERING_RAD * STEERING_FEEDFORWARD_CLAMP_FACTOR;

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
            sd->A[IDX_DELTA_ACTUAL][i] = 0;
            sd->A[IDX_DRATE_PREV][i] = 0;
            sd->A[IDX_ACCEL_PREV][i] = 0;

            sd->N[i][0] = 0;
            sd->N[i][1] = 0;

            if (i < NX_DENSE) {
                sd->A[i][IDX_DRATE_PREV] = 0;
                sd->A[i][IDX_ACCEL_PREV] = 0;
            }
            if (i < NX_FRENET) {
                sd->B[i][0] = 0;
            }
        }
        sd->B[IDX_DELTA_ACTUAL][1] = 0;  /* δ_actual integrator not affected by accel */
        sd->B[IDX_DRATE_PREV][1] = 0;    /* δ̇_prev not affected by accel */
        sd->B[IDX_ACCEL_PREV][0] = 0;    /* a_prev not affected by δ̇ */

        /* --- End sparse zeroing --- */

        /* Per-step Frenet linearization */
        float kappa_k = reference_trajectory[k].path_curvature;

        ControlInput_t lin_control;
        lin_control.steer_ang = atanf(VP_WHEELBASE_M * kappa_k);
        if (lin_control.steer_ang > delta_clamp)
            lin_control.steer_ang = delta_clamp;
        if (lin_control.steer_ang < -delta_clamp)
            lin_control.steer_ang = -delta_clamp;
        lin_control.long_acc = 0;

        lin_state.flong_vel =
            reference_trajectory[k].reference_velocity;
        if (lin_state.flong_vel < MIN_LINEARIZATION_VELOCITY)
            lin_state.flong_vel = MIN_LINEARIZATION_VELOCITY;

        float A_step[5][5];
        float B_step[5][2];

        vehicle_model_compute_frenet_linearization(
            &lin_state, &lin_control,
            config.time_step,
            kappa_k,
            A_step, B_step);

        /* Stabilize fast dynamics (omega row = 4) per stage */
        /* Clamp the yaw-rate row's self-coupling coefficient to the stability
         * boundary to prevent discrete-time instability in the fast lateral
         * dynamics at large time steps. */
        {
            int row = 4;  /* Yaw-rate row (omega) in the 5-state Frenet model */
            float abs_aii = fabsf(A_step[row][row]);
            if (abs_aii > STABILITY_LIMIT) {
                float target = (A_step[row][row] < 0)
                    ? -STABILITY_LIMIT : STABILITY_LIMIT;
                float num = target - 1.0f;
                float den = A_step[row][row] - 1.0f;
                if (den != 0) {
                    float scale = num/den;
                    for (int j = 0; j < 5; j++) {
                        if (j != row) A_step[row][j] = A_step[row][j] * scale;
                    }
                    B_step[row][0] = B_step[row][0] * scale;
                    B_step[row][1] = B_step[row][1] * scale;
                }
                A_step[row][row] = target;
            }
        }

        /* === Augmented A matrix (8×8) === */

        /* Top-left 5×5: per-step Frenet A (e_y, e_psi, vx, vy, omega) */
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
            sd->A[i][j] = A_step[i][j];

        /* Column 5 of A (rows 0-4): steering effect via δ_actual state. */
        for (int i = 0; i < 5; i++)
            sd->A[i][IDX_DELTA_ACTUAL] = B_step[i][0];

        /* A[5][5] = 1: δ_actual integrator (δ_{k+1} = δ_k + dt*δ̇) */
        sd->A[IDX_DELTA_ACTUAL][IDX_DELTA_ACTUAL] = 1.0f;

        /* Rows 6-7: already zeroed above */
        /* Cols 6-7: already zeroed above */

        /* === Augmented B matrix (8×2) === */

        /* Rows 0-4, col 0: already zeroed above */

        /* Rows 0-4, col 1: acceleration effect on dynamics */
        for (int i = 0; i < 5; i++)
            sd->B[i][1] = B_step[i][1];

        /* Row 5: δ_actual integrator — B[5][0] = dt */
        sd->B[IDX_DELTA_ACTUAL][0] = config.time_step;

        /* Row 6: δ̇_prev_{k+1} = u[0] = δ̇ */
        sd->B[IDX_DRATE_PREV][0] = 1.0f;

        /* Row 7: a_prev_{k+1} = u[1] = a */
        sd->B[IDX_ACCEL_PREV][1] = 1.0f;

        /* === Q_diag (8 elements): state tracking weights === */
        sd->Q_diag[0] = RICCATI_COST_FACTOR * config.weight_lateral_error;
        sd->Q_diag[1] = RICCATI_COST_FACTOR * config.weight_heading_error;
        sd->Q_diag[2] = RICCATI_COST_FACTOR * config.weight_velocity;
        sd->Q_diag[3] = RICCATI_COST_FACTOR * config.weight_lateral_velocity;
        sd->Q_diag[4] = RICCATI_COST_FACTOR * config.weight_yaw_rate;
        sd->Q_diag[IDX_DELTA_ACTUAL] = RICCATI_COST_FACTOR * config.weight_delta_actual;
        sd->Q_diag[IDX_DRATE_PREV] = RICCATI_COST_FACTOR * config.weight_steering_rate;
        sd->Q_diag[IDX_ACCEL_PREV] = RICCATI_COST_FACTOR * config.weight_acceleration_rate;

        /* Apply cross-call scaling for step 0 (jerk/rate penalties) */
        if (k == 0) {
            sd->Q_diag[IDX_DRATE_PREV] = RICCATI_COST_FACTOR * (config.weight_steering_rate * config.cross_call_rate_scale);
            sd->Q_diag[IDX_ACCEL_PREV] = RICCATI_COST_FACTOR * (config.weight_acceleration_rate * config.cross_call_rate_scale);
        }

        /* === q (8 elements): linear state cost (tracking references) === */
        sd->q[0] = -(sd->Q_diag[0] * reference_trajectory[k].reference_lateral_error);
        sd->q[1] = -(sd->Q_diag[1] * reference_trajectory[k].reference_heading_error);

        sd->q[2] = -(sd->Q_diag[2] * reference_trajectory[k].reference_velocity);

        sd->q[3] = -(sd->Q_diag[3] * reference_trajectory[k].reference_lateral_velocity);
        sd->q[4] = -(sd->Q_diag[4] * reference_trajectory[k].reference_yaw_rate);

        /* δ_actual reference: feedforward steering δ_ff = atan(L*κ) */
        {
            float delta_ff_k = atan(VP_WHEELBASE_M * kappa_k);
            sd->q[IDX_DELTA_ACTUAL] = -(sd->Q_diag[IDX_DELTA_ACTUAL] * delta_ff_k);
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

        /* === Cross-cost N (8×2) === */
        /* N[6][0]: couples δ̇_prev (x[6]) with δ̇ (u[0]) — steering jerk */
        sd->N[IDX_DRATE_PREV][0] = -(RICCATI_COST_FACTOR * config.weight_steering_rate);
        /* N[7][1]: couples a_prev (x[7]) with a (u[1]) — accel rate */
        sd->N[IDX_ACCEL_PREV][1] = -(RICCATI_COST_FACTOR * config.weight_acceleration_rate);

        if (k == 0) {
            sd->N[IDX_DRATE_PREV][0] = -(RICCATI_COST_FACTOR * (config.weight_steering_rate * config.cross_call_rate_scale));
            sd->N[IDX_ACCEL_PREV][1] = -(RICCATI_COST_FACTOR * (config.weight_acceleration_rate * config.cross_call_rate_scale));
        }

        /* === State bounds (8 elements) === */

        /* e_y: wall constraints from step 1 through horizon (all remaining steps). */
        int wall_active = (k >= 1);

        if (wall_active) {
            float left_bound = reference_trajectory[k].left_wall_bound;
            float right_bound = reference_trajectory[k].right_wall_bound;
            if (left_bound <= WALL_MARGIN || right_bound <= WALL_MARGIN) {
                /* Invalid wall bounds -> force infeasible interval for safe stop behavior. */
                sd->x_lb[0] = 1.0f;
                sd->x_ub[0] = -1.0f;
            } else {
                sd->x_lb[0] = -(right_bound - WALL_MARGIN);
                sd->x_ub[0] = left_bound - WALL_MARGIN;
            }
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
        sd->x_lb[IDX_DELTA_ACTUAL] = -VP_MAX_STEERING_RAD;
        sd->x_ub[IDX_DELTA_ACTUAL] = VP_MAX_STEERING_RAD;

        /* States 6-7 (prev controls): unconstrained */
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
            float a_max = VP_MAX_ACCEL_MPS2;
            float a_min = VP_MIN_ACCEL_MPS2;

            if (v_ref_k > V_SWITCH) {
                float scale = V_SWITCH / v_ref_k;
                sd->u_ub[1] = a_max * scale;
                sd->u_lb[1] = a_min * scale;  /* a_min is negative */
            } else {
                sd->u_ub[1] = a_max;
                sd->u_lb[1] = a_min;
            }
        }
    }

    /* Terminal state cost */
    float terminal_Q[RICCATI_MAX_NX];
    float terminal_q[RICCATI_MAX_NX];
    memset(terminal_Q, 0, sizeof(terminal_Q));
    memset(terminal_q, 0, sizeof(terminal_q));

    terminal_Q[0] = RICCATI_COST_FACTOR * config.weight_lateral_error;
    terminal_Q[1] = RICCATI_COST_FACTOR * config.weight_heading_error;
    terminal_Q[2] = RICCATI_COST_FACTOR * config.weight_velocity;
    terminal_Q[3] = RICCATI_COST_FACTOR * config.weight_lateral_velocity;
    terminal_Q[4] = RICCATI_COST_FACTOR * config.weight_yaw_rate;
    terminal_Q[IDX_DELTA_ACTUAL] = RICCATI_COST_FACTOR * config.weight_delta_actual;
    /* No jerk/rate penalty on terminal prev controls (Q[6:7] = 0) */

    /* Terminal q: tracking at last reference */
    if (N > 0) {
        terminal_q[0] = -(terminal_Q[0] * reference_trajectory[N-1].reference_lateral_error);
        terminal_q[1] = -(terminal_Q[1] * reference_trajectory[N-1].reference_heading_error);
        terminal_q[2] = -(terminal_Q[2] * reference_trajectory[N-1].reference_velocity);
        terminal_q[3] = -(terminal_Q[3] * reference_trajectory[N-1].reference_lateral_velocity);
        terminal_q[4] = -(terminal_Q[4] * reference_trajectory[N-1].reference_yaw_rate);
        /* δ_actual terminal: track feedforward */
        {
            float kappa_N = reference_trajectory[N-1].path_curvature;
            float delta_ff_N = atan(VP_WHEELBASE_M * kappa_N);
            terminal_q[IDX_DELTA_ACTUAL] = -(terminal_Q[IDX_DELTA_ACTUAL] * delta_ff_N);
        }
    }

    /* ---------------------------------------------------------------
     * Step 3: Build augmented initial state (8 elements)
     * --------------------------------------------------------------- */
    float x0[RICCATI_MAX_NX];
    memset(x0, 0, sizeof(x0));
    x0[0] = frenet->flat_error;
    x0[1] = frenet->fhead_error;
    x0[2] = frenet->flong_vel;
    x0[3] = frenet->flat_vel;
    x0[4] = frenet->fyaw_rate;
    x0[IDX_DELTA_ACTUAL] = actual_steering_angle;      /* Physical servo position */
    x0[IDX_DRATE_PREV] = prev_control.steer_ang;  /* Previous δ̇ command */
    x0[IDX_ACCEL_PREV] = prev_control.long_acc;

    /* ---------------------------------------------------------------
     * Step 4: Warm-start management
     * --------------------------------------------------------------- */
    float cur_curvature = reference_trajectory[0].path_curvature;
    /* Warm-start: reuse ADMM state unless curvature changed drastically. */
    {
        float kappa_diff = fabsf(cur_curvature - warm_start_prev_curvature);
        if (!admm_state.initialized || kappa_diff > WARMSTART_CURVATURE_RESET_THRESHOLD) {
            riccati_admm_state_init(&admm_state);
        }
    }
    warm_start_prev_curvature = cur_curvature;

    /* ---------------------------------------------------------------
     * Step 5: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */
    RiccatiAdmmConfig_t solver_config = {
        .rho = get_env_float("RHO", ADMM_RHO),
        .rho_u = get_env_float("RHO_U", ADMM_RHO_U),
        .tolerance = get_env_float("TOL", config.solver_convergence_tolerance),
        .max_iterations = get_env_int("MAX_ITER", (int)config.max_solver_iterations),
        .adaptive_rho = 1,
        .alpha = get_env_float("ALPHA", ADMM_ALPHA),
    };

    RiccatiSolution_t riccati_sol;
    memset(&riccati_sol, 0, sizeof(riccati_sol));

    RiccatiStatus_t rstatus = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, x0,
        NX_AUG, NU, N,
        &solver_config, &admm_state, &riccati_sol);

    /* Step 6: map solver output [delta_rate, accel] to actuator command. */
    float delta_rate = admm_state.z_u[0][0];
    float accel = admm_state.z_u[0][1];

    /* Compute steering angle command: δ_actual + dt * δ̇ */
    float delta_cmd = actual_steering_angle + config.time_step * delta_rate;

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
