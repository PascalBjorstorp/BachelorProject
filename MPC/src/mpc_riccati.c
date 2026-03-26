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
#include "util_math.h"
#include "riccati_solver.h"
#include "vehicle_model.h"
#include "mpc_types.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>


/*==========================================================================
* Helper function
*===========================================================================*/

static float get_env_float(const char *name, float default_val)
{
    const char *env = getenv(name);
    return env ? strtof(env, NULL) : default_val;
}


static int get_env_float_if_exists(const char *name, float *out)
{
    const char *env = getenv(name);
    if (env) {
        *out = strtof(env, NULL);
        return 1;
    }
    return 0;
}

static int get_env_int(const char *name, int default_val)
{
    const char *env = getenv(name);
    return env ? (int)strtol(env, NULL, 10) : default_val;
}

static int get_env_int_if_exists(const char *name, int *out)
{
    const char *env = getenv(name);
    if (env) {
        *out = (int)strtol(env, NULL, 10);
        return 1;
    }
    return 0;
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
    .prediction_horizon_steps = MPC_PREDICTION_HORIZON,
    .time_step = MPC_TIME_STEP_SECONDS,

    /* State tracking weights (Frenet frame). */
    .weight_lateral_error    = 5000.0f,
    .weight_heading_error    = 500.0f,
    .weight_velocity         = 165.0f,
    .weight_lateral_velocity = 10.0f,
    .weight_yaw_rate         = 5.0f,

    /* Control effort weights */
    .weight_steering_effort      = 0.18f,
    .weight_acceleration_effort  = 0.01f,

    /* Control rate weights (W_JERK, W_ACCEL_RATE) */
    .weight_steering_rate        = 0.15f,
    .weight_acceleration_rate    = 0.1f,

    /* Cross-call rate scale: CONTROL_DT / PRED_DT = 0.005 / 0.02 = 0.25. */
    .cross_call_rate_scale = 0.25f,

    /* Solver parameters */
    .max_solver_iterations = MPC_MAXIMUM_ITERATIONS,
    .solver_convergence_tolerance = MPC_CONVERGENCE_TOLERANCE
    };


    /* Environment variable overrides */

    /* --- Environment overrides --- */
    cfg.weight_lateral_error    = get_env_float("MPC_W_LAT_ERROR", cfg.weight_lateral_error);
    cfg.weight_heading_error    = get_env_float("MPC_W_HEADING", cfg.weight_heading_error);
    cfg.weight_velocity         = get_env_float("MPC_W_VELOCITY", cfg.weight_velocity);
    cfg.weight_lateral_velocity = get_env_float("MPC_W_LAT_VEL", cfg.weight_lateral_velocity);
    cfg.weight_yaw_rate         = get_env_float("MPC_W_YAW_RATE", cfg.weight_yaw_rate);

    cfg.weight_steering_effort     = get_env_float("MPC_W_STEER_EFFORT", cfg.weight_steering_effort);
    cfg.weight_acceleration_effort = get_env_float("MPC_W_ACCEL_EFFORT", cfg.weight_acceleration_effort);

    cfg.weight_steering_rate     = get_env_float("MPC_W_STEER_RATE", cfg.weight_steering_rate);
    cfg.weight_acceleration_rate = get_env_float("MPC_W_TORQUE_RATE", cfg.weight_acceleration_rate);

    cfg.cross_call_rate_scale = get_env_float("MPC_CROSS_CALL_SCALE", cfg.cross_call_rate_scale);

    int horizon = get_env_int("HORIZON", cfg.prediction_horizon_steps);
    if (horizon >= 1 && horizon <= MPC_PREDICTION_HORIZON)
        cfg.prediction_horizon_steps = horizon;

    float dt = get_env_float("PRED_DT", cfg.time_step);
    cfg.time_step = dt;

    /* Auto-update cross-call scaling if not explicitly set */
    if (getenv("MPC_CROSS_CALL_SCALE") == NULL)
        cfg.cross_call_rate_scale = 0.005f / dt;

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
    prev_control.steer_ang = 0;
    prev_control.long_acc = 0;
    actual_steering_angle = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
    initialized = 1;
}

static void mpc_riccati_initialize_with_configuration(const MpcConfiguration_t *cfg)
{
    config = cfg ? *cfg : get_default_configuration();
    vehicle_model_initialize();
    prev_control.steer_ang = 0;
    prev_control.long_acc = 0;
    actual_steering_angle = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
    initialized = 1;
}

static void mpc_riccati_reset(void)
{
    prev_control.steer_ang = 0;
    prev_control.long_acc = 0;
    actual_steering_angle = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
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

    /* ---------------------------------------------------------------
     * State filter: EMA (exponential moving average) for noise rejection.
     * filtered = alpha * measured + (1 - alpha) * prev_filtered
     * alpha = 1.0 → no filtering (bypass), alpha = 0.5 → strong smoothing.
     * Override at runtime via MPC_EMA_ALPHA environment variable.
     * Default: 1.0 (disabled). Recommended for noisy sensors: 0.7-0.9.
     * --------------------------------------------------------------- */
    FrenetState_t filtered_state = *current_frenet_state;
    {
        static float ema_alpha = 0.7f;
        static int initialized = 0;
        static FrenetState_t prev_filtered;

        if (!initialized) {
            prev_filtered = *current_frenet_state;
            initialized = 1;
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


    int N = config.prediction_horizon_steps;
    if (N > MPC_PREDICTION_HORIZON) N = MPC_PREDICTION_HORIZON;

    /* ---------------------------------------------------------------
     * Step 1: Prepare model constants for per-step linearization.
     *
     * Full Frenet linearization is computed per horizon point in the
     * loop below (A_step/B_step), using each step's curvature and
     * feedforward steering operating point.
     * --------------------------------------------------------------- */
    VehicleParameters_t vp = vehicle_model_get_parameters();
    float delta_clamp = vp.max_steering_angle * 0.5f;

    FrenetState_t lin_state = *frenet;
    if (lin_state.flong_vel < MIN_LINEARIZATION_VELOCITY)
        lin_state.flong_vel = MIN_LINEARIZATION_VELOCITY;

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
    float w_steer_rate_eff = config.weight_steering_effort;   /* Penalizes δ̇ magnitude */
    float w_accel_eff = config.weight_acceleration_effort;
    float w_steer_jerk = config.weight_steering_rate;         /* Penalizes δ̇ change (jerk) */
    float w_accel_rate = config.weight_acceleration_rate;

    /* Small weight on δ_actual² to prefer centered steering */
    float w_delta_actual = 0.5f;

    /* Build per-step data array */
    RiccatiStepData_t step_data[MPC_PREDICTION_HORIZON];
    /* NOTE: Do NOT use memset on the whole array (wastes ~31KB of zeroing).
     * Instead, zero only the sparse blocks that the loop doesn't fill. */

    for (int k = 0; k < N; k++) {
        RiccatiStepData_t *sd = &step_data[k];

        /* --- Sparse zeroing (replaces memset) --- */

        /* A rows 5,6,7: zero all columns (delta integrator + prev-controls
         * have no cross-coupling to Frenet states in cols 0..4) */
        for (int j = 0; j < NX_AUG; j++) {
            sd->A[IDX_DELTA_ACTUAL][j] = 0;
            sd->A[IDX_DRATE_PREV][j]   = 0;
            sd->A[IDX_ACCEL_PREV][j]   = 0;
        }
        /* A cols 6,7: zero for rows 0..5 */
        for (int i = 0; i < NX_DENSE; i++) {
            sd->A[i][IDX_DRATE_PREV] = 0;
            sd->A[i][IDX_ACCEL_PREV] = 0;
        }
        /* B rows 0-4, col 0: zero (δ̇ doesn't directly affect Frenet) */
        for (int i = 0; i < NX_FRENET; i++)
            sd->B[i][0] = 0;
        /* B sparse zeros for prev-control rows */
        sd->B[IDX_DRATE_PREV][1] = 0;  /* δ̇_prev not affected by accel */
        sd->B[IDX_ACCEL_PREV][0] = 0;  /* a_prev not affected by δ̇ */
        /* N cross-cost: zero all, non-zero entries set below */
        for (int i = 0; i < NX_AUG; i++) {
            sd->N[i][0] = 0;
            sd->N[i][1] = 0;
        }

        /* --- End sparse zeroing --- */

        /* Per-step Frenet linearization */
        float kappa_k = reference_trajectory[k].path_curvature;

        ControlInput_t lin_control;
        lin_control.steer_ang = atanf(vp.wheelbase_meters * kappa_k);
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
        {
            int row = 4;
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

        /* Column 5 of A (rows 0-4): steering effect via δ_actual.
         * This is the old B_frenet[:,0] — steering no longer comes
         * through the control, it comes through the δ_actual state. */
        for (int i = 0; i < 5; i++)
            sd->A[i][IDX_DELTA_ACTUAL] = B_step[i][0];

        /* A[5][5] = 1: δ_actual integrator (δ_{k+1} = δ_k + dt*δ̇) */
        sd->A[IDX_DELTA_ACTUAL][IDX_DELTA_ACTUAL] = 1.0f;

        /* Rows 6-7: already zeroed above */
        /* Cols 6-7: already zeroed above */

        /* === Augmented B matrix (8×2) === */

        /* Rows 0-4, col 0: already zeroed above */

        /* Rows 0-4, col 1: acceleration effect on dynamics (unchanged) */
        for (int i = 0; i < 5; i++)
            sd->B[i][1] = B_step[i][1];

        /* Row 5: δ_actual integrator — B[5][0] = dt */
        sd->B[IDX_DELTA_ACTUAL][0] = config.time_step;

        /* Row 6: δ̇_prev_{k+1} = u[0] = δ̇ */
        sd->B[IDX_DRATE_PREV][0] = 1.0f;

        /* Row 7: a_prev_{k+1} = u[1] = a */
        sd->B[IDX_ACCEL_PREV][1] = 1.0f;

        /* === Q_diag (8 elements): state tracking weights === */
        sd->Q_diag[0] = 2.0f * config.weight_lateral_error;
        sd->Q_diag[1] = 2.0f * config.weight_heading_error;
        sd->Q_diag[2] = 2.0f * config.weight_velocity;
        sd->Q_diag[3] = 2.0f * config.weight_lateral_velocity;
        sd->Q_diag[4] = 2.0f * config.weight_yaw_rate;
        sd->Q_diag[IDX_DELTA_ACTUAL] = 2.0f * w_delta_actual;
        sd->Q_diag[IDX_DRATE_PREV] = 2.0f * w_steer_jerk;
        sd->Q_diag[IDX_ACCEL_PREV] = 2.0f * w_accel_rate;

        /* Apply cross-call scaling for step 0 (jerk/rate penalties) */
        if (k == 0) {
            sd->Q_diag[IDX_DRATE_PREV] = 2.0f * (w_steer_jerk * config.cross_call_rate_scale);
            sd->Q_diag[IDX_ACCEL_PREV] = 2.0f * (w_accel_rate * config.cross_call_rate_scale);
        }

        /* === q (8 elements): linear state cost (tracking references) === */
        sd->q[0] = -(sd->Q_diag[0] * reference_trajectory[k].reference_lateral_error);
        sd->q[1] = -(sd->Q_diag[1] * reference_trajectory[k].reference_heading_error);

        /* Curvature-based velocity limiting: cap reference velocity to
         * v_max(κ) = √(a_lat_max / |κ|). This prevents the MPC from targeting
         * corner speeds that exceed the tire's lateral grip envelope, reducing
         * understeer from Pacejka tire saturation. */
        {
            float v_ref_k = reference_trajectory[k].reference_velocity;
            float kappa_k = fabsf(reference_trajectory[k].path_curvature);
            if (kappa_k > 0.01f) {
                static float max_lat_accel = 0;
                static int lat_accel_cached = 0;
                if (!lat_accel_cached) {
                    const char *env_val = getenv("MPC_MAX_LAT_ACCEL");
                    max_lat_accel = env_val ? atof(env_val) : MPC_MAX_LAT_ACCEL_DEFAULT;
                    lat_accel_cached = 1;
                }
                float v_max_lat = sqrt(max_lat_accel / kappa_k);
                if (v_ref_k > v_max_lat) v_ref_k = v_max_lat;
            }

            sd->q[2] = -(sd->Q_diag[2] * v_ref_k);
        }

        sd->q[3] = -(sd->Q_diag[3] * reference_trajectory[k].reference_lateral_velocity);
        sd->q[4] = -(sd->Q_diag[4] * reference_trajectory[k].reference_yaw_rate);

        /* δ_actual reference: feedforward steering δ_ff = atan(L*κ) */
        {
            float kappa_k = reference_trajectory[k].path_curvature;
            float delta_ff_k = atan(vp.wheelbase_meters * kappa_k);
            sd->q[IDX_DELTA_ACTUAL] = -(sd->Q_diag[IDX_DELTA_ACTUAL] * delta_ff_k);
        }
        sd->q[IDX_DRATE_PREV] = 0;  /* No tracking ref for δ̇_prev */
        sd->q[IDX_ACCEL_PREV] = 0;  /* No tracking ref for a_prev */

        /* === R_diag (2 elements): control cost === */
        /* R[0]: weight on |δ̇|² = effort + jerk penalty */
        sd->R_diag[0] = 2.0f * (w_steer_rate_eff + w_steer_jerk);
        /* R[1]: weight on |a|² = effort + rate penalty */
        sd->R_diag[1] = 2.0f * (w_accel_eff + w_accel_rate);

        if (k == 0) {
            sd->R_diag[0] = 2.0f * (w_steer_rate_eff + (w_steer_jerk * config.cross_call_rate_scale));
            sd->R_diag[1] = 2.0f * (w_accel_eff + (w_accel_rate * config.cross_call_rate_scale));
        }

        /* r: no constant control bias */
        sd->r[0] = 0;
        sd->r[1] = 0;

        /* === Cross-cost N (8×2) === */
        /* N[6][0]: couples δ̇_prev (x[6]) with δ̇ (u[0]) — steering jerk */
        sd->N[IDX_DRATE_PREV][0] = -(2.0f * w_steer_jerk);
        /* N[7][1]: couples a_prev (x[7]) with a (u[1]) — accel rate */
        sd->N[IDX_ACCEL_PREV][1] = -(2.0f * w_accel_rate);

        if (k == 0) {
            sd->N[IDX_DRATE_PREV][0] = -(2.0f * (w_steer_jerk * config.cross_call_rate_scale));
            sd->N[IDX_ACCEL_PREV][1] = -(2.0f * (w_accel_rate * config.cross_call_rate_scale));
        }

        /* === State bounds (8 elements) === */

        /* Initialize soft constraint weights to 0 (hard by default) */
        for (int s = 0; s < NX_AUG; s++)
            sd->x_soft_weight[s] = 0;

        /* Runtime overrides via env vars: WALL_SOFT_K, WALL_END, WALL_MARGIN, WALL_STRIDE.
         * Cached after first call for performance. */
        float wall_soft_k = WALL_SOFT_STIFFNESS_DEFAULT;
        float wall_margin = WALL_MARGIN_DEFAULT;
        int wall_end = WALL_CONSTRAINT_END_DEFAULT;
        int wall_stride = WALL_CONSTRAINT_STRIDE_DEFAULT;
        {
            static int initialized = 0;
            static float env_soft_k = 0;
            static float env_wall_margin = 0;
            static int env_wall_end = 0;
            static int env_wall_stride = 0;

            if (!initialized) {
                wall_soft_k = env_soft_k = get_env_float("WALL_SOFT_K", WALL_SOFT_STIFFNESS_DEFAULT);
                wall_margin = env_wall_margin = get_env_float("WALL_MARGIN", WALL_MARGIN_DEFAULT);
                env_wall_end = get_env_int("WALL_END", WALL_CONSTRAINT_END_DEFAULT);
                env_wall_stride = get_env_int("WALL_STRIDE", WALL_CONSTRAINT_STRIDE_DEFAULT);
                if (env_wall_stride < 1) env_wall_stride = 1;

                initialized = 1;
            }

            wall_soft_k = env_soft_k;
            wall_end = env_wall_end;
            wall_margin = env_wall_margin;
            wall_stride = env_wall_stride;
            if (wall_stride < 1) wall_stride = 1;
        }

        /* e_y: wall constraints (near-term: steps START..END, every STRIDE) */
        int wall_active = (k >= WALL_CONSTRAINT_START) &&
                          (k <= wall_end) &&
                          ((k - WALL_CONSTRAINT_START) % wall_stride == 0);

        if (wall_active &&
            reference_trajectory[k].left_wall_bound < 4.0f &&
            reference_trajectory[k].right_wall_bound < 4.0f) {
            sd->x_lb[0] = -(reference_trajectory[k].right_wall_bound - wall_margin);
            sd->x_ub[0] = reference_trajectory[k].left_wall_bound - wall_margin;
            /* Use soft constraint for walls (0 = hard, >0 = soft stiffness) */
            sd->x_soft_weight[0] = wall_soft_k;
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
        sd->x_lb[IDX_DELTA_ACTUAL] = -(vp.max_steering_angle);
        sd->x_ub[IDX_DELTA_ACTUAL] = vp.max_steering_angle;

        /* States 6-7 (prev controls): unconstrained */
        sd->x_lb[IDX_DRATE_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_DRATE_PREV] = BIG_BOUND;
        sd->x_lb[IDX_ACCEL_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_ACCEL_PREV] = BIG_BOUND;

        /* === Control bounds === */
        /* u[0] = δ̇: steering RATE limit (the key benefit of 8-state!) */
        sd->u_lb[0] = -(MAX_STEERING_RATE);
        sd->u_ub[0] = MAX_STEERING_RATE;

        /* u[1] = acceleration: speed-dependent power limit (v_switch model).
         * Above v_switch=7.319 m/s, the motor power is constant, so:
         *   a_max_eff = a_max * v_switch / v
         * This matches the f1tenth gym's STDynamicsModel exactly. */
        {
            float v_ref_k = reference_trajectory[k].reference_velocity;
            float a_max = vp.max_acceleration;
            float a_min = vp.min_acceleration;

            if (v_ref_k > V_SWITCH) {
                /* a_max_eff = a_max * v_switch / v_ref */
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

    terminal_Q[0] = 2.0f * config.weight_lateral_error;
    terminal_Q[1] = 2.0f * config.weight_heading_error;
    terminal_Q[2] = 2.0f * config.weight_velocity;
    terminal_Q[3] = 2.0f * config.weight_lateral_velocity;
    terminal_Q[4] = 2.0f * config.weight_yaw_rate;
    terminal_Q[IDX_DELTA_ACTUAL] = 2.0f * w_delta_actual;
    /* No jerk/rate penalty on terminal prev controls (Q[6:7] = 0) */

    /* Terminal q: tracking at last reference */
    if (N > 0) {
        terminal_q[0] = -(terminal_Q[0] * reference_trajectory[N-1].reference_lateral_error);
        terminal_q[1] = -(terminal_Q[1] * reference_trajectory[N-1].reference_heading_error);
        /* Apply curvature-based velocity limit to terminal reference too */
        {
            float v_ref_term = reference_trajectory[N-1].reference_velocity;
            float kappa_term = fabsf(reference_trajectory[N-1].path_curvature);
            if (kappa_term > 0.01f) {
                static float max_lat_accel_t = 0;
                static int lat_accel_t_cached = 0;
                if (!lat_accel_t_cached) {
                    const char *env_val = getenv("MPC_MAX_LAT_ACCEL");
                    max_lat_accel_t = env_val ? atof(env_val) : MPC_MAX_LAT_ACCEL_DEFAULT;
                    lat_accel_t_cached = 1;
                }
                float v_max_lat_t = sqrtf(max_lat_accel_t / kappa_term);
                if (v_ref_term > v_max_lat_t) v_ref_term = v_max_lat_t;
            }
            terminal_q[2] = -(terminal_Q[2] * v_ref_term);
        }
        terminal_q[3] = -(terminal_Q[3] * reference_trajectory[N-1].reference_lateral_velocity);
        terminal_q[4] = -(terminal_Q[4] * reference_trajectory[N-1].reference_yaw_rate);
        /* δ_actual terminal: track feedforward */
        {
            float kappa_N = reference_trajectory[N-1].path_curvature;
            float delta_ff_N = atan(vp.wheelbase_meters * kappa_N);
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
    /* Warm-start: reuse ADMM state unless curvature changed drastically.
     * With float32 precision, convergence is more reliable, so we
     * don't invalidate on non-convergence (matching FPGA OPT-3). */
    {
        float kappa_diff = fabsf(cur_curvature - warm_start_prev_curvature);
        if (!admm_state.initialized || kappa_diff > 0.5f) {
            riccati_admm_state_init(&admm_state);
        }
    }
    warm_start_prev_curvature = cur_curvature;

    /* ---------------------------------------------------------------
     * Step 5: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */
    RiccatiAdmmConfig_t solver_config;
    riccati_admm_config_init(&solver_config);
    solver_config.max_iterations = (int)config.max_solver_iterations;
    /* Pass through convergence tolerance from MPC config */
    solver_config.tolerance = config.solver_convergence_tolerance;
    /* Environment variable overrides for solver tuning (cached after first call) */
    {
        static int solver_env_cached = 0;
        static float cached_tol = 0;
        static float cached_rho = 0;
        static float cached_rho_u = 0;
        static float cached_alpha = 0;
        static int cached_max_iter = 0;
        static int has_tol = 0, has_rho = 0, has_rho_u = 0, has_alpha = 0, has_max_iter = 0;

    if (!solver_env_cached) {

        has_tol      = get_env_float_if_exists("MPC_SOLVER_TOL", &cached_tol);
        has_rho      = get_env_float_if_exists("MPC_SOLVER_RHO", &cached_rho);
        has_rho_u    = get_env_float_if_exists("MPC_SOLVER_RHO_U", &cached_rho_u);
        has_alpha    = get_env_float_if_exists("MPC_SOLVER_ALPHA", &cached_alpha);
        has_max_iter = get_env_int_if_exists("MPC_SOLVER_MAX_ITER", &cached_max_iter);

        solver_env_cached = 1;
    }

    if (has_tol)      solver_config.tolerance = cached_tol;
    if (has_rho)      solver_config.rho = cached_rho;
    if (has_rho_u)    solver_config.rho_u = cached_rho_u;
    if (has_alpha)    solver_config.alpha = cached_alpha;
    if (has_max_iter) solver_config.max_iterations = cached_max_iter;

    }

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
    float delta_rate = admm_state.z_u[0][0];
    float accel = admm_state.z_u[0][1];

    /* Compute steering angle command: δ_actual + dt * δ̇ */
    float delta_cmd = actual_steering_angle + config.time_step * delta_rate;

    /* Clamp to physical steering limits */
    if (delta_cmd > vp.max_steering_angle)
        delta_cmd = vp.max_steering_angle;
    if (delta_cmd < -vp.max_steering_angle)
        delta_cmd = -vp.max_steering_angle;

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
