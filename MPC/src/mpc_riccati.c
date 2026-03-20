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

/** Wall constraint margin — default from latest best hardware tuning run.
 *  Override at runtime via WALL_MARGIN environment variable. */
#define WALL_MARGIN_DEFAULT FP_CONST(0.855)

/** Wall constraints: only first few horizon steps for near-term safety.
 *  Override at runtime via WALL_END environment variable.
 *  WALL_STRIDE controls step spacing (1=every step, 2=every other, etc.).
 *  Override at runtime via WALL_STRIDE environment variable. */
#define WALL_CONSTRAINT_START  1
#define WALL_CONSTRAINT_STRIDE_DEFAULT 3
#define WALL_CONSTRAINT_END_DEFAULT 20    /* last horizon step to constrain (0=disable) */

/** Soft wall constraint stiffness (0 = hard box constraint).
 *  When > 0, wall constraints use a quadratic penalty instead of hard clipping:
 *    g(z) = (k/2) * max(0, z - ub)^2 + (k/2) * max(0, lb - z)^2
 *  The ADMM z-update uses the proximal operator, allowing controlled violation.
 *  Higher k = stiffer (500+ approaches hard). Lower k = more flexible.
 *  Recommended: 200-500 for tight corridors, 0 for wide tracks.
 *  Override at runtime via WALL_SOFT_K environment variable. */
#define WALL_SOFT_STIFFNESS_DEFAULT FP_CONST(657.0)

/** v_switch: above this velocity, max acceleration = a_max * v_switch / v.
 *  From f1tenth gym STDynamicsModel: v_switch = 7.319 m/s.
 *  Models constant-power regime: P = F*v = const → a_max(v) ∝ 1/v. */
#define V_SWITCH FP_CONST(7.319)

/** Maximum lateral acceleration for curvature-based velocity limiting [m/s²].
 *  v_max(κ) = √(a_lat_max / |κ|), capping reference velocities in corners.
 *  Physically correct value: mu*g = 0.745 * 9.81 = 7.31 m/s².
 *  Override at runtime via MPC_MAX_LAT_ACCEL environment variable. */
#define MPC_MAX_LAT_ACCEL_DEFAULT FP_CONST(7.3078)

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MpcConfiguration_t config;
static int initialized = 0;
static ControlInput_t prev_control;
static fixed_point_t actual_steering_angle = 0;  /* Servo physical position */
static RiccatiAdmmState_t admm_state;
static fixed_point_t warm_start_prev_curvature = 0;


/*===========================================================================
 * Default Configuration
 *===========================================================================*/

MpcConfiguration_t get_default_configuration(void)
{
    MpcConfiguration_t cfg;

    cfg.prediction_horizon_steps = MPC_DEFAULT_PREDICTION_HORIZON;
    cfg.time_step_seconds = MPC_DEFAULT_TIME_STEP_SECONDS;

    /* State tracking weights (Frenet frame).
     * Defaults updated to latest best hardware tuning row (RND_76). */
    cfg.weight_lateral_error    = FP_CONST(5000.0);
    cfg.weight_heading_error    = FP_CONST(500.0);
    cfg.weight_velocity         = FP_CONST(165.0);
    cfg.weight_lateral_velocity = FP_CONST(10.0);
    cfg.weight_yaw_rate         = FP_CONST(5.0);

    /* Control effort weights */
    cfg.weight_steering_effort      = FP_CONST(0.18);
    cfg.weight_acceleration_effort  = FP_CONST(0.01);

    /* Control rate weights (W_JERK, W_ACCEL_RATE) */
    cfg.weight_steering_rate        = FP_CONST(0.15);
    cfg.weight_acceleration_rate    = FP_CONST(0.1);

    /* Cross-call rate scale: CONTROL_DT / PRED_DT = 0.005 / 0.02 = 0.25. */
    cfg.cross_call_rate_scale = FP_CONST(0.25);

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

        /* Horizon and prediction dt overrides (also used by sim/hardware node) */
        if ((env_val = getenv("HORIZON")) != NULL) {
            int h = atoi(env_val);
            if (h >= 1 && h <= MAX_HORIZON) cfg.prediction_horizon_steps = h;
        }
        if ((env_val = getenv("PRED_DT")) != NULL) {
            double dt = atof(env_val);
            cfg.time_step_seconds = DOUBLE_TO_FP(dt);
            /* Auto-update cross_call_rate_scale unless explicitly set.
             * CONTROL_DT = 0.005s (200 Hz), scale = CONTROL_DT / PRED_DT */
            if (getenv("MPC_CROSS_CALL_SCALE") == NULL)
                cfg.cross_call_rate_scale = DOUBLE_TO_FP(0.005 / dt);
        }
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
    initialized = 1;
}

static void mpc_riccati_reset(void)
{
    prev_control.steering_angle_radians = 0;
    prev_control.acceleration_meters_per_second_squared = 0;
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

    /* ---------------------------------------------------------------
     * State filter: EMA (exponential moving average) for noise rejection.
     * filtered = alpha * measured + (1 - alpha) * prev_filtered
     * alpha = 1.0 → no filtering (bypass), alpha = 0.5 → strong smoothing.
     * Override at runtime via MPC_EMA_ALPHA environment variable.
     * Default: 1.0 (disabled). Recommended for noisy sensors: 0.7-0.9.
     * --------------------------------------------------------------- */
    FrenetState_t filtered_state = *current_frenet_state;
    {
        static fixed_point_t ema_alpha = 0;
        static int ema_cached = 0;
        static FrenetState_t prev_filtered;
        static int prev_valid = 0;

        if (!ema_cached) {
            const char *env_val = getenv("MPC_EMA_ALPHA");
            ema_alpha = env_val ? DOUBLE_TO_FP(atof(env_val)) : FP_CONST(0.7);
            ema_cached = 1;
        }

        if (ema_alpha < FP_ONE && prev_valid) {
            fixed_point_t a = ema_alpha;
            fixed_point_t b = fp_sub(FP_ONE, a);
            filtered_state.lateral_error_meters = fp_add(
                fp_mul(a, current_frenet_state->lateral_error_meters),
                fp_mul(b, prev_filtered.lateral_error_meters));
            filtered_state.heading_error_radians = fp_add(
                fp_mul(a, current_frenet_state->heading_error_radians),
                fp_mul(b, prev_filtered.heading_error_radians));
            filtered_state.longitudinal_velocity_meters_per_second = fp_add(
                fp_mul(a, current_frenet_state->longitudinal_velocity_meters_per_second),
                fp_mul(b, prev_filtered.longitudinal_velocity_meters_per_second));
            filtered_state.lateral_velocity_meters_per_second = fp_add(
                fp_mul(a, current_frenet_state->lateral_velocity_meters_per_second),
                fp_mul(b, prev_filtered.lateral_velocity_meters_per_second));
            filtered_state.yaw_rate_radians_per_second = fp_add(
                fp_mul(a, current_frenet_state->yaw_rate_radians_per_second),
                fp_mul(b, prev_filtered.yaw_rate_radians_per_second));
        }
        prev_filtered = filtered_state;
        prev_valid = 1;
    }
    const FrenetState_t *frenet = &filtered_state;

    int N = config.prediction_horizon_steps;
    if (N > MAX_HORIZON) N = MAX_HORIZON;

    /* ---------------------------------------------------------------
     * Step 1: Prepare model constants for per-step linearization.
     *
     * Full Frenet linearization is computed per horizon point in the
     * loop below (A_step/B_step), using each step's curvature and
     * feedforward steering operating point.
     * --------------------------------------------------------------- */
    VehicleParameters_t vp = vehicle_model_get_parameters();
    fixed_point_t delta_clamp = vp.maximum_steering_angle_radians * 0.5f;

    FrenetState_t lin_state = *frenet;
    if (lin_state.longitudinal_velocity_meters_per_second < MIN_LINEARIZATION_VELOCITY)
        lin_state.longitudinal_velocity_meters_per_second = MIN_LINEARIZATION_VELOCITY;

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
    fixed_point_t w_delta_actual = FP_CONST(0.5);

    /* Build per-step data array */
    RiccatiStepData_t step_data[MAX_HORIZON];
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
        fixed_point_t kappa_k = reference_trajectory[k].path_curvature_radians_per_meter;

        ControlInput_t lin_control;
        lin_control.steering_angle_radians = fp_atan(fp_mul(vp.wheelbase_meters, kappa_k));
        if (lin_control.steering_angle_radians > delta_clamp)
            lin_control.steering_angle_radians = delta_clamp;
        if (lin_control.steering_angle_radians < fp_neg(delta_clamp))
            lin_control.steering_angle_radians = fp_neg(delta_clamp);
        lin_control.acceleration_meters_per_second_squared = 0;

        lin_state.longitudinal_velocity_meters_per_second =
            reference_trajectory[k].reference_velocity_meters_per_second;
        if (lin_state.longitudinal_velocity_meters_per_second < MIN_LINEARIZATION_VELOCITY)
            lin_state.longitudinal_velocity_meters_per_second = MIN_LINEARIZATION_VELOCITY;

        fixed_point_t A_step[5][5];
        fixed_point_t B_step[5][2];

        vehicle_model_compute_frenet_linearization(
            &lin_state, &lin_control,
            config.time_step_seconds,
            kappa_k,
            A_step, B_step);

        /* Stabilize fast dynamics (omega row = 4) per stage */
        {
            int row = 4;
            fixed_point_t abs_aii = fp_abs(A_step[row][row]);
            if (abs_aii > STABILITY_LIMIT) {
                fixed_point_t target = (A_step[row][row] < 0)
                    ? fp_neg(STABILITY_LIMIT) : STABILITY_LIMIT;
                fixed_point_t num = fp_sub(target, FP_ONE);
                fixed_point_t den = fp_sub(A_step[row][row], FP_ONE);
                if (den != 0) {
                    fixed_point_t scale = fp_div(num, den);
                    for (int j = 0; j < 5; j++) {
                        if (j != row) A_step[row][j] = fp_mul(A_step[row][j], scale);
                    }
                    B_step[row][0] = fp_mul(B_step[row][0], scale);
                    B_step[row][1] = fp_mul(B_step[row][1], scale);
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
        sd->A[IDX_DELTA_ACTUAL][IDX_DELTA_ACTUAL] = FP_ONE;

        /* Rows 6-7: already zeroed above */
        /* Cols 6-7: already zeroed above */

        /* === Augmented B matrix (8×2) === */

        /* Rows 0-4, col 0: already zeroed above */

        /* Rows 0-4, col 1: acceleration effect on dynamics (unchanged) */
        for (int i = 0; i < 5; i++)
            sd->B[i][1] = B_step[i][1];

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

        /* Curvature-based velocity limiting: cap reference velocity to
         * v_max(κ) = √(a_lat_max / |κ|). This prevents the MPC from targeting
         * corner speeds that exceed the tire's lateral grip envelope, reducing
         * understeer from Pacejka tire saturation. */
        {
            fixed_point_t v_ref_k = reference_trajectory[k].reference_velocity_meters_per_second;
            fixed_point_t kappa_k = fp_abs(reference_trajectory[k].path_curvature_radians_per_meter);
            if (kappa_k > FP_CONST(0.01)) {
                static fixed_point_t max_lat_accel = 0;
                static int lat_accel_cached = 0;
                if (!lat_accel_cached) {
                    const char *env_val = getenv("MPC_MAX_LAT_ACCEL");
                    max_lat_accel = env_val ? DOUBLE_TO_FP(atof(env_val)) : MPC_MAX_LAT_ACCEL_DEFAULT;
                    lat_accel_cached = 1;
                }
                fixed_point_t v_max_lat = fp_sqrt(fp_div(max_lat_accel, kappa_k));
                if (v_ref_k > v_max_lat) v_ref_k = v_max_lat;
            }

            sd->q[2] = fp_neg(fp_mul(sd->Q_diag[2], v_ref_k));
        }

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

        /* Initialize soft constraint weights to 0 (hard by default) */
        for (int s = 0; s < NX_AUG; s++)
            sd->x_soft_weight[s] = 0;

        /* Runtime overrides via env vars: WALL_SOFT_K, WALL_END, WALL_MARGIN, WALL_STRIDE.
         * Cached after first call for performance. */
        fixed_point_t wall_soft_k = WALL_SOFT_STIFFNESS_DEFAULT;
        fixed_point_t wall_margin = WALL_MARGIN_DEFAULT;
        int wall_end = WALL_CONSTRAINT_END_DEFAULT;
        int wall_stride = WALL_CONSTRAINT_STRIDE_DEFAULT;
        {
            static int env_checked = 0;
            static fixed_point_t env_soft_k = 0;
            static fixed_point_t env_wall_margin = 0;
            static int env_wall_end = 0;
            static int env_wall_stride = 0;
            if (!env_checked) {
                const char *env_val = getenv("WALL_SOFT_K");
                if (env_val) env_soft_k = (fixed_point_t)atof(env_val);
                else env_soft_k = WALL_SOFT_STIFFNESS_DEFAULT;
                env_val = getenv("WALL_END");
                if (env_val) env_wall_end = atoi(env_val);
                else env_wall_end = WALL_CONSTRAINT_END_DEFAULT;
                env_val = getenv("WALL_MARGIN");
                if (env_val) env_wall_margin = DOUBLE_TO_FP(atof(env_val));
                else env_wall_margin = WALL_MARGIN_DEFAULT;
                env_val = getenv("WALL_STRIDE");
                if (env_val) env_wall_stride = atoi(env_val);
                else env_wall_stride = WALL_CONSTRAINT_STRIDE_DEFAULT;
                env_checked = 1;
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
            reference_trajectory[k].left_wall_bound_meters < FP_CONST(4.0) &&
            reference_trajectory[k].right_wall_bound_meters < FP_CONST(4.0)) {
            sd->x_lb[0] = fp_neg(fp_sub(reference_trajectory[k].right_wall_bound_meters, wall_margin));
            sd->x_ub[0] = fp_sub(reference_trajectory[k].left_wall_bound_meters, wall_margin);
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
        /* Apply curvature-based velocity limit to terminal reference too */
        {
            fixed_point_t v_ref_term = reference_trajectory[N-1].reference_velocity_meters_per_second;
            fixed_point_t kappa_term = fp_abs(reference_trajectory[N-1].path_curvature_radians_per_meter);
            if (kappa_term > FP_CONST(0.01)) {
                static fixed_point_t max_lat_accel_t = 0;
                static int lat_accel_t_cached = 0;
                if (!lat_accel_t_cached) {
                    const char *env_val = getenv("MPC_MAX_LAT_ACCEL");
                    max_lat_accel_t = env_val ? DOUBLE_TO_FP(atof(env_val)) : MPC_MAX_LAT_ACCEL_DEFAULT;
                    lat_accel_t_cached = 1;
                }
                fixed_point_t v_max_lat_t = fp_sqrt(fp_div(max_lat_accel_t, kappa_term));
                if (v_ref_term > v_max_lat_t) v_ref_term = v_max_lat_t;
            }
            terminal_q[2] = -fp_mul(terminal_Q[2], v_ref_term);
        }
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
    x0[0] = frenet->lateral_error_meters;
    x0[1] = frenet->heading_error_radians;
    x0[2] = frenet->longitudinal_velocity_meters_per_second;
    x0[3] = frenet->lateral_velocity_meters_per_second;
    x0[4] = frenet->yaw_rate_radians_per_second;
    x0[IDX_DELTA_ACTUAL] = actual_steering_angle;      /* Physical servo position */
    x0[IDX_DRATE_PREV] = prev_control.steering_angle_radians;  /* Previous δ̇ command */
    x0[IDX_ACCEL_PREV] = prev_control.acceleration_meters_per_second_squared;

    /* ---------------------------------------------------------------
     * Step 4: Warm-start management
     * --------------------------------------------------------------- */
    fixed_point_t cur_curvature = reference_trajectory[0].path_curvature_radians_per_meter;
    /* Warm-start: reuse ADMM state unless curvature changed drastically.
     * With float32 precision, convergence is more reliable, so we
     * don't invalidate on non-convergence (matching FPGA OPT-3). */
    {
        fixed_point_t kappa_diff = fp_abs(fp_sub(cur_curvature, warm_start_prev_curvature));
        if (!admm_state.initialized || kappa_diff > FP_CONST(0.5)) {
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
    /* Pass through convergence tolerance from MPC config */
    solver_config.tolerance = config.solver_convergence_tolerance;
    /* Environment variable overrides for solver tuning (cached after first call) */
    {
        static int solver_env_cached = 0;
        static fixed_point_t cached_tol = 0;
        static fixed_point_t cached_rho = 0;
        static fixed_point_t cached_rho_u = 0;
        static fixed_point_t cached_alpha = 0;
        static int cached_max_iter = 0;
        static int has_tol = 0, has_rho = 0, has_rho_u = 0, has_alpha = 0, has_max_iter = 0;

        if (!solver_env_cached) {
            const char *env_val;
            if ((env_val = getenv("TOL")) != NULL) {
                cached_tol = DOUBLE_TO_FP(atof(env_val)); has_tol = 1;
            }
            if ((env_val = getenv("RHO")) != NULL) {
                cached_rho = DOUBLE_TO_FP(atof(env_val)); has_rho = 1;
            }
            if ((env_val = getenv("RHO_U")) != NULL) {
                cached_rho_u = DOUBLE_TO_FP(atof(env_val)); has_rho_u = 1;
            }
            if ((env_val = getenv("ALPHA")) != NULL) {
                cached_alpha = DOUBLE_TO_FP(atof(env_val)); has_alpha = 1;
            }
            if ((env_val = getenv("MAX_ITER")) != NULL) {
                cached_max_iter = atoi(env_val); has_max_iter = 1;
            }
            solver_env_cached = 1;
        }
        if (has_tol) solver_config.tolerance = cached_tol;
        if (has_rho) solver_config.rho = cached_rho;
        if (has_rho_u) solver_config.rho_u = cached_rho_u;
        if (has_alpha) solver_config.alpha = cached_alpha;
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

    ControlInput_t raw_control;
    raw_control.steering_angle_radians = delta_cmd;
    raw_control.acceleration_meters_per_second_squared = accel;

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
    prev_control.steering_angle_radians = delta_rate;
    prev_control.acceleration_meters_per_second_squared = saturated.acceleration_meters_per_second_squared;
    return result->solver_status;
}
