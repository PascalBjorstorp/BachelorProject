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

/** Augmented state dimension (Frenet + previous control) */
#define NX_AUG 7

/** Control dimension (delta, acceleration) */
#define NU 2

/** Maximum horizon steps */
#ifdef MPC_HLS_TARGET
#define MAX_HORIZON 20
#else
#define MAX_HORIZON 50
#endif

/** Big number for unconstrained states */
#define BIG_BOUND FP_CONST(100.0)

/** Minimum linearization velocity (same as condensed approach) */
#define MIN_LINEARIZATION_VELOCITY FP_CONST(2.0)

/** A-row stability limit (same as condensed approach) */
#define STABILITY_LIMIT FP_CONST(0.95)

/** Wall constraint margin (same as condensed approach) */
#define WALL_MARGIN FP_CONST(0.10)

/** Match PG's sparse wall constraints: start at step 2, every 3 steps */
#define WALL_CONSTRAINT_START  2
#define WALL_CONSTRAINT_STRIDE 3

/** Maximum steering change per PREDICTION step (2.85 rad/s × 0.05s) */
#define MAX_STEER_CHANGE_PER_STEP FP_CONST(0.1425)

/** Maximum steering change per CONTROL interval (2.85 rad/s × 0.005s) */
#define MAX_STEER_CHANGE_PER_CALL FP_CONST(0.01425)

/** v_switch: power-limited acceleration threshold [m/s] (matches gym) */
#define V_SWITCH FP_CONST(7.319)

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

static MpcConfiguration_t config;
static int initialized = 0;
static ControlInput_t prev_control;
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

    /* State tracking weights (Frenet frame)
     * Matched to the working condensed MPC values:
     * - Lateral error dominates (50): strong lane-keeping
     * - Heading error moderate (5): prevents over-correction
     * - Velocity tracked (2.0): reasonable speed following
     * - vy/omega dampened (5.0): side-slip/yaw suppression */
    cfg.weight_lateral_error    = FP_CONST(100.0);
    cfg.weight_heading_error    = FP_CONST(200.0);
    cfg.weight_velocity         = FP_CONST(3.0);
    cfg.weight_lateral_velocity = FP_CONST(10.0);
    cfg.weight_yaw_rate         = FP_CONST(10.0);

    /* Control effort weights
     * Critical: R_steer = 0.5 provides Hessian regularization.
     * With R=0.01, the Riccati gain K ≈ -(B^T P B)^{-1} G is
     * hypersensitive to state changes, causing sign flips. */
    cfg.weight_steering_effort      = FP_CONST(0.5);
    cfg.weight_acceleration_effort  = FP_CONST(0.05);

    /* Control rate weights
     * With the trust region on step 0 preventing inter-call oscillation,
     * the rate penalty can be moderate (2.0 instead of 10.0).
     * This allows faster steering response on tight corners while
     * maintaining smooth intra-horizon control sequences. */
    cfg.weight_steering_rate        = FP_CONST(5.0);
    cfg.weight_acceleration_rate    = FP_CONST(0.2);

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
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
    initialized = 1;
}

static void mpc_riccati_reset(void)
{
    prev_control.steering_angle_radians = 0;
    prev_control.acceleration_meters_per_second_squared = 0;
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = 0;
}

void mpc_set_actual_previous_control(const ControlInput_t *actual)
{
    if (actual) {
        prev_control = *actual;
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
     * Step 2: Build augmented per-step data
     *
     * Augmented state: x_aug = [e_y, e_psi, vx, vy, omega, δ_prev, a_prev]
     * Control: u = [δ, a] (actual control, NOT increment)
     *
     * Augmented dynamics:
     *   x_aug_{k+1} = A_aug * x_aug_k + B_aug * u_k
     *   where A_aug = [A_frenet  0(5x2)]   B_aug = [B_frenet]
     *                 [0(2x5)    0(2x2)]            [I(2x2)  ]
     *
     * Because u_{k} directly gives u_prev for step k+1:
     *   delta_prev_{k+1} = u_k[0]  (steering command becomes previous)
     *   accel_prev_{k+1} = u_k[1]
     *
     * Stage cost:
     *   (x_frenet - ref)^T Q (x_frenet - ref)       [tracking]
     *   + w_effort * u^T R_eff u                      [effort]
     *   + w_rate * (u - x_aug[5:6])^T R_rate (u - x_aug[5:6])  [rate]
     *
     * In the standard Riccati form: l = 0.5 x^T Q_m x + q^T x + 0.5 u^T R_m u + r^T u + x^T N u
     *
     * Q_m (augmented diagonal):
     *   [2*w_lateral, 2*w_heading, 2*w_vel, 2*w_latvel, 2*w_yaw, 2*w_rate_steer, 2*w_rate_accel]
     * q_m: [-2*Q*ref, 0, 0] (tracking linear term)
     *
     * R_m (diagonal): [2*(w_effort_steer + w_rate_steer), 2*(w_effort_accel + w_rate_accel)]
     * r_m: [0, 0] (no constant control bias)
     *
     * Cross-cost N (7×2):
     *   N[5][0] = -2 * w_rate_steer  (couples delta_prev with delta)
     *   N[6][1] = -2 * w_rate_accel  (couples accel_prev with accel)
     *   (From expanding: w_rate*(u - u_prev)^2: cross-term is -2*w_rate*u_prev*u)
     * --------------------------------------------------------------- */

    /* Precompute cost weights */
    fixed_point_t w_steer_eff = config.weight_steering_effort;
    fixed_point_t w_accel_eff = config.weight_acceleration_effort;
    fixed_point_t w_steer_rate = config.weight_steering_rate;
    fixed_point_t w_accel_rate = config.weight_acceleration_rate;

    /* Build per-step data array */
    RiccatiStepData_t step_data[MAX_HORIZON];
    memset(step_data, 0, sizeof(step_data));

    for (int k = 0; k < N; k++) {
        RiccatiStepData_t *sd = &step_data[k];

        /* Augmented A matrix (7×7) */
        /* Top-left 5×5: Frenet A with per-step curvature correction */
        for (int i = 0; i < 5; i++)
            for (int j = 0; j < 5; j++)
                sd->A[i][j] = A_base[i][j];

        /* A[1][2] varies with per-step curvature */
        sd->A[1][2] = fp_neg(fp_mul(config.time_step_seconds,
            reference_trajectory[k].path_curvature_radians_per_meter));

        /* Bottom-right 2×2: zeros (u_prev is overwritten by B_aug) */
        /* Already zero from memset */

        /* Augmented B matrix (7×2) */
        for (int i = 0; i < 5; i++)
            for (int a = 0; a < 2; a++)
                sd->B[i][a] = B_base[i][a];

        /* B1: ZOH correction — DISABLED for testing.
         * TODO: re-enable after other changes are validated.
         * sd->B[1][0] = fp_mul(fp_div(config.time_step_seconds, FP_TWO), B_base[4][0]); */

        /* u_prev_{k+1} = u_k: identity block */
        sd->B[5][0] = FP_ONE;  /* delta_prev = delta */
        sd->B[6][1] = FP_ONE;  /* accel_prev = accel */

        /* Q_diag: tracking weights (doubled for 0.5*x^T*Q*x convention)
         * plus rate weights on the augmented u_prev states.
         *
         * B2: Discount removed (γ=1.0). The exponential discount caused
         * a 7.4× discontinuity with the terminal cost and made the MPC
         * "give up" on tracking at mid-horizon. Consistent weighting
         * throughout the horizon improves heading error significantly.
         */
        sd->Q_diag[0] = fp_mul(FP_TWO, config.weight_lateral_error);
        sd->Q_diag[1] = fp_mul(FP_TWO, config.weight_heading_error);
        sd->Q_diag[2] = fp_mul(FP_TWO, config.weight_velocity);
        sd->Q_diag[3] = fp_mul(FP_TWO, config.weight_lateral_velocity);
        sd->Q_diag[4] = fp_mul(FP_TWO, config.weight_yaw_rate);
        sd->Q_diag[5] = fp_mul(FP_TWO, w_steer_rate);
        sd->Q_diag[6] = fp_mul(FP_TWO, w_accel_rate);

        /* Apply cross-call scaling for step 0 */
        if (k == 0) {
            sd->Q_diag[5] = fp_mul(FP_TWO, fp_mul(w_steer_rate, config.cross_call_rate_scale));
            sd->Q_diag[6] = fp_mul(FP_TWO, fp_mul(w_accel_rate, config.cross_call_rate_scale));
        }

        /* q: linear state cost (tracking references) — discounted like Q */
        sd->q[0] = fp_neg(fp_mul(sd->Q_diag[0], reference_trajectory[k].reference_lateral_error_meters));
        sd->q[1] = fp_neg(fp_mul(sd->Q_diag[1], reference_trajectory[k].reference_heading_error_radians));
        sd->q[2] = fp_neg(fp_mul(sd->Q_diag[2], reference_trajectory[k].reference_velocity_meters_per_second));
        sd->q[3] = fp_neg(fp_mul(sd->Q_diag[3], reference_trajectory[k].reference_lateral_velocity_meters_per_second));
        sd->q[4] = fp_neg(fp_mul(sd->Q_diag[4], reference_trajectory[k].reference_yaw_rate_radians_per_second));
        sd->q[5] = 0;  /* No tracking reference for u_prev */
        sd->q[6] = 0;

        /* R_diag: control effort + rate weights (doubled for 0.5*u^T*R*u convention)
         *
         * The trust region on step 0 (see control bounds below) limits the
         * inter-call steering change to the physical actuator rate. This
         * allows using low R for responsive Riccati gains — the solver
         * produces the optimal control for tracking, and the trust region
         * ensures the output changes at most at the actuator rate per call. */
        sd->R_diag[0] = fp_mul(FP_TWO, fp_add(w_steer_eff, w_steer_rate));
        sd->R_diag[1] = fp_mul(FP_TWO, fp_add(w_accel_eff, w_accel_rate));

        if (k == 0) {
            sd->R_diag[0] = fp_mul(FP_TWO, fp_add(w_steer_eff,
                fp_mul(w_steer_rate, config.cross_call_rate_scale)));
            sd->R_diag[1] = fp_mul(FP_TWO, fp_add(w_accel_eff,
                fp_mul(w_accel_rate, config.cross_call_rate_scale)));
        }

        /* B3+B4: Feedforward — DISABLED for testing.
         * TODO: re-enable after other changes are validated. */
        sd->r[0] = 0;
        sd->r[1] = 0;

        /* Cross-cost N: couples u_prev (x_aug[5:6]) with u */
        /* N[5][0] = -2 * w_rate_steer, N[6][1] = -2 * w_rate_accel */
        sd->N[5][0] = fp_neg(fp_mul(FP_TWO, w_steer_rate));
        sd->N[6][1] = fp_neg(fp_mul(FP_TWO, w_accel_rate));

        if (k == 0) {
            sd->N[5][0] = fp_neg(fp_mul(FP_TWO, fp_mul(w_steer_rate, config.cross_call_rate_scale)));
            sd->N[6][1] = fp_neg(fp_mul(FP_TWO, fp_mul(w_accel_rate, config.cross_call_rate_scale)));
        }

        /* State bounds */
        /* Frenet states: only e_y is bounded (walls) */
        /* Match PG's sparse wall constraints: only enforce at steps
         * k = 2, 5, 8, 11, 14, 17 (start=2, stride=3). This reduces
         * constraint density and matches the PG formulation. */
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

        for (int s = 1; s < NX_AUG; s++) {
            sd->x_lb[s] = -BIG_BOUND;
            sd->x_ub[s] = BIG_BOUND;
        }

        /* Control bounds: simple physical limits (A4/A5 disabled for testing) */
        sd->u_lb[0] = fp_neg(vp.maximum_steering_angle_radians);
        sd->u_ub[0] = vp.maximum_steering_angle_radians;
        sd->u_lb[1] = vp.minimum_acceleration_meters_per_second_squared;
        sd->u_ub[1] = vp.maximum_acceleration_meters_per_second_squared;
    }

    /* Terminal state cost (same Q as stage cost, no rate penalty on u_prev) */
    fixed_point_t terminal_Q[RICCATI_MAX_NX];
    fixed_point_t terminal_q[RICCATI_MAX_NX];
    memset(terminal_Q, 0, sizeof(terminal_Q));
    memset(terminal_q, 0, sizeof(terminal_q));

    terminal_Q[0] = fp_mul(FP_TWO, config.weight_lateral_error);
    terminal_Q[1] = fp_mul(FP_TWO, config.weight_heading_error);
    terminal_Q[2] = fp_mul(FP_TWO, config.weight_velocity);
    terminal_Q[3] = fp_mul(FP_TWO, config.weight_lateral_velocity);
    terminal_Q[4] = fp_mul(FP_TWO, config.weight_yaw_rate);
    /* No rate penalty on terminal u_prev (Q[5:6] = 0) */

    /* Terminal q: tracking at last reference */
    if (N > 0) {
        terminal_q[0] = -fp_mul(terminal_Q[0], reference_trajectory[N-1].reference_lateral_error_meters);
        terminal_q[1] = -fp_mul(terminal_Q[1], reference_trajectory[N-1].reference_heading_error_radians);
        terminal_q[2] = -fp_mul(terminal_Q[2], reference_trajectory[N-1].reference_velocity_meters_per_second);
        terminal_q[3] = -fp_mul(terminal_Q[3], reference_trajectory[N-1].reference_lateral_velocity_meters_per_second);
        terminal_q[4] = -fp_mul(terminal_Q[4], reference_trajectory[N-1].reference_yaw_rate_radians_per_second);
    }

    /* ---------------------------------------------------------------
     * Step 3: Build augmented initial state
     * --------------------------------------------------------------- */
    fixed_point_t x0[RICCATI_MAX_NX];
    memset(x0, 0, sizeof(x0));
    x0[0] = current_frenet_state->lateral_error_meters;
    x0[1] = current_frenet_state->heading_error_radians;
    x0[2] = current_frenet_state->longitudinal_velocity_meters_per_second;
    x0[3] = current_frenet_state->lateral_velocity_meters_per_second;
    x0[4] = current_frenet_state->yaw_rate_radians_per_second;
    x0[5] = prev_control.steering_angle_radians;       /* Previous steering */
    x0[6] = prev_control.acceleration_meters_per_second_squared;  /* Previous accel */

    /* ---------------------------------------------------------------
     * Step 4: Warm-start management
     * --------------------------------------------------------------- */
    fixed_point_t cur_curvature = reference_trajectory[0].path_curvature_radians_per_meter;
    /* Always cold-start: warm-start duals bias the solver toward saturation,
     * causing ±max_steer oscillation even with proper weights. With R=0.5 and
     * rate=10, the cold-start unconstrained Riccati produces near-optimal
     * solutions that need at most 1-5 ADMM iterations for constraint cleanup. */
    riccati_admm_state_init(&admm_state);
    warm_start_prev_curvature = cur_curvature;

    /* ---------------------------------------------------------------
     * Step 5: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */
    RiccatiAdmmConfig_t solver_config;
    riccati_admm_config_init(&solver_config);
    /* ADMM convergence tolerance: use the riccati_admm_config_init defaults
     * (0.1), NOT the MPC config tolerance (0.02). The MPC config tolerance
     * was designed for projected gradient which converges to tighter values.
     * ADMM with Q16.16 fixed-point needs more headroom.
     * Similarly, cap max iterations at 200 — ADMM doesn't benefit from
     * running 2000 iterations with adaptive rho, as rho can escalate and
     * make the solver do pure projection. */
    /* solver_config.tolerance and max_iterations stay at riccati_admm_config_init defaults */

    RiccatiSolution_t riccati_sol;
    memset(&riccati_sol, 0, sizeof(riccati_sol));

    RiccatiStatus_t rstatus = riccati_admm_solve(
        step_data, terminal_Q, terminal_q, x0,
        NX_AUG, NU, N,
        &solver_config, &admm_state, &riccati_sol);

    /* ---------------------------------------------------------------
     * Step 6: Extract first control from ADMM z_u (feasible projection)
     * --------------------------------------------------------------- */
    /* In ADMM, z_u is the constrained projection — always feasible.
     * The raw solution u may exceed bounds (unconstrained Riccati output).
     * Use z_u for the control output, which is the best feasible estimate. */
    ControlInput_t raw_control;
    raw_control.steering_angle_radians = admm_state.z_u[0][0];
    raw_control.acceleration_meters_per_second_squared = admm_state.z_u[0][1];

    ControlInput_t saturated = vehicle_model_saturate_control(&raw_control);

    result->optimal_control = saturated;
    result->iterations_used = (uint16_t)riccati_sol.iterations;
    result->final_cost = riccati_sol.primal_residual;

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

    prev_control = saturated;
    return result->solver_status;
}
