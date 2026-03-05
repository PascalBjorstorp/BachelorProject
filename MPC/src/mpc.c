/**
 * @file mpc.c
 * @brief Model Predictive Control Implementation
 *
 * Implements the MPC controller for F1/10th autonomous vehicle.
 * Uses quadratic programming to optimize control inputs over a
 * prediction horizon while respecting vehicle constraints.
 *
 * Algorithm Overview:
 * 1. Linearize vehicle model around current state
 * 2. Build QP cost matrices from tracking error and control effort
 * 3. Build QP constraint matrices from actuator limits
 * 4. Solve QP using projected gradient descent
 * 5. Extract first control input from optimal sequence
 *
 * All arithmetic uses Q16.16 fixed-point for FPGA compatibility.
 */

#include "mpc.h"
#include "fp_math.h"
#include "qp_solver.h"
#include "vehicle_model.h"
#include <string.h>
#include <stdio.h>
#ifndef MPC_HLS_TARGET
#include <stdlib.h>
#endif

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/** Number of states in Frenet vehicle model: [e_y, e_psi, v_x, v_y, omega] */
#define STATE_DIMENSION 5

/** Number of control inputs: [steering, acceleration] */
#define CONTROL_DIMENSION 2

/** Maximum prediction horizon supported */
#define MAXIMUM_HORIZON_STEPS 50

/*===========================================================================
 * Module State (Static)
 *===========================================================================*/

/** Current MPC configuration */
static MpcConfiguration_t current_configuration;

/** Flag indicating MPC has been initialized */
static int mpc_initialized_flag = 0;

/** Previous control input (for rate limiting) */
static ControlInput_t previous_control_input;

/** Previous QP solution for warm-starting (shifted by one step) */
static fixed_point_t warm_start_variables[QP_MAXIMUM_VARIABLES];
static int warm_start_available = 0;

/** Previous reference curvature (first horizon step) for warm-start invalidation.
 *  When the curvature changes significantly between calls (e.g., straight→curve
 *  transition), the previous solution is a poor initial guess and warm-starting
 *  biases the solver toward the stale straight-line plan. */
static fixed_point_t warm_start_prev_curvature = 0;

/** Curvature-change threshold for warm-start invalidation.
 *  If |kappa_new - kappa_old| exceeds this, cold-start the solver.
 *  0.5 rad/m ≈ 2m turning radius change — catches straight→curve transitions
 *  while keeping warm-start active during steady curves. */
#define WARM_START_CURVATURE_THRESHOLD FP_CONST(0.1)

/** Phi matrices and free-response e_y for wall constraint building.
 *  Populated by build_qp_from_prediction(), used by build_wall_constraints(). */
static fixed_point_t saved_Phi[MAXIMUM_HORIZON_STEPS][STATE_DIMENSION][CONTROL_DIMENSION];
static fixed_point_t saved_free_response_ey[MAXIMUM_HORIZON_STEPS];



/** Full-precision (int64) gradient vector for auto-scaling.
 *  The gradient can exceed INT32 range due to large Phi*Q*d products.
 *  Stored in int64 so auto-scaling can shift BEFORE clamping to int32. */
static int64_t gradient_int64[QP_MAXIMUM_VARIABLES];

/** Full-precision (int64) Hessian staging buffer for auto-scaling.
 *  With large horizons (N≥20), accumulated Phi^T·Q·Phi products exceed INT32
 *  range. Previous approach clamped to INT32 before scaling, destroying relative
 *  magnitudes and causing solver divergence. This buffer preserves full int64
 *  precision until the uniform auto-scale shift is computed. */
static int64_t hessian_int64[QP_MAXIMUM_VARIABLES * QP_MAXIMUM_VARIABLES];
static int saved_horizon_steps = 0;

/*===========================================================================
 * Default Configuration
 *===========================================================================*/

static MpcConfiguration_t get_default_configuration(void)
{
    MpcConfiguration_t config;

    /* Prediction horizon and timing */
    config.prediction_horizon_steps = MPC_DEFAULT_PREDICTION_HORIZON;
    config.time_step_seconds = MPC_DEFAULT_TIME_STEP_SECONDS;

    /* State tracking weights (Frenet frame)
     *
     * With the condensed MPC formulation, each weight is effectively
     * amplified by the prediction horizon (N=10 steps). So moderate
     * values here produce strong tracking behavior.
     *
     * In Frenet coordinates, the state is [e_y, e_psi, v_x, v_y, w].
     * Lateral error (e_y) and heading error (e_psi) replace global XY.
     * Position and heading tracking now work together naturally.
     *
     * NOTE: All weights scaled to prevent Q16.16 overflow in
     * condensed Hessian construction (Phi^T Q Phi accumulates large
     * products over 20-step horizon).
     *
     * Weight rationale:
     *   Q_lat / R_steer ratio controls tracking-vs-damping tradeoff.
     *   Q_vel must be large enough for the MPC accel output to drive
     *   v_cmd meaningfully (the sim uses v_cmd = vx + a*T, not raw raceline v).
     */
    config.weight_lateral_error  = FP_CONST(50.0);               /* strong lateral tracking */
    config.weight_heading_error  = FP_CONST(5.0);                 /* reduced vs lateral — prevents heading correction from overriding lateral position */
    config.weight_velocity   = FP_CONST(2.0);                   /* velocity tracking */
    config.weight_lateral_velocity = FP_CONST(5.0);               /* strong sideslip penalty — low rear grip (C_Sr=3.32) requires aggressive damping */
    config.weight_yaw_rate   = FP_CONST(5.0);                    /* strong yaw damping — prevents oversteer spinout with measured tire params */

    /* Control effort weights — provide Hessian regularization and damping.
     * With N=20 condensed QP, regularization is needed for numerical
     * stability (condition number).  R_steer=0.5 bounds kappa(H) ≈ 1500.
     * The ε·I diagonal regularization provides an additional floor. */
    config.weight_steering_effort  = FP_CONST(0.5);     /* regularize steering + condition H */
    config.weight_acceleration_effort     = FP_CONST(0.1);    /* penalize large accelerations */

    /* Control rate weights — tuned so MPC naturally limits steering rate
     * without relying on the external rate clamp.  A high steering-rate
     * penalty makes large step-to-step changes expensive, preventing the
     * bang-bang oscillation that occurs when the external clamp truncates
     * a multi-step plan. */
    config.weight_steering_rate  = FP_CONST(10.0);                   /* high values prevent bang-bang steering oscillation */
    config.weight_acceleration_rate     = FP_CONST(0.1);                    /* prevent acceleration chattering */

    /* Cross-call rate scale: applied to the rate penalty between successive
     * MPC calls (u[0] vs u_prev).  0.3 allows sufficient inter-call steering
     * changes while maintaining smooth transitions. */
    config.cross_call_rate_scale = FP_CONST(0.3);

    /* Solver parameters */
    config.maximum_solver_iterations = MPC_DEFAULT_MAXIMUM_ITERATIONS;
    config.solver_convergence_tolerance = MPC_DEFAULT_CONVERGENCE_TOLERANCE;

#ifndef MPC_HLS_TARGET
    /* Environment variable overrides for runtime tuning.
     * Example: MPC_W_LAT_ERROR=3.0 MPC_W_STEER_RATE=0.01 ros2 launch ...
     * Avoids rebuild for quick parameter exploration. */
    {
        const char *env_val;
        if ((env_val = getenv("MPC_W_LAT_ERROR")) != NULL)
            config.weight_lateral_error = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_HEADING")) != NULL)
            config.weight_heading_error = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_VELOCITY")) != NULL)
            config.weight_velocity = DOUBLE_TO_FP(atof(env_val));
        /* Note: weight_lateral_velocity and weight_yaw_rate are overridden
         * by speed-dependent blending in build_qp_from_prediction(),
         * so env var overrides for them are not supported. */
        if ((env_val = getenv("MPC_W_STEER_RATE")) != NULL)
            config.weight_steering_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_STEER_EFFORT")) != NULL)
            config.weight_steering_effort = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_ACCEL_RATE")) != NULL)
            config.weight_acceleration_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_CROSS_CALL_SCALE")) != NULL)
            config.cross_call_rate_scale = DOUBLE_TO_FP(atof(env_val));
    }
#endif

    return config;
}

/*===========================================================================
 * QP Problem Construction — Condensed MPC Formulation
 *===========================================================================
 *
 * Linearized vehicle model: x[k+1] = A x[k] + B u[k]
 *
 * Prediction over N steps from initial state x[0]:
 *   x[k] = A^k x[0] + sum_{j=0}^{k-1} A^{k-1-j} B u[j]
 *
 * Define Phi[m] = A^m * B  (how a control input propagates m steps later)
 *
 * The "lifted" mapping from control u[j] to state x[k] is Phi[k-1-j].
 *
 * Cost function:
 *   J = sum_{k=1}^{N} (x[k] - ref[k])^T Q (x[k] - ref[k])
 *     + sum_{k=0}^{N-1} u[k]^T R u[k]
 *     + sum_{k=0}^{N-1} w_rate (u[k] - u[k-1])^2
 *
 * After substituting the prediction into QP form (min 0.5 U^T H U + f^T U):
 *
 *   H = 2 * (Su^T Q_bar Su + R_bar + Rate)
 *   f = 2 * Su^T Q_bar d + f_rate
 *
 * where d[k] = A^{k+1} x[0] - ref[k] (free-response tracking error)
 */

/**
 * Clamp A[4][4] (yaw-rate eigenvalue) to the stability limit and
 * consistently scale the rest of row 4 to preserve dynamics ratios.
 *
 * At low speed, |A[4][4]| > 1 → unstable Phi propagation. This clamps
 * to [-limit, +limit] and scales A[4][2..3] and optionally B[4][0..1].
 *
 * @param A         5×5 state matrix (row 4 modified in-place)
 * @param B         5×2 input matrix (row 4 scaled if non-NULL)
 * @param limit     Stability clamp magnitude (e.g. 0.95)
 */
static void clamp_yaw_rate_row(fixed_point_t A[5][5],
                               fixed_point_t B[5][2],
                               fixed_point_t limit)
{
    fixed_point_t abs_a44 = fp_abs(A[4][4]);
    if (abs_a44 <= limit) return;

    fixed_point_t target = (A[4][4] < 0) ? fp_neg(limit) : limit;
    fixed_point_t den = fp_sub(A[4][4], FP_ONE);
    if (den == 0) { A[4][4] = target; return; }

    fixed_point_t scale = fp_div(fp_sub(target, FP_ONE), den);
    for (int j = 0; j < STATE_DIMENSION; j++)
    {
        if (j != 4) A[4][j] = fp_mul(A[4][j], scale);
    }
    if (B != NULL)
    {
        B[4][0] = fp_mul(B[4][0], scale);
        B[4][1] = fp_mul(B[4][1], scale);
    }
    A[4][4] = target;
}

/**
 * Build the QP Hessian and linear cost from predicted vehicle dynamics.
 *
 * @param current_state        Current Frenet state x[0]
 * @param reference_trajectory Reference points for steps 1..N
 * @param horizon_steps        Number of prediction steps N
 * @param total_variables      Total QP variables (controls + slacks)
 * @param linear_cost_vector   Output: Linear cost vector f (total_variables)
 */
static void build_qp_from_prediction(
    const FrenetState_t *current_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    int horizon_steps,
    int total_variables,
    fixed_point_t *linear_cost_vector)
{
    int n_vars = total_variables;  /* Hessian stride includes slack columns */

    /* ---------------------------------------------------------------
     * Step 1: Linearize vehicle model in Frenet frame
     *
     * Uses the Frenet linearization which produces a 5×5 A and 5×2 B
     * for state [e_y, e_psi, v_x, v_y, omega].
     *
     * The path curvature from the first reference point is used for
     * the linearization (constant across horizon). Body dynamics are
     * linearized at the actual velocity with a minimum floor.
     *
     * Steering is linearized at δ=0 to prevent velocity-steering
     * cross-coupling oscillation (same rationale as global version).
     * --------------------------------------------------------------- */
    fixed_point_t A[5][5];
    fixed_point_t B[5][2];

    ControlInput_t linearization_control;
    linearization_control.steering_angle_radians = 0;
    linearization_control.acceleration_meters_per_second_squared = 0;

    /* Create Frenet state with representative velocity for linearization.
     *
     * The condensed MPC uses a SINGLE linearization for the entire horizon.
     * Using the current velocity underestimates dynamics during acceleration
     * (A[0][1] = dt*vx too small → heading error growth underestimated).
     * Using the average reference velocity across the horizon provides a
     * better representative operating point for the body-frame dynamics
     * (tire force derivatives, vy/omega coupling). */
    FrenetState_t linearization_state = *current_state;
    fixed_point_t min_linearization_velocity = FP_CONST(2.0);

    /* Compute average reference velocity across horizon */
    {
        int64_t vsum = 0;
        for (int k = 0; k < horizon_steps; k++)
            vsum += (int64_t)reference_trajectory[k].reference_velocity_meters_per_second;
        fixed_point_t avg_ref_vx = (fixed_point_t)(vsum / horizon_steps);
        /* Use max of current and average reference — handles both accel and decel */
        fixed_point_t lin_vx = linearization_state.longitudinal_velocity_meters_per_second;
        if (avg_ref_vx > lin_vx) lin_vx = avg_ref_vx;
        if (lin_vx < min_linearization_velocity) lin_vx = min_linearization_velocity;
        linearization_state.longitudinal_velocity_meters_per_second = lin_vx;
    }

    /* Use curvature from first reference point */
    fixed_point_t path_curvature = reference_trajectory[0].path_curvature_radians_per_meter;

    vehicle_model_compute_frenet_linearization(
        &linearization_state,
        &linearization_control,
        current_configuration.time_step_seconds,
        path_curvature,
        A, B);

    /* ---------------------------------------------------------------
     * Precompute velocity-independent tire stiffness constants.
     *
     * At the MPC operating point (δ=0, vy=0, ω=0), slip angles are
     * zero and Pacejka stiffness is at its linear peak:
     *   C_f = μ · C_Sf · F_zf,  C_r = μ · C_Sr · F_zr
     *
     * These are SPEED-INDEPENDENT. The speed enters only through the
     * slip angle derivatives dα/dvy = -1/vx, dα/dω = ∓l/vx, giving
     * body-dynamics A entries that scale as 1/vx:
     *
     *   A[3][3] = 1 - dt·(C_f+C_r)/(m·vx)           (vy damping)
     *   A[3][4] = dt·(-lf·C_f+lr·C_r-m·vx²)/(m·vx)  (vy-ω coupling)
     *   A[4][3] = dt·(-lf·C_f+lr·C_r)/(Iz·vx)       (ω-vy coupling)
     *   A[4][4] = 1 - dt·(lf²·C_f+lr²·C_r)/(Iz·vx)  (ω damping)
     *
     * Precomputing the velocity-independent numerators allows cheap
     * per-step A updates using just fp_div(numerator, vx_k).
     * --------------------------------------------------------------- */
    VehicleParameters_t vp = vehicle_model_get_parameters();
    fixed_point_t lf_p = vp.distance_cg_to_front_axle_meters;
    fixed_point_t lr_p = vp.distance_cg_to_rear_axle_meters;
    fixed_point_t mass_p = vp.vehicle_mass_kg;
    fixed_point_t Iz_p = vp.yaw_moment_of_inertia_kgm2;
    fixed_point_t inv_m_p = fp_recip(mass_p);
    fixed_point_t inv_Iz_p = fp_recip(Iz_p);
    fixed_point_t dt_p = current_configuration.time_step_seconds;

    /* Linear tire stiffness at α=0: C = μ·C_S·F_z */
    const fixed_point_t mu_p = F110_FRICTION_COEFFICIENT;
    fixed_point_t mg_p = fp_mul(mass_p, vp.gravity_acceleration_meters_per_second_squared);
    fixed_point_t L_wb = fp_add(lf_p, lr_p);
    fixed_point_t inv_L_p = fp_recip(L_wb);
    fixed_point_t F_zf_p = fp_mul(fp_mul(mg_p, lr_p), inv_L_p);
    fixed_point_t F_zr_p = fp_mul(fp_mul(mg_p, lf_p), inv_L_p);
    fixed_point_t C_f = fp_mul(mu_p, fp_mul(vp.front_cornering_stiffness, F_zf_p));
    fixed_point_t C_r = fp_mul(mu_p, fp_mul(vp.rear_cornering_stiffness, F_zr_p));

    /* Velocity-independent numerators for per-step A body updates.
     * Each is multiplied by dt and divided by vx at each step. */
    fixed_point_t CfCr_sum = fp_add(C_f, C_r);          /* C_f + C_r */
    fixed_point_t lfCf_lrCr = fp_sub(fp_mul(lr_p, C_r),
                                     fp_mul(lf_p, C_f)); /* -lf·C_f + lr·C_r */
    fixed_point_t lf2Cf_lr2Cr = fp_add(fp_mul(fp_mul(lf_p, lf_p), C_f),
                                       fp_mul(fp_mul(lr_p, lr_p), C_r));
                                                          /* lf²·C_f + lr²·C_r */

    /* Stabilize yaw rate dynamics for initial linearization */
    const fixed_point_t stability_limit = FP_CONST(0.95);
    clamp_yaw_rate_row(A, B, stability_limit);

    /* ---------------------------------------------------------------
     * Step 2: Compute Phi[m] = A[m] * Phi[m-1] for m = 0 .. N-1
     *
     * Phi[0] = B
     * Phi[m] = A[m] * Phi[m-1]
     *
     * where A[m] varies per step:
     *   - A[0][1] = dt*v_ref[k]     (heading→lateral kinematic)
     *   - A[1][2] = -dt*kappa[k]    (curvature coupling)
     *   - A[3][3..4], A[4][3..4]    (body dynamics at v_ref[k])
     *
     * The body-dynamics variation captures how tire force coupling
     * changes with speed (1/vx scaling), critical when the horizon
     * spans a large speed range (e.g., 14→6 m/s at hairpin entry).
     *
     * Phi[m] tells us: if I apply a control impulse u at step j,
     * how much does the state change m steps later?
     * --------------------------------------------------------------- */
    static fixed_point_t Phi[MAXIMUM_HORIZON_STEPS][STATE_DIMENSION][CONTROL_DIMENSION];

    /* Precomputed Phi[m][s][a] * Q[s] for all (m, a, s).
     * Indexed [m][a][s] (not [m][s][a]) for cache-friendly inner loops.
     * Eliminates one int64 multiply per term in Hessian and gradient. */
    static fixed_point_t PhiQ[MAXIMUM_HORIZON_STEPS][CONTROL_DIMENSION][STATE_DIMENSION];

    /* ---------------------------------------------------------------
     * Steps 2+3 FUSED: Phi propagation + free-response tracking error
     *
     * Both operations propagate with the same A matrix (only A[1][2]
     * varies per step with path curvature). Fusing them into a single
     * loop keeps A in cache/registers and eliminates redundant
     * curvature lookups.
     *
     * Phi[0] = B                    (direct control mapping)
     * Phi[m] = A * Phi[m-1]        (for m = 1..N-1)
     * x_free[k] = A * x_prev       (for k = 0..N-1)
     * d[k] = x_free[k] - ref[k]   (tracking error)
     * --------------------------------------------------------------- */
    static fixed_point_t d[MAXIMUM_HORIZON_STEPS][STATE_DIMENSION];

    fixed_point_t x0[STATE_DIMENSION] = {
        current_state->lateral_error_meters,
        current_state->heading_error_radians,
        current_state->longitudinal_velocity_meters_per_second,
        current_state->lateral_velocity_meters_per_second,
        current_state->yaw_rate_radians_per_second
    };

    fixed_point_t x_prev[STATE_DIMENSION];
    for (int s = 0; s < STATE_DIMENSION; s++)
        x_prev[s] = x0[s];

    /* Phi[0] = B (direct control-to-state mapping, no A propagation) */
    for (int r = 0; r < STATE_DIMENSION; r++)
        for (int c = 0; c < CONTROL_DIMENSION; c++)
            Phi[0][r][c] = B[r][c];

    /* Step k=0: free response only (Phi[0]=B already set, uses kappa[0]).
     * Also update ALL velocity-dependent A entries at step 0's velocity. */
    A[1][2] = fp_neg(fp_mul(current_configuration.time_step_seconds,
                            reference_trajectory[0].path_curvature_radians_per_meter));
    {
        fixed_point_t ref_vx_0 = fp_max(
            reference_trajectory[0].reference_velocity_meters_per_second,
            min_linearization_velocity);
        A[0][1] = fp_mul(current_configuration.time_step_seconds, ref_vx_0);

        /* Per-step body dynamics at step 0 velocity */
        fixed_point_t inv_vx_0 = fp_recip(ref_vx_0);
        A[3][3] = fp_sub(FP_ONE,
            fp_mul(dt_p, fp_mul(fp_mul(CfCr_sum, inv_vx_0), inv_m_p)));
        A[3][4] = fp_sub(
            fp_mul(dt_p, fp_mul(fp_mul(lfCf_lrCr, inv_vx_0), inv_m_p)),
            fp_mul(dt_p, ref_vx_0));
        A[4][3] = fp_mul(dt_p, fp_mul(fp_mul(lfCf_lrCr, inv_vx_0), inv_Iz_p));
        A[4][4] = fp_sub(FP_ONE,
            fp_mul(dt_p, fp_mul(fp_mul(lf2Cf_lr2Cr, inv_vx_0), inv_Iz_p)));

        /* Yaw-rate stability clamp */
        clamp_yaw_rate_row(A, NULL, stability_limit);
    }
    {
        fixed_point_t x_free[STATE_DIMENSION];
        for (int r = 0; r < STATE_DIMENSION; r++)
        {
            fixed_point_t sum = 0;
            for (int j = 0; j < STATE_DIMENSION; j++)
                sum = fp_add(sum, fp_mul(A[r][j], x_prev[j]));
            x_free[r] = sum;
        }
        d[0][0] = fp_sub(x_free[0], reference_trajectory[0].reference_lateral_error_meters);
        d[0][1] = fp_normalize_angle(fp_sub(x_free[1], reference_trajectory[0].reference_heading_error_radians));
        d[0][2] = fp_sub(x_free[2], reference_trajectory[0].reference_velocity_meters_per_second);
        d[0][3] = fp_sub(x_free[3], reference_trajectory[0].reference_lateral_velocity_meters_per_second);
        d[0][4] = fp_sub(x_free[4], reference_trajectory[0].reference_yaw_rate_radians_per_second);

        saved_free_response_ey[0] = x_free[0];
        for (int s = 0; s < STATE_DIMENSION; s++)
            x_prev[s] = x_free[s];
        x_prev[1] = fp_normalize_angle(x_prev[1]);
    }

    /* Steps k=1..N-1: Phi propagation AND free response.
     * Per-step updates to A matrix:
     *   A[0][1] = dt * v_ref[k]    (heading→lateral drift)
     *   A[1][2] = -dt * kappa[k]   (curvature coupling)
     *   A[3][3], A[3][4], A[4][3], A[4][4] = body dynamics at v_ref[k]
     *
     * The body-dynamics entries scale as 1/vx (tire force derivatives).
     * Using per-step velocity gives accurate dynamics across the horizon
     * even when speed varies significantly (e.g., 14→6 m/s at hairpin). */
    for (int k = 1; k < horizon_steps; k++)
    {
        A[1][2] = fp_neg(fp_mul(current_configuration.time_step_seconds,
                                reference_trajectory[k].path_curvature_radians_per_meter));
        /* Per-step velocity for ALL velocity-dependent A entries */
        {
            fixed_point_t ref_vx_k = fp_max(
                reference_trajectory[k].reference_velocity_meters_per_second,
                min_linearization_velocity);
            A[0][1] = fp_mul(current_configuration.time_step_seconds, ref_vx_k);

            /* Per-step body dynamics: recompute A[3][3..4], A[4][3..4]
             * at this step's reference velocity.
             *
             * A[3][3] = 1 - dt*(C_f+C_r)/(m*vx)
             * A[3][4] = dt*(-lf*C_f+lr*C_r)/(m*vx) - dt*vx  (centripetal)
             * A[4][3] = dt*(-lf*C_f+lr*C_r)/(Iz*vx)
             * A[4][4] = 1 - dt*(lf²*C_f+lr²*C_r)/(Iz*vx)
             */
            fixed_point_t inv_vx_k = fp_recip(ref_vx_k);

            /* vy damping: 1 - dt*(C_f+C_r)/(m*vx) */
            A[3][3] = fp_sub(FP_ONE,
                fp_mul(dt_p, fp_mul(fp_mul(CfCr_sum, inv_vx_k), inv_m_p)));

            /* vy-omega coupling: dt*(-lf*C_f+lr*C_r)/(m*vx) - dt*vx */
            A[3][4] = fp_sub(
                fp_mul(dt_p, fp_mul(fp_mul(lfCf_lrCr, inv_vx_k), inv_m_p)),
                fp_mul(dt_p, ref_vx_k));

            /* omega-vy coupling: dt*(-lf*C_f+lr*C_r)/(Iz*vx) */
            A[4][3] = fp_mul(dt_p, fp_mul(fp_mul(lfCf_lrCr, inv_vx_k), inv_Iz_p));

            /* omega damping: 1 - dt*(lf²*C_f+lr²*C_r)/(Iz*vx) */
            A[4][4] = fp_sub(FP_ONE,
                fp_mul(dt_p, fp_mul(fp_mul(lf2Cf_lr2Cr, inv_vx_k), inv_Iz_p)));

            /* Yaw-rate stability clamp (per-step) */
            clamp_yaw_rate_row(A, NULL, stability_limit);
        }

        /* Phi[k] = A * Phi[k-1] */
        for (int r = 0; r < STATE_DIMENSION; r++)
            for (int c = 0; c < CONTROL_DIMENSION; c++)
            {
                fixed_point_t sum = 0;
                for (int j = 0; j < STATE_DIMENSION; j++)
                    sum = fp_add(sum, fp_mul(A[r][j], Phi[k - 1][j][c]));
                Phi[k][r][c] = sum;
            }

        /* Free response: x_free = A * x_prev (same A, already in cache) */
        {
            fixed_point_t x_free[STATE_DIMENSION];
            for (int r = 0; r < STATE_DIMENSION; r++)
            {
                fixed_point_t sum = 0;
                for (int j = 0; j < STATE_DIMENSION; j++)
                    sum = fp_add(sum, fp_mul(A[r][j], x_prev[j]));
                x_free[r] = sum;
            }
            d[k][0] = fp_sub(x_free[0], reference_trajectory[k].reference_lateral_error_meters);
            d[k][1] = fp_normalize_angle(fp_sub(x_free[1], reference_trajectory[k].reference_heading_error_radians));
            d[k][2] = fp_sub(x_free[2], reference_trajectory[k].reference_velocity_meters_per_second);
            d[k][3] = fp_sub(x_free[3], reference_trajectory[k].reference_lateral_velocity_meters_per_second);
            d[k][4] = fp_sub(x_free[4], reference_trajectory[k].reference_yaw_rate_radians_per_second);
            saved_free_response_ey[k] = x_free[0];
            for (int s = 0; s < STATE_DIMENSION; s++)
                x_prev[s] = x_free[s];
            x_prev[1] = fp_normalize_angle(x_prev[1]);
        }
    }

    /* Save Phi matrices and horizon for wall constraint building */
    memcpy(saved_Phi, Phi,
           horizon_steps * STATE_DIMENSION * CONTROL_DIMENSION * sizeof(fixed_point_t));
    saved_horizon_steps = horizon_steps;

    /* Debug: print Phi matrices showing control-to-state coupling */
#ifdef MPC_DEBUG_PRINT
    printf("[MPC-DBG] Phi[0][0:1][0] (direct δ→e_y,e_ψ): %.6f, %.6f\n",
           FP_TO_DOUBLE(Phi[0][0][0]), FP_TO_DOUBLE(Phi[0][1][0]));
    printf("[MPC-DBG] Phi[1][0:1][0] (1-step δ→e_y,e_ψ): %.6f, %.6f\n",
           FP_TO_DOUBLE(Phi[1][0][0]), FP_TO_DOUBLE(Phi[1][1][0]));
    printf("[MPC-DBG] Phi[2][0:1][0] (2-step δ→e_y,e_ψ): %.6f, %.6f\n",
           FP_TO_DOUBLE(Phi[2][0][0]), FP_TO_DOUBLE(Phi[2][1][0]));
    printf("[MPC-DBG] B[3][0]=%.6f B[4][0]=%.6f (δ→vy, δ→ω)\n",
           FP_TO_DOUBLE(B[3][0]), FP_TO_DOUBLE(B[4][0]));
    printf("[MPC-DBG] A[0][1]=%.6f A[0][3]=%.6f A[1][4]=%.6f\n",
           FP_TO_DOUBLE(A[0][1]), FP_TO_DOUBLE(A[0][3]), FP_TO_DOUBLE(A[1][4]));
#endif

    /* (Step 3 fused into Step 2 above) */

    /* ---------------------------------------------------------------
     * Step 4: State cost weight vector (diagonal Q matrix)
     *
     * Frenet state weights: [e_y, e_psi, v_x, v_y, omega]
     * Lateral error and heading error are the primary tracking states.
     *
     * Speed-dependent yaw/sideslip weights: at low speed the car is
     * dynamically stable regardless of steering, so yaw/vy weights are
     * kept low to allow the MPC to freely steer through hairpins.
     * At high speed (≥8 m/s), with the measured tire parameters
     * (C_Sr=3.32, low rear grip), large steering causes spinout.
     * High yaw/vy weights prevent the MPC from choosing solutions
     * that produce dangerous yaw-rate or sideslip.
     *
     * Linear blend:  w(v) = w_low  for v ≤ v_lo
     *                w(v) = w_high for v ≥ v_hi
     *                interpolated in between.
     * --------------------------------------------------------------- */
    fixed_point_t vx_for_weights = fp_abs(
        current_state->longitudinal_velocity_meters_per_second);
    const fixed_point_t v_lo  = FP_CONST(6.0);
    const fixed_point_t v_hi  = FP_CONST(10.0);
    const fixed_point_t w_vy_low  = FP_CONST(0.3);
    const fixed_point_t w_vy_high = FP_CONST(2.0);
    const fixed_point_t w_yr_low  = FP_CONST(0.3);
    const fixed_point_t w_yr_high = FP_CONST(2.0);

    fixed_point_t eff_w_vy, eff_w_yr;
    if (vx_for_weights <= v_lo) {
        eff_w_vy = w_vy_low;
        eff_w_yr = w_yr_low;
    } else if (vx_for_weights >= v_hi) {
        eff_w_vy = w_vy_high;
        eff_w_yr = w_yr_high;
    } else {
        /* Linear interpolation: alpha = (vx - v_lo) / (v_hi - v_lo) */
        fixed_point_t alpha = fp_div(fp_sub(vx_for_weights, v_lo),
                                     fp_sub(v_hi, v_lo));
        eff_w_vy = fp_add(w_vy_low, fp_mul(alpha, fp_sub(w_vy_high, w_vy_low)));
        eff_w_yr = fp_add(w_yr_low, fp_mul(alpha, fp_sub(w_yr_high, w_yr_low)));
    }

    /* Speed-dependent lateral weight boost:
     * At low speed (v≤8): w_lat unchanged (proven stable)
     * At high speed (v>8): w_lat multiplied by (1 + 1.0*(v-8))
     *   v=8: ×1.0 (50),  v=10: ×3.0 (150),  v=12: ×5.0 (250)
     * This prevents the high-speed lateral tracking failure (e_y=-0.80m drift)
     * that caused crashes at 11.5 m/s. The boost doesn't affect low-speed
     * sections where w_lat=50 was proven stable. */
    fixed_point_t effective_w_lat = current_configuration.weight_lateral_error;
    fixed_point_t effective_w_heading = current_configuration.weight_heading_error;
    {
        const fixed_point_t v_boost_start = FP_CONST(8.0);
        fixed_point_t vx_abs = fp_abs(
            current_state->longitudinal_velocity_meters_per_second);
        if (vx_abs > v_boost_start)
        {
            fixed_point_t v_excess = fp_sub(vx_abs, v_boost_start);
            /* w_lat: slope 1.0 → more aggressive at high speed */
            fixed_point_t lat_boost = fp_add(FP_ONE,
                fp_mul(FP_CONST(1.0), v_excess));
            effective_w_lat = fp_mul(effective_w_lat, lat_boost);
            /* w_heading: slope 0.5 → moderate heading boost at high speed
             * v=10: 5*2=10, v=12: 5*3=15. Combats heading error buildup. */
            fixed_point_t head_boost = fp_add(FP_ONE,
                fp_mul(FP_CONST(0.5), v_excess));
            effective_w_heading = fp_mul(effective_w_heading, head_boost);
        }
    }

    fixed_point_t Q[STATE_DIMENSION] = {
        effective_w_lat,
        effective_w_heading,
        current_configuration.weight_velocity,
        eff_w_vy,
        eff_w_yr
    };

    /* Build list of active (non-zero weight) state indices */
    int active_states[STATE_DIMENSION];
    int num_active_states = 0;
    for (int s = 0; s < STATE_DIMENSION; s++)
    {
        if (Q[s] != 0)
        {
            active_states[num_active_states++] = s;
        }
    }

    /* ---------------------------------------------------------------
     * Step 4b: Precompute PhiQ[m][a][s] = Phi[m][s][a] * Q[s]
     *
     * This product appears in EVERY inner-loop term of both Hessian
     * and gradient construction. Precomputing it once eliminates one
     * int64 multiply per term — roughly halving int64 operations
     * in the two most expensive construction loops.
     *
     * Only active states (Q[s] != 0) are computed; inactive entries
     * stay zero from memset, contributing nothing in the fixed-bound
     * STATE_DIMENSION inner loops (which enables HLS full unrolling).
     * --------------------------------------------------------------- */
    memset(PhiQ, 0, (size_t)horizon_steps * CONTROL_DIMENSION * STATE_DIMENSION * sizeof(fixed_point_t));
    for (int m = 0; m < horizon_steps; m++)
    {
        for (int si = 0; si < num_active_states; si++)
        {
            int s = active_states[si];
            for (int a = 0; a < CONTROL_DIMENSION; a++)
            {
                PhiQ[m][a][s] = (fixed_point_t)(((int64_t)Phi[m][s][a] * Q[s]) >> FP_FRAC_BITS);
            }
        }
    }

    /* ---------------------------------------------------------------
     * Step 5: Build Hessian matrix
     *
     * H = 2 * Su^T Q_bar Su  (tracking)
     *   + 2 * R_bar          (control effort)
     *   + Rate penalty        (smoothness)
     *
     * Exponential discount γ^k reduces the weight of distant prediction
     * steps.  With γ=0.9 and N=20: step 10 is weighted 35%, step 19 is
     * weighted 14%.  This prevents the condensed QP from over-committing
     * to distant curvature changes (e.g., a hairpin 0.5s away) at the
     * expense of near-term tracking stability.
     *
     * H_block(ci, cj) = 2 * sum_{k} γ^k * Phi[k-ci]^T Q Phi[k-cj]
     * --------------------------------------------------------------- */

    /* Per-state exponential discount factors γ^k.
     * Tracking states (lateral, heading) use γ=0.9 to prevent
     * distant-horizon overshoot from dominating the gradient.
     * Body states (velocity, lateral velocity, yaw rate) are undiscounted
     * so velocity tracking remains full-strength (braking for corners). */
    static fixed_point_t gamma_track_power[MAXIMUM_HORIZON_STEPS];
    {
        const fixed_point_t gamma_track = FP_CONST(0.90);
        gamma_track_power[0] = FP_ONE;
        for (int k = 1; k < horizon_steps; k++) {
            gamma_track_power[k] = fp_mul(gamma_track, gamma_track_power[k - 1]);
        }
    }

    memset(hessian_int64, 0,
        (size_t)n_vars * (size_t)n_vars * sizeof(int64_t));

    /* ---------------------------------------------------------------
     * Step 5a: Precompute G[mi][mj] = PhiQ[mi]^T * Phi[mj]  (2×2 block)
     *
     * This separates the MULTIPLY phase (G precomputation, N² blocks)
     * from the ACCUMULATE phase (Hessian summation, addition only).
     * For N=10: 2400 multiplies (precompute) + 1100 additions (accumulate)
     * vs. 9240 multiplies in the original triple loop.
     * ~2.6× faster Hessian construction.
     * --------------------------------------------------------------- */
    /* Split G into tracking (states 0,1) and body (states 2,3,4) groups
     * so that per-state gamma discounting can be applied in the accumulation. */
    static int64_t G_track[MAXIMUM_HORIZON_STEPS][MAXIMUM_HORIZON_STEPS][CONTROL_DIMENSION][CONTROL_DIMENSION];
    static int64_t G_body[MAXIMUM_HORIZON_STEPS][MAXIMUM_HORIZON_STEPS][CONTROL_DIMENSION][CONTROL_DIMENSION];

    for (int mi = 0; mi < horizon_steps; mi++)
    {
        for (int mj = 0; mj < horizon_steps; mj++)
        {
            for (int a = 0; a < CONTROL_DIMENSION; a++)
            {
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                {
                    int64_t sum_track = 0;
                    int64_t sum_body  = 0;
                    for (int s = 0; s < 2; s++)  /* states 0,1: lateral, heading */
                    {
                        sum_track += ((int64_t)PhiQ[mi][a][s] * Phi[mj][s][b]) >> FP_FRAC_BITS;
                    }
                    for (int s = 2; s < STATE_DIMENSION; s++)  /* states 2,3,4: vel, lat_vel, yaw_rate */
                    {
                        sum_body += ((int64_t)PhiQ[mi][a][s] * Phi[mj][s][b]) >> FP_FRAC_BITS;
                    }
                    G_track[mi][mj][a][b] = sum_track;
                    G_body[mi][mj][a][b]  = sum_body;
                }
            }
        }
    }

    /* ---------------------------------------------------------------
     * Step 5b: Accumulate Hessian from precomputed G blocks.
     * Tracking states (0,1) are discounted by γ_track^k; body states
     * (2,3,4) are undiscounted (added directly).
     * --------------------------------------------------------------- */
    for (int ci = 0; ci < horizon_steps; ci++)
    {
        for (int cj = ci; cj < horizon_steps; cj++)
        {
            int64_t block64[CONTROL_DIMENSION][CONTROL_DIMENSION];
            for (int a = 0; a < CONTROL_DIMENSION; a++)
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                    block64[a][b] = 0;

            /* Sum per-group discounted G blocks */
            for (int k = cj; k < horizon_steps; k++)
            {
                int mi = k - ci;
                int mj = k - cj;
                int64_t gk_track = (int64_t)gamma_track_power[k];
                for (int a = 0; a < CONTROL_DIMENSION; a++)
                    for (int b = 0; b < CONTROL_DIMENSION; b++)
                        block64[a][b] += ((gk_track * G_track[mi][mj][a][b]) >> FP_FRAC_BITS)
                                       + G_body[mi][mj][a][b];
            }

            /* Write 2×2 block to int64 Hessian (with ×2 for QP convention).
             * No INT32 clamping — preserves full precision for auto-scaling. */
            int row = ci * CONTROL_DIMENSION;
            int col = cj * CONTROL_DIMENSION;

            for (int a = 0; a < CONTROL_DIMENSION; a++)
            {
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                {
                    int64_t val64 = block64[a][b] * 2;

                    hessian_int64[(row + a) * n_vars + (col + b)] = val64;

                    /* Symmetric entry */
                    if (ci != cj)
                    {
                        hessian_int64[(col + b) * n_vars + (row + a)] = val64;
                    }
                }
            }
        }
    }

    /* Control effort contribution: 2*R on diagonal (saturating add) */
    fixed_point_t two_w_steer = fp_mul(FP_TWO,
        current_configuration.weight_steering_effort);
    fixed_point_t two_w_torque = fp_mul(FP_TWO,
        current_configuration.weight_acceleration_effort);

    for (int ci = 0; ci < horizon_steps; ci++)
    {
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);
        hessian_int64[idx_s] += (int64_t)two_w_steer;
        hessian_int64[idx_v] += (int64_t)two_w_torque;
    }

    /* Rate penalty contribution: tridiagonal band structure.
     * All constant products hoisted out of the loop to avoid recomputation.
     * Structure: +2w on boundary diagonals, +4w on interior diagonals,
     * -2w on off-diagonals (k,k-1) and (k-1,k).
     *
     * Velocity-dependent scaling: at higher speeds, the dynamics evolve
     * faster — the same physical steering correction covers fewer time
     * steps.  Reducing the rate penalty proportionally lets the MPC
     * respond quickly at high speed while still damping oscillation
     * in slow-speed turns.
     *
     * w_sr_eff = w_sr * min(1, v_threshold / vx)
     * For vx < v_threshold (≈5 m/s): full rate penalty (hairpin damping)
     * For vx = 14 m/s: rate penalty × 0.36 (fast heading correction) */
    fixed_point_t w_sr = current_configuration.weight_steering_rate;
    fixed_point_t w_vr = current_configuration.weight_acceleration_rate;
    {
        const fixed_point_t v_threshold = FP_CONST(5.0);
        fixed_point_t current_vx = fp_abs(
            current_state->longitudinal_velocity_meters_per_second);
        if (current_vx > v_threshold)
        {
            w_sr = fp_div(fp_mul(w_sr, v_threshold), current_vx);
        }
    }
    fixed_point_t w_sr_cross = fp_mul(w_sr, current_configuration.cross_call_rate_scale);
    fixed_point_t w_vr_cross = fp_mul(w_vr, current_configuration.cross_call_rate_scale);

    /* Precompute all rate penalty constants (avoids fp_mul inside loops) */
    fixed_point_t two_sr       = fp_mul(FP_TWO, w_sr);
    fixed_point_t two_vr       = fp_mul(FP_TWO, w_vr);
    fixed_point_t four_sr      = fp_mul((fixed_point_t)(4 * FP_ONE), w_sr);
    fixed_point_t four_vr      = fp_mul((fixed_point_t)(4 * FP_ONE), w_vr);
    fixed_point_t neg_two_sr   = fp_neg(two_sr);
    fixed_point_t neg_two_vr   = fp_neg(two_vr);
    fixed_point_t two_sr_cross = fp_mul(FP_TWO, w_sr_cross);
    fixed_point_t two_vr_cross = fp_mul(FP_TWO, w_vr_cross);

    /* Step 0: cross-call rate + first within-horizon rate */
    {
        int idx_s = 0;
        int idx_v = 1 * n_vars + 1;
        hessian_int64[idx_s] += (int64_t)two_sr_cross;
        hessian_int64[idx_v] += (int64_t)two_vr_cross;
        if (horizon_steps > 1)
        {
            hessian_int64[idx_s] += (int64_t)two_sr;
            hessian_int64[idx_v] += (int64_t)two_vr;
        }
    }

    /* Interior steps 1..N-2: +4w diagonal, -2w off-diagonal */
    for (int ci = 1; ci < horizon_steps - 1; ci++)
    {
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);
        hessian_int64[idx_s] += (int64_t)four_sr;
        hessian_int64[idx_v] += (int64_t)four_vr;

        /* Off-diagonal: H[ci-1, ci] = H[ci, ci-1] = -2*w_rate */
        int prev_s = ((ci - 1) * 2) * n_vars + (ci * 2);
        int prev_v = ((ci - 1) * 2 + 1) * n_vars + (ci * 2 + 1);
        int sym_s  = (ci * 2) * n_vars + ((ci - 1) * 2);
        int sym_v  = (ci * 2 + 1) * n_vars + ((ci - 1) * 2 + 1);
        hessian_int64[prev_s] += (int64_t)neg_two_sr;
        hessian_int64[sym_s]  += (int64_t)neg_two_sr;
        hessian_int64[prev_v] += (int64_t)neg_two_vr;
        hessian_int64[sym_v]  += (int64_t)neg_two_vr;
    }

    /* Last step N-1: +2w diagonal, -2w off-diagonal to step N-2 */
    if (horizon_steps > 1)
    {
        int ci = horizon_steps - 1;
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);
        hessian_int64[idx_s] += (int64_t)two_sr;
        hessian_int64[idx_v] += (int64_t)two_vr;

        int prev_s = ((ci - 1) * 2) * n_vars + (ci * 2);
        int prev_v = ((ci - 1) * 2 + 1) * n_vars + (ci * 2 + 1);
        int sym_s  = (ci * 2) * n_vars + ((ci - 1) * 2);
        int sym_v  = (ci * 2 + 1) * n_vars + ((ci - 1) * 2 + 1);
        hessian_int64[prev_s] += (int64_t)neg_two_sr;
        hessian_int64[sym_s]  += (int64_t)neg_two_sr;
        hessian_int64[prev_v] += (int64_t)neg_two_vr;
        hessian_int64[sym_v]  += (int64_t)neg_two_vr;
    }

    /* ---------------------------------------------------------------
     * Step 6: Build linear cost vector
     *
     * f = 2 * Su^T Q_bar d  (tracking)
     *   + f_rate             (rate penalty for u[0] vs u_prev)
     *
     * f_track[ci][a] = 2 * sum_{k=ci}^{N-1} Phi[k-ci][s][a] * Q[s] * d[k][s]
     * --------------------------------------------------------------- */
    memset(linear_cost_vector, 0, (size_t)n_vars * sizeof(fixed_point_t));
    memset(gradient_int64, 0, (size_t)n_vars * sizeof(int64_t));

    /* Tracking contribution: 2 * Su^T Q d — using int64_t accumulator.
     * Per-state gamma discount matching the Hessian split:
     *   states 0,1 (lateral, heading):  γ_track^k
     *   states 2,3,4 (vel, lat_vel, yaw): undiscounted
     * Store in gradient_int64[] to preserve full precision for auto-scaling. */
    for (int ci = 0; ci < horizon_steps; ci++)
    {
        for (int a = 0; a < CONTROL_DIMENSION; a++)
        {
            int64_t sum64 = 0;

            for (int k = ci; k < horizon_steps; k++)
            {
                int m = k - ci;
                int64_t gk_track = (int64_t)gamma_track_power[k];

                /* Tracking states (0, 1): discounted by γ_track */
                for (int s = 0; s < 2; s++)
                {
                    int64_t term = ((int64_t)PhiQ[m][a][s] * d[k][s]) >> FP_FRAC_BITS;
                    sum64 += (gk_track * term) >> FP_FRAC_BITS;
                }
                /* Body states (2, 3, 4): undiscounted (γ_body = 1.0) */
                for (int s = 2; s < STATE_DIMENSION; s++)
                {
                    sum64 += ((int64_t)PhiQ[m][a][s] * d[k][s]) >> FP_FRAC_BITS;
                }
            }

            /* Multiply by 2, store in int64 (no clamping yet) */
            gradient_int64[ci * CONTROL_DIMENSION + a] = sum64 * 2;
        }
    }

    /* Cross-call rate penalty linear cost: from (u[0] - u_prev)^2 expansion
     * f_rate[0] = -2 * w_sr_cross * u_prev.steering
     * f_rate[1] = -2 * w_vr_cross * u_prev.velocity
     *
     * Uses the SCALED weights (w_sr_cross, w_vr_cross) to match the
     * scaled diagonal in the Hessian for step 0.
     * Add to int64 gradient buffer (rate terms are small, won't overflow int64).
     */
    gradient_int64[0] -= (int64_t)fp_mul(
            FP_TWO,
            fp_mul(w_sr_cross,
                previous_control_input.steering_angle_radians));

    gradient_int64[1] -= (int64_t)fp_mul(
            FP_TWO,
            fp_mul(w_vr_cross,
                previous_control_input.acceleration_meters_per_second_squared));
    
    /* Debug: print first element of linear cost (int64 value) */
#ifdef MPC_DEBUG_PRINT
    printf("[MPC-DBG] d[0][1]=%.4f, Phi[0][1][0]=%.4f, f64[0]=%.4f\n",
           FP_TO_DOUBLE(d[0][1]), FP_TO_DOUBLE(Phi[0][1][0]),
           (double)gradient_int64[0] / 65536.0);
    printf("[MPC-DBG] d[0][0]=%.4f (e_y), d[0][1]=%.4f (e_psi)\n",
           FP_TO_DOUBLE(d[0][0]), FP_TO_DOUBLE(d[0][1]));
#endif
}

/**
 * Build constraint matrices for actuator limits and wall boundaries.
 *
 * Actuator constraints (box constraints):
 * - Steering angle: |delta| <= max_steering
 * - Acceleration: min_accel <= a <= max_accel
 *
 * Wall boundary constraints (general linear constraints):
 * - e_y[k+1] <= left_bound[k]   (don't exceed left wall)
 * - -e_y[k+1] <= right_bound[k] (don't exceed right wall)
 *
 * In the condensed MPC formulation:
 *   e_y[k+1] = free_ey[k] + sum_{j=0}^{k} Phi[k-j][0][*] * u[j]
 * So wall constraints are linear in the control vector u.
 *
 * @param horizon_steps        Number of prediction steps
 * @param reference_trajectory Reference points (for wall bounds)
 * @param constraint_matrix    Output constraint matrix
 * @param constraint_bounds    Output constraint bounds vector
 * @param constraint_count     Output number of constraints
 */

/* Penalty weight for slack variables (soft wall constraints).
 * Higher values make the wall constraint "harder" - the car will
 * strongly prefer staying within bounds, but the QP stays feasible.
 * Generalizes to obstacle avoidance: add more slack-penalized constraints. */
#define WALL_SLACK_PENALTY_WEIGHT  DOUBLE_TO_FP(40.0)

/** First prediction step to apply wall constraints (skip 0,1 where Phi[0]=0) */
#define WALL_CONSTRAINT_START  2

/** Apply wall constraints every N-th step to reduce constraint density */
#define WALL_CONSTRAINT_STRIDE 4

/** Extra safety margin subtracted from wall bounds (meters) */
#define WALL_MARGIN_FP DOUBLE_TO_FP(0.05)

/**
 * Count how many wall-constrained prediction steps there will be.
 * This is needed BEFORE building the QP to determine the slack variable count.
 */
static int count_wall_constraint_steps(
    const TrajectoryReferencePoint_t *reference_trajectory,
    int horizon_steps)
{
    /* Check if any bounds are meaningful (< 4.0m) */
    int use_wall_constraints = 0;
    for (int k = 0; k < horizon_steps && !use_wall_constraints; k++) {
        if (reference_trajectory[k].left_wall_bound_meters < DOUBLE_TO_FP(4.0) ||
            reference_trajectory[k].right_wall_bound_meters < DOUBLE_TO_FP(4.0)) {
            use_wall_constraints = 1;
        }
    }
    if (!use_wall_constraints) return 0;

    int count = 0;
    for (int k = WALL_CONSTRAINT_START; k < horizon_steps; k += WALL_CONSTRAINT_STRIDE) {
        count++;
    }
    return count;
}

/**
 * Build constraint matrices for actuator limits and wall boundaries
 * with SLACK VARIABLES for soft wall constraints.
 *
 * Actuator constraints (box-like, always satisfiable):
 * - Steering angle: |delta| <= max_steering
 * - Acceleration: min_accel <= a <= max_accel
 *
 * Wall boundary constraints (soft via slack variables):
 *   Phi * u_controls - s_left  <= left_bound  - free_ey     (stay within left wall)
 *  -Phi * u_controls - s_right <= right_bound + free_ey     (stay within right wall)
 *   -s_left  <= 0   (slack >= 0)
 *   -s_right <= 0   (slack >= 0)
 *
 * The slack variables absorb infeasibility: if the car is too close to
 * a wall, s > 0 allows the constraint to be satisfied. The large penalty
 * in the cost (ρ * s²) strongly discourages actual wall violation.
 *
 * This architecture extends directly to obstacle avoidance — just add
 * more constraint rows with their own slack variables.
 *
 * @param horizon_steps        Number of prediction steps
 * @param reference_trajectory Reference points (for wall bounds)
 * @param total_variables      Total QP variables (controls + slacks)
 * @param n_slack_variables    Number of slack variables (wall_steps * 2)
 * @param constraint_matrix    Output constraint matrix
 * @param constraint_bounds    Output constraint bounds vector
 * @param constraint_count     Output number of constraints
 */
static void build_qp_constraints(
    int horizon_steps,
    const TrajectoryReferencePoint_t *reference_trajectory,
    int total_variables,
    int n_slack_variables,
    fixed_point_t current_velocity,
    fixed_point_t *constraint_matrix,
    fixed_point_t *constraint_bounds,
    uint16_t *constraint_count)
{
    VehicleParameters_t vehicle_params = vehicle_model_get_parameters();

    /* Speed-dependent steering limit.
     *
     * With measured tire stiffness (C_alpha_f=33.4, C_alpha_r=41.0 N/rad),
     * the car is near-neutral and spins out if excessive steering is
     * applied at speed.  Limit max steering so the steady-state lateral
     * acceleration stays within the tires' capacity:
     *
     *   delta_max = a_lat_limit * L / vx^2   (kinematic approx)
     *
     * Clamped between [delta_min_floor, hardware_max].
     * At low speed (≤v_threshold) the hardware maximum is used. */
    fixed_point_t hardware_max_steer = vehicle_params.maximum_steering_angle_radians;
    fixed_point_t effective_max_steer = hardware_max_steer;
    {
        /* Use current velocity for speed-dependent limit */
        fixed_point_t vx_abs = fp_abs(current_velocity);
        const fixed_point_t v_threshold = FP_CONST(3.0);
        if (vx_abs > v_threshold)
        {
            /* a_lat_limit * L / vx^2 — use mu*g * 0.6 as conservative limit */
            const fixed_point_t a_lat_limit = FP_CONST(4.4);  /* 0.6 * mu * g */
            fixed_point_t L_wb = fp_add(
                vehicle_params.distance_cg_to_front_axle_meters,
                vehicle_params.distance_cg_to_rear_axle_meters);
            fixed_point_t delta_v = fp_div(fp_mul(a_lat_limit, L_wb),
                                          fp_mul(vx_abs, vx_abs));

            /* Steering floor — horizon-dependent.
             *
             * For small horizons (N≤10, including FPGA deployment):
             *   Fixed floor of 0.20 rad ensures sufficient tracking
             *   authority for error correction on tight curves.
             *
             * For large horizons (N>10, e.g. N=20 testing):
             *   Scale floor inversely with horizon.  With more prediction
             *   steps the MPC plans further ahead, needing less immediate
             *   authority.  The tighter constraint also improves PGD solver
             *   convergence by reducing the feasible region for gentle
             *   curves where optimal steering is tiny (e.g. R=30: δ*=0.011
             *   but ±0.20 constraint causes solver chattering). */
            fixed_point_t delta_min_floor;
            if (horizon_steps <= 10) {
                delta_min_floor = FP_CONST(0.20);
            } else {
                /* 2.0/N: gives 0.10 for N=20, 0.067 for N=30 */
                delta_min_floor = fp_div(FP_CONST(2.0),
                    (fixed_point_t)(horizon_steps * FP_ONE));
                /* Minimum absolute floor to prevent zero-authority edge cases */
                const fixed_point_t abs_minimum = FP_CONST(0.05);
                if (delta_min_floor < abs_minimum) delta_min_floor = abs_minimum;
            }

            if (delta_v < delta_min_floor) delta_v = delta_min_floor;
            if (delta_v < effective_max_steer) effective_max_steer = delta_v;
        }
    }

    int total_controls = horizon_steps * CONTROL_DIMENSION;
    int wall_step_count = n_slack_variables / 2;

    /* Constraint counts */
    int actuator_constraints_per_step = 4;
    int total_actuator = horizon_steps * actuator_constraints_per_step;
    int total_wall = wall_step_count * 2;      /* left + right per step */
    int total_slack_box = n_slack_variables;    /* s >= 0 for each slack */
    int total_constraints = total_actuator + total_wall + total_slack_box;

    /* Clamp to QP solver capacity */
    if (total_constraints > QP_MAXIMUM_CONSTRAINTS) {
        /* Reduce wall constraints to fit (keeping slack box constraints matched) */
        int available = QP_MAXIMUM_CONSTRAINTS - total_actuator;
        /* Each wall step needs: 2 wall constraints + 2 slack box constraints = 4 */
        int max_wall_steps = available / 4;
        wall_step_count = max_wall_steps;
        total_wall = wall_step_count * 2;
        total_slack_box = wall_step_count * 2;
        n_slack_variables = wall_step_count * 2;
        total_constraints = total_actuator + total_wall + total_slack_box;
    }
    *constraint_count = (uint16_t)total_constraints;

    /* Clear constraint matrix (total_constraints × total_variables) */
    memset(constraint_matrix, 0,
           (size_t)total_constraints * (size_t)total_variables * sizeof(fixed_point_t));

    /* --- Actuator box constraints --- */
    for (int step = 0; step < horizon_steps; step++)
    {
        int control_base = step * CONTROL_DIMENSION;
        int constraint_base = step * actuator_constraints_per_step;

        /* Constraint 0: steering <= effective_max_steering */
        constraint_matrix[(constraint_base + 0) * total_variables + control_base] =
            FP_ONE;
        constraint_bounds[constraint_base + 0] =
            effective_max_steer;

        /* Constraint 1: -steering <= effective_max_steering */
        constraint_matrix[(constraint_base + 1) * total_variables + control_base] =
            fp_neg(FP_ONE);
        constraint_bounds[constraint_base + 1] =
            effective_max_steer;

        /* Constraint 2: acceleration <= max_acceleration */
        constraint_matrix[(constraint_base + 2) * total_variables + (control_base + 1)] =
            FP_ONE;
        constraint_bounds[constraint_base + 2] =
            vehicle_params.maximum_acceleration_meters_per_second_squared;

        /* Constraint 3: -acceleration <= -min_acceleration */
        constraint_matrix[(constraint_base + 3) * total_variables + (control_base + 1)] =
            fp_neg(FP_ONE);
        constraint_bounds[constraint_base + 3] =
            fp_neg(vehicle_params.minimum_acceleration_meters_per_second_squared);
    }

    /* --- Wall boundary constraints with slack variables ---
     *
     * For prediction step k:
     * e_y[k+1] = free_ey[k] + sum_j Phi[k-j][0][*] * u[j]
     *
     * Left wall:  Phi * u - s_left  <= left_bound  - free_ey
     * Right wall: -Phi * u - s_right <= right_bound + free_ey
     *
     * The -1 coefficient on the slack variable allows it to absorb
     * any constraint violation, keeping the QP always feasible.
     */
    int wall_row_idx = 0;
    int slack_col_base = total_controls;  /* Slack variables start after controls */

    for (int k = WALL_CONSTRAINT_START; k < horizon_steps && wall_row_idx < wall_step_count; k += WALL_CONSTRAINT_STRIDE)
    {
        int left_row  = total_actuator + wall_row_idx * 2;
        int right_row = total_actuator + wall_row_idx * 2 + 1;
        int slack_left_col  = slack_col_base + wall_row_idx * 2;
        int slack_right_col = slack_col_base + wall_row_idx * 2 + 1;

        /* Build constraint rows from Phi matrices (control columns) */
        for (int j = 0; j <= k; j++)
        {
            int m = k - j;
            int u_base = j * CONTROL_DIMENSION;

            /* Left wall: +Phi[m][0][*] for control columns */
            constraint_matrix[left_row * total_variables + u_base]     = saved_Phi[m][0][0];
            constraint_matrix[left_row * total_variables + u_base + 1] = saved_Phi[m][0][1];

            /* Right wall: -Phi[m][0][*] for control columns */
            constraint_matrix[right_row * total_variables + u_base]     = fp_neg(saved_Phi[m][0][0]);
            constraint_matrix[right_row * total_variables + u_base + 1] = fp_neg(saved_Phi[m][0][1]);
        }

        /* Slack variable columns: -1 for each slack */
        constraint_matrix[left_row  * total_variables + slack_left_col]  = fp_neg(FP_ONE);
        constraint_matrix[right_row * total_variables + slack_right_col] = fp_neg(FP_ONE);

        /* Constraint bounds with margin */
        fixed_point_t free_ey = saved_free_response_ey[k];
        fixed_point_t left_bound  = fp_sub(reference_trajectory[k].left_wall_bound_meters, WALL_MARGIN_FP);
        fixed_point_t right_bound = fp_sub(reference_trajectory[k].right_wall_bound_meters, WALL_MARGIN_FP);

        if (left_bound < DOUBLE_TO_FP(0.05)) left_bound = DOUBLE_TO_FP(0.05);
        if (right_bound < DOUBLE_TO_FP(0.05)) right_bound = DOUBLE_TO_FP(0.05);

        constraint_bounds[left_row]  = fp_sub(left_bound, free_ey);
        constraint_bounds[right_row] = fp_add(right_bound, free_ey);

#ifdef MPC_DEBUG_PRINT
        if (wall_row_idx <= 2) {
            printf("[MPC-DBG] Wall k=%d: free_ey=%.3f, L=%.3f, R=%.3f, "
                   "L_rhs=%.3f, R_rhs=%.3f (slack cols %d,%d)\n",
                   k, FP_TO_DOUBLE(free_ey),
                   FP_TO_DOUBLE(left_bound), FP_TO_DOUBLE(right_bound),
                   FP_TO_DOUBLE(constraint_bounds[left_row]),
                   FP_TO_DOUBLE(constraint_bounds[right_row]),
                   slack_left_col, slack_right_col);
        }
#endif
        wall_row_idx++;
    }

    /* --- Slack box constraints: s >= 0 (i.e., -s <= 0) --- */
    for (int i = 0; i < n_slack_variables; i++)
    {
        int row = total_actuator + total_wall + i;
        int col = total_controls + i;
        constraint_matrix[row * total_variables + col] = fp_neg(FP_ONE);
        constraint_bounds[row] = 0;  /* -s <= 0 → s >= 0 */
    }
}

/*===========================================================================
 * Public API Implementation
 *===========================================================================*/

void mpc_initialize(void)
{
    mpc_initialize_with_configuration(NULL);
}

void mpc_initialize_with_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration != NULL)
    {
        current_configuration = *configuration;
    }
    else
    {
        current_configuration = get_default_configuration();
    }

    vehicle_model_initialize();

    previous_control_input.steering_angle_radians = 0;
    previous_control_input.acceleration_meters_per_second_squared = 0;

    memset(warm_start_variables, 0, sizeof(warm_start_variables));
    warm_start_available = 0;
    warm_start_prev_curvature = 0;

    mpc_initialized_flag = 1;
}

MpcSolverStatus_t mpc_compute_optimal_control(
    const FrenetState_t *current_frenet_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result)
{
    /* Validate inputs */
    if (current_frenet_state == NULL ||
        reference_trajectory == NULL ||
        result == NULL)
    {
        if (result != NULL)
        {
            result->solver_status = MPC_STATUS_ERROR;
        }
        return MPC_STATUS_ERROR;
    }

    if (!mpc_initialized_flag)
    {
        mpc_initialize();
    }

    /* Get horizon (capped to maximum) */
    int horizon = current_configuration.prediction_horizon_steps;
    if (horizon > MAXIMUM_HORIZON_STEPS)
    {
        horizon = MAXIMUM_HORIZON_STEPS;
    }

    int total_controls = horizon * CONTROL_DIMENSION;

    /* Count wall constraint steps to determine slack variable count.
     * Slack variables make wall constraints SOFT — always feasible.
     * This architecture extends directly to obstacle avoidance. */
    int n_wall_steps = count_wall_constraint_steps(reference_trajectory, horizon);
    int n_slacks = n_wall_steps * 2;  /* left + right slack per wall step */
    int total_vars = total_controls + n_slacks;

    if (total_vars > QP_MAXIMUM_VARIABLES) {
        /* Reduce wall steps to fit */
        int available = QP_MAXIMUM_VARIABLES - total_controls;
        n_slacks = (available > 0) ? available - (available % 2) : 0;
        n_wall_steps = n_slacks / 2;
        total_vars = total_controls + n_slacks;
    }

    /* Build QP problem — lightweight init (dimensions + flags only).
     * The build functions below memset their own output regions,
     * so there is no need to clear the full 90KB problem struct here. */
    QuadraticProgramProblem_t qp_problem;
    QuadraticProgramConfig_t qp_config;
    QuadraticProgramSolution_t qp_solution;

    qp_problem.variable_count = total_vars;
    qp_problem.constraint_count = 0;
    qp_problem.use_warm_start = 0;
    qp_solver_initialize_config(&qp_config);

    /* Build QP Hessian and linear cost from predicted dynamics.
     * Hessian is built in hessian_int64[]; gradient in gradient_int64[].
     * Auto-scaling below writes to qp_problem.hessian_matrix. */
    build_qp_from_prediction(
        current_frenet_state,
        reference_trajectory,
        horizon,
        total_vars,
        qp_problem.linear_cost_vector);

    /* Add slack variable penalty to Hessian diagonal.
     * Cost: ρ * s² → Hessian diagonal entry: 2 * ρ
     * This strongly discourages wall violation while keeping the QP feasible. */
    {
        fixed_point_t two_rho = fp_mul(FP_TWO, WALL_SLACK_PENALTY_WEIGHT);
        for (int i = 0; i < n_slacks; i++)
        {
            int idx = total_controls + i;
            hessian_int64[idx * total_vars + idx] = (int64_t)two_rho;
        }
    }

    /* Hessian regularization is applied AFTER auto-scaling below.
     * Adding it before auto-scaling is ineffective because the scaler
     * right-shifts everything by the same amount, making the ε-to-H ratio
     * the same as without regularization. Post-scaling regularization
     * directly sets the ratio of ε to the scaled Hessian entries. */

    /* Auto-scale Hessian and gradient to fit Q16.16 range.
     *
     * The condensed MPC Hessian (Phi^T Q Phi) can exceed INT32_MAX
     * due to large B matrix entries and accumulated products over the horizon.
     * The gradient (Phi^T Q d) accumulates even more due to error*weight products.
     *
     * Since scaling H and f by the same factor doesn't change the QP solution
     * (argmin is invariant), we find the maximum |entry| across both H (int32)
     * and f (int64), then right-shift to bring everything into representable range.
     *
     * CRITICAL: The gradient is stored in gradient_int64[] (int64) to preserve
     * full precision. Previous approach of clamping to INT32 before scaling
     * destroyed the gradient direction, causing solver infeasibility.
     */
    {
        /* Find maximum absolute value across Hessian (int64) and gradient (int64).
         * Both are now in full int64 precision — no pre-clamping to INT32. */
        int64_t max_abs = 0;
        for (int i = 0; i < total_vars * total_vars; i++)
        {
            int64_t v = hessian_int64[i];
            if (v < 0) v = -v;
            if (v > max_abs) max_abs = v;
        }
        for (int i = 0; i < total_vars; i++)
        {
            int64_t v = gradient_int64[i];
            if (v < 0) v = -v;
            if (v > max_abs) max_abs = v;
        }

        /* Compute shift: bring max_abs below INT32_MAX/4 for safety margin */
        int shift_bits = 0;
        int64_t target_val = (int64_t)INT32_MAX / 4;  /* ~536 million */
        while (max_abs > target_val && shift_bits < 62)
        {
            max_abs >>= 1;
            shift_bits++;
        }

#ifdef MPC_DEBUG_PRINT
        printf("[MPC-DBG] Auto-scale shift_bits=%d, f64[0]=%.2f, f64[1]=%.2f\n",
               shift_bits,
               (double)gradient_int64[0] / 65536.0,
               (double)gradient_int64[1] / 65536.0);
#endif

        /* Apply shift to int64 Hessian and store as int32 */
        for (int i = 0; i < total_vars * total_vars; i++)
        {
            int64_t shifted = hessian_int64[i] >> shift_bits;
            if (shifted > INT32_MAX) shifted = INT32_MAX;
            else if (shifted < INT32_MIN) shifted = INT32_MIN;
            qp_problem.hessian_matrix[i] = (fixed_point_t)shifted;
        }

        /* Apply shift to int64 gradient and store as int32 */
        for (int i = 0; i < total_vars; i++)
        {
            int64_t shifted = gradient_int64[i] >> shift_bits;
            /* Clamp (should rarely be needed after proper scaling) */
            if (shifted > INT32_MAX) shifted = INT32_MAX;
            else if (shifted < INT32_MIN) shifted = INT32_MIN;
            qp_problem.linear_cost_vector[i] = (fixed_point_t)shifted;
        }
    }

    /* Post-scaling Hessian regularization: add ε·I to control diagonals.
     * Applied AFTER auto-scaling so ε directly controls the ratio of
     * regularization to the scaled Hessian entries.
     *
     * Without this, the projected gradient solver converges in 0 iterations
     * at high speed because the Gershgorin step sizes (1/row_sum) are tiny
     * relative to the huge Hessian entries. Adding ε increases all diagonal
     * entries → smaller row sums → larger step sizes → more solver progress.
     *
     * ε = 2.0 (fixed, applied to the already-scaled Hessian). The auto-scaler
     * brings max(|H|) to ~536M (INT32_MAX/4), so typical diagonal entries
     * after scaling are 1K-100K. Adding 2*65536 = 131072 ensures the
     * regularization is at least 0.1-10% of the diagonal — enough to
     * improve conditioning without changing the solution significantly. */
    {
        /* Separate ε for steering and acceleration:
         * - ε_accel=50: strong regularization needed for brake bias to work
         *   (natural H_aa ≈ 0.1, ε must dominate for a_target anchoring)
         * - ε_steer=15: reduced regularization so tracking gradient has
         *   more influence. ε=50 made PGD step sizes tiny, preventing the
         *   solver from steering aggressively enough on curves.
         *
         * Scale with horizon: larger horizons produce worse-conditioned QPs
         * (condition number grows ~O(N²)).  Scaling ε proportionally to N
         * maintains the regularization-to-tracking ratio across horizons.
         * For N=10 (FPGA): unchanged.  For N=20 (testing): 2× stronger. */
        int horizon_scale = (horizon > 10) ? horizon : 10;
        fixed_point_t eps_steer = FP_CONST(15.0) * horizon_scale / 10;
        fixed_point_t eps_accel = FP_CONST(50.0) * horizon_scale / 10;
        for (int k = 0; k < horizon; k++)
        {
            int i_steer = k * CONTROL_DIMENSION + 0;
            int i_accel = k * CONTROL_DIMENSION + 1;
            int idx_s = i_steer * total_vars + i_steer;
            int idx_a = i_accel * total_vars + i_accel;
            qp_problem.hessian_matrix[idx_s] = fp_add_sat(
                qp_problem.hessian_matrix[idx_s], eps_steer);
            qp_problem.hessian_matrix[idx_a] = fp_add_sat(
                qp_problem.hessian_matrix[idx_a], eps_accel);
        }

        /* Acceleration bias: shift ε equilibrium from a≈0 to a≈a_target.
         *
         * With ε=50 dominating the acceleration Hessian (natural H_aa ≈ 0.1),
         * the QP optimum for acceleration is pinned near a≈0 regardless of
         * velocity error. This prevents both meaningful braking AND acceleration.
         *
         * Fix: replace ε·a² with ε·(a - a_target)², where:
         *   a_target = K_bias · (v_ref[0] - v_current)
         *
         * Expanding: ε·a² - 2·ε·a_target·a + const
         * The diagonal gets ε (already added above).
         * The gradient gets -2·ε·a_target (linear bias, added below).
         *
         * This makes the ε equilibrium: a_opt ≈ a_target ± (f_natural/(2ε)).
         * Since ε=50 >> velocity tracking gradient, a_opt ≈ a_target.
         *
         * Result:
         *   - Straights (v_ref=12, v=6): a_target=+12→clamped to a_max → full accel
         *   - Curve entry (v_ref=4.7, v=7): a_target=-4.6 → strong braking
         *   - Steady curve (v_ref=4.7, v=5): a_target=-0.6 → mild braking */
        /* Use MIN v_ref across the horizon for the brake bias.
         * The reference buffer has exactly horizon entries. */
        fixed_point_t bias_vx = current_frenet_state->longitudinal_velocity_meters_per_second;
        int brake_lookahead = horizon;
        fixed_point_t min_vref = reference_trajectory[0].reference_velocity_meters_per_second;
        for (int k = 1; k < brake_lookahead; k++)
        {
            fixed_point_t vr_k = reference_trajectory[k].reference_velocity_meters_per_second;
            if (vr_k < min_vref)
                min_vref = vr_k;
        }
        const fixed_point_t K_bias = FP_CONST(2.0);
        fixed_point_t a_target = fp_mul(K_bias, fp_sub(min_vref, bias_vx));
        /* Clamp a_target to [-7.0, +1.1]:
         * upper=+1.1: safe at 11.82 m/s, +1.15 crashed at 120s
         * lower=-7: aggressive braking (physics limit -7.7) for curve entry */
        if (a_target > FP_CONST(1.1))
            a_target = FP_CONST(1.1);
        if (a_target < FP_CONST(-7.0))
            a_target = FP_CONST(-7.0);
        /* Add -2·ε_accel·a_target to each acceleration gradient entry */
        fixed_point_t neg_two_eps_atarget = fp_neg(fp_mul(eps_accel, a_target));
        for (int k = 0; k < horizon; k++)
        {
            int i_accel = k * CONTROL_DIMENSION + 1;
            qp_problem.linear_cost_vector[i_accel] = fp_add_sat(
                qp_problem.linear_cost_vector[i_accel], neg_two_eps_atarget);
        }

        /* Steering bias: shift ε equilibrium from δ≈0 to δ≈δ_target[k].
         *
         * Same principle as acceleration bias. Without this, ε·δ² pulls
         * the optimal steering toward zero, causing persistent under-steering
         * on curves. The solver output was 0.04 rad when 0.10+ was needed,
         * because ε=50 overwhelms the lateral tracking gradient.
         *
         * Fix: replace ε·δ² with ε·(δ - δ_target)², where:
         *   δ_target[k] = L·κ[k] + PD_correction · decay[k]
         * This matches the warm-start targets exactly.
         *
         * With ε >> H_natural, the QP optimum becomes:
         *   δ* ≈ δ_target + f_tracking/(2ε)
         * So the regularization centers at the feedforward+PD point, and
         * the MPC tracking gradient provides fine corrections on top.
         *
         * Result:
         *   - Straights (κ≈0, e_y≈0): δ_target≈0, same as before
         *   - Curves with error: δ_target = L·κ + PD correction
         *   - MPC horizon still refines the solution from there */
        {
            VehicleParameters_t vp_steer_bias = vehicle_model_get_parameters();
            fixed_point_t wb_bias = vp_steer_bias.wheelbase_meters;
            fixed_point_t bias_ey   = current_frenet_state->lateral_error_meters;
            fixed_point_t bias_epsi = current_frenet_state->heading_error_radians;
            const fixed_point_t Kp_bias = FP_CONST(0.3);
            const fixed_point_t Kd_bias = FP_CONST(0.3);
            fixed_point_t pd_bias = fp_neg(fp_add(fp_mul(Kp_bias, bias_ey),
                                                  fp_mul(Kd_bias, bias_epsi)));
            for (int k = 0; k < horizon; k++)
            {
                int i_steer = k * CONTROL_DIMENSION + 0;
                fixed_point_t kappa_k = reference_trajectory[k].path_curvature_radians_per_meter;
                fixed_point_t ff_k = fp_mul(wb_bias, kappa_k);
                fixed_point_t decay = (k < 3) ? FP_ONE
                    : fp_div(FP_CONST(3.0), FP_CONST((double)(k + 1)));
                fixed_point_t steer_target = fp_add(ff_k, fp_mul(pd_bias, decay));
                fixed_point_t neg_twoeps_starget = fp_neg(fp_mul(eps_steer, steer_target));
                qp_problem.linear_cost_vector[i_steer] = fp_add_sat(
                    qp_problem.linear_cost_vector[i_steer], neg_twoeps_starget);
            }
        }
    }

    /* Build constraints (actuator limits + soft wall boundaries with slack) */
    build_qp_constraints(
        horizon,
        reference_trajectory,
        total_vars,
        n_slacks,
        current_frenet_state->longitudinal_velocity_meters_per_second,
        qp_problem.constraint_matrix,
        qp_problem.constraint_bounds,
        &qp_problem.constraint_count);

    /* Configure solver.
     * Scale max iterations with horizon: larger QPs (more variables) need
     * more PGD iterations for convergence.  For N=10 (FPGA): unchanged.
     * For N=20: 2× more iterations to handle the worse conditioning. */
    {
        int base_iter = (int)current_configuration.maximum_solver_iterations;
        int eff_iter = base_iter;
        if (horizon > 10)
            eff_iter = base_iter * horizon / 10;
        if (eff_iter > 2000) eff_iter = 2000;
        qp_config.maximum_iterations = (uint16_t)eff_iter;
    }
    qp_config.convergence_tolerance = current_configuration.solver_convergence_tolerance;
    qp_config.enable_verbose_output = 0;

    /* Warm-start: initialize QP solution from shifted previous solution.
     * Shift control sequence by one step: u[0]=u_prev[1], u[1]=u_prev[2], ...
     * Last step repeats the last known value.
     * Slack variables start at zero (no warm-start for slacks).
     *
     * INVALIDATION: If the reference curvature changed significantly since
     * the last call (straight→curve transition), the previous solution is a
     * poor initial guess that biases the solver toward "keep going straight".
     * In that case, use curvature-based feedforward as initial point instead
     * of zeros — this starts the solver close to the steady-state solution.
     *
     * FEEDFORWARD INITIALIZATION:
     * The optimal MPC steering under perfect tracking is δ = L * κ
     * (small-angle approximation of atan(L·κ), accurate for |L·κ| < 0.3).
     * Starting from feedforward eliminates the need for the solver to
     * "discover" the base steering from scratch, reducing required
     * iterations dramatically (from 5000 to ~50 in practice). */
    fixed_point_t current_ref_curvature = reference_trajectory[0].path_curvature_radians_per_meter;
    int curvature_changed = (fp_abs(fp_sub(current_ref_curvature, warm_start_prev_curvature))
                             > WARM_START_CURVATURE_THRESHOLD);
    warm_start_prev_curvature = current_ref_curvature;

    if (warm_start_available && total_controls >= CONTROL_DIMENSION && !curvature_changed)
    {
        for (int i = 0; i < total_controls - CONTROL_DIMENSION; i++)
        {
            qp_problem.initial_point[i] =
                warm_start_variables[i + CONTROL_DIMENSION];
        }
        /* Repeat last control for the final step */
        for (int i = 0; i < CONTROL_DIMENSION; i++)
        {
            qp_problem.initial_point[total_controls - CONTROL_DIMENSION + i] =
                warm_start_variables[total_controls - CONTROL_DIMENSION + i];
        }
        /* Override steering with curvature feedforward + PD correction: 
         *   δ_warm[k] = L·κ[k]  +  K_p·e_y  +  K_d·e_ψ
         *
         * The pure feedforward (L·κ) starts the solver at the steady-state
         * steering for zero error. When the car HAS error, the solver must
         * iterate from near-zero correction to the optimal. At high speed,
         * poor Hessian conditioning limits PGD step sizes, causing the solver
         * to barely move from feedforward — yielding dangerously small 
         * correction terms.
         *
         * Adding a proportional-derivative correction seed gives the solver
         * a starting point much closer to the true optimum. The solver then
         * only needs to refine the already-reasonable warm-start.
         *
         * Gains chosen so the correction is moderate (won't exceed the
         * speed-dependent steering limit for typical errors 0-0.5m): 
         *   K_p = 0.15 rad/m,  K_d = 0.5 rad/rad
         *
         * SIGN: In Frenet coordinates, positive δ → positive ω → positive
         * heading rate → positive de_y (via e_ψ×v_x coupling). So to
         * correct e_y > 0 (car too far left), we need NEGATIVE δ correction.
         * Hence: pd = -(K_p·e_y + K_d·e_ψ). */
        VehicleParameters_t vp_ws = vehicle_model_get_parameters();
        fixed_point_t wb = vp_ws.wheelbase_meters;
        fixed_point_t warm_ey   = current_frenet_state->lateral_error_meters;
        fixed_point_t warm_epsi = current_frenet_state->heading_error_radians;
        const fixed_point_t K_p_warm = FP_CONST(0.15);
        const fixed_point_t K_d_warm = FP_CONST(0.5);
        fixed_point_t pd_correction = fp_neg(fp_add(fp_mul(K_p_warm, warm_ey),
                                             fp_mul(K_d_warm, warm_epsi)));
        for (int k = 0; k < horizon; k++)
        {
            fixed_point_t kappa_k = reference_trajectory[k].path_curvature_radians_per_meter;
            fixed_point_t ff_k = fp_mul(wb, kappa_k);
            fixed_point_t decay = (k < 3) ? FP_ONE : fp_div(FP_CONST(3.0), FP_CONST((double)(k + 1)));
            qp_problem.initial_point[k * CONTROL_DIMENSION + 0] = 
                fp_add(ff_k, fp_mul(pd_correction, decay));
        }
        /* Initialize slack variables to zero */
        for (int i = total_controls; i < total_vars; i++)
        {
            qp_problem.initial_point[i] = 0;
        }
        qp_problem.use_warm_start = 1;
    }
    else
    {
        /* Cold-start with curvature feedforward: δ_ff[k] = L · κ[k]
         * This is the steady-state steering needed to follow the reference
         * curvature with zero lateral/heading error. Starting from this
         * point, the solver only needs to find small corrections. */
        VehicleParameters_t vp = vehicle_model_get_parameters();
        fixed_point_t wheelbase = vp.wheelbase_meters;

        /* Same PD correction as warm-start path (NEGATED for correct feedback) */
        fixed_point_t cold_ey   = current_frenet_state->lateral_error_meters;
        fixed_point_t cold_epsi = current_frenet_state->heading_error_radians;
        fixed_point_t cold_pd = fp_neg(fp_add(fp_mul(FP_CONST(0.15), cold_ey),
                                       fp_mul(FP_CONST(0.5), cold_epsi)));

        for (int k = 0; k < horizon; k++)
        {
            fixed_point_t kappa_k = reference_trajectory[k].path_curvature_radians_per_meter;
            fixed_point_t delta_ff = fp_mul(wheelbase, kappa_k);
            fixed_point_t decay = (k < 3) ? FP_ONE : fp_div(FP_CONST(3.0), FP_CONST((double)(k + 1)));
            qp_problem.initial_point[k * CONTROL_DIMENSION + 0] = 
                fp_add(delta_ff, fp_mul(cold_pd, decay));
            qp_problem.initial_point[k * CONTROL_DIMENSION + 1] = 0;
        }
        for (int i = total_controls; i < total_vars; i++)
        {
            qp_problem.initial_point[i] = 0;
        }
        qp_problem.use_warm_start = 1;

#ifdef MPC_DEBUG_PRINT
        if (curvature_changed)
            printf("[MPC-DBG] Warm-start INVALIDATED: curvature change, using feedforward init (L*κ)\n");
        else
            printf("[MPC-DBG] Cold-start with feedforward init (L*κ = %.4f)\n",
                   FP_TO_DOUBLE(fp_mul(wheelbase, current_ref_curvature)));
#endif
    }

    /* ---------------------------------------------------------------
     * Trust-region bounds: tighten actuator constraints around warm-start.
     *
     * With proper int64 step computation, the QP solver can now iterate
     * at high speed. Trust-region prevents degenerate full-lock solutions
     * from linearization error at hairpins.
     *
     * Fix: bound the solution to lie within Δ of the warm-start:
     *   u_warm[k] - Δ ≤ u[k] ≤ u_warm[k] + Δ
     * intersected with physical limits.
     *
     * NOTE: Steering trust-region re-enabled with wider base.
     * The heading error clamp in the free response prevents the QP
     * from being biased toward curvature-tracking at the expense of
     * lateral correction, so the trust-region no longer locks in bad solutions.
     * --------------------------------------------------------------- */
    {
        fixed_point_t current_vx = fp_abs(
            current_frenet_state->longitudinal_velocity_meters_per_second);

        const fixed_point_t delta_steer_base = FP_CONST(0.5);   /* wider than before */
        const fixed_point_t delta_accel_base = FP_CONST(6.0);
        const fixed_point_t v_ref_trust = FP_CONST(8.0);

        fixed_point_t v_scale = fp_div(current_vx, v_ref_trust);
        if (v_scale < FP_ONE) v_scale = FP_ONE;

        fixed_point_t delta_steer = fp_div(delta_steer_base, v_scale);
        fixed_point_t delta_accel = fp_div(delta_accel_base, v_scale);

        const fixed_point_t min_delta_steer = FP_CONST(0.10);
        const fixed_point_t min_delta_accel = FP_CONST(2.0);
        if (delta_steer < min_delta_steer) delta_steer = min_delta_steer;
        if (delta_accel < min_delta_accel) delta_accel = min_delta_accel;

        int actuator_constraints_per_step = 4;
        for (int k = 0; k < horizon; k++)
        {
            int cb = k * actuator_constraints_per_step;
            fixed_point_t u_warm_steer = qp_problem.initial_point[k * CONTROL_DIMENSION + 0];
            fixed_point_t u_warm_accel = qp_problem.initial_point[k * CONTROL_DIMENSION + 1];

            /* Tighten steering upper */
            fixed_point_t steer_upper = fp_add(u_warm_steer, delta_steer);
            if (steer_upper < qp_problem.constraint_bounds[cb + 0])
                qp_problem.constraint_bounds[cb + 0] = steer_upper;

            /* Tighten steering lower */
            fixed_point_t steer_lower = fp_sub(delta_steer, u_warm_steer);
            if (steer_lower < qp_problem.constraint_bounds[cb + 1])
                qp_problem.constraint_bounds[cb + 1] = steer_lower;

            /* Tighten accel upper: b₂ = min(current_b₂, u_warm + Δ) */
            fixed_point_t accel_upper = fp_add(u_warm_accel, delta_accel);
            if (accel_upper < qp_problem.constraint_bounds[cb + 2])
                qp_problem.constraint_bounds[cb + 2] = accel_upper;

            /* Tighten accel lower: b₃ = min(current_b₃, Δ - u_warm) */
            fixed_point_t accel_lower = fp_sub(delta_accel, u_warm_accel);
            if (accel_lower < qp_problem.constraint_bounds[cb + 3])
                qp_problem.constraint_bounds[cb + 3] = accel_lower;
        }
    }

    /* Solve QP */
    QuadraticProgramStatus_t qp_status = qp_solver_solve(
        &qp_problem, &qp_config, &qp_solution);

#ifdef MPC_DEBUG_PRINT
    /* Report slack variable values (how much wall constraint was relaxed) */
    if (n_slacks > 0) {
        fixed_point_t max_slack = 0;
        for (int i = 0; i < n_slacks; i++) {
            fixed_point_t s = qp_solution.optimal_variables[total_controls + i];
            if (s > max_slack) max_slack = s;
        }
        if (max_slack > 0) {
            printf("[MPC-DBG] Wall slack active: max=%.4f m (constraint relaxed)\n",
                   FP_TO_DOUBLE(max_slack));
        }
    }
#endif

    /* Save control portion of solution for next warm-start */
    memcpy(warm_start_variables, qp_solution.optimal_variables,
           total_controls * sizeof(fixed_point_t));
    warm_start_available = 1;

    /* Extract first control from solution */
    fixed_point_t optimal_steering = qp_solution.optimal_variables[0];
    fixed_point_t optimal_accel    = qp_solution.optimal_variables[1];

    /* Saturate control to vehicle limits */
    ControlInput_t raw_control;
    raw_control.steering_angle_radians = optimal_steering;
    raw_control.acceleration_meters_per_second_squared = optimal_accel;

    ControlInput_t saturated_control = vehicle_model_saturate_control(&raw_control);

    /* Fill result structure */
    result->optimal_control = saturated_control;
    result->iterations_used = qp_solution.iteration_count;
    result->final_cost = qp_solution.constraint_residual; /* Using residual as cost proxy */

    /* Map QP status to MPC status.
     *
     * QP_STATUS_INFEASIBLE maps to MAX_ITERATIONS_REACHED (not INFEASIBLE)
     * because the wall constraints are SOFT (with slack variables and
     * quadratic penalty).  The PGD solver always enforces actuator box
     * constraints (Phase 2 clamping), so the control output is always
     * within physical limits.  The "infeasibility" only means the soft
     * wall constraints exceeded the convergence tolerance — this is
     * expected for extreme states (e_psi≈π, large e_y) where the MPC
     * still produces the best feasible control. */
    switch (qp_status)
    {
    case QP_STATUS_OPTIMAL:
        result->solver_status = MPC_STATUS_SUCCESS;
        break;
    case QP_STATUS_MAXIMUM_ITERATIONS_REACHED:
    case QP_STATUS_INFEASIBLE:
        result->solver_status = MPC_STATUS_MAXIMUM_ITERATIONS_REACHED;
        break;
    default:
        result->solver_status = MPC_STATUS_ERROR;
        break;
    }

    /* Store control for next iteration's rate penalty */
    previous_control_input = saturated_control;

    return result->solver_status;
}

MpcConfiguration_t mpc_get_configuration(void)
{
    return current_configuration;
}

void mpc_set_configuration(const MpcConfiguration_t *configuration)
{
    if (configuration != NULL)
    {
        current_configuration = *configuration;
    }
}

void mpc_reset(void)
{
    /* Clear previous control (no rate penalty on first control after reset) */
    previous_control_input.steering_angle_radians = 0;
    previous_control_input.acceleration_meters_per_second_squared = 0;

    /* Clear warm-start */
    memset(warm_start_variables, 0, sizeof(warm_start_variables));
    warm_start_available = 0;
    warm_start_prev_curvature = 0;
}
