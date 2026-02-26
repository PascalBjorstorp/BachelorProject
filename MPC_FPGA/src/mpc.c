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

/** Number of states in Frenet vehicle model: [e_y, e_psi, v_x, v_y, omega, omega_w] */
#define STATE_DIMENSION 6

/** Number of control inputs: [steering, motor_torque] */
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
static int saved_horizon_steps = 0;

/*===========================================================================
 * Heading Angle Normalization
 *===========================================================================*/

/**
 * Normalize an angle to the [-pi, +pi] range.
 * Prevents large discontinuities at the ±pi boundary.
 */
static fixed_point_t normalize_angle(fixed_point_t angle)
{
    while (angle > FP_PI)
    {
        angle = fp_sub(angle, FP_TWO_PI);
    }
    while (angle < -FP_PI)
    {
        angle = fp_add(angle, FP_TWO_PI);
    }
    return angle;
}

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
     * In Frenet coordinates, the state is [e_y, e_psi, v_x, v_y, w, w_w].
     * Lateral error (e_y) and heading error (e_psi) replace global XY.
     * Position and heading tracking now work together naturally.
     *
     * NOTE: All weights scaled to prevent Q16.16 overflow in
     * condensed Hessian construction (Phi^T Q Phi accumulates large
     * products over 10-step horizon).
     */
    config.weight_lateral_error  = FP_CONST(10.0);               /* keep car on path */
    config.weight_heading_error  = FP_CONST(7.0);                /* heading alignment (needed for corner tracking) */
    config.weight_velocity   = FP_CONST(0.05);                    /* deprioritize speed vs heading */
    config.weight_lateral_velocity = FP_CONST(0.3);               /* penalize sideslip to prevent drifting */
    config.weight_yaw_rate   = FP_CONST(0.07);                    /* light yaw damping; 0 causes corner exit instability; >0.15 causes curve entry aggression */
    config.weight_wheel_speed = FP_CONST(0.01);                   /* light penalty on wheel speed */

    /* Control effort weights */
    config.weight_steering_effort  = FP_CONST(0.001);    /* regularize steering magnitude */
    config.weight_torque_effort     = FP_CONST(0.01);    /* regularize torque magnitude */

    /* Control rate weights — tuned so MPC naturally limits steering rate
     * without relying on the external rate clamp.  A high steering-rate
     * penalty makes large step-to-step changes expensive, preventing the
     * bang-bang oscillation that occurs when the external clamp truncates
     * a multi-step plan.  The external clamp (±0.15 rad/step) remains as
     * a safety net. */
    config.weight_steering_rate  = FP_CONST(1.0);                    /* smooth steering; penalizes step-to-step steering changes */
    config.weight_torque_rate     = FP_CONST(0.1);                    /* prevent torque chattering */

    /* Cross-call rate scale: 1.0 = call interval matches dt (default for offline use).
     * Set to smaller value (e.g., 0.1) when MPC is called 10× faster than dt.
     */
    config.cross_call_rate_scale = FP_ONE;

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
        if ((env_val = getenv("MPC_W_LAT_VEL")) != NULL)
            config.weight_lateral_velocity = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_YAW_RATE")) != NULL)
            config.weight_yaw_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_STEER_RATE")) != NULL)
            config.weight_steering_rate = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_STEER_EFFORT")) != NULL)
            config.weight_steering_effort = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_TORQUE_RATE")) != NULL)
            config.weight_torque_rate = DOUBLE_TO_FP(atof(env_val));
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
 * Build the QP Hessian and linear cost from predicted vehicle dynamics.
 *
 * @param current_state        Current Frenet state x[0]
 * @param reference_trajectory Reference points for steps 1..N
 * @param horizon_steps        Number of prediction steps N
 * @param total_variables      Total QP variables (controls + slacks)
 * @param hessian_matrix       Output: Hessian matrix H (total_variables × total_variables)
 * @param linear_cost_vector   Output: Linear cost vector f (total_variables)
 */
static void build_qp_from_prediction(
    const FrenetState_t *current_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    int horizon_steps,
    int total_variables,
    fixed_point_t *hessian_matrix,
    fixed_point_t *linear_cost_vector)
{
    int n_vars = total_variables;  /* Hessian stride includes slack columns */

    /* ---------------------------------------------------------------
     * Step 1: Linearize vehicle model in Frenet frame
     *
     * Uses the Frenet linearization which produces a 6×6 A and 6×2 B
     * for state [e_y, e_psi, v_x, v_y, omega, omega_w].
     *
     * The path curvature from the first reference point is used for
     * the linearization (constant across horizon). Body dynamics are
     * linearized at the actual velocity with a minimum floor.
     *
     * Steering is linearized at δ=0 to prevent velocity-steering
     * cross-coupling oscillation (same rationale as global version).
     * --------------------------------------------------------------- */
    fixed_point_t A[6][6];
    fixed_point_t B[6][2];

    ControlInput_t linearization_control;
    linearization_control.steering_angle_radians = 0;
    linearization_control.motor_torque_newton_meters = 0;

    /* Create Frenet state with velocity floor for linearization */
    FrenetState_t linearization_state = *current_state;
    fixed_point_t min_linearization_velocity = FP_CONST(2.0);
    if (linearization_state.longitudinal_velocity_meters_per_second < min_linearization_velocity)
        linearization_state.longitudinal_velocity_meters_per_second = min_linearization_velocity;

    /* Ensure wheel speed is consistent with velocity floor */
    VehicleParameters_t vp = vehicle_model_get_parameters();
    if (linearization_state.longitudinal_velocity_meters_per_second > current_state->longitudinal_velocity_meters_per_second)
    {
        /* Velocity was clamped upward — adjust wheel speed proportionally
         * to maintain the same slip ratio, preventing F_x weight transfer
         * from distorting the linearization at low speed. */
        linearization_state.wheel_speed_radians_per_second =
            fp_div(linearization_state.longitudinal_velocity_meters_per_second,
                   vp.wheel_radius_meters);
    }
    else if (linearization_state.wheel_speed_radians_per_second == 0 &&
             linearization_state.longitudinal_velocity_meters_per_second > 0)
    {
        linearization_state.wheel_speed_radians_per_second =
            fp_div(linearization_state.longitudinal_velocity_meters_per_second,
                   vp.wheel_radius_meters);
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
     * Stabilize wheel dynamics (row 5 of A matrix = omega_w in Frenet).
     *
     * With small wheel inertia, the wheel dynamics
     * time constant τ = Iw·vx/(Cx·Rw²) ≈ 0.022s at vx=1 m/s,
     * much smaller than dt=50ms. Forward Euler is unstable when
     * dt >> τ, producing |A[5][5]| >> 1 (e.g., -43 at low speed).
     *
     * Fix: rescale row 5 of A so A[5][5] is clamped to [-0.95, 0.95].
     * --------------------------------------------------------------- */
    {
        fixed_point_t abs_a55 = fp_abs(A[5][5]);
        const fixed_point_t stability_limit = FP_CONST(0.95);

        if (abs_a55 > stability_limit)
        {
            fixed_point_t target = (A[5][5] < 0) ? fp_neg(stability_limit) : stability_limit;
            fixed_point_t num = fp_sub(target, FP_ONE);
            fixed_point_t den = fp_sub(A[5][5], FP_ONE);

            if (den != 0)
            {
                fixed_point_t scale = fp_div(num, den);

                for (int j = 0; j < STATE_DIMENSION; j++)
                {
                    if (j != 5)
                    {
                        A[5][j] = fp_mul(A[5][j], scale);
                    }
                }
                /* Also scale B row 5 to maintain consistency */
                B[5][0] = fp_mul(B[5][0], scale);
                B[5][1] = fp_mul(B[5][1], scale);
            }

            A[5][5] = target;
        }
    }

    /* ---------------------------------------------------------------
     * Stabilize yaw rate dynamics (row 4 of A matrix = ω in Frenet).
     *
     * The lateral tire force derivatives dFy/dω scale as 1/vx through
     * the slip angle derivative dα/dω = lf/(vx² + ...). At the
     * linearization velocity floor of 2.0 m/s, the yaw rate time
     * constant τ ≈ Iz / (C_Sf·F_zf·lf·dt_tire / vx) can be ~19 ms,
     * smaller than dt=50ms. This produces |A[4][4]| > 1 (e.g., -1.65),
     * causing oscillatory Phi matrices and steering bang-bang.
     *
     * Fix: same rescaling as row 5 — clamp A[4][4] to [-0.95, 0.95].
     * --------------------------------------------------------------- */
    {
        fixed_point_t abs_a44 = fp_abs(A[4][4]);
        const fixed_point_t stability_limit = FP_CONST(0.95);

        if (abs_a44 > stability_limit)
        {
            fixed_point_t target = (A[4][4] < 0) ? fp_neg(stability_limit) : stability_limit;
            fixed_point_t num = fp_sub(target, FP_ONE);
            fixed_point_t den = fp_sub(A[4][4], FP_ONE);

            if (den != 0)
            {
                fixed_point_t scale = fp_div(num, den);

                for (int j = 0; j < STATE_DIMENSION; j++)
                {
                    if (j != 4)
                    {
                        A[4][j] = fp_mul(A[4][j], scale);
                    }
                }
                /* Also scale B row 4 to maintain consistency */
                B[4][0] = fp_mul(B[4][0], scale);
                B[4][1] = fp_mul(B[4][1], scale);
            }

            A[4][4] = target;
        }
    }

    /* ---------------------------------------------------------------
     * Step 2: Compute Phi[m] = A[m] * Phi[m-1] for m = 0 .. N-1
     *
     * Phi[0] = B
     * Phi[m] = A[m] * Phi[m-1]
     *
     * where A[m] varies per step because the path curvature changes
     * along the prediction horizon. Only A[1][2] = -dt*kappa[m]
     * differs between steps; all body-frame dynamics (rows 2-5) are
     * the same since they don't depend on path curvature.
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
        current_state->yaw_rate_radians_per_second,
        current_state->wheel_speed_radians_per_second
    };

    fixed_point_t x_prev[STATE_DIMENSION];
    for (int s = 0; s < STATE_DIMENSION; s++)
        x_prev[s] = x0[s];

    /* Phi[0] = B (direct control-to-state mapping, no A propagation) */
    for (int r = 0; r < STATE_DIMENSION; r++)
        for (int c = 0; c < CONTROL_DIMENSION; c++)
            Phi[0][r][c] = B[r][c];

    /* Step k=0: free response only (Phi[0]=B already set, uses kappa[0]) */
    A[1][2] = fp_neg(fp_mul(current_configuration.time_step_seconds,
                            reference_trajectory[0].path_curvature_radians_per_meter));
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
        d[0][1] = normalize_angle(fp_sub(x_free[1], reference_trajectory[0].reference_heading_error_radians));
        d[0][2] = fp_sub(x_free[2], reference_trajectory[0].reference_velocity_meters_per_second);
        d[0][3] = fp_sub(x_free[3], reference_trajectory[0].reference_lateral_velocity_meters_per_second);
        d[0][4] = fp_sub(x_free[4], reference_trajectory[0].reference_yaw_rate_radians_per_second);
        d[0][5] = fp_sub(x_free[5], reference_trajectory[0].reference_wheel_speed_radians_per_second);
        saved_free_response_ey[0] = x_free[0];
        for (int s = 0; s < STATE_DIMENSION; s++)
            x_prev[s] = x_free[s];
        x_prev[1] = normalize_angle(x_prev[1]);
    }

    /* Steps k=1..N-1: Phi propagation AND free response share A[1][2] */
    for (int k = 1; k < horizon_steps; k++)
    {
        A[1][2] = fp_neg(fp_mul(current_configuration.time_step_seconds,
                                reference_trajectory[k].path_curvature_radians_per_meter));

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
            d[k][1] = normalize_angle(fp_sub(x_free[1], reference_trajectory[k].reference_heading_error_radians));
            d[k][2] = fp_sub(x_free[2], reference_trajectory[k].reference_velocity_meters_per_second);
            d[k][3] = fp_sub(x_free[3], reference_trajectory[k].reference_lateral_velocity_meters_per_second);
            d[k][4] = fp_sub(x_free[4], reference_trajectory[k].reference_yaw_rate_radians_per_second);
            d[k][5] = fp_sub(x_free[5], reference_trajectory[k].reference_wheel_speed_radians_per_second);
            saved_free_response_ey[k] = x_free[0];
            for (int s = 0; s < STATE_DIMENSION; s++)
                x_prev[s] = x_free[s];
            x_prev[1] = normalize_angle(x_prev[1]);
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
     * Frenet state weights: [e_y, e_psi, v_x, v_y, omega, omega_w]
     * Lateral error and heading error are the primary tracking states.
     * --------------------------------------------------------------- */
    fixed_point_t Q[STATE_DIMENSION] = {
        current_configuration.weight_lateral_error,
        current_configuration.weight_heading_error,
        current_configuration.weight_velocity,
        current_configuration.weight_lateral_velocity,
        current_configuration.weight_yaw_rate,
        current_configuration.weight_wheel_speed
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
     * H_block(ci, cj) = 2 * sum_{k=max(ci,cj)}^{N-1} Phi[k-ci]^T Q Phi[k-cj]
     *
     * where the sum ranges over all prediction steps k where both
     * control ci and control cj have an effect on state x[k+1].
     * --------------------------------------------------------------- */
    memset(hessian_matrix, 0,
        (size_t)n_vars * (size_t)n_vars * sizeof(fixed_point_t));

    /* ---------------------------------------------------------------
     * Step 5a: Precompute G[mi][mj] = PhiQ[mi]^T * Phi[mj]  (2×2 block)
     *
     * This separates the MULTIPLY phase (G precomputation, N² blocks)
     * from the ACCUMULATE phase (Hessian summation, addition only).
     * For N=10: 2400 multiplies (precompute) + 1100 additions (accumulate)
     * vs. 9240 multiplies in the original triple loop.
     * ~2.6× faster Hessian construction.
     * --------------------------------------------------------------- */
    static int64_t G[MAXIMUM_HORIZON_STEPS][MAXIMUM_HORIZON_STEPS][CONTROL_DIMENSION][CONTROL_DIMENSION];

    for (int mi = 0; mi < horizon_steps; mi++)
    {
        for (int mj = 0; mj < horizon_steps; mj++)
        {
            for (int a = 0; a < CONTROL_DIMENSION; a++)
            {
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                {
                    int64_t sum = 0;
                    for (int s = 0; s < STATE_DIMENSION; s++)
                    {
                        sum += ((int64_t)PhiQ[mi][a][s] * Phi[mj][s][b]) >> FP_FRAC_BITS;
                    }
                    G[mi][mj][a][b] = sum;
                }
            }
        }
    }

    /* ---------------------------------------------------------------
     * Step 5b: Accumulate Hessian from precomputed G blocks (addition only)
     * --------------------------------------------------------------- */
    for (int ci = 0; ci < horizon_steps; ci++)
    {
        for (int cj = ci; cj < horizon_steps; cj++)
        {
            int64_t block64[CONTROL_DIMENSION][CONTROL_DIMENSION];
            for (int a = 0; a < CONTROL_DIMENSION; a++)
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                    block64[a][b] = 0;

            /* Sum precomputed G blocks — no multiplies needed */
            for (int k = cj; k < horizon_steps; k++)
            {
                int mi = k - ci;
                int mj = k - cj;
                for (int a = 0; a < CONTROL_DIMENSION; a++)
                    for (int b = 0; b < CONTROL_DIMENSION; b++)
                        block64[a][b] += G[mi][mj][a][b];
            }

            /* Write 2×2 block to Hessian (with ×2 for QP convention) */
            int row = ci * CONTROL_DIMENSION;
            int col = cj * CONTROL_DIMENSION;

            for (int a = 0; a < CONTROL_DIMENSION; a++)
            {
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                {
                    int64_t val64 = block64[a][b] * 2;
                    /* Clamp to int32_t */
                    fixed_point_t val;
                    if (val64 > INT32_MAX) val = INT32_MAX;
                    else if (val64 < INT32_MIN) val = INT32_MIN;
                    else val = (fixed_point_t)val64;

                    hessian_matrix[(row + a) * n_vars + (col + b)] = val;

                    /* Symmetric entry */
                    if (ci != cj)
                    {
                        hessian_matrix[(col + b) * n_vars + (row + a)] = val;
                    }
                }
            }
        }
    }

    /* Control effort contribution: 2*R on diagonal (saturating add) */
    fixed_point_t two_w_steer = fp_mul(FP_TWO,
        current_configuration.weight_steering_effort);
    fixed_point_t two_w_torque = fp_mul(FP_TWO,
        current_configuration.weight_torque_effort);

    for (int ci = 0; ci < horizon_steps; ci++)
    {
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);
        hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s], two_w_steer);
        hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v], two_w_torque);
    }

    /* Rate penalty contribution: tridiagonal band structure.
     * All constant products hoisted out of the loop to avoid recomputation.
     * Structure: +2w on boundary diagonals, +4w on interior diagonals,
     * -2w on off-diagonals (k,k-1) and (k-1,k). */
    fixed_point_t w_sr = current_configuration.weight_steering_rate;
    fixed_point_t w_vr = current_configuration.weight_torque_rate;
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
        hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s], two_sr_cross);
        hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v], two_vr_cross);
        if (horizon_steps > 1)
        {
            hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s], two_sr);
            hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v], two_vr);
        }
    }

    /* Interior steps 1..N-2: +4w diagonal, -2w off-diagonal */
    for (int ci = 1; ci < horizon_steps - 1; ci++)
    {
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);
        hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s], four_sr);
        hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v], four_vr);

        /* Off-diagonal: H[ci-1, ci] = H[ci, ci-1] = -2*w_rate */
        int prev_s = ((ci - 1) * 2) * n_vars + (ci * 2);
        int prev_v = ((ci - 1) * 2 + 1) * n_vars + (ci * 2 + 1);
        int sym_s  = (ci * 2) * n_vars + ((ci - 1) * 2);
        int sym_v  = (ci * 2 + 1) * n_vars + ((ci - 1) * 2 + 1);
        hessian_matrix[prev_s] = fp_add_sat(hessian_matrix[prev_s], neg_two_sr);
        hessian_matrix[sym_s]  = fp_add_sat(hessian_matrix[sym_s],  neg_two_sr);
        hessian_matrix[prev_v] = fp_add_sat(hessian_matrix[prev_v], neg_two_vr);
        hessian_matrix[sym_v]  = fp_add_sat(hessian_matrix[sym_v],  neg_two_vr);
    }

    /* Last step N-1: +2w diagonal, -2w off-diagonal to step N-2 */
    if (horizon_steps > 1)
    {
        int ci = horizon_steps - 1;
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);
        hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s], two_sr);
        hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v], two_vr);

        int prev_s = ((ci - 1) * 2) * n_vars + (ci * 2);
        int prev_v = ((ci - 1) * 2 + 1) * n_vars + (ci * 2 + 1);
        int sym_s  = (ci * 2) * n_vars + ((ci - 1) * 2);
        int sym_v  = (ci * 2 + 1) * n_vars + ((ci - 1) * 2 + 1);
        hessian_matrix[prev_s] = fp_add_sat(hessian_matrix[prev_s], neg_two_sr);
        hessian_matrix[sym_s]  = fp_add_sat(hessian_matrix[sym_s],  neg_two_sr);
        hessian_matrix[prev_v] = fp_add_sat(hessian_matrix[prev_v], neg_two_vr);
        hessian_matrix[sym_v]  = fp_add_sat(hessian_matrix[sym_v],  neg_two_vr);
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
     * Store in gradient_int64[] to preserve full precision for auto-scaling. */
    for (int ci = 0; ci < horizon_steps; ci++)
    {
        for (int a = 0; a < CONTROL_DIMENSION; a++)
        {
            int64_t sum64 = 0;

            for (int k = ci; k < horizon_steps; k++)
            {
                int m = k - ci;

                for (int s = 0; s < STATE_DIMENSION; s++)
                {
                    /* PhiQ[m][a][s] already contains Phi[m][s][a]*Q[s] */
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
                previous_control_input.motor_torque_newton_meters));
    
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
 * - Motor torque: min_torque <= T <= max_torque
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
#define WALL_SLACK_PENALTY_WEIGHT  DOUBLE_TO_FP(80.0)

/** First prediction step to apply wall constraints (skip 0,1 where Phi[0]=0) */
#define WALL_CONSTRAINT_START  2

/** Apply wall constraints every N-th step to reduce constraint density */
#define WALL_CONSTRAINT_STRIDE 3

/** Extra safety margin subtracted from wall bounds (meters) */
#define WALL_MARGIN_FP DOUBLE_TO_FP(0.10)

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
 * - Motor torque: min_torque <= T <= max_torque
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

    /* Steering limit: use hardware maximum.
     * The Pacejka tire model in the linearization naturally captures
     * tire saturation at high slip angles, so no additional speed-dependent
     * steering constraint is needed.  The reduced cornering stiffness
     * (measured: C_Sf=2.259, C_Sr=3.909) already accurately reflects
     * how much lateral force each steering input actually produces. */
    fixed_point_t effective_max_steer = vehicle_params.maximum_steering_angle_radians;

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

        /* Constraint 0: steering <= effective_max_steering (speed-dependent) */
        constraint_matrix[(constraint_base + 0) * total_variables + control_base] =
            FP_ONE;
        constraint_bounds[constraint_base + 0] =
            effective_max_steer;

        /* Constraint 1: -steering <= effective_max_steering (speed-dependent) */
        constraint_matrix[(constraint_base + 1) * total_variables + control_base] =
            fp_neg(FP_ONE);
        constraint_bounds[constraint_base + 1] =
            effective_max_steer;

        /* Constraint 2: torque <= max_torque */
        constraint_matrix[(constraint_base + 2) * total_variables + (control_base + 1)] =
            FP_ONE;
        constraint_bounds[constraint_base + 2] =
            vehicle_params.maximum_motor_torque_newton_meters;

        /* Constraint 3: -torque <= -min_torque */
        constraint_matrix[(constraint_base + 3) * total_variables + (control_base + 1)] =
            fp_neg(FP_ONE);
        constraint_bounds[constraint_base + 3] =
            fp_neg(vehicle_params.minimum_motor_torque_newton_meters);
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
    current_configuration = get_default_configuration();

    /* Initialize vehicle model */
    vehicle_model_initialize();

    /* Clear previous control */
    previous_control_input.steering_angle_radians = 0;
    previous_control_input.motor_torque_newton_meters = 0;

    /* Clear warm-start */
    memset(warm_start_variables, 0, sizeof(warm_start_variables));
    warm_start_available = 0;
    warm_start_prev_curvature = 0;

    mpc_initialized_flag = 1;
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

    /* Initialize vehicle model */
    vehicle_model_initialize();

    /* Clear previous control */
    previous_control_input.steering_angle_radians = 0;
    previous_control_input.motor_torque_newton_meters = 0;

    /* Clear warm-start */
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
     * Uses total_vars for Hessian stride — slack columns are zero. */
    build_qp_from_prediction(
        current_frenet_state,
        reference_trajectory,
        horizon,
        total_vars,
        qp_problem.hessian_matrix,
        qp_problem.linear_cost_vector);

    /* Add slack variable penalty to Hessian diagonal.
     * Cost: ρ * s² → Hessian diagonal entry: 2 * ρ
     * This strongly discourages wall violation while keeping the QP feasible. */
    {
        fixed_point_t two_rho = fp_mul(FP_TWO, WALL_SLACK_PENALTY_WEIGHT);
        for (int i = 0; i < n_slacks; i++)
        {
            int idx = total_controls + i;
            qp_problem.hessian_matrix[idx * total_vars + idx] = two_rho;
        }
    }

    /* Auto-scale Hessian and gradient to fit Q16.16 range.
     *
     * The condensed MPC Hessian (Phi^T Q Phi) can exceed INT32_MAX
     * due to large B matrix entries (especially torque→wheel_speed: B[6][1]≈2.12).
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
        /* Find maximum absolute value across Hessian (int32) and gradient (int64) */
        int64_t max_abs = 0;
        for (int i = 0; i < total_vars * total_vars; i++)
        {
            int64_t v = (int64_t)qp_problem.hessian_matrix[i];
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
        while (max_abs > target_val && shift_bits < 30)
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

        if (shift_bits > 0)
        {
            for (int i = 0; i < total_vars * total_vars; i++)
            {
                qp_problem.hessian_matrix[i] >>= shift_bits;
            }
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

    /* Configure solver */
    qp_config.maximum_iterations = current_configuration.maximum_solver_iterations;
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
     * In that case, cold-start to let the solver find the new optimum
     * without the stale bias. */
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
        /* Initialize slack variables to zero */
        for (int i = total_controls; i < total_vars; i++)
        {
            qp_problem.initial_point[i] = 0;
        }
        qp_problem.use_warm_start = 1;
    }
    else if (curvature_changed)
    {
#ifdef MPC_DEBUG_PRINT
        printf("[MPC-DBG] Warm-start INVALIDATED: curvature change %.4f → %.4f\n",
               FP_TO_DOUBLE(warm_start_prev_curvature),
               FP_TO_DOUBLE(current_ref_curvature));
#endif
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
    fixed_point_t optimal_torque   = qp_solution.optimal_variables[1];

    /* Saturate control to vehicle limits */
    ControlInput_t raw_control;
    raw_control.steering_angle_radians = optimal_steering;
    raw_control.motor_torque_newton_meters = optimal_torque;

    ControlInput_t saturated_control = vehicle_model_saturate_control(&raw_control);

    /* Fill result structure */
    result->optimal_control = saturated_control;
    result->iterations_used = qp_solution.iteration_count;
    result->final_cost = qp_solution.constraint_residual; /* Using residual as cost proxy */

    /* Map QP status to MPC status */
    switch (qp_status)
    {
    case QP_STATUS_OPTIMAL:
        result->solver_status = MPC_STATUS_SUCCESS;
        break;
    case QP_STATUS_MAXIMUM_ITERATIONS_REACHED:
        result->solver_status = MPC_STATUS_MAXIMUM_ITERATIONS_REACHED;
        break;
    case QP_STATUS_INFEASIBLE:
        result->solver_status = MPC_STATUS_INFEASIBLE;
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
    previous_control_input.motor_torque_newton_meters = 0;

    /* Clear warm-start */
    memset(warm_start_variables, 0, sizeof(warm_start_variables));
    warm_start_available = 0;
    warm_start_prev_curvature = 0;
}
