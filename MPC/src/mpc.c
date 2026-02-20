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
#include <stdlib.h>

/*===========================================================================
 * Internal Constants
 *===========================================================================*/

/** Number of states in vehicle model: [x, y, heading, v_x, v_y, omega, omega_w] */
#define STATE_DIMENSION 7

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

    /* State tracking weights
     *
     * With the condensed MPC formulation, each weight is effectively
     * amplified by the prediction horizon (N=10 steps). So moderate
     * values here produce strong tracking behavior.
     *
     * Position weights are set to ZERO because global XY tracking
     * fights against heading tracking in corners. When looking ahead on a curved
     * path, steering toward future waypoint positions often requires pointing
     * AWAY from the correct heading direction.
     *
     * With zero position weights, the MPC only tracks heading and velocity.
     * Position following is achieved indirectly by tracking the correct heading
     * (which naturally follows the path via vehicle kinematics).
     *
     * Future improvement: Use Frenet (path-relative) coordinates instead of
     * global XY to properly decouple lateral and heading tracking.
     *
     * NOTE: All weights scaled by 0.1 to prevent Q16.16 overflow in
     * condensed Hessian construction (Phi^T Q Phi accumulates large
     * products over 10-step horizon). Relative ratios preserved.
     */
    config.weight_position_x = FP_CONST(0.0);                      /* 0.0 - position tracked via cross-track heading correction */
    config.weight_position_y = FP_CONST(0.0);                      /* 0.0 - position tracked via cross-track heading correction */
    config.weight_heading    = FP_CONST(5);                     /* 5.0 - primary tracking objective (dominant) */
    config.weight_velocity   = FP_CONST(0.05);                    /* 0.05 - deprioritize speed vs heading */
    config.weight_lateral_velocity = FP_CONST(0.05);              /* 0.05 - light sideslip penalty */
    config.weight_yaw_rate   = FP_CONST(0.01);                     /* 0.01 - light yaw damping (use MPC_W_YAW_RATE to tune) */
    config.weight_wheel_speed = FP_CONST(0.01);                   /* 0.01 - light penalty on wheel speed */

    /* Control effort weights */
    config.weight_steering_effort  = FP_CONST(0.001);    /* 0.001 */
    config.weight_torque_effort     = FP_CONST(0.01);    /* 0.01 - regularize torque magnitude */

    /* Control rate weights — tuned so MPC naturally limits steering rate
     * without relying on the external rate clamp.  A high steering-rate
     * penalty makes large step-to-step changes expensive, preventing the
     * bang-bang oscillation that occurs when the external clamp truncates
     * a multi-step plan.  The external clamp (±0.15 rad/step) remains as
     * a safety net. */
    config.weight_steering_rate  = FP_CONST(0.5);                    /* 0.5 - prevent oscillation (was 0.05) */
    config.weight_torque_rate     = FP_CONST(0.1);                    /* 0.1 - prevent torque chattering */

    /* Cross-call rate scale: 1.0 = call interval matches dt (default for offline use).
     * Set to smaller value (e.g., 0.1) when MPC is called 10× faster than dt.
     */
    config.cross_call_rate_scale = FP_ONE;                            /* 1.0 */

    /* Solver parameters */
    config.maximum_solver_iterations = MPC_DEFAULT_MAXIMUM_ITERATIONS;
    config.solver_convergence_tolerance = MPC_DEFAULT_CONVERGENCE_TOLERANCE;

    /* Environment variable overrides for runtime tuning.
     * Example: MPC_W_HEADING=2.0 MPC_W_STEER_RATE=0.01 ros2 launch ...
     * Avoids rebuild for quick parameter exploration. */
    {
        const char *env_val;
        if ((env_val = getenv("MPC_W_POS_X")) != NULL)
            config.weight_position_x = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_POS_Y")) != NULL)
            config.weight_position_y = DOUBLE_TO_FP(atof(env_val));
        if ((env_val = getenv("MPC_W_HEADING")) != NULL)
            config.weight_heading = DOUBLE_TO_FP(atof(env_val));
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
 * @param current_state        Current vehicle state x[0]
 * @param reference_trajectory Reference points for steps 1..N
 * @param horizon_steps        Number of prediction steps N
 * @param hessian_matrix       Output: Hessian matrix H (n_vars × n_vars)
 * @param linear_cost_vector   Output: Linear cost vector f (n_vars)
 */
static void build_qp_from_prediction(
    const VehicleState_t *current_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    int horizon_steps,
    fixed_point_t *hessian_matrix,
    fixed_point_t *linear_cost_vector)
{
    int n_vars = horizon_steps * CONTROL_DIMENSION;

    /* ---------------------------------------------------------------
     * Step 1: Linearize vehicle model at current operating point
     *
     * Uses the ACTUAL vehicle velocity for linearization, with a
     * minimum floor of 2.0 m/s. This is critical because:
     *   - At reference velocity (e.g. 20 m/s), B[2][0] is 4× larger
     *     than at actual velocity (5 m/s), making the MPC think small
     *     steering will produce large heading changes.
     *   - Result: MPC commands ~4× too little steering when slow.
     *   - Using actual velocity ensures the MPC commands the right
     *     steering angle for the real dynamics.
     *   - The 2.0 m/s floor prevents singularity near standstill.
     *   - Steering is linearized at δ=0 (NOT previous control) to
     *     prevent velocity-steering cross-coupling oscillation.
     *     Using previous δ=±0.42 causes B[2][1]=±0.067, creating
     *     Hessian coupling H[steer][vel]≈50 that, with velocity at 20,
     *     overwhelms tracking (gradient 1014 vs tracking 45).
     * --------------------------------------------------------------- */
    fixed_point_t A[7][7];
    fixed_point_t B[7][2];

    /* Use actual vehicle state for linearization, with minimum velocity floor.
     * Linearize steering at delta=0 to eliminate velocity-steering cross-coupling. */
    ControlInput_t linearization_control;
    linearization_control.steering_angle_radians = 0;  /* Always linearize at delta=0 */
    linearization_control.motor_torque_newton_meters = 0;  /* Linearize at zero torque */

    /* Create a copy of current state with velocity floor for linearization */
    VehicleState_t linearization_state = *current_state;
    fixed_point_t min_linearization_velocity = FP_CONST(2.0);
    if (linearization_state.longitudinal_velocity_meters_per_second < min_linearization_velocity)
        linearization_state.longitudinal_velocity_meters_per_second = min_linearization_velocity;

    /* Ensure wheel speed is consistent with velocity floor for linearization */
    VehicleParameters_t vp = vehicle_model_get_parameters();
    if (linearization_state.wheel_speed_radians_per_second == 0 &&
        linearization_state.longitudinal_velocity_meters_per_second > 0)
    {
        linearization_state.wheel_speed_radians_per_second =
            fp_div(linearization_state.longitudinal_velocity_meters_per_second,
                   vp.wheel_radius_meters);
    }

    vehicle_model_compute_linearization(
        &linearization_state,
        &linearization_control,
        current_configuration.time_step_seconds,
        A, B);

    /* ---------------------------------------------------------------
     * Stabilize wheel dynamics (row 6 of A matrix).
     *
     * With small wheel inertia, the wheel dynamics
     * time constant τ = Iw·vx/(Cx·Rw²) ≈ 0.022s at vx=1 m/s,
     * much smaller than dt=50ms. Forward Euler is unstable when
     * dt >> τ, producing |A[6][6]| >> 1 (e.g., -43 at low speed).
     *
     * Fix: rescale row 6 of A so A[6][6] is clamped to [-0.95, 0.95].
     * The scale factor preserves relative coupling magnitudes,
     * effectively approximating implicit Euler for the stiff wheel
     * subsystem. The control effect (B[6][1]) is preserved.
     * --------------------------------------------------------------- */
    {
        fixed_point_t abs_a66 = fp_abs(A[6][6]);
        const fixed_point_t stability_limit = FP_CONST(0.95);

        if (abs_a66 > stability_limit)
        {
            /* Scale factor = (target - 1) / (A[6][6] - 1)
             * Maps the continuous eigenvalue contribution to the clamped range
             * while preserving relative off-diagonal magnitudes. */
            fixed_point_t target = (A[6][6] < 0) ? fp_neg(stability_limit) : stability_limit;
            fixed_point_t num = fp_sub(target, FP_ONE);
            fixed_point_t den = fp_sub(A[6][6], FP_ONE);

            /* Avoid division by zero (shouldn't happen as |a66| > 0.95
             * implies a66 != 1, so den != 0) */
            if (den != 0)
            {
                fixed_point_t scale = fp_div(num, den);

                /* Scale off-diagonal entries in row 6 */
                for (int j = 0; j < STATE_DIMENSION; j++)
                {
                    if (j != 6)
                    {
                        A[6][j] = fp_mul(A[6][j], scale);
                    }
                }
            }

            A[6][6] = target;
        }
    }

    /* ---------------------------------------------------------------
     * Step 2: Compute Phi[m] = A^m * B for m = 0 .. N-1
     *
     * Phi[0] = B
     * Phi[m] = A * Phi[m-1]
     *
     * Phi[m] tells us: if I apply a control impulse u at step j,
     * how much does the state change m steps later?
     * --------------------------------------------------------------- */
    static fixed_point_t Phi[MAXIMUM_HORIZON_STEPS][STATE_DIMENSION][CONTROL_DIMENSION];

    /* Phi[0] = B */
    for (int r = 0; r < STATE_DIMENSION; r++)
    {
        for (int c = 0; c < CONTROL_DIMENSION; c++)
        {
            Phi[0][r][c] = B[r][c];
        }
    }

    /* Phi[m] = A * Phi[m-1] */
    for (int m = 1; m < horizon_steps; m++)
    {
        for (int r = 0; r < STATE_DIMENSION; r++)
        {
            for (int c = 0; c < CONTROL_DIMENSION; c++)
            {
                fixed_point_t sum = 0;
                for (int k = 0; k < STATE_DIMENSION; k++)
                {
                    sum = fp_add(sum,
                        fp_mul(A[r][k], Phi[m - 1][k][c]));
                }
                Phi[m][r][c] = sum;
            }
        }
    }

    /* ---------------------------------------------------------------
     * Step 3: Compute free-response tracking error
     *
     * x_free[k] = A^{k+1} * x[0]  (state at step k+1 with zero control)
     * d[k] = x_free[k] - reference[k]  (tracking error)
     *
     * Heading error is wrapped to [-pi, pi] to prevent discontinuity.
     * --------------------------------------------------------------- */
    static fixed_point_t d[MAXIMUM_HORIZON_STEPS][STATE_DIMENSION];

    fixed_point_t x0[STATE_DIMENSION] = {
        current_state->position_x_meters,
        current_state->position_y_meters,
        current_state->heading_angle_radians,
        current_state->longitudinal_velocity_meters_per_second,
        current_state->lateral_velocity_meters_per_second,
        current_state->yaw_rate_radians_per_second,
        current_state->wheel_speed_radians_per_second
    };

    /* Propagate free response incrementally: x_next = A * x_prev */
    fixed_point_t x_prev[STATE_DIMENSION];
    for (int s = 0; s < STATE_DIMENSION; s++)
    {
        x_prev[s] = x0[s];
    }

    for (int k = 0; k < horizon_steps; k++)
    {
        fixed_point_t x_free[STATE_DIMENSION];

        /* x_free = A * x_prev */
        for (int r = 0; r < STATE_DIMENSION; r++)
        {
            fixed_point_t sum = 0;
            for (int j = 0; j < STATE_DIMENSION; j++)
            {
                sum = fp_add(sum,
                    fp_mul(A[r][j], x_prev[j]));
            }
            x_free[r] = sum;
        }

        /* Tracking error: d = x_free - reference */
        d[k][0] = fp_sub(x_free[0],
            reference_trajectory[k].reference_position_x_meters);
        d[k][1] = fp_sub(x_free[1],
            reference_trajectory[k].reference_position_y_meters);
        d[k][2] = normalize_angle(fp_sub(x_free[2],
            reference_trajectory[k].reference_heading_radians));
        d[k][3] = fp_sub(x_free[3],
            reference_trajectory[k].reference_velocity_meters_per_second);
        d[k][4] = fp_sub(x_free[4],
            reference_trajectory[k].reference_lateral_velocity_meters_per_second);
        d[k][5] = fp_sub(x_free[5],
            reference_trajectory[k].reference_yaw_rate_radians_per_second);
        d[k][6] = fp_sub(x_free[6],
            reference_trajectory[k].reference_wheel_speed_radians_per_second);

        /* Advance for next iteration, normalizing heading to [-pi, pi] */
        for (int s = 0; s < STATE_DIMENSION; s++)
        {
            x_prev[s] = x_free[s];
        }
        /* Normalize predicted heading to prevent wrap-around issues */
        x_prev[2] = normalize_angle(x_prev[2]);
    }

    /* ---------------------------------------------------------------
     * Step 4: State cost weight vector (diagonal Q matrix)
     *
     * Only states with non-zero weights need to be processed in the
     * Hessian and gradient loops. Track which states are active to
     * skip unnecessary fixed-point multiplications (saves ~50% when
     * position weights are zero).
     * --------------------------------------------------------------- */
    fixed_point_t Q[STATE_DIMENSION] = {
        current_configuration.weight_position_x,
        current_configuration.weight_position_y,
        current_configuration.weight_heading,
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

    /* Tracking contribution: Su^T Q Su */
    for (int ci = 0; ci < horizon_steps; ci++)
    {
        for (int cj = ci; cj < horizon_steps; cj++)
        {
            /* Accumulate 2×2 block for (ci, cj) using int64_t to avoid overflow.
             * The inner loop sums up to num_active_states × (cj..horizon) products,
             * which can exceed int32_t range in Q16.16. */
            int64_t block64[CONTROL_DIMENSION][CONTROL_DIMENSION];
            for (int a = 0; a < CONTROL_DIMENSION; a++)
                for (int b = 0; b < CONTROL_DIMENSION; b++)
                    block64[a][b] = 0;

            /* Sum over all prediction steps where both controls matter */
            for (int k = cj; k < horizon_steps; k++)
            {
                int mi = k - ci;  /* Phi index for control ci */
                int mj = k - cj;  /* Phi index for control cj */

                /* block += Phi[mi]^T * diag(Q) * Phi[mj], active states only */
                for (int a = 0; a < CONTROL_DIMENSION; a++)
                {
                    for (int b = 0; b < CONTROL_DIMENSION; b++)
                    {
                        int64_t term64 = 0;
                        for (int si = 0; si < num_active_states; si++)
                        {
                            int s = active_states[si];
                            /* Phi[mi][s][a] * Q[s] * Phi[mj][s][b] — use int64_t */
                            int64_t phi_q = ((int64_t)Phi[mi][s][a] * Q[s]) >> FP_FRAC_BITS;
                            term64 += (phi_q * Phi[mj][s][b]) >> FP_FRAC_BITS;
                        }
                        block64[a][b] += term64;
                    }
                }
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

    /* Rate penalty contribution (all additions use saturating arithmetic) */
    fixed_point_t w_sr = current_configuration.weight_steering_rate;
    fixed_point_t w_vr = current_configuration.weight_torque_rate;
    fixed_point_t w_sr_cross = fp_mul(w_sr, current_configuration.cross_call_rate_scale);
    fixed_point_t w_vr_cross = fp_mul(w_vr, current_configuration.cross_call_rate_scale);

    for (int ci = 0; ci < horizon_steps; ci++)
    {
        int idx_s = (ci * 2) * n_vars + (ci * 2);
        int idx_v = (ci * 2 + 1) * n_vars + (ci * 2 + 1);

        if (ci == 0)
        {
            hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s],
                fp_mul(FP_TWO, w_sr_cross));
            hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v],
                fp_mul(FP_TWO, w_vr_cross));
            if (horizon_steps > 1)
            {
                hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s],
                    fp_mul(FP_TWO, w_sr));
                hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v],
                    fp_mul(FP_TWO, w_vr));
            }
        }
        else if (ci < horizon_steps - 1)
        {
            hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s],
                fp_mul((fixed_point_t)(4 * FP_ONE), w_sr));
            hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v],
                fp_mul((fixed_point_t)(4 * FP_ONE), w_vr));
        }
        else
        {
            hessian_matrix[idx_s] = fp_add_sat(hessian_matrix[idx_s],
                fp_mul(FP_TWO, w_sr));
            hessian_matrix[idx_v] = fp_add_sat(hessian_matrix[idx_v],
                fp_mul(FP_TWO, w_vr));
        }

        /* Off-diagonal rate: H[k-1,k] = H[k,k-1] = -2*w_rate */
        if (ci > 0)
        {
            int prev_s = ((ci - 1) * 2) * n_vars + (ci * 2);
            int prev_v = ((ci - 1) * 2 + 1) * n_vars + (ci * 2 + 1);
            int sym_s  = (ci * 2) * n_vars + ((ci - 1) * 2);
            int sym_v  = (ci * 2 + 1) * n_vars + ((ci - 1) * 2 + 1);

            fixed_point_t neg_2_sr = fp_neg(fp_mul(FP_TWO, w_sr));
            fixed_point_t neg_2_vr = fp_neg(fp_mul(FP_TWO, w_vr));

            hessian_matrix[prev_s] = fp_add_sat(hessian_matrix[prev_s], neg_2_sr);
            hessian_matrix[sym_s]  = fp_add_sat(hessian_matrix[sym_s],  neg_2_sr);
            hessian_matrix[prev_v] = fp_add_sat(hessian_matrix[prev_v], neg_2_vr);
            hessian_matrix[sym_v]  = fp_add_sat(hessian_matrix[sym_v],  neg_2_vr);
        }
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

    /* Tracking contribution: 2 * Su^T Q d — using int64_t accumulator */
    for (int ci = 0; ci < horizon_steps; ci++)
    {
        for (int a = 0; a < CONTROL_DIMENSION; a++)
        {
            int64_t sum64 = 0;

            for (int k = ci; k < horizon_steps; k++)
            {
                int m = k - ci;

                for (int si = 0; si < num_active_states; si++)
                {
                    int s = active_states[si];
                    /* Phi[m][s][a] * Q[s] * d[k][s] — int64_t intermediates */
                    int64_t phi_q = ((int64_t)Phi[m][s][a] * Q[s]) >> FP_FRAC_BITS;
                    sum64 += (phi_q * d[k][s]) >> FP_FRAC_BITS;
                }
            }

            /* Multiply by 2 and clamp */
            int64_t val64 = sum64 * 2;
            fixed_point_t val;
            if (val64 > INT32_MAX) val = INT32_MAX;
            else if (val64 < INT32_MIN) val = INT32_MIN;
            else val = (fixed_point_t)val64;

            linear_cost_vector[ci * CONTROL_DIMENSION + a] = val;
        }
    }

    /* Cross-call rate penalty linear cost: from (u[0] - u_prev)^2 expansion
     * f_rate[0] = -2 * w_sr_cross * u_prev.steering
     * f_rate[1] = -2 * w_vr_cross * u_prev.velocity
     *
     * Uses the SCALED weights (w_sr_cross, w_vr_cross) to match the
     * scaled diagonal in the Hessian for step 0.
     */
    linear_cost_vector[0] = fp_sub(
        linear_cost_vector[0],
        fp_mul(
            FP_TWO,
            fp_mul(w_sr_cross,
                previous_control_input.steering_angle_radians)));

    linear_cost_vector[1] = fp_sub(
        linear_cost_vector[1],
        fp_mul(
            FP_TWO,
            fp_mul(w_vr_cross,
                previous_control_input.motor_torque_newton_meters)));
    
    /* Debug: print first element of linear cost and Phi[0][2][0] (steering->heading) */
#ifdef MPC_DEBUG_PRINT
    printf("[MPC-DBG] d[0][2]=%.4f, Phi[0][2][0]=%.4f, f[0]=%.4f\n",
           FP_TO_DOUBLE(d[0][2]), FP_TO_DOUBLE(Phi[0][2][0]),
           FP_TO_DOUBLE(linear_cost_vector[0]));
    printf("[MPC-DBG] d[0][0]=%.4f, d[0][1]=%.4f, Phi[0][0][0]=%.4f, Phi[0][1][0]=%.4f\n",
           FP_TO_DOUBLE(d[0][0]), FP_TO_DOUBLE(d[0][1]),
           FP_TO_DOUBLE(Phi[0][0][0]), FP_TO_DOUBLE(Phi[0][1][0]));
#endif
}

/**
 * Build constraint matrices for actuator limits.
 *
 * Constraints:
 * - Steering angle: |delta| <= max_steering
 * - Velocity: min_velocity <= v <= max_velocity
 *
 * In matrix form: A * u <= b
 * Where each control has upper and lower bounds.
 *
 * @param horizon_steps Number of prediction steps
 * @param constraint_matrix Output constraint matrix
 * @param constraint_bounds Output constraint bounds vector
 * @param constraint_count Output number of constraints
 */
static void build_qp_constraints(
    int horizon_steps,
    fixed_point_t *constraint_matrix,
    fixed_point_t *constraint_bounds,
    uint16_t *constraint_count)
{
    VehicleParameters_t vehicle_params = vehicle_model_get_parameters();
    int total_controls = horizon_steps * CONTROL_DIMENSION;

    /* 4 constraints per time step: upper/lower for steering and velocity */
    int constraints_per_step = 4;
    *constraint_count = (uint16_t)(horizon_steps * constraints_per_step);
    int total_constraints = horizon_steps * constraints_per_step;

    /* Clear matrices */
    memset(constraint_matrix, 0,
           total_constraints * total_controls * sizeof(fixed_point_t));

    for (int step = 0; step < horizon_steps; step++)
    {
        int control_base = step * CONTROL_DIMENSION;
        int constraint_base = step * constraints_per_step;

        /* Constraint 0: steering <= max_steering */
        constraint_matrix[(constraint_base + 0) * total_controls + control_base] =
            FP_ONE;
        constraint_bounds[constraint_base + 0] =
            vehicle_params.maximum_steering_angle_radians;

        /* Constraint 1: -steering <= max_steering (i.e., steering >= -max) */
        constraint_matrix[(constraint_base + 1) * total_controls + control_base] =
            fp_neg(FP_ONE);
        constraint_bounds[constraint_base + 1] =
            vehicle_params.maximum_steering_angle_radians;

        /* Constraint 2: torque <= max_torque */
        constraint_matrix[(constraint_base + 2) * total_controls + (control_base + 1)] =
            FP_ONE;
        constraint_bounds[constraint_base + 2] =
            vehicle_params.maximum_motor_torque_newton_meters;

        /* Constraint 3: -torque <= -min_torque (i.e., torque >= min_torque) */
        constraint_matrix[(constraint_base + 3) * total_controls + (control_base + 1)] =
            fp_neg(FP_ONE);
        constraint_bounds[constraint_base + 3] =
            fp_neg(vehicle_params.minimum_motor_torque_newton_meters);
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

    mpc_initialized_flag = 1;
}

MpcSolverStatus_t mpc_compute_optimal_control(
    const VehicleState_t *current_vehicle_state,
    const TrajectoryReferencePoint_t *reference_trajectory,
    MpcSolverResult_t *result)
{
    /* Validate inputs */
    if (current_vehicle_state == NULL ||
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

    /* Build QP problem */
    QuadraticProgramProblem_t qp_problem;
    QuadraticProgramConfig_t qp_config;
    QuadraticProgramSolution_t qp_solution;

    qp_solver_initialize_problem(&qp_problem);
    qp_solver_initialize_config(&qp_config);

    /* Set problem dimensions */
    qp_problem.variable_count = total_controls;

    /* Build QP Hessian and linear cost from predicted dynamics */
    build_qp_from_prediction(
        current_vehicle_state,
        reference_trajectory,
        horizon,
        qp_problem.hessian_matrix,
        qp_problem.linear_cost_vector);

    /* Auto-scale Hessian and gradient to fit Q16.16 range.
     *
     * The condensed MPC Hessian (Phi^T Q Phi) can exceed INT32_MAX
     * due to large B matrix entries (especially torque→wheel_speed: B[6][1]≈2.12).
     * Since scaling H and f by the same factor doesn't change the QP solution
     * (argmin is invariant), we find the maximum |entry| and right-shift
     * both H and f to bring everything into representable range.
     */
    {
        /* Find maximum absolute Hessian entry */
        int64_t max_abs = 0;
        for (int i = 0; i < total_controls * total_controls; i++)
        {
            int64_t v = (int64_t)qp_problem.hessian_matrix[i];
            if (v < 0) v = -v;
            if (v > max_abs) max_abs = v;
        }
        for (int i = 0; i < total_controls; i++)
        {
            int64_t v = (int64_t)qp_problem.linear_cost_vector[i];
            if (v < 0) v = -v;
            if (v > max_abs) max_abs = v;
        }

        /* Compute shift: bring max_abs below INT32_MAX/4 for safety margin */
        int shift_bits = 0;
        int64_t target = (int64_t)INT32_MAX / 4;  /* ~536 million */
        while (max_abs > target && shift_bits < 20)
        {
            max_abs >>= 1;
            shift_bits++;
        }

        if (shift_bits > 0)
        {
            for (int i = 0; i < total_controls * total_controls; i++)
            {
                qp_problem.hessian_matrix[i] >>= shift_bits;
            }
            for (int i = 0; i < total_controls; i++)
            {
                qp_problem.linear_cost_vector[i] >>= shift_bits;
            }
        }
    }

    /* Build constraints */
    build_qp_constraints(
        horizon,
        qp_problem.constraint_matrix,
        qp_problem.constraint_bounds,
        &qp_problem.constraint_count);

    /* Configure solver */
    qp_config.maximum_iterations = current_configuration.maximum_solver_iterations;
    qp_config.convergence_tolerance = current_configuration.solver_convergence_tolerance;
    qp_config.enable_verbose_output = 0;

    /* Warm-start: initialize QP solution from shifted previous solution.
     * Shift control sequence by one step: u[0]=u_prev[1], u[1]=u_prev[2], ...
     * Last step repeats the last known value. */
    if (warm_start_available && total_controls >= CONTROL_DIMENSION)
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
        qp_problem.use_warm_start = 1;
    }

    /* Solve QP */
    QuadraticProgramStatus_t qp_status = qp_solver_solve(
        &qp_problem, &qp_config, &qp_solution);

    /* Save solution for next warm-start */
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
}
