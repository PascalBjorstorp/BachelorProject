/**
 * @file mpc_riccati_hls.cpp
 * @brief MPC Riccati-ADMM Compute Function — HLS-Synthesizable
 * @details Builds the augmented 8-state QP from the current Frenet state and
 *          fixed-horizon references, then solves via Riccati-ADMM and maps
 *          solver outputs to saturated steering/acceleration commands.
 * @dependencies fp_math_hls.h, riccati_solver_hls.h, mpc_fpga_types.h
 */

#include "../include/fp_math_hls.h"
#include "../include/riccati_solver_hls.h"
#include "../include/mpc_fpga_types.h"
#ifdef MPC_RUNTIME_TUNE
#include "../include/mpc_runtime_tune.h"
#endif
#if defined(MPC_HLS_BUILD) && defined(MPC_USE_AP_FIXED)
#include "../include/fp_types_hls.hpp"
#endif

#include <stdio.h>
#include <stdlib.h>

/* Forward declaration of vehicle model (defined in vehicle_model_hls.c) */
extern void compute_frenet_AB_hls(
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa, fp_QP_t dt,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU]);

extern void saturate_control_hls(
    fp_QP_t steer_in, fp_QP_t accel_in,
    fp_QP_t *steer_out, fp_QP_t *accel_out);

/**
 * MPC compute: build QP and solve.
 *
 * @param state_ey      Lateral error [m]
 * @param state_epsi    Heading error [rad]
 * @param state_vx      Longitudinal velocity [m/s]
 * @param state_vy      Lateral velocity [m/s]
 * @param state_omega   Yaw rate [rad/s]
 * @param ref           Reference trajectory (MPC_HORIZON points)
 * @param persist       Persistent MPC state (modified in-place)
 * @param admm_state    ADMM warm-start state (modified in-place)
 * @param out_steering  Output: steering angle command [rad]
 * @param out_accel     Output: acceleration command [m/s^2]
 * @param out_status    Output: 0=optimal, 1=max_iter, 2=error
 * @param out_iters     Output: ADMM iterations used
 * @return None.
 */
void mpc_compute_hls(
    fp_QP_t state_ey,
    fp_QP_t state_epsi,
    fp_QP_t state_vx,
    fp_QP_t state_vy,
    fp_QP_t state_omega,
    const MpcRefPoint_t ref[MPC_HORIZON],
    MpcPersistState_t *persist,
    AdmmState_t *admm_state,
    fp_QP_t *out_steering,
    fp_QP_t *out_accel,
    int *out_status,
    int *out_iters)
{
    const int N = MPC_HORIZON;
    fp_QP_t dt = MPC_DT;
    int k, i, j;

#ifdef MPC_HLS_BUILD
    const int trace_enable = 0;
    const int call_id = 0;
#else
    static int trace_enable = -1;
    static int trace_call = 0;
    if (trace_enable < 0) {
        const char *env = getenv("MPC_SOLVER_TRACE");
        trace_enable = (env && atoi(env) != 0) ? 1 : 0;
    }
    const int call_id = trace_call++;
#endif

    /* ---------------------------------------------------------------
     * Step 1: Keep first-point curvature for warm-start validation.
     * Full model linearization is performed per-step in the loop below.
     * --------------------------------------------------------------- */
    fp_QP_t kappa0 = ref[0].kappa;

    /* ---------------------------------------------------------------
     * Step 2: Build augmented per-step data (8-state formulation)
     * --------------------------------------------------------------- */

    StepData_t step_data[MPC_HORIZON];
#pragma HLS BIND_STORAGE variable=step_data type=ram_2p impl=bram
    /* Zero only sparse fields that are not explicitly overwritten below. */
    const fp_QP_t half_steer = VP_MAX_STEER >> 1;
    const fp_QP_t qp_half_steer = half_steer;
    const fp_QP_t qp_min_lin_vel = MIN_LIN_VEL;

    for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=8
#pragma HLS DEPENDENCE variable=step_data inter false
        StepData_t *sd = &step_data[k];

        /* Zero sparse blocks not explicitly written by the assignments below. */
        /* A rows 5,6,7 = 0 (delta integrator + prev-controls have no
         * cross-coupling to Frenet states in cols 0..4) */
        for (j = 0; j < MPC_NX_AUG; j++) {
#pragma HLS UNROLL
            sd->A[IDX_DELTA_ACT][j] = 0;
            sd->A[IDX_DELTA_RATE_PREV][j] = 0;
            sd->A[IDX_ACCEL_PREV][j] = 0;
        }
        /* A cols 6,7 = 0 for rows 0..5 */
        for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
            sd->A[i][IDX_DELTA_RATE_PREV] = 0;
            sd->A[i][IDX_ACCEL_PREV] = 0;
        }
        /* B rows 0-4 col 0 = 0 (delta_rate doesn't directly affect Frenet) */
        for (i = 0; i < 5; i++) {
#pragma HLS UNROLL
            sd->B[i][0] = 0;
        }
        /* B rows 6,7 for col not set */
        sd->B[IDX_DELTA_RATE_PREV][1] = 0;
        sd->B[IDX_ACCEL_PREV][0] = 0;
        /* N_cross: zero all, then set the two non-zero entries below */
        for (i = 0; i < MPC_NX_AUG; i++) {
#pragma HLS UNROLL
            sd->N_cross[i][0] = 0;
            sd->N_cross[i][1] = 0;
        }

        /* Per-step Frenet linearization */
        fp_QP_t A_step[MPC_NX_FRENET][MPC_NX_FRENET];
        fp_QP_t B_step[MPC_NX_FRENET][MPC_NU];

        fp_QP_t kappa_k = ref[k].kappa;
        fp_QP_t dff_k = fp_atan(fp_mul(VP_WHEELBASE, kappa_k));
        dff_k = fp_clamp(dff_k, -qp_half_steer, qp_half_steer);

        fp_QP_t lin_vx_k;
        lin_vx_k = (ref[k].velocity > qp_min_lin_vel) ? ref[k].velocity
                                                       : qp_min_lin_vel;

        compute_frenet_AB_hls(
            lin_vx_k, state_vy, state_omega,
            dff_k, 0,
            kappa_k, dt,
            A_step, B_step);

        /* Stabilize fast dynamics (omega row = 4) per stage */
        {
            fp_QP_t a44 = A_step[4][4];
            fp_QP_t abs_a44 = fp_abs(a44);
            if (abs_a44 > STABILITY_LIMIT_VAL) {
                fp_QP_t target = (a44 < 0) ? (fp_QP_t)(-STABILITY_LIMIT_VAL)
                                                : STABILITY_LIMIT_VAL;
                fp_QP_t num = target - FP_ONE;
                fp_QP_t den = a44 - FP_ONE;
                if (den != 0) {
                    fp_QP_t scale = fp_mul(num, fp_recip(den));
                    for (j = 0; j < MPC_NX_FRENET; j++) {
#pragma HLS UNROLL
                        if (j != 4) A_step[4][j] = fp_mul(A_step[4][j], scale);
                    }
                    B_step[4][0] = fp_mul(B_step[4][0], scale);
                    B_step[4][1] = fp_mul(B_step[4][1], scale);
                }
                A_step[4][4] = target;
            }
        }

        /* === Augmented A (8x8) === */

        /* Top-left 5x5: per-step Frenet dynamics */
        for (i = 0; i < 5; i++) {
#pragma HLS UNROLL
            for (j = 0; j < 5; j++) {
#pragma HLS UNROLL
                sd->A[i][j] = A_step[i][j];
            }
        }

        /* Column 5: steering effect via delta_actual */
        for (i = 0; i < 5; i++) {
#pragma HLS UNROLL
            sd->A[i][IDX_DELTA_ACT] = B_step[i][0];
        }

        /* A[5][5] = 1 (delta integrator) */
        sd->A[IDX_DELTA_ACT][IDX_DELTA_ACT] = FP_ONE;
        /* Rows/cols 6,7 of A: all zero (from sparse zeroing above) */

        /* === Augmented B (8x2) === */
        /* Rows 0-4, col 0: ZERO (delta_rate does not directly affect Frenet) */
        /* Rows 0-4, col 1: acceleration effect */
        for (i = 0; i < 5; i++) {
#pragma HLS UNROLL
            sd->B[i][1] = B_step[i][1];
        }
        /* B[5][0] = dt (delta integrator) */
        sd->B[IDX_DELTA_ACT][0] = dt;
        sd->B[IDX_DELTA_ACT][1] = 0;  /* delta not affected by accel */
        /* B[6][0] = 1 (delta_rate_prev = delta_rate) */
        sd->B[IDX_DELTA_RATE_PREV][0] = FP_ONE;
        /* B[7][1] = 1 (accel_prev = accel) */
        sd->B[IDX_ACCEL_PREV][1] = FP_ONE;

        /* === Q_diag (8 elements) — precomputed constants === */
        sd->Q_diag[0] = MPC_Q2_LAT_ERROR;
        sd->Q_diag[1] = MPC_Q2_HEADING;
        sd->Q_diag[2] = MPC_Q2_VELOCITY;
        sd->Q_diag[3] = MPC_Q2_LAT_VEL;
        sd->Q_diag[4] = MPC_Q2_YAW_RATE;
        sd->Q_diag[IDX_DELTA_ACT] = MPC_Q2_DELTA_ACT;
        sd->Q_diag[IDX_DELTA_RATE_PREV] = MPC_Q2_STEER_JERK;
        sd->Q_diag[IDX_ACCEL_PREV] = MPC_Q2_ACCEL_RATE;

        /* Cross-call scaling for step 0 */
        if (k == 0) {
            sd->Q_diag[IDX_DELTA_RATE_PREV] = MPC_Q2_JERK_CS;
            sd->Q_diag[IDX_ACCEL_PREV] = MPC_Q2_ARATE_CS;
        }

        /* === q (8 elements): linear cost from tracking references === */
        /* References for e_y and e_psi are 0 (path following) */
        sd->q[0] = 0;
        sd->q[1] = 0;
        sd->q[2] = -fp_mul(MPC_Q2_VELOCITY, ref[k].velocity);
        sd->q[3] = 0;  /* vy_ref = 0 */
        sd->q[4] = -fp_mul(MPC_Q2_YAW_RATE, ref[k].yaw_rate);
        /* delta_actual reference: feedforward steering */
        sd->q[IDX_DELTA_ACT] = -fp_mul(MPC_Q2_DELTA_ACT, dff_k);
        sd->q[IDX_DELTA_RATE_PREV] = 0;
        sd->q[IDX_ACCEL_PREV] = 0;

        /* === R_diag (2 elements) — precomputed constants === */
        sd->R_diag[0] = MPC_R2_STEER;
        sd->R_diag[1] = MPC_R2_ACCEL;
        if (k == 0) {
            sd->R_diag[0] = MPC_R2_STEER_CS;
            sd->R_diag[1] = MPC_R2_ACCEL_CS;
        }
        sd->r[0] = 0;
        sd->r[1] = 0;

        /* === Cross-cost N (8x2) — precomputed constants === */
        sd->N_cross[IDX_DELTA_RATE_PREV][0] = MPC_N2_STEER_JERK;
        sd->N_cross[IDX_ACCEL_PREV][1] = MPC_N2_ACCEL_RATE;
        if (k == 0) {
            sd->N_cross[IDX_DELTA_RATE_PREV][0] = -MPC_Q2_JERK_CS;
            sd->N_cross[IDX_ACCEL_PREV][1] = -MPC_Q2_ARATE_CS;
        }

        /* === State bounds === */
        /* e_y: wall constraints active for all steps when bounds are valid. */
        sd->x_lb[0] = -(ref[k].right_bound - WALL_MARGIN);
        sd->x_ub[0] = ref[k].left_bound - WALL_MARGIN;

        /* States 1-4: unconstrained */
        for (i = 1; i < 5; i++) {
            sd->x_lb[i] = -BIG_BOUND;
            sd->x_ub[i] =  BIG_BOUND;
        }
        /* State 5 (delta_actual): steering angle limit */
        sd->x_lb[IDX_DELTA_ACT] = -VP_MAX_STEER;
        sd->x_ub[IDX_DELTA_ACT] = VP_MAX_STEER;
        /* States 6-7: unconstrained */
        sd->x_lb[IDX_DELTA_RATE_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_DELTA_RATE_PREV] =  BIG_BOUND;
        sd->x_lb[IDX_ACCEL_PREV] = -BIG_BOUND;
        sd->x_ub[IDX_ACCEL_PREV] =  BIG_BOUND;

        /* === Control bounds === */
        /* u[0] = steering rate limit */
        sd->u_lb[0] = -VP_MAX_STEER_RATE;
        sd->u_ub[0] = VP_MAX_STEER_RATE;

        /* u[1] = acceleration with speed-dependent power limit */
        fp_QP_t v_ref_k = ref[k].velocity;
        if (v_ref_k > V_SWITCH) {
            fp_QP_t scale = fp_mul(V_SWITCH, fp_recip(v_ref_k));
            sd->u_ub[1] = fp_mul(VP_MAX_ACCEL, scale);
            sd->u_lb[1] = fp_mul(VP_MIN_ACCEL, scale);
        } else {
            sd->u_ub[1] = VP_MAX_ACCEL;
            sd->u_lb[1] = VP_MIN_ACCEL;
        }
        
    }

    /* ---------------------------------------------------------------
     * Step 3: Terminal cost
     * --------------------------------------------------------------- */
    fp_QP_t terminal_q_diag[MPC_NX_AUG];
    fp_QP_t terminal_q_linear[MPC_NX_AUG];
    for (i = 0; i < MPC_NX_AUG; i++) {
#pragma HLS UNROLL
        terminal_q_diag[i] = 0;
        terminal_q_linear[i] = 0;
    }

    terminal_q_diag[0] = MPC_Q2_LAT_ERROR;
    terminal_q_diag[1] = MPC_Q2_HEADING;
    terminal_q_diag[2] = MPC_Q2_VELOCITY;
    terminal_q_diag[3] = MPC_Q2_LAT_VEL;
    terminal_q_diag[4] = MPC_Q2_YAW_RATE;
    terminal_q_diag[IDX_DELTA_ACT] = MPC_Q2_DELTA_ACT;

    if (N > 0) {
        terminal_q_linear[2] = -fp_mul(terminal_q_diag[2], ref[N-1].velocity);
        terminal_q_linear[4] = -fp_mul(terminal_q_diag[4], ref[N-1].yaw_rate);
        fp_QP_t kN = ref[N-1].kappa;
        fp_QP_t dff_N = fp_atan(fp_mul(VP_WHEELBASE, kN));  /* feedforward steering from curvature */
        terminal_q_linear[IDX_DELTA_ACT] = -fp_mul(terminal_q_diag[IDX_DELTA_ACT], dff_N);
    }

    /* ---------------------------------------------------------------
     * Step 4: Initial state (8 elements)
     * --------------------------------------------------------------- */
    fp_QP_t x0[MPC_NX_AUG];
    x0[0] = state_ey;
    x0[1] = state_epsi;
    x0[2] = state_vx;
    x0[3] = state_vy;
    x0[4] = state_omega;
    x0[IDX_DELTA_ACT]  = persist->actual_steering;
    x0[IDX_DELTA_RATE_PREV] = persist->prev_steer_rate;
    x0[IDX_ACCEL_PREV] = persist->prev_accel;

    if (trace_enable && call_id < 64) {
        const StepData_t *sd0 = &step_data[0];
        printf(
            "TRACE_QP0,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
            call_id,
            FP_TO_DOUBLE(state_ey), FP_TO_DOUBLE(state_epsi), FP_TO_DOUBLE(state_vx),
            FP_TO_DOUBLE(sd0->A[2][2]), FP_TO_DOUBLE(sd0->A[3][3]), FP_TO_DOUBLE(sd0->A[4][4]),
            FP_TO_DOUBLE(sd0->B[2][0]), FP_TO_DOUBLE(sd0->B[3][0]), FP_TO_DOUBLE(sd0->B[4][0]),
            FP_TO_DOUBLE(sd0->q[2]), FP_TO_DOUBLE(sd0->q[IDX_DELTA_ACT]),
            FP_TO_DOUBLE(sd0->R_diag[0]), FP_TO_DOUBLE(sd0->R_diag[1]),
            FP_TO_DOUBLE(sd0->x_lb[0]), FP_TO_DOUBLE(sd0->x_ub[0]),
            FP_TO_DOUBLE(x0[IDX_DELTA_ACT]), FP_TO_DOUBLE(x0[IDX_DELTA_RATE_PREV]));
    }

    /* ---------------------------------------------------------------
     * Step 5: Warm-start management
     * --------------------------------------------------------------- */
    {
        /* Invalidate warm-start only on cold start or large curvature jump.
         * Keeping warm-start after a max-iteration return is still useful. */
        fp_QP_t kappa_diff = fp_abs(kappa0 - persist->prev_curvature);
        if (!admm_state->initialized || kappa_diff > FP_QP_CONST(0.5)) {
            admm_state->initialized = 0;
        }
    }
    persist->prev_curvature = kappa0;

    /* ---------------------------------------------------------------
     * Step 6: Solve via Riccati-ADMM
     * --------------------------------------------------------------- */
    AdmmConfig_t solver_cfg;
    solver_cfg.rho            = ADMM_RHO_DEFAULT;
    solver_cfg.rho_u          = ADMM_RHO_U_DEFAULT;
    solver_cfg.tolerance      = ADMM_TOL_DEFAULT;
    solver_cfg.alpha          = ADMM_ALPHA_DEFAULT;
#ifdef MPC_RUNTIME_TUNE
    solver_cfg.max_iterations = mpc_rt_max_admm_iter;
#else
    solver_cfg.max_iterations = MPC_MAX_ADMM_ITER;
#endif
    solver_cfg.adaptive_rho   = 1;

    MpcSolution_t sol;
    /* Zero-initialize solution */
    sol.iterations = 0;
    sol.primal_residual = 0;
    sol.dual_residual = 0;
    sol.status = MPC_STATUS_MAX_ITER;
    for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II=1
        for (i = 0; i < MPC_NX_AUG; i++) {
            sol.x[k][i] = 0;
        }
    }
    for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=1
        for (i = 0; i < MPC_NU; i++) {
            sol.u[k][i] = 0;
        }
    }

    MpcStatus_t rstatus = riccati_admm_solve_hls(
        step_data, terminal_q_diag, terminal_q_linear, x0,
        &solver_cfg, admm_state, &sol);

    /* ---------------------------------------------------------------
     * Step 7: Extract control output
     *
     * u = [delta_rate, accel]. Convert delta_rate to steering angle:
     *   delta_cmd = delta_actual + dt * delta_rate
     * --------------------------------------------------------------- */
    fp_QP_t delta_rate = admm_state->z_u[0][0];
    fp_QP_t accel      = admm_state->z_u[0][1];

    fp_QP_t delta_cmd = persist->actual_steering + fp_mul(dt, delta_rate);

    /* Clamp to physical limits */
    delta_cmd = fp_clamp(delta_cmd, -VP_MAX_STEER, VP_MAX_STEER);

    /* Final saturation */
    fp_QP_t steer_sat, accel_sat;
    saturate_control_hls(delta_cmd, accel, &steer_sat, &accel_sat);

    /* ---------------------------------------------------------------
     * Step 8: Update persistent state and outputs
     * --------------------------------------------------------------- */
    persist->prev_delta_cmd  = delta_cmd;
    persist->prev_steer_rate = delta_rate;
    persist->prev_accel      = accel_sat;
    persist->prev_converged  = (rstatus == MPC_STATUS_OPTIMAL) ? 1 : 0;

    *out_steering = steer_sat;
    *out_accel    = accel_sat;
    *out_status   = (int)rstatus;
    *out_iters    = sol.iterations;
}
