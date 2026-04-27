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

extern void compute_frenet_AB_and_next_hls(
    fp_QP_t ey, fp_QP_t epsi,
    fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd,
    fp_QP_t kappa,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU],
    fp_QP_t next_state[MPC_NX_FRENET]);

extern void saturate_control_hls(
    fp_QP_t steer_in, fp_QP_t accel_in,
    fp_QP_t *steer_out, fp_QP_t *accel_out);

static fp_QP_t fp_util_clamp(fp_QP_t value, fp_QP_t lower, fp_QP_t upper)
{
    if (value < lower) return lower;
    if (value > upper) return upper;
    return value;
}

static void compute_wall_ey_bounds_hls(
    fp_QP_t left_wall_bound,
    fp_QP_t right_wall_bound,
    fp_QP_t wall_margin,
    fp_QP_t *out_x_lb,
    fp_QP_t *out_x_ub)
{
    fp_QP_t x_lb = wall_margin - right_wall_bound;
    fp_QP_t x_ub = left_wall_bound - wall_margin;

    if (x_lb > x_ub) {
        fp_QP_t mid = fp_mul(FP_QP_CONST(0.5), x_lb + x_ub);
        x_lb = mid;
        x_ub = mid;
    }

    *out_x_lb = x_lb;
    *out_x_ub = x_ub;
}

static fp_QP_t compute_wall_biased_ey_ref_hls(
    fp_QP_t base_ref,
    fp_QP_t x_lb,
    fp_QP_t x_ub)
{
    fp_QP_t ref = base_ref;
    const fp_QP_t desired_lb = x_lb + WALL_BIAS_CLEAR_M;
    const fp_QP_t desired_ub = x_ub - WALL_BIAS_CLEAR_M;
    if (desired_lb <= desired_ub) {
        ref = fp_util_clamp(ref, desired_lb, desired_ub);
    } else {
        ref = fp_mul(FP_QP_CONST(0.5), x_lb + x_ub);
    }

    return fp_util_clamp(ref, -WALL_BIAS_MAX_M, WALL_BIAS_MAX_M);
}

static void compute_stage_wall_constraints_hls(
    const MpcRefPoint_t ref[MPC_HORIZON],
    int center_idx,
    fp_QP_t *out_x_lb,
    fp_QP_t *out_x_ub,
    fp_QP_t *out_ey_ref)
{
#pragma HLS INLINE
    fp_QP_t left_bound = ref[center_idx].left_bound;
    fp_QP_t right_bound = ref[center_idx].right_bound;

    const int window = WALL_BOUND_WINDOW;
    if (window > 0) {
        const int j0 = center_idx - window;
        const int j1 = center_idx + window;
        for (int j = 0; j < MPC_HORIZON; ++j) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
            if (j >= j0 && j <= j1) {
                if (ref[j].left_bound < left_bound) left_bound = ref[j].left_bound;
                if (ref[j].right_bound < right_bound) right_bound = ref[j].right_bound;
            }
        }
    }

    fp_QP_t wall_x_lb, wall_x_ub;
    compute_wall_ey_bounds_hls(
        left_bound,
        right_bound,
        WALL_MARGIN,
        &wall_x_lb,
        &wall_x_ub);

    fp_QP_t wall_x_lb_con = wall_x_lb + WALL_BIAS_CLEAR_M;
    fp_QP_t wall_x_ub_con = wall_x_ub - WALL_BIAS_CLEAR_M;
    if (wall_x_lb_con > wall_x_ub_con) {
        const fp_QP_t mid = fp_mul(FP_QP_CONST(0.5), wall_x_lb + wall_x_ub);
        wall_x_lb_con = mid;
        wall_x_ub_con = mid;
    }

    *out_x_lb = wall_x_lb_con;
    *out_x_ub = wall_x_ub_con;
    *out_ey_ref = compute_wall_biased_ey_ref_hls(0, wall_x_lb_con, wall_x_ub_con);
}

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
extern "C" void mpc_compute_hls(
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
    int k, i, j;

    /* ---------------------------------------------------------------
     * Step 1: Keep first-point curvature for warm-start validation.
     * Full model linearization is performed per-step in the loop below.
     * --------------------------------------------------------------- */
    fp_QP_t kappa0 = ref[0].kappa;

    /* ---------------------------------------------------------------
     * Step 2: Build augmented per-step data (8-state formulation)
     * --------------------------------------------------------------- */

    StepData_t step_data[MPC_HORIZON];
    /* Zero only sparse fields that are not explicitly overwritten below. */
    const fp_QP_t qp_min_lin_vel = MIN_LIN_VEL;
    const fp_QP_t kappa_diff_for_lin = fp_abs(kappa0 - persist->prev_curvature);
    int ws_matches_current = 0;
    if (admm_state->initialized) {
        const fp_QP_t ws_ey_err = fp_abs(admm_state->z_x[0][0] - state_ey);
        const fp_QP_t ws_epsi_err = fp_abs(admm_state->z_x[0][1] - state_epsi);
        const fp_QP_t ws_vx_err = fp_abs(admm_state->z_x[0][2] - state_vx);
        const fp_QP_t ws_vy_err = fp_abs(admm_state->z_x[0][3] - state_vy);
        const fp_QP_t ws_omega_err = fp_abs(admm_state->z_x[0][4] - state_omega);
        const fp_QP_t ws_delta_err = fp_abs(admm_state->z_x[0][IDX_DELTA_ACT] - persist->actual_steering);
        const fp_QP_t ws_drate_err = fp_abs(admm_state->z_x[0][IDX_DELTA_RATE_PREV] - persist->prev_steer_rate);
        const fp_QP_t ws_accel_err = fp_abs(admm_state->z_x[0][IDX_ACCEL_PREV] - persist->prev_accel);
        ws_matches_current = (ws_ey_err < FP_QP_CONST(0.5) &&
                              ws_epsi_err < FP_QP_CONST(0.5) &&
                              ws_vx_err < FP_QP_CONST(1.0) &&
                              ws_vy_err < FP_QP_CONST(0.5) &&
                              ws_omega_err < FP_QP_CONST(0.6) &&
                              ws_delta_err < FP_QP_CONST(0.3) &&
                              ws_drate_err < FP_QP_CONST(1.0) &&
                              ws_accel_err < FP_QP_CONST(2.0))
                                 ? 1
                                 : 0;
    }

    /* Rolling linearization state. The setup loop only needs x_k and x_{k+1},
     * so keeping this in scalars avoids horizon RAMs and read muxes. */
    fp_QP_t lin_ey = state_ey;
    fp_QP_t lin_epsi = state_epsi;
    fp_QP_t lin_vx = (state_vx > qp_min_lin_vel) ? state_vx : qp_min_lin_vel;
    fp_QP_t lin_vy = state_vy;
    fp_QP_t lin_omega = state_omega;
    fp_QP_t lin_delta = fp_clamp(persist->actual_steering,
                                 -VP_MAX_STEER,
                                 VP_MAX_STEER);
    fp_QP_t rollout_steer_rate = persist->prev_steer_rate;
    fp_QP_t rollout_accel = persist->prev_accel;
    fp_QP_t wall_x_lb_stage[MPC_HORIZON];
    fp_QP_t wall_x_ub_stage[MPC_HORIZON];
    fp_QP_t wall_ey_ref_stage[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=wall_x_lb_stage complete dim=1
#pragma HLS ARRAY_PARTITION variable=wall_x_ub_stage complete dim=1
#pragma HLS ARRAY_PARTITION variable=wall_ey_ref_stage complete dim=1

    for (k = 0; k < N; ++k) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
        compute_stage_wall_constraints_hls(
            ref, k,
            &wall_x_lb_stage[k],
            &wall_x_ub_stage[k],
            &wall_ey_ref_stage[k]);
    }

    for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=256
#pragma HLS DEPENDENCE variable=step_data inter false
        StepData_t *sd = &step_data[k];

        fp_QP_t uk0 = rollout_steer_rate;
        fp_QP_t uk1 = rollout_accel;

        /* Per-step Frenet linearization + rollout */
        fp_QP_t A_step[MPC_NX_FRENET][MPC_NX_FRENET];
        fp_QP_t B_step[MPC_NX_FRENET][MPC_NU];
        fp_QP_t next_frenet[MPC_NX_FRENET];
#pragma HLS ARRAY_PARTITION variable=A_step complete dim=0
#pragma HLS ARRAY_PARTITION variable=B_step complete dim=0

        fp_QP_t kappa_k = ref[k].kappa;
        fp_QP_t lin_delta_k = lin_delta;
        fp_QP_t lin_vx_k = lin_vx;

        compute_frenet_AB_and_next_hls(
            lin_ey, lin_epsi,
            lin_vx_k, lin_vy, lin_omega,
            lin_delta_k, uk1,
            kappa_k,
            A_step, B_step,
            next_frenet);

        fp_QP_t next_ey = next_frenet[0];
        fp_QP_t next_epsi = next_frenet[1];
        fp_QP_t next_vx = next_frenet[2];
        fp_QP_t next_vy = next_frenet[3];
        fp_QP_t next_omega = next_frenet[4];
        fp_QP_t next_delta = fp_clamp(lin_delta + fp_mul(MPC_DT, uk0),
                                      -VP_MAX_STEER, VP_MAX_STEER);

        /* Only the 6x6 dense A block is consumed by the Riccati pass. */
        for (j = 0; j < 5; j++) {
#pragma HLS UNROLL
            sd->A[IDX_DELTA_ACT][j] = 0;
        }
        /* Feedforward steering from path curvature. */
        fp_QP_t dff_raw_k = fp_atan_tire_approx(fp_mul(VP_WHEELBASE, kappa_k));
        fp_QP_t dff_k = dff_raw_k;
        if (dff_raw_k < -VP_MAX_STEER) {
            dff_k = -VP_MAX_STEER;
        } else if (dff_raw_k > VP_MAX_STEER) {
            dff_k = VP_MAX_STEER;
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

        /* === Augmented B (8x2) === */
        /* Solver only consumes acceleration coupling for vx/vy/yaw-rate. */
        for (i = 2; i < 5; i++) {
#pragma HLS UNROLL
            sd->B[i][1] = B_step[i][1];
        }
        /* B[5][0] = dt (delta integrator) */
        sd->B[IDX_DELTA_ACT][0] = MPC_DT;

        /* === Affine dynamics bias d: x_{k+1} = A x_k + B u_k + d_k ===
         * Use explicit sparsity of augmented A/B to avoid dense 8x8/8x2 MACs. */
        fp_qp_raw_t xk_raw[MPC_NX_DENSE];
        fp_qp_raw_t xk1_raw[MPC_NX_DENSE];
        fp_qp_raw_t uk0_raw = fp_qp_raw_from_QP(uk0);
        fp_qp_raw_t uk1_raw = fp_qp_raw_from_QP(uk1);
#pragma HLS ARRAY_PARTITION variable=xk_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=xk1_raw complete dim=1
        xk_raw[0] = fp_qp_raw_from_QP(lin_ey);
        xk_raw[1] = fp_qp_raw_from_QP(lin_epsi);
        xk_raw[2] = fp_qp_raw_from_QP(lin_vx);
        xk_raw[3] = fp_qp_raw_from_QP(lin_vy);
        xk_raw[4] = fp_qp_raw_from_QP(lin_omega);
        xk_raw[IDX_DELTA_ACT] = fp_qp_raw_from_QP(lin_delta);
        xk1_raw[0] = fp_qp_raw_from_QP(next_ey);
        xk1_raw[1] = fp_qp_raw_from_QP(next_epsi);
        xk1_raw[2] = fp_qp_raw_from_QP(next_vx);
        xk1_raw[3] = fp_qp_raw_from_QP(next_vy);
        xk1_raw[4] = fp_qp_raw_from_QP(next_omega);
        xk1_raw[IDX_DELTA_ACT] = fp_qp_raw_from_QP(next_delta);

        fp_qp_raw_t A_col0_raw[5], A_col1_raw[5], A_col2_raw[5], A_col3_raw[5], A_col4_raw[5];
        fp_qp_raw_t B_col0_raw[5], B_col1_raw[5];
#pragma HLS ARRAY_PARTITION variable=A_col0_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=A_col1_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=A_col2_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=A_col3_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=A_col4_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=B_col0_raw complete dim=1
#pragma HLS ARRAY_PARTITION variable=B_col1_raw complete dim=1
        for (i = 0; i < 5; i++) {
            #pragma HLS UNROLL
            A_col0_raw[i] = fp_qp_raw_from_QP(A_step[i][0]);
            A_col1_raw[i] = fp_qp_raw_from_QP(A_step[i][1]);
            A_col2_raw[i] = fp_qp_raw_from_QP(A_step[i][2]);
            A_col3_raw[i] = fp_qp_raw_from_QP(A_step[i][3]);
            A_col4_raw[i] = fp_qp_raw_from_QP(A_step[i][4]);
            B_col0_raw[i] = fp_qp_raw_from_QP(B_step[i][0]);
            B_col1_raw[i] = fp_qp_raw_from_QP(B_step[i][1]);
        }

        fp_QP_t d_dense[MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable=d_dense complete dim=1
        for (i = 0; i < 5; i++) {
#pragma HLS PIPELINE II=1
            fp_raw_acc_t affine_term;

            if (i == 0) {
                fp_raw_acc_t a0 = (fp_mul_qp_raw(A_col0_raw[i], xk_raw[0])) >> FP_FRAC_BITS;
                fp_raw_acc_t a1 = (fp_mul_qp_raw(A_col1_raw[i], xk_raw[1])) >> FP_FRAC_BITS;
                fp_raw_acc_t a2 = (fp_mul_qp_raw(A_col2_raw[i], xk_raw[2])) >> FP_FRAC_BITS;
                fp_raw_acc_t a3 = (fp_mul_qp_raw(A_col3_raw[i], xk_raw[3])) >> FP_FRAC_BITS;
                fp_raw_acc_t a01 = a0 + a1;
                fp_raw_acc_t a23 = a2 + a3;
                affine_term = (fp_raw_acc_t)xk1_raw[i] - (a01 + a23);
            } else if (i == 1) {
                fp_raw_acc_t a0 = (fp_mul_qp_raw(A_col0_raw[i], xk_raw[0])) >> FP_FRAC_BITS;
                fp_raw_acc_t a1 = (fp_mul_qp_raw(A_col1_raw[i], xk_raw[1])) >> FP_FRAC_BITS;
                fp_raw_acc_t a2 = (fp_mul_qp_raw(A_col2_raw[i], xk_raw[2])) >> FP_FRAC_BITS;
                fp_raw_acc_t a4 = (fp_mul_qp_raw(A_col4_raw[i], xk_raw[4])) >> FP_FRAC_BITS;
                fp_raw_acc_t a01 = a0 + a1;
                fp_raw_acc_t a24 = a2 + a4;
                affine_term = (fp_raw_acc_t)xk1_raw[i] - (a01 + a24);
            } else {
                fp_raw_acc_t a2 = (fp_mul_qp_raw(A_col2_raw[i], xk_raw[2])) >> FP_FRAC_BITS;
                fp_raw_acc_t a3 = (fp_mul_qp_raw(A_col3_raw[i], xk_raw[3])) >> FP_FRAC_BITS;
                fp_raw_acc_t a4 = (fp_mul_qp_raw(A_col4_raw[i], xk_raw[4])) >> FP_FRAC_BITS;
                fp_raw_acc_t b0 = (fp_mul_qp_raw(B_col0_raw[i], xk_raw[IDX_DELTA_ACT])) >> FP_FRAC_BITS;
                fp_raw_acc_t b1 = (fp_mul_qp_raw(B_col1_raw[i], uk1_raw)) >> FP_FRAC_BITS;
                fp_raw_acc_t a23 = a2 + a3;
                fp_raw_acc_t a4b0 = a4 + b0;
                fp_raw_acc_t sum0 = a23 + a4b0;
                affine_term = (fp_raw_acc_t)xk1_raw[i] - (sum0 + b1);
            }

            affine_term = fp_clip_raw_to_qp(affine_term);
            d_dense[i] = fp_QP_from_qp_raw((fp_qp_raw_t)affine_term);
        }

        {
            fp_raw_acc_t d5 = (fp_raw_acc_t)xk1_raw[IDX_DELTA_ACT]
                            - (fp_raw_acc_t)xk_raw[IDX_DELTA_ACT]
                            - ((fp_mul_qp_raw(fp_qp_raw_from_QP(MPC_DT), uk0_raw)) >> FP_FRAC_BITS);
            d5 = fp_clip_raw_to_qp(d5);
            d_dense[IDX_DELTA_ACT] = fp_QP_from_qp_raw((fp_qp_raw_t)d5);
        }
        sd->d0 = d_dense[0];
        sd->d1 = d_dense[1];
        sd->d2 = d_dense[2];
        sd->d3 = d_dense[3];
        sd->d4 = d_dense[4];
        sd->d5 = d_dense[5];
        /* d6 and d7 are structurally zero because those states are the
         * previous-control latches: x6_next=u0 and x7_next=u1. */

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
        /* References for e_y and e_psi are zero-centered, with e_y biased
         * inside the active wall corridor for obstacle-course feasibility. */
        fp_QP_t wall_x_lb_k = wall_x_lb_stage[k];
        fp_QP_t wall_x_ub_k = wall_x_ub_stage[k];
        fp_QP_t ey_ref_k = wall_ey_ref_stage[k];
        sd->q[0] = -fp_mul(MPC_Q2_LAT_ERROR, ey_ref_k);
        sd->q[1] = 0;
        sd->q[2] = -fp_mul(MPC_Q2_VELOCITY, ref[k].velocity);
        sd->q[3] = 0;  /* vy_ref = 0 */
        sd->q[4] = -fp_mul(MPC_Q2_YAW_RATE, ref[k].yaw_rate);
        /* delta_actual reference: feedforward steering */
        fp_QP_t q_delta_act = fp_mul(MPC_Q2_DELTA_ACT, dff_k);
        sd->q[IDX_DELTA_ACT] = -q_delta_act;
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
        sd->N_delta_rate = MPC_N2_STEER_JERK;
        sd->N_accel = MPC_N2_ACCEL_RATE;
        if (k == 0) {
            sd->N_delta_rate = -MPC_Q2_JERK_CS;
            sd->N_accel = -MPC_Q2_ARATE_CS;
        }

        /* === State bounds === */
        sd->x_lb[0] = wall_x_lb_k;
        sd->x_ub[0] = wall_x_ub_k;

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

        /* u[1] = acceleration with speed-dependent power limit.
         * Use predicted operating speed so the car can still accelerate
         * when below target velocity. */
        fp_QP_t v_op_k = lin_vx_k;
        if (v_op_k > V_SWITCH) {
            fp_QP_t scale = fp_mul(V_SWITCH, fp_recip(v_op_k));
            sd->u_ub[1] = fp_mul(VP_MAX_ACCEL, scale);
            sd->u_lb[1] = fp_mul(VP_MIN_ACCEL, scale);
        } else {
            sd->u_ub[1] = VP_MAX_ACCEL;
            sd->u_lb[1] = VP_MIN_ACCEL;
        }

        lin_ey = next_ey;
        lin_epsi = next_epsi;
        lin_vx = next_vx;
        lin_vy = next_vy;
        lin_omega = next_omega;
        lin_delta = next_delta;
        rollout_steer_rate = 0;
        rollout_accel = 0;
    }

    /* ---------------------------------------------------------------
     * Step 3: Terminal cost
     * --------------------------------------------------------------- */
    fp_QP_t terminal_q_diag[MPC_NX_AUG];
    fp_QP_t terminal_q_linear[MPC_NX_AUG];
    fp_QP_t terminal_x_lb[MPC_NX_AUG];
    fp_QP_t terminal_x_ub[MPC_NX_AUG];
    for (i = 0; i < MPC_NX_AUG; i++) {
#pragma HLS UNROLL
        terminal_q_diag[i] = 0;
        terminal_q_linear[i] = 0;
        terminal_x_lb[i] = -BIG_BOUND;
        terminal_x_ub[i] = BIG_BOUND;
    }

    terminal_q_diag[0] = MPC_Q2_LAT_ERROR;
    terminal_q_diag[1] = MPC_Q2_HEADING;
    terminal_q_diag[2] = MPC_Q2_VELOCITY;
    terminal_q_diag[3] = MPC_Q2_LAT_VEL;
    terminal_q_diag[4] = MPC_Q2_YAW_RATE;
    terminal_q_diag[IDX_DELTA_ACT] = MPC_Q2_DELTA_ACT;

    if (N > 0) {
        fp_QP_t terminal_wall_x_lb = wall_x_lb_stage[N - 1];
        fp_QP_t terminal_wall_x_ub = wall_x_ub_stage[N - 1];
        fp_QP_t terminal_ey_ref = wall_ey_ref_stage[N - 1];

        terminal_q_linear[0] = -fp_mul(terminal_q_diag[0], terminal_ey_ref);
        terminal_q_linear[2] = -fp_mul(terminal_q_diag[2], ref[N-1].velocity);
        terminal_q_linear[4] = -fp_mul(terminal_q_diag[4], ref[N-1].yaw_rate);
        fp_QP_t kN = ref[N-1].kappa;
        fp_QP_t dff_N = fp_atan_tire_approx(fp_mul(VP_WHEELBASE, kN));  /* feedforward steering from curvature */
        dff_N = fp_clamp(dff_N, -VP_MAX_STEER, VP_MAX_STEER);
        terminal_q_linear[IDX_DELTA_ACT] = -fp_mul(terminal_q_diag[IDX_DELTA_ACT], dff_N);

        terminal_x_lb[0] = terminal_wall_x_lb;
        terminal_x_ub[0] = terminal_wall_x_ub;
        
        terminal_x_lb[IDX_DELTA_ACT] = -VP_MAX_STEER;
        terminal_x_ub[IDX_DELTA_ACT] = VP_MAX_STEER;
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

    /* ---------------------------------------------------------------
     * Step 5: Warm-start management
     * --------------------------------------------------------------- */
    {
        /* Invalidate warm-start only on cold start or large curvature jump.*/
        fp_QP_t kappa_diff = fp_abs(kappa0 - persist->prev_curvature);
        if (!admm_state->initialized || !persist->prev_converged || !ws_matches_current ||
            kappa_diff > FP_QP_CONST(0.5)) {
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
    solver_cfg.max_iterations = MPC_MAX_ADMM_ITER;
    solver_cfg.adaptive_rho   = MPC_HLS_ADAPTIVE_RHO;

    MpcSolution_t sol;
    /* Zero-initialize solution */
    sol.iterations = 0;
    sol.primal_residual = 0;
    sol.dual_residual = 0;
    sol.status = MPC_STATUS_MAX_ITER;

    MpcStatus_t rstatus = riccati_admm_solve_hls(
        step_data, terminal_q_diag, terminal_q_linear, terminal_x_lb, terminal_x_ub, x0,
        &solver_cfg, admm_state, &sol);

    /* ---------------------------------------------------------------
     * Step 7: Extract control output
     *
     * u = [delta_rate, accel]. Convert delta_rate to steering angle:
     *   delta_cmd = delta_actual + dt * delta_rate
     * --------------------------------------------------------------- */
    fp_QP_t delta_rate = admm_state->z_u[0][0];
    fp_QP_t accel      = admm_state->z_u[0][1];

    fp_QP_t delta_cmd = persist->actual_steering + fp_mul(MPC_DT, delta_rate);

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
