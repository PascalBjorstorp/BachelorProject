/**
 * @file mpc_riccati_hls.cpp
 * @brief MPC Riccati-ADMM Compute Function — HLS-Synthesizable
 * @details Builds the augmented 8-state QP from the current Frenet state and
 *          fixed-horizon references, then solves via Riccati-ADMM and maps
 *          solver outputs to saturated steering/acceleration commands.
 * @dependencies fp_math_hls.h, riccati_solver_hls.h, mpc_fpga_types.h
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"
#include "../include/riccati_solver_hls.h"
#ifdef MPC_RUNTIME_TUNE
#include "../include/mpc_runtime_tune.h"
#endif
#if defined(MPC_HLS_BUILD) && defined(MPC_USE_AP_FIXED)
#include "../include/fp_types_hls.hpp"
#endif

#include <stdio.h>
#include <stdlib.h>

extern void compute_frenet_AB_and_next_hls(
    fp_QP_t ey, fp_QP_t epsi, fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd, fp_QP_t kappa,
    fp_QP_t reference_velocity,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU], fp_QP_t next_state[MPC_NX_FRENET]);

extern void saturate_control_hls(fp_QP_t steer_in, fp_QP_t accel_in,
                                 fp_QP_t *steer_out, fp_QP_t *accel_out);

static fp_QP_t fp_util_clamp(fp_QP_t value, fp_QP_t lower, fp_QP_t upper) {
  if (value < lower)
    return lower;
  if (value > upper)
    return upper;
  return value;
}

static void compute_wall_ey_bounds_hls(fp_QP_t left_wall_bound,
                                       fp_QP_t right_wall_bound,
                                       fp_QP_t wall_margin, fp_QP_t *out_x_lb,
                                       fp_QP_t *out_x_ub) {
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

static fp_QP_t compute_wall_biased_ey_ref_hls(fp_QP_t base_ref, fp_QP_t x_lb,
                                              fp_QP_t x_ub, fp_QP_t clearance_m,
                                              fp_QP_t max_shift_m) {
  /* CPU-equivalent logic:
   * 1) choose target interval (optionally tightened by clearance)
   * 2) project base_ref into target interval
   * 3) clamp correction magnitude by max_shift
   * 4) clamp final reference to physical [x_lb, x_ub] */
  if (!(max_shift_m > FP_QP_CONST(0.0))) {
    max_shift_m = BIG_BOUND;
  }

  fp_QP_t target_lb = x_lb;
  fp_QP_t target_ub = x_ub;
  if (clearance_m > FP_QP_CONST(0.0)) {
    const fp_QP_t desired_lb = x_lb + clearance_m;
    const fp_QP_t desired_ub = x_ub - clearance_m;
    if (desired_lb <= desired_ub) {
      target_lb = desired_lb;
      target_ub = desired_ub;
    }
  }

  const fp_QP_t target_ref = fp_util_clamp(base_ref, target_lb, target_ub);
  fp_QP_t corr = target_ref - base_ref;
  corr = fp_util_clamp(corr, -max_shift_m, max_shift_m);
  const fp_QP_t ref = base_ref + corr;

  return fp_util_clamp(ref, x_lb, x_ub);
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
void mpc_compute_hls(fp_QP_t state_ey, fp_QP_t state_epsi, fp_QP_t state_vx,
                     fp_QP_t state_vy, fp_QP_t state_omega,
                     const MpcRefPoint_t ref[MPC_HORIZON],
                     MpcPersistState_t *persist, AdmmState_t *admm_state,
                     fp_QP_t *out_steering, fp_QP_t *out_accel, int *out_status,
                     int *out_iters) {
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable = admm_state->z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->y_u complete dim = 2
  int k, i, j;

  /* ---------------------------------------------------------------
   * Step 1: Keep first-point curvature for warm-start validation.
   * Full model linearization is performed per-step in the loop below.
   * --------------------------------------------------------------- */
  fp_QP_t kappa0 = ref[0].path_curvature;

  /* ---------------------------------------------------------------
   * Step 2: Build augmented per-step data (8-state formulation)
   * --------------------------------------------------------------- */

  StepData_t step_data[MPC_HORIZON];
/* Bind step_data to LUTRAM (distributed LUT-based RAM) instead of block BRAM.
 * The violation: step_data_x_lb_U BRAM is placed at RAMB36_X2Y24 — far from
 * the ADMM update logic at SLICE_X48. The BRAM clock net (4.851ns) arrives
 * 0.4ns later than the destination FF clock (4.984ns after pessimism removal),
 * consuming all timing margin → -0.140ns slack.
 * LUTRAM is distributed through the fabric near the logic that reads it,
 * eliminating remote BRAM placement and the associated clock skew. */
#pragma HLS BIND_STORAGE variable=step_data type=RAM_2P impl=LUTRAM latency=1

  /* Zero only sparse fields that are not explicitly overwritten below. */
  const fp_QP_t qp_min_lin_vel = MIN_LIN_VEL;

  /* Save terminal constraint values from last loop iteration */
  fp_QP_t terminal_wall_x_lb_con = -BIG_BOUND;
  fp_QP_t terminal_wall_x_ub_con = BIG_BOUND;
  fp_QP_t terminal_ey_ref = FP_QP_CONST(0.0);
  fp_QP_t terminal_dff_raw = FP_QP_CONST(0.0);

  /* Rolling linearization state. The setup loop only needs x_k and x_{k+1},
   * so keeping this in scalars avoids horizon RAMs and read muxes. */
  fp_QP_t lin_ey = state_ey;
  fp_QP_t lin_epsi = state_epsi;
  fp_QP_t lin_vx = (state_vx > qp_min_lin_vel) ? state_vx : qp_min_lin_vel;
  fp_QP_t lin_vy = state_vy;
  fp_QP_t lin_omega = state_omega;
  fp_QP_t lin_delta =
      fp_clamp(persist->actual_steering, -VP_MAX_STEER, VP_MAX_STEER);
  fp_QP_t rollout_steer_rate = persist->prev_steer_rate;

  /* Extract wall bounds into flat partitioned arrays BEFORE the window loop.
   * Accessing ref[j].left_wall_bound inside a PIPELINE II=1 loop forces
   * Vivado to compute j*sizeof(MpcRefPoint_t) as a 64×66-bit → 129-bit
   * address multiply at latency=2. This creates a 4-DSP PCOUT cascade
   * (5.423ns) — the confirmed post-route critical path at -0.433ns slack.
   * With flat arrays, HLS uses constant-stride integer addition instead of
   * a wide multiply, reducing the path to a single register + add (~0.4ns). */
  fp_QP_t ref_left_wall[MPC_HORIZON];
  fp_QP_t ref_right_wall[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable = ref_left_wall complete dim = 1
#pragma HLS ARRAY_PARTITION variable = ref_right_wall complete dim = 1
  /* No PIPELINE on this extraction loop either. The ref[] pointer access still
   * generates BRAM address MUXes (ref_7_6_address, 26 LUT each) that give
   * VITIS_LOOP_193_1 a 4.471ns CP — only 0.029ns margin after 0.5ns uncertainty.
   * Without PIPELINE, HLS generates a simple FSM reading one ref[k] per cycle
   * (20 cycles, identical throughput) with no pipeline controller routing risk. */
  for (k = 0; k < MPC_HORIZON; k++) {
    ref_left_wall[k]  = ref[k].left_wall_bound;
    ref_right_wall[k] = ref[k].right_wall_bound;
  }

  fp_QP_t wall_left_min[MPC_HORIZON];
  fp_QP_t wall_right_min[MPC_HORIZON];
  /* wall_left_min/right_min accessed sequentially by k in both this loop and
   * the main setup loop — no parallel access, so ARRAY_PARTITION complete
   * is unnecessary and adds 2x20x30=1200 parallel routing bits (congestion). */
  /* Optimized: Use tree-based min reduction to reduce logic depth from 12 to ~5
   * levels. Gather candidates in first unroll, then reduce in tree stages to
   * break combinatorial path. This fixes timing violations on wall boundary
   * calculation (Path16, 22, 25 from place & route report).
   * 
   * Key insight: original cascading if-statements create deeply nested logic.
   * Tree reduction creates a balanced structure that synthesizes to shallower
   * LUT trees. The comparison-select pairs are scheduled in parallel at each
   * level, reducing overall depth.
   * 
   * Vitis 2025.2 HLS optimization: Using explicit ternary operators allows
   * better tree inference during RTL generation. */
  for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS UNROLL factor=1
    /* Stage 1: Gather candidates (7 window positions)
     * Vivado infers direct mux selects over ref_left_wall partitioned registers
     * (no arithmetic). Using array bounds checking + clipping pattern. */
    fp_QP_t left_cand[2*WALL_BOUND_WINDOW + 1];
    fp_QP_t right_cand[2*WALL_BOUND_WINDOW + 1];
#pragma HLS ARRAY_PARTITION variable=left_cand complete dim=1
#pragma HLS ARRAY_PARTITION variable=right_cand complete dim=1

    for (int dj = -WALL_BOUND_WINDOW; dj <= WALL_BOUND_WINDOW; dj++) {
#pragma HLS UNROLL
      int j = k + dj;
      int idx = dj + WALL_BOUND_WINDOW;
      if (j >= 0 && j < MPC_HORIZON) {
        left_cand[idx]  = ref_left_wall[j];
        right_cand[idx] = ref_right_wall[j];
      } else {
        /* Sentinel: use BIG_BOUND for out-of-range so they lose all comparisons */
        left_cand[idx]  = BIG_BOUND;
        right_cand[idx] = BIG_BOUND;
      }
    }

    /* Stage 2a: First level tree reduction (7 → 4 candidates via 3 comparisons)
     * Reduces 7 values via 3 parallel min operations + 1 passthrough.
     * Each comparison generates one LUT6, all 4 operations happen in parallel. */
    fp_QP_t left_l2_0 = (left_cand[0] < left_cand[1]) ? left_cand[0] : left_cand[1];
    fp_QP_t left_l2_1 = (left_cand[2] < left_cand[3]) ? left_cand[2] : left_cand[3];
    fp_QP_t left_l2_2 = (left_cand[4] < left_cand[5]) ? left_cand[4] : left_cand[5];
    fp_QP_t left_l2_3 = left_cand[6];  /* Center element, no comparison needed */

    fp_QP_t right_l2_0 = (right_cand[0] < right_cand[1]) ? right_cand[0] : right_cand[1];
    fp_QP_t right_l2_1 = (right_cand[2] < right_cand[3]) ? right_cand[2] : right_cand[3];
    fp_QP_t right_l2_2 = (right_cand[4] < right_cand[5]) ? right_cand[4] : right_cand[5];
    fp_QP_t right_l2_3 = right_cand[6];

    /* Stage 2b: Second level tree reduction (4 → 2 candidates via 2 comparisons)
     * Reduces 4 values via 2 parallel min operations. Scheduled in next cycle
     * after Stage 2a comparisons route (no combinatorial dependency). */
    fp_QP_t left_l3_0  = (left_l2_0 < left_l2_1) ? left_l2_0 : left_l2_1;
    fp_QP_t left_l3_1  = (left_l2_2 < left_l2_3) ? left_l2_2 : left_l2_3;

    fp_QP_t right_l3_0 = (right_l2_0 < right_l2_1) ? right_l2_0 : right_l2_1;
    fp_QP_t right_l3_1 = (right_l2_2 < right_l2_3) ? right_l2_2 : right_l2_3;

    /* Stage 3: Final comparison (2 → 1 candidate)
     * Single comparison selects overall minimum. */
    fp_QP_t left_bound_k  = (left_l3_0 < left_l3_1) ? left_l3_0 : left_l3_1;
    fp_QP_t right_bound_k = (right_l3_0 < right_l3_1) ? right_l3_0 : right_l3_1;

    wall_left_min[k]  = left_bound_k;
    wall_right_min[k] = right_bound_k;
  }

  for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
    // True loop-carried dependency: lin_* state rolls forward each iteration.
    // Auto-pipelining (from syn.compile.pipeline_loops) creates II violations
    // here.
#pragma HLS PIPELINE off
    StepData_t *sd = &step_data[k];

    fp_QP_t uk0 = rollout_steer_rate;
    /* Match CPU operating-point choice: linearize around previous applied
     * acceleration across the full horizon. */
    fp_QP_t uk1 = persist->prev_accel;

    /* Per-step Frenet linearization + rollout */
    fp_QP_t A_step[MPC_NX_FRENET][MPC_NX_FRENET];
    fp_QP_t B_step[MPC_NX_FRENET][MPC_NU];
    fp_QP_t next_frenet[MPC_NX_FRENET];
#pragma HLS ARRAY_PARTITION variable = A_step complete dim = 0
#pragma HLS ARRAY_PARTITION variable = B_step complete dim = 0
#pragma HLS ARRAY_PARTITION variable = next_frenet complete dim = 1

    fp_QP_t kappa_k = ref[k].path_curvature;
    /* Feedforward steering from path curvature via LUT atan.
     * VP_WHEELBASE*kappa < 0.412 for unclamped regime → |x| always < 1
     * so fp_recip branch inside fp_atan_lut is never taken. */
    fp_QP_t dff_raw_k = fp_atan_lut(fp_mul(VP_WHEELBASE, kappa_k));
    fp_QP_t dff_k = dff_raw_k;
    if (dff_raw_k < -VP_MAX_STEER) {
      dff_k = -VP_MAX_STEER;
    } else if (dff_raw_k > VP_MAX_STEER) {
      dff_k = VP_MAX_STEER;
    }
    fp_QP_t lin_delta_k = dff_k;
    fp_QP_t lin_vx_k = lin_vx;
    fp_QP_t left_bound_k = wall_left_min[k];
    fp_QP_t right_bound_k = wall_right_min[k];

    fp_QP_t wall_x_lb, wall_x_ub;
    compute_wall_ey_bounds_hls(left_bound_k, right_bound_k, WALL_MARGIN,
                               &wall_x_lb, &wall_x_ub);

    fp_QP_t wall_x_lb_con = wall_x_lb;
    fp_QP_t wall_x_ub_con = wall_x_ub;
    if (WALL_BIAS_CLEAR_M > FP_QP_CONST(0.0)) {
      fp_QP_t desired_lb = wall_x_lb + WALL_BIAS_CLEAR_M;
      fp_QP_t desired_ub = wall_x_ub - WALL_BIAS_CLEAR_M;
      if (desired_lb <= desired_ub) {
        wall_x_lb_con = desired_lb;
        wall_x_ub_con = desired_ub;
      }
      /* else: clearance made corridor infeasible — keep wall_x_lb/ub
       * (pre-clearance) */
    }

    fp_QP_t ey_ref_k = compute_wall_biased_ey_ref_hls(
        ref[k].reference_lateral_error, wall_x_lb_con, wall_x_ub_con,
        FP_QP_CONST(0.0), /* clearance already applied to bounds above */
        WALL_BIAS_MAX_M);

    if (k == (MPC_HORIZON - 1)) {
      terminal_wall_x_lb_con = wall_x_lb_con;
      terminal_wall_x_ub_con = wall_x_ub_con;
      terminal_ey_ref = ey_ref_k;
      terminal_dff_raw = dff_raw_k;
    }

    compute_frenet_AB_and_next_hls(lin_ey, lin_epsi, lin_vx_k, lin_vy,
                                   lin_omega, lin_delta_k, uk1, kappa_k,
                                   ref[k].reference_velocity,
                                   A_step,
                                   B_step, next_frenet);

    /* Match CPU safeguard: clamp yaw-rate diagonal to stability limit. */
    {
      const int row = 4;
      fp_QP_t aii = A_step[row][row];
      fp_QP_t abs_aii = fp_abs(aii);
      if (abs_aii > STABILITY_LIMIT_VAL) {
        const fp_QP_t limit = STABILITY_LIMIT_VAL;
        const fp_QP_t neg_limit = fp_QP_t(0) - limit;
        A_step[row][row] = (aii < fp_QP_t(0)) ? neg_limit : limit;
      }
    }

    fp_QP_t next_ey = next_frenet[0];
    fp_QP_t next_epsi = next_frenet[1];
    fp_QP_t next_vx = next_frenet[2];
    fp_QP_t next_vy = next_frenet[3];
    fp_QP_t next_omega = next_frenet[4];
    fp_QP_t next_delta =
        fp_clamp(lin_delta + fp_mul(MPC_DT, uk0), -VP_MAX_STEER, VP_MAX_STEER);

    /* Only the 6x6 dense A block is consumed by the Riccati pass. */
    for (j = 0; j < 5; j++) {
#pragma HLS UNROLL
      sd->A[IDX_DELTA_ACT][j] = 0;
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
    sd->B_vx_accel = B_step[2][1];
    sd->B_vy_accel = B_step[3][1];
    sd->B_omega_accel = B_step[4][1];
    /* B[5][0] = dt (delta integrator) */
    sd->B_delta_rate = MPC_DT;

    /* === Affine dynamics bias d: x_{k+1} = A x_k + B u_k + d_k ===
     * Use explicit sparsity of augmented A/B to avoid dense 8x8/8x2 MACs. */
    fp_qp_raw_t xk_raw[MPC_NX_DENSE];
    fp_qp_raw_t xk1_raw[MPC_NX_DENSE];
    fp_qp_raw_t uk0_raw = fp_qp_raw_from_QP(uk0);
    fp_qp_raw_t uk1_raw = fp_qp_raw_from_QP(uk1);
    fp_qp_raw_t lin_delta_k_raw = fp_qp_raw_from_QP(lin_delta_k);
#pragma HLS ARRAY_PARTITION variable = xk_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = xk1_raw complete dim = 1
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

    fp_qp_raw_t A_col0_raw[5], A_col1_raw[5], A_col2_raw[5], A_col3_raw[5],
        A_col4_raw[5];
    fp_qp_raw_t B_col0_raw[5], B_col1_raw[5];
#pragma HLS ARRAY_PARTITION variable = A_col0_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = A_col1_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = A_col2_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = A_col3_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = A_col4_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = B_col0_raw complete dim = 1
#pragma HLS ARRAY_PARTITION variable = B_col1_raw complete dim = 1
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
#pragma HLS ARRAY_PARTITION variable = d_dense complete dim = 1
    for (i = 0; i < 5; i++) {
      /* CPU-parity affine bias: use full Frenet row contribution
       * d_i = x_{k+1,i} - A_i*x_k - B_i*u_k
       * for rows 0..4 (no sparsity shortcuts on rows 0/1). */
      fp_raw_acc_t a0 =
          (fp_mul_qp_raw(A_col0_raw[i], xk_raw[0])) >> FP_FRAC_BITS;
      fp_raw_acc_t a1 =
          (fp_mul_qp_raw(A_col1_raw[i], xk_raw[1])) >> FP_FRAC_BITS;
      fp_raw_acc_t a2 =
          (fp_mul_qp_raw(A_col2_raw[i], xk_raw[2])) >> FP_FRAC_BITS;
      fp_raw_acc_t a3 =
          (fp_mul_qp_raw(A_col3_raw[i], xk_raw[3])) >> FP_FRAC_BITS;
      fp_raw_acc_t a4 =
          (fp_mul_qp_raw(A_col4_raw[i], xk_raw[4])) >> FP_FRAC_BITS;
      fp_raw_acc_t b0 =
          (fp_mul_qp_raw(B_col0_raw[i], lin_delta_k_raw)) >> FP_FRAC_BITS;
      fp_raw_acc_t b1 =
          (fp_mul_qp_raw(B_col1_raw[i], uk1_raw)) >> FP_FRAC_BITS;

      fp_raw_acc_t affine_term = (fp_raw_acc_t)xk1_raw[i]
                               - (a0 + a1 + a2 + a3 + a4 + b0 + b1);
      affine_term = fp_clip_raw_to_qp(affine_term);
      d_dense[i] = fp_QP_from_qp_raw((fp_qp_raw_t)affine_term);
    }

    {
      fp_raw_acc_t d5 =
          (fp_raw_acc_t)xk1_raw[IDX_DELTA_ACT] -
          (fp_raw_acc_t)xk_raw[IDX_DELTA_ACT] -
          ((fp_mul_qp_raw(fp_qp_raw_from_QP(MPC_DT), uk0_raw)) >> FP_FRAC_BITS);
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
    if (k == 0) {
      sd->Q_diag[IDX_DELTA_RATE_PREV] = MPC_Q2_JERK_CS;
      sd->Q_diag[IDX_ACCEL_PREV] = MPC_Q2_ARATE_CS;
    }

    /* === q (8 elements): linear cost from tracking references === */
    /* References for e_y and e_psi are zero-centered, with e_y biased
     * inside the active wall corridor for obstacle-course feasibility. */
    sd->q[0] = -fp_mul(MPC_Q2_LAT_ERROR, ey_ref_k);
    sd->q[1] = -fp_mul(MPC_Q2_HEADING, ref[k].reference_heading_error);
    sd->q[2] = -fp_mul(MPC_Q2_VELOCITY, ref[k].reference_velocity);
    sd->q[3] = -fp_mul(MPC_Q2_LAT_VEL, ref[k].reference_lateral_velocity);
    sd->q[4] = -fp_mul(MPC_Q2_YAW_RATE, ref[k].reference_yaw_rate);
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
    sd->x_lb[0] = wall_x_lb_con;
    sd->x_ub[0] = wall_x_ub_con;

    /* States 1-4: unconstrained */
    for (i = 1; i < 5; i++) {
      sd->x_lb[i] = -BIG_BOUND;
      sd->x_ub[i] = BIG_BOUND;
    }
    /* State 5 (delta_actual): steering angle limit */
    sd->x_lb[IDX_DELTA_ACT] = -VP_MAX_STEER;
    sd->x_ub[IDX_DELTA_ACT] = VP_MAX_STEER;
    /* States 6-7: unconstrained */
    sd->x_lb[IDX_DELTA_RATE_PREV] = -BIG_BOUND;
    sd->x_ub[IDX_DELTA_RATE_PREV] = BIG_BOUND;
    sd->x_lb[IDX_ACCEL_PREV] = -BIG_BOUND;
    sd->x_ub[IDX_ACCEL_PREV] = BIG_BOUND;

    /* === Control bounds === */
    /* u[0] = steering rate limit */
    sd->u_lb[0] = -VP_MAX_STEER_RATE;
    sd->u_ub[0] = VP_MAX_STEER_RATE;

    /* u[1] = acceleration with speed-dependent power limit.
     * Blend reference and model velocity for operating-point power limit.
     * This allows better throttle response near reference speed. */
    fp_QP_t v_ref_k = ref[k].reference_velocity;
    fp_QP_t v_model_k = lin_vx_k;
    if (v_model_k < MIN_LIN_VEL)
      v_model_k = MIN_LIN_VEL;

    fp_QP_t v_for_limit =
        fp_mul(FP_QP_CONST(0.7), v_model_k) + fp_mul(FP_QP_CONST(0.3), v_ref_k);

    if (v_for_limit < MIN_LIN_VEL)
      v_for_limit = MIN_LIN_VEL;

    if (v_for_limit > V_SWITCH) {
      fp_QP_t scale = fp_mul(V_SWITCH, fp_recip(v_for_limit));
      sd->u_ub[1] = fp_mul(VP_MAX_ACCEL, scale);
      sd->u_lb[1] = VP_MIN_ACCEL;
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

  /* Use terminal constraint values saved from last loop iteration */
  /* wall_x_lb_con, wall_x_ub_con, ey_ref_k were computed inline and saved when
   * k==MPC_HORIZON-1 */

  terminal_x_lb[0] = terminal_wall_x_lb_con;
  terminal_x_ub[0] = terminal_wall_x_ub_con;
  terminal_q_linear[0] = -fp_mul(terminal_q_diag[0], terminal_ey_ref);
  terminal_q_linear[1] =
      -fp_mul(terminal_q_diag[1], ref[MPC_HORIZON - 1].reference_heading_error);
  terminal_q_linear[2] =
      -fp_mul(terminal_q_diag[2], ref[MPC_HORIZON - 1].reference_velocity);
  terminal_q_linear[3] = -fp_mul(
      terminal_q_diag[3], ref[MPC_HORIZON - 1].reference_lateral_velocity);
  terminal_q_linear[4] =
      -fp_mul(terminal_q_diag[4], ref[MPC_HORIZON - 1].reference_yaw_rate);
  /* Reuse dff_raw saved from the last loop iteration (k=MPC_HORIZON-1).
   * Both use ref[MPC_HORIZON-1].path_curvature — no duplicate hardware. */
  fp_QP_t dff_N = fp_clamp(terminal_dff_raw, -VP_MAX_STEER, VP_MAX_STEER);
  terminal_q_linear[IDX_DELTA_ACT] =
      -fp_mul(terminal_q_diag[IDX_DELTA_ACT], dff_N);

  terminal_x_lb[IDX_DELTA_ACT] = -VP_MAX_STEER;
  terminal_x_ub[IDX_DELTA_ACT] = VP_MAX_STEER;

  /* ---------------------------------------------------------------
   * Step 4: Initial state (8 elements)
   * --------------------------------------------------------------- */
  fp_QP_t x0[MPC_NX_AUG];
  x0[0] = state_ey;
  x0[1] = state_epsi;
  x0[2] = state_vx;
  x0[3] = state_vy;
  x0[4] = state_omega;
  x0[IDX_DELTA_ACT] = persist->actual_steering;
  x0[IDX_DELTA_RATE_PREV] = persist->prev_steer_rate;
  x0[IDX_ACCEL_PREV] = persist->prev_accel;

  /* ---------------------------------------------------------------
   * Step 5: Warm-start management
   * --------------------------------------------------------------- */
  {
    const fp_QP_t kappa_diff = fp_abs(kappa0 - persist->prev_curvature);
    const int model_signature_changed =
        (persist->prev_model_signature != MPC_MODEL_SIGNATURE);
    const int curvature_jump = (kappa_diff > MPC_WS_CURVATURE_THRESH);
    const int hard_reset_reason =
        model_signature_changed || curvature_jump;

    if (!admm_state->initialized || hard_reset_reason) {
      admm_state->initialized = 0;
    }
  }

  /* ---------------------------------------------------------------
   * Step 6: Solve via Riccati-ADMM
   * --------------------------------------------------------------- */
  AdmmConfig_t solver_cfg;
  solver_cfg.rho = ADMM_RHO_DEFAULT;
  solver_cfg.rho_u = ADMM_RHO_U_DEFAULT;
  solver_cfg.tolerance = ADMM_TOL_DEFAULT;
  solver_cfg.max_iterations = ADMM_MAX_ITER_DEFAULT;
  solver_cfg.adaptive_rho = ADMM_ADAPTIVE_RHO_DEFAULT;

  MpcSolution_t sol;
  /* Zero-initialize solution */
  sol.iterations = 0;
  sol.primal_residual = 0;
  sol.dual_residual = 0;
  sol.status = MPC_STATUS_MAX_ITER;

  MpcStatus_t rstatus = riccati_admm_solve_hls(
      step_data, terminal_q_diag, terminal_q_linear, terminal_x_lb,
      terminal_x_ub, x0, &solver_cfg, admm_state, &sol);

  /* ---------------------------------------------------------------
   * Step 7: Extract control output
   *
   * u = [delta_rate, accel]. Convert delta_rate to steering angle:
   *   delta_cmd = delta_actual + dt * delta_rate
   * --------------------------------------------------------------- */
  fp_QP_t delta_rate = admm_state->z_u[0][0];
  fp_QP_t accel = admm_state->z_u[0][1];

  fp_QP_t delta_cmd = persist->actual_steering + fp_mul(MPC_DT, delta_rate);

  /* Clamp to physical limits */
  delta_cmd = fp_clamp(delta_cmd, -VP_MAX_STEER, VP_MAX_STEER);

  /* Final saturation */
  fp_QP_t steer_sat, accel_sat;
  saturate_control_hls(delta_cmd, accel, &steer_sat, &accel_sat);

  /* ---------------------------------------------------------------
   * Step 8: Update persistent state and outputs
   * --------------------------------------------------------------- */
  persist->prev_delta_cmd = delta_cmd;
  persist->prev_steer_rate = delta_rate;
  persist->prev_accel = accel_sat;
  persist->prev_curvature = kappa0;
  persist->prev_model_signature = MPC_MODEL_SIGNATURE;

  *out_steering = steer_sat;
  *out_accel = accel_sat;
  *out_status = (int)rstatus;
  *out_iters = sol.iterations;
}
