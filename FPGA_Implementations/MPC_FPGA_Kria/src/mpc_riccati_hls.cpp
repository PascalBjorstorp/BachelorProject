/**
 * @file mpc_riccati_hls.cpp
 * @brief MPC Riccati-ADMM compute path for HLS.
 *
 * Builds the augmented 8-state QP from the current Frenet state and fixed
 * horizon references, solves it with Riccati-ADMM, and maps the projected
 * control to saturated steering/acceleration outputs.
 *
 * Active warm-start policy in this file:
 *   - cold start -> full ADMM reset
 *   - warm start reset on model-signature jump, first-point curvature jump,
 *     or bound-incompatible persisted e_y state
 *   - solver status is OPTIMAL or MAX_ITER
 *
 * Notes:
 *   - This file intentionally uses the public fp_QP_t-facing interface.
 *   - The active numeric rewrite lives below this layer in the solver/math
 *     files; this wrapper stays policy- and assembly-oriented.
 */

#include "../include/mpc_riccati_hls.h"
#include "../include/riccati_solver_hls.h"

extern void compute_frenet_AB_and_next_hls(
    fp_QP_t ey, fp_QP_t epsi, fp_QP_t vx, fp_QP_t vy, fp_QP_t omega,
    fp_QP_t delta, fp_QP_t a_cmd, fp_QP_t kappa, fp_QP_t reference_velocity,
    fp_QP_t A_fr[MPC_NX_FRENET][MPC_NX_FRENET],
    fp_QP_t B_fr[MPC_NX_FRENET][MPC_NU], fp_QP_t next_state[MPC_NX_FRENET]);

extern void saturate_control_hls(fp_QP_t steer_in, fp_QP_t accel_in,
                                 fp_QP_t *steer_out, fp_QP_t *accel_out);

static void compute_wall_ey_bounds_hls(fp_QP_t left_wall_bound,
                                       fp_QP_t right_wall_bound,
                                       fp_QP_t wall_margin, fp_QP_t *out_x_lb,
                                       fp_QP_t *out_x_ub) {
#pragma HLS INLINE
  fp_QP_t x_lb = wall_margin - right_wall_bound;
  fp_QP_t x_ub = left_wall_bound - wall_margin;

  if (x_lb > x_ub) {
    const fp_sum2_QP_raw_t mid_sum =
        (fp_sum2_QP_raw_t)fp_qp_raw_from_QP(x_lb) +
        (fp_sum2_QP_raw_t)fp_qp_raw_from_QP(x_ub);
    const fp_QP_t mid = fp_QP_from_qp_raw((fp_QP_raw_t)(mid_sum >> 1));
    x_lb = mid;
    x_ub = mid;
  }

  *out_x_lb = x_lb;
  *out_x_ub = x_ub;
}

static fp_QP_t compute_wall_biased_ey_ref_hls(fp_QP_t base_ref, fp_QP_t x_lb,
                                              fp_QP_t x_ub,
                                              fp_QP_t clearance_m,
                                              fp_QP_t max_shift_m) {
#pragma HLS INLINE
  /* CPU-equivalent logic:
   * 1) choose target interval (optionally tightened by clearance)
   * 2) project base_ref into target interval
   * 3) clamp correction magnitude by max_shift
   * 4) clamp final reference to physical [x_lb, x_ub]
   */
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

  const fp_QP_t target_ref = fp_clamp(base_ref, target_lb, target_ub);
  fp_QP_t corr = fp_clamp(target_ref - base_ref, -max_shift_m, max_shift_m);
  return fp_clamp(base_ref + corr, x_lb, x_ub);
}

static fp_QP_t compute_window_min_hls(const fp_QP_t ref_wall[MPC_HORIZON],
                                      int center_idx) {
#pragma HLS INLINE off
  fp_QP_t cand[2 * WALL_BOUND_WINDOW + 1];
#pragma HLS ARRAY_PARTITION variable = cand complete dim = 1

  for (int dj = -WALL_BOUND_WINDOW; dj <= WALL_BOUND_WINDOW; dj++) {
MPC_HLS_UNROLL()
    const int jj = center_idx + dj;
    const int idx = dj + WALL_BOUND_WINDOW;
    cand[idx] = (jj >= 0 && jj < MPC_HORIZON) ? ref_wall[jj] : BIG_BOUND;
  }

  fp_QP_t l2_0 = (cand[0] < cand[1]) ? cand[0] : cand[1];
  fp_QP_t l2_1 = (cand[2] < cand[3]) ? cand[2] : cand[3];
  fp_QP_t l2_2 = (cand[4] < cand[5]) ? cand[4] : cand[5];
  fp_QP_t l2_3 = cand[6];

  fp_QP_t l3_0 = (l2_0 < l2_1) ? l2_0 : l2_1;
  fp_QP_t l3_1 = (l2_2 < l2_3) ? l2_2 : l2_3;
  return (l3_0 < l3_1) ? l3_0 : l3_1;
}

/* Full ADMM reset for HLS-friendly cold starts.
 * Mirrors the CPU riccati_admm_state_init() behavior without memset. */
static void reset_admm_state_hls(AdmmState_t *admm_state) {
#pragma HLS INLINE off
  mpc_admm_reset_all_hls(admm_state);
}

/* Vitis HLS codegen workaround. Computing this exact product+shift inline in
 * compute_affine_bias_dense_hls is miscompiled in RTL only at the
 * FP_CAST_SITE_MUL_MPC_RICCATI_B0 site: csim correct on every row; RTL wrong
 * bit-identically and schedule-invariant (verified across UNROLL+mul-cap,
 * UNROLL, and PIPELINE II=1 builds; lin_delta_k/B_step inputs verified
 * identical RTL vs csim). The identical math synthesizes correctly when it
 * lives in its own INLINE-off module, so isolate just b0 here. Keeps a0..a4,
 * b1, the adder tree and all cycle-tuned pragmas of the caller unchanged. */
static fp_QP_raw_t affine_b0_term_hls(fp_QP_raw_t b0_coeff_raw,
                                      fp_QP_raw_t lin_delta_k_raw) {
#pragma HLS INLINE off
  return fp_shift_right_cast_to_qp_site(
      fp_mul_QP_raw(b0_coeff_raw, lin_delta_k_raw), FP_FRAC_BITS,
      FP_CAST_SITE_MUL_MPC_RICCATI_B0);
}

static void compute_affine_bias_dense_hls(
    const fp_QP_t A_step[MPC_NX_FRENET][MPC_NX_FRENET],
    const fp_QP_t B_step[MPC_NX_FRENET][MPC_NU], fp_QP_t lin_ey,
    fp_QP_t lin_epsi, fp_QP_t lin_vx, fp_QP_t lin_vy, fp_QP_t lin_omega,
    fp_QP_t lin_delta, fp_QP_t uk0, fp_QP_t uk1, fp_QP_t lin_delta_k,
    fp_QP_t next_ey, fp_QP_t next_epsi, fp_QP_t next_vx, fp_QP_t next_vy,
    fp_QP_t next_omega, fp_QP_t next_delta, StepData_t *sd) {
#pragma HLS INLINE off
  int i;
  fp_QP_raw_t xk_raw[MPC_NX_DENSE];
  fp_QP_raw_t xk1_raw[MPC_NX_DENSE];
  fp_QP_raw_t uk0_raw = fp_qp_raw_from_QP(uk0);
  fp_QP_raw_t uk1_raw = fp_qp_raw_from_QP(uk1);
  fp_QP_raw_t lin_delta_k_raw = fp_qp_raw_from_QP(lin_delta_k);
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

  fp_QP_raw_t d_dense_raw[MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable = d_dense_raw complete dim = 1

  /* Fused multiply-accumulate + cast: keep the arithmetic local to this
   * helper, but do not fully unroll the 5-row body. The caller only needs
   * this module to stay well below its outer II=110 budget, so a simple
   * pipelined row loop is cheaper physically than materializing all 5 rows
   * in parallel. */
  for (i = 0; i < 5; i++) {
MPC_HLS_PIPELINE(1)
    const fp_QP_raw_t A0_raw = fp_qp_raw_from_QP(A_step[i][0]);
    const fp_QP_raw_t A1_raw = fp_qp_raw_from_QP(A_step[i][1]);
    const fp_QP_raw_t A2_raw = fp_qp_raw_from_QP(A_step[i][2]);
    const fp_QP_raw_t A3_raw = fp_qp_raw_from_QP(A_step[i][3]);
    const fp_QP_raw_t A4_raw = fp_qp_raw_from_QP(A_step[i][4]);
    const fp_QP_raw_t B0_raw = fp_qp_raw_from_QP(B_step[i][0]);
    const fp_QP_raw_t B1_raw = fp_qp_raw_from_QP(B_step[i][1]);

    const fp_QP_raw_t a0 =
        fp_shift_right_cast_to_qp_site(fp_mul_QP_raw(A0_raw, xk_raw[0]),
                                       FP_FRAC_BITS,
                                       FP_CAST_SITE_MUL_MPC_RICCATI_A0);
    const fp_QP_raw_t a1 =
        fp_shift_right_cast_to_qp_site(fp_mul_QP_raw(A1_raw, xk_raw[1]),
                                       FP_FRAC_BITS,
                                       FP_CAST_SITE_MUL_MPC_RICCATI_A1);
    const fp_QP_raw_t a2 =
        fp_shift_right_cast_to_qp_site(fp_mul_QP_raw(A2_raw, xk_raw[2]),
                                       FP_FRAC_BITS,
                                       FP_CAST_SITE_MUL_MPC_RICCATI_A2);
    const fp_QP_raw_t a3 =
        fp_shift_right_cast_to_qp_site(fp_mul_QP_raw(A3_raw, xk_raw[3]),
                                       FP_FRAC_BITS,
                                       FP_CAST_SITE_MUL_MPC_RICCATI_A3);
    const fp_QP_raw_t a4 =
        fp_shift_right_cast_to_qp_site(fp_mul_QP_raw(A4_raw, xk_raw[4]),
                                       FP_FRAC_BITS,
                                       FP_CAST_SITE_MUL_MPC_RICCATI_A4);
    const fp_QP_raw_t b0 = affine_b0_term_hls(B0_raw, lin_delta_k_raw);
    const fp_QP_raw_t b1 =
        fp_shift_right_cast_to_qp_site(fp_mul_QP_raw(B1_raw, uk1_raw),
                                       FP_FRAC_BITS,
                                       FP_CAST_SITE_MUL_MPC_RICCATI_B1);

    const fp_sum2_QP_raw_t s01 = (fp_sum2_QP_raw_t)a0 + (fp_sum2_QP_raw_t)a1;
    const fp_sum2_QP_raw_t s23 = (fp_sum2_QP_raw_t)a2 + (fp_sum2_QP_raw_t)a3;
    const fp_sum2_QP_raw_t s45 = (fp_sum2_QP_raw_t)a4 + (fp_sum2_QP_raw_t)b0;
    const fp_sum2_QP_raw_t s67 = (fp_sum2_QP_raw_t)b1;
    FP_WPROBE(FP_WP_SUM2_QP_RAW, s01.to_int64());
    FP_WPROBE(FP_WP_SUM2_QP_RAW, s23.to_int64());
    FP_WPROBE(FP_WP_SUM2_QP_RAW, s45.to_int64());
    FP_WPROBE(FP_WP_SUM2_QP_RAW, s67.to_int64());

    const fp_sum4_QP_raw_t s0123 = (fp_sum4_QP_raw_t)s01 + (fp_sum4_QP_raw_t)s23;
    const fp_sum4_QP_raw_t s4567 = (fp_sum4_QP_raw_t)s45 + (fp_sum4_QP_raw_t)s67;
    FP_WPROBE(FP_WP_SUM4_QP_RAW, s0123.to_int64());
    FP_WPROBE(FP_WP_SUM4_QP_RAW, s4567.to_int64());

    const fp_sum8_QP_raw_t acc =
        (fp_sum8_QP_raw_t)xk1_raw[i] - ((fp_sum8_QP_raw_t)s0123 + (fp_sum8_QP_raw_t)s4567);
    FP_WPROBE(FP_WP_SUM8_QP_RAW, acc.to_int64());
    d_dense_raw[i] = (fp_QP_raw_t)acc;
  }

  /* IDX_DELTA_ACT row uses a different formula and is computed once
   * outside the main loop so it can overlap with it. */
  const fp_sum8_QP_raw_t acc_delta_act =
      (fp_sum8_QP_raw_t)xk1_raw[IDX_DELTA_ACT] -
      (fp_sum8_QP_raw_t)xk_raw[IDX_DELTA_ACT] -
      (fp_sum8_QP_raw_t)fp_shift_right_cast_to_qp_site(
          fp_mul_QP_raw(fp_qp_raw_from_QP(MPC_DT), uk0_raw), FP_FRAC_BITS,
          FP_CAST_SITE_MUL_MPC_RICCATI_DT_UK0);
  FP_WPROBE(FP_WP_SUM8_QP_RAW, acc_delta_act.to_int64());
  d_dense_raw[IDX_DELTA_ACT] = (fp_QP_raw_t)acc_delta_act;

  for (i = 0; i < MPC_NX_DENSE; ++i) {
MPC_HLS_PIPELINE(1)
    sd->d[i] = d_dense_raw[i];
  }
}

/* Writeback helper for persistent state.
 *
 * Marked INLINE off to create an explicit HLS module boundary between the
 * mpc_compute_hls FSM and the persist register file. Without this, the
 * loop-done state bit at the end of mpc_compute_hls (e.g. ap_CS_fsm[73])
 * drives the persist register CE/D pins through a long combinational
 * fanout (~60 endpoints), producing the bucket B WNS path. The dedicated
 * helper forces HLS to register the persist writes behind a handshake. */
static void mpc_persist_writeback_hls(MpcPersistState_t *persist,
                                       fp_QP_t prev_delta_cmd,
                                       fp_QP_t prev_steer_rate,
                                       fp_QP_t prev_accel,
                                       fp_QP_t prev_curvature,
                                       int prev_model_signature) {
#pragma HLS INLINE off
  persist->prev_delta_cmd = prev_delta_cmd;
  persist->prev_steer_rate = prev_steer_rate;
  persist->prev_accel = prev_accel;
  persist->prev_curvature = prev_curvature;
  persist->prev_model_signature = prev_model_signature;
}

static void init_solver_cfg_hls(AdmmConfig_t *cfg) {
#pragma HLS INLINE
  cfg->rho = ADMM_RHO_DEFAULT;
  cfg->rho_u = ADMM_RHO_U_DEFAULT;
  cfg->tolerance = ADMM_TOL_DEFAULT;
  cfg->max_iterations = ADMM_MAX_ITER_DEFAULT;
  cfg->adaptive_rho = 1;
}

static void init_solution_hls(MpcSolution_t *sol) {
#pragma HLS INLINE
  sol->iterations = 0;
  sol->primal_residual = FP_QP_CONST(0.0);
  sol->dual_residual = FP_QP_CONST(0.0);
  sol->status = MPC_STATUS_MAX_ITER;
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
 * @param out_steering  Output steering command [rad]
 * @param out_accel     Output acceleration command [m/s^2]
 * @param out_status    Output solver status
 * @param out_iters     Output ADMM iterations used
 */
void mpc_compute_hls(fp_QP_t state_ey, fp_QP_t state_epsi, fp_QP_t state_vx,
                     fp_QP_t state_vy, fp_QP_t state_omega,
                     const MpcRefPoint_t ref[MPC_HORIZON],
                     MpcPersistState_t *persist, AdmmState_t *admm_state,
                     fp_QP_t *out_steering, fp_QP_t *out_accel, int *out_status,
                     int *out_iters) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable = admm_state->z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->y_u complete dim = 2
  /* Share the affine-bias helper across horizon iterations. At II=150 there
   * is enough slack to reuse one instance instead of materializing ~20 clones
   * inside Pipeline_VITIS_LOOP_389_4. */
#pragma HLS ALLOCATION function instances = compute_affine_bias_dense_hls limit = 1

  const fp_QP_t kZero = FP_QP_CONST(0.0);
  const fp_QP_t kHalf = FP_QP_CONST(0.5);
  const fp_QP_t qp_min_lin_vel = MIN_LIN_VEL;

  int k, i, j;

  /* ---------------------------------------------------------------
   * Step 1: first-point curvature for active warm-start validation
   * --------------------------------------------------------------- */
  const fp_QP_t kappa0 = ref[0].path_curvature;

  /* ---------------------------------------------------------------
   * Step 2: build per-step augmented QP data
   * --------------------------------------------------------------- */
  StepData_t step_data[MPC_HORIZON];
#pragma HLS BIND_STORAGE variable = step_data type = RAM_2P impl = BRAM latency = 1
  fp_QP_raw_t B_sparse[MPC_HORIZON][MPC_BSP_N];
#pragma HLS ARRAY_PARTITION variable = B_sparse complete dim = 2

  fp_QP_t terminal_wall_x_lb_con = -BIG_BOUND;
  fp_QP_t terminal_wall_x_ub_con = BIG_BOUND;
  fp_QP_t terminal_ey_ref = kZero;
  fp_QP_t terminal_dff_raw = kZero;

  /* Rolling operating point */
  fp_QP_t lin_ey = state_ey;
  fp_QP_t lin_epsi = state_epsi;
  fp_QP_t lin_vx = (state_vx > qp_min_lin_vel) ? state_vx : qp_min_lin_vel;
  fp_QP_t lin_vy = state_vy;
  fp_QP_t lin_omega = state_omega;
  fp_QP_t lin_delta =
      fp_clamp(persist->actual_steering, -VP_MAX_STEER, VP_MAX_STEER);
  fp_QP_t rollout_steer_rate = persist->prev_steer_rate;

  /* Flatten wall bounds before the sliding-window reduction. */
  fp_QP_t ref_left_wall[MPC_HORIZON];
  fp_QP_t ref_right_wall[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable = ref_left_wall complete dim = 1
#pragma HLS ARRAY_PARTITION variable = ref_right_wall complete dim = 1

  for (k = 0; k < MPC_HORIZON; k++) {
    ref_left_wall[k] = ref[k].left_wall_bound;
    ref_right_wall[k] = ref[k].right_wall_bound;
  }

  fp_QP_t wall_left_min[MPC_HORIZON];
  fp_QP_t wall_right_min[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable = wall_left_min complete dim = 1
#pragma HLS ARRAY_PARTITION variable = wall_right_min complete dim = 1

  for (k = 0; k < MPC_HORIZON; k++) {
    wall_left_min[k] = compute_window_min_hls(ref_left_wall, k);
    wall_right_min[k] = compute_window_min_hls(ref_right_wall, k);
  }

  for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
  /* II floor is the single-instance compute_frenet_AB_and_next_hls latency*/
MPC_HLS_PIPELINE(100)
    StepData_t *sd = &step_data[k];

    const fp_QP_t uk0 = rollout_steer_rate;
    const fp_QP_t uk1 = persist->prev_accel;

    fp_QP_t A_step[MPC_NX_FRENET][MPC_NX_FRENET];
    fp_QP_t B_step[MPC_NX_FRENET][MPC_NU];
    fp_QP_t next_frenet[MPC_NX_FRENET];
#pragma HLS ARRAY_PARTITION variable = A_step complete dim = 2
#pragma HLS ARRAY_PARTITION variable = B_step complete dim = 2
#pragma HLS ARRAY_PARTITION variable = next_frenet complete dim = 1

    const fp_QP_t kappa_k = ref[k].path_curvature;
    const fp_QP_t dff_raw_k =
        fp_atan_lut(fp_mul_site(VP_WHEELBASE, kappa_k,
                                FP_CAST_SITE_MUL_MR_DFF_RAW));
    const fp_QP_t dff_k = fp_clamp(dff_raw_k, -VP_MAX_STEER, VP_MAX_STEER);

    const fp_QP_t lin_delta_k = dff_k;
    const fp_QP_t lin_vx_k = lin_vx;
    const fp_QP_t left_bound_k = wall_left_min[k];
    const fp_QP_t right_bound_k = wall_right_min[k];

    fp_QP_t wall_x_lb, wall_x_ub;
    compute_wall_ey_bounds_hls(left_bound_k, right_bound_k, WALL_MARGIN,
                               &wall_x_lb, &wall_x_ub);

    fp_QP_t wall_x_lb_con = wall_x_lb;
    fp_QP_t wall_x_ub_con = wall_x_ub;
    if (WALL_BIAS_CLEAR_M > kZero) {
      const fp_QP_t desired_lb = wall_x_lb + WALL_BIAS_CLEAR_M;
      const fp_QP_t desired_ub = wall_x_ub - WALL_BIAS_CLEAR_M;
      if (desired_lb <= desired_ub) {
        wall_x_lb_con = desired_lb;
        wall_x_ub_con = desired_ub;
      }
    }

    const fp_QP_t ey_ref_k = compute_wall_biased_ey_ref_hls(
        ref[k].reference_lateral_error, wall_x_lb_con, wall_x_ub_con,
        WALL_REF_CLEAR_M, WALL_BIAS_MAX_M);

    if (k == (MPC_HORIZON - 1)) {
      terminal_wall_x_lb_con = wall_x_lb_con;
      terminal_wall_x_ub_con = wall_x_ub_con;
      terminal_ey_ref = ey_ref_k;
      terminal_dff_raw = dff_raw_k;
    }

    compute_frenet_AB_and_next_hls(
        lin_ey, lin_epsi, lin_vx_k, lin_vy, lin_omega, lin_delta_k, uk1,
        kappa_k, ref[k].reference_velocity, A_step, B_step, next_frenet);

    /* CPU-parity yaw-rate diagonal stability clamp */
    {
      const int row = 4;
      const fp_QP_t limit = STABILITY_LIMIT_VAL;
      fp_QP_t aii = A_step[row][row];

      if (fp_abs(aii) > limit) {
        if (aii < fp_QP_t(0)) {
          A_step[row][row] = fp_QP_t(0) - limit;
        } else {
          A_step[row][row] = limit;
        }
      }
    }

    const fp_QP_t next_ey = next_frenet[0];
    const fp_QP_t next_epsi = next_frenet[1];
    const fp_QP_t next_vx = next_frenet[2];
    const fp_QP_t next_vy = next_frenet[3];
    const fp_QP_t next_omega = next_frenet[4];
    const fp_QP_t next_delta =
        fp_clamp(lin_delta +
                     fp_mul_site(MPC_DT, uk0, FP_CAST_SITE_MUL_MR_DELTA_PRED),
                 -VP_MAX_STEER, VP_MAX_STEER);

    /* Only the 6x6 dense A block is consumed by the Riccati pass. */
    for (j = 0; j < 5; j++) {
MPC_HLS_UNROLL()
      sd->A[IDX_DELTA_ACT][j] = 0;
    }

    /* === Augmented A === */
    for (i = 0; i < 5; i++) {
MPC_HLS_UNROLL()
      for (j = 0; j < 5; j++) {
MPC_HLS_UNROLL()
        sd->A[i][j] = fp_qp_raw_from_QP(A_step[i][j]);
      }
    }

    for (i = 0; i < 5; i++) {
MPC_HLS_UNROLL()
      sd->A[i][IDX_DELTA_ACT] = fp_qp_raw_from_QP(B_step[i][0]);
    }
    sd->A[IDX_DELTA_ACT][IDX_DELTA_ACT] = fp_qp_raw_from_QP(FP_ONE);

    /* === Augmented B (register-resident, see B_sparse above) === */
    B_sparse[k][MPC_BSP_VX_ACCEL] = fp_qp_raw_from_QP(B_step[2][1]);
    B_sparse[k][MPC_BSP_VY_ACCEL] = fp_qp_raw_from_QP(B_step[3][1]);
    B_sparse[k][MPC_BSP_OMEGA_ACCEL] = fp_qp_raw_from_QP(B_step[4][1]);
    B_sparse[k][MPC_BSP_DELTA_RATE] = fp_qp_raw_from_QP((fp_QP_t)MPC_DT);

    /* === Affine dynamics bias d === */
    compute_affine_bias_dense_hls(
      A_step, B_step, lin_ey, lin_epsi, lin_vx, lin_vy, lin_omega,
      lin_delta, uk0, uk1, lin_delta_k, next_ey, next_epsi, next_vx,
      next_vy, next_omega, next_delta, sd);

    /* === Costs ===
     * HLS 200-448 (step_data_2 II>=6): same non-binding class as the sd->d[]
     * writes (see compute_affine_bias_dense_hls). step_data is RAM_2P (1 write
     * port) so these 6 q[] stores serialize -> structural II floor 6. The
     * enclosing QP-assembly loop is pinned at PIPELINE II=108 (6 << 108) so
     * this costs 0 cycles and is not a regression. Do NOT partition q[] to
     * silence it: struct-member sub-array partition on this INLINE-off path
     * is the LayoutTransform clang-segfault pattern, for zero cycle gain. */
    sd->q[0] = fp_qp_raw_from_QP(-fp_mul_site(MPC_Q2_LAT_ERROR, ey_ref_k,
                                              FP_CAST_SITE_MUL_MR_Q_LAT));
    sd->q[1] = fp_qp_raw_from_QP(-fp_mul_site(MPC_Q2_HEADING,
                                              ref[k].reference_heading_error,
                                              FP_CAST_SITE_MUL_MR_Q_HDG));
    sd->q[2] = fp_qp_raw_from_QP(-fp_mul_site(MPC_Q2_VELOCITY,
                                              ref[k].reference_velocity,
                                              FP_CAST_SITE_MUL_MR_Q_VEL));
    sd->q[3] = fp_qp_raw_from_QP(-fp_mul_site(MPC_Q2_LAT_VEL,
                                              ref[k].reference_lateral_velocity,
                                              FP_CAST_SITE_MUL_MR_Q_LAT_VEL));
    sd->q[4] = fp_qp_raw_from_QP(-fp_mul_site(MPC_Q2_YAW_RATE,
                                              ref[k].reference_yaw_rate,
                                              FP_CAST_SITE_MUL_MR_Q_YAW));
    sd->q[IDX_DELTA_ACT] = fp_qp_raw_from_QP(
        -fp_mul_site(MPC_Q2_DELTA_ACT, dff_k, FP_CAST_SITE_MUL_MR_Q_DELTA));
    sd->ey_lb = wall_x_lb_con;
    sd->ey_ub = wall_x_ub_con;

    fp_QP_t v_ref_k = ref[k].reference_velocity;
    fp_QP_t v_model_k = (lin_vx_k < MIN_LIN_VEL) ? MIN_LIN_VEL : lin_vx_k;
    fp_QP_t v_for_limit =
        fp_mul_site(FP_QP_CONST(0.7), v_model_k,
                    FP_CAST_SITE_MUL_MR_V_BLEND_07) +
        fp_mul_site(FP_QP_CONST(0.3), v_ref_k,
                    FP_CAST_SITE_MUL_MR_V_BLEND_03);
    if (v_for_limit < MIN_LIN_VEL) {
      v_for_limit = MIN_LIN_VEL;
    }

    if (v_for_limit > V_SWITCH) {
      const fp_QP_t scale = fp_mul_site(V_SWITCH, fp_recip(v_for_limit),
                                        FP_CAST_SITE_MUL_MR_SCALE_VSWITCH);
      sd->accel_ub =
          fp_mul_site(VP_MAX_ACCEL, scale, FP_CAST_SITE_MUL_MR_UUB_SCALE);
    } else {
      sd->accel_ub = VP_MAX_ACCEL;
    }

    lin_ey = next_ey;
    lin_epsi = next_epsi;
    lin_vx = next_vx;
    lin_vy = next_vy;
    lin_omega = next_omega;
    lin_delta = next_delta;
    rollout_steer_rate = kZero;
  }

  /* ---------------------------------------------------------------
   * Step 3: terminal cost/bounds
   * --------------------------------------------------------------- */
  fp_QP_t terminal_q_diag[MPC_NX_AUG];
  fp_QP_t terminal_q_linear[MPC_NX_AUG];
  fp_QP_t terminal_x_lb[MPC_NX_AUG];
  fp_QP_t terminal_x_ub[MPC_NX_AUG];

  for (i = 0; i < MPC_NX_AUG; i++) {
MPC_HLS_UNROLL()
    terminal_q_diag[i] = kZero;
    terminal_q_linear[i] = kZero;
    terminal_x_lb[i] = -BIG_BOUND;
    terminal_x_ub[i] = BIG_BOUND;
  }

  terminal_q_diag[0] = MPC_Q2_LAT_ERROR;
  terminal_q_diag[1] = MPC_Q2_HEADING;
  terminal_q_diag[2] = MPC_Q2_VELOCITY;
  terminal_q_diag[3] = MPC_Q2_LAT_VEL;
  terminal_q_diag[4] = MPC_Q2_YAW_RATE;
  terminal_q_diag[IDX_DELTA_ACT] = MPC_Q2_DELTA_ACT;

  terminal_x_lb[0] = terminal_wall_x_lb_con;
  terminal_x_ub[0] = terminal_wall_x_ub_con;
  terminal_x_lb[IDX_DELTA_ACT] = -VP_MAX_STEER;
  terminal_x_ub[IDX_DELTA_ACT] = VP_MAX_STEER;

  terminal_q_linear[0] =
      -fp_mul_site(terminal_q_diag[0], terminal_ey_ref,
                   FP_CAST_SITE_MUL_MR_TERM_Q_EY);
  terminal_q_linear[1] =
      -fp_mul_site(terminal_q_diag[1], ref[MPC_HORIZON - 1].reference_heading_error,
                   FP_CAST_SITE_MUL_MR_TERM_Q_HDG);
  terminal_q_linear[2] =
      -fp_mul_site(terminal_q_diag[2], ref[MPC_HORIZON - 1].reference_velocity,
                   FP_CAST_SITE_MUL_MR_TERM_Q_VEL);
  terminal_q_linear[3] =
      -fp_mul_site(terminal_q_diag[3],
                   ref[MPC_HORIZON - 1].reference_lateral_velocity,
                   FP_CAST_SITE_MUL_MR_TERM_Q_LAT_VEL);
  terminal_q_linear[4] =
      -fp_mul_site(terminal_q_diag[4], ref[MPC_HORIZON - 1].reference_yaw_rate,
                   FP_CAST_SITE_MUL_MR_TERM_Q_YAW);

  {
    const fp_QP_t dff_N = fp_clamp(terminal_dff_raw, -VP_MAX_STEER, VP_MAX_STEER);
    terminal_q_linear[IDX_DELTA_ACT] =
        -fp_mul_site(terminal_q_diag[IDX_DELTA_ACT], dff_N,
                     FP_CAST_SITE_MUL_MR_TERM_Q_DELTA);
  }

  /* ---------------------------------------------------------------
   * Step 4: initial augmented state
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
   * Step 5: active warm-start policy
   * --------------------------------------------------------------- */
  {
    /* Cache prev_curvature locally so HLS reads it once and the
     * comparator below sees a clean stack variable rather than a port
     * back to the persist register file. Helps shorten the path from
     * the wall_*_min loop FSM into the warm-start decision. */
    const fp_QP_t prev_curvature_cached = persist->prev_curvature;
    const fp_QP_t kappa_diff = fp_abs(kappa0 - prev_curvature_cached);
    const int curvature_jump = (kappa_diff > MPC_WS_CURVATURE_THRESH);
    const int signature_jump = (persist->prev_model_signature != MPC_MODEL_SIGNATURE);

    int bound_incompatible = 0;
    if (admm_state->initialized) {
      const fp_QP_t ey0 = admm_state->z_x[0][IDX_EY];
      const fp_QP_t eyN = admm_state->z_x[MPC_HORIZON][IDX_EY];
      const fp_QP_t tol = MPC_WS_BOUND_THRESH;

      if (ey0 < (step_data[0].ey_lb - tol) ||
          ey0 > (step_data[0].ey_ub + tol) ||
          eyN < (terminal_x_lb[IDX_EY] - tol) ||
          eyN > (terminal_x_ub[IDX_EY] + tol)) {
        bound_incompatible = 1;
      }
    }

    if (!admm_state->initialized || signature_jump || curvature_jump ||
        bound_incompatible) {
      reset_admm_state_hls(admm_state);
    }
  }

  /* ---------------------------------------------------------------
   * Step 6: solve via Riccati-ADMM
   * --------------------------------------------------------------- */
  AdmmConfig_t solver_cfg;
  init_solver_cfg_hls(&solver_cfg);

  MpcSolution_t sol;
  init_solution_hls(&sol);

  const MpcStatus_t rstatus = riccati_admm_solve_hls(
      step_data, B_sparse, terminal_q_diag, terminal_q_linear, terminal_x_lb,
      terminal_x_ub, x0, &solver_cfg, admm_state, &sol);

  /* ---------------------------------------------------------------
   * Step 7: extract projected control and map to commands
   * --------------------------------------------------------------- */
  const fp_QP_t delta_rate = admm_state->z_u[0][0];
  const fp_QP_t accel = admm_state->z_u[0][1];

  fp_QP_t delta_cmd =
      fp_clamp(persist->actual_steering +
                   fp_mul_site(MPC_DT, delta_rate,
                               FP_CAST_SITE_MUL_MR_PERSIST_STEER),
               -VP_MAX_STEER, VP_MAX_STEER);

  fp_QP_t steer_sat, accel_sat;
  saturate_control_hls(delta_cmd, accel, &steer_sat, &accel_sat);

  /* ---------------------------------------------------------------
   * Step 8: persist and return outputs
   * --------------------------------------------------------------- */
  mpc_persist_writeback_hls(persist, delta_cmd, delta_rate, accel_sat, kappa0,
                            MPC_MODEL_SIGNATURE);

  *out_steering = steer_sat;
  *out_accel = accel_sat;
  *out_status = (int)rstatus;
  *out_iters = sol.iterations;
}
