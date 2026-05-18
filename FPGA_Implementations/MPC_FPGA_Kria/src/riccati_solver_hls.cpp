/**
 * @file riccati_solver_hls.cpp
 * @brief Riccati-ADMM Solver — HLS-Synthesizable Implementation
 * @details Solves constrained LQR using ADMM with Riccati recursion and
 *          fixed-size matrices tailored for FPGA synthesis. The implementation
 *          exploits the augmented-state sparsity pattern to reduce arithmetic
 *          and memory pressure in backward and forward passes.
 * @dependencies riccati_solver_hls.h, fp_math_hls.h
 */

#include "../include/riccati_solver_hls.h"
#include "../include/fp_math_hls.h"
#include <climits>

#define MPC_HLS_DEBUG_TRACE_MAX 256

#ifndef __SYNTHESIS__

static MpcHlsDebugIterSample_t g_hls_debug_trace[MPC_HLS_DEBUG_TRACE_MAX];
static int g_hls_debug_trace_count = 0;

extern "C" int riccati_hls_debug_get_trace_count(void) {
  return g_hls_debug_trace_count;
}

extern "C" int
riccati_hls_debug_get_trace_sample(int index, MpcHlsDebugIterSample_t *out) {
  if (!out || index < 0 || index >= g_hls_debug_trace_count)
    return -1;
  *out = g_hls_debug_trace[index];
  return 0;
}

static void hls_debug_trace_push(int iter, fp_QP_t primal_res, fp_QP_t dual_res,
                                 fp_QP_t state_primal, fp_QP_t state_dual,
                                 fp_QP_t ctrl_primal, fp_QP_t ctrl_dual,
                                 fp_QP_t rho, fp_QP_t rho_u, fp_QP_t u0_steer,
                                 fp_QP_t u0_accel, fp_QP_t z0_steer,
                                 fp_QP_t z0_accel, fp_QP_t y0_steer,
                                 fp_QP_t y0_accel) {
  if (g_hls_debug_trace_count >= MPC_HLS_DEBUG_TRACE_MAX)
    return;
  MpcHlsDebugIterSample_t *s = &g_hls_debug_trace[g_hls_debug_trace_count++];
  s->iter = iter;
  s->primal_residual = (float)primal_res;
  s->dual_residual = (float)dual_res;
  s->state_primal_residual = (float)state_primal;
  s->state_dual_residual = (float)state_dual;
  s->ctrl_primal_residual = (float)ctrl_primal;
  s->ctrl_dual_residual = (float)ctrl_dual;
  s->rho = (float)rho;
  s->rho_u = (float)rho_u;
  s->u0_steer = (float)u0_steer;
  s->u0_accel = (float)u0_accel;
  s->z0_steer = (float)z0_steer;
  s->z0_accel = (float)z0_accel;
  s->y0_steer = (float)y0_steer;
  s->y0_accel = (float)y0_accel;
  s->scale_rho = 0;
  s->scale_rho_u = 0;
}

#else

extern "C" int riccati_hls_debug_get_trace_count(void) {
  return 0;
}

extern "C" int
riccati_hls_debug_get_trace_sample(int index, MpcHlsDebugIterSample_t *out) {
  (void)index;
  (void)out;
  return -1;
}

static void hls_debug_trace_push(int iter, fp_QP_t primal_res, fp_QP_t dual_res,
                                 fp_QP_t state_primal, fp_QP_t state_dual,
                                 fp_QP_t ctrl_primal, fp_QP_t ctrl_dual,
                                 fp_QP_t rho, fp_QP_t rho_u, fp_QP_t u0_steer,
                                 fp_QP_t u0_accel, fp_QP_t z0_steer,
                                 fp_QP_t z0_accel, fp_QP_t y0_steer,
                                 fp_QP_t y0_accel) {
  (void)iter;
  (void)primal_res;
  (void)dual_res;
  (void)state_primal;
  (void)state_dual;
  (void)ctrl_primal;
  (void)ctrl_dual;
  (void)rho;
  (void)rho_u;
  (void)u0_steer;
  (void)u0_accel;
  (void)z0_steer;
  (void)z0_accel;
  (void)y0_steer;
  (void)y0_accel;
}

#endif

static inline fp_QP_t step_ey_lb(const StepData_t *sd) {
#pragma HLS INLINE
  return sd->ey_lb;
}

static inline fp_QP_t step_ey_ub(const StepData_t *sd) {
#pragma HLS INLINE
  return sd->ey_ub;
}

static inline fp_QP_t step_accel_ub(const StepData_t *sd) {
#pragma HLS INLINE
  return sd->accel_ub;
}

static void
admm_update_state_channel_raw(fp_QP_t x_val, fp_QP_t *z_slot, fp_QP_t *y_slot,
                              fp_QP_t rho, fp_QP_t lb, fp_QP_t ub,
                              fp_QP_t *state_primal, fp_QP_t *state_dual,
                              fp_QP_t *z_norm_k, fp_QP_t *lambda_norm_k) {
#pragma HLS INLINE
  const fp_QP_raw_t x_raw = fp_qp_raw_from_QP(x_val);
  const fp_QP_raw_t y_raw = fp_qp_raw_from_QP(*y_slot);
  const fp_QP_raw_t lb_raw = fp_qp_raw_from_QP(lb);
  const fp_QP_raw_t ub_raw = fp_qp_raw_from_QP(ub);

  fp_sum2_QP_raw_t val = (fp_sum2_QP_raw_t)x_raw + (fp_sum2_QP_raw_t)y_raw;
  if (val < (fp_sum2_QP_raw_t)lb_raw)
    val = (fp_sum2_QP_raw_t)lb_raw;
  if (val > (fp_sum2_QP_raw_t)ub_raw)
    val = (fp_sum2_QP_raw_t)ub_raw;

  const fp_QP_raw_t z_new_raw = (fp_QP_raw_t)val;
  const fp_QP_t z_new = fp_QP_from_qp_raw(z_new_raw);

  const fp_QP_raw_t z_prev_raw = fp_qp_raw_from_QP(*z_slot);
  const fp_QP_raw_t dz_raw =
      fp_sub_cast_qp_raw(z_new_raw, z_prev_raw, FP_CAST_SITE_SUM2_SUB_CAST);
  const fp_QP_t dd = fp_abs(fp_mul_site(rho, fp_QP_from_qp_raw(dz_raw),
                                        FP_CAST_SITE_MUL_RS_DUAL_STATE_DD));
  if (dd > *state_dual)
    *state_dual = dd;

  const fp_QP_raw_t y_new_raw =
      fp_add3_cast_qp_raw(x_raw, (fp_QP_raw_t)(-z_new_raw), y_raw,
                          FP_CAST_SITE_SUM2_ADD3_CAST);
  const fp_QP_t y_new = fp_QP_from_qp_raw(y_new_raw);
  *y_slot = y_new;

  fp_QP_t pd = fp_abs(x_val - z_new);
  if (pd > *state_primal)
    *state_primal = pd;

  fp_QP_t abs_z = fp_abs(z_new);
  fp_QP_t abs_l =
      fp_abs(fp_mul_site(rho, y_new, FP_CAST_SITE_MUL_RS_DUAL_STATE_ABSL));
  if (abs_z > *z_norm_k)
    *z_norm_k = abs_z;
  if (abs_l > *lambda_norm_k)
    *lambda_norm_k = abs_l;
  *z_slot = z_new;
}

static void
admm_update_control_channel_raw(fp_QP_t u_val, fp_QP_t *z_slot, fp_QP_t *y_slot,
                                fp_QP_t rho_u, fp_QP_t lb, fp_QP_t ub,
                                fp_QP_t *ctrl_primal, fp_QP_t *ctrl_dual,
                                fp_QP_t *z_norm_k, fp_QP_t *lambda_norm_k) {
#pragma HLS INLINE
  const fp_QP_raw_t u_raw = fp_qp_raw_from_QP(u_val);
  const fp_QP_raw_t y_raw = fp_qp_raw_from_QP(*y_slot);
  const fp_QP_raw_t lb_raw = fp_qp_raw_from_QP(lb);
  const fp_QP_raw_t ub_raw = fp_qp_raw_from_QP(ub);

  fp_sum2_QP_raw_t val = (fp_sum2_QP_raw_t)u_raw + (fp_sum2_QP_raw_t)y_raw;
  if (val < (fp_sum2_QP_raw_t)lb_raw)
    val = (fp_sum2_QP_raw_t)lb_raw;
  if (val > (fp_sum2_QP_raw_t)ub_raw)
    val = (fp_sum2_QP_raw_t)ub_raw;

  const fp_QP_raw_t z_new_raw = (fp_QP_raw_t)val;
  const fp_QP_t z_new = fp_QP_from_qp_raw(z_new_raw);

  const fp_QP_raw_t z_prev_raw = fp_qp_raw_from_QP(*z_slot);
  const fp_QP_raw_t dz_raw =
      fp_sub_cast_qp_raw(z_new_raw, z_prev_raw, FP_CAST_SITE_SUM2_SUB_CAST);
  const fp_QP_t dd = fp_abs(fp_mul_site(rho_u, fp_QP_from_qp_raw(dz_raw),
                                        FP_CAST_SITE_MUL_RS_DUAL_CTRL_DD));
  if (dd > *ctrl_dual)
    *ctrl_dual = dd;

  const fp_QP_raw_t y_new_raw =
      fp_add3_cast_qp_raw(u_raw, (fp_QP_raw_t)(-z_new_raw), y_raw,
                          FP_CAST_SITE_SUM2_ADD3_CAST);
  const fp_QP_t y_new = fp_QP_from_qp_raw(y_new_raw);
  *y_slot = y_new;

  fp_QP_t pd = fp_abs(u_val - z_new);
  if (pd > *ctrl_primal)
    *ctrl_primal = pd;

  fp_QP_t abs_z = fp_abs(z_new);
  fp_QP_t abs_l =
      fp_abs(fp_mul_site(rho_u, y_new, FP_CAST_SITE_MUL_RS_DUAL_CTRL_ABSL));
  if (abs_z > *z_norm_k)
    *z_norm_k = abs_z;
  if (abs_l > *lambda_norm_k)
    *lambda_norm_k = abs_l;
  *z_slot = z_new;
}

/*===========================================================================
 * Riccati Backward + Forward Pass
 *
 * Backward: compute K_k, kk_k for k = N-1 .. 0
 * Forward:  roll out x_k, u_k from x0
 *
 * Exploits A/B sparsity:
 *   A: dense 6x6 block (rows/cols 0-5), rows 6-7 and cols 6-7 zero
 *   B: represented by the external sparse `B_sparse` horizon array,
 *      with implicit latch rows x6_next=u0 and x7_next=u1
 *===========================================================================*/

/* Forward Riccati pass extracted as its own INLINE-off module.
 *
 * Step 1a of the principled riccati_pass_hls modularization. Whole-array
 * interface (K[][][], kk[][], step_data[], x_out[][], u_out[][]) with
 * partition pragmas matching the caller exactly - the proven-safe
 * pattern (same shape as the existing riccati_pass_hls signature),
 * NOT the dim=0/struct-member/slice pattern that segfaulted the
 * LayoutTransform pass. Bit-identical arithmetic: pure code move.
 *
 * Breaks the monolithic riccati_pass FSM into backward+forward modules
 * so Vivado floorplans them as separate clusters (congestion relief)
 * and de-risks the 3-D whole-array K/kk interface before the larger
 * backward-pass split + rho-memoization. */
static void
riccati_forward_pass(const StepData_t step_data[MPC_HORIZON],
                     const fp_QP_raw_t B_sparse[MPC_HORIZON][MPC_BSP_N],
                     const fp_QP_t x0[MPC_NX_AUG],
                     const fp_K_raw_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG],
                     const fp_K_raw_t kk[MPC_HORIZON][MPC_NU],
                     fp_QP_t x_out[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG],
                     fp_QP_t u_out[MPC_HORIZON][MPC_NU]) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable = B_sparse complete dim = 2
#pragma HLS ARRAY_PARTITION variable = x_out complete dim = 2
#pragma HLS ARRAY_PARTITION variable = u_out complete dim = 2
#pragma HLS ARRAY_PARTITION variable = K complete dim = 2
#pragma HLS ARRAY_PARTITION variable = K complete dim = 3
#pragma HLS ARRAY_PARTITION variable = kk complete dim = 2
#pragma HLS ALLOCATION function instances = sum8_K_QP_raw limit = 2
#pragma HLS ALLOCATION function instances = sum6_QP_raw   limit = 4
#pragma HLS ALLOCATION operation instances = mul limit = 16
  int i, k, s;

  for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
    x_out[0][s] = x0[s];
  }

  /* Forward Riccati pass: recurrence-bound (x_out[k+1] depends on
   * x_out[k]). Sequential by necessity. */
  for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS LOOP_FLATTEN off
    const StepData_t *sd = &step_data[k];
    const fp_QP_raw_t d0_fwd = sd->d[0];
    const fp_QP_raw_t d1_fwd = sd->d[1];
    const fp_QP_raw_t d2_fwd = sd->d[2];
    const fp_QP_raw_t d3_fwd = sd->d[3];
    const fp_QP_raw_t d4_fwd = sd->d[4];
    const fp_QP_raw_t d5_fwd = sd->d[5];

    fp_QP_raw_t xk_raw[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = xk_raw complete dim = 1
    for (s = 0; s < MPC_NX_AUG; ++s) {
#pragma HLS UNROLL
      xk_raw[s] = fp_qp_raw_from_QP(x_out[k][s]);
    }

    fp_sum8_K_QP_t prod_sum0 = sum8_K_QP_raw(
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][0], xk_raw[0]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][1], xk_raw[1]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][2], xk_raw[2]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][3], xk_raw[3]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][4], xk_raw[4]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][5], xk_raw[5]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][6], xk_raw[6]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][0][7], xk_raw[7]));

    fp_sum8_K_QP_t prod_sum1 = sum8_K_QP_raw(
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][0], xk_raw[0]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][1], xk_raw[1]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][2], xk_raw[2]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][3], xk_raw[3]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][4], xk_raw[4]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][5], xk_raw[5]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][6], xk_raw[6]),
        (fp_sum8_K_QP_t)fp_mul_K_QP(K[k][1][7], xk_raw[7]));

    const fp_QP_raw_t u0_raw =
        add_cast_QP_raw((fp_QP_raw_t)kk[k][0],
                        fp_shift_right_cast<fp_QP_raw_t>(prod_sum0,
                                                         FP_FRAC_BITS));
    const fp_QP_raw_t u1_raw =
        add_cast_QP_raw((fp_QP_raw_t)kk[k][1],
                        fp_shift_right_cast<fp_QP_raw_t>(prod_sum1,
                                                         FP_FRAC_BITS));

    const fp_QP_t u0_k = fp_QP_from_qp_raw(u0_raw);
    const fp_QP_t u1_k = fp_QP_from_qp_raw(u1_raw);
    u_out[k][0] = u0_k;
    u_out[k][1] = u1_k;

    for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II = 1
      fp_QP_raw_t d_i = d0_fwd;
      if (i == 1) d_i = d1_fwd;
      else if (i == 2) d_i = d2_fwd;
      else if (i == 3) d_i = d3_fwd;
      else if (i == 4) d_i = d4_fwd;
      else if (i == 5) d_i = d5_fwd;

      fp_sum6_QP_mul_t ax_sum = sum6_QP_raw(
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(sd->A[i][0], xk_raw[0]),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(sd->A[i][1], xk_raw[1]),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(sd->A[i][2], xk_raw[2]),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(sd->A[i][3], xk_raw[3]),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(sd->A[i][4], xk_raw[4]),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(sd->A[i][5], xk_raw[5]));

      fp_sum6_QP_mul_t sum =
          ((fp_sum6_QP_mul_t)d_i << FP_FRAC_BITS) +
          (fp_sum6_QP_mul_t)ax_sum;

      if (i == 2) {
        sum += (fp_sum6_QP_mul_t)fp_mul_QP_raw(
            B_sparse[k][MPC_BSP_VX_ACCEL], u1_raw);
      } else if (i == 3) {
        sum += (fp_sum6_QP_mul_t)fp_mul_QP_raw(
            B_sparse[k][MPC_BSP_VY_ACCEL], u1_raw);
      } else if (i == 4) {
        sum += (fp_sum6_QP_mul_t)fp_mul_QP_raw(
            B_sparse[k][MPC_BSP_OMEGA_ACCEL], u1_raw);
      } else if (i == 5) {
        sum += (fp_sum6_QP_mul_t)fp_mul_QP_raw(
            B_sparse[k][MPC_BSP_DELTA_RATE], u0_raw);
      }

      fp_QP_raw_t result = fp_shift_right_cast<fp_QP_raw_t>(sum, FP_FRAC_BITS);
      x_out[k + 1][i] = fp_QP_from_qp_raw(result);
    }

    x_out[k + 1][IDX_DELTA_RATE_PREV] = u0_k;
    x_out[k + 1][IDX_ACCEL_PREV] = u1_k;
  }
}

/* Backward Riccati pass extracted as its own INLINE-off module.
 *
 * Step 1b of the modularization. Step 1a (forward-pass split) moved
 * post-placement WNS from a chronic -0.28 ns to +0.40 ns by breaking
 * the monolithic riccati_pass FSM; this completes the split so the
 * remaining heavy backward FSM is its own placeable cluster, relieving
 * the residual level-6 routing congestion (concentrated in the old
 * riccati_pass_hls / fu_1791).
 *
 * Same proven-safe whole-array interface as riccati_forward_pass:
 * K[][][]/kk[][]/step_data[]/z_x[][]/... with caller-matching
 * partition pragmas. NO dim=0 / struct-member / array-slice params
 * (the pattern that segfaulted the LayoutTransform pass). Bit-
 * identical arithmetic: pure code move. */
static void
riccati_backward_pass(const StepData_t step_data[MPC_HORIZON],
                      const fp_QP_raw_t B_sparse[MPC_HORIZON][MPC_BSP_N],
                      const fp_QP_t terminal_q_diag[MPC_NX_AUG],
                      const fp_QP_t terminal_q_linear[MPC_NX_AUG],
                      fp_QP_t rho, fp_QP_t rho_u,
                      const fp_QP_t z_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG],
                      const fp_QP_t y_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG],
                      const fp_QP_t z_u[MPC_HORIZON][MPC_NU],
                      const fp_QP_t y_u[MPC_HORIZON][MPC_NU],
                      fp_K_raw_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG],
                      fp_K_raw_t kk[MPC_HORIZON][MPC_NU]) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable = B_sparse complete dim = 2
#pragma HLS ARRAY_PARTITION variable = z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = K complete dim = 2
#pragma HLS ARRAY_PARTITION variable = K complete dim = 3
#pragma HLS ARRAY_PARTITION variable = kk complete dim = 2
#pragma HLS ALLOCATION function instances = fp_recip         limit = 2
#pragma HLS ALLOCATION function instances = sum6_P_QP_raw    limit = 2
#pragma HLS ALLOCATION function instances = sum6_MG_QP_raw   limit = 1
#pragma HLS ALLOCATION function instances = sum8_P_MIX_raw   limit = 21
#pragma HLS ALLOCATION function instances = sum8_P_MIX_raw_pupdate limit = 21

  const int nx = MPC_NX_AUG;
  const int nu = MPC_NU;
  const int N = MPC_HORIZON;

  fp_P_raw_t P[MPC_NX_AUG][MPC_NX_AUG];
  fp_P_raw_t p[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = P complete dim = 2
#pragma HLS BIND_STORAGE variable = P type = RAM_2P impl = LUTRAM latency = 1
#pragma HLS ARRAY_PARTITION variable = p complete dim = 1

  int s, i, j, a, b, k;

  for (i = 0; i < nx; i++) {
#pragma HLS UNROLL
    for (j = 0; j < nx; j++) {
#pragma HLS UNROLL
      P[i][j] = 0;
    }
    p[i] = 0;
  }

  for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
    P[s][s] = fp_P_raw_from_QP(terminal_q_diag[s]);
    p[s] = fp_P_raw_from_QP(terminal_q_linear[s]);
  }
  {
    const int idx = IDX_EY;
    P[idx][idx] += fp_P_raw_from_QP(rho);
    fp_QP_raw_t zx_minus_yx =
        fp_sub_cast_qp_raw(fp_qp_raw_from_QP(z_x[N][idx]),
                           fp_qp_raw_from_QP(y_x[N][idx]),
                           FP_CAST_SITE_SUM2_SUB_CAST);
    fp_QP_mul_t rho_state_mul =
        fp_mul_QP_raw(fp_qp_raw_from_QP(rho), zx_minus_yx);
    const fp_P_raw_t rho_state_term =
        (fp_P_raw_t)(rho_state_mul >> FP_FRAC_BITS);
    p[idx] = (fp_P_raw_t)((fp_sum2_P_raw_t)p[idx] +
                          (fp_sum2_P_raw_t)(-rho_state_term));
  }
  {
    const int idx = IDX_DELTA_ACT;
    P[idx][idx] += fp_P_raw_from_QP(rho);
    fp_QP_raw_t zx_minus_yx =
        fp_sub_cast_qp_raw(fp_qp_raw_from_QP(z_x[N][idx]),
                           fp_qp_raw_from_QP(y_x[N][idx]),
                           FP_CAST_SITE_SUM2_SUB_CAST);
    fp_QP_mul_t rho_state_mul =
        fp_mul_QP_raw(fp_qp_raw_from_QP(rho), zx_minus_yx);
    const fp_P_raw_t rho_state_term =
        (fp_P_raw_t)(rho_state_mul >> FP_FRAC_BITS);
    p[idx] = (fp_P_raw_t)((fp_sum2_P_raw_t)p[idx] +
                          (fp_sum2_P_raw_t)(-rho_state_term));
  }

  /* Backward Riccati pass: NOT pipelineable.
   *
   * This loop has a hard mathematical recurrence: P_k depends on
   * P_{k+1} through the full Riccati update (M -> S -> 2x2 invert ->
   * K/G -> dense P-update). Each iteration needs the complete result
   * of the previous one, so PIPELINE cannot overlap iterations.
   *
   * Tried II=40 (final II=53, 8.0 ns path), helper-extracted S +
   * II=40 (8.35 ns), and II=80 with internally-pipelined S helper.
   * All catastrophic (last: WNS -4.66 ns, 28k failing endpoints).
   * The serial dependency is structural - leave this sequential. */
  for (k = N - 1; k >= 0; k--) {
#pragma HLS LOOP_FLATTEN off
    const StepData_t *sd = &step_data[k];
    const fp_QP_raw_t d0_raw = sd->d[0];
    const fp_QP_raw_t d1_raw = sd->d[1];
    const fp_QP_raw_t d2_raw = sd->d[2];
    const fp_QP_raw_t d3_raw = sd->d[3];
    const fp_QP_raw_t d4_raw = sd->d[4];
    const fp_QP_raw_t d5_raw = sd->d[5];

    fp_P_raw_t q_aug_diag[MPC_NX_AUG];
    fp_P_raw_t q_aug_linear[MPC_NX_AUG];
    fp_QP_raw_t r_aug_diag[MPC_NU];
    fp_MG_raw_t r_aug_linear[MPC_NU];
#pragma HLS ARRAY_PARTITION variable = q_aug_diag complete dim = 1
#pragma HLS ARRAY_PARTITION variable = q_aug_linear complete dim = 1
#pragma HLS ARRAY_PARTITION variable = r_aug_diag complete dim = 1
#pragma HLS ARRAY_PARTITION variable = r_aug_linear complete dim = 1

    const bool first_stage = (k == 0);
    q_aug_diag[0] = fp_P_raw_from_QP(MPC_Q2_LAT_ERROR);
    q_aug_diag[1] = fp_P_raw_from_QP(MPC_Q2_HEADING);
    q_aug_diag[2] = fp_P_raw_from_QP(MPC_Q2_VELOCITY);
    q_aug_diag[3] = fp_P_raw_from_QP(MPC_Q2_LAT_VEL);
    q_aug_diag[4] = fp_P_raw_from_QP(MPC_Q2_YAW_RATE);
    q_aug_diag[IDX_DELTA_ACT] = fp_P_raw_from_QP(MPC_Q2_DELTA_ACT);
    fp_QP_t q_diag_delta_prev = (fp_QP_t)MPC_Q2_STEER_JERK;
    fp_QP_t q_diag_accel_prev = (fp_QP_t)MPC_Q2_ACCEL_RATE;
    if (first_stage) {
      q_diag_delta_prev = (fp_QP_t)MPC_Q2_JERK_CS;
      q_diag_accel_prev = (fp_QP_t)MPC_Q2_ARATE_CS;
    }
    q_aug_diag[IDX_DELTA_RATE_PREV] = fp_P_raw_from_QP(q_diag_delta_prev);
    q_aug_diag[IDX_ACCEL_PREV] = fp_P_raw_from_QP(q_diag_accel_prev);

    q_aug_linear[0] = (fp_P_raw_t)sd->q[0];
    q_aug_linear[1] = (fp_P_raw_t)sd->q[1];
    q_aug_linear[2] = (fp_P_raw_t)sd->q[2];
    q_aug_linear[3] = (fp_P_raw_t)sd->q[3];
    q_aug_linear[4] = (fp_P_raw_t)sd->q[4];
    q_aug_linear[IDX_DELTA_ACT] = (fp_P_raw_t)sd->q[IDX_DELTA_ACT];
    q_aug_linear[IDX_DELTA_RATE_PREV] = 0;
    q_aug_linear[IDX_ACCEL_PREV] = 0;

    {
      const int idx = IDX_EY;
      q_aug_diag[idx] += fp_P_raw_from_QP(rho);
      fp_QP_raw_t zx_minus_yx =
          fp_sub_cast_qp_raw(fp_qp_raw_from_QP(z_x[k][idx]),
                             fp_qp_raw_from_QP(y_x[k][idx]),
                             FP_CAST_SITE_SUM2_SUB_CAST);
      fp_QP_mul_t rho_state_mul =
          fp_mul_QP_raw(fp_qp_raw_from_QP(rho), zx_minus_yx);
      const fp_P_raw_t rho_state_term =
          (fp_P_raw_t)(rho_state_mul >> FP_FRAC_BITS);
      q_aug_linear[idx] =
          (fp_P_raw_t)((fp_sum2_P_raw_t)q_aug_linear[idx] +
                       (fp_sum2_P_raw_t)(-rho_state_term));
    }

    {
      const int idx = IDX_DELTA_ACT;
      q_aug_diag[idx] += fp_P_raw_from_QP(rho);
      fp_QP_raw_t zx_minus_yx =
          fp_sub_cast_qp_raw(fp_qp_raw_from_QP(z_x[k][idx]),
                             fp_qp_raw_from_QP(y_x[k][idx]),
                             FP_CAST_SITE_SUM2_SUB_CAST);
      fp_QP_mul_t rho_state_mul =
          fp_mul_QP_raw(fp_qp_raw_from_QP(rho), zx_minus_yx);
      const fp_P_raw_t rho_state_term =
          (fp_P_raw_t)(rho_state_mul >> FP_FRAC_BITS);
      q_aug_linear[idx] =
          (fp_P_raw_t)((fp_sum2_P_raw_t)q_aug_linear[idx] +
                       (fp_sum2_P_raw_t)(-rho_state_term));
    }

    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
      fp_QP_t r_diag = (fp_QP_t)MPC_R2_ACCEL;
      if (a == 0) {
        r_diag = (fp_QP_t)MPC_R2_STEER;
      }
      if (first_stage) {
        r_diag = (fp_QP_t)MPC_R2_ACCEL_CS;
        if (a == 0) {
          r_diag = (fp_QP_t)MPC_R2_STEER_CS;
        }
      }
      r_aug_diag[a] =
          add_cast_QP_raw(fp_qp_raw_from_QP(r_diag), fp_qp_raw_from_QP(rho_u));
      fp_QP_raw_t zu_minus_yu =
          fp_sub_cast_qp_raw(fp_qp_raw_from_QP(z_u[k][a]),
                             fp_qp_raw_from_QP(y_u[k][a]),
                             FP_CAST_SITE_SUM2_SUB_CAST);
      fp_QP_mul_t rho_ctrl_mul =
          fp_mul_QP_raw(fp_qp_raw_from_QP(rho_u), zu_minus_yu);
      const fp_MG_raw_t rho_ctrl_term =
          (fp_MG_raw_t)(rho_ctrl_mul >> FP_FRAC_BITS);
      fp_sum2_QP_raw_t rlin = -(fp_sum2_QP_raw_t)rho_ctrl_term;
      r_aug_linear[a] = (fp_MG_raw_t)rlin;
    }

    fp_MG_raw_t M[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = M complete dim = 1
#pragma HLS ARRAY_PARTITION variable = M complete dim = 2

    const fp_QP_raw_t b00 = B_sparse[k][MPC_BSP_DELTA_RATE];
    const fp_QP_raw_t b10 = B_sparse[k][MPC_BSP_VX_ACCEL];
    const fp_QP_raw_t b11 = B_sparse[k][MPC_BSP_VY_ACCEL];
    const fp_QP_raw_t b12 = B_sparse[k][MPC_BSP_OMEGA_ACCEL];

    for (j = 0; j < nx; j++) {
#pragma HLS UNROLL
      fp_sum2_P_QP_t s0 =
          (fp_sum2_P_QP_t)fp_mul_QP_P(b00, P[IDX_DELTA_ACT][j]) +
          ((fp_sum2_P_QP_t)P[IDX_DELTA_RATE_PREV][j] << FP_FRAC_BITS);

      fp_sum4_P_QP_t s1 =
          (fp_sum4_P_QP_t)fp_mul_QP_P(b10, P[2][j]) +
          (fp_sum4_P_QP_t)fp_mul_QP_P(b11, P[3][j]) +
          (fp_sum4_P_QP_t)fp_mul_QP_P(b12, P[4][j]) +
          ((fp_sum4_P_QP_t)P[IDX_ACCEL_PREV][j] << FP_FRAC_BITS);

      M[0][j] = fp_shift_right_cast<fp_MG_raw_t>(s0, FP_FRAC_BITS);
      M[1][j] = fp_shift_right_cast<fp_MG_raw_t>(s1, FP_FRAC_BITS);
    }

    fp_QP_raw_t S[2][2];
    {
      fp_sum2_MG_QP_t mb00 =
          (fp_sum2_MG_QP_t)fp_mul_MG_QP(M[0][IDX_DELTA_ACT], b00) +
          ((fp_sum2_MG_QP_t)M[0][IDX_DELTA_RATE_PREV] << FP_FRAC_BITS);

      fp_sum4_MG_QP_t mb01 =
          (fp_sum4_MG_QP_t)fp_mul_MG_QP(M[0][2], b10) +
          (fp_sum4_MG_QP_t)fp_mul_MG_QP(M[0][3], b11) +
          (fp_sum4_MG_QP_t)fp_mul_MG_QP(M[0][4], b12) +
          ((fp_sum4_MG_QP_t)M[0][IDX_ACCEL_PREV] << FP_FRAC_BITS);

      fp_sum2_MG_QP_t mb10 =
          (fp_sum2_MG_QP_t)fp_mul_MG_QP(M[1][IDX_DELTA_ACT], b00) +
          ((fp_sum2_MG_QP_t)M[1][IDX_DELTA_RATE_PREV] << FP_FRAC_BITS);

      fp_sum4_MG_QP_t mb11 =
          (fp_sum4_MG_QP_t)fp_mul_MG_QP(M[1][2], b10) +
          (fp_sum4_MG_QP_t)fp_mul_MG_QP(M[1][3], b11) +
          (fp_sum4_MG_QP_t)fp_mul_MG_QP(M[1][4], b12) +
          ((fp_sum4_MG_QP_t)M[1][IDX_ACCEL_PREV] << FP_FRAC_BITS);

      S[0][0] =
          (fp_QP_raw_t)((fp_sum2_QP_raw_t)r_aug_diag[0] +
                        (fp_sum2_QP_raw_t)fp_shift_right_cast<fp_QP_raw_t>(mb00, FP_FRAC_BITS));
      S[0][1] = fp_shift_right_cast<fp_QP_raw_t>(mb01, FP_FRAC_BITS);
      S[1][0] = fp_shift_right_cast<fp_QP_raw_t>(mb10, FP_FRAC_BITS);
      S[1][1] =
          (fp_QP_raw_t)((fp_sum2_QP_raw_t)r_aug_diag[1] +
                        (fp_sum2_QP_raw_t)fp_shift_right_cast<fp_QP_raw_t>(mb11, FP_FRAC_BITS));

      fp_sum2_QP_raw_t s01 = (((fp_sum2_QP_raw_t)S[0][1]) +
                              ((fp_sum2_QP_raw_t)S[1][0])) >> 1;
      S[0][1] = (fp_QP_raw_t)s01;
      S[1][0] = (fp_QP_raw_t)s01;
    }

    fp_QP_raw_t Si[2][2];
    if (invert_2x2_qp_hls(S, Si) < 0) {
      const fp_QP_raw_t eps_qp_raw =
          fp_qp_raw_from_QP(fp_qp_from_neg_pow2(FP_INVERT_2X2_DIAG_FALLBACK_MIN_EXP));
      const fp_QP_t s00_qp = fp_QP_from_qp_raw((S[0][0] > eps_qp_raw) ? S[0][0] : eps_qp_raw);
      const fp_QP_t s11_qp = fp_QP_from_qp_raw((S[1][1] > eps_qp_raw) ? S[1][1] : eps_qp_raw);
      Si[0][0] = fp_qp_raw_from_QP(fp_recip(s00_qp));
      Si[0][1] = 0;
      Si[1][0] = 0;
      Si[1][1] = fp_qp_raw_from_QP(fp_recip(s11_qp));
    }

    fp_MG_raw_t G[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = G complete dim = 1
#pragma HLS ARRAY_PARTITION variable = G complete dim = 2
    for (a = 0; a < nu; a++) {
      for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II = 1
        fp_sum6_MG_QP_t sum = sum6_MG_QP_raw(
            (fp_sum6_MG_QP_t)fp_mul_MG_QP(M[a][0], sd->A[0][j]),
            (fp_sum6_MG_QP_t)fp_mul_MG_QP(M[a][1], sd->A[1][j]),
            (fp_sum6_MG_QP_t)fp_mul_MG_QP(M[a][2], sd->A[2][j]),
            (fp_sum6_MG_QP_t)fp_mul_MG_QP(M[a][3], sd->A[3][j]),
            (fp_sum6_MG_QP_t)fp_mul_MG_QP(M[a][4], sd->A[4][j]),
            (fp_sum6_MG_QP_t)fp_mul_MG_QP(M[a][5], sd->A[5][j]));
        G[a][j] = fp_shift_right_cast<fp_MG_raw_t>(sum, FP_FRAC_BITS);
      }
    }
    fp_QP_t g06 = (fp_QP_t)MPC_N2_STEER_JERK;
    fp_QP_t g17 = (fp_QP_t)MPC_N2_ACCEL_RATE;
    if (first_stage) {
      g06 = (fp_QP_t)(-MPC_Q2_JERK_CS);
      g17 = (fp_QP_t)(-MPC_Q2_ARATE_CS);
    }
    G[0][6] = fp_MG_raw_from_QP(g06);
    G[1][6] = 0;
    G[0][7] = 0;
    G[1][7] = fp_MG_raw_from_QP(g17);

    for (j = 0; j < nx; j++) {
#pragma HLS PIPELINE II = 1
      fp_sum2_QP_MG_t val0 =
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[0][0], G[0][j]) +
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[0][1], G[1][j]);
      fp_sum2_QP_MG_t val1 =
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[1][0], G[0][j]) +
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[1][1], G[1][j]);
      K[k][0][j] = fp_shift_right_cast<fp_K_raw_t>(-val0, FP_FRAC_BITS);
      K[k][1][j] = fp_shift_right_cast<fp_K_raw_t>(-val1, FP_FRAC_BITS);
    }

    fp_P_raw_t p_shift[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = p_shift complete dim = 1
    for (i = 0; i < nx; i++) {
#pragma HLS PIPELINE II = 1
      fp_sum6_P_QP_t pd_sum = sum6_P_QP_raw(
          (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][0], d0_raw),
          (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][1], d1_raw),
          (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][2], d2_raw),
          (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][3], d3_raw),
          (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][4], d4_raw),
          (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][5], d5_raw));
      p_shift[i] =
          (fp_P_raw_t)((fp_sum2_P_raw_t)p[i] +
                       (fp_sum2_P_raw_t)fp_shift_right_cast<fp_P_raw_t>(pd_sum, FP_FRAC_BITS));
    }

    fp_MG_raw_t Bp[MPC_NU];
#pragma HLS ARRAY_PARTITION variable = Bp complete dim = 1
    {
      const fp_P_raw_t ps2 = p_shift[2];
      const fp_P_raw_t ps3 = p_shift[3];
      const fp_P_raw_t ps4 = p_shift[4];
      const fp_P_raw_t ps5 = p_shift[IDX_DELTA_ACT];
      const fp_P_raw_t ps6 = p_shift[IDX_DELTA_RATE_PREV];
      const fp_P_raw_t ps7 = p_shift[IDX_ACCEL_PREV];

      fp_sum2_P_QP_t bp0 =
          (fp_sum2_P_QP_t)fp_mul_QP_P(b00, ps5) +
          ((fp_sum2_P_QP_t)ps6 << FP_FRAC_BITS);
      Bp[0] = fp_shift_right_cast<fp_MG_raw_t>(bp0, FP_FRAC_BITS);

      fp_sum4_P_QP_t bp1 =
          (fp_sum4_P_QP_t)fp_mul_QP_P(b10, ps2) +
          (fp_sum4_P_QP_t)fp_mul_QP_P(b11, ps3) +
          (fp_sum4_P_QP_t)fp_mul_QP_P(b12, ps4) +
          ((fp_sum4_P_QP_t)ps7 << FP_FRAC_BITS);
      Bp[1] = fp_shift_right_cast<fp_MG_raw_t>(bp1, FP_FRAC_BITS);
    }

    {
      fp_MG_raw_t rhs0 =
          (fp_MG_raw_t)((fp_sum2_MG_raw_t)r_aug_linear[0] + (fp_sum2_MG_raw_t)Bp[0]);
      fp_MG_raw_t rhs1 =
          (fp_MG_raw_t)((fp_sum2_MG_raw_t)r_aug_linear[1] + (fp_sum2_MG_raw_t)Bp[1]);
      fp_sum2_QP_MG_t val0 =
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[0][0], rhs0) +
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[0][1], rhs1);
      fp_sum2_QP_MG_t val1 =
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[1][0], rhs0) +
          (fp_sum2_QP_MG_t)fp_mul_QP_MG(Si[1][1], rhs1);
      kk[k][0] = fp_shift_right_cast<fp_K_raw_t>(-val0, FP_FRAC_BITS);
      kk[k][1] = fp_shift_right_cast<fp_K_raw_t>(-val1, FP_FRAC_BITS);
    }

    fp_P_raw_t PA[MPC_NX_DENSE][MPC_NX_DENSE];
#pragma HLS ARRAY_PARTITION variable = PA complete dim = 1
#pragma HLS BIND_STORAGE variable = PA type = RAM_2P impl = LUTRAM latency = 1

    for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
      PA[i][0] = P[i][0];
    }

    for (i = 0; i < 6; i++) {
      for (j = 1; j < 6; j++) {
#pragma HLS PIPELINE II = 1
        fp_sum6_P_QP_t sum = sum6_P_QP_raw(
            (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][0], sd->A[0][j]),
            (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][1], sd->A[1][j]),
            (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][2], sd->A[2][j]),
            (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][3], sd->A[3][j]),
            (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][4], sd->A[4][j]),
            (fp_sum6_P_QP_t)fp_mul_P_QP(P[i][5], sd->A[5][j]));
        PA[i][j] = fp_shift_right_cast<fp_P_raw_t>(sum, FP_FRAC_BITS);
      }
    }

#pragma HLS ALLOCATION operation instances = mul limit = 32
    {
#define COMPUTE_P_SUM_RAW_TO(II, JJ, DST)                                            \
  do {                                                                               \
    (DST) = sum8_P_MIX_raw_pupdate(                                                  \
        (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[0][(II)], PA[0][(JJ)]),                    \
        (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[1][(II)], PA[1][(JJ)]),                    \
        (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[2][(II)], PA[2][(JJ)]),                    \
        (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[3][(II)], PA[3][(JJ)]),                    \
        (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[4][(II)], PA[4][(JJ)]),                    \
        (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[5][(II)], PA[5][(JJ)]),                    \
        (fp_sum8_P_MIX_t)fp_mul_MG_K(G[0][(II)], K[k][0][(JJ)]),                   \
        (fp_sum8_P_MIX_t)fp_mul_MG_K(G[1][(II)], K[k][1][(JJ)]));                  \
  } while (0)

#define CAST_P_MIX(VALUE)                                                            \
  fp_shift_right_cast<fp_P_raw_t>((VALUE), FP_FRAC_BITS)

      {
        fp_sum8_P_MIX_t p00_raw, p01_raw, p02_raw, p03_raw, p04_raw, p05_raw;
        COMPUTE_P_SUM_RAW_TO(0, 0, p00_raw);
        COMPUTE_P_SUM_RAW_TO(0, 1, p01_raw);
        COMPUTE_P_SUM_RAW_TO(0, 2, p02_raw);
        COMPUTE_P_SUM_RAW_TO(0, 3, p03_raw);
        COMPUTE_P_SUM_RAW_TO(0, 4, p04_raw);
        COMPUTE_P_SUM_RAW_TO(0, 5, p05_raw);
        fp_P_raw_t r0_0 = CAST_P_MIX(p00_raw);
        fp_P_raw_t r0_1 = CAST_P_MIX(p01_raw);
        fp_P_raw_t r0_2 = CAST_P_MIX(p02_raw);
        fp_P_raw_t r0_3 = CAST_P_MIX(p03_raw);
        fp_P_raw_t r0_4 = CAST_P_MIX(p04_raw);
        fp_P_raw_t r0_5 = CAST_P_MIX(p05_raw);
        P[0][0] = q_aug_diag[0] + r0_0;
        P[0][1] = P[1][0] = r0_1;
        P[0][2] = P[2][0] = r0_2;
        P[0][3] = P[3][0] = r0_3;
        P[0][4] = P[4][0] = r0_4;
        P[0][5] = P[5][0] = r0_5;
      }

      {
        fp_sum8_P_MIX_t p11_raw, p12_raw, p13_raw, p14_raw, p15_raw;
        COMPUTE_P_SUM_RAW_TO(1, 1, p11_raw);
        COMPUTE_P_SUM_RAW_TO(1, 2, p12_raw);
        COMPUTE_P_SUM_RAW_TO(1, 3, p13_raw);
        COMPUTE_P_SUM_RAW_TO(1, 4, p14_raw);
        COMPUTE_P_SUM_RAW_TO(1, 5, p15_raw);
        fp_P_raw_t r1_1 = CAST_P_MIX(p11_raw);
        fp_P_raw_t r1_2 = CAST_P_MIX(p12_raw);
        fp_P_raw_t r1_3 = CAST_P_MIX(p13_raw);
        fp_P_raw_t r1_4 = CAST_P_MIX(p14_raw);
        fp_P_raw_t r1_5 = CAST_P_MIX(p15_raw);
        P[1][1] = q_aug_diag[1] + r1_1;
        P[1][2] = P[2][1] = r1_2;
        P[1][3] = P[3][1] = r1_3;
        P[1][4] = P[4][1] = r1_4;
        P[1][5] = P[5][1] = r1_5;
      }

      {
        fp_sum8_P_MIX_t p22_raw, p23_raw, p24_raw, p25_raw;
        COMPUTE_P_SUM_RAW_TO(2, 2, p22_raw);
        COMPUTE_P_SUM_RAW_TO(2, 3, p23_raw);
        COMPUTE_P_SUM_RAW_TO(2, 4, p24_raw);
        COMPUTE_P_SUM_RAW_TO(2, 5, p25_raw);
        fp_P_raw_t r2_2 = CAST_P_MIX(p22_raw);
        fp_P_raw_t r2_3 = CAST_P_MIX(p23_raw);
        fp_P_raw_t r2_4 = CAST_P_MIX(p24_raw);
        fp_P_raw_t r2_5 = CAST_P_MIX(p25_raw);
        P[2][2] = q_aug_diag[2] + r2_2;
        P[2][3] = P[3][2] = r2_3;
        P[2][4] = P[4][2] = r2_4;
        P[2][5] = P[5][2] = r2_5;
      }

      {
        fp_sum8_P_MIX_t p33_raw, p34_raw, p35_raw;
        COMPUTE_P_SUM_RAW_TO(3, 3, p33_raw);
        COMPUTE_P_SUM_RAW_TO(3, 4, p34_raw);
        COMPUTE_P_SUM_RAW_TO(3, 5, p35_raw);
        fp_P_raw_t r3_3 = CAST_P_MIX(p33_raw);
        fp_P_raw_t r3_4 = CAST_P_MIX(p34_raw);
        fp_P_raw_t r3_5 = CAST_P_MIX(p35_raw);
        P[3][3] = q_aug_diag[3] + r3_3;
        P[3][4] = P[4][3] = r3_4;
        P[3][5] = P[5][3] = r3_5;
      }

      {
        fp_sum8_P_MIX_t p44_raw, p45_raw;
        COMPUTE_P_SUM_RAW_TO(4, 4, p44_raw);
        COMPUTE_P_SUM_RAW_TO(4, 5, p45_raw);
        fp_P_raw_t r4_4 = CAST_P_MIX(p44_raw);
        fp_P_raw_t r4_5 = CAST_P_MIX(p45_raw);
        P[4][4] = q_aug_diag[4] + r4_4;
        P[4][5] = P[5][4] = r4_5;
      }

      {
        fp_sum8_P_MIX_t p55_raw;
        COMPUTE_P_SUM_RAW_TO(5, 5, p55_raw);
        fp_P_raw_t r5_5 = CAST_P_MIX(p55_raw);
        P[5][5] = q_aug_diag[5] + r5_5;
      }

#undef CAST_P_MIX
#undef COMPUTE_P_SUM_RAW_TO
    }

    {
      fp_P_raw_t gtk_i6[6], gtk_i7[6];
      fp_P_raw_t gtk_66 = 0, gtk_67 = 0, gtk_77 = 0;
#pragma HLS ARRAY_PARTITION variable = gtk_i6 complete dim = 1
#pragma HLS ARRAY_PARTITION variable = gtk_i7 complete dim = 1

      for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
        fp_sum2_MG_K_t s6 =
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[0][i], K[k][0][6]) +
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[1][i], K[k][1][6]);
        fp_sum2_MG_K_t s7 =
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[0][i], K[k][0][7]) +
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[1][i], K[k][1][7]);

        gtk_i6[i] = fp_shift_right_cast<fp_P_raw_t>(s6, FP_FRAC_BITS);
        gtk_i7[i] = fp_shift_right_cast<fp_P_raw_t>(s7, FP_FRAC_BITS);
      }

      {
        fp_sum2_MG_K_t s66 =
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[0][6], K[k][0][6]);
        fp_sum2_MG_K_t s67 =
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[0][6], K[k][0][7]);
        fp_sum2_MG_K_t s77 =
            (fp_sum2_MG_K_t)fp_mul_MG_K(G[1][7], K[k][1][7]);

        gtk_66 = fp_shift_right_cast<fp_P_raw_t>(s66, FP_FRAC_BITS);
        gtk_67 = fp_shift_right_cast<fp_P_raw_t>(s67, FP_FRAC_BITS);
        gtk_77 = fp_shift_right_cast<fp_P_raw_t>(s77, FP_FRAC_BITS);
      }

      for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
        P[i][6] = gtk_i6[i];
        P[6][i] = gtk_i6[i];
        P[i][7] = gtk_i7[i];
        P[7][i] = gtk_i7[i];
      }
      P[6][6] = q_aug_diag[6] + gtk_66;
      P[7][7] = q_aug_diag[7] + gtk_77;
      P[6][7] = gtk_67;
      P[7][6] = gtk_67;
    }

    fp_P_raw_t p_new[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = p_new complete dim = 1
    for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II = 1
      fp_sum8_P_MIX_t total = sum8_P_MIX_raw(
          (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[0][i], p_shift[0]),
          (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[1][i], p_shift[1]),
          (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[2][i], p_shift[2]),
          (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[3][i], p_shift[3]),
          (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[4][i], p_shift[4]),
          (fp_sum6_P_QP_t)fp_mul_QP_P(sd->A[5][i], p_shift[5]),
          (fp_sum8_P_MIX_t)fp_mul_MG_K(G[0][i], kk[k][0]),
          (fp_sum8_P_MIX_t)fp_mul_MG_K(G[1][i], kk[k][1]));
      p_new[i] =
          (fp_P_raw_t)((fp_sum2_P_raw_t)q_aug_linear[i] +
                       (fp_sum2_P_raw_t)fp_shift_right_cast<fp_P_raw_t>(total, FP_FRAC_BITS));
    }

    for (i = 6; i < nx; i++) {
#pragma HLS PIPELINE II = 1
      fp_sum2_MG_K_t total =
          (fp_sum2_MG_K_t)fp_mul_MG_K(G[0][i], kk[k][0]) +
          (fp_sum2_MG_K_t)fp_mul_MG_K(G[1][i], kk[k][1]);
      p_new[i] =
          (fp_P_raw_t)((fp_sum2_P_raw_t)q_aug_linear[i] +
                       (fp_sum2_P_raw_t)fp_shift_right_cast<fp_P_raw_t>(total, FP_FRAC_BITS));
    }

    for (i = 0; i < nx; i++) {
#pragma HLS UNROLL
      p[i] = p_new[i];
    }
  }
}

/* Thin wrapper: declares the K/kk handoff arrays and chains the two
 * extracted modules. Signature unchanged so the riccati_admm_solve_hls
 * call site is untouched. K/kk are whole arrays partitioned identically
 * to the sub-function params (proven-safe interface). */
static void
riccati_pass_hls(const StepData_t step_data[MPC_HORIZON],
                 const fp_QP_raw_t B_sparse[MPC_HORIZON][MPC_BSP_N],
                 const fp_QP_t terminal_q_diag[MPC_NX_AUG],
                 const fp_QP_t terminal_q_linear[MPC_NX_AUG],
                 const fp_QP_t x0[MPC_NX_AUG], fp_QP_t rho, fp_QP_t rho_u,
                 const fp_QP_t z_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG],
                 const fp_QP_t y_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG],
                 const fp_QP_t z_u[MPC_HORIZON][MPC_NU],
                 const fp_QP_t y_u[MPC_HORIZON][MPC_NU],
                 fp_QP_t x_out[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG],
                 fp_QP_t u_out[MPC_HORIZON][MPC_NU]) {
#pragma HLS INLINE off
#pragma HLS ARRAY_PARTITION variable = B_sparse complete dim = 2
  fp_K_raw_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG];
  fp_K_raw_t kk[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable = K complete dim = 2
#pragma HLS ARRAY_PARTITION variable = K complete dim = 3
#pragma HLS ARRAY_PARTITION variable = kk complete dim = 2

  riccati_backward_pass(step_data, B_sparse, terminal_q_diag,
                        terminal_q_linear, rho, rho_u, z_x, y_x, z_u, y_u, K,
                        kk);
  riccati_forward_pass(step_data, B_sparse, x0, K, kk, x_out, u_out);
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

MpcStatus_t riccati_admm_solve_hls(const StepData_t step_data[MPC_HORIZON],
                                   const fp_QP_raw_t B_sparse[MPC_HORIZON]
                                                 [MPC_BSP_N],
                                   const fp_QP_t terminal_q_diag[MPC_NX_AUG],
                                   const fp_QP_t terminal_q_linear[MPC_NX_AUG],
                                   const fp_QP_t terminal_x_lb[MPC_NX_AUG],
                                   const fp_QP_t terminal_x_ub[MPC_NX_AUG],
                                   const fp_QP_t x0[MPC_NX_AUG],
                                   const AdmmConfig_t *config,
                                   AdmmState_t *admm_state,
                                   MpcSolution_t *solution) {
#pragma HLS INLINE
#pragma HLS ARRAY_PARTITION variable = B_sparse complete dim = 2
#pragma HLS DISAGGREGATE variable = admm_state
#ifndef __SYNTHESIS__
  g_hls_debug_trace_count = 0;
#endif

#pragma HLS ALLOCATION function instances = riccati_pass_hls limit = 1

  /* Runtime config is authoritative on each solve invocation. */
  const fp_QP_t cfg_rho =
      (config && config->rho > FP_QP_CONST(0.0))
          ? config->rho
          : FP_QP_CONST(ADMM_RHO_MIN);

  const fp_QP_t cfg_rho_u =
      (config && config->rho_u > FP_QP_CONST(0.0))
          ? config->rho_u
          : cfg_rho;

  fp_QP_t rho =
      (admm_state->initialized && admm_state->rho > FP_QP_CONST(0.0))
          ? admm_state->rho
          : cfg_rho;

  fp_QP_t rho_u =
      (admm_state->initialized && admm_state->rho_u > FP_QP_CONST(0.0))
          ? admm_state->rho_u
          : cfg_rho_u;

  if (rho < FP_QP_CONST(ADMM_RHO_MIN))
    rho = FP_QP_CONST(ADMM_RHO_MIN);
  if (rho_u < FP_QP_CONST(ADMM_RHO_MIN))
    rho_u = FP_QP_CONST(ADMM_RHO_MIN);
  if (rho > FP_QP_CONST(ADMM_RHO_MAX))
    rho = FP_QP_CONST(ADMM_RHO_MAX);
  if (rho_u > FP_QP_CONST(ADMM_RHO_MAX))
    rho_u = FP_QP_CONST(ADMM_RHO_MAX);
    
  int max_iter = config->max_iterations;
  fp_QP_t abs_tolerance = config->tolerance;
  const fp_QP_t rel_tolerance = FP_QP_CONST(0.02);

  /* Local ADMM variables */
  fp_QP_t z_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG];
  fp_QP_t z_u[MPC_HORIZON][MPC_NU];
  fp_QP_t y_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG];
  fp_QP_t y_u[MPC_HORIZON][MPC_NU];
  fp_QP_t sol_x[MPC_HORIZON_PLUS_ONE][MPC_NX_AUG];
  fp_QP_t sol_u[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable = z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = sol_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = sol_u complete dim = 2
  /* LUTRAM binding on the three big state buffers. Measured from
   * 2026-05-15 22:09 routed report: 6 of the top 15 failing paths were
   * y_x BRAM CLKARDCLK -> icmp_ln160 comparators in the ADMM state
   * update, plus sol_x -> z_x BRAM DINADIN/DINBDIN setup. LUTRAM has
   * no BRAM-internal clock-to-out delay so these paths simplify to
   * pure logic + routing. The control buffers (z_u, y_u, sol_u) are
   * 40 entries each and not in the failing-path list, so leave them
   * alone. */
#pragma HLS BIND_STORAGE variable = z_x type = RAM_2P impl = LUTRAM latency = 1
#pragma HLS BIND_STORAGE variable = y_x type = RAM_2P impl = LUTRAM latency = 1
#pragma HLS BIND_STORAGE variable = sol_x type = RAM_2P impl = LUTRAM latency = 1

  int k, s, a, i;

  const bool cold_start = !admm_state->initialized;

  /* CPU parity warm-start:
   * - cold start: zero all ADMM buffers
   * - warm start: reuse full persisted z/y buffers directly (no horizon shift)
   *
   * Keep the outer k-loop rolled. Unrolling it by 2 forces even/odd banking
   * on the ADMM buffers, which adds a parity-select mux on every hot read in
   * riccati_pass_hls.
   */
  if (cold_start) {
    for (k = 0; k < MPC_HORIZON; k++) {
      for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
        z_x[k][s] = 0;
        y_x[k][s] = 0;
      }
    }
    for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
      z_x[MPC_HORIZON][s] = 0;
      y_x[MPC_HORIZON][s] = 0;
    }
    for (k = 0; k < MPC_HORIZON; k++) {
      for (a = 0; a < MPC_NU; a++) {
#pragma HLS UNROLL
        z_u[k][a] = 0;
        y_u[k][a] = 0;
      }
    }
  } else {
    for (k = 0; k < MPC_HORIZON; k++) {
      for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
        z_x[k][s] = admm_state->z_x[k][s];
        y_x[k][s] = admm_state->y_x[k][s];
      }
    }
    for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
      z_x[MPC_HORIZON][s] = admm_state->z_x[MPC_HORIZON][s];
      y_x[MPC_HORIZON][s] = admm_state->y_x[MPC_HORIZON][s];
    }
    for (k = 0; k < MPC_HORIZON; k++) {
      for (a = 0; a < MPC_NU; a++) {
#pragma HLS UNROLL
        z_u[k][a] = admm_state->z_u[k][a];
        y_u[k][a] = admm_state->y_u[k][a];
      }
    }
  }

  MpcStatus_t status = MPC_STATUS_MAX_ITER;
  solution->iterations = 0;
  fp_QP_t final_primal_residual = 0;
  fp_QP_t final_dual_residual = 0;
  const int total_passes = max_iter + (cold_start ? 1 : 0);
  int iter;

  for (iter = 0; iter < total_passes; iter++) {
#pragma HLS LOOP_TRIPCOUNT min = 1 max = MPC_MAX_ADMM_PASS_COUNT avg = 5
    const bool bootstrap_pass = cold_start && (iter == 0);
    const int admm_iter = cold_start ? (iter - 1) : iter;
    const fp_QP_t pass_rho = bootstrap_pass ? (fp_QP_t)0 : rho;
    const fp_QP_t pass_rho_u = bootstrap_pass ? (fp_QP_t)0 : rho_u;

    /* Shared call site prevents HLS from cloning the Riccati datapath for
     * cold-start and ADMM passes. */
    riccati_pass_hls(step_data, B_sparse, terminal_q_diag, terminal_q_linear,
                     x0, pass_rho, pass_rho_u, z_x, y_x, z_u, y_u, sol_x,
                     sol_u);

    if (bootstrap_pass) {
      /* Initialize z from projection of unconstrained solution. */
      for (k = 0; k < MPC_HORIZON_PLUS_ONE; k++) {
        for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
          z_x[k][s] = sol_x[k][s];
        }
        {
          const int idx = IDX_EY;
          fp_QP_t val = sol_x[k][idx];
          fp_QP_t xlb = terminal_x_lb[idx];
          fp_QP_t xub = terminal_x_ub[idx];
          if (k < MPC_HORIZON) {
            xlb = step_ey_lb(&step_data[k]);
            xub = step_ey_ub(&step_data[k]);
          }
          if (val < xlb)
            val = xlb;
          if (val > xub)
            val = xub;
          z_x[k][idx] = val;
        }
        {
          const int idx = IDX_DELTA_ACT;
          fp_QP_t val = sol_x[k][idx];
          fp_QP_t xlb = terminal_x_lb[idx];
          fp_QP_t xub = terminal_x_ub[idx];
          if (k < MPC_HORIZON) {
            xlb = (fp_QP_t)(-VP_MAX_STEER);
            xub = (fp_QP_t)(VP_MAX_STEER);
          }
          if (val < xlb)
            val = xlb;
          if (val > xub)
            val = xub;
          z_x[k][idx] = val;
        }
      }
      for (k = 0; k < MPC_HORIZON; k++) {
        for (a = 0; a < MPC_NU; a++) {
#pragma HLS UNROLL
          fp_QP_t val = sol_u[k][a];
          fp_QP_t lb = VP_MIN_ACCEL;
          fp_QP_t ub = step_accel_ub(&step_data[k]);
          if (a == 0) {
            lb = (fp_QP_t)(-VP_MAX_STEER_RATE);
            ub = (fp_QP_t)(VP_MAX_STEER_RATE);
          }
          if (val < lb)
            val = lb;
          if (val > ub)
            val = ub;
          z_u[k][a] = val;
        }
      }

      /* Initialize y (dual) from constraint violation. */
      for (k = 0; k < MPC_HORIZON_PLUS_ONE; k++) {
        y_x[k][IDX_EY] = sol_x[k][IDX_EY] - z_x[k][IDX_EY];
        y_x[k][IDX_DELTA_ACT] = sol_x[k][IDX_DELTA_ACT] - z_x[k][IDX_DELTA_ACT];
      }
      for (k = 0; k < MPC_HORIZON; k++) {
        for (a = 0; a < MPC_NU; a++) {
#pragma HLS UNROLL
          y_u[k][a] = sol_u[k][a] - z_u[k][a];
        }
      }
      #ifndef __SYNTHESIS__
      hls_debug_trace_push(-1, 0, 0, 0, 0, 0, 0, rho, rho_u, sol_u[0][0],
                           sol_u[0][1], z_u[0][0], z_u[0][1], y_u[0][0],
                           y_u[0][1]);
      #endif
      continue;
    }

    /* Compute primal scaling norms inside existing z/y loops. */
    fp_QP_t x_norm = 0;
    fp_QP_t u_norm = 0;

    /* --- Fused z-update, y-update, and residual computation ---
     * Dual residual uses rho*(z_new - z_old), where z_old is read
     * before writing z_new in each component. */
    fp_QP_t state_primal = 0, state_dual = 0;
    fp_QP_t ctrl_primal = 0, ctrl_dual = 0;
    fp_QP_t z_norm = 0, lambda_norm = 0;

    /* Per-channel accumulators remove false dependencies between channels. */
    fp_QP_t primal_ey = 0, dual_ey = 0, znorm_ey = 0, lnorm_ey = 0;
    fp_QP_t primal_da = 0, dual_da = 0, znorm_da = 0, lnorm_da = 0;
    fp_QP_t primal_u0 = 0, dual_u0 = 0, znorm_u0 = 0, lnorm_u0 = 0;
    fp_QP_t primal_u1 = 0, dual_u1 = 0, znorm_u1 = 0, lnorm_u1 = 0;

    /* State z/y update */
    for (k = 0; k < MPC_HORIZON_PLUS_ONE; k++) {
      /* QP assembly only gives finite bounds to e_y and delta_actual.
       * Keep unconstrained states out of the expensive ADMM projection path. */
      for (s = 0; s < MPC_NX_AUG; s++) {
#pragma HLS UNROLL
        if (s != IDX_EY && s != IDX_DELTA_ACT) {
          z_x[k][s] = sol_x[k][s];
        }
      }

      fp_QP_t x_norm_k =
          fp_max_abs_state8(sol_x[k][0], sol_x[k][1], sol_x[k][2], sol_x[k][3],
                         sol_x[k][4], sol_x[k][5], sol_x[k][6], sol_x[k][7]);
      if (x_norm_k > x_norm)
        x_norm = x_norm_k;

      {
        const int idx = IDX_EY;
        fp_QP_t ey_lb = terminal_x_lb[idx];
        fp_QP_t ey_ub = terminal_x_ub[idx];
        if (k < MPC_HORIZON) {
          ey_lb = step_ey_lb(&step_data[k]);
          ey_ub = step_ey_ub(&step_data[k]);
        }
        admm_update_state_channel_raw(
            sol_x[k][idx], &z_x[k][idx], &y_x[k][idx], rho,
            ey_lb, ey_ub,
            &primal_ey,
            &dual_ey, &znorm_ey, &lnorm_ey);
      }

      {
        const int idx = IDX_DELTA_ACT;
        fp_QP_t delta_lb = terminal_x_lb[idx];
        fp_QP_t delta_ub = terminal_x_ub[idx];
        if (k < MPC_HORIZON) {
          delta_lb = (fp_QP_t)(-VP_MAX_STEER);
          delta_ub = (fp_QP_t)(VP_MAX_STEER);
        }
        admm_update_state_channel_raw(
            sol_x[k][idx], &z_x[k][idx], &y_x[k][idx], rho,
            delta_lb, delta_ub,
            &primal_da,
            &dual_da, &znorm_da, &lnorm_da);
      }
    }

    state_primal = (primal_ey > primal_da) ? primal_ey : primal_da;
    state_dual = (dual_ey > dual_da) ? dual_ey : dual_da;
    {
      fp_QP_t z_norm_state = (znorm_ey > znorm_da) ? znorm_ey : znorm_da;
      fp_QP_t lnorm_state = (lnorm_ey > lnorm_da) ? lnorm_ey : lnorm_da;
      if (z_norm_state > z_norm)
        z_norm = z_norm_state;
      if (lnorm_state > lambda_norm)
        lambda_norm = lnorm_state;
    }

    /* Control z/y update — dual residual computed inline */
    for (k = 0; k < MPC_HORIZON; k++) {
      const StepData_t *sd = &step_data[k];

      fp_QP_t u_norm_k = fp_max_abs_ctrl2(sol_u[k][0], sol_u[k][1]);
      if (u_norm_k > u_norm)
        u_norm = u_norm_k;

      admm_update_control_channel_raw(
          sol_u[k][0], &z_u[k][0], &y_u[k][0], rho_u,
          (fp_QP_t)(-VP_MAX_STEER_RATE), (fp_QP_t)(VP_MAX_STEER_RATE),
          &primal_u0, &dual_u0, &znorm_u0, &lnorm_u0);

      admm_update_control_channel_raw(
          sol_u[k][1], &z_u[k][1], &y_u[k][1], rho_u,
          VP_MIN_ACCEL, step_accel_ub(sd),
          &primal_u1, &dual_u1, &znorm_u1, &lnorm_u1);
    }

    ctrl_primal = (primal_u0 > primal_u1) ? primal_u0 : primal_u1;
    ctrl_dual = (dual_u0 > dual_u1) ? dual_u0 : dual_u1;
    {
      fp_QP_t z_norm_ctrl = (znorm_u0 > znorm_u1) ? znorm_u0 : znorm_u1;
      fp_QP_t lnorm_ctrl = (lnorm_u0 > lnorm_u1) ? lnorm_u0 : lnorm_u1;
      if (z_norm_ctrl > z_norm)
        z_norm = z_norm_ctrl;
      if (lnorm_ctrl > lambda_norm)
        lambda_norm = lnorm_ctrl;
    }
    fp_QP_t primal_res =
        state_primal > ctrl_primal ? state_primal : ctrl_primal;
    fp_QP_t dual_res = state_dual > ctrl_dual ? state_dual : ctrl_dual;
    fp_QP_t max_norm = x_norm;
    if (u_norm > max_norm)
      max_norm = u_norm;
    if (z_norm > max_norm)
      max_norm = z_norm;
    fp_QP_t eps_primal =
        abs_tolerance + fp_mul_site(rel_tolerance, max_norm,
                                    FP_CAST_SITE_MUL_RS_EPS_PRIMAL_REL);
    fp_QP_t eps_dual =
        abs_tolerance + fp_mul_site(rel_tolerance, lambda_norm,
                                    FP_CAST_SITE_MUL_RS_EPS_DUAL_REL);

    final_primal_residual = primal_res;
    final_dual_residual = dual_res;
    #ifndef __SYNTHESIS__
    hls_debug_trace_push(admm_iter, primal_res, dual_res, state_primal,
                         state_dual, ctrl_primal, ctrl_dual, rho, rho_u,
                         sol_u[0][0], sol_u[0][1], z_u[0][0], z_u[0][1],
                         y_u[0][0], y_u[0][1]);
    const int trace_idx = g_hls_debug_trace_count - 1;
    #endif

    /* Convergence check */
    if (primal_res <= eps_primal && dual_res <= eps_dual) {
      status = MPC_STATUS_OPTIMAL;
      break;
    }

    /* Adaptive rho: rebalance state and control channels independently. */
    if (config->adaptive_rho && admm_iter > 0) {
      const fp_QP_t adapt_ratio_state = FP_QP_CONST(2.0);
      const fp_QP_t adapt_ratio_ctrl = FP_QP_CONST(2.0);

      /* Hoist multiplications so the scheduler can overlap them. */
      const fp_QP_t state_dual_scaled =
          fp_mul_site(adapt_ratio_state, state_dual,
                      FP_CAST_SITE_MUL_RS_ADAPT_STATE_DUAL);
      const fp_QP_t state_primal_scaled =
          fp_mul_site(adapt_ratio_state, state_primal,
                      FP_CAST_SITE_MUL_RS_ADAPT_STATE_PRIMAL);
      const fp_QP_t ctrl_dual_scaled = fp_mul_site(
          adapt_ratio_ctrl, ctrl_dual, FP_CAST_SITE_MUL_RS_ADAPT_CTRL_DUAL);
      const fp_QP_t ctrl_primal_scaled = fp_mul_site(
          adapt_ratio_ctrl, ctrl_primal, FP_CAST_SITE_MUL_RS_ADAPT_CTRL_PRIMAL);

      int scale_rho = 0;
      int scale_rho_u = 0;

      if (state_primal > state_dual_scaled && rho <= FP_QP_CONST(ADMM_RHO_MAX))
        scale_rho = 1;
      else if (state_dual > state_primal_scaled &&
               rho >= FP_QP_CONST(ADMM_RHO_MIN))
        scale_rho = -1;

      if (ctrl_primal > ctrl_dual_scaled && rho_u <= FP_QP_CONST(ADMM_RHO_MAX))
        scale_rho_u = 1;
      else if (ctrl_dual > ctrl_primal_scaled &&
               rho_u >= FP_QP_CONST(ADMM_RHO_MIN))
        scale_rho_u = -1;

      /* Only enter loops if at least one penalty needs updating. */
      if (scale_rho != 0 || scale_rho_u != 0) {
        #ifndef __SYNTHESIS__
        if (trace_idx >= 0 && trace_idx < g_hls_debug_trace_count) {
          g_hls_debug_trace[trace_idx].scale_rho = scale_rho;
          g_hls_debug_trace[trace_idx].scale_rho_u = scale_rho_u;
        }
        #endif

        /* --- State penalty update ---------------------------------------- */
        if (scale_rho != 0) {
          if (scale_rho > 0) {
            rho <<= 1;
            rho = (rho > FP_QP_CONST(ADMM_RHO_MAX)) ? FP_QP_CONST(ADMM_RHO_MAX)
                                                    : rho;
          } else {
            rho >>= 1;
            rho = (rho < FP_QP_CONST(ADMM_RHO_MIN)) ? FP_QP_CONST(ADMM_RHO_MIN)
                                                    : rho;
          }
        }

        /* --- Control penalty update --------------------------------------- */
        if (scale_rho_u != 0) {
          if (scale_rho_u > 0) {
            rho_u <<= 1;
            rho_u = (rho_u > FP_QP_CONST(ADMM_RHO_MAX))
                        ? FP_QP_CONST(ADMM_RHO_MAX)
                        : rho_u;
          } else {
            rho_u >>= 1;
            rho_u = (rho_u < FP_QP_CONST(ADMM_RHO_MIN))
                        ? FP_QP_CONST(ADMM_RHO_MIN)
                        : rho_u;
          }
        }

        /* --- Dual variable rescaling -------------- */

        /* State duals: single merged loop, direction resolved statically. */
        if (scale_rho != 0) {
          for (k = 0; k < MPC_HORIZON_PLUS_ONE; k++) {
#pragma HLS PIPELINE II = 1
            if (scale_rho > 0) {
              y_x[k][IDX_EY] >>= 1;
              y_x[k][IDX_DELTA_ACT] >>= 1;
            } else {
              y_x[k][IDX_EY] <<= 1;
              y_x[k][IDX_DELTA_ACT] <<= 1;
            }
          }
        }

        /* Control duals: single merged loop, direction resolved statically. */
        if (scale_rho_u != 0) {
          for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II = 1
            for (a = 0; a < MPC_NU; a++) {
              if (scale_rho_u > 0) {
                y_u[k][a] >>= 1;
              } else {
                y_u[k][a] <<= 1;
              }
            }
          }
        }
      }
    }
  } /* end ADMM loop */

  const int completed_admm_iters =
      cold_start ? ((iter < total_passes) ? iter : (total_passes - 1))
                 : ((iter < total_passes) ? (iter + 1) : total_passes);
  solution->iterations = completed_admm_iters;
  solution->primal_residual = final_primal_residual;
  solution->dual_residual = final_dual_residual;

  /* Save ADMM state for warm-starting.
   * Policy:
   *  - OPTIMAL or MAX_ITER : keep the full warm start (primal z_*, dual y_*,
   *                          and the live rho/rho_u)
   *  - any other status    : clear the warm start (zero everything, rho = 0,
   *                          initialized = 0)
   */
  if (status == MPC_STATUS_OPTIMAL || status == MPC_STATUS_MAX_ITER) {
    for (k = 0; k < MPC_HORIZON_PLUS_ONE; k++) {
  #pragma HLS PIPELINE II = 1
      for (i = 0; i < MPC_NX_AUG; i++) {
        admm_state->z_x[k][i] = z_x[k][i];
        admm_state->y_x[k][i] = y_x[k][i];
      }
    }
    for (k = 0; k < MPC_HORIZON; k++) {
  #pragma HLS PIPELINE II = 1
      for (a = 0; a < MPC_NU; a++) {
        admm_state->z_u[k][a] = z_u[k][a];
        admm_state->y_u[k][a] = y_u[k][a];
      }
    }
    admm_state->rho = rho;
    admm_state->rho_u = rho_u;
    admm_state->initialized = 1;
  } else {
    for (k = 0; k < MPC_HORIZON_PLUS_ONE; k++) {
  #pragma HLS PIPELINE II = 1
      for (i = 0; i < MPC_NX_AUG; i++) {
        admm_state->z_x[k][i] = FP_QP_CONST(0.0);
        admm_state->y_x[k][i] = FP_QP_CONST(0.0);
      }
    }
    for (k = 0; k < MPC_HORIZON; k++) {
  #pragma HLS PIPELINE II = 1
      for (a = 0; a < MPC_NU; a++) {
        admm_state->z_u[k][a] = FP_QP_CONST(0.0);
        admm_state->y_u[k][a] = FP_QP_CONST(0.0);
      }
    }
    admm_state->rho = FP_QP_CONST(0.0);
    admm_state->rho_u = FP_QP_CONST(0.0);
    admm_state->initialized = 0;
  }

  solution->status = status;
  return status;
}
