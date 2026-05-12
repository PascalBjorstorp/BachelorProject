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

static void sanitize_warm_start_from_primal(
    const StepData_t step_data[MPC_HORIZON],
    const fp_QP_t terminal_x_lb[MPC_NX_AUG],
    const fp_QP_t terminal_x_ub[MPC_NX_AUG],
    const fp_QP_t sol_x[MPC_HORIZON + 1][MPC_NX_AUG],
    const fp_QP_t sol_u[MPC_HORIZON][MPC_NU],
    fp_QP_t z_x[MPC_HORIZON + 1][MPC_NX_AUG],
    fp_QP_t z_u[MPC_HORIZON][MPC_NU],
    fp_QP_t y_x[MPC_HORIZON + 1][MPC_NX_AUG],
    fp_QP_t y_u[MPC_HORIZON][MPC_NU]) {
#pragma HLS INLINE off
  int k, s, a;
  for (k = 0; k <= MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (s = 0; s < MPC_NX_AUG; ++s) {
      fp_QP_t v = sol_x[k][s];
      if (s == IDX_EY || s == IDX_DELTA_ACT) {
        const fp_QP_t lb =
            (k < MPC_HORIZON) ? step_data[k].x_lb[s] : terminal_x_lb[s];
        const fp_QP_t ub =
            (k < MPC_HORIZON) ? step_data[k].x_ub[s] : terminal_x_ub[s];
        if (v < lb)
          v = lb;
        if (v > ub)
          v = ub;
      }
      z_x[k][s] = v;
      y_x[k][s] = FP_QP_CONST(0.0);
    }
  }
  for (k = 0; k < MPC_HORIZON; ++k) {
#pragma HLS PIPELINE II = 1
    for (a = 0; a < MPC_NU; ++a) {
      fp_QP_t v = sol_u[k][a];
      if (v < step_data[k].u_lb[a])
        v = step_data[k].u_lb[a];
      if (v > step_data[k].u_ub[a])
        v = step_data[k].u_ub[a];
      z_u[k][a] = v;
      y_u[k][a] = FP_QP_CONST(0.0);
    }
  }
}

static fp_raw_acc_t shift_right_clip_sum_acc_QP_to_acc(fp_sum6_acc_QP_t value, int shift) {
#pragma HLS INLINE
  fp_sum6_acc_QP_t shifted = value >> shift;
  const fp_sum6_acc_QP_t max_acc = (fp_sum6_acc_QP_t)fp_raw_acc_max();
  const fp_sum6_acc_QP_t min_acc = (fp_sum6_acc_QP_t)fp_raw_acc_min();
  if (shifted > max_acc)
    return fp_raw_acc_max();
  if (shifted < min_acc)
    return fp_raw_acc_min();
  return (fp_raw_acc_t)shifted;
}

static fp_raw_acc_t shift_right_clip_sum_QP_QP_to_acc(fp_sum6_QP_mul_t value, int shift) {
#pragma HLS INLINE
  fp_sum6_QP_mul_t shifted = value >> shift;
  const fp_sum6_QP_mul_t max_acc = (fp_sum6_QP_mul_t)fp_raw_acc_max();
  const fp_sum6_QP_mul_t min_acc = (fp_sum6_QP_mul_t)fp_raw_acc_min();
  if (shifted > max_acc)
    return fp_raw_acc_max();
  if (shifted < min_acc)
    return fp_raw_acc_min();
  return (fp_raw_acc_t)shifted;
}

static fp_QP_raw_t shift_right_clip_sum_acc_QP_to_qp(fp_sum6_acc_QP_t value, int shift) {
#pragma HLS INLINE
  fp_sum6_acc_QP_t shifted = value >> shift;
  const fp_sum6_acc_QP_t max_qp = (fp_sum6_acc_QP_t)fp_qp_raw_max_acc();
  const fp_sum6_acc_QP_t min_qp = (fp_sum6_acc_QP_t)fp_qp_raw_min_acc();
  if (shifted > max_qp)
    return (fp_QP_raw_t)fp_qp_raw_max_acc();
  if (shifted < min_qp)
    return (fp_QP_raw_t)fp_qp_raw_min_acc();
  return (fp_QP_raw_t)shifted;
}

static fp_QP_raw_t shift_right_clip_sum_QP_QP_to_qp(fp_sum6_QP_mul_t value, int shift) {
#pragma HLS INLINE
  fp_sum6_QP_mul_t shifted = value >> shift;
  const fp_sum6_QP_mul_t max_qp = (fp_sum6_QP_mul_t)fp_qp_raw_max_acc();
  const fp_sum6_QP_mul_t min_qp = (fp_sum6_QP_mul_t)fp_qp_raw_min_acc();
  if (shifted > max_qp)
    return (fp_QP_raw_t)fp_qp_raw_max_acc();
  if (shifted < min_qp)
    return (fp_QP_raw_t)fp_qp_raw_min_acc();
  return (fp_QP_raw_t)shifted;
}

static fp_sum6_acc_QP_t sum6_acc_QP_raw(fp_sum6_acc_QP_t a0,
                                        fp_sum6_acc_QP_t a1,
                                        fp_sum6_acc_QP_t a2,
                                        fp_sum6_acc_QP_t a3,
                                        fp_sum6_acc_QP_t a4,
                                        fp_sum6_acc_QP_t a5) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_sum6_acc_QP_t s01 = a0 + a1;
  fp_sum6_acc_QP_t s23 = a2 + a3;
  fp_sum6_acc_QP_t s45 = a4 + a5;
  fp_sum6_acc_QP_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum6_QP_mul_t sum6_QP_raw(fp_sum6_QP_mul_t a0,
                                    fp_sum6_QP_mul_t a1,
                                    fp_sum6_QP_mul_t a2,
                                    fp_sum6_QP_mul_t a3,
                                    fp_sum6_QP_mul_t a4,
                                    fp_sum6_QP_mul_t a5) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_sum6_QP_mul_t s01 = a0 + a1;
  fp_sum6_QP_mul_t s23 = a2 + a3;
  fp_sum6_QP_mul_t s45 = a4 + a5;
  fp_sum6_QP_mul_t s0123 = s01 + s23;
  return s0123 + s45;
}

static fp_sum6_acc_QP_t sum8_acc_QP_raw(fp_sum6_acc_QP_t a0,
                                        fp_sum6_acc_QP_t a1,
                                        fp_sum6_acc_QP_t a2,
                                        fp_sum6_acc_QP_t a3,
                                        fp_sum6_acc_QP_t a4,
                                        fp_sum6_acc_QP_t a5,
                                        fp_sum6_acc_QP_t a6,
                                        fp_sum6_acc_QP_t a7) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_sum6_acc_QP_t s01 = a0 + a1;
  fp_sum6_acc_QP_t s23 = a2 + a3;
  fp_sum6_acc_QP_t s45 = a4 + a5;
  fp_sum6_acc_QP_t s67 = a6 + a7;
  fp_sum6_acc_QP_t s0123 = s01 + s23;
  fp_sum6_acc_QP_t s4567 = s45 + s67;
  return s0123 + s4567;
}

static fp_sum6_QP_mul_t sum8_QP_raw(fp_sum6_QP_mul_t a0,
                                    fp_sum6_QP_mul_t a1,
                                    fp_sum6_QP_mul_t a2,
                                    fp_sum6_QP_mul_t a3,
                                    fp_sum6_QP_mul_t a4,
                                    fp_sum6_QP_mul_t a5,
                                    fp_sum6_QP_mul_t a6,
                                    fp_sum6_QP_mul_t a7) {
#pragma HLS INLINE off
#pragma HLS PIPELINE II = 1
  fp_sum6_QP_mul_t s01 = a0 + a1;
  fp_sum6_QP_mul_t s23 = a2 + a3;
  fp_sum6_QP_mul_t s45 = a4 + a5;
  fp_sum6_QP_mul_t s67 = a6 + a7;
  fp_sum6_QP_mul_t s0123 = s01 + s23;
  fp_sum6_QP_mul_t s4567 = s45 + s67;
  return s0123 + s4567;
}

static fp_QP_t fp_max2(fp_QP_t a, fp_QP_t b) {
#pragma HLS INLINE
  return (a > b) ? a : b;
}

static fp_QP_t max_abs_state8(fp_QP_t x0, fp_QP_t x1, fp_QP_t x2, fp_QP_t x3,
                              fp_QP_t x4, fp_QP_t x5, fp_QP_t x6, fp_QP_t x7) {
#pragma HLS INLINE
  fp_QP_t m0 = fp_max2(fp_abs(x0), fp_abs(x1));
  fp_QP_t m1 = fp_max2(fp_abs(x2), fp_abs(x3));
  fp_QP_t m2 = fp_max2(fp_abs(x4), fp_abs(x5));
  fp_QP_t m3 = fp_max2(fp_abs(x6), fp_abs(x7));
  fp_QP_t m4 = fp_max2(m0, m1);
  fp_QP_t m5 = fp_max2(m2, m3);
  return fp_max2(m4, m5);
}

static fp_QP_t max_abs_ctrl2(fp_QP_t x0, fp_QP_t x1) {
#pragma HLS INLINE
  return fp_max2(fp_abs(x0), fp_abs(x1));
}

static void
admm_update_state_channel_raw(fp_QP_t x_val, fp_QP_t *z_slot, fp_QP_t *y_slot,
                              fp_QP_t rho, fp_QP_t lb, fp_QP_t ub,
                              fp_QP_t *state_primal, fp_QP_t *state_dual,
                              fp_QP_t *z_norm_k, fp_QP_t *lambda_norm_k) {
#pragma HLS INLINE
  fp_raw_acc_t val = fp_raw_acc_from_qp(x_val) + fp_raw_acc_from_qp(*y_slot);
  if (val < fp_raw_acc_from_qp(lb))
    val = fp_raw_acc_from_qp(lb);
  if (val > fp_raw_acc_from_qp(ub))
    val = fp_raw_acc_from_qp(ub);
  fp_QP_t z_new = fp_qp_from_raw_acc(val);

  fp_QP_t z_prev = *z_slot;
  fp_QP_t dz_qp = fp_qp_from_raw_acc(fp_raw_acc_from_qp(z_new) -
                                     fp_raw_acc_from_qp(z_prev));
  fp_QP_t dd = fp_abs(fp_mul(rho, dz_qp));
  if (dd > *state_dual)
    *state_dual = dd;

  fp_QP_t y_new =
      fp_qp_from_raw_acc(fp_raw_acc_from_qp(x_val) - fp_raw_acc_from_qp(z_new) +
                         fp_raw_acc_from_qp(*y_slot));
  *y_slot = y_new;

  fp_QP_t pd = fp_abs(x_val - z_new);
  if (pd > *state_primal)
    *state_primal = pd;

  fp_QP_t abs_z = fp_abs(z_new);
  fp_QP_t abs_l = fp_abs(fp_mul(rho, y_new));
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
  fp_raw_acc_t val = fp_raw_acc_from_qp(u_val) + fp_raw_acc_from_qp(*y_slot);
  if (val < fp_raw_acc_from_qp(lb))
    val = fp_raw_acc_from_qp(lb);
  if (val > fp_raw_acc_from_qp(ub))
    val = fp_raw_acc_from_qp(ub);
  fp_QP_t z_new = fp_qp_from_raw_acc(val);

  fp_QP_t z_prev = *z_slot;
  fp_QP_t dz_qp = fp_qp_from_raw_acc(fp_raw_acc_from_qp(z_new) -
                                     fp_raw_acc_from_qp(z_prev));
  fp_QP_t dd = fp_abs(fp_mul(rho_u, dz_qp));
  if (dd > *ctrl_dual)
    *ctrl_dual = dd;

  fp_QP_t y_new =
      fp_qp_from_raw_acc(fp_raw_acc_from_qp(u_val) - fp_raw_acc_from_qp(z_new) +
                         fp_raw_acc_from_qp(*y_slot));
  *y_slot = y_new;

  fp_QP_t pd = fp_abs(u_val - z_new);
  if (pd > *ctrl_primal)
    *ctrl_primal = pd;

  fp_QP_t abs_z = fp_abs(z_new);
  fp_QP_t abs_l = fp_abs(fp_mul(rho_u, y_new));
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
 *   B: represented as sparse scalars:
 *      B_delta_rate (= B[5][0]),
 *      B_vx_accel  (= B[2][1]),
 *      B_vy_accel  (= B[3][1]),
 *      B_omega_accel (= B[4][1]),
 *      with implicit latch rows x6_next=u0 and x7_next=u1
 *===========================================================================*/

static void
riccati_pass_hls(const StepData_t step_data[MPC_HORIZON],
                 const fp_QP_t *terminal_q_diag,
                 const fp_QP_t *terminal_q_linear, const fp_QP_t *x0,
                 fp_QP_t rho, fp_QP_t rho_u, const fp_QP_t z_x[][MPC_NX_AUG],
                 const fp_QP_t y_x[][MPC_NX_AUG], const fp_QP_t z_u[][MPC_NU],
                 const fp_QP_t y_u[][MPC_NU], fp_QP_t x_out[][MPC_NX_AUG],
                 fp_QP_t u_out[][MPC_NU]) {
#pragma HLS ARRAY_PARTITION variable = z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = x_out complete dim = 2
#pragma HLS ARRAY_PARTITION variable = u_out complete dim = 2
#pragma HLS ALLOCATION function instances = fp_recip        limit = 1
#pragma HLS ALLOCATION function instances = sum6_acc_QP_raw limit = 1
#pragma HLS ALLOCATION function instances = sum6_QP_raw     limit = 1
#pragma HLS ALLOCATION function instances = sum8_acc_QP_raw limit = 21
#pragma HLS ALLOCATION function instances = sum8_QP_raw     limit = 2

  const int nx = MPC_NX_AUG;
  const int nu = MPC_NU;
  const int N = MPC_HORIZON;

  fp_QP_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG];
  fp_QP_t kk[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable = K complete dim = 2
#pragma HLS ARRAY_PARTITION variable = K complete dim = 3
#pragma HLS ARRAY_PARTITION variable = kk complete dim = 2

  fp_raw_acc_t P[MPC_NX_AUG][MPC_NX_AUG];
  fp_raw_acc_t p[MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = P complete dim = 2
#pragma HLS BIND_STORAGE variable = P type = RAM_2P impl = LUTRAM latency = 1
#pragma HLS ARRAY_PARTITION variable = p complete dim = 1

  /* Initialize terminal cost: P_N = Q_N [+ rho*I if constrained] */
  int s, i, j, a, b, k;

  for (i = 0; i < nx; i++) {
#pragma HLS UNROLL
    for (j = 0; j < nx; j++) {
#pragma HLS UNROLL
      P[i][j] = 0;
    }
    p[i] = 0;
  }

  {
    for (s = 0; s < nx; s++) {
      P[s][s] = fp_raw_acc_from_qp(terminal_q_diag[s]);
      p[s] = fp_raw_acc_from_qp(terminal_q_linear[s]);
    }
    {
      const int idx = IDX_EY;
      P[idx][idx] += fp_raw_acc_from_qp(rho);

      fp_raw_acc_t zx_minus_yx =
          fp_raw_acc_from_qp(z_x[N][idx]) - fp_raw_acc_from_qp(y_x[N][idx]);
      fp_acc_QP_mul_t rho_state_mul =
          fp_mul_QP_acc(fp_qp_raw_from_QP(rho), zx_minus_yx);

      p[idx] -= fp_shift_right_clip_to_acc(rho_state_mul, FP_FRAC_BITS);
    }
    {
      const int idx = IDX_DELTA_ACT;
      P[idx][idx] += fp_raw_acc_from_qp(rho);

      fp_raw_acc_t zx_minus_yx =
          fp_raw_acc_from_qp(z_x[N][idx]) - fp_raw_acc_from_qp(y_x[N][idx]);
      fp_acc_QP_mul_t rho_state_mul =
          fp_mul_QP_acc(fp_qp_raw_from_QP(rho), zx_minus_yx);

      p[idx] -= fp_shift_right_clip_to_acc(rho_state_mul, FP_FRAC_BITS);
    }
  }

  /* ===== Backward pass: k = N-1 down to 0 ===== */
  for (k = N - 1; k >= 0; k--) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
#pragma HLS LOOP_FLATTEN off
    const StepData_t *sd = &step_data[k];
    const fp_QP_raw_t d0_raw = fp_qp_raw_from_QP(sd->d0);
    const fp_QP_raw_t d1_raw = fp_qp_raw_from_QP(sd->d1);
    const fp_QP_raw_t d2_raw = fp_qp_raw_from_QP(sd->d2);
    const fp_QP_raw_t d3_raw = fp_qp_raw_from_QP(sd->d3);
    const fp_QP_raw_t d4_raw = fp_qp_raw_from_QP(sd->d4);
    const fp_QP_raw_t d5_raw = fp_qp_raw_from_QP(sd->d5);

    /* Augmented costs */
    fp_raw_acc_t q_aug_diag[MPC_NX_AUG];
    fp_raw_acc_t q_aug_linear[MPC_NX_AUG];
    fp_raw_acc_t r_aug_diag[MPC_NU];
    fp_raw_acc_t r_aug_linear[MPC_NU];

    for (s = 0; s < nx; s++) {
    #pragma HLS UNROLL
      q_aug_diag[s] = fp_raw_acc_from_qp(sd->Q_diag[s]);
      q_aug_linear[s] = fp_raw_acc_from_qp(sd->q[s]);
    }

    {
      const int idx = IDX_EY;
      q_aug_diag[idx] += fp_raw_acc_from_qp(rho);

      fp_raw_acc_t zx_minus_yx =
          fp_raw_acc_from_qp(z_x[k][idx]) - fp_raw_acc_from_qp(y_x[k][idx]);
      fp_acc_QP_mul_t rho_state_mul =
          fp_mul_QP_acc(fp_qp_raw_from_QP(rho), zx_minus_yx);

      q_aug_linear[idx] -= fp_shift_right_clip_to_acc(rho_state_mul, FP_FRAC_BITS);
    }

    {
      const int idx = IDX_DELTA_ACT;
      q_aug_diag[idx] += fp_raw_acc_from_qp(rho);

      fp_raw_acc_t zx_minus_yx =
          fp_raw_acc_from_qp(z_x[k][idx]) - fp_raw_acc_from_qp(y_x[k][idx]);
      fp_acc_QP_mul_t rho_state_mul =
          fp_mul_QP_acc(fp_qp_raw_from_QP(rho), zx_minus_yx);

      q_aug_linear[idx] -= fp_shift_right_clip_to_acc(rho_state_mul, FP_FRAC_BITS);
    }

    for (a = 0; a < nu; a++) {
    #pragma HLS UNROLL
      r_aug_diag[a] =
          fp_raw_acc_from_qp(sd->R_diag[a]) + fp_raw_acc_from_qp(rho_u);

      fp_raw_acc_t zu_minus_yu =
          fp_raw_acc_from_qp(z_u[k][a]) - fp_raw_acc_from_qp(y_u[k][a]);
      fp_acc_QP_mul_t rho_ctrl_mul =
          fp_mul_QP_acc(fp_qp_raw_from_QP(rho_u), zu_minus_yu);

      r_aug_linear[a] =
          fp_raw_acc_from_qp(sd->r[a]) -
          fp_shift_right_clip_to_acc(rho_ctrl_mul, FP_FRAC_BITS);
    }

    /* Step 1: M = B^T * P (nu x nx) — accumulate only nonzero B rows */
    fp_raw_acc_t M[MPC_NU][MPC_NX_AUG];
    #pragma HLS ARRAY_PARTITION variable = M complete dim = 1
    #pragma HLS ARRAY_PARTITION variable = M complete dim = 2

    fp_QP_raw_t b00 = fp_qp_raw_from_QP(sd->B_delta_rate);
    fp_QP_raw_t b10 = fp_qp_raw_from_QP(sd->B_vx_accel);
    fp_QP_raw_t b11 = fp_qp_raw_from_QP(sd->B_vy_accel);
    fp_QP_raw_t b12 = fp_qp_raw_from_QP(sd->B_omega_accel);

    for (j = 0; j < nx; j++) {
    #pragma HLS UNROLL
      fp_sum6_acc_QP_t s0 =
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b00, P[IDX_DELTA_ACT][j]) +
          ((fp_sum6_acc_QP_t)P[IDX_DELTA_RATE_PREV][j] << FP_FRAC_BITS);

      fp_sum6_acc_QP_t s1 =
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b10, P[2][j]) +
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b11, P[3][j]) +
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b12, P[4][j]) +
          ((fp_sum6_acc_QP_t)P[IDX_ACCEL_PREV][j] << FP_FRAC_BITS);

      M[0][j] = shift_right_clip_sum_acc_QP_to_acc(s0, FP_FRAC_BITS);
      M[1][j] = shift_right_clip_sum_acc_QP_to_acc(s1, FP_FRAC_BITS);
    }

    /* Step 2: S = r_aug_diag + M*B (2x2) */
    fp_raw_acc_t S[2][2];
    {
      fp_sum6_acc_QP_t mb00 =
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[0][IDX_DELTA_ACT], b00) +
          ((fp_sum6_acc_QP_t)M[0][IDX_DELTA_RATE_PREV] << FP_FRAC_BITS);

      fp_sum6_acc_QP_t mb01 =
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[0][2], b10) +
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[0][3], b11) +
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[0][4], b12) +
          ((fp_sum6_acc_QP_t)M[0][IDX_ACCEL_PREV] << FP_FRAC_BITS);

      fp_sum6_acc_QP_t mb10 =
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[1][IDX_DELTA_ACT], b00) +
          ((fp_sum6_acc_QP_t)M[1][IDX_DELTA_RATE_PREV] << FP_FRAC_BITS);

      fp_sum6_acc_QP_t mb11 =
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[1][2], b10) +
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[1][3], b11) +
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[1][4], b12) +
          ((fp_sum6_acc_QP_t)M[1][IDX_ACCEL_PREV] << FP_FRAC_BITS);

      S[0][0] = r_aug_diag[0] + shift_right_clip_sum_acc_QP_to_acc(mb00, FP_FRAC_BITS);
      S[0][1] = shift_right_clip_sum_acc_QP_to_acc(mb01, FP_FRAC_BITS);
      S[1][0] = shift_right_clip_sum_acc_QP_to_acc(mb10, FP_FRAC_BITS);
      S[1][1] = r_aug_diag[1] + shift_right_clip_sum_acc_QP_to_acc(mb11, FP_FRAC_BITS);

      fp_raw_acc_t s01 = (S[0][1] + S[1][0]) >> 1;
      S[0][1] = s01;
      S[1][0] = s01;
    }

    /* Step 3: Invert S (2x2) */
    fp_raw_acc_t Si[2][2];
    if (invert_2x2_hls(S, Si) < 0) {
      const fp_raw_acc_t eps = fp_raw_acc_from_neg_pow2(FP_INVERT_2X2_DIAG_FALLBACK_MIN_EXP);

      fp_raw_acc_t s00 = (S[0][0] > eps) ? S[0][0] : eps;
      fp_raw_acc_t s11 = (S[1][1] > eps) ? S[1][1] : eps;

      fp_QP_t s00_qp = fp_qp_from_raw_acc(fp_clip_raw_to_qp(s00));
      fp_QP_t s11_qp = fp_qp_from_raw_acc(fp_clip_raw_to_qp(s11));

      Si[0][0] = fp_raw_acc_from_qp(fp_recip(s00_qp));
      Si[0][1] = 0;
      Si[1][0] = 0;
      Si[1][1] = fp_raw_acc_from_qp(fp_recip(s11_qp));
    }

    fp_QP_t Si_qp[2][2];
    #pragma HLS ARRAY_PARTITION variable = Si_qp complete dim = 1
    #pragma HLS ARRAY_PARTITION variable = Si_qp complete dim = 2

    for (a = 0; a < nu; a++) {
    #pragma HLS UNROLL
      for (b = 0; b < nu; b++) {
    #pragma HLS UNROLL
        Si_qp[a][b] = fp_qp_from_raw_acc(Si[a][b]);
      }
    }

    /* Step 4: G = M*A + N^T (nu x nx) — exploit A sparsity */
    fp_raw_acc_t G[MPC_NU][MPC_NX_AUG];
#pragma HLS ARRAY_PARTITION variable = G complete dim = 1
#pragma HLS ARRAY_PARTITION variable = G complete dim = 2
    for (a = 0; a < nu; a++) {
      for (j = 0; j < 6; j++) {
    #pragma HLS PIPELINE II = 1
        fp_sum6_acc_QP_t sum = sum6_acc_QP_raw(
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[a][0], fp_qp_raw_from_QP(sd->A[0][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[a][1], fp_qp_raw_from_QP(sd->A[1][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[a][2], fp_qp_raw_from_QP(sd->A[2][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[a][3], fp_qp_raw_from_QP(sd->A[3][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[a][4], fp_qp_raw_from_QP(sd->A[4][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(M[a][5], fp_qp_raw_from_QP(sd->A[5][j])));
        G[a][j] = shift_right_clip_sum_acc_QP_to_acc(sum, FP_FRAC_BITS);
      }
    }

    G[0][6] = fp_raw_acc_from_qp(sd->N_delta_rate);
    G[1][6] = 0;
    G[0][7] = 0;
    G[1][7] = fp_raw_acc_from_qp(sd->N_accel);

    /* Step 5: K = -S^{-1} * G (nu x nx) */
    for (a = 0; a < nu; a++) {
      for (j = 0; j < nx; j++) {
    #pragma HLS PIPELINE II = 1
        fp_sum6_acc_QP_t val = 0;
        for (b = 0; b < nu; b++) {
    #pragma HLS UNROLL
          val += (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(Si_qp[a][b]),
                                                G[b][j]);
        }
        fp_QP_raw_t k_raw =
            shift_right_clip_sum_acc_QP_to_qp(-val, FP_FRAC_BITS);
        K[k][a][j] = fp_QP_from_qp_raw(k_raw);
      }
    }

    /* Step 6: kk = -S^{-1} * (r_aug_linear + B^T * p) */
    fp_raw_acc_t p_shift[MPC_NX_AUG];
    #pragma HLS ARRAY_PARTITION variable = p_shift complete
    for (i = 0; i < nx; i++) {
    #pragma HLS PIPELINE II = 1
      fp_sum6_acc_QP_t pd_sum = sum6_acc_QP_raw(
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][0], d0_raw),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][1], d1_raw),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][2], d2_raw),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][3], d3_raw),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][4], d4_raw),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][5], d5_raw));

      p_shift[i] = p[i] + shift_right_clip_sum_acc_QP_to_acc(pd_sum, FP_FRAC_BITS);
    }

    fp_raw_acc_t Bp[MPC_NU];
    {
      fp_raw_acc_t ps2 = p_shift[2];
      fp_raw_acc_t ps3 = p_shift[3];
      fp_raw_acc_t ps4 = p_shift[4];
      fp_raw_acc_t ps5 = p_shift[IDX_DELTA_ACT];
      fp_raw_acc_t ps6 = p_shift[IDX_DELTA_RATE_PREV];
      fp_raw_acc_t ps7 = p_shift[IDX_ACCEL_PREV];

      fp_sum6_acc_QP_t bp0 =
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b00, ps5) +
          ((fp_sum6_acc_QP_t)ps6 << FP_FRAC_BITS);
      Bp[0] = shift_right_clip_sum_acc_QP_to_acc(bp0, FP_FRAC_BITS);

      fp_sum6_acc_QP_t bp1 =
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b10, ps2) +
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b11, ps3) +
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(b12, ps4) +
          ((fp_sum6_acc_QP_t)ps7 << FP_FRAC_BITS);
      Bp[1] = shift_right_clip_sum_acc_QP_to_acc(bp1, FP_FRAC_BITS);
    }

    for (a = 0; a < nu; a++) {
    #pragma HLS UNROLL
      fp_sum6_acc_QP_t val = 0;
      for (b = 0; b < nu; b++) {
    #pragma HLS UNROLL
        val += (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(Si_qp[a][b]),
                                              (r_aug_linear[b] + Bp[b]));
      }
      fp_QP_raw_t kk_raw =
          shift_right_clip_sum_acc_QP_to_qp(-val, FP_FRAC_BITS);
      kk[k][a] = fp_QP_from_qp_raw(kk_raw);
    }

    /* Step 7: P = Q_diag + A^T*P*A + G^T*K */
    /* PA = P * A (only 6x6 dense block needed)
     * A sparsity: col 0 has only A[0][0]=1 (rows 1-5 are 0).
     * Col 5 has up to 4 nonzero entries (steering coupling).
     * Exploit col-0 identity: PA[i][0] = P[i][0], no multiply. */
    fp_raw_acc_t PA[MPC_NX_DENSE][MPC_NX_DENSE];
/* Partition rows only (dim=1). dim=0 (all-register) caused congestion level 6
 * by creating 36x46=1656 bits of parallel FFs all simultaneously routed to the
 * 21 COMPUTE_P_SUM_TO multiplier inputs. Using LUTRAM instead of block BRAM
 * avoids the BRAM-SP routing issue (the original reason for dim=0) because
 * LUTRAM writes use standard fabric routing, not the BRAM SP pin. */
#pragma HLS ARRAY_PARTITION variable = PA complete dim = 1
#pragma HLS BIND_STORAGE variable = PA type = RAM_2P impl = LUTRAM latency = 1

    /* Col 0: A[0][0]=1, rest zero → PA[i][0] = P[i][0] */
    for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
      PA[i][0] = P[i][0];
    }
    /* Cols 1-5: route the adder tree through sum8_acc_QP_raw (INLINE off).
     * sum8_acc_QP_raw creates a module boundary with registered I/Os, placing
     * the 3-level CARRY8 tree inside the sub-module.  The PA LUTRAM write
     * then only sees a registered output → bit-select → RAMD32, eliminating
     * the carry-chain→LUTRAM routing hop that caused the -0.333 ns violation.
     * Adds 1 cycle to pipeline depth (≈6k cycles over 50 ADMM iterations). */
    for (i = 0; i < 6; i++) {
      for (j = 1; j < 6; j++) {
    #pragma HLS PIPELINE II = 1
        fp_sum6_acc_QP_t sum = sum6_acc_QP_raw(
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][0], fp_qp_raw_from_QP(sd->A[0][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][1], fp_qp_raw_from_QP(sd->A[1][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][2], fp_qp_raw_from_QP(sd->A[2][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][3], fp_qp_raw_from_QP(sd->A[3][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][4], fp_qp_raw_from_QP(sd->A[4][j])),
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(P[i][5], fp_qp_raw_from_QP(sd->A[5][j])));

        PA[i][j] = shift_right_clip_sum_acc_QP_to_acc(sum, FP_FRAC_BITS);
      }
    }

    /* P = Q_diag + A^T*PA + G^T*K
     * All 21 upper-triangle entries computed in parallel (COMPUTE_P_SUM_TO)
     * with ALLOCATION capping multiplier instances at 40.
     * - Fully unrolled (147 logical muls): 871 DSPs, 3817 riccati cycles →
     * timing violation
     * - Sequential loop  (8 physical muls): 252 DSPs, 5798 riccati cycles →
     * 29µs (too slow)
     * - Allocation=40   (40 physical muls): ~180 DSPs, ~4200 riccati cycles
     * - limit=74 was tested; added 178 DSPs with no latency change
     * HLS time-multiplexes 147 logical muls onto physical in ceil(147/limit)
     * rounds. */
#pragma HLS ALLOCATION operation instances = mul limit = 75
    {
#define COMPUTE_P_SUM_TO(II, JJ, DST)                                            \
  do {                                                                           \
    fp_sum6_acc_QP_t _sum_tmp = sum8_acc_QP_raw(                                       \
        (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[0][(II)]), PA[0][(JJ)]), \
        (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[1][(II)]), PA[1][(JJ)]), \
        (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[2][(II)]), PA[2][(JJ)]), \
        (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[3][(II)]), PA[3][(JJ)]), \
        (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[4][(II)]), PA[4][(JJ)]), \
        (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[5][(II)]), PA[5][(JJ)]), \
        (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][(II)], fp_qp_raw_from_QP(K[k][0][(JJ)])), \
        (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][(II)], fp_qp_raw_from_QP(K[k][1][(JJ)]))); \
    (DST) = shift_right_clip_sum_acc_QP_to_acc(_sum_tmp, FP_FRAC_BITS);              \
  } while (0)

      fp_raw_acc_t r0_0, r0_1, r0_2, r0_3, r0_4, r0_5;
      COMPUTE_P_SUM_TO(0, 0, r0_0);
      COMPUTE_P_SUM_TO(0, 1, r0_1);
      COMPUTE_P_SUM_TO(0, 2, r0_2);
      COMPUTE_P_SUM_TO(0, 3, r0_3);
      COMPUTE_P_SUM_TO(0, 4, r0_4);
      COMPUTE_P_SUM_TO(0, 5, r0_5);
      P[0][0] = q_aug_diag[0] + r0_0;
      P[0][1] = P[1][0] = r0_1;
      P[0][2] = P[2][0] = r0_2;
      P[0][3] = P[3][0] = r0_3;
      P[0][4] = P[4][0] = r0_4;
      P[0][5] = P[5][0] = r0_5;

      fp_raw_acc_t r1_1, r1_2, r1_3, r1_4, r1_5;
      COMPUTE_P_SUM_TO(1, 1, r1_1);
      COMPUTE_P_SUM_TO(1, 2, r1_2);
      COMPUTE_P_SUM_TO(1, 3, r1_3);
      COMPUTE_P_SUM_TO(1, 4, r1_4);
      COMPUTE_P_SUM_TO(1, 5, r1_5);
      P[1][1] = q_aug_diag[1] + r1_1;
      P[1][2] = P[2][1] = r1_2;
      P[1][3] = P[3][1] = r1_3;
      P[1][4] = P[4][1] = r1_4;
      P[1][5] = P[5][1] = r1_5;

      fp_raw_acc_t r2_2, r2_3, r2_4, r2_5;
      COMPUTE_P_SUM_TO(2, 2, r2_2);
      COMPUTE_P_SUM_TO(2, 3, r2_3);
      COMPUTE_P_SUM_TO(2, 4, r2_4);
      COMPUTE_P_SUM_TO(2, 5, r2_5);
      P[2][2] = q_aug_diag[2] + r2_2;
      P[2][3] = P[3][2] = r2_3;
      P[2][4] = P[4][2] = r2_4;
      P[2][5] = P[5][2] = r2_5;

      fp_raw_acc_t r3_3, r3_4, r3_5;
      COMPUTE_P_SUM_TO(3, 3, r3_3);
      COMPUTE_P_SUM_TO(3, 4, r3_4);
      COMPUTE_P_SUM_TO(3, 5, r3_5);
      P[3][3] = q_aug_diag[3] + r3_3;
      P[3][4] = P[4][3] = r3_4;
      P[3][5] = P[5][3] = r3_5;

      fp_raw_acc_t r4_4, r4_5;
      COMPUTE_P_SUM_TO(4, 4, r4_4);
      COMPUTE_P_SUM_TO(4, 5, r4_5);
      P[4][4] = q_aug_diag[4] + r4_4;
      P[4][5] = P[5][4] = r4_5;

      fp_raw_acc_t r5_5;
      COMPUTE_P_SUM_TO(5, 5, r5_5);
      P[5][5] = q_aug_diag[5] + r5_5;

#undef COMPUTE_P_SUM_TO
    }

    /* Cols 6,7 cross-block: AtPA=0, only G^T*K contributes.
     * Compute rows 0-5 cross-entries and 2x2 lower-right in one pass. */
    {
      fp_raw_acc_t gtk_i6[6], gtk_i7[6];
      fp_raw_acc_t gtk_66 = 0, gtk_67 = 0, gtk_77 = 0;
      #pragma HLS ARRAY_PARTITION variable = gtk_i6 complete dim = 1
      #pragma HLS ARRAY_PARTITION variable = gtk_i7 complete dim = 1

      for (i = 0; i < 6; i++) {
      #pragma HLS UNROLL
        fp_sum6_acc_QP_t s6 =
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][i], fp_qp_raw_from_QP(K[k][0][6])) +
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][i], fp_qp_raw_from_QP(K[k][1][6]));
        fp_sum6_acc_QP_t s7 =
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][i], fp_qp_raw_from_QP(K[k][0][7])) +
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][i], fp_qp_raw_from_QP(K[k][1][7]));

        gtk_i6[i] = shift_right_clip_sum_acc_QP_to_acc(s6, FP_FRAC_BITS);
        gtk_i7[i] = shift_right_clip_sum_acc_QP_to_acc(s7, FP_FRAC_BITS);
      }

      {
        fp_sum6_acc_QP_t s66 =
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][6], fp_qp_raw_from_QP(K[k][0][6])) +
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][6], fp_qp_raw_from_QP(K[k][1][6]));
        fp_sum6_acc_QP_t s67 =
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][6], fp_qp_raw_from_QP(K[k][0][7])) +
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][6], fp_qp_raw_from_QP(K[k][1][7]));
        fp_sum6_acc_QP_t s77 =
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][7], fp_qp_raw_from_QP(K[k][0][7])) +
            (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][7], fp_qp_raw_from_QP(K[k][1][7]));

        gtk_66 = shift_right_clip_sum_acc_QP_to_acc(s66, FP_FRAC_BITS);
        gtk_67 = shift_right_clip_sum_acc_QP_to_acc(s67, FP_FRAC_BITS);
        gtk_77 = shift_right_clip_sum_acc_QP_to_acc(s77, FP_FRAC_BITS);
      }

      /* Store cross-block (rows 0-5 × cols 6-7) */
      for (i = 0; i < 6; i++) {
#pragma HLS UNROLL
        P[i][6] = gtk_i6[i];
        P[6][i] = gtk_i6[i];
        P[i][7] = gtk_i7[i];
        P[7][i] = gtk_i7[i];
      }
      /* 2x2 lower-right block */
      P[6][6] = q_aug_diag[6] + gtk_66;
      P[7][7] = q_aug_diag[7] + gtk_77;
      fp_raw_acc_t p67 = gtk_67;
      P[6][7] = p67;
      P[7][6] = p67;
    }

    /* Step 8: p_new = q_aug_linear + A^T*p + G^T*kk */
    fp_raw_acc_t p_new[MPC_NX_AUG];
    for (i = 0; i < 6; i++) {
    #pragma HLS PIPELINE II = 1
      fp_sum6_acc_QP_t total = sum8_acc_QP_raw(
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[0][i]), p_shift[0]),
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[1][i]), p_shift[1]),
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[2][i]), p_shift[2]),
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[3][i]), p_shift[3]),
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[4][i]), p_shift[4]),
          (fp_sum6_acc_QP_t)fp_mul_QP_acc(fp_qp_raw_from_QP(sd->A[5][i]), p_shift[5]),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][i], fp_qp_raw_from_QP(kk[k][0])),
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][i], fp_qp_raw_from_QP(kk[k][1])));

      p_new[i] = q_aug_linear[i] + shift_right_clip_sum_acc_QP_to_acc(total, FP_FRAC_BITS);
    }

    for (i = 6; i < nx; i++) {
    #pragma HLS PIPELINE II = 1
      fp_sum6_acc_QP_t total =
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[0][i], fp_qp_raw_from_QP(kk[k][0])) +
          (fp_sum6_acc_QP_t)fp_mul_acc_QP(G[1][i], fp_qp_raw_from_QP(kk[k][1]));

      p_new[i] = q_aug_linear[i] + shift_right_clip_sum_acc_QP_to_acc(total, FP_FRAC_BITS);
    }

    for (i = 0; i < nx; i++) {
#pragma HLS UNROLL factor = MPC_HLS_AFFINE_PNEW_UNROLL
      p[i] = p_new[i];
    }

  } /* end backward pass */

  /* ===== Forward pass: roll out x, u from x0 ===== */
  for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
    x_out[0][s] = x0[s];
  }

  for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
#pragma HLS LOOP_FLATTEN off
    const StepData_t *sd = &step_data[k];

    const fp_raw_acc_t d0_fwd = fp_raw_acc_from_qp(sd->d0);
    const fp_raw_acc_t d1_fwd = fp_raw_acc_from_qp(sd->d1);
    const fp_raw_acc_t d2_fwd = fp_raw_acc_from_qp(sd->d2);
    const fp_raw_acc_t d3_fwd = fp_raw_acc_from_qp(sd->d3);
    const fp_raw_acc_t d4_fwd = fp_raw_acc_from_qp(sd->d4);
    const fp_raw_acc_t d5_fwd = fp_raw_acc_from_qp(sd->d5);

    /* Cache current state locally to avoid repeated RAM reads and same-iter
     * write/read pressure between u_out and state rollout. */
    const fp_QP_t xk0 = x_out[k][0];
    const fp_QP_t xk1 = x_out[k][1];
    const fp_QP_t xk2 = x_out[k][2];
    const fp_QP_t xk3 = x_out[k][3];
    const fp_QP_t xk4 = x_out[k][4];
    const fp_QP_t xk5 = x_out[k][5];
    const fp_QP_t xk6 = x_out[k][6];
    const fp_QP_t xk7 = x_out[k][7];

    /* u_k = K_k * x_k + kk_k */
    fp_sum6_QP_mul_t prod_sum0 = sum8_QP_raw(
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][0]), fp_qp_raw_from_QP(xk0)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][1]), fp_qp_raw_from_QP(xk1)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][2]), fp_qp_raw_from_QP(xk2)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][3]), fp_qp_raw_from_QP(xk3)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][4]), fp_qp_raw_from_QP(xk4)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][5]), fp_qp_raw_from_QP(xk5)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][6]), fp_qp_raw_from_QP(xk6)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][0][7]), fp_qp_raw_from_QP(xk7)));

    fp_sum6_QP_mul_t prod_sum1 = sum8_QP_raw(
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][0]), fp_qp_raw_from_QP(xk0)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][1]), fp_qp_raw_from_QP(xk1)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][2]), fp_qp_raw_from_QP(xk2)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][3]), fp_qp_raw_from_QP(xk3)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][4]), fp_qp_raw_from_QP(xk4)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][5]), fp_qp_raw_from_QP(xk5)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][6]), fp_qp_raw_from_QP(xk6)),
        (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(K[k][1][7]), fp_qp_raw_from_QP(xk7)));

    fp_raw_acc_t u0_sum = fp_raw_acc_from_qp(kk[k][0]) +
                  shift_right_clip_sum_QP_QP_to_acc(prod_sum0, FP_FRAC_BITS);
    fp_raw_acc_t u1_sum = fp_raw_acc_from_qp(kk[k][1]) +
                  shift_right_clip_sum_QP_QP_to_acc(prod_sum1, FP_FRAC_BITS);

    u0_sum = fp_clip_raw_to_qp(u0_sum);
    u1_sum = fp_clip_raw_to_qp(u1_sum);

    const fp_QP_t u0_k = fp_QP_from_qp_raw((fp_QP_raw_t)u0_sum);
    const fp_QP_t u1_k = fp_QP_from_qp_raw((fp_QP_raw_t)u1_sum);
    u_out[k][0] = u0_k;
    u_out[k][1] = u1_k;

    /* x_{k+1} = A_k * x_k + B_k * u_k
     * Dense rows 0..5: A*x + B*u */
    for (i = 0; i < 6; i++) {
    #pragma HLS PIPELINE II = 1
      fp_raw_acc_t d_i = d0_fwd;
      if (i == 1) d_i = d1_fwd;
      else if (i == 2) d_i = d2_fwd;
      else if (i == 3) d_i = d3_fwd;
      else if (i == 4) d_i = d4_fwd;
      else if (i == 5) d_i = d5_fwd;

      fp_sum6_QP_mul_t ax_sum = sum6_QP_raw(
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->A[i][0]), fp_qp_raw_from_QP(xk0)),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->A[i][1]), fp_qp_raw_from_QP(xk1)),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->A[i][2]), fp_qp_raw_from_QP(xk2)),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->A[i][3]), fp_qp_raw_from_QP(xk3)),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->A[i][4]), fp_qp_raw_from_QP(xk4)),
          (fp_sum6_QP_mul_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->A[i][5]), fp_qp_raw_from_QP(xk5)));

      fp_sum6_acc_QP_t sum =
          ((fp_sum6_acc_QP_t)d_i << FP_FRAC_BITS) +
          (fp_sum6_acc_QP_t)ax_sum;

      if (i == 2) {
        sum += (fp_sum6_acc_QP_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->B_vx_accel),
                                        fp_qp_raw_from_QP(u1_k));
      } else if (i == 3) {
        sum += (fp_sum6_acc_QP_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->B_vy_accel),
                                        fp_qp_raw_from_QP(u1_k));
      } else if (i == 4) {
        sum += (fp_sum6_acc_QP_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->B_omega_accel),
                                        fp_qp_raw_from_QP(u1_k));
      } else if (i == 5) {
        sum += (fp_sum6_acc_QP_t)fp_mul_QP_raw(fp_qp_raw_from_QP(sd->B_delta_rate),
                                        fp_qp_raw_from_QP(u0_k));
      }

      fp_QP_raw_t result = shift_right_clip_sum_acc_QP_to_qp(sum, FP_FRAC_BITS);
      x_out[k + 1][i] = fp_QP_from_qp_raw(result);
    }

    /* Rows 6,7: x_prev = u (identity in B, zero in A) */
    x_out[k + 1][IDX_DELTA_RATE_PREV] = u0_k;
    x_out[k + 1][IDX_ACCEL_PREV] = u1_k;
  }
}

/*===========================================================================
 * Main Solver: Riccati-ADMM
 *===========================================================================*/

MpcStatus_t riccati_admm_solve_hls(const StepData_t step_data[MPC_HORIZON],
                                   const fp_QP_t terminal_q_diag[MPC_NX_AUG],
                                   const fp_QP_t terminal_q_linear[MPC_NX_AUG],
                                   const fp_QP_t terminal_x_lb[MPC_NX_AUG],
                                   const fp_QP_t terminal_x_ub[MPC_NX_AUG],
                                   const fp_QP_t x0[MPC_NX_AUG],
                                   const AdmmConfig_t *config,
                                   AdmmState_t *admm_state,
                                   MpcSolution_t *solution) {
#pragma HLS INLINE
#pragma HLS DISAGGREGATE variable = admm_state
#pragma HLS ARRAY_PARTITION variable = admm_state->z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = admm_state->y_u complete dim = 2
#ifndef __SYNTHESIS__
  g_hls_debug_trace_count = 0;
#endif
  const int nx = MPC_NX_AUG;
  const int nu = MPC_NU;
  const int N = MPC_HORIZON;

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
  fp_QP_t z_x[MPC_HORIZON + 1][MPC_NX_AUG];
  fp_QP_t z_u[MPC_HORIZON][MPC_NU];
  fp_QP_t y_x[MPC_HORIZON + 1][MPC_NX_AUG];
  fp_QP_t y_u[MPC_HORIZON][MPC_NU];
  fp_QP_t sol_x[MPC_HORIZON + 1][MPC_NX_AUG];
  fp_QP_t sol_u[MPC_HORIZON][MPC_NU];
#pragma HLS ARRAY_PARTITION variable = z_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = z_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = y_u complete dim = 2
#pragma HLS ARRAY_PARTITION variable = sol_x complete dim = 2
#pragma HLS ARRAY_PARTITION variable = sol_u complete dim = 2

  int k, s, a, i;

  const bool cold_start = !admm_state->initialized;

  /* CPU parity warm-start:
   * - cold start: zero all ADMM buffers
   * - warm start: reuse full persisted z/y buffers directly (no horizon shift)
   */
  if (cold_start) {
    for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II = 1
      for (s = 0; s < nx; s++) {
        z_x[k][s] = 0;
        y_x[k][s] = 0;
      }
    }
    for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II = 1
      for (a = 0; a < nu; a++) {
        z_u[k][a] = 0;
        y_u[k][a] = 0;
      }
    }
  } else {
    for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II = 1
      for (s = 0; s < nx; s++) {
        z_x[k][s] = admm_state->z_x[k][s];
        y_x[k][s] = admm_state->y_x[k][s];
      }
    }
    for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II = 1
      for (a = 0; a < nu; a++) {
        z_u[k][a] = admm_state->z_u[k][a];
        y_u[k][a] = admm_state->y_u[k][a];
      }
    }
  }

  MpcStatus_t status = MPC_STATUS_MAX_ITER;
  solution->iterations = 0;
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
    riccati_pass_hls(step_data, terminal_q_diag, terminal_q_linear, x0,
                     pass_rho, pass_rho_u, (const fp_QP_t(*)[MPC_NX_AUG])z_x,
                     (const fp_QP_t(*)[MPC_NX_AUG])y_x,
                     (const fp_QP_t(*)[MPC_NU])z_u,
                     (const fp_QP_t(*)[MPC_NU])y_u, sol_x, sol_u);

    if (bootstrap_pass) {
      /* Initialize z from projection of unconstrained solution. */
      for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON_PLUS_ONE max = MPC_HORIZON_PLUS_ONE
        for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
          z_x[k][s] = sol_x[k][s];
        }
        {
          const int idx = IDX_EY;
          fp_QP_t val = sol_x[k][idx];
          const fp_QP_t xlb =
              (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx];
          const fp_QP_t xub =
              (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx];
          if (val < xlb)
            val = xlb;
          if (val > xub)
            val = xub;
          z_x[k][idx] = val;
        }
        {
          const int idx = IDX_DELTA_ACT;
          fp_QP_t val = sol_x[k][idx];
          const fp_QP_t xlb =
              (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx];
          const fp_QP_t xub =
              (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx];
          if (val < xlb)
            val = xlb;
          if (val > xub)
            val = xub;
          z_x[k][idx] = val;
        }
      }
      for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
        for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
          fp_QP_t val = sol_u[k][a];
          if (val < step_data[k].u_lb[a])
            val = step_data[k].u_lb[a];
          if (val > step_data[k].u_ub[a])
            val = step_data[k].u_ub[a];
          z_u[k][a] = val;
        }
      }

      /* Initialize y (dual) from constraint violation. */
      for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON_PLUS_ONE max = MPC_HORIZON_PLUS_ONE
        y_x[k][IDX_EY] = sol_x[k][IDX_EY] - z_x[k][IDX_EY];
        y_x[k][IDX_DELTA_ACT] = sol_x[k][IDX_DELTA_ACT] - z_x[k][IDX_DELTA_ACT];
      }
      for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
        for (a = 0; a < nu; a++) {
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
    for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON_PLUS_ONE max = MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II = MPC_HLS_STATE_ZY_II
      /* QP assembly only gives finite bounds to e_y and delta_actual.
       * Keep unconstrained states out of the expensive ADMM projection path. */
      for (s = 0; s < nx; s++) {
#pragma HLS UNROLL
        if (s != IDX_EY && s != IDX_DELTA_ACT) {
          z_x[k][s] = sol_x[k][s];
        }
      }

      fp_QP_t x_norm_k =
          max_abs_state8(sol_x[k][0], sol_x[k][1], sol_x[k][2], sol_x[k][3],
                         sol_x[k][4], sol_x[k][5], sol_x[k][6], sol_x[k][7]);
      if (x_norm_k > x_norm)
        x_norm = x_norm_k;

      {
        const int idx = IDX_EY;
        admm_update_state_channel_raw(
            sol_x[k][idx], &z_x[k][idx], &y_x[k][idx], rho,
            (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx],
            (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx], &primal_ey,
            &dual_ey, &znorm_ey, &lnorm_ey);
      }

      {
        const int idx = IDX_DELTA_ACT;
        admm_update_state_channel_raw(
            sol_x[k][idx], &z_x[k][idx], &y_x[k][idx], rho,
            (k < N) ? step_data[k].x_lb[idx] : terminal_x_lb[idx],
            (k < N) ? step_data[k].x_ub[idx] : terminal_x_ub[idx], &primal_da,
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
    for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
#pragma HLS PIPELINE II = MPC_HLS_CTRL_ZY_II
      const StepData_t *sd = &step_data[k];

      fp_QP_t u_norm_k = max_abs_ctrl2(sol_u[k][0], sol_u[k][1]);
      if (u_norm_k > u_norm)
        u_norm = u_norm_k;

      admm_update_control_channel_raw(
          sol_u[k][0], &z_u[k][0], &y_u[k][0], rho_u, sd->u_lb[0], sd->u_ub[0],
          &primal_u0, &dual_u0, &znorm_u0, &lnorm_u0);

      admm_update_control_channel_raw(
          sol_u[k][1], &z_u[k][1], &y_u[k][1], rho_u, sd->u_lb[1], sd->u_ub[1],
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
    fp_QP_t eps_primal = abs_tolerance + fp_mul(rel_tolerance, max_norm);
    fp_QP_t eps_dual = abs_tolerance + fp_mul(rel_tolerance, lambda_norm);

    solution->iterations = admm_iter + 1;
    solution->primal_residual = primal_res;
    solution->dual_residual = dual_res;
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
      const fp_QP_t adapt_ratio_state = FP_QP_CONST(5.0);
      const fp_QP_t adapt_ratio_ctrl = FP_QP_CONST(5.0);

      /* Hoist multiplications so the scheduler can overlap them. */
      const fp_QP_t state_dual_scaled = fp_mul(adapt_ratio_state, state_dual);
      const fp_QP_t state_primal_scaled =
          fp_mul(adapt_ratio_state, state_primal);
      const fp_QP_t ctrl_dual_scaled = fp_mul(adapt_ratio_ctrl, ctrl_dual);
      const fp_QP_t ctrl_primal_scaled = fp_mul(adapt_ratio_ctrl, ctrl_primal);

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
          for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON_PLUS_ONE max = MPC_HORIZON_PLUS_ONE
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
          for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min = MPC_HORIZON max = MPC_HORIZON
#pragma HLS PIPELINE II = 1
            for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
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

  /* Save ADMM state for warm-starting.
   * Policy:
   *  - OPTIMAL  : keep full primal/dual warm start
   *  - MAX_ITER : keep projected primal iterate only, zero duals, reset rho
   *  - ERROR    : invalidate warm start
   */
  if (status == MPC_STATUS_OPTIMAL || status == MPC_STATUS_MAX_ITER) {
    for (k = 0; k <= N; k++) {
  #pragma HLS PIPELINE II = 1
      for (i = 0; i < MPC_NX_AUG; i++) {
        admm_state->z_x[k][i] = z_x[k][i];
        admm_state->y_x[k][i] = y_x[k][i];
      }
    }
    for (k = 0; k < N; k++) {
  #pragma HLS PIPELINE II = 1
      for (a = 0; a < nu; a++) {
        admm_state->z_u[k][a] = z_u[k][a];
        admm_state->y_u[k][a] = y_u[k][a];
      }
    }
    admm_state->rho = rho;
    admm_state->rho_u = rho_u;
    admm_state->initialized = 1;
  } else {
    for (k = 0; k <= N; k++) {
  #pragma HLS PIPELINE II = 1
      for (i = 0; i < MPC_NX_AUG; i++) {
        admm_state->z_x[k][i] = FP_QP_CONST(0.0);
        admm_state->y_x[k][i] = FP_QP_CONST(0.0);
      }
    }
    for (k = 0; k < N; k++) {
  #pragma HLS PIPELINE II = 1
      for (a = 0; a < nu; a++) {
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