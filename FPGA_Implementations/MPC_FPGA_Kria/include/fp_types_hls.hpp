/**
 * @file fp_types_hls.hpp
 * @brief Fixed-point type families for Kria MPC.
 *
 * Canonical numeric spaces:
 *   1) External payload : raw QP-width words
 *   2) QP family        : main stored scalar family for model/QP-facing values
 *   3) Solver internals : wider post-shift storage and exact-width raw products
 *   4) FN family        : Frenet / vehicle-model family
 *   5) Riccati families : specialized stored families for P, M/G, K
 *      (the tiny S/Si 2x2 path is intentionally collapsed back to QP width)
 *
 * There is no legacy compatibility layer in this file.
 */

#ifndef FP_TYPES_HLS_HPP
#define FP_TYPES_HLS_HPP

#ifdef __cplusplus

#include "mpc_fpga_constants.h"
#include <ap_fixed.h>
#include <ap_int.h>

enum FpCastAuditId {
  FP_CAST_AUDIT_SUM2_QP_RAW_TO_QP = 0,
  FP_CAST_AUDIT_SR_QP_MUL_TO_QP,
  FP_CAST_AUDIT_COUNT
};

enum FpCastSiteId {
  FP_CAST_SITE_UNKNOWN = 0,
  FP_CAST_SITE_SUM2_ADD_QP_RAW,
  FP_CAST_SITE_SUM2_SUB_CAST,
  FP_CAST_SITE_SUM2_ADD3_CAST,
  FP_CAST_SITE_MUL_FP_MUL_QP_RAW_Q,
  FP_CAST_SITE_MUL_FP_MUL,
  FP_CAST_SITE_MUL_FP_SQ,
  FP_CAST_SITE_MUL_MPC_RICCATI_A0,
  FP_CAST_SITE_MUL_MPC_RICCATI_A1,
  FP_CAST_SITE_MUL_MPC_RICCATI_A2,
  FP_CAST_SITE_MUL_MPC_RICCATI_A3,
  FP_CAST_SITE_MUL_MPC_RICCATI_A4,
  FP_CAST_SITE_MUL_MPC_RICCATI_B0,
  FP_CAST_SITE_MUL_MPC_RICCATI_B1,
  FP_CAST_SITE_MUL_MPC_RICCATI_DT_UK0,
  FP_CAST_SITE_MUL_RICCATI_RHO_STATE_PN_EY,
  FP_CAST_SITE_MUL_RICCATI_RHO_STATE_PN_DELTA,
  FP_CAST_SITE_MUL_RICCATI_RHO_STATE_QK_EY,
  FP_CAST_SITE_MUL_RICCATI_RHO_STATE_QK_DELTA,
  FP_CAST_SITE_MUL_RICCATI_RHO_CTRL,
  FP_CAST_SITE_MUL_TOP_REF_VX_KAPPA,
  FP_CAST_SITE_MUL_TOP_STEER_RATE,
  FP_CAST_SITE_MUL_MR_HALF_BOUNDS,
  FP_CAST_SITE_MUL_MR_DFF_RAW,
  FP_CAST_SITE_MUL_MR_DELTA_PRED,
  FP_CAST_SITE_MUL_MR_Q_LAT,
  FP_CAST_SITE_MUL_MR_Q_HDG,
  FP_CAST_SITE_MUL_MR_Q_VEL,
  FP_CAST_SITE_MUL_MR_Q_LAT_VEL,
  FP_CAST_SITE_MUL_MR_Q_YAW,
  FP_CAST_SITE_MUL_MR_Q_DELTA,
  FP_CAST_SITE_MUL_MR_V_BLEND_07,
  FP_CAST_SITE_MUL_MR_V_BLEND_03,
  FP_CAST_SITE_MUL_MR_SCALE_VSWITCH,
  FP_CAST_SITE_MUL_MR_UUB_SCALE,
  FP_CAST_SITE_MUL_MR_TERM_Q_EY,
  FP_CAST_SITE_MUL_MR_TERM_Q_HDG,
  FP_CAST_SITE_MUL_MR_TERM_Q_VEL,
  FP_CAST_SITE_MUL_MR_TERM_Q_LAT_VEL,
  FP_CAST_SITE_MUL_MR_TERM_Q_YAW,
  FP_CAST_SITE_MUL_MR_TERM_Q_DELTA,
  FP_CAST_SITE_MUL_MR_PERSIST_STEER,
  FP_CAST_SITE_MUL_RS_DUAL_STATE_DD,
  FP_CAST_SITE_MUL_RS_DUAL_STATE_ABSL,
  FP_CAST_SITE_MUL_RS_DUAL_CTRL_DD,
  FP_CAST_SITE_MUL_RS_DUAL_CTRL_ABSL,
  FP_CAST_SITE_MUL_RS_INV_DET00,
  FP_CAST_SITE_MUL_RS_INV_DET01,
  FP_CAST_SITE_MUL_RS_INV_SI00,
  FP_CAST_SITE_MUL_RS_INV_SI01,
  FP_CAST_SITE_MUL_RS_INV_SI10,
  FP_CAST_SITE_MUL_RS_INV_SI11,
  FP_CAST_SITE_MUL_RS_EPS_PRIMAL_REL,
  FP_CAST_SITE_MUL_RS_EPS_DUAL_REL,
  FP_CAST_SITE_MUL_RS_ADAPT_STATE_DUAL,
  FP_CAST_SITE_MUL_RS_ADAPT_STATE_PRIMAL,
  FP_CAST_SITE_MUL_RS_ADAPT_CTRL_DUAL,
  FP_CAST_SITE_MUL_RS_ADAPT_CTRL_PRIMAL,
  FP_CAST_SITE_COUNT
};

#ifdef CAST_AUDIT
extern "C" void fp_cast_audit_reset(void);
extern "C" unsigned long long fp_cast_audit_get_count(int id);
extern "C" const char *fp_cast_audit_get_name(int id);
extern "C" void fp_cast_audit_print_summary(void);
extern "C" void fp_cast_audit_bump(int id);
extern "C" void fp_cast_audit_bump_sum2_site(int site_id);
extern "C" void fp_cast_audit_bump_mulqp_site(int site_id);
#else
static inline void fp_cast_audit_bump(int id) {
#pragma HLS INLINE
  (void)id;
}
static inline void fp_cast_audit_bump_sum2_site(int site_id) {
#pragma HLS INLINE
  (void)site_id;
}
static inline void fp_cast_audit_bump_mulqp_site(int site_id) {
#pragma HLS INLINE
  (void)site_id;
}
#endif

/*==============================================================================
 * Compile-time configuration
 *============================================================================*/

#ifndef MPC_HLS_QP_WIDTH
#define MPC_HLS_QP_WIDTH MPC_FPGA_QP_WIDTH
#endif

#ifndef MPC_HLS_QP_INT_BITS
#define MPC_HLS_QP_INT_BITS MPC_FPGA_QP_INT_BITS
#endif

#ifndef MPC_HLS_SOLVER_ACC_WIDTH
#define MPC_HLS_SOLVER_ACC_WIDTH 44
#endif

#ifndef MPC_HLS_FN_WIDTH
#define MPC_HLS_FN_WIDTH 26
#endif

#ifndef MPC_HLS_FN_INT_BITS
#define MPC_HLS_FN_INT_BITS 9
#endif

#ifndef MPC_HLS_FN_GUARD
#define MPC_HLS_FN_GUARD 18
#endif

#define MPC_HLS_QP_FRAC_BITS (MPC_HLS_QP_WIDTH - MPC_HLS_QP_INT_BITS)
#define MPC_HLS_FN_FRAC_BITS (MPC_HLS_FN_WIDTH - MPC_HLS_FN_INT_BITS)

static_assert(MPC_HLS_QP_WIDTH == MPC_FPGA_QP_WIDTH,
              "External payload width must match QP width for raw QP transport");
static_assert(MPC_HLS_QP_INT_BITS == MPC_FPGA_QP_INT_BITS,
              "External payload format must match QP format for raw QP transport");

/*------------------------------------------------------------------------------
 * Specialized Riccati family widths
 *
 * Important:
 * All Riccati families keep the same fractional resolution as fp_QP_t.
 * Only the integer span is changed. This keeps shifts/scaling simple.
 *
 * Chosen first-pass widths with headroom:
 *   P  : keep wide (same as existing solver accumulator storage)
 *   MG : B^T P, M A, G, Bp
 *   S  : Schur complement / control Hessian block
 *   Si : inverse(S)
 *   K  : feedback gain / affine term storage
 *
 * Raw intermediate families are guard-sized on purpose, in the same spirit as
 * the FN family accumulator. They are not forced to exact full-width products.
 * The guard values below reflect replay-backed Riccati bounds plus margin.
 *----------------------------------------------------------------------------*/

#ifndef MPC_HLS_P_WIDTH
#define MPC_HLS_P_WIDTH 40
#endif

#ifndef MPC_HLS_MG_WIDTH
#define MPC_HLS_MG_WIDTH 35
#endif

#ifndef MPC_HLS_S_WIDTH
#define MPC_HLS_S_WIDTH MPC_HLS_QP_WIDTH
#endif

#ifndef MPC_HLS_SI_WIDTH
#define MPC_HLS_SI_WIDTH MPC_HLS_QP_WIDTH
#endif

#ifndef MPC_HLS_K_WIDTH
#define MPC_HLS_K_WIDTH 28
#endif

#ifndef MPC_HLS_P_QP_GUARD
#define MPC_HLS_P_QP_GUARD 21
#endif

#ifndef MPC_HLS_MG_QP_GUARD
#define MPC_HLS_MG_QP_GUARD 18
#endif

#ifndef MPC_HLS_S_QP_GUARD
#define MPC_HLS_S_QP_GUARD MPC_HLS_MG_QP_GUARD
#endif

#ifndef MPC_HLS_SI_MG_GUARD
#define MPC_HLS_SI_MG_GUARD MPC_HLS_MG_QP_GUARD
#endif

#ifndef MPC_HLS_MG_K_GUARD
#define MPC_HLS_MG_K_GUARD 24
#endif

#ifndef MPC_HLS_K_QP_GUARD
#define MPC_HLS_K_QP_GUARD 20
#endif

#define MPC_HLS_P_FRAC_BITS  MPC_HLS_QP_FRAC_BITS
#define MPC_HLS_MG_FRAC_BITS MPC_HLS_QP_FRAC_BITS
#define MPC_HLS_S_FRAC_BITS  MPC_HLS_QP_FRAC_BITS
#define MPC_HLS_SI_FRAC_BITS MPC_HLS_QP_FRAC_BITS
#define MPC_HLS_K_FRAC_BITS  MPC_HLS_QP_FRAC_BITS

#define MPC_HLS_P_INT_BITS  (MPC_HLS_P_WIDTH  - MPC_HLS_P_FRAC_BITS)
#define MPC_HLS_MG_INT_BITS (MPC_HLS_MG_WIDTH - MPC_HLS_MG_FRAC_BITS)
#define MPC_HLS_S_INT_BITS  (MPC_HLS_S_WIDTH  - MPC_HLS_S_FRAC_BITS)
#define MPC_HLS_SI_INT_BITS (MPC_HLS_SI_WIDTH - MPC_HLS_SI_FRAC_BITS)
#define MPC_HLS_K_INT_BITS  (MPC_HLS_K_WIDTH  - MPC_HLS_K_FRAC_BITS)

static_assert(MPC_HLS_P_WIDTH  > MPC_HLS_QP_FRAC_BITS,  "P width too small");
static_assert(MPC_HLS_MG_WIDTH > MPC_HLS_QP_FRAC_BITS,  "MG width too small");
static_assert(MPC_HLS_S_WIDTH  > MPC_HLS_QP_FRAC_BITS,  "S width too small");
static_assert(MPC_HLS_SI_WIDTH > MPC_HLS_QP_FRAC_BITS,  "Si width too small");
static_assert(MPC_HLS_K_WIDTH  > MPC_HLS_QP_FRAC_BITS,  "K width too small");

static_assert(MPC_HLS_P_QP_GUARD  > 0, "P/QP guard must be positive");
static_assert(MPC_HLS_MG_QP_GUARD > 0, "MG/QP guard must be positive");
static_assert(MPC_HLS_S_QP_GUARD  > 0, "S/QP guard must be positive");
static_assert(MPC_HLS_SI_MG_GUARD > 0, "Si/MG guard must be positive");
static_assert(MPC_HLS_MG_K_GUARD  > 0, "MG/K guard must be positive");
static_assert(MPC_HLS_K_QP_GUARD  > 0, "K/QP guard must be positive");

/*==============================================================================
 * Canonical type families
 *============================================================================*/

/* Main QP family */
typedef ap_fixed<MPC_HLS_QP_WIDTH, MPC_HLS_QP_INT_BITS, AP_TRN, AP_WRAP> fp_QP_t;
typedef ap_int<MPC_HLS_QP_WIDTH> fp_QP_raw_t;

/* External payload words are raw QP bits carried in 32-bit OpenCL lanes. */
typedef fp_QP_raw_t fp_stream_raw_t;

/* Internal solver post-shift storage */
typedef ap_int<MPC_HLS_SOLVER_ACC_WIDTH> fp_raw_acc_t;

/* Canonical raw product families */
typedef ap_int<(2 * MPC_HLS_QP_WIDTH)> fp_QP_mul_t;                               // QP * QP
typedef ap_int<(MPC_HLS_SOLVER_ACC_WIDTH + MPC_HLS_QP_WIDTH)> fp_acc_QP_mul_t;   // acc * QP or QP * acc
typedef ap_int<(2 * MPC_HLS_SOLVER_ACC_WIDTH)> fp_acc_mul_t;                      // acc * acc

/* Canonical raw sum families */
typedef ap_int<(2 * MPC_HLS_QP_WIDTH + 1)> fp_sum2_QP_mul_t;
typedef ap_int<(2 * MPC_HLS_QP_WIDTH + 3)> fp_sum6_QP_mul_t;
typedef ap_int<(MPC_HLS_SOLVER_ACC_WIDTH + MPC_HLS_QP_WIDTH + 1)> fp_sum2_acc_QP_t;
typedef ap_int<(MPC_HLS_SOLVER_ACC_WIDTH + MPC_HLS_QP_WIDTH + 3)> fp_sum6_acc_QP_t;

/* FN family */
typedef ap_fixed<MPC_HLS_FN_WIDTH, MPC_HLS_FN_INT_BITS, AP_TRN, AP_WRAP> fp_FN_t;
typedef ap_int<MPC_HLS_FN_WIDTH> fp_fn_raw_t;
typedef ap_int<(MPC_HLS_FN_WIDTH + MPC_HLS_FN_GUARD)> fp_fn_accum_t;

/*==============================================================================
 * Specialized Riccati families
 *============================================================================*/

/* P / p / PA / q_aug / p_shift family */
typedef ap_fixed<MPC_HLS_P_WIDTH, MPC_HLS_P_INT_BITS, AP_TRN, AP_WRAP> fp_P_t;
typedef ap_int<MPC_HLS_P_WIDTH> fp_P_raw_t;

/* M / G / Bp family */
typedef ap_fixed<MPC_HLS_MG_WIDTH, MPC_HLS_MG_INT_BITS, AP_TRN, AP_WRAP> fp_MG_t;
typedef ap_int<MPC_HLS_MG_WIDTH> fp_MG_raw_t;

/* S / Si family */
typedef ap_fixed<MPC_HLS_S_WIDTH, MPC_HLS_S_INT_BITS, AP_TRN, AP_WRAP> fp_S_t;
typedef ap_int<MPC_HLS_S_WIDTH> fp_S_raw_t;
typedef ap_fixed<MPC_HLS_SI_WIDTH, MPC_HLS_SI_INT_BITS, AP_TRN, AP_WRAP> fp_Si_t;
typedef ap_int<MPC_HLS_SI_WIDTH> fp_Si_raw_t;

/* K / kk family */
typedef ap_fixed<MPC_HLS_K_WIDTH, MPC_HLS_K_INT_BITS, AP_TRN, AP_WRAP> fp_K_t;
typedef ap_int<MPC_HLS_K_WIDTH> fp_K_raw_t;

/*------------------------------------------------------------------------------
 * Guard-sized specialized raw product / sum families
 *
 * These widths are intentionally smaller than exact full products. The goal is
 * to match replay-backed Riccati worst cases with margin rather than pay for
 * bit growth that never occurs in practice.
 *----------------------------------------------------------------------------*/

typedef ap_int<(MPC_HLS_P_WIDTH  + MPC_HLS_P_QP_GUARD)>  fp_P_QP_mul_t;   // target 66b
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD)> fp_MG_QP_mul_t;  // target 60b
typedef ap_int<(MPC_HLS_S_WIDTH  + MPC_HLS_S_QP_GUARD)>  fp_S_QP_mul_t;   // target 54b
typedef ap_int<(MPC_HLS_SI_WIDTH + MPC_HLS_SI_MG_GUARD)> fp_Si_MG_mul_t;  // target 51b
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD)>  fp_MG_K_mul_t;   // target 66b
typedef ap_int<(MPC_HLS_K_WIDTH  + MPC_HLS_K_QP_GUARD)>  fp_K_QP_mul_t;   // target 52b

/* Common adder-tree output widths include the extra carry bits required by the
 * actual tree fan-in, but still stay guard-sized rather than exact full-width.
 */
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 3)> fp_sum8_MG_QP_t;
typedef ap_int<(MPC_HLS_P_WIDTH  + MPC_HLS_P_QP_GUARD  + 3)> fp_sum8_P_QP_t;
typedef ap_int<(MPC_HLS_QP_WIDTH + MPC_HLS_MG_QP_GUARD + 1)> fp_sum2_Si_MG_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD  + 1)> fp_sum2_MG_K_t;
typedef ap_int<(MPC_HLS_QP_WIDTH + 1)> fp_sum2_QP_raw_t;
typedef ap_int<(MPC_HLS_P_WIDTH  + 1)> fp_sum2_P_raw_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + 1)> fp_sum2_MG_raw_t;
typedef ap_int<(MPC_HLS_QP_WIDTH + 2)> fp_sum4_QP_raw_t;
typedef ap_int<(MPC_HLS_QP_WIDTH + 3)> fp_sum8_QP_raw_t;

/* Riccati-local sum families used by solver adder trees */
typedef ap_int<(MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD + 3)> fp_sum6_P_QP_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 3)> fp_sum6_MG_QP_t;
typedef ap_int<(MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD + 1)> fp_sum2_P_QP_t;
typedef ap_int<(MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD + 2)> fp_sum4_P_QP_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 1)> fp_sum2_MG_QP_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 2)> fp_sum4_MG_QP_t;
enum {
  MPC_HLS_P_MIX_MUL_WIDTH =
      ((MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD) >
       (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD))
          ? (MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD)
          : (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD)
};
typedef ap_int<(MPC_HLS_P_MIX_MUL_WIDTH + 3)> fp_sum8_P_MIX_t;
typedef ap_int<(MPC_HLS_K_WIDTH + MPC_HLS_K_QP_GUARD + 3)> fp_sum8_K_QP_t;
typedef ap_int<(MPC_HLS_QP_WIDTH + MPC_HLS_MG_QP_GUARD + 1)> fp_sum2_QP_MG_t;

/*==============================================================================
 * Raw QP transport conversion helpers
 *============================================================================*/

static inline fp_QP_t fp_QP_from_raw(fp_stream_raw_t raw) {
#pragma HLS INLINE
  fp_QP_t out = 0;
  out.range(MPC_HLS_QP_WIDTH - 1, 0) = raw.range(MPC_HLS_QP_WIDTH - 1, 0);
  return out;
}

static inline fp_stream_raw_t fp_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  fp_stream_raw_t out = 0;
  out.range(MPC_HLS_QP_WIDTH - 1, 0) = value.range(MPC_HLS_QP_WIDTH - 1, 0);
  return out;
}

static inline fp_QP_raw_t fp_qp_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  fp_QP_raw_t out = 0;
  out.range(MPC_HLS_QP_WIDTH - 1, 0) = value.range(MPC_HLS_QP_WIDTH - 1, 0);
  return out;
}

static inline fp_QP_t fp_QP_from_qp_raw(fp_QP_raw_t raw) {
#pragma HLS INLINE
  fp_QP_t out = 0;
  out.range(MPC_HLS_QP_WIDTH - 1, 0) = raw.range(MPC_HLS_QP_WIDTH - 1, 0);
  return out;
}

static inline fp_raw_acc_t fp_raw_acc_from_qp(fp_QP_t value) {
#pragma HLS INLINE
  return (fp_raw_acc_t)fp_qp_raw_from_QP(value);
}

/*==============================================================================
 * Specialized family raw/fixed conversion helpers
 *============================================================================*/

static inline fp_P_raw_t fp_P_raw_from_P(fp_P_t value) {
#pragma HLS INLINE
  fp_P_raw_t out = 0;
  out.range(MPC_HLS_P_WIDTH - 1, 0) = value.range(MPC_HLS_P_WIDTH - 1, 0);
  return out;
}

static inline fp_P_t fp_P_from_raw(fp_P_raw_t raw) {
#pragma HLS INLINE
  fp_P_t out = 0;
  out.range(MPC_HLS_P_WIDTH - 1, 0) = raw.range(MPC_HLS_P_WIDTH - 1, 0);
  return out;
}

static inline fp_P_raw_t fp_P_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return (fp_P_raw_t)fp_qp_raw_from_QP(value);
}

static inline fp_P_t fp_P_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_P_from_raw(fp_P_raw_from_QP(value));
}

static inline fp_MG_raw_t fp_MG_raw_from_MG(fp_MG_t value) {
#pragma HLS INLINE
  fp_MG_raw_t out = 0;
  out.range(MPC_HLS_MG_WIDTH - 1, 0) = value.range(MPC_HLS_MG_WIDTH - 1, 0);
  return out;
}

static inline fp_MG_t fp_MG_from_raw(fp_MG_raw_t raw) {
#pragma HLS INLINE
  fp_MG_t out = 0;
  out.range(MPC_HLS_MG_WIDTH - 1, 0) = raw.range(MPC_HLS_MG_WIDTH - 1, 0);
  return out;
}

static inline fp_MG_raw_t fp_MG_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return (fp_MG_raw_t)fp_qp_raw_from_QP(value);
}

static inline fp_MG_t fp_MG_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_MG_from_raw(fp_MG_raw_from_QP(value));
}

static inline fp_S_raw_t fp_S_raw_from_S(fp_S_t value) {
#pragma HLS INLINE
  fp_S_raw_t out = 0;
  out.range(MPC_HLS_S_WIDTH - 1, 0) = value.range(MPC_HLS_S_WIDTH - 1, 0);
  return out;
}

static inline fp_S_t fp_S_from_raw(fp_S_raw_t raw) {
#pragma HLS INLINE
  fp_S_t out = 0;
  out.range(MPC_HLS_S_WIDTH - 1, 0) = raw.range(MPC_HLS_S_WIDTH - 1, 0);
  return out;
}

static inline fp_S_raw_t fp_S_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return (fp_S_raw_t)fp_qp_raw_from_QP(value);
}

static inline fp_S_t fp_S_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_S_from_raw(fp_S_raw_from_QP(value));
}

static inline fp_Si_raw_t fp_Si_raw_from_Si(fp_Si_t value) {
#pragma HLS INLINE
  fp_Si_raw_t out = 0;
  out.range(MPC_HLS_SI_WIDTH - 1, 0) = value.range(MPC_HLS_SI_WIDTH - 1, 0);
  return out;
}

static inline fp_Si_t fp_Si_from_raw(fp_Si_raw_t raw) {
#pragma HLS INLINE
  fp_Si_t out = 0;
  out.range(MPC_HLS_SI_WIDTH - 1, 0) = raw.range(MPC_HLS_SI_WIDTH - 1, 0);
  return out;
}

static inline fp_Si_raw_t fp_Si_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return (fp_Si_raw_t)fp_qp_raw_from_QP(value);
}

static inline fp_Si_t fp_Si_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_Si_from_raw(fp_Si_raw_from_QP(value));
}

static inline fp_K_raw_t fp_K_raw_from_K(fp_K_t value) {
#pragma HLS INLINE
  fp_K_raw_t out = 0;
  out.range(MPC_HLS_K_WIDTH - 1, 0) = value.range(MPC_HLS_K_WIDTH - 1, 0);
  return out;
}

static inline fp_K_t fp_K_from_raw(fp_K_raw_t raw) {
#pragma HLS INLINE
  fp_K_t out = 0;
  out.range(MPC_HLS_K_WIDTH - 1, 0) = raw.range(MPC_HLS_K_WIDTH - 1, 0);
  return out;
}

static inline fp_K_raw_t fp_K_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return (fp_K_raw_t)fp_qp_raw_from_QP(value);
}

static inline fp_K_t fp_K_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_K_from_raw(fp_K_raw_from_QP(value));
}

/*==============================================================================
 * Range / casting helpers
 *============================================================================*/

static inline fp_raw_acc_t fp_qp_raw_min_acc() {
#pragma HLS INLINE
  return -(((fp_raw_acc_t)1) << (MPC_HLS_QP_WIDTH - 1));
}

static inline fp_raw_acc_t fp_qp_raw_max_acc() {
#pragma HLS INLINE
  return ((((fp_raw_acc_t)1) << (MPC_HLS_QP_WIDTH - 1)) - 1);
}

static inline fp_raw_acc_t fp_raw_acc_min() {
#pragma HLS INLINE
  return -(((fp_raw_acc_t)1) << (MPC_HLS_SOLVER_ACC_WIDTH - 1));
}

static inline fp_raw_acc_t fp_raw_acc_max() {
#pragma HLS INLINE
  return ((((fp_raw_acc_t)1) << (MPC_HLS_SOLVER_ACC_WIDTH - 1)) - 1);
}

static inline fp_P_raw_t fp_P_raw_min() {
#pragma HLS INLINE
  return -(((fp_P_raw_t)1) << (MPC_HLS_P_WIDTH - 1));
}

static inline fp_P_raw_t fp_P_raw_max() {
#pragma HLS INLINE
  return ((((fp_P_raw_t)1) << (MPC_HLS_P_WIDTH - 1)) - 1);
}

static inline fp_MG_raw_t fp_MG_raw_min() {
#pragma HLS INLINE
  return -(((fp_MG_raw_t)1) << (MPC_HLS_MG_WIDTH - 1));
}

static inline fp_MG_raw_t fp_MG_raw_max() {
#pragma HLS INLINE
  return ((((fp_MG_raw_t)1) << (MPC_HLS_MG_WIDTH - 1)) - 1);
}

static inline fp_S_raw_t fp_S_raw_min() {
#pragma HLS INLINE
  return -(((fp_S_raw_t)1) << (MPC_HLS_S_WIDTH - 1));
}

static inline fp_S_raw_t fp_S_raw_max() {
#pragma HLS INLINE
  return ((((fp_S_raw_t)1) << (MPC_HLS_S_WIDTH - 1)) - 1);
}

static inline fp_Si_raw_t fp_Si_raw_min() {
#pragma HLS INLINE
  return -(((fp_Si_raw_t)1) << (MPC_HLS_SI_WIDTH - 1));
}

static inline fp_Si_raw_t fp_Si_raw_max() {
#pragma HLS INLINE
  return ((((fp_Si_raw_t)1) << (MPC_HLS_SI_WIDTH - 1)) - 1);
}

static inline fp_K_raw_t fp_K_raw_min() {
#pragma HLS INLINE
  return -(((fp_K_raw_t)1) << (MPC_HLS_K_WIDTH - 1));
}

static inline fp_K_raw_t fp_K_raw_max() {
#pragma HLS INLINE
  return ((((fp_K_raw_t)1) << (MPC_HLS_K_WIDTH - 1)) - 1);
}

template <int IN_WIDTH, int OUT_WIDTH>
static inline bool fp_signed_fits_width(ap_int<IN_WIDTH> value) {
#pragma HLS INLINE
  static_assert(IN_WIDTH >= OUT_WIDTH, "input width must be >= output width");
  const int top_width = IN_WIDTH - OUT_WIDTH + 1;
  ap_int<top_width> top = value.range(IN_WIDTH - 1, OUT_WIDTH - 1);
  return (top == 0) || (top == -1);
}

static inline fp_QP_t fp_qp_from_raw_acc(fp_raw_acc_t raw) {
#pragma HLS INLINE
  return fp_QP_from_qp_raw((fp_QP_raw_t)raw);
}

static inline fp_QP_raw_t fp_cast_S_raw_to_qp(fp_S_raw_t value) {
#pragma HLS INLINE
  return (fp_QP_raw_t)value;
}

static inline fp_QP_raw_t fp_cast_Si_raw_to_qp(fp_Si_raw_t value) {
#pragma HLS INLINE
  /* Si is narrower than QP and uses the same fractional scaling.
   * Therefore widening to QP is always safe and should be a sign-extending cast,
   * not a comparison against QP limits cast down into Si width.
   */
  return (fp_QP_raw_t)value;
}

static inline fp_QP_raw_t fp_cast_K_raw_to_qp(fp_K_raw_t value) {
#pragma HLS INLINE
  /* K is narrower than QP and uses the same fractional scaling.
   * Therefore widening to QP is always safe and should be a sign-extending cast,
   * not a comparison against QP limits cast down into K width.
   */
  return (fp_QP_raw_t)value;
}

static inline fp_QP_t fp_qp_from_P_raw(fp_P_raw_t raw) {
#pragma HLS INLINE
  return fp_QP_from_qp_raw((fp_QP_raw_t)raw);
}

static inline fp_QP_t fp_qp_from_MG_raw(fp_MG_raw_t raw) {
#pragma HLS INLINE
  return fp_QP_from_qp_raw((fp_QP_raw_t)raw);
}

static inline fp_QP_t fp_qp_from_K_raw(fp_K_raw_t raw) {
#pragma HLS INLINE
  return fp_QP_from_qp_raw(fp_cast_K_raw_to_qp(raw));
}

static inline fp_QP_raw_t cast_sum2_qp_raw_to_qp_site(fp_sum2_QP_raw_t value,
                                                       int site_id) {
#pragma HLS INLINE
  (void)site_id;
  return (fp_QP_raw_t)value;
}

static inline fp_QP_raw_t cast_sum2_qp_raw_to_qp(fp_sum2_QP_raw_t value) {
#pragma HLS INLINE
  return cast_sum2_qp_raw_to_qp_site(value, FP_CAST_SITE_UNKNOWN);
}

static inline fp_QP_raw_t add_cast_QP_raw(fp_QP_raw_t a, fp_QP_raw_t b) {
#pragma HLS INLINE
  fp_sum2_QP_raw_t sum = (fp_sum2_QP_raw_t)a + (fp_sum2_QP_raw_t)b;
  return cast_sum2_qp_raw_to_qp_site(sum, FP_CAST_SITE_SUM2_ADD_QP_RAW);
}

/*==============================================================================
 * Shift-right + cast helpers
 *============================================================================*/

static inline fp_QP_raw_t fp_shift_right_cast_to_qp_site(fp_QP_mul_t value,
                                                          int shift,
                                                          int site_id) {
#pragma HLS INLINE
  (void)site_id;
  return (fp_QP_raw_t)(value >> shift);
}

static inline fp_QP_raw_t fp_shift_right_cast_to_qp(fp_QP_mul_t value,
                                                    int shift) {
#pragma HLS INLINE
  return fp_shift_right_cast_to_qp_site(value, shift, FP_CAST_SITE_UNKNOWN);
}

/*==============================================================================
 * FN helpers
 *============================================================================*/

static inline fp_fn_raw_t fp_fn_raw_from_FN(fp_FN_t value) {
#pragma HLS INLINE
  fp_fn_raw_t out = 0;
  out.range(MPC_HLS_FN_WIDTH - 1, 0) = value.range(MPC_HLS_FN_WIDTH - 1, 0);
  return out;
}

static inline fp_FN_t fp_FN_from_fn_raw(fp_fn_raw_t raw) {
#pragma HLS INLINE
  fp_FN_t out = 0;
  out.range(MPC_HLS_FN_WIDTH - 1, 0) = raw.range(MPC_HLS_FN_WIDTH - 1, 0);
  return out;
}

static inline fp_FN_t fp_FN_from_QP(fp_QP_t qp_value) {
#pragma HLS INLINE
  return (fp_FN_t)qp_value;
}

static inline fp_QP_t fp_QP_from_FN(fp_FN_t fn_value) {
#pragma HLS INLINE
  return (fp_QP_t)fn_value;
}

#endif  // __cplusplus
#endif  /* FP_TYPES_HLS_HPP */
