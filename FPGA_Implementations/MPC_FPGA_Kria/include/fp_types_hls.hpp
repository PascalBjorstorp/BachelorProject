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
#include "fp_width_profile_config.hpp"
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

#if (MPC_HLS_WIDTH_PROFILE == MPC_HLS_PROFILE_WORST)
#define MPC_HLS_PROFILE_SELECT(obs_bits, worst_bits) (worst_bits)
#elif (MPC_HLS_WIDTH_PROFILE >= 1)
#define MPC_HLS_PROFILE_SELECT(obs_bits, worst_bits) ((obs_bits) + MPC_HLS_WIDTH_PROFILE)
#else
#error "MPC_HLS_WIDTH_PROFILE must be >= 1 or MPC_HLS_PROFILE_WORST (-1)"
#endif

#if (MPC_HLS_STORE_WIDTH_PAD >= 0)
#define MPC_HLS_STORE_SELECT(obs_bits) ((obs_bits) + MPC_HLS_STORE_WIDTH_PAD)
#else
#error "MPC_HLS_STORE_WIDTH_PAD must be >= 0"
#endif

#ifndef MPC_HLS_QP_WIDTH
#define MPC_HLS_QP_WIDTH MPC_FPGA_QP_WIDTH
#endif

#ifndef MPC_HLS_QP_INT_BITS
#define MPC_HLS_QP_INT_BITS MPC_FPGA_QP_INT_BITS
#endif

#ifndef MPC_HLS_QP_GUARD
#define MPC_HLS_QP_GUARD (MPC_HLS_PROFILE_SELECT(MPC_HLS_QP_MUL_OBS_WIDTH, (2 * MPC_HLS_QP_WIDTH)) - MPC_HLS_QP_WIDTH)
#endif

#define MPC_HLS_QP_STORE_OBS_WIDTH 32
#define MPC_HLS_FN_STORE_OBS_WIDTH 26
#define MPC_HLS_P_STORE_OBS_WIDTH 40
#define MPC_HLS_MG_STORE_OBS_WIDTH 34
#define MPC_HLS_K_STORE_OBS_WIDTH 26

#ifndef MPC_HLS_FN_WIDTH
#define MPC_HLS_FN_WIDTH MPC_HLS_STORE_SELECT(MPC_HLS_FN_STORE_OBS_WIDTH)
#endif

#ifndef MPC_HLS_FN_INT_BITS
#define MPC_HLS_FN_INT_BITS 9
#endif

#ifndef MPC_HLS_FN_GUARD
#define MPC_HLS_FN_GUARD (MPC_HLS_PROFILE_SELECT(MPC_HLS_FN_MUL_OBS_WIDTH, (2 * MPC_HLS_FN_WIDTH)) - MPC_HLS_FN_WIDTH)
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
 * Store policy:
 *   - QP store remains fixed to the external payload width
 *   - FN/P/MG/K stores default to exact, but may be widened by
 *     MPC_HLS_STORE_WIDTH_PAD for replay experiments
 *   - S/Si stay QP-collapsed; there is no independent stored S/Si family
 *
 * Product / sum policy:
 *   - raw product widths: observed max + profile padding
 *   - accumulator widths: observed max + profile padding
 *   - where a typedef still serves both ITEM and SUM roles, it uses the
 *     tight common width needed by both
 *
 * Active profile:
 *   MPC_HLS_WIDTH_PROFILE = 1   => observed + 1
 *   MPC_HLS_WIDTH_PROFILE = 4   => observed + 4
 *   MPC_HLS_WIDTH_PROFILE = 8   => observed + 8
 *   MPC_HLS_WIDTH_PROFILE = -1  => algebraic worst case
 *
 * Replay-backed production widths:
 *   QP_GUARD               QP_MUL                            51 -> 52
 *   FN_GUARD               FN_MUL                            44 -> 45
 *   FN_WIDTH               FN_STORE                          26 -> 26
 *   P_WIDTH                P_STORE                           40 -> 40
 *   MG_WIDTH               MG_STORE                          34 -> 34
 *   K_WIDTH                K_STORE                           26 -> 26
 *   P_QP_GUARD            P_QP_MUL                          59 -> 60
 *   MG_QP_GUARD           MG_QP_MUL                         52 -> 53
 *   MG_K_GUARD            MG_K_MUL                          58 -> 59
 *   K_QP_GUARD            K_QP_MUL                          45 -> 46
 *   SUM2_QP_RAW           dense/raw/clamp sum2              31 -> 32
 *   SUM4_QP_RAW           dense/raw stage-2                 25 -> 26
 *   SUM8_QP_RAW           dense/raw final                   24 -> 25
 *   SUM6_QP_MUL_WIDTH     SUM6_QP_tree / QP_ITEM            44 -> 45
 *   SUM2_P_RAW            P-family raw sum2                 39 -> 40
 *   SUM6_P_QP_WIDTH       SUM6_P_QP / P_QP_ITEM             58 -> 59
 *   SUM2_P_QP_WIDTH       P/QP pair sum                     51 -> 52
 *   SUM4_P_QP_WIDTH       P/QP quad sum                     50 -> 51
 *   P_MIX_ITEM_WIDTH      P_MIX_ITEM                        59 -> 60
 *   SUM2_P_MIX_WIDTH      SUM2_P_MIX                        59 -> 60
 *   SUM4_P_MIX_WIDTH      SUM4_P_MIX                        57 -> 58
 *   SUM8_P_MIX_WIDTH      SUM8_P_MIX                        56 -> 57
 *   SUM8_P_MIX_PUP_WIDTH  SUM8_P_MIX_pupdate                57 -> 58
 *   SUM2_MG_RAW           MG-family raw sum2                33 -> 34
 *   SUM6_MG_QP_WIDTH      SUM6_MG_QP / MG_QP_ITEM           52 -> 53
 *   SUM2_MG_QP_WIDTH      MG/QP pair sum                    45 -> 46
 *   SUM4_MG_QP_WIDTH      MG/QP quad sum                    44 -> 45
 *   SUM2_QP_MG_WIDTH      QP*MG pair sum                    44 -> 45
 *   SUM2_MG_K_WIDTH       MG*K pair sum                     48 -> 49
 *   K_QP_ITEM_WIDTH       K_QP item                         45 -> 46
 *   SUM2_K_QP_WIDTH       K_QP pair sum                     44 -> 45
 *   SUM4_K_QP_WIDTH       K_QP quad sum                     44 -> 45
 *   SUM8_K_QP_WIDTH       K_QP final sum                    43 -> 44
 *   QP_RECIP_SHIFT_WIDTH  reciprocal de-normalize shift     25 -> 26
 *   FN_RECIP_SHIFT_WIDTH  reciprocal de-normalize shift     21 -> 22
 *   QP_DET_MUL_WIDTH      2x2 inverse determinant subtract  50 -> 51
 *----------------------------------------------------------------------------*/

#ifndef MPC_HLS_P_WIDTH
#define MPC_HLS_P_WIDTH MPC_HLS_STORE_SELECT(MPC_HLS_P_STORE_OBS_WIDTH)
#endif

#ifndef MPC_HLS_MG_WIDTH
#define MPC_HLS_MG_WIDTH MPC_HLS_STORE_SELECT(MPC_HLS_MG_STORE_OBS_WIDTH)
#endif

#ifndef MPC_HLS_K_WIDTH
#define MPC_HLS_K_WIDTH MPC_HLS_STORE_SELECT(MPC_HLS_K_STORE_OBS_WIDTH)
#endif

#define MPC_HLS_QP_MUL_OBS_WIDTH 51
#define MPC_HLS_FN_MUL_OBS_WIDTH 44
#define MPC_HLS_P_QP_OBS_WIDTH 59
#define MPC_HLS_MG_QP_OBS_WIDTH 52
#define MPC_HLS_MG_K_OBS_WIDTH 58
#define MPC_HLS_K_QP_OBS_WIDTH 45
#define MPC_HLS_SUM6_QP_OBS_WIDTH 44
#define MPC_HLS_SUM2_QP_RAW_OBS_WIDTH 31
#define MPC_HLS_SUM4_QP_RAW_OBS_WIDTH 25
#define MPC_HLS_SUM8_QP_RAW_OBS_WIDTH 24
#define MPC_HLS_SUM6_P_QP_OBS_WIDTH 58
#define MPC_HLS_SUM2_P_QP_OBS_WIDTH 51
#define MPC_HLS_SUM4_P_QP_OBS_WIDTH 50
#define MPC_HLS_SUM2_P_RAW_OBS_WIDTH 39
#define MPC_HLS_P_MIX_ITEM_OBS_WIDTH 59
#define MPC_HLS_SUM2_P_MIX_OBS_WIDTH 59
#define MPC_HLS_SUM4_P_MIX_OBS_WIDTH 57
#define MPC_HLS_SUM8_P_MIX_OBS_WIDTH 56
#define MPC_HLS_SUM8_P_MIX_PUP_OBS_WIDTH 57
#define MPC_HLS_SUM6_MG_QP_OBS_WIDTH 52
#define MPC_HLS_SUM2_MG_QP_OBS_WIDTH 45
#define MPC_HLS_SUM4_MG_QP_OBS_WIDTH 44
#define MPC_HLS_SUM2_MG_RAW_OBS_WIDTH 33
#define MPC_HLS_SUM2_MG_K_OBS_WIDTH 48
#define MPC_HLS_SUM2_QP_MG_OBS_WIDTH 44
#define MPC_HLS_K_QP_ITEM_OBS_WIDTH 45
#define MPC_HLS_SUM2_K_QP_OBS_WIDTH 44
#define MPC_HLS_SUM4_K_QP_OBS_WIDTH 44
#define MPC_HLS_SUM8_K_QP_OBS_WIDTH 43
#define MPC_HLS_QP_RECIP_SHIFT_OBS_WIDTH 25
#define MPC_HLS_FN_RECIP_SHIFT_OBS_WIDTH 21
#define MPC_HLS_QP_DET_MUL_OBS_WIDTH 50

#ifndef MPC_HLS_P_QP_GUARD
#define MPC_HLS_P_QP_GUARD (MPC_HLS_PROFILE_SELECT(MPC_HLS_P_QP_OBS_WIDTH, (MPC_HLS_P_WIDTH + MPC_HLS_QP_WIDTH)) - MPC_HLS_P_WIDTH)
#endif

#ifndef MPC_HLS_MG_QP_GUARD
#define MPC_HLS_MG_QP_GUARD (MPC_HLS_PROFILE_SELECT(MPC_HLS_MG_QP_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_QP_WIDTH)) - MPC_HLS_MG_WIDTH)
#endif

#ifndef MPC_HLS_MG_K_GUARD
#define MPC_HLS_MG_K_GUARD (MPC_HLS_PROFILE_SELECT(MPC_HLS_MG_K_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_K_WIDTH)) - MPC_HLS_MG_WIDTH)
#endif

#ifndef MPC_HLS_K_QP_GUARD
#define MPC_HLS_K_QP_GUARD (MPC_HLS_PROFILE_SELECT(MPC_HLS_K_QP_OBS_WIDTH, (MPC_HLS_K_WIDTH + MPC_HLS_QP_WIDTH)) - MPC_HLS_K_WIDTH)
#endif

#ifndef MPC_HLS_SUM6_QP_MUL_WIDTH
#define MPC_HLS_SUM6_QP_MUL_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM6_QP_OBS_WIDTH, (MPC_HLS_QP_WIDTH + MPC_HLS_QP_GUARD + 3))
#endif

#ifndef MPC_HLS_SUM2_QP_RAW_WIDTH
#define MPC_HLS_SUM2_QP_RAW_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_QP_RAW_OBS_WIDTH, (MPC_HLS_QP_WIDTH + 2))
#endif

#ifndef MPC_HLS_SUM4_QP_RAW_WIDTH
#define MPC_HLS_SUM4_QP_RAW_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM4_QP_RAW_OBS_WIDTH, (MPC_HLS_QP_WIDTH + 3))
#endif

#ifndef MPC_HLS_SUM8_QP_RAW_WIDTH
#define MPC_HLS_SUM8_QP_RAW_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM8_QP_RAW_OBS_WIDTH, (MPC_HLS_QP_WIDTH + 4))
#endif

#ifndef MPC_HLS_SUM6_P_QP_WIDTH
#define MPC_HLS_SUM6_P_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM6_P_QP_OBS_WIDTH, (MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD + 3))
#endif

#ifndef MPC_HLS_SUM2_P_QP_WIDTH
#define MPC_HLS_SUM2_P_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_P_QP_OBS_WIDTH, (MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD + 1))
#endif

#ifndef MPC_HLS_SUM4_P_QP_WIDTH
#define MPC_HLS_SUM4_P_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM4_P_QP_OBS_WIDTH, (MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD + 2))
#endif

#ifndef MPC_HLS_SUM2_P_RAW_WIDTH
#define MPC_HLS_SUM2_P_RAW_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_P_RAW_OBS_WIDTH, (MPC_HLS_P_WIDTH + 1))
#endif

#ifndef MPC_HLS_P_MIX_ITEM_WIDTH
#define MPC_HLS_P_MIX_ITEM_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_P_MIX_ITEM_OBS_WIDTH, (((MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD) > (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD)) ? (MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD) : (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD)))
#endif

#ifndef MPC_HLS_SUM2_P_MIX_WIDTH
#define MPC_HLS_SUM2_P_MIX_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_P_MIX_OBS_WIDTH, (MPC_HLS_P_MIX_ITEM_WIDTH + 1))
#endif

#ifndef MPC_HLS_SUM4_P_MIX_WIDTH
#define MPC_HLS_SUM4_P_MIX_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM4_P_MIX_OBS_WIDTH, (MPC_HLS_P_MIX_ITEM_WIDTH + 2))
#endif

#ifndef MPC_HLS_SUM8_P_MIX_WIDTH
#define MPC_HLS_SUM8_P_MIX_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM8_P_MIX_OBS_WIDTH, (MPC_HLS_P_MIX_ITEM_WIDTH + 3))
#endif

#ifndef MPC_HLS_SUM8_P_MIX_PUP_WIDTH
#define MPC_HLS_SUM8_P_MIX_PUP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM8_P_MIX_PUP_OBS_WIDTH, (MPC_HLS_P_MIX_ITEM_WIDTH + 3))
#endif

#ifndef MPC_HLS_SUM6_MG_QP_WIDTH
#define MPC_HLS_SUM6_MG_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM6_MG_QP_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 3))
#endif

#ifndef MPC_HLS_SUM2_MG_QP_WIDTH
#define MPC_HLS_SUM2_MG_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_MG_QP_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 1))
#endif

#ifndef MPC_HLS_SUM4_MG_QP_WIDTH
#define MPC_HLS_SUM4_MG_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM4_MG_QP_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 2))
#endif

#ifndef MPC_HLS_SUM2_MG_RAW_WIDTH
#define MPC_HLS_SUM2_MG_RAW_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_MG_RAW_OBS_WIDTH, (MPC_HLS_MG_WIDTH + 1))
#endif

#ifndef MPC_HLS_SUM2_MG_K_WIDTH
#define MPC_HLS_SUM2_MG_K_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_MG_K_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD + 1))
#endif

#ifndef MPC_HLS_SUM2_QP_MG_WIDTH
#define MPC_HLS_SUM2_QP_MG_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_QP_MG_OBS_WIDTH, (MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD + 1))
#endif

#ifndef MPC_HLS_K_QP_ITEM_WIDTH
#define MPC_HLS_K_QP_ITEM_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_K_QP_ITEM_OBS_WIDTH, (MPC_HLS_K_WIDTH + MPC_HLS_K_QP_GUARD))
#endif

#ifndef MPC_HLS_SUM2_K_QP_WIDTH
#define MPC_HLS_SUM2_K_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM2_K_QP_OBS_WIDTH, (MPC_HLS_K_QP_ITEM_WIDTH + 1))
#endif

#ifndef MPC_HLS_SUM4_K_QP_WIDTH
#define MPC_HLS_SUM4_K_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM4_K_QP_OBS_WIDTH, (MPC_HLS_K_QP_ITEM_WIDTH + 2))
#endif

#ifndef MPC_HLS_SUM8_K_QP_WIDTH
#define MPC_HLS_SUM8_K_QP_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_SUM8_K_QP_OBS_WIDTH, (MPC_HLS_K_QP_ITEM_WIDTH + 3))
#endif

#ifndef MPC_HLS_QP_RECIP_SHIFT_WIDTH
#define MPC_HLS_QP_RECIP_SHIFT_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_QP_RECIP_SHIFT_OBS_WIDTH, (MPC_HLS_QP_WIDTH + MPC_HLS_QP_FRAC_BITS))
#endif

#ifndef MPC_HLS_FN_RECIP_SHIFT_WIDTH
#define MPC_HLS_FN_RECIP_SHIFT_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_FN_RECIP_SHIFT_OBS_WIDTH, (MPC_HLS_FN_WIDTH + MPC_HLS_FN_FRAC_BITS))
#endif

#ifndef MPC_HLS_QP_DET_MUL_WIDTH
#define MPC_HLS_QP_DET_MUL_WIDTH MPC_HLS_PROFILE_SELECT(MPC_HLS_QP_DET_MUL_OBS_WIDTH, ((2 * MPC_HLS_QP_WIDTH) + 1))
#endif

#define MPC_HLS_P_FRAC_BITS  MPC_HLS_QP_FRAC_BITS
#define MPC_HLS_MG_FRAC_BITS MPC_HLS_QP_FRAC_BITS
#define MPC_HLS_K_FRAC_BITS  MPC_HLS_QP_FRAC_BITS

#define MPC_HLS_P_INT_BITS  (MPC_HLS_P_WIDTH  - MPC_HLS_P_FRAC_BITS)
#define MPC_HLS_MG_INT_BITS (MPC_HLS_MG_WIDTH - MPC_HLS_MG_FRAC_BITS)
#define MPC_HLS_K_INT_BITS  (MPC_HLS_K_WIDTH  - MPC_HLS_K_FRAC_BITS)

static_assert(MPC_HLS_P_WIDTH  > MPC_HLS_QP_FRAC_BITS,  "P width too small");
static_assert(MPC_HLS_MG_WIDTH > MPC_HLS_QP_FRAC_BITS,  "MG width too small");
static_assert(MPC_HLS_K_WIDTH  > MPC_HLS_QP_FRAC_BITS,  "K width too small");

static_assert(MPC_HLS_P_QP_GUARD  > 0, "P/QP guard must be positive");
static_assert(MPC_HLS_MG_QP_GUARD > 0, "MG/QP guard must be positive");
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

/* Canonical raw product families */
typedef ap_int<(MPC_HLS_QP_WIDTH + MPC_HLS_QP_GUARD)> fp_QP_mul_t;

/* Canonical raw sum families */
typedef ap_int<MPC_HLS_SUM6_QP_MUL_WIDTH> fp_sum6_QP_mul_t;

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

typedef ap_int<(MPC_HLS_P_WIDTH  + MPC_HLS_P_QP_GUARD)>  fp_P_QP_mul_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_QP_GUARD)> fp_MG_QP_mul_t;
typedef ap_int<(MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD)>  fp_MG_K_mul_t;
typedef ap_int<(MPC_HLS_K_WIDTH  + MPC_HLS_K_QP_GUARD)>  fp_K_QP_mul_t;

typedef ap_int<MPC_HLS_SUM2_QP_RAW_WIDTH> fp_sum2_QP_raw_t;
typedef ap_int<MPC_HLS_SUM4_QP_RAW_WIDTH> fp_sum4_QP_raw_t;
typedef ap_int<MPC_HLS_SUM8_QP_RAW_WIDTH> fp_sum8_QP_raw_t;
typedef ap_int<MPC_HLS_SUM2_P_RAW_WIDTH> fp_sum2_P_raw_t;
typedef ap_int<MPC_HLS_SUM2_MG_RAW_WIDTH> fp_sum2_MG_raw_t;
typedef ap_int<MPC_HLS_SUM6_P_QP_WIDTH> fp_sum6_P_QP_t;
typedef ap_int<MPC_HLS_SUM6_MG_QP_WIDTH> fp_sum6_MG_QP_t;
typedef ap_int<MPC_HLS_SUM2_P_QP_WIDTH> fp_sum2_P_QP_t;
typedef ap_int<MPC_HLS_SUM4_P_QP_WIDTH> fp_sum4_P_QP_t;
typedef ap_int<MPC_HLS_SUM2_MG_QP_WIDTH> fp_sum2_MG_QP_t;
typedef ap_int<MPC_HLS_SUM4_MG_QP_WIDTH> fp_sum4_MG_QP_t;
typedef ap_int<MPC_HLS_SUM2_MG_K_WIDTH> fp_sum2_MG_K_t;
enum {
  MPC_HLS_P_MIX_MUL_WIDTH =
      ((MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD) >
       (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD))
          ? (MPC_HLS_P_WIDTH + MPC_HLS_P_QP_GUARD)
          : (MPC_HLS_MG_WIDTH + MPC_HLS_MG_K_GUARD)
};
typedef ap_int<MPC_HLS_P_MIX_ITEM_WIDTH> fp_P_mix_item_t;
typedef ap_int<MPC_HLS_SUM2_P_MIX_WIDTH> fp_sum2_P_MIX_t;
typedef ap_int<MPC_HLS_SUM4_P_MIX_WIDTH> fp_sum4_P_MIX_t;
typedef ap_int<MPC_HLS_SUM8_P_MIX_WIDTH> fp_sum8_P_MIX_t;
typedef ap_int<MPC_HLS_SUM8_P_MIX_PUP_WIDTH> fp_sum8_P_MIX_pup_t;
typedef ap_int<MPC_HLS_K_QP_ITEM_WIDTH> fp_K_qp_item_t;
typedef ap_int<MPC_HLS_SUM2_K_QP_WIDTH> fp_sum2_K_QP_t;
typedef ap_int<MPC_HLS_SUM4_K_QP_WIDTH> fp_sum4_K_QP_t;
typedef ap_int<MPC_HLS_SUM8_K_QP_WIDTH> fp_sum8_K_QP_t;
typedef ap_int<MPC_HLS_SUM2_QP_MG_WIDTH> fp_sum2_QP_MG_t;
typedef ap_int<MPC_HLS_QP_RECIP_SHIFT_WIDTH> fp_QP_recip_shift_t;
typedef ap_int<MPC_HLS_FN_RECIP_SHIFT_WIDTH> fp_FN_recip_shift_t;
typedef ap_int<MPC_HLS_QP_DET_MUL_WIDTH> fp_QP_det_mul_t;

#include "fp_width_probe.hpp"

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
  FP_WPROBE_STORE(FP_WP_QP_STORE, out.to_int64(), MPC_HLS_QP_FRAC_BITS);
  return out;
}

static inline fp_QP_t fp_QP_from_qp_raw(fp_QP_raw_t raw) {
#pragma HLS INLINE
  fp_QP_t out = 0;
  out.range(MPC_HLS_QP_WIDTH - 1, 0) = raw.range(MPC_HLS_QP_WIDTH - 1, 0);
  return out;
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

static inline fp_QP_raw_t fp_cast_K_raw_to_qp(fp_K_raw_t value) {
#pragma HLS INLINE
  /* K is narrower than QP and uses the same fractional scaling.
   * Therefore widening to QP is always safe and should be a sign-extending cast,
   * not a comparison against QP limits cast down into K width.
   */
  return (fp_QP_raw_t)value;
}

static inline fp_QP_raw_t cast_sum2_qp_raw_to_qp_site(fp_sum2_QP_raw_t value,
                                                       int site_id) {
#pragma HLS INLINE
  (void)site_id;
  FP_WPROBE(FP_WP_SUM2_QP_RAW, value.to_int64());
  return (fp_QP_raw_t)value;
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
  FP_WPROBE_STORE(FP_WP_FN_STORE, out.to_int64(), MPC_HLS_FN_FRAC_BITS);
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
