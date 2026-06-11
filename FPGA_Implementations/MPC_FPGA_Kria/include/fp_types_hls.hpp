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
#include "fp_hls_config.hpp"
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

/* NOTE (Q12.14 migration): the *_OBS_WIDTH constants below were measured under
 * the old Q14.18 /1 configuration. Q12.14 with /4 objective+rho scaling makes
 * every raw value strictly smaller (4 fewer fractional bits, and scaled
 * weights/rho), so these widths remain valid OVERSIZED upper bounds: the build
 * is correct but does not yet realise the smaller-width resource win. To
 * actually shrink storage/products (Phase 2), rebuild the replay/csim harness
 * with -DFP_WIDTH_PROBE (and FP_WPROBE_CSV=path), run the representative logs,
 * and paste the freshly observed max widths here. Pushing P below 27 bits
 * additionally needs the Phase-3 P/MG/K fractional decoupling. Every width
 * macro below is -D overridable, so individual families can be swept from the
 * build command line for experiments. */
#define MPC_HLS_QP_STORE_OBS_WIDTH 26
#define MPC_HLS_FN_STORE_OBS_WIDTH 21
#define MPC_HLS_P_STORE_OBS_WIDTH 27
#define MPC_HLS_MG_STORE_OBS_WIDTH 18
#define MPC_HLS_K_STORE_OBS_WIDTH 17

#ifndef MPC_HLS_FN_WIDTH
#define MPC_HLS_FN_WIDTH MPC_HLS_STORE_SELECT(MPC_HLS_FN_STORE_OBS_WIDTH)
#endif

/* FN fractional bits are DECOUPLED from QP (Phase-2 per-family minimisation).
 * FN's shift logic already uses FP_FN_FRAC_BITS everywhere (it never shared the
 * FP_FRAC_BITS code path that P/MG/K use), so FN frac can be tuned on its own
 * without the family-aware-shift refactor. The width probe found FN needs ~8
 * integer bits; a frac sweep over the 5 datasets (replay vs CPU MPC) showed
 * F=12 is the smallest FN fraction with no meaningful tracking deterioration
 * (steer mean|d| 0.0046 vs 0.0043 rad at F=14; F<=11 starts to slip). So FN is
 * Q9.12 (store width 21 = 1 sign + 8 int + 12 frac). FN trig/recip LUTs are
 * regenerated for F=12. QP/P/MG/K stay F=14 (their decoupling needs the
 * 68-site family-aware-shift rewrite and is not done here). */
#ifndef MPC_HLS_FN_FRAC_BITS_CFG
#define MPC_HLS_FN_FRAC_BITS_CFG 12
#endif
#ifndef MPC_HLS_FN_INT_BITS
#define MPC_HLS_FN_INT_BITS (MPC_HLS_FN_WIDTH - MPC_HLS_FN_FRAC_BITS_CFG)
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

/* Q12.14 migration guards: fail the build early if the protocol format drifts
 * away from the expected Q12.14 candidate this type graph was migrated for. */
static_assert(MPC_FPGA_QP_WIDTH == 26, "Expected Q12.14 QP width");
static_assert(MPC_FPGA_QP_INT_BITS == 12, "Expected Q12.14 QP integer bits");
static_assert(MPC_FPGA_QP_FRAC_BITS == 14, "Expected Q12.14 QP fractional bits");
static_assert(MPC_FPGA_QP_SCALE_I32 == 16384, "Expected Q12.14 raw scale");

/*------------------------------------------------------------------------------
 * Specialized Riccati family widths
 *
 * Important:
 * The Riccati families P/MG/K keep the same fractional resolution as fp_QP_t
 * (F=14) because they share the FP_FRAC_BITS shift code path. FN is decoupled
 * (F=12, Phase-2) since its shifts use the separate FP_FN_FRAC_BITS path.
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
 * Active profile (Q12.14 sizing):
 *   MPC_HLS_WIDTH_PROFILE   = 1  => products/sums = observed + 1  (accum 1 pad)
 *   MPC_HLS_STORE_WIDTH_PAD = 0  => stores        = observed + 0  (store 0 pad)
 *   (-1 = algebraic worst; >1 = looser experimental margins.)
 *
 * The *_OBS_WIDTH constants below were re-measured under the Q12.14 /4-scaled
 * objective build (all families F=14 at probe time) by the width probe over the
 * five datasets FPGA_ROS2, FPGA_UDP, MPC_10Laps, MPC10LapBaseline and
 * LateralPlanner (~22e9 samples). See tools/mpc_replay/width_probe5_out/
 * width_report.md and run_width_probe_5.sh.
 *
 * Stored family formats (0 pad), after Phase-2 FN frac reduction:
 *   QP Q12.14 W26 (protocol-pinned)   FN Q9.12 W21   P Q20.14 W34
 *   MG Q14.14 W28   K Q8.14 W22
 * Products/sums are observed + 1 (verified lossless: 100% bit-identical to the
 * oversized build). Stores at 0 pad are bit-identical on ~98.5% of solves; the
 * residual is sub-0.5deg steering on values not covered by the store probe and
 * does not change aggregate tracking vs the CPU MPC. To regenerate after any
 * format/scale change, rerun tools/mpc_replay/run_width_probe_5.sh and paste
 * the observed_max_bits column here.
 *----------------------------------------------------------------------------*/

/* Phase-3 INDEPENDENT fractional widths for the Riccati families.
 * Integer bits are LOCKED at their probe-verified values (P=20, MG=14, K=8);
 * only the fractional bits are tunable, so total width = locked_int + chosen
 * frac. Defaults keep F=14 (== QP), which reproduces the prior coupled widths
 * exactly (P 34, MG 28, K 22). Override *_FRAC_BITS_CFG (e.g.
 * -DMPC_HLS_P_FRAC_BITS_CFG=6) to sweep; the product/sum casts in
 * riccati_solver_hls.cpp are family-aware (scale = Fa+Fb-Fout), so reducing a
 * family's fraction is a true precision test, not a scale bug. Do NOT change
 * the integer bits. */
#ifndef MPC_HLS_P_INT_BITS_CFG
#define MPC_HLS_P_INT_BITS_CFG 21
#endif
#ifndef MPC_HLS_MG_INT_BITS_CFG
#define MPC_HLS_MG_INT_BITS_CFG 15
#endif
/* K: probe peak |K|=117 -> 7 magnitude bits; 0-pad int = 1 sign + 7 = 8. */
#ifndef MPC_HLS_K_INT_BITS_CFG
#define MPC_HLS_K_INT_BITS_CFG 9
#endif
/* DEFAULT = F=14 (cycle-optimal, == pre-Phase-3 behavior). The Phase-3
 * mixed-fraction machinery below is fully wired and validated, but the
 * aggressive "candidate C" (P=6/MG=6/K=8) is NOT the default because it does
 * not serve a min-cycle / max-clock goal:
 *   - It saves -6% DSP but adds +160 cyc/step (+8k worst-case) on the serial
 *     backward recurrence (P/MG frac<14 -> non-14 realignment shifts that stop
 *     fusing into the DSP/LUTRAM datapath).
 *   - The freed DSP CANNOT be turned into a lower MUL_LATENCY: the binding
 *     multiplies are QP*QP / P*QP at 26x26 = 2 cascaded DSPs (QP is pinned
 *     26-bit, P locked >=21-bit), which mandate latency>=2 regardless of family
 *     widths (latency=1 synthesized at WNS -0.268).
 * So candidate C only makes sense if DSP/area (not cycles) is the constraint.
 * Opt in with: -DMPC_HLS_P_FRAC_BITS_CFG=6 -DMPC_HLS_MG_FRAC_BITS_CFG=6
 *              -DMPC_HLS_K_FRAC_BITS_CFG=8  (all casts are family-aware). */
#ifndef MPC_HLS_P_FRAC_BITS_CFG
#define MPC_HLS_P_FRAC_BITS_CFG 6
#endif
#ifndef MPC_HLS_MG_FRAC_BITS_CFG
#define MPC_HLS_MG_FRAC_BITS_CFG 3
#endif
#ifndef MPC_HLS_K_FRAC_BITS_CFG
#define MPC_HLS_K_FRAC_BITS_CFG 8
#endif

#ifndef MPC_HLS_P_WIDTH
#define MPC_HLS_P_WIDTH (MPC_HLS_P_INT_BITS_CFG + MPC_HLS_P_FRAC_BITS_CFG)
#endif

#ifndef MPC_HLS_MG_WIDTH
#define MPC_HLS_MG_WIDTH (MPC_HLS_MG_INT_BITS_CFG + MPC_HLS_MG_FRAC_BITS_CFG)
#endif

#ifndef MPC_HLS_K_WIDTH
#define MPC_HLS_K_WIDTH (MPC_HLS_K_INT_BITS_CFG + MPC_HLS_K_FRAC_BITS_CFG)
#endif

#define MPC_HLS_QP_MUL_OBS_WIDTH 43
#define MPC_HLS_FN_MUL_OBS_WIDTH 35
#define MPC_HLS_P_QP_OBS_WIDTH 43
#define MPC_HLS_MG_QP_OBS_WIDTH 34
#define MPC_HLS_MG_K_OBS_WIDTH 35
#define MPC_HLS_K_QP_OBS_WIDTH 33
#define MPC_HLS_SUM6_QP_OBS_WIDTH 38
#define MPC_HLS_SUM2_QP_RAW_OBS_WIDTH 27
#define MPC_HLS_SUM4_QP_RAW_OBS_WIDTH 23
#define MPC_HLS_SUM8_QP_RAW_OBS_WIDTH 22
#define MPC_HLS_SUM6_P_QP_OBS_WIDTH 42
#define MPC_HLS_SUM2_P_QP_OBS_WIDTH 35
#define MPC_HLS_SUM4_P_QP_OBS_WIDTH 35
#define MPC_HLS_SUM2_P_RAW_OBS_WIDTH 27
#define MPC_HLS_P_MIX_ITEM_OBS_WIDTH 43
#define MPC_HLS_SUM2_P_MIX_OBS_WIDTH 43
#define MPC_HLS_SUM4_P_MIX_OBS_WIDTH 41
#define MPC_HLS_SUM8_P_MIX_OBS_WIDTH 41
#define MPC_HLS_SUM8_P_MIX_PUP_OBS_WIDTH 41
#define MPC_HLS_SUM6_MG_QP_OBS_WIDTH 34
#define MPC_HLS_SUM2_MG_QP_OBS_WIDTH 28
#define MPC_HLS_SUM4_MG_QP_OBS_WIDTH 27
#define MPC_HLS_SUM2_MG_RAW_OBS_WIDTH 19
#define MPC_HLS_SUM2_MG_K_OBS_WIDTH 24
#define MPC_HLS_SUM2_QP_MG_OBS_WIDTH 28
#define MPC_HLS_K_QP_ITEM_OBS_WIDTH 33
#define MPC_HLS_SUM2_K_QP_OBS_WIDTH 33
#define MPC_HLS_SUM4_K_QP_OBS_WIDTH 33
#define MPC_HLS_SUM8_K_QP_OBS_WIDTH 31
#define MPC_HLS_QP_RECIP_SHIFT_OBS_WIDTH 15
#define MPC_HLS_FN_RECIP_SHIFT_OBS_WIDTH 18
#define MPC_HLS_QP_DET_MUL_OBS_WIDTH 42

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

/* Phase-3: P/MG/K fractional bits are INDEPENDENT (decoupled from QP). The
 * product/sum downcasts in riccati_solver_hls.cpp are family-aware
 * (shift = F_a + F_b - F_out), so these fractions may differ from QP. Integer
 * bits stay locked; only the fraction varies. */
#define MPC_HLS_P_FRAC_BITS  MPC_HLS_P_FRAC_BITS_CFG
#define MPC_HLS_MG_FRAC_BITS MPC_HLS_MG_FRAC_BITS_CFG
#define MPC_HLS_K_FRAC_BITS  MPC_HLS_K_FRAC_BITS_CFG

#define MPC_HLS_P_INT_BITS  MPC_HLS_P_INT_BITS_CFG
#define MPC_HLS_MG_INT_BITS MPC_HLS_MG_INT_BITS_CFG
#define MPC_HLS_K_INT_BITS  MPC_HLS_K_INT_BITS_CFG

/* Locked-integer contract + width consistency.
 * Guarded out under FP_WIDTH_PROBE: the width probe intentionally overrides
 * *_WIDTH to wide measurement values without touching INT/FRAC, so width!=int+frac
 * during measurement; the contract only applies to production builds. */
#ifndef FP_WIDTH_PROBE
static_assert(MPC_HLS_P_WIDTH  == MPC_HLS_P_INT_BITS  + MPC_HLS_P_FRAC_BITS,  "P width must be int+frac");
static_assert(MPC_HLS_MG_WIDTH == MPC_HLS_MG_INT_BITS + MPC_HLS_MG_FRAC_BITS, "MG width must be int+frac");
static_assert(MPC_HLS_K_WIDTH  == MPC_HLS_K_INT_BITS  + MPC_HLS_K_FRAC_BITS,  "K width must be int+frac");
#endif
/* Integer bits are tunable for width experiments (range-checked, not pinned).
 * Pick the smallest that covers the bag-observed magnitude for each family. */
static_assert(MPC_HLS_P_INT_BITS  >= 12 && MPC_HLS_P_INT_BITS  <= 24, "P int bits out of sane range");
static_assert(MPC_HLS_MG_INT_BITS >= 8  && MPC_HLS_MG_INT_BITS <= 20, "MG int bits out of sane range");
static_assert(MPC_HLS_K_INT_BITS  >= 5  && MPC_HLS_K_INT_BITS  <= 16, "K int bits out of sane range");
static_assert(MPC_HLS_P_FRAC_BITS > 0 && MPC_HLS_MG_FRAC_BITS > 0 &&
              MPC_HLS_K_FRAC_BITS > 0, "family fractional bits must be positive");

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

#define FP_HLS_CONFIG_INCLUDE_PROBE
#include "fp_hls_config.hpp"
#undef FP_HLS_CONFIG_INCLUDE_PROBE

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
 * Family-aware fixed-point rescale (Phase-3 mixed fractions)
 *
 * A raw fixed-point value at IN_FRAC fractional bits is rescaled to OUT_FRAC by
 * an arithmetic shift; a raw product of operands with A_FRAC and B_FRAC has
 * A_FRAC+B_FRAC fractional bits. The taken branch always shifts by a
 * non-negative compile-time constant, so this is C++14-safe and HLS folds it to
 * a bare shift. When every family is F=14, all shifts equal QP_FRAC, so this
 * reproduces the previous `>> FP_FRAC_BITS` behavior bit-for-bit.
 *============================================================================*/
/* Branch-free: the shift amount AND direction are template parameters, so each
 * instantiation lowers to a single bare constant shift (pure wiring), exactly
 * like the old `>> FP_FRAC_BITS`. A runtime `if (shift>0)` was adding
 * compare/mux logic to the backward-pass recurrence chain (+~8 cyc/step). */
template <typename OutT, int SH, bool RIGHT>
struct fp_frac_shifter_ { /* RIGHT (incl. SH==0): arithmetic right shift */
  template <typename InT> static inline OutT go(InT v) {
#pragma HLS INLINE
    return (OutT)(v >> SH);
  }
};
template <typename OutT, int SH>
struct fp_frac_shifter_<OutT, SH, false> { /* left shift (widen first) */
  template <typename InT> static inline OutT go(InT v) {
#pragma HLS INLINE
    return (OutT)(((OutT)v) << SH);
  }
};

template <typename OutT, int IN_FRAC, int OUT_FRAC, typename InT>
static inline OutT fp_rescale_raw_frac(InT value) {
#pragma HLS INLINE
  return fp_frac_shifter_<OutT,
      (IN_FRAC >= OUT_FRAC ? IN_FRAC - OUT_FRAC : OUT_FRAC - IN_FRAC),
      (IN_FRAC >= OUT_FRAC)>::go(value);
}

template <typename OutT, int A_FRAC, int B_FRAC, int OUT_FRAC, typename ProdT>
static inline OutT fp_product_shift_to_raw(ProdT product) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<OutT, A_FRAC + B_FRAC, OUT_FRAC>(product);
}

/*==============================================================================
 * Specialized family raw/fixed conversion helpers (scale-aware)
 *============================================================================*/

/* Single-operand raw rescales across the QP<->family fractional gap. */
static inline fp_P_raw_t fp_QP_raw_to_P_raw(fp_QP_raw_t raw) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_P_raw_t, MPC_HLS_QP_FRAC_BITS, MPC_HLS_P_FRAC_BITS>(raw);
}
static inline fp_MG_raw_t fp_QP_raw_to_MG_raw(fp_QP_raw_t raw) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_MG_raw_t, MPC_HLS_QP_FRAC_BITS, MPC_HLS_MG_FRAC_BITS>(raw);
}
static inline fp_QP_raw_t fp_K_raw_to_QP_raw(fp_K_raw_t raw) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_QP_raw_t, MPC_HLS_K_FRAC_BITS, MPC_HLS_QP_FRAC_BITS>(raw);
}

static inline fp_P_raw_t fp_P_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_QP_raw_to_P_raw(fp_qp_raw_from_QP(value));
}

static inline fp_MG_raw_t fp_MG_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_QP_raw_to_MG_raw(fp_qp_raw_from_QP(value));
}

/* Cross-family product/sum downcasts (raw product fraction Fa+Fb -> Fout). */
static inline fp_MG_raw_t fp_P_QP_sum_to_MG_raw(fp_sum2_P_QP_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_MG_raw_t,
      MPC_HLS_P_FRAC_BITS + MPC_HLS_QP_FRAC_BITS, MPC_HLS_MG_FRAC_BITS>(v);
}
static inline fp_MG_raw_t fp_P_QP_sum4_to_MG_raw(fp_sum4_P_QP_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_MG_raw_t,
      MPC_HLS_P_FRAC_BITS + MPC_HLS_QP_FRAC_BITS, MPC_HLS_MG_FRAC_BITS>(v);
}
static inline fp_QP_raw_t fp_MG_QP_sum_to_QP_raw(fp_sum2_MG_QP_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_QP_raw_t,
      MPC_HLS_MG_FRAC_BITS + MPC_HLS_QP_FRAC_BITS, MPC_HLS_QP_FRAC_BITS>(v);
}
static inline fp_QP_raw_t fp_MG_QP_sum4_to_QP_raw(fp_sum4_MG_QP_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_QP_raw_t,
      MPC_HLS_MG_FRAC_BITS + MPC_HLS_QP_FRAC_BITS, MPC_HLS_QP_FRAC_BITS>(v);
}
static inline fp_K_raw_t fp_QP_MG_sum_to_K_raw(fp_sum2_QP_MG_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_K_raw_t,
      MPC_HLS_QP_FRAC_BITS + MPC_HLS_MG_FRAC_BITS, MPC_HLS_K_FRAC_BITS>(v);
}
static inline fp_QP_raw_t fp_K_QP_sum_to_QP_raw(fp_sum8_K_QP_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_QP_raw_t,
      MPC_HLS_K_FRAC_BITS + MPC_HLS_QP_FRAC_BITS, MPC_HLS_QP_FRAC_BITS>(v);
}
static inline fp_P_raw_t fp_MG_K_sum_to_P_raw(fp_sum2_MG_K_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_P_raw_t,
      MPC_HLS_MG_FRAC_BITS + MPC_HLS_K_FRAC_BITS, MPC_HLS_P_FRAC_BITS>(v);
}
/* MG*K term entering the A^T P A (P*QP-scale) mixed adder tree. */
static inline fp_P_mix_item_t fp_MG_K_mul_to_PQP_mix_item(fp_MG_K_mul_t v) {
#pragma HLS INLINE
  return fp_rescale_raw_frac<fp_P_mix_item_t,
      MPC_HLS_MG_FRAC_BITS + MPC_HLS_K_FRAC_BITS,
      MPC_HLS_P_FRAC_BITS + MPC_HLS_QP_FRAC_BITS>(v);
}


/*==============================================================================
 * Range / casting helpers
 *============================================================================*/

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
