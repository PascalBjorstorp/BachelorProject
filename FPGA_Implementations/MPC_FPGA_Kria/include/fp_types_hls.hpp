/**
 * @file fp_types_hls.hpp
 * @brief Fixed-point type families for Kria MPC.
 *
 * Canonical numeric spaces:
 *   1) IO boundary      : protocol-facing Q16.16
 *   2) QP family        : main stored scalar family for model/QP-facing values
 *   3) Solver internals : wider post-shift storage and exact-width raw products
 *   4) FN family        : Frenet / vehicle-model family
 *
 * There is no legacy compatibility layer in this file.
 */

#ifndef FP_TYPES_HLS_HPP
#define FP_TYPES_HLS_HPP

#ifdef __cplusplus

#include <ap_fixed.h>
#include <ap_int.h>

/*==============================================================================
 * Compile-time configuration
 *============================================================================*/

#ifndef MPC_HLS_IO_WIDTH
#define MPC_HLS_IO_WIDTH 32
#endif

#ifndef MPC_HLS_IO_INT_BITS
#define MPC_HLS_IO_INT_BITS 16
#endif

#ifndef MPC_HLS_QP_WIDTH
#define MPC_HLS_QP_WIDTH 32
#endif

#ifndef MPC_HLS_QP_INT_BITS
#define MPC_HLS_QP_INT_BITS 14
#endif

#ifndef MPC_HLS_SOLVER_ACC_WIDTH
#define MPC_HLS_SOLVER_ACC_WIDTH 48
#endif

#ifndef MPC_HLS_FN_WIDTH
#define MPC_HLS_FN_WIDTH 26
#endif

#ifndef MPC_HLS_FN_INT_BITS
#define MPC_HLS_FN_INT_BITS 9
#endif

#define MPC_HLS_IO_FRAC_BITS (MPC_HLS_IO_WIDTH - MPC_HLS_IO_INT_BITS)
#define MPC_HLS_QP_FRAC_BITS (MPC_HLS_QP_WIDTH - MPC_HLS_QP_INT_BITS)
#define MPC_HLS_FN_FRAC_BITS (MPC_HLS_FN_WIDTH - MPC_HLS_FN_INT_BITS)

/*==============================================================================
 * Canonical type families
 *============================================================================*/

/* IO family */
typedef ap_fixed<MPC_HLS_IO_WIDTH, MPC_HLS_IO_INT_BITS, AP_TRN, AP_WRAP> fp_io_t;
typedef ap_int<MPC_HLS_IO_WIDTH> fp_stream_raw_t;

/* Main QP family */
typedef ap_fixed<MPC_HLS_QP_WIDTH, MPC_HLS_QP_INT_BITS, AP_TRN, AP_WRAP> fp_QP_t;
typedef ap_int<MPC_HLS_QP_WIDTH> fp_QP_raw_t;

/* Internal solver post-shift storage */
typedef ap_int<MPC_HLS_SOLVER_ACC_WIDTH> fp_raw_acc_t;

/* Exact-width raw product families */
typedef ap_int<(2 * MPC_HLS_QP_WIDTH)> fp_QP_mul_t;                               // QP * QP
typedef ap_int<(MPC_HLS_SOLVER_ACC_WIDTH + MPC_HLS_QP_WIDTH)> fp_acc_QP_mul_t;   // acc * QP or QP * acc
typedef ap_int<(2 * MPC_HLS_SOLVER_ACC_WIDTH)> fp_acc_mul_t;                      // acc * acc

/* Exact-width sum families */
typedef ap_int<(2 * MPC_HLS_QP_WIDTH + 1)> fp_sum2_QP_mul_t;
typedef ap_int<(2 * MPC_HLS_QP_WIDTH + 3)> fp_sum6_QP_mul_t;
typedef ap_int<(MPC_HLS_SOLVER_ACC_WIDTH + MPC_HLS_QP_WIDTH + 1)> fp_sum2_acc_QP_t;
typedef ap_int<(MPC_HLS_SOLVER_ACC_WIDTH + MPC_HLS_QP_WIDTH + 3)> fp_sum6_acc_QP_t;

/* FN family */
typedef ap_fixed<MPC_HLS_FN_WIDTH, MPC_HLS_FN_INT_BITS, AP_TRN, AP_WRAP> fp_FN_t;
typedef ap_int<MPC_HLS_FN_WIDTH> fp_fn_raw_t;
typedef ap_int<(2 * MPC_HLS_FN_WIDTH)> fp_fn_accum_t;

/*==============================================================================
 * IO / QP conversion helpers
 *============================================================================*/

static inline fp_io_t fp_io_from_raw(fp_stream_raw_t raw) {
#pragma HLS INLINE
  fp_io_t out = 0;
  out.range(MPC_HLS_IO_WIDTH - 1, 0) = raw.range(MPC_HLS_IO_WIDTH - 1, 0);
  return out;
}

static inline fp_stream_raw_t fp_raw_from_io(fp_io_t value) {
#pragma HLS INLINE
  return (fp_stream_raw_t)value.range(MPC_HLS_IO_WIDTH - 1, 0);
}

static inline fp_QP_t fp_QP_from_raw(fp_stream_raw_t raw) {
#pragma HLS INLINE
  return (fp_QP_t)fp_io_from_raw(raw);
}

static inline fp_stream_raw_t fp_raw_from_QP(fp_QP_t value) {
#pragma HLS INLINE
  return fp_raw_from_io((fp_io_t)value);
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
 * Range / clipping helpers
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

static inline fp_raw_acc_t fp_clip_raw_to_qp(fp_raw_acc_t value) {
#pragma HLS INLINE
  if (value > fp_qp_raw_max_acc())
    return fp_qp_raw_max_acc();
  if (value < fp_qp_raw_min_acc())
    return fp_qp_raw_min_acc();
  return value;
}

static inline fp_QP_t fp_qp_from_raw_acc(fp_raw_acc_t raw) {
#pragma HLS INLINE
  return fp_QP_from_qp_raw((fp_QP_raw_t)fp_clip_raw_to_qp(raw));
}

/*==============================================================================
 * Shift-right + clip helpers
 *============================================================================*/

static inline fp_raw_acc_t fp_shift_right_clip_to_acc(fp_QP_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  fp_QP_mul_t shifted = value >> shift;
  const fp_QP_mul_t max_acc = (fp_QP_mul_t)fp_raw_acc_max();
  const fp_QP_mul_t min_acc = (fp_QP_mul_t)fp_raw_acc_min();
  if (shifted > max_acc)
    return fp_raw_acc_max();
  if (shifted < min_acc)
    return fp_raw_acc_min();
  return (fp_raw_acc_t)shifted;
}

static inline fp_raw_acc_t fp_shift_right_clip_to_acc(fp_acc_QP_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  fp_acc_QP_mul_t shifted = value >> shift;
  const fp_acc_QP_mul_t max_acc = (fp_acc_QP_mul_t)fp_raw_acc_max();
  const fp_acc_QP_mul_t min_acc = (fp_acc_QP_mul_t)fp_raw_acc_min();
  if (shifted > max_acc)
    return fp_raw_acc_max();
  if (shifted < min_acc)
    return fp_raw_acc_min();
  return (fp_raw_acc_t)shifted;
}

static inline fp_raw_acc_t fp_shift_right_clip_to_acc(fp_acc_mul_t value,
                                                      int shift) {
#pragma HLS INLINE
  fp_acc_mul_t shifted = value >> shift;
  const fp_acc_mul_t max_acc = (fp_acc_mul_t)fp_raw_acc_max();
  const fp_acc_mul_t min_acc = (fp_acc_mul_t)fp_raw_acc_min();
  if (shifted > max_acc)
    return fp_raw_acc_max();
  if (shifted < min_acc)
    return fp_raw_acc_min();
  return (fp_raw_acc_t)shifted;
}

static inline fp_QP_raw_t fp_shift_right_clip_to_qp(fp_QP_mul_t value,
                                                    int shift) {
#pragma HLS INLINE
  fp_QP_mul_t shifted = value >> shift;
  const fp_QP_mul_t max_qp = (fp_QP_mul_t)fp_qp_raw_max_acc();
  const fp_QP_mul_t min_qp = (fp_QP_mul_t)fp_qp_raw_min_acc();
  if (shifted > max_qp)
    return (fp_QP_raw_t)fp_qp_raw_max_acc();
  if (shifted < min_qp)
    return (fp_QP_raw_t)fp_qp_raw_min_acc();
  return (fp_QP_raw_t)shifted;
}

static inline fp_QP_raw_t fp_shift_right_clip_to_qp(fp_acc_QP_mul_t value,
                                                    int shift) {
#pragma HLS INLINE
  fp_acc_QP_mul_t shifted = value >> shift;
  const fp_acc_QP_mul_t max_qp = (fp_acc_QP_mul_t)fp_qp_raw_max_acc();
  const fp_acc_QP_mul_t min_qp = (fp_acc_QP_mul_t)fp_qp_raw_min_acc();
  if (shifted > max_qp)
    return (fp_QP_raw_t)fp_qp_raw_max_acc();
  if (shifted < min_qp)
    return (fp_QP_raw_t)fp_qp_raw_min_acc();
  return (fp_QP_raw_t)shifted;
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

static inline fp_fn_raw_t fp_shift_right_clip_to_fn(fp_fn_accum_t value,
                                                    int shift) {
#pragma HLS INLINE
  fp_fn_accum_t shifted = value >> shift;
  const fp_fn_accum_t max_fn =
      ((((fp_fn_accum_t)1) << (MPC_HLS_FN_WIDTH - 1)) - 1);
  const fp_fn_accum_t min_fn =
      -(((fp_fn_accum_t)1) << (MPC_HLS_FN_WIDTH - 1));
  if (shifted > max_fn)
    return (fp_fn_raw_t)max_fn;
  if (shifted < min_fn)
    return (fp_fn_raw_t)min_fn;
  return (fp_fn_raw_t)shifted;
}

#endif  // __cplusplus
#endif  /* FP_TYPES_HLS_HPP */