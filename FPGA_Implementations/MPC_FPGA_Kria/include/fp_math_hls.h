/**
 * @file fp_math_hls.h
 * @brief Q16.16 fixed-point math helpers for HLS synthesis.
 */

#ifndef FP_MATH_HLS_H
#define FP_MATH_HLS_H

#include "fp_types_hls.hpp"
#include <cstdint>
#include <climits>

/* Automatic bitwidth adjustment for countLeadingZeros normalization.
 * When casting ap_int<N> to unsigned long long for clz, the value occupies
 * lower N bits [N-1:0], leaving (64-N) unused high bits [63:N].
 * This requires adjustment when switching from __builtin_clzll to countLeadingZeros.
 */
#define FP_RAW_ACC_WIDTH (MPC_HLS_RICCATI_WIDTH + MPC_HLS_RAW_ACC_GUARD_BITS)
#define FP_CLZ_ULL_ADJUSTMENT (64 - FP_RAW_ACC_WIDTH)

/* Fixed-point base constants */
#define FP_FRAC_BITS       (MPC_HLS_RICCATI_WIDTH - MPC_HLS_RICCATI_INT_BITS)
#define FP_IO_CONST(x)     ((fp_io_t)(x))
#define FP_QP_CONST(x)     ((fp_QP_t)(x))
#define FP_ACCUM_CONST(x)  ((fp_accum_t)(x))
#define FP_CONST(x)        FP_QP_CONST(x)
#define FP_ONE             FP_QP_CONST(1.0)
#define FP_TWO             FP_QP_CONST(2.0)
#define FP_HALF            FP_QP_CONST(0.5)
#define FP_PI              FP_QP_CONST(3.14159265358979323846)
#define FP_PI_HALF         FP_QP_CONST(1.57079632679489661923)
#define FP_TWO_PI          FP_QP_CONST(6.28318530717958647693)

#define RECIP_ITERATIONS   3

#define INV_FACT_2         FP_QP_CONST(0.5)
#define INV_FACT_3         FP_QP_CONST(0.16666666666666666)
#define INV_FACT_4         FP_QP_CONST(0.041666666666666664)
#define INV_FACT_5         FP_QP_CONST(0.008333333333333333)

#define ATAN_COEF_3        FP_QP_CONST(0.3333333333333333)
#define ATAN_COEF_5        FP_QP_CONST(0.2)
#define ATAN_COEF_7        FP_QP_CONST(0.14285714285714285)
#define FP_HALF_CONST      FP_HALF
#define FP_ATAN_HALF       FP_QP_CONST(0.4636476090008061)
#define FP_INV_TWO_PI      FP_QP_CONST(0.15915494309189535)

/* Function-based helpers (no define aliases for arithmetic/conversions). */
static inline float FP_TO_FLOAT(fp_QP_t x) { return (float)x; }
static inline fp_QP_t FLOAT_TO_FP(float x) { return (fp_QP_t)x; }
static inline double FP_TO_DOUBLE(fp_QP_t x) { return (double)x; }
static inline fp_QP_t DOUBLE_TO_FP(double x) { return (fp_QP_t)x; }

template <typename T>
static inline T fp_mul_t(T a, T b)
{
    return (T)(a * b);
}

/* Forward declaration used by fp_div to keep slash out of hot call-sites. */
fp_QP_t fp_recip(fp_QP_t x);

/* Implemented out-of-line in fp_math_hls.cpp to enforce a pipelined DSP multiply.
    Note: When fp_io_t and fp_QP_t are identical (both Q32.16), the fp_QP_t overload
    serves both via implicit conversion. No separate fp_io_t overload needed. */

#if (MPC_HLS_RICCATI_WIDTH != MPC_HLS_IO_WIDTH) || (MPC_HLS_RICCATI_INT_BITS != MPC_HLS_IO_INT_BITS)
fp_io_t fp_mul(fp_io_t a, fp_io_t b);      /* Q16.16 I/O multiplication (when distinct from Riccati) */
#endif

fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b);      /* Riccati domain multiplication (DSP-pipelined) */
fp_accum_t fp_mul(fp_accum_t a, fp_accum_t b); /* Accumulator domain multiplication */

static inline fp_QP_t fp_div(fp_QP_t a, fp_QP_t b)
{
    if (a == 0 || b == 0) return 0;
    return fp_mul(a, fp_recip(b));
}

static inline fp_QP_t fp_abs(fp_QP_t a)
{
    if (a < 0) {
        fp_QP_t neg = -a;
        return neg;
    }
    return a;
}

static inline fp_QP_t fp_clamp(fp_QP_t val, fp_QP_t lo, fp_QP_t hi)
{
    if (val < lo) return lo;
    if (val > hi) return hi;
    return val;
}

fp_QP_t fp_normalize_angle(fp_QP_t angle);
fp_QP_t fp_sin(fp_QP_t angle);
fp_QP_t fp_cos(fp_QP_t angle);
fp_QP_t fp_atan(fp_QP_t x);
fp_QP_t fp_atan_tire_approx(fp_QP_t x);

fp_raw_acc_t reciprocal_raw(fp_raw_acc_t det);
int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2]);

#endif
