/**
 * @file fp_math_hls.cpp
 * @brief Q16.16 fixed-point math kernels for HLS synthesis.
 * @details Implements non-inline trigonometric and reciprocal kernels used by
 *          the FPGA MPC pipeline. Arithmetic is performed in fixed-point with
 *          deterministic iteration counts suited for synthesis scheduling.
 * @dependencies fp_math_hls.h
 */

#include "../include/fp_math_hls.h"
#include <cstdint>
#include <climits>

/* When fp_io_t and fp_QP_t are different types (different precisions),
   define an overload for fp_io_t. When they're the same (both Q32.16),
   rely on implicit conversion to avoid redefinition errors. */
#if (MPC_HLS_RICCATI_WIDTH != MPC_HLS_IO_WIDTH) || (MPC_HLS_RICCATI_INT_BITS != MPC_HLS_IO_INT_BITS)
fp_io_t fp_mul(fp_io_t a, fp_io_t b)
{
#pragma HLS INLINE
    fp_io_t product = a * b;
    return product;
}
#endif

/* Overloaded fp_mul for QP type (internal Riccati precision).
    When fp_io_t and fp_QP_t are the same type (both Q32.16), this overload
    serves both; no separate fp_io_t overload is defined (see conditional above). */
fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b)
{
#pragma HLS INLINE off
    fp_QP_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=4
    return product;
}

/* Overloaded fp_mul for accumulator type (distinct from QP domain). */
fp_accum_t fp_mul(fp_accum_t a, fp_accum_t b)
{
#pragma HLS INLINE
    fp_accum_t product = a * b;
    return product;
}

fp_raw_acc_t reciprocal_raw(fp_raw_acc_t det)
{
#pragma HLS INLINE
    if (det == 0) return 0;

    fp_raw_acc_t sign = (det < 0) ? (fp_raw_acc_t)-1 : (fp_raw_acc_t)1;
    fp_raw_acc_t abs_det = (det < 0) ? (fp_raw_acc_t)(-det) : det;

    int lead_zeros = abs_det.countLeadingZeros() + (2 * FP_FRAC_BITS) - FP_RAW_ACC_WIDTH;
    if (lead_zeros < 0) lead_zeros = 0;
    if (lead_zeros > (FP_RAW_ACC_WIDTH - 2)) lead_zeros = (FP_RAW_ACC_WIDTH - 2);

    fp_raw_acc_t est = ((fp_raw_acc_t)1) << lead_zeros;

    for (int i = 0; i < 3; i++) {
#pragma HLS PIPELINE II=6
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
        fp_raw_acc_t prod = abs_det * est;
        prod >>= FP_FRAC_BITS;
        fp_raw_acc_t corr = (((fp_raw_acc_t)1) << FP_FRAC_BITS) - prod;
        fp_raw_acc_t adj  = est * corr;
        adj >>= FP_FRAC_BITS;
        est = est + adj;
    }

    if (sign < 0) est = (fp_raw_acc_t)(-est);
    return est;
}

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2])
{
#pragma HLS INLINE
    fp_raw_acc_t det = ((S[0][0] * S[1][1]) >> FP_FRAC_BITS)
                    - ((S[0][1] * S[1][0]) >> FP_FRAC_BITS);

    fp_raw_acc_t det_eps = (FP_FRAC_BITS >= 12)
        ? (((fp_raw_acc_t)1) << (FP_FRAC_BITS - 12))
        : (fp_raw_acc_t)1;

    if (det == 0 || (det > -det_eps && det < det_eps)) {
        return -1;
    }

    fp_raw_acc_t inv_det = reciprocal_raw(det);

    fp_raw_acc_t si00 = S[1][1] * inv_det;
    Si[0][0] =  si00 >> FP_FRAC_BITS;
    fp_raw_acc_t si01 = S[0][1] * inv_det;
    Si[0][1] = -(si01 >> FP_FRAC_BITS);
    fp_raw_acc_t si10 = S[1][0] * inv_det;
    Si[1][0] = -(si10 >> FP_FRAC_BITS);
    fp_raw_acc_t si11 = S[0][0] * inv_det;
    Si[1][1] =  si11 >> FP_FRAC_BITS;

    return 0;
}

/*===========================================================================
 * Normalize Angle to [-pi, pi]
 *===========================================================================*/

fp_QP_t fp_normalize_angle(fp_QP_t angle)
{
#pragma HLS INLINE
    /* Bounded fp-domain wrap loop avoids int casting for phase reduction. */
    for (int i = 0; i < 4; i++) {
#pragma HLS UNROLL
        if (angle > FP_PI) angle = angle - FP_TWO_PI;
        if (angle < -FP_PI) angle = angle + FP_TWO_PI;
    }

    return angle;
}

/*===========================================================================
 * Reciprocal: 1/x (Newton-Raphson, fixed iteration count)
 *===========================================================================*/

fp_QP_t fp_recip(fp_QP_t x)
{
#pragma HLS INLINE off
    if (x == 0) return 0;

    bool neg = (x < 0);
    fp_QP_t abs_x = fp_abs(x);

    /* Width-aware bit-domain normalization without variable-trip loops. */
    fp_qp_raw_t abs_raw_signed = fp_qp_raw_from_QP(abs_x);
    ap_uint<MPC_HLS_RICCATI_WIDTH> abs_raw = (ap_uint<MPC_HLS_RICCATI_WIDTH>)abs_raw_signed;

    const int one_bit = FP_FRAC_BITS;
    const int half_bit = FP_FRAC_BITS - 1;
    int shift = 0;

    int clz = (int)abs_raw.countLeadingZeros();
    int msb = (MPC_HLS_RICCATI_WIDTH - 1) - clz;

    if (msb > one_bit) {
        shift = msb - one_bit;
    } else if (msb < half_bit) {
        shift = -(half_bit - msb);
    }

    if (shift >= 0) {
        ap_uint<MPC_HLS_RICCATI_WIDTH> right_norm = abs_raw >> shift;
        ap_uint<MPC_HLS_RICCATI_WIDTH> one_raw = ((ap_uint<MPC_HLS_RICCATI_WIDTH>)1) << FP_FRAC_BITS;
        if (right_norm > one_raw && shift < (MPC_HLS_RICCATI_WIDTH - 2)) {
            shift++;
        }
    }

    if (shift > (MPC_HLS_RICCATI_WIDTH - 2)) shift = (MPC_HLS_RICCATI_WIDTH - 2);
    if (shift < -(MPC_HLS_RICCATI_WIDTH - 2)) shift = -(MPC_HLS_RICCATI_WIDTH - 2);

    fp_QP_t x_norm = (shift >= 0) ? (abs_x >> shift) : (abs_x << (-shift));

    fp_QP_t est = FP_QP_CONST(1.5);
    for (int i = 0; i < 4; i++) {
#pragma HLS PIPELINE II=2
#pragma HLS LOOP_TRIPCOUNT min=4 max=4
        est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));
    }

    /* Undo normalization: if x = x_norm * 2^shift, then 1/x = (1/x_norm) * 2^-shift. */
    if (shift > 0) {
        est >>= shift;
    } else if (shift < 0) {
        est <<= (-shift);
    }

    if (neg) est = -est;
    return est;
}

/*===========================================================================
 * Sine: range reduction + truncated Taylor series
 *===========================================================================*/

fp_QP_t fp_sin(fp_QP_t angle)
{
#pragma HLS INLINE
    angle = fp_normalize_angle(angle);

    int negate = 0;
    if (angle > FP_PI_HALF) {
        angle = FP_PI - angle;
    } else if (angle < -FP_PI_HALF) {
        angle = FP_PI + angle;
        negate = 1;
    }

    fp_QP_t x2 = fp_mul(angle, angle);
    fp_QP_t result = angle;
    fp_QP_t term = angle;

    term = fp_mul(term, x2);
    result = result - fp_mul(term, INV_FACT_3);

    term = fp_mul(term, x2);
    result = result + fp_mul(term, INV_FACT_5);

    if (negate) result = -result;
    return result;
}

/*===========================================================================
 * Cosine: range reduction + truncated Taylor series
 *===========================================================================*/

fp_QP_t fp_cos(fp_QP_t angle)
{
#pragma HLS INLINE
    angle = fp_normalize_angle(angle);
    angle = fp_abs(angle);

    int negate = 0;
    if (angle > FP_PI_HALF) {
        angle = FP_PI - angle;
        negate = 1;
    }

    fp_QP_t x2 = fp_mul(angle, angle);
    fp_QP_t result = FP_ONE;
    fp_QP_t term = x2;

    result = result - fp_mul(term, INV_FACT_2);
    term = fp_mul(term, x2);
    result = result + fp_mul(term, INV_FACT_4);

    if (negate) result = -result;
    return result;
}

/*===========================================================================
 * Arctangent helper for |x| <= 0.5
 *===========================================================================*/

static fp_QP_t fp_atan_small(fp_QP_t x)
{
#pragma HLS INLINE
    fp_QP_t x2 = fp_mul(x, x);
    fp_QP_t term = x;
    fp_QP_t result = x;

    term = fp_mul(term, x2);
    result = result - fp_mul(term, ATAN_COEF_3);
    term = fp_mul(term, x2);
    result = result + fp_mul(term, ATAN_COEF_5);
    term = fp_mul(term, x2);
    result = result - fp_mul(term, ATAN_COEF_7);

    return result;
}

/*===========================================================================
 * Arctangent with piecewise range reduction
 *===========================================================================*/

fp_QP_t fp_atan(fp_QP_t x)
{
#pragma HLS INLINE
    if (x == 0) return 0;

    bool neg = (x < 0);
    fp_QP_t abs_x = fp_abs(x);
    fp_QP_t result;

    if (abs_x <= FP_HALF_CONST) {
        result = fp_atan_small(abs_x);
    } else if (abs_x <= FP_ONE) {
        /* atan(x) = atan(0.5) + atan((x-0.5)/(1+0.5*x)) */
        fp_QP_t num = abs_x - FP_HALF_CONST;
        fp_QP_t den = FP_ONE + (abs_x >> 1);  /* 0.5*x via shift */
        fp_QP_t inv_den = fp_recip(den);
        fp_QP_t reduced = fp_mul(num, inv_den);
        result = FP_ATAN_HALF + fp_atan_small(reduced);
    } else {
        /* atan(x) = pi/2 - atan(1/x) */
        fp_QP_t inv_x = fp_recip(abs_x);
        if (inv_x <= FP_HALF_CONST) {
            result = FP_PI_HALF - fp_atan_small(inv_x);
        } else {
            fp_QP_t num = inv_x - FP_HALF_CONST;
            fp_QP_t den = FP_ONE + (inv_x >> 1);  /* 0.5*x via shift */
            fp_QP_t inv_den = fp_recip(den);
            fp_QP_t reduced = fp_mul(num, inv_den);
            fp_QP_t atan_inv = FP_ATAN_HALF + fp_atan_small(reduced);
            result = FP_PI_HALF - atan_inv;
        }
    }

    if (neg) result = -result;
    return result;
}

/*===========================================================================
 * Cubic atan approximation for tire-model angle terms
 *===========================================================================*/

fp_QP_t fp_atan_tire_approx(fp_QP_t x)
{
#pragma HLS INLINE
    fp_QP_t x2 = fp_mul(x, x);
    fp_QP_t x3 = fp_mul(x2, x);
    return x - fp_mul(x3, FP_QP_CONST(0.33333333));
}
