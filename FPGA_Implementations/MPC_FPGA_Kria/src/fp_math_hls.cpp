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

/* Overloaded fp_mul for QP type (internal Riccati precision).
    When fp_io_t and fp_QP_t are the same type (both Q32.16), this overload
    serves both; no separate fp_io_t overload is defined (see conditional above). */
fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b)
{
#pragma HLS INLINE off
    fp_QP_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    return product;
}

fp_raw_acc_t fp_mul_qp_raw(fp_qp_raw_t a, fp_qp_raw_t b)
{
#pragma HLS INLINE off
    fp_raw_acc_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    return product;
}

fp_raw_acc_t fp_mul_qp_acc(fp_qp_raw_t a, fp_raw_acc_t b)
{
#pragma HLS INLINE off
    fp_raw_acc_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    return product;
}

fp_raw_acc_t fp_mul_acc_qp(fp_raw_acc_t a, fp_qp_raw_t b)
{
#pragma HLS INLINE off
    fp_raw_acc_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    return product;
}

fp_raw_acc_t fp_mul_raw_acc(fp_raw_acc_t a, fp_raw_acc_t b)
{
#pragma HLS INLINE off
    fp_raw_acc_t product = a * b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    return product;
}

fp_raw_acc_t reciprocal_raw(fp_raw_acc_t det)
{
#pragma HLS INLINE off
    if (det == 0) return 0;

    fp_raw_acc_t sign = (det < 0) ? (fp_raw_acc_t)-1 : (fp_raw_acc_t)1;
    fp_raw_acc_t abs_det = (det < 0) ? (fp_raw_acc_t)(-det) : det;

    int lead_zeros = abs_det.countLeadingZeros() + (2 * FP_FRAC_BITS) - FP_RAW_ACC_WIDTH;
    if (lead_zeros < 0) lead_zeros = 0;
    if (lead_zeros > (FP_RAW_ACC_WIDTH - 2)) lead_zeros = (FP_RAW_ACC_WIDTH - 2);

    fp_raw_acc_t est = ((fp_raw_acc_t)1) << lead_zeros;

    fp_raw_acc_t prod0 = abs_det * est;
    #pragma HLS BIND_OP variable=prod0 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    prod0 >>= FP_FRAC_BITS;
    fp_raw_acc_t corr0 = (((fp_raw_acc_t)1) << FP_FRAC_BITS) - prod0;
    fp_raw_acc_t adj0 = est * corr0;
    #pragma HLS BIND_OP variable=adj0 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    adj0 >>= FP_FRAC_BITS;
    est = est + adj0;

    fp_raw_acc_t prod1 = abs_det * est;
    #pragma HLS BIND_OP variable=prod1 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    prod1 >>= FP_FRAC_BITS;
    fp_raw_acc_t corr1 = (((fp_raw_acc_t)1) << FP_FRAC_BITS) - prod1;
    fp_raw_acc_t adj1 = est * corr1;
    #pragma HLS BIND_OP variable=adj1 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    adj1 >>= FP_FRAC_BITS;
    est = est + adj1;

    fp_raw_acc_t prod2 = abs_det * est;
    #pragma HLS BIND_OP variable=prod2 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    prod2 >>= FP_FRAC_BITS;
    fp_raw_acc_t corr2 = (((fp_raw_acc_t)1) << FP_FRAC_BITS) - prod2;
    fp_raw_acc_t adj2 = est * corr2;
    #pragma HLS BIND_OP variable=adj2 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    adj2 >>= FP_FRAC_BITS;
    est = est + adj2;

    if (sign < 0) est = (fp_raw_acc_t)(-est);
    return est;
}

int invert_2x2_hls(fp_raw_acc_t S[2][2], fp_raw_acc_t Si[2][2])
{
#pragma HLS INLINE off
    fp_raw_acc_t det00 = S[0][0] * S[1][1];
    #pragma HLS BIND_OP variable=det00 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    fp_raw_acc_t det01 = S[0][1] * S[1][0];
    #pragma HLS BIND_OP variable=det01 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    fp_raw_acc_t det = (det00 >> FP_FRAC_BITS) - (det01 >> FP_FRAC_BITS);

    fp_raw_acc_t det_eps = (FP_FRAC_BITS >= 12)
        ? (((fp_raw_acc_t)1) << (FP_FRAC_BITS - 12))
        : (fp_raw_acc_t)1;

    if (det == 0 || (det > -det_eps && det < det_eps)) {
        return -1;
    }

    fp_raw_acc_t inv_det = reciprocal_raw(det);

    fp_raw_acc_t si00 = S[1][1] * inv_det;
    #pragma HLS BIND_OP variable=si00 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    Si[0][0] =  si00 >> FP_FRAC_BITS;
    fp_raw_acc_t si01 = S[0][1] * inv_det;
    #pragma HLS BIND_OP variable=si01 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    Si[0][1] = -(si01 >> FP_FRAC_BITS);
    fp_raw_acc_t si10 = S[1][0] * inv_det;
    #pragma HLS BIND_OP variable=si10 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
    Si[1][0] = -(si10 >> FP_FRAC_BITS);
    fp_raw_acc_t si11 = S[0][0] * inv_det;
    #pragma HLS BIND_OP variable=si11 op=mul impl=dsp latency=MPC_HLS_MUL_LATENCY
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
    for (int i = 0; i < 3; i++) {
#pragma HLS UNROLL
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
#pragma HLS INLINE off
#pragma HLS PIPELINE
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
#pragma HLS INLINE off
#pragma HLS PIPELINE
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
 * Cubic atan approximation used throughout the MPC vehicle model.
 *===========================================================================*/

fp_QP_t fp_atan_tire_approx(fp_QP_t x)
{
#pragma HLS INLINE off
    fp_QP_t x2 = fp_mul(x, x);
    fp_QP_t x3 = fp_mul(x2, x);
    fp_QP_t cubic = fp_mul(x3, FP_QP_CONST(0.33333333));
    return x - cubic;
}
