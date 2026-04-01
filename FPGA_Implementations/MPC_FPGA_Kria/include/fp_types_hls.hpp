/**
 * @file fp_types_hls.hpp
 * @brief Mixed-precision internal fixed-point aliases for HLS builds.
 *
 * This header keeps the external ABI in Q16.16 int32 while enabling
 * optional ap_fixed internal arithmetic in hot paths.
 */

#ifndef FP_TYPES_HLS_HPP
#define FP_TYPES_HLS_HPP

#ifdef __cplusplus

#include <stdint.h>
#include <limits.h>

#if defined(MPC_USE_AP_FIXED)
#include <ap_fixed.h>
#include <ap_int.h>

typedef ap_fixed<32, 16, AP_RND_CONV, AP_SAT> fp_state_t;
typedef ap_fixed<24, 8, AP_RND_CONV, AP_SAT> fp_ctrl_t;
typedef ap_fixed<40, 20, AP_RND_CONV, AP_SAT> fp_cost_t;
typedef ap_fixed<56, 33, AP_RND_CONV, AP_SAT> fp_acc_t;
typedef ap_fixed<28, 6, AP_RND_CONV, AP_SAT> fp_trig_t;
typedef ap_fixed<24, 10, AP_TRN, AP_WRAP> fp_vm_t;
typedef ap_fixed<40, 18, AP_TRN, AP_WRAP> fp_vm_acc_t;
typedef ap_fixed<20, 9, AP_TRN, AP_WRAP> fp_recip_t;
typedef ap_fixed<30, 14, AP_TRN, AP_WRAP> fp_recip_acc_t;

static inline fp_state_t fp_state_from_q16(int32_t raw)
{
#pragma HLS INLINE
    fp_state_t out = 0;
    out.range(31, 0) = (ap_int<32>)raw;
    return out;
}

static inline int32_t fp_q16_from_state(fp_state_t value)
{
#pragma HLS INLINE
    ap_int<32> raw = value.range(31, 0);
    return (int32_t)raw;
}

static inline int32_t fp_q16_mul_backend(int32_t a, int32_t b)
{
#pragma HLS INLINE
    fp_acc_t prod = (fp_acc_t)fp_state_from_q16(a) * (fp_acc_t)fp_state_from_q16(b);
    return fp_q16_from_state((fp_state_t)prod);
}

static inline int32_t fp_q16_div_backend(int32_t a, int32_t b)
{
#pragma HLS INLINE
    if (a == 0 || b == 0) return 0;
    fp_state_t qa = fp_state_from_q16(a);
    fp_state_t qb = fp_state_from_q16(b);
    return fp_q16_from_state((fp_state_t)(qa / qb));
}

#else

typedef int32_t fp_state_t;
typedef int32_t fp_ctrl_t;
typedef int32_t fp_cost_t;
typedef int64_t fp_acc_t;
typedef int32_t fp_trig_t;
typedef int32_t fp_vm_t;
typedef int64_t fp_vm_acc_t;
typedef int32_t fp_recip_t;
typedef int64_t fp_recip_acc_t;

static inline fp_state_t fp_state_from_q16(int32_t raw)
{
    return raw;
}

static inline int32_t fp_q16_from_state(fp_state_t value)
{
    return value;
}

static inline int32_t fp_q16_mul_backend(int32_t a, int32_t b)
{
    return (int32_t)(((int64_t)a * (int64_t)b) >> 16);
}

static inline int32_t fp_q16_div_backend(int32_t a, int32_t b)
{
    if (a == 0 || b == 0) return 0;
    return (int32_t)(((int64_t)a << 16) / (int64_t)b);
}

#endif

#endif

#endif /* FP_TYPES_HLS_HPP */
