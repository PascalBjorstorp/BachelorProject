/**
 * @file fp_types_hls.hpp
 * @brief AP-fixed type families for Kria MPC.
 *
 * IO boundary is fixed to Q16.16 to match the receiver protocol.
 * Internal families are separated into QP and accumulator domains.
 */

#ifndef FP_TYPES_HLS_HPP
#define FP_TYPES_HLS_HPP

#ifdef __cplusplus

#include <ap_fixed.h>
#include <ap_int.h>

/* IO boundary: must remain Q16.16 for protocol compatibility. */
#ifndef MPC_HLS_IO_WIDTH
#define MPC_HLS_IO_WIDTH 32
#endif

#ifndef MPC_HLS_IO_INT_BITS
#define MPC_HLS_IO_INT_BITS 16
#endif

#ifndef MPC_HLS_RICCATI_WIDTH
#define MPC_HLS_RICCATI_WIDTH 32
#endif

#ifndef MPC_HLS_RICCATI_INT_BITS
#define MPC_HLS_RICCATI_INT_BITS 16
#endif

#ifndef MPC_HLS_ACCUM_WIDTH
#define MPC_HLS_ACCUM_WIDTH 24
#endif

#ifndef MPC_HLS_ACCUM_INT_BITS
#define MPC_HLS_ACCUM_INT_BITS 12
#endif

#ifndef MPC_HLS_RAW_ACC_GUARD_BITS
#define MPC_HLS_RAW_ACC_GUARD_BITS 20
#endif

/* IO boundary is FIXED to Q16.16 for protocol compatibility with Jetson/Kria.
   Internal Riccati precision can vary (26-32 bits) for optimization sweeps.
   Both assertions verify valid ranges independently. */
static_assert(MPC_HLS_IO_WIDTH == 32 && MPC_HLS_IO_INT_BITS == 16,
              "IO boundary must stay Q16.16 (32,16) for protocol compatibility");
static_assert(MPC_HLS_IO_WIDTH > MPC_HLS_IO_INT_BITS,
              "MPC_HLS_IO_WIDTH must exceed MPC_HLS_IO_INT_BITS");
static_assert(MPC_HLS_RICCATI_WIDTH > MPC_HLS_RICCATI_INT_BITS,
              "MPC_HLS_RICCATI_WIDTH must exceed MPC_HLS_RICCATI_INT_BITS");
static_assert(MPC_HLS_ACCUM_WIDTH > MPC_HLS_ACCUM_INT_BITS,
              "MPC_HLS_ACCUM_WIDTH must exceed MPC_HLS_ACCUM_INT_BITS");

typedef ap_int<32> fp_stream_raw_t;
typedef ap_fixed<MPC_HLS_IO_WIDTH, MPC_HLS_IO_INT_BITS, AP_TRN, AP_WRAP> fp_io_t;
typedef ap_fixed<MPC_HLS_RICCATI_WIDTH, MPC_HLS_RICCATI_INT_BITS, AP_TRN, AP_WRAP> fp_QP_t;
typedef ap_fixed<MPC_HLS_ACCUM_WIDTH, MPC_HLS_ACCUM_INT_BITS, AP_TRN, AP_WRAP> fp_accum_t;
typedef ap_int<MPC_HLS_RICCATI_WIDTH> fp_qp_raw_t;
typedef ap_int<(MPC_HLS_RICCATI_WIDTH + MPC_HLS_RAW_ACC_GUARD_BITS)> fp_raw_acc_t;

/**
 * @brief Convert packed raw value to IO family type.
 * @param raw Packed stream payload.
 * @return IO family value.
 */
static inline fp_io_t fp_io_from_raw(fp_stream_raw_t raw)
{
#pragma HLS INLINE
    fp_io_t out = 0;
    out.range(31, 0) = raw;
    return out;
}

/**
 * @brief Convert packed raw value to QP family type.
 * @param raw Packed stream payload.
 * @return QP family value.
 */
static inline fp_QP_t fp_QP_from_raw(fp_stream_raw_t raw)
{
#pragma HLS INLINE
    return (fp_QP_t)fp_io_from_raw(raw);
}

static inline fp_qp_raw_t fp_qp_raw_from_QP(fp_QP_t value)
{
#pragma HLS INLINE
    fp_qp_raw_t out;
    out.range(MPC_HLS_RICCATI_WIDTH - 1, 0) = value.range(MPC_HLS_RICCATI_WIDTH - 1, 0);
    return out;
}

static inline fp_QP_t fp_QP_from_qp_raw(fp_qp_raw_t raw)
{
#pragma HLS INLINE
    fp_QP_t out;
    out.range(MPC_HLS_RICCATI_WIDTH - 1, 0) = raw.range(MPC_HLS_RICCATI_WIDTH - 1, 0);
    return out;
}

static inline fp_raw_acc_t fp_raw_acc_from_qp(fp_QP_t value)
{
#pragma HLS INLINE
    return (fp_raw_acc_t)fp_qp_raw_from_QP(value);
}

static inline fp_raw_acc_t fp_qp_raw_min_acc()
{
#pragma HLS INLINE
    return -(((fp_raw_acc_t)1) << (MPC_HLS_RICCATI_WIDTH - 1));
}

static inline fp_raw_acc_t fp_qp_raw_max_acc()
{
#pragma HLS INLINE
    return ((((fp_raw_acc_t)1) << (MPC_HLS_RICCATI_WIDTH - 1)) - 1);
}

static inline fp_raw_acc_t fp_clip_raw_to_qp(fp_raw_acc_t value)
{
#pragma HLS INLINE
    if (value > fp_qp_raw_max_acc()) return fp_qp_raw_max_acc();
    if (value < fp_qp_raw_min_acc()) return fp_qp_raw_min_acc();
    return value;
}

static inline fp_QP_t fp_qp_from_raw_acc(fp_raw_acc_t raw)
{
#pragma HLS INLINE
    fp_raw_acc_t clipped = fp_clip_raw_to_qp(raw);
    return fp_QP_from_qp_raw((fp_qp_raw_t)clipped);
}



/**
 * @brief Convert packed raw value to accumulator family type.
 * @param raw Packed stream payload.
 * @return Accumulator family value.
 */
static inline fp_accum_t fp_accum_from_raw(fp_stream_raw_t raw)
{
#pragma HLS INLINE
    return (fp_accum_t)fp_io_from_raw(raw);
}

/**
 * @brief Convert IO family value to packed raw payload.
 * @param value IO family value.
 * @return Packed raw payload.
 */
static inline fp_stream_raw_t fp_raw_from_io(fp_io_t value)
{
#pragma HLS INLINE
    return (fp_stream_raw_t)value.range(31, 0);
}

/**
 * @brief Convert QP family value to packed raw payload.
 * @param value QP family value.
 * @return Packed raw payload.
 */
static inline fp_stream_raw_t fp_raw_from_QP(fp_QP_t value)
{
#pragma HLS INLINE
    return fp_raw_from_io((fp_io_t)value);
}

/**
 * @brief Convert accumulator family value to packed raw payload.
 * @param value Accumulator family value.
 * @return Packed raw payload.
 */
static inline fp_stream_raw_t fp_raw_from_accum(fp_accum_t value)
{
#pragma HLS INLINE
    return fp_raw_from_io((fp_io_t)value);
}

#endif

#endif /* FP_TYPES_HLS_HPP */
