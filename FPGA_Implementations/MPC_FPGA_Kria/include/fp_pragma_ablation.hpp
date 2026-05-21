#ifndef FP_PRAGMA_ABLATION_HPP
#define FP_PRAGMA_ABLATION_HPP

/* Pragma ablation switches for the MPC FPGA build.
 *
 * The PIPELINE and UNROLL pragmas in src/ and include/fp_math_hls.h are
 * routed through these macros so a single dial flips a whole class of
 * directives without removing any pragma from the source tree (the Vitis
 * clang reflow pass has historically crashed when load-bearing PIPELINE
 * pragmas were physically stripped — see memory/vitis-pipeline-pragmas-
 * load-bearing.md).
 *
 * Set MPC_HLS_ABLATION_MODE below (or override with -DMPC_HLS_ABLATION_MODE=N
 * in hls_config.cfg) to one of:
 *
 *   0 = Baseline     -- pipeline + unroll  (the original behavior)
 *   1 = Unroll-only  -- pipelines disabled, unrolls active
 *   2 = Sequential   -- pipelines disabled, unrolls disabled (CPU-like)
 *
 * Pragmas left untouched (orthogonal knobs, not part of this ablation):
 *   INLINE / INLINE off, ARRAY_PARTITION, BIND_STORAGE, BIND_OP,
 *   ALLOCATION, INTERFACE, LOOP_TRIPCOUNT, LOOP_FLATTEN, DISAGGREGATE,
 *   LATENCY.
 */

#define MPC_HLS_ABLATION_BASELINE   0
#define MPC_HLS_ABLATION_UNROLL_ONLY 1
#define MPC_HLS_ABLATION_SEQUENTIAL 2

#ifndef MPC_HLS_ABLATION_MODE
#define MPC_HLS_ABLATION_MODE MPC_HLS_ABLATION_BASELINE
#endif

#define MPC_HLS_STR_(x) #x
#define MPC_HLS_XSTR_(x) MPC_HLS_STR_(x)

#if MPC_HLS_ABLATION_MODE >= MPC_HLS_ABLATION_UNROLL_ONLY
#define MPC_HLS_PIPELINE(N) _Pragma("HLS PIPELINE off")
#else
#define MPC_HLS_PIPELINE(N) _Pragma(MPC_HLS_XSTR_(HLS PIPELINE II = N))
#endif

#if MPC_HLS_ABLATION_MODE >= MPC_HLS_ABLATION_SEQUENTIAL
#define MPC_HLS_UNROLL() _Pragma("HLS UNROLL factor=1")
#define MPC_HLS_UNROLL_FACTOR(N) _Pragma("HLS UNROLL factor=1")
#else
#define MPC_HLS_UNROLL() _Pragma("HLS UNROLL")
#define MPC_HLS_UNROLL_FACTOR(N) _Pragma(MPC_HLS_XSTR_(HLS UNROLL factor = N))
#endif

#endif /* FP_PRAGMA_ABLATION_HPP */
