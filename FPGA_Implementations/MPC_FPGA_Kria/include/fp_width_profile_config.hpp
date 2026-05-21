#ifndef FP_WIDTH_PROFILE_CONFIG_HPP
#define FP_WIDTH_PROFILE_CONFIG_HPP

/* Width profile selector for non-store intermediate/product/sum families.
 *
 * QP transport/store remains fixed to the external 32-bit payload format.
 * Other stored families may optionally be widened with MPC_HLS_STORE_WIDTH_PAD.
 *
 * Profiles:
 *   1   -> measured minimum-safe widths (observed + 1)
 *   4   -> observed + 4
 *   8   -> observed + 8
 *  -1   -> algebraic worst-case widths
 *
 * Override from the compiler if needed:
 *   -DMPC_HLS_WIDTH_PROFILE=4
 */
#define MPC_HLS_PROFILE_PLUS1 1
#define MPC_HLS_PROFILE_PLUS4 4
#define MPC_HLS_PROFILE_PLUS8 8
#define MPC_HLS_PROFILE_WORST (-1)

/* Optional store widening for replay experiments.
 *
 * This applies only to legal internal stores:
 *   FN, P, MG, K
 *
 * QP store/transport stays fixed to MPC_FPGA_QP_WIDTH.
 *
 * Typical values:
 *   0   -> exact stored widths
 *   4   -> stored widths observed + 4
 */
#define MPC_HLS_STORE_PAD_EXACT 0
#define MPC_HLS_STORE_PAD_PLUS1 1
#define MPC_HLS_STORE_PAD_PLUS4 4

#ifndef MPC_HLS_WIDTH_PROFILE
#define MPC_HLS_WIDTH_PROFILE MPC_HLS_PROFILE_PLUS1
#endif

#ifndef MPC_HLS_STORE_WIDTH_PAD
#define MPC_HLS_STORE_WIDTH_PAD MPC_HLS_STORE_PAD_EXACT
#endif

#endif /* FP_WIDTH_PROFILE_CONFIG_HPP */
