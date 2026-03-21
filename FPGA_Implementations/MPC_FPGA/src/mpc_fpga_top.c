/**
 * @file mpc_fpga_top.c
 * @brief Top-Level HLS Function with AXI-Stream + AXI-Lite Interface
 *
 * This is the synthesizable top function for the MPC FPGA IP core.
 * It wraps the MPC Riccati-ADMM solver with:
 *   - AXI-Stream input for state + reference data (fastest transfer)
 *   - AXI-Lite registers for control and output
 *
 * Stream Format (128-bit words):
 *   Beat 0: [e_y | e_psi | vx | vy]
 *   Beat 1: [omega | steering | horizon_length | reserved]
 *   Beat 2..N+1: [ref_vx[i] | ref_kappa[i] | ref_left[i] | ref_right[i]]
 *
 * Total: 2 + MPC_HORIZON beats = 21 beats for 19-point horizon
 */

#include "../include/fp_math_hls.h"
#include "../include/mpc_fpga_types.h"
#include "../include/riccati_solver_hls.h"

#ifdef __SYNTHESIS__
#include <hls_stream.h>
#include <ap_int.h>
#include <ap_axi_sdata.h>
#endif

/* Stream data type: 128 bits = 4 × 32-bit values */
typedef ap_uint<128> stream_word_t;
typedef hls::axis<stream_word_t, 0, 0, 0> axis_word_t;

/* Forward declarations */
extern void mpc_compute_hls(
    fixed_point_t state_ey,
    fixed_point_t state_epsi,
    fixed_point_t state_vx,
    fixed_point_t state_vy,
    fixed_point_t state_omega,
    const MpcRefPoint_t ref[MPC_HORIZON],
    MpcPersistState_t *persist,
    AdmmState_t *admm_state,
    fixed_point_t *out_steering,
    fixed_point_t *out_accel,
    int *out_status,
    int *out_iters);

/**
 * MPC FPGA Top-Level Function with AXI-Stream input.
 */
void mpc_fpga_top(
    /* AXI-Stream input: state + horizon data */
    hls::stream<axis_word_t>& input_stream,
    
    /* Outputs via AXI-Lite registers */
    int *out_steering_fp,
    int *out_accel_fp,
    int *out_status,
    int *out_iterations)
{
    /* ===== Interface Pragmas ===== */
#pragma HLS INTERFACE axis port=input_stream
#pragma HLS INTERFACE s_axilite port=return bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_accel_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_status bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_iterations bundle=ctrl

#pragma HLS ALLOCATION operation instances=mul limit=2

    /* ===== Static Persistent State (survives between calls) ===== */
    static AdmmState_t admm_state;
#pragma HLS BIND_STORAGE variable=admm_state type=ram_2p impl=bram

    static MpcPersistState_t persist;
#pragma HLS BIND_STORAGE variable=persist type=register
    static int first_call = 1;

    /* ===== Initialize on first call ===== */
    if (first_call) {
        persist.prev_steer_rate = 0;
        persist.prev_accel = 0;
        persist.prev_delta_cmd = 0;
        persist.actual_steering = 0;
        persist.prev_curvature = 0;
        persist.prev_converged = 0;
        admm_state.initialized = 0;
        first_call = 0;
    }

    /* ===== Read input stream ===== */
    
    /* Beat 0: State part 1 [e_y | e_psi | vx | vy] */
    axis_word_t beat0 = input_stream.read();
    stream_word_t data0 = beat0.data;
    fixed_point_t e_y   = (fixed_point_t)(int)(data0.range(31, 0));
    fixed_point_t e_psi = fp_normalize_angle((fixed_point_t)(int)(data0.range(63, 32)));
    fixed_point_t vx    = (fixed_point_t)(int)(data0.range(95, 64));
    fixed_point_t vy    = (fixed_point_t)(int)(data0.range(127, 96));
    
    /* Beat 1: State part 2 [omega | steering | horizon_length | reserved] */
    axis_word_t beat1 = input_stream.read();
    stream_word_t data1 = beat1.data;
    fixed_point_t omega    = (fixed_point_t)(int)(data1.range(31, 0));
    fixed_point_t steering = (fixed_point_t)(int)(data1.range(63, 32));
    int horizon_length     = (int)(data1.range(95, 64));
    /* Reserved: data1.range(127, 96) */
    
    /* Update actual steering from measurement */
    persist.actual_steering = steering;
    
    /* Validate horizon */
    int horizon_size = horizon_length;
    if (horizon_size <= 0) {
        *out_steering_fp = 0;
        *out_accel_fp    = 0;
        *out_status      = 3;  /* NO_TRAJECTORY */
        *out_iterations  = 0;
        /* Drain remaining stream data if any */
        return;
    }
    if (horizon_size > MPC_HORIZON) horizon_size = MPC_HORIZON;
    
    /* Beat 2..N+1: Reference trajectory */
    MpcRefPoint_t ref[MPC_HORIZON];
#pragma HLS ARRAY_PARTITION variable=ref complete dim=0

    int k;
    for (k = 0; k < MPC_HORIZON; k++) {
#pragma HLS PIPELINE II=1
        axis_word_t beat_ref = input_stream.read();
        stream_word_t data_ref = beat_ref.data;
        
        /* Extract 4 × 32-bit values from 128-bit word */
        ref[k].velocity    = (fixed_point_t)(int)(data_ref.range(31, 0));
        
        fixed_point_t kappa = (fixed_point_t)(int)(data_ref.range(63, 32));
        if (kappa > FP_CONST(1.5))  kappa = FP_CONST(1.5);
        if (kappa < FP_CONST(-1.5)) kappa = FP_CONST(-1.5);
        ref[k].kappa = kappa;
        
        ref[k].left_bound  = (fixed_point_t)(int)(data_ref.range(95, 64));
        ref[k].right_bound = (fixed_point_t)(int)(data_ref.range(127, 96));
    }

    /* ===== Run MPC solver ===== */
    fixed_point_t steer_out, accel_out;
    int status, iters;

    mpc_compute_hls(
        e_y, e_psi, vx, vy, omega,
        ref, &persist, &admm_state,
        &steer_out, &accel_out,
        &status, &iters);

    *out_steering_fp = (int)steer_out;
    *out_accel_fp    = (int)accel_out;
    *out_status      = status;
    *out_iterations  = iters;
}
