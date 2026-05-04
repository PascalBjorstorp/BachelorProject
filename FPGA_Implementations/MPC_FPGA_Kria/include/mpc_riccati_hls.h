#ifndef MPC_RICCATI_HLS_H
#define MPC_RICCATI_HLS_H

#include "fp_math_hls.h"
#include "mpc_fpga_types.h"


void mpc_compute_hls(
    fp_QP_t state_ey,
    fp_QP_t state_epsi,
    fp_QP_t state_vx,
    fp_QP_t state_vy,
    fp_QP_t state_omega,
    const MpcRefPoint_t ref[MPC_HORIZON],
    MpcPersistState_t *persist,
    AdmmState_t *admm_state,
    fp_QP_t *out_steering,
    fp_QP_t *out_accel,
    int *out_status,
    int *out_iters);



#endif