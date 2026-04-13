#ifndef MPC_INTERNAL_DIAG_H
#define MPC_INTERNAL_DIAG_H

#include <stdint.h>

#ifndef MPC_VAR_DIAG_ENABLE
#define MPC_VAR_DIAG_ENABLE 0
#endif

typedef enum {
    MPC_DIAG_RECIP_ABS_X = 0,
    MPC_DIAG_RECIP_EST,
    MPC_DIAG_RECIP_PROD,
    MPC_DIAG_VM_VX_SAFE,
    MPC_DIAG_VM_D_F,
    MPC_DIAG_VM_D_R,
    MPC_DIAG_VM_C_EFF_F,
    MPC_DIAG_VM_C_EFF_R,
    MPC_DIAG_ASSEMBLY_Q_VX,
    MPC_DIAG_ASSEMBLY_Q_DELTA,
    MPC_DIAG_ASSEMBLY_DFF,
    MPC_DIAG_RICCATI_P,
    MPC_DIAG_RICCATI_M,
    MPC_DIAG_RICCATI_S,
    MPC_DIAG_RICCATI_G,
    MPC_DIAG_RICCATI_K,
    MPC_DIAG_ADMM_PRIMAL,
    MPC_DIAG_ADMM_DUAL,
    MPC_DIAG_ADMM_RHO,
    MPC_DIAG_ADMM_RHO_U,
    MPC_DIAG_VAR_COUNT
} MpcInternalDiagVarId;

typedef struct {
    double min_val;
    double max_val;
    double abs_max_val;
    double min_nonzero_delta;
    double last_val;
    int has_last;
    uint64_t samples;
} MpcInternalVarStat;

#ifdef __cplusplus
extern "C" {
#endif

void mpc_internal_diag_reset(void);
void mpc_internal_diag_observe(int var_id, double value);
void mpc_internal_diag_dump_csv(const char *path);

#ifdef __cplusplus
}
#endif

#if MPC_VAR_DIAG_ENABLE
#define MPC_DIAG_OBS_RAW(ID, VALUE) mpc_internal_diag_observe((int)(ID), (double)(VALUE))
#else
#define MPC_DIAG_OBS_RAW(ID, VALUE) do { (void)(ID); (void)(VALUE); } while (0)
#endif

#endif
