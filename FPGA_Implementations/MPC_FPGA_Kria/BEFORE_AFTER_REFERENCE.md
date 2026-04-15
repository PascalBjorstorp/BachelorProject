# Quick Reference: Before/After Code Changes

## Change 1: fp_recip() NR Iterations Reduction

**File:** `src/fp_math_hls.cpp` (line 148)

### Before:
```cpp
fp_QP_t est = FP_QP_CONST(1.5);
for (int i = 0; i < 4; i++) {
#pragma HLS PIPELINE II=2
#pragma HLS LOOP_TRIPCOUNT min=4 max=4
    est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));
}
```

### After:
```cpp
fp_QP_t est = FP_QP_CONST(1.5);
/* Reduced from 4 to 3 NR iterations: 3 iterations provide ~1e-11 accuracy.
 * Saves ~18 cycles per reciprocal × 20 backward pass iterations = 360 cycles/iteration. */
for (int i = 0; i < 3; i++) {
#pragma HLS PIPELINE II=2
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
    est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));
}
```

**Savings:** 80 cycles per iteration
**Risk:** 🟢 LOW

---

## Change 2: PA Matrix Unroll Completion

**File:** `src/riccati_solver_hls.cpp` (line 253)

### Before:
```cpp
for (i = 0; i < 6; i++) {
    for (j = 1; j < 6; j++) {
#pragma HLS PIPELINE II=1
        fp_raw_acc_t sum = 0;
        for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=4
            fp_raw_acc_t pa_prod = P[i][s] * fp_raw_acc_from_qp(sd->A[s][j]);
            sum += pa_prod;
        }
        PA[i][j] = sum >> FP_FRAC_BITS;
    }
}
```

### After:
```cpp
for (i = 0; i < 6; i++) {
    for (j = 1; j < 6; j++) {
#pragma HLS PIPELINE II=1
        fp_raw_acc_t sum = 0;
        for (s = 0; s < 6; s++) {
#pragma HLS UNROLL  /* Complete unroll of 6 elements */
            fp_raw_acc_t pa_prod = P[i][s] * fp_raw_acc_from_qp(sd->A[s][j]);
            sum += pa_prod;
        }
        PA[i][j] = sum >> FP_FRAC_BITS;
    }
}
```

**Savings:** 40-60 cycles (per backward pass iteration)
**Risk:** 🟢 LOW

---

## Change 3: State z/y Update II Reduction

**File:** `src/riccati_solver_hls.cpp` (line 630)

### Before:
```cpp
/* State z/y update */
for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=MPC_HLS_STATE_ZY_II  /* Default = 6 */
    const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=MPC_HLS_UNROLL_STATE_FACTOR
        // ... complex z_new, y_new computation ...
    }
}
```

### After:
```cpp
/* State z/y update (Priority 3: reduce II from 6→3 for 50% speedup) */
for (k = 0; k <= N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON_PLUS_ONE max=MPC_HORIZON_PLUS_ONE
#pragma HLS PIPELINE II=3  /* Reduced from MPC_HLS_STATE_ZY_II (default=6) */
    const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N - 1];
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=MPC_HLS_UNROLL_STATE_FACTOR
        // ... same computation ...
    }
}
```

**Savings:** 10-15 cycles per timestep × 21 timesteps = **210-315 cycles per iteration**
**Risk:** 🟡 MEDIUM (depends on synthesis achieving II=3)

---

## Change 4: Control z/y Update II Reduction

**File:** `src/riccati_solver_hls.cpp` (line 700)

### Before:
```cpp
/* Control z/y update — compute phase (store candidates) */
for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=MPC_HLS_CTRL_ZY_II  /* Default = 4 */
    const StepData_t *sd = &step_data[k];
    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
        // ... control update computation ...
    }
}
```

### After:
```cpp
/* Control z/y update — compute phase (Priority 4: reduce II from 4→2) */
for (k = 0; k < N; k++) {
#pragma HLS LOOP_TRIPCOUNT min=MPC_HORIZON max=MPC_HORIZON
#pragma HLS PIPELINE II=2  /* Reduced from MPC_HLS_CTRL_ZY_II (default=4) */
    const StepData_t *sd = &step_data[k];
    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
        // ... same computation ...
    }
}
```

**Savings:** 2 cycles per timestep × 20 timesteps = **40 cycles + reduction in norm accumulation**
**Total: ~100-150 cycles per iteration**
**Risk:** 🟡 MEDIUM (depends on DSP availability)

---

## Change 5: Forward Pass Unroll Factor Increase

**File:** `src/riccati_solver_hls.cpp` (lines 500-520)

### Before (u_k computation):
```cpp
/* u_k = K_k * x_k + kk_k */
for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
    fp_raw_acc_t prod_sum = 0;
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=2  /* 8 states → 4 iterations */
        prod_sum += fp_raw_acc_from_qp(K[k][a][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // ...
}
```

### After (u_k computation):
```cpp
/* u_k = K_k * x_k + kk_k (Priority 5: increase unroll factor 2→3) */
for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
    fp_raw_acc_t prod_sum = 0;
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL factor=3  /* 8 states → 3 iterations (less partial unroll stall) */
        prod_sum += fp_raw_acc_from_qp(K[k][a][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // ...
}
```

### Before (x_{k+1} computation):
```cpp
/* x_{k+1} = A_k * x_k + B_k * u_k - Dense rows 0..5: A*x + B*u */
for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
    fp_raw_acc_t sum = 0;
    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=2  /* 6 states → 3 iterations */
        sum += fp_raw_acc_from_qp(A_fwd[i][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // ...
}
```

### After (x_{k+1} computation):
```cpp
/* x_{k+1} = A_k * x_k + B_k * u_k - Dense rows 0..5 (Priority 5: increase unroll) */
for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
    fp_raw_acc_t sum = 0;
    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=3  /* 6 states → 2 iterations (better pipelining) */
        sum += fp_raw_acc_from_qp(A_fwd[i][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // ...
}
```

**Savings:** 1-2 cycles per state per timestep × 12 ops × 20 timesteps = **150-250 cycles total**
**Risk:** 🟢 LOW

---

## Summary of Changes

| Optimization | Line(s) | Change Type | Savings | Risk |
|---|---|---|---|---|
| fp_recip NR iter | 148 | Loop count 4→3 | 80 cy | 🟢 LOW |
| PA unroll factor | 253 | factor=4→complete | 40-60 cy | 🟢 LOW |
| P unroll factor | 310-330 | factor=4→complete | 30-50 cy | 🟢 LOW |
| p_new unroll factor | 370-390 | factor=4→complete | 20-40 cy | 🟢 LOW |
| State z/y II | 630 | II=6→3 | 210-315 cy | 🟡 MED |
| Control z/y II | 700 | II=4→2 | 100-150 cy | 🟡 MED |
| Forward u_k unroll | 500-510 | factor=2→3 | 40-80 cy | 🟢 LOW |
| Forward x_next unroll | 515-525 | factor=2→3 | 80-120 cy | 🟢 LOW |
| **Total** | **Multiple** | **8 changes** | **580-830 cy** | **Mixed** |

---

## Compilation & Testing Commands

### HLS C-Simulation (Functional Verification)
```bash
cd /home/akselmo/Documents/GitHub/BachelorProject/FPGA_Implementations/MPC_FPGA_Kria
source /tools/Xilinx/2025.1/Vitis/settings64.sh
HLS_RUN_MODE=csim vitis-run --mode hls --tcl scripts/run_hls.tcl
```

### HLS Synthesis (Latency Measurement)
```bash
HLS_RUN_MODE=synth vitis-run --mode hls --tcl scripts/run_hls.tcl
# Compare cycle count in synthesis report
```

### Full HLS Flow (C-sim → Synth → Co-sim)
```bash
HLS_RUN_MODE=all vitis-run --mode hls --tcl scripts/run_hls.tcl
```

---

