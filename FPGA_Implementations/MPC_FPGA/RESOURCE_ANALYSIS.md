# FPGA Resource Usage Analysis — MPC Riccati-ADMM on ZU3EG (Ultra96-V2)

**Date:** 2026-03-07  
**Target:** Xilinx Zynq UltraScale+ ZU3EG  
**Tool assumed:** Vitis HLS 2023.x+  
**Configuration:** Q16.16 fixed-point, N=20, nx=8 (augmented), nu=2, max 8 ADMM iterations

---

## 1. HLS Pragma Catalog

### 1.1 INTERFACE (24 pragmas) — `mpc_fpga_top.c`
All ports use AXI-Lite on the `ctrl` bundle:
```
#pragma HLS INTERFACE s_axilite port=return          bundle=ctrl
#pragma HLS INTERFACE s_axilite port=mode            bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_index        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_x_fp         bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_y_fp         bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_psi_fp       bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_vx_fp        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_kappa_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_ax_fp        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_left_bound_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_right_bound_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=wp_total        bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_x_fp      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_y_fp      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_theta_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_vx_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_vy_fp     bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_omega_fp  bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=state_wp_idx    bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_steering_fp bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_accel_fp    bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_status      bundle=ctrl
#pragma HLS INTERFACE s_axilite port=out_iterations  bundle=ctrl
```
**Impact:** Single AXI-Lite slave with ~24 registers. ~2,000–3,000 LUTs for the adapter.

### 1.2 BIND_STORAGE (12 pragmas)

| Variable | Type | Impl | File |
|----------|------|------|------|
| `trajectory[1024]` | `ram_2p` | `bram` | mpc_fpga_top.c |
| `admm_state` (persistent) | `ram_2p` | `bram` | mpc_fpga_top.c |
| `persist` | — | `register` | mpc_fpga_top.c |
| `K[20][2][8]` | `ram_2p` | `bram` | riccati_solver_hls.c |
| `step_data[20]` | `ram_2p` | `bram` | mpc_riccati_hls.c |
| `z_x[21][8]` | `ram_2p` | `bram` | riccati_solver_hls.c |
| `z_u[20][2]` | `ram_2p` | `bram` | riccati_solver_hls.c |
| `y_x[21][8]` | `ram_2p` | `bram` | riccati_solver_hls.c |
| `y_u[20][2]` | `ram_2p` | `bram` | riccati_solver_hls.c |

Plus 1 `BIND_OP`:
| Variable | Op | Impl | File |
|----------|------|------|------|
| `product` (in `fp_mul`) | `mul` | `dsp` | fp_math_hls.h |

### 1.3 ARRAY_PARTITION (16 pragmas)

| Variable | Type | Factor/Dim | Elements | Effect | File |
|----------|------|------------|----------|--------|------|
| `ref[20]` (MpcRefPoint_t) | `complete` | dim=0 | 80 × 32b | **All registers** | mpc_fpga_top.c |
| `K[20][2][8]` | `complete` | dim=3 | 8 sub-arrays [20][2] | 8 BRAM banks | riccati_solver_hls.c |
| `kk[20][2]` | `complete` | dim=2 | 2 sub-arrays [20] | 2 BRAM/LUTRAM banks | riccati_solver_hls.c |
| `P[8][8]` (int64_t) | `cyclic` | factor=8, dim=1 | 8 row banks | Distributed | riccati_solver_hls.c |
| `P[8][8]` (int64_t) | `cyclic` | factor=2, dim=2 | ×2 col banks (16 total) | Distributed | riccati_solver_hls.c |
| `M[2][8]` (int64_t) | `complete` | dim=1 | 2 row banks | Registers | riccati_solver_hls.c |
| `M[2][8]` (int64_t) | `cyclic` | factor=4, dim=2 | ×4 col banks (8 total) | Distributed | riccati_solver_hls.c |
| `G[2][8]` (int64_t) | `complete` | dim=1 | 2 row banks | Registers | riccati_solver_hls.c |
| `G[2][8]` (int64_t) | `cyclic` | factor=4, dim=2 | ×4 col banks (8 total) | Distributed | riccati_solver_hls.c |
| `PA[6][6]` (int64_t) | `complete` | dim=1 | 6 row banks | **All registers** | riccati_solver_hls.c |
| `PA[6][6]` (int64_t) | `complete` | dim=2 | ×6 col banks (36 total) | **All registers** | riccati_solver_hls.c |
| `z_x[21][8]` | `cyclic` | factor=4, dim=2 | 4 col banks | BRAM (4 ports) | riccati_solver_hls.c |
| `z_u[20][2]` | `complete` | dim=2 | 2 sub-arrays | BRAM (2 ports) | riccati_solver_hls.c |
| `y_x[21][8]` | `cyclic` | factor=4, dim=2 | 4 col banks | BRAM (4 ports) | riccati_solver_hls.c |
| `y_u[20][2]` | `complete` | dim=2 | 2 sub-arrays | BRAM (2 ports) | riccati_solver_hls.c |
| `x_is_con[21][8]` | `complete` | dim=2 | 8 sub-arrays | Registers/LUTRAM | riccati_solver_hls.c |

### 1.4 PIPELINE (19 pragmas)

| Location | II | Context |
|----------|----|---------|
| `reciprocal_64` Newton loop | 4 | 4 iterations, 3 muls each |
| `fp_recip` Newton loop | 4 | 4 iterations, 3 muls each |
| Backward M computation (j loop) | 1 | 8 iters: B^T × P column |
| Backward G computation (j=0..5) | 1 | 12 iters: M×A + N^T |
| Backward K computation (j loop) | 1 | 16 iters: S^{-1} × G |
| Backward PA computation (i×j) | 1 | 36 iters: P × A dense block |
| Backward P update (i×j, 3 loops) | 1 | 36+12+16 iters |
| Backward p update (i=0..5) | 1 | 6 iters: A^T × p + G^T × kk |
| Forward u computation (s loop) | 1 | 16 iters: K × x |
| Forward x computation (dense rows) | 1 | 6 iters: A×x + B×u |
| Ref building loop (k=0..19) | 1 | Top-level ref trajectory |
| ADMM z/y state update | 4 | 21×8 iters (unroll factor=4) |
| ADMM z/y control update | 1 | 20×2 iters |
| ADMM warm-start copy (×4 loops) | 1 | Initialization |
| ADMM save state (×2 loops) | 1 | Post-solve |
| Solution init (×2 loops) | 1 | Pre-solve zeroing |
| Adaptive rho y-scaling (×4 loops) | 1 | Every 2nd ADMM iteration |

### 1.5 UNROLL (24 pragmas)

| Location | Factor | Iterations |
|----------|--------|------------|
| Terminal P,p init (3 loops) | full | 8, 8, 8 |
| Terminal P diagonal setup | full | 8 |
| Backward Q_aug/q_aug setup | full | 8 |
| Backward R_aug/r_aug setup | full | 2 |
| Backward G unroll over a | full | 2 |
| Backward K unroll over a | full | 2 |
| Backward kk unroll over a | full | 2 |
| Backward p_new rows 6,7 | full | 2 |
| Forward x0 copy | full | 8 |
| Forward u unroll over a | full | 2 |
| step_data A zeroing (3×) | full | 8 each |
| step_data B, N_cross zeroing | full | 5, 8 |
| step_data A_base copy | full | 5×5 |
| step_data A col 5 fill | full | 5 |
| step_data B col 1 fill | full | 5 |
| Constrained flags (x_is_con) | full | 8 |
| ADMM z/y state (inner) | factor=4 | 8-wide |
| ADMM z/y control (inner) | full | 2 |
| ADMM cold-start z-init (inner) | full | 8 |
| ADMM y-init (inner) | full | 8 + 2 |
| Adaptive rho y-scale (inner) | full | 8 + 2 |
| vehicle_model A/B init | full | 5×5 + 5×2 |
| terminal_Q/q init | full | 8 |

### 1.6 LOOP_TRIPCOUNT (20 pragmas)

| Loop | min | max | avg | Count |
|------|-----|-----|-----|-------|
| Backward pass (k=N-1..0) | 20 | 20 | — | 20 |
| Forward pass (k=0..19) | 20 | 20 | — | 20 |
| ADMM main loop | 1 | 8 | 2 | ≤8 |
| Constrained flags (k=0..20) | 21 | 21 | — | 21 |
| Cold-start z-init (k=0..20) | 21 | 21 | — | 21 |
| Cold-start z-project (k=0..20) | 21 | 21 | — | 21 |
| Cold-start u-project (k=0..19) | 20 | 20 | — | 20 |
| Cold-start y-init (×2) | 21+20 | 21+20 | — | 41 |
| ADMM z/y state update (k=0..20) | 21 | 21 | — | 21 |
| ADMM z/y control update (k=0..19) | 20 | 20 | — | 20 |
| Adaptive rho scaling (×4) | 21+20 | 21+20 | — | 41 |
| reciprocal_64 CLZ | 1 | 31 | — | ≤31 |
| reciprocal_64 Newton | 4 | 4 | — | 4 |
| fp_recip CLZ | 1 | 31 | — | ≤31 |
| fp_recip Newton | 6 | 6 | — | 6* |
| fp_normalize_angle (×2) | 0 | 4 | — | ≤4 |

*Note: RECIP_ITERATIONS=4 but tripcount says 6 — minor mismatch.

### 1.7 Other Pragmas

| Pragma | Target | File |
|--------|--------|------|
| `INLINE` | `reciprocal_64` | riccati_solver_hls.c |
| `INLINE` | `invert_2x2_hls` | riccati_solver_hls.c |
| `INLINE` | `compute_frenet_AB_hls` | vehicle_model_hls.c |
| `INLINE` | `saturate_control_hls` | vehicle_model_hls.c |
| `INLINE` | `fp_normalize_angle` | fp_math_hls.c |
| `INLINE` | `fp_sin` | fp_math_hls.c |
| `INLINE` | `fp_cos` | fp_math_hls.c |
| `INLINE` | `fp_atan_small` | fp_math_hls.c |
| `INLINE` | `fp_atan` | fp_math_hls.c |
| `INLINE off` | `fp_recip` | fp_math_hls.c |
| `LOOP_FLATTEN off` | Backward pass | riccati_solver_hls.c |
| `LOOP_FLATTEN off` | Forward pass | riccati_solver_hls.c |
| `ALLOCATION instances=sdiv limit=1` | `invert_2x2_hls` | riccati_solver_hls.c |

**Total pragma count: ~95 pragmas across all files.**

---

## 2. Array Sizes and Storage Mapping

### 2.1 BRAM-Stored Arrays

| Array | Dimensions | Element | Total Bytes | Partition | BRAM18K Est. |
|-------|-----------|---------|-------------|-----------|-------------|
| `trajectory` | 1024 × 8 fields | int32_t | 32,768 | none (struct) | 16 |
| `admm_state` (persistent) | see substruct | int32_t | 1,676 | none | 4 |
| `step_data[20]` | 20 × 136 words | int32_t | 10,880 | none (struct) | 8 |
| `K[20][2][8]` | 320 total | int32_t | 1,280 | complete dim=3 → 8 banks | 8 |
| `z_x[21][8]` | 168 total | int32_t | 672 | cyclic×4 dim=2 → 4 banks | 4 |
| `z_u[20][2]` | 40 total | int32_t | 160 | complete dim=2 → 2 banks | 2 |
| `y_x[21][8]` | 168 total | int32_t | 672 | cyclic×4 dim=2 → 4 banks | 4 |
| `y_u[20][2]` | 40 total | int32_t | 160 | complete dim=2 → 2 banks | 2 |
| `solution.x[21][8]` | 168 total | int32_t | 672 | auto (likely BRAM) | 4 |
| `solution.u[20][2]` | 40 total | int32_t | 160 | auto (likely BRAM) | 2 |
| **Total** | | | **~49 KB** | | **~54 BRAM18K** |

54 BRAM18K = **27 BRAM36K equivalent.**

### 2.2 Register-Stored Arrays (Fully Partitioned → FFs)

| Array | Dimensions | Element | Total FFs |
|-------|-----------|---------|-----------|
| `ref[20]` (MpcRefPoint_t) | 80 words | int32_t | 2,560 |
| `PA[6][6]` | 36 entries | int64_t | 2,304 |
| `M[2][8]` | 16 entries | int64_t | 1,024 |
| `G[2][8]` | 16 entries | int64_t | 1,024 |
| `P[8][8]` (16 banks, ~4 entries each) | 64 entries | int64_t | ~4,096 |
| `x_is_con[21][8]` | 168 entries | uint8_t | ~168 |
| `persist` (MpcPersistState_t) | 6 words | int32_t | 192 |
| Loop-local `Q_aug`, `q_aug`, `R_aug`, `r_aug` | 8+8+2+2 | int64_t | 1,280 |
| **Subtotal partitioned arrays** | | | **~12,648 FFs** |

---

## 3. DSP48E2 Usage Estimate

### 3.1 Multiply Operation Classification

**`fp_mul` (32×32→64 with `BIND_OP dsp`):**  
On DSP48E2 (27×18 native), a 32×32 multiply requires **2–3 DSP48E2s** depending on HLS decomposition. Vivado typically uses **3 DSP48E2s** for full 32×32→64 accuracy.

**int64_t × int32_t (Riccati inner products):**  
e.g., `P[i][s] * (int64_t)sd->A[s][j]`. In practice, the int64_t value in Q16.16 MPC has ~48 useful bits. Vivado will map this to **2–4 DSP48E2s** per multiply.

**int64_t × int64_t (reciprocal_64):**  
3 multiplies per Newton iteration × 4 iterations = 12 multiplies, pipelined II=4. Needs **~4 DSP48E2s per multiply** → but pipelining allows reuse. Estimated: **6–12 DSP48E2s** for the reciprocal hardware.

### 3.2 Concurrent DSP Count per Pipeline Stage

The backward pass runs 20 sequential iterations (LOOP_FLATTEN off). Within each iteration, Vitis HLS can share DSPs across the 8 sequential steps. The *peak* DSP usage is determined by the widest pipeline stage:

| Pipeline Stage | Inner Muls | DSPs per Mul | Peak DSPs |
|---------------|------------|-------------|-----------|
| **PA = P×A** (6×6, II=1) | 6 (int64×int32) | 3 | **18** |
| **P update** (6×6, II=1) | 6+2 = 8 (mixed) | 3 | **24** |
| **M = B^T×P** (8 cols, II=1) | 8 (int64×int32) | 3 | **24** |
| **G = M×A+N** (2×6, II=1) | 6 (int64×int32) | 3 | **18** |
| **K = -S^{-1}×G** (2×8, II=1) | 2 (int64×int64) | 4 | **8** |
| **reciprocal_64** (II=4) | 3 (int64×int64) | 4 | **12** |

Since steps 1–8 are **sequential** within each backward iteration, HLS can time-multiplex DSP resources. The peak concurrent requirement is the **largest single pipeline stage ≈ 24 DSP48E2s**.

### 3.3 Additional DSP Consumers

| Consumer | Estimated DSPs | Notes |
|----------|---------------|-------|
| `fp_recip` (INLINE off → single instance) | 9–12 | 3 muls/iter × 3 DSPs, pipelined |
| Vehicle model (INLINE → merged) | 0 (shared) | Uses fp_mul but runs before ADMM loop |
| Trig functions (INLINE) | 0 (shared) | sin/cos/atan Taylor terms share DSP with above |
| ADMM z/y over-relaxation | 4–8 | `ADMM_OVER_RELAX_MINUS1 * diff` (int64×int32) |
| Forward pass (K×x, A×x+B×u) | 0 (shared) | Reuses backward pass DSPs |
| Adaptive rho (fp_mul) | 0 (shared) | Rare (every 2nd iteration) |

### 3.4 Total DSP Estimate

```
Peak concurrent DSPs (Riccati pipeline):  24
fp_recip hardware instance:              12
Over-relaxation (concurrent with z/y):    8
AXI-Lite and misc:                         2
                                         ----
Estimated minimum:                        46
```

With HLS overhead, register duplication, and imperfect sharing:

**Conservative estimate: 80–150 DSP48E2s**  
**Realistic estimate: 90–120 DSP48E2s**

---

## 4. BRAM Usage Summary

| Category | BRAM18K | BRAM36K Equiv. |
|----------|---------|----------------|
| Trajectory (1024 waypoints × 8 fields) | 16 | 8 |
| ADMM state (persistent warm-start) | 4 | 2 |
| StepData (20 × 136 words) | 8 | 4 |
| Riccati gains K[20][2] × 8 banks | 8 | 4 |
| ADMM local z_x, z_u, y_x, y_u | 12 | 6 |
| Solution arrays (x, u) | 6 | 3 |
| **Total** | **~54** | **~27** |

---

## 5. Overall Utilization Estimate — ZU3EG

| Resource | ZU3EG Available | Estimated Usage | Utilization |
|----------|----------------|-----------------|-------------|
| **LUT** | 70,560 | 25,000 – 40,000 | **35 – 57%** |
| **FF** | 141,120 | 25,000 – 35,000 | **18 – 25%** |
| **BRAM36K** | 216 | 27 – 35 | **12 – 16%** |
| **DSP48E2** | 360 | 80 – 150 | **22 – 42%** |
| **URAM** | 0 | 0 | **N/A** |

### Verdict: ✅ Fits comfortably on the ZU3EG

**Tightest resource: LUTs (35–57%)**, driven by:
- AXI-Lite interface with 24 register ports
- Inlined trig functions (sin, cos, atan Taylor series) duplicated across call sites
- Muxing for BRAM access through partitioned arrays
- Control logic for ADMM outer loop with adaptive rho

**Most comfortable resource: BRAM (12–16%)** — ample headroom for larger trajectories or additional features.

**DSP margin is healthy (22–42%)** — the design exploits temporal multiplexing well through sequential backward-pass steps.

---

## 6. Potential Resource Bottlenecks

### 6.1 LUT Pressure from Inlined Functions
- `fp_sin`, `fp_cos`, `fp_atan` are all `INLINE` and called from `compute_frenet_AB_hls` which is itself `INLINE`.
- Each atan call includes branching, 2 possible `fp_recip` calls, and a 7-term Taylor series.
- The vehicle model calls: 2× `fp_cos`, 2× `fp_sin`, 5× `fp_atan`, 5× `fp_recip`.
- All inlined → **duplicated hardware for each call site** → significant LUT bloat.

### 6.2 P Matrix Partitioning
- `P[8][8]` (int64_t) with cyclic factor=8 dim=1 × factor=2 dim=2 = **16 banks**.
- Each bank holds 4 × 64-bit entries = 256 bits — too small for BRAM, will go to LUTRAM or FFs.
- 64 × 64 bits = **4,096 FFs in registers** or equivalent LUTRAM.
- This is necessary for II=1 on the 6×6 matrix operations but is a significant FF consumer.

### 6.3 PA Fully Partitioned
- `PA[6][6]` (int64_t) with `complete` on both dims → **36 × 64 = 2,304 FFs**.
- Required for the fused P-update to read all PA entries simultaneously.
- If nx were larger, this would not scale (nx² × 64 growth).

### 6.4 `reciprocal_64` Inlined into Backward Pass
- `reciprocal_64` is `INLINE`, so it's duplicated inside the backward pass loop body.
- The CLZ loop (up to 31 iterations) + 4 Newton iterations with `PIPELINE II=4` creates a latency bubble of **~50 cycles per step** in the backward pass.
- This is the **longest single operation** in each backward step.

### 6.5 ADMM Z/Y Update Pipeline
- `PIPELINE II=4` with `UNROLL factor=4` on the inner 8-element state vector.
- The over-relaxation multiply (`ADMM_OVER_RELAX_MINUS1 * diff`) is int64_t arithmetic.
- 21 × 2 = 42 pipeline stages (plus startup) → ~170 cycles for state z/y update.

---

## 7. Optimization Suggestions

### 7.1 Reduce LUT usage (if over 55%)
- **Un-inline `fp_atan`**: Change to `#pragma HLS INLINE off`. The vehicle model calls it 5 times, so a single shared instance would save ~5,000–8,000 LUTs at the cost of ~200 cycles latency (called sequentially).
- **Un-inline `fp_sin`/`fp_cos`**: Similar argument. Each has ~7 fp_mul operations. Shared instances save ~2,000–4,000 LUTs.
- Trade-off: the vehicle model is called **once** before the ADMM loop, so the latency penalty is negligible.

### 7.2 Reduce BRAM usage (if needed for other IPs)
- **Reduce `MAX_TRAJECTORY_SIZE`**: Currently 1024, consuming 16 BRAM18K. If your track has ≤512 waypoints, halving this saves 8 BRAM18K. If ≤256, saves 12.
- **Pack StepData fields**: The struct has many small arrays that HLS decomposes. Consider packing A and B into a single flat array with manual indexing for tighter BRAM packing.

### 7.3 Reduce DSP usage (if over 40%)
- **Use `impl=fabric`** for low-criticality multiplies (e.g., adaptive rho scaling, warm-start copies) instead of DSP. This pushes multiplies to LUT-based logic.
- **Share fp_recip**: Already `INLINE off` — good. Verify Vitis creates only one hardware instance.

### 7.4 Reduce latency (if solver is too slow)
- **Partially unroll the backward pass**: `#pragma HLS UNROLL factor=2` on the k-loop would double DSP usage (~200–240 DSPs) but halve backward pass latency.
- **Pre-compute S^{-1}**: The `reciprocal_64` Newton iteration is the bottleneck (~50 cycles). A lookup table for initial estimates could reduce Newton iterations from 4 to 2.

### 7.5 Achievable Clock Frequency Estimate
Typical Vitis HLS targeting ZYNQ UltraScale+ achieves **200–300 MHz** for Q16.16 fixed-point designs. At 300 MHz:

| Phase | Cycles | Time |
|-------|--------|------|
| Vehicle linearization | ~200 | 0.7 µs |
| StepData construction (20 steps) | ~1,000 | 3.3 µs |
| Cold-start Riccati pass | ~5,000 | 16.7 µs |
| Per ADMM iteration (Riccati + z/y) | ~5,500 | 18.3 µs |
| 8 ADMM iterations (worst case) | ~44,000 | 147 µs |
| **Total worst-case (cold start)** | **~50,000** | **~167 µs** |
| **Typical (warm, 2 iterations)** | **~12,000** | **~40 µs** |

At 400 Hz MPC rate (2.5 ms budget): **uses 1.6–6.7% of the time budget** — extremely fast.

---

## 8. Risk Assessment

| Risk | Severity | Likelihood | Mitigation |
|------|----------|------------|------------|
| LUT overflow (>100%) | High | Low (est. 35–57%) | Un-inline trig functions |
| DSP overflow (>100%) | High | Very Low (est. 22–42%) | Use fabric for non-critical muls |
| Timing closure at 300 MHz | Medium | Medium | Target 250 MHz initially |
| BRAM port conflicts | Medium | Low | ram_2p + partitioning handles it |
| Cold-start latency spike | Low | Medium | Pre-seed ADMM state from CPU |

**Bottom line:** This design should synthesize and meet timing on the ZU3EG with comfortable margins on all resource types. The primary concern is LUT utilization from deeply inlined trig functions, which can be addressed by selectively removing INLINE pragmas.
