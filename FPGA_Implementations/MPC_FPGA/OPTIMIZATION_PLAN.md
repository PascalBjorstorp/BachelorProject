# FPGA MPC Optimization Plan — COMPLETED

## Target: Ultra96-V2 (xczu3eg-sbva484-1-e)
- **Before optimization**: DSP 561/360 (155%), LUT 66.9K/70.5K (94%) — DID NOT FIT
- **After optimization**: DSP 343/360 (95%), LUT 65.1K/70.5K (92%) — FITS ✓
- **Timing**: 8.671ns estimated (target 10ns), Fmax ~115 MHz — Meets ✓

## Optimizations Applied

### 1. Removed terminal cost UNROLL
**File**: `src/riccati_solver_hls.c`
- Removed `#pragma HLS UNROLL` from terminal cost init loop
- Negligible latency impact (~8 extra cycles per pass)

### 2. Precomputed dt × INV_MASS/INV_IZ
**File**: `include/mpc_fpga_types.h`, `src/vehicle_model_hls.c`
- Added `VP_DT_INV_MASS` and `VP_DT_INV_IZ` defines (placed after MPC_DT)
- Replaced 10 `fp_mul(dt, fp_mul(expr, VP_INV_MASS/IZ))` patterns with `fp_mul(expr, VP_DT_INV_MASS/IZ)`
- Saves 10 fp_mul calls per linearization

### 3. Trimmed fp_atan_small Taylor series
**File**: `src/fp_math_hls.c`
- Removed x^11/11 and x^13/13 terms (ATAN_COEF_11, ATAN_COEF_13)
- Max error <3 LSB at |x|=0.5 — negligible for slip angle computation
- Saves 4 fp_mul calls per fp_atan_small invocation

### 4. Un-inlined vehicle model (CRITICAL for DSP)
**File**: `src/vehicle_model_hls.c`
- Changed `#pragma HLS INLINE` to `#pragma HLS INLINE off`
- Added `#pragma HLS ALLOCATION operation instances=mul limit=6`
- Separates vehicle model's ~60 fp_mul from Riccati pass, preventing DSP sharing explosion

### 5. Removed PIPELINE II=1 from Riccati matrix loops (CRITICAL for DSP)
**File**: `src/riccati_solver_hls.c`
- Removed 10 `#pragma HLS PIPELINE II=1` pragmas from matrix-multiply inner loops
- These forced sum-of-products to execute in single cycle (6-8 parallel multiplies each)
- Without pipeline: sums execute sequentially, massively reducing DSP demand
- **This was the single most impactful optimization: 427→180 DSP in riccati_pass**

### 6. Selective PIPELINE II=2 on P*A matmul (latency optimization)
**File**: `src/riccati_solver_hls.c`
- Re-added `#pragma HLS PIPELINE II=2` to the P*A matrix multiply only
- Balances DSP (329→343, +14 DSP) vs latency (0.443→0.352 ms/pass, -21%)
- Total compute: 4.030→3.204 ms worst case (-20%)

### 7. Reduced MAX_ADMM_ITER from 20 to 8
**File**: `include/mpc_fpga_types.h`
- Simulation average: 1.1 iterations, max observed: 11 (now capped at 8)
- No quality degradation: 6/6 PASS, identical metrics
- Caps worst-case FPGA compute at ~3.2 ms

### 8. Added MUL_LIMIT pragma to Riccati pass
**File**: `src/riccati_solver_hls.c`
- Added `#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_MUL_LIMIT`
- Constrains maximum concurrent multiplier instances

## Per-function DSP breakdown (final)
| Function | DSP | Notes |
|---|---|---|
| riccati_pass_hls2 | 194 | PIPELINE II=2 on P*A only |
| compute_frenet_AB_hls | 68 | Vehicle model (un-inlined) |
| riccati_admm_solve_hls | 208 | riccati_pass + ADMM overhead |
| mpc_compute_hls | 308 | riccati_admm + vehicle + setup |
| **mpc_fpga_top** | **343** | **Total design** |

## Final Resource Utilization (N=19, cl050 optimal params)
| Resource | Used | Available | Utilization |
|---|---|---|---|
| DSP | 343 | 360 | 95% |
| LUT | 65,072 | 70,560 | 92% |
| BRAM_18K | 92 | 432 | 21% |
| FF | 37,630 | 141,120 | 26% |

## Timing Budget (worst case)
| Component | Time |
|---|---|
| Jetson→FPGA (ROS2/Ethernet) | ~2.0 ms |
| FPGA compute (8 ADMM iters) | ~3.2 ms |
| **Total worst case** | **~5.2 ms** |
| Average case (1.1 iters) | ~2.4 ms |

## Synthesis Comparison
| Config | DSP | LUT | riccati_pass | Max compute |
|---|---|---|---|---|
| No PIPELINE | 329 (91%) | 63,685 (90%) | 0.443ms | 4.030ms |
| **PIPELINE II=2 (final)** | **343 (95%)** | **65,072 (92%)** | **0.352ms** | **3.204ms** |
| PIPELINE II=1 | 361 (100%) | 64,608 (91%) | 0.265ms | ~2.65ms |

## Cl050 optimal parameters (from full sweep)
```
MPC_HORIZON=19, MPC_DT=0.04s, MPC_MAX_ADMM_ITER=8
Q_LAT=500, Q_HDG=2000, Q_VEL=30, Q_LAT_VEL=100
Q_YAW=22, W_DELTA_ACT=0.795, W_ACCEL_RATE=0.1
WALL_MARGIN=0.4, WALL_END=10, ADMM_ALPHA=1.5
```
Test result: 6/6 PASS, 0 wall collisions, 69% time above 5 m/s, max_vel=11.99 m/s
