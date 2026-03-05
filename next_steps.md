# MPC Next Steps — Optimization, Cleanup & Implementation Tasks

## Context

This is a fixed-point (Q16.16) Model Predictive Controller for an F1/10th autonomous racing car. The core files are in `MPC/src/` and `MPC/include/`. The ROS2 node is `MPC/sim/mpc_ros2_node.c`. The target deployment is a Xilinx Ultra96 (Zynq UltraScale+) via Vitis HLS. All 445 unit/integration tests currently pass. The car reaches 11.80 m/s but the goal is 12 m/s. The MPC uses Projected Gradient Descent (PGD) to solve a QP at each step, with a dynamic bicycle model in Frenet coordinates (5 states: lateral error, heading error, vx, vy, yaw rate; 2 controls: steering, acceleration).

---

## 1. Remove Hacky Workarounds — Let the MPC Stand on Its Own

### 1.1 Remove the acceleration bias + clamp mechanism

**Location:** `MPC/src/mpc.c` lines ~1500–1560

The ε-bias mechanism shifts the QP equilibrium for acceleration from `a≈0` to `a≈a_target`, where `a_target = 2.0 × (v_ref - v_current)`. This was needed because `ε_accel=50` dominates the Hessian diagonal and drowns out the natural velocity tracking weight (`w_vel=2.0`). A clamp at `+1.1 m/s²` / `-7.0 m/s²` prevents the bias from overshooting. This clamp is what prevents the car from reaching 12 m/s.

**What to do:**
- Reduce `ε_accel` from 50 to a small value (try 1.0–5.0) — just enough for QP regularization, not enough to dominate the cost function.
- Increase `w_vel` if needed so velocity tracking drives acceleration decisions naturally.
- Remove the entire `a_target` bias computation and clamp (lines ~1500–1560 in mpc.c).
- Remove the corresponding steering bias if it becomes unnecessary after ε reduction.
- Let the MPC's own acceleration constraints (`max_accel=8.0`, `min_accel=-7.7` from vehicle model) bound acceleration naturally.
- The goal is: the QP naturally produces sensible acceleration from the velocity tracking cost, without artificial equilibrium shifting.
- **Test target:** car reaches 12 m/s on Spielberg without collisions, all 445 tests pass.

### 1.2 Remove or reduce the ε-softening if possible

**Location:** `MPC/src/mpc.c` lines ~1480–1500

`ε_steer=15*N/10` and `ε_accel=50*N/10` add large diagonal terms to the Hessian for regularization. With proper weight tuning, ε can likely be reduced significantly (ε=0.1–1.0 is typical in MPC literature for regularization). The current values (15 and 50) are 10–100× larger than the tracking weights and effectively override the cost function, requiring the bias mechanism to compensate.

**What to do:**
- Lower ε values iteratively while monitoring QP solver convergence (check for `status != 0` increase).
- If the QP becomes ill-conditioned without ε, find the minimum ε that maintains solver stability.
- Document the final ε values and why they're needed.

---

## 2. Optimization for Performance

### 2.1 Replace per-iteration int64 division with reciprocal multiply in PGD solver

**Location:** `MPC/src/qp_solver.c` ~lines 330–345

Every PGD iteration performs 20–24 int64 divisions for the gradient step: `step = gradient[i] * FP_ONE / step_denom[i]`. On FPGA, int64 division costs 30–80 cycles. The step denominators (`step_denom_64[]`) are computed once before the iteration loop and don't change.

**What to do:**
- Precompute `inv_step_denom[i] = (FP_ONE * FP_ONE) / step_denom_64[i]` once before the loop.
- Replace the division with multiplication: `step = (int64_t)gradient[i] * inv_step_denom[i] >> FP_FRAC_BITS`.
- Verify numerical equivalence with existing tests.
- **Estimated impact:** 40–60% solver speedup on FPGA.

### 2.2 Precompute vehicle/tire constants as compile-time defines for HLS

**Location:** `MPC/src/mpc.c` ~lines 310–350 (parameter computation block)

Every MPC call recomputes `C_f`, `C_r`, `F_zf`, `F_zr`, `CfCr_sum`, `lfCf_lrCr`, `lf2Cf_lr2Cr`, `inv_mass`, `inv_Iz`, `inv_L_wb` from the vehicle model. These never change at runtime.

**What to do:**
- Create `#define` or `static const` for all derived vehicle constants.
- Eliminate the `vehicle_model_get_parameters()` calls in the hot path.
- Gate behind `#ifdef MPC_HLS_TARGET` if runtime configurability is needed for testing.
- **Saves:** ~30 `fp_mul` + 3 `fp_recip` per MPC call.

### 2.3 LUT-based `fp_recip` for velocity range

**Location:** `MPC/src/fp_math.c` ~lines 55–75

`fp_recip()` uses 4 Newton-Raphson iterations (16 fixed-point operations). Called 10× per MPC step for `1/vx_k` during Phi propagation.

**What to do:**
- Create a 256-entry BRAM lookup table for the range `vx ∈ [2.0, 20.0]` in Q16.16.
- Use linear interpolation between entries + 1 Newton refinement for full Q16.16 precision.
- **Saves:** ~120 operations per MPC step (12 ops saved × 10 calls).

### 2.4 Fuse gradient step + projection per-variable in PGD

**Location:** `MPC/src/qp_solver.c` ~lines 305–380

Currently: compute full `H×u` vector → compute full gradient → compute full step → project. Three separate passes over 24 variables.

**What to do:**
- Pipeline per-variable: compute `Hu[i]` → `grad[i]` → `u_new[i] = u[i] - grad[i]*inv_step[i]` → clamp in one fused pass.
- Eliminates the `next_variables[]` buffer and saves ~10% iteration latency.

---

## 3. FPGA/HLS Porting — Required Changes

### 3.1 Shrink buffer sizes for fixed N=10

**Location:** `MPC/include/qp_solver.h` ~lines 35–47

Current `QP_MAXIMUM_VARIABLES=80`, `QP_MAXIMUM_CONSTRAINTS=200` allocate ~550 KB of static arrays. Ultra96 has 256 KB BRAM. At N=10, only ~27 KB is used.

**What to do:**
```c
#ifdef MPC_HLS_TARGET
  #define QP_MAXIMUM_VARIABLES    24   // 10*2 + 4 slacks
  #define QP_MAXIMUM_CONSTRAINTS  56   // 40 actuator + 4*2 wall + 4*2 slack_box
  #define MAXIMUM_HORIZON_STEPS   10
#endif
```
- This reduces memory from ~550 KB to ~27 KB, fitting entirely in on-chip BRAM.
- The `G_track`/`G_body` arrays alone drop from 320 KB to 6.4 KB.

### 3.2 Make horizon N and time step dt compile-time constants for HLS

**Location:** `MPC/include/mpc_types.h` ~lines 510–520

**What to do:**
- Define `MPC_HORIZON_STEPS 10` and `MPC_TIME_STEP 3277` (0.05s in Q16.16) as compile-time constants under `#ifdef MPC_HLS_TARGET`.
- Replace all `horizon_steps` variables with the constant in the HLS build.
- This enables HLS to fully unroll all horizon loops and perform constant propagation.

### 3.3 Replace unbounded `while` loops in `fp_normalize_angle`

**Location:** `MPC/src/fp_math.c` ~lines 37–41

```c
while (angle > FP_PI)  angle -= FP_TWO_PI;
while (angle < -FP_PI) angle += FP_TWO_PI;
```

HLS cannot synthesize unbounded loops.

**What to do:**
- Replace with bounded `for` loops (max 4 iterations covers any physically reasonable angle):
```c
for (int i = 0; i < 4 && angle > FP_PI; i++)
    angle -= FP_TWO_PI;
for (int i = 0; i < 4 && angle < -FP_PI; i++)
    angle += FP_TWO_PI;
```
- Or use single-cycle integer modular arithmetic.

### 3.4 Add HLS array partitioning pragmas

**Location:** Various arrays in `MPC/src/mpc.c`, `MPC/src/fp_math.c`, `MPC/src/qp_solver.c`

**What to do — add pragmas for these arrays:**
| Array | Partition | Rationale |
|-------|-----------|-----------|
| `Phi[10][5][2]` | `complete dim=3` | Both control columns in parallel |
| `PhiQ[10][2][5]` | `complete dim=2` | Both actuator entries per state |
| `Q[5]` | `complete` | Only 5 elements |
| `d[10][5]` | `complete dim=2` | All states per gradient term |
| `A[5][5]`, `B[5][2]` | `complete` | Small, fully accessed per step |
| `gradient[24]` | `complete` | Parallel read/write in PGD |
| `accum[24]` | `complete` | Eliminates dependency in mat-vec |

- Estimated 3–8× speedup for QP construction on FPGA.

### 3.5 Stack arrays → static for HLS

**Location:** `MPC/src/qp_solver.c` ~lines 259–262

`qp_solver_solve()` allocates `gradient[80]`, `next_variables[80]`, `hessian_times_variables[80]`, `constraint_metadata[200]`, `step_denom_64[80]` on the stack (~5 KB total).

**What to do:**
- Convert to `static` arrays for HLS (FPGA doesn't have a stack in the traditional sense).
- The `ConstraintMetadata_t` struct contains `nonzero_cols[QP_MAXIMUM_VARIABLES]` (80 bytes per constraint). For box constraints (1 nonzero each), only 1 byte is used. Consider specializing for HLS with `MAX_NONZEROS_PER_CONSTRAINT=4`.

---

## 4. Lower Priority Improvements

### 4.1 Restructure Hessian accumulation for fixed loop bounds

**Location:** `MPC/src/mpc.c` ~lines 780–810

The triple loop `ci × cj × k` has variable upper bound (`k = cj..N-1`), preventing HLS trip count analysis. Restructure as fixed-bound loop with masking or add `#pragma HLS LOOP_TRIPCOUNT min=1 max=10`.

### 4.2 Upper-triangular Hessian storage

**Location:** `MPC/include/qp_solver.h` ~line 88

Store only upper triangle of symmetric Hessian: 24×25/2 = 300 entries instead of 576. Saves 48% BRAM. Requires updating the symmetric mat-vec multiply to use packed indexing.

### 4.3 Multi-multiply primitives

**Location:** `MPC/include/fp_math.h`

Create `fp_mul3(a,b,c)` that keeps int64 intermediate to avoid redundant `>> FP_FRAC_BITS` shifts in chained multiplications. Saves ~1 cycle per eliminated shift across hundreds of calls in tire force computation (`MPC/src/vehicle_model.c` ~lines 470–540).

### 4.4 Explore Hessian symmetry in mat-vec multiply

The 2×2 block symmetric mat-vec in `fp_math.c` already exploits symmetry. For FPGA, the off-diagonal blocks with different `bj` indices write to disjoint accumulator pairs and are fully parallelizable. Mark for pipeline with II=1.

---

## 5. Testing Requirements

After any changes:
1. Build tests: `cd MPC/test && gcc -O2 -I../include -lm test_mpc_comprehensive.c ../src/*.c -o test && ./test`
2. Run all four test suites (445 tests total must pass):
   - `test_mpc_comprehensive` (307 tests)
   - `test_mpc_raceline` (85 tests)
   - `test_rk4_sim` (47 tests)
   - `test_sim_drive` (6 tests)
3. ROS2 simulation: `cd f1tenth_sim && ./run_logged_sim.sh 60` — check for 0 collisions, >99% solver success
4. Target: 12 m/s max speed, 0 collisions, all tests green

## 6. File Locations

| File | Path | Lines | Purpose |
|------|------|-------|---------|
| MPC core | `MPC/src/mpc.c` | 1896 | QP construction, warm-start, constraints, ε-bias |
| QP solver | `MPC/src/qp_solver.c` | ~500 | PGD solver with Gershgorin preconditioning |
| Fixed-point math | `MPC/src/fp_math.c` | ~450 | Matrix ops, normalize_angle, reciprocal |
| Vehicle model | `MPC/src/vehicle_model.c` | ~550 | Dynamic bicycle model, linearization |
| ROS2 node | `MPC/sim/mpc_ros2_node.c` | 1223 | Odometry, trajectory loading, drive publishing |
| MPC types | `MPC/include/mpc_types.h` | 527 | State/config/reference structs |
| QP types | `MPC/include/qp_solver.h` | ~120 | QP problem struct, buffer sizes |
| FP math header | `MPC/include/fp_math.h` | ~120 | Fixed-point macros, inline functions |
| Vehicle model header | `MPC/include/vehicle_model.h` | 213 | Vehicle params, model interface |
| MPC header | `MPC/include/mpc.h` | 99 | MPC public API |

## Priority Order

1. **Section 1** (remove hacks) — most impactful for correctness and reaching 12 m/s
2. **Section 2** (performance optimization) — needed for real-time FPGA execution
3. **Section 3** (HLS porting) — required for Ultra96 deployment
4. **Section 4** (nice-to-have) — further optimization after core work is done
