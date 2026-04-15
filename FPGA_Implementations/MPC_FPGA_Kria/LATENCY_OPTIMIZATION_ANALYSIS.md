# MPC FPGA HLS Latency Optimization Analysis

**Current State:**
- Minimum latency: **3,721 cycles** (~14.9 µs at 4ns period, 250 MHz nominal)
- Current WNS: **-0.948 ns** (timing still needs optimization)
- Constraint: **5ns period** (200 MHz actual after margin)
- Target: **2,000 cycles** (~8 µs) = **46% cycle reduction**

## 1. Code Path Analysis

### Riccati Solver Iteration Breakdown

```
Per-iteration cycle distribution (estimated):
├── Backward pass (riccati_pass_hls):        ~2,800 cycles (75%)
│   ├── Terminal cost init:                    ~50 cycles
│   ├── Loop k=N-1..0 (20 outer loop iterations):
│   │   ├── M = B^T*P compute:                ~40 cycles
│   │   ├── S = r_aug + M*B compute:          ~30 cycles
│   │   ├── 2x2 matrix inversion (S^{-1}):   ~100 cycles (RECIPROCAL BOTTLENECK)
│   │   ├── G = M*A + N^T compute:           ~150 cycles
│   │   ├── K = -S^{-1}*G compute:           ~60 cycles
│   │   ├── kk = -S^{-1}*(r_aug_lin + B^T*p) compute: ~40 cycles
│   │   ├── P = Q_diag + A^T*PA + G^T*K:     ~200 cycles (DENSE 6x6 ACCUMULATION)
│   │   └── p_new = q_aug_linear + A^T*p + G^T*kk: ~70 cycles
│   │
│   └── Forward pass (N=20 timesteps):
│       ├── u_k = K_k*x_k + kk_k:             ~30 cycles/step
│       ├── x_{k+1} = A_k*x_k + B_k*u_k:    ~40 cycles/step
│       └── Total forward:                   ~1,400 cycles (blocked by backward)
│
├── ADMM Dual Updates (per iteration):      ~800 cycles (21%)
│   ├── State z/y update (II=6 bottleneck):  ~500 cycles
│   ├── Control z/y update (II=4):           ~150 cycles
│   ├── Residual norm computation:           ~100 cycles
│   └── Convergence check & rho adapt:       ~50 cycles
│
└── Overhead (loop control, conversions):     ~120 cycles (3%)
```

## 2. Identified Bottlenecks (Ranked by Impact)

### **🔴 CRITICAL: Reciprocal Function Latency (100-120 cycles total per iteration)**

**Location:** `fp_math_hls.cpp::fp_recip()` called from `invert_2x2_hls()`

**Current Implementation:**
```cpp
for (int i = 0; i < 4; i++) {
#pragma HLS PIPELINE II=2              // ← II=2 requires 8 cycles minimum
    est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));
}
// Plus: 10 cycles shift/normalization overhead
// Total: ~18 cycles per reciprocal, called once per backward-loop iteration
// → 20 backward iterations × 18 = 360 cycles just for reciprocals
```

**Why it matters:**
- Called **20 times per backward pass** (once per k timestep for 2×2 inversion)
- Each call takes **~18 cycles** (4 NR iterations at II=2)
- Total: **360 cycles/iteration** = **9.7% of total latency**

**Estimated Savings: 100-150 cycles (2.7-4% of total)**

---

### **🔴 CRITICAL: State z/y Update Loop (II=6, should be II=2-3)**

**Location:** `riccati_solver_hls.cpp` line ~625-680

**Current Pipeline:**
```cpp
for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II=MPC_HLS_STATE_ZY_II  // ← Default II=6 (UNDERPIPELINED!)
    // Complex conditionals and multiple fp_mul() calls per iteration
    // Compute: z_new, y_new, primal/dual residuals
}
```

**Problem:**
- 11 states × 21 timesteps = 231 total elements to process
- At II=6: **1,386 cycles** just for this one loop
- Could achieve II=2-3 by:
  - Removing data dependencies between state indices
  - Unrolling the inner `s` loop completely (8 elements)
  - Moving conditional logic outside the pipeline

**Estimated Savings: 500-700 cycles (13.4-18.8% of total)**

---

### **🔴 CRITICAL: Dense 6×6 Matrix Operations (P update)**

**Location:** `riccati_solver_hls.cpp` line ~310-360 (P matrix update)

**Current Code:**
```cpp
for (int idx = 0; idx < 21; idx++) {
#pragma HLS PIPELINE II=1
    // A^T * PA (6×6 multiply)
    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=4  // ← Partial unroll, leaves 2-3 cycles per iteration
        // Accumulate products
    }
    // G^T * K multiply (2×6 × 6×N multiply, partial)
    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL
        // 2 elements unrolled, brief
    }
}
```

**Issues:**
1. Inner loop unroll factor=4 on 6 elements → partial unroll stalls
2. II=1 claimed but loop carries prevent true II=1
3. 6×6 dense block processes 21 upper-triangle elements sequentially

**Optimization Path:**
- Increase unroll factor to 6 (complete inner loop)
- Use array partitioning to enable true II=1
- Fuse A^T*PA + G^T*K into single pass

**Estimated Savings: 200-300 cycles (5.4-8.1% of total)**

---

### **🟠 HIGH: Control z/y Update Loop (II=4)**

**Location:** `riccati_solver_hls.cpp` line ~695-730

**Current:**
```cpp
for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=MPC_HLS_CTRL_ZY_II  // ← II=4, could be II=2
    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL  // ← 2 controls, fully unrolled
        // Complex z_new, y_new, residual computation
    }
}
```

**Issues:**
- 20 timesteps × 4 cycles/II = 80 cycles just for loop overhead
- Multiple `fp_mul()` calls (~12 DSPs) create dependencies
- II=4 is conservative given only 2 control inputs

**Estimated Savings: 150-200 cycles (4.0-5.4% of total)**

---

### **🟠 HIGH: Forward Pass Loop Carries**

**Location:** `riccati_solver_hls.cpp` line ~468-520

**Current:**
```cpp
for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
    fp_raw_acc_t sum = 0;
    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=2  // ← Partial: 6 elements → 3 iterations
        sum += fp_raw_acc_from_qp(A_fwd[i][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // ...more accumulations
}
```

**Issues:**
- Inner loop partial unroll (factor=2 on 6 elements) leaves stalls
- Accumulation prevents true pipeline, forces II=2-3 effective
- Repeated 20 times in forward pass

**Estimated Savings: 150-250 cycles (4.0-6.7% of total)**

---

### **🟡 MEDIUM: G Matrix Computation (M*A Inner Loop)**

**Location:** `riccati_solver_hls.cpp` line ~228-250

**Current:**
```cpp
for (a = 0; a < nu; a++) {
    for (j = 0; j < 6; j++) {
#pragma HLS PIPELINE II=1
        fp_raw_acc_t sum = 0;
        for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=3  // ← Partial unroll (6 → 2 iterations)
            // Multiply-accumulate
        }
    }
}
```

**Issues:**
- Unroll factor=3 on 6 elements leaves 2-cycle stall per iteration
- II=1 claimed but accumulator creates true II=2
- 2×6 = 12 matrix multiply operations

**Estimated Savings: 80-150 cycles (2.1-4.0% of total)**

---

### **🟡 MEDIUM: Bit-Width Precision Excess**

**Current Fixed-Point Types:**
```cpp
#define MPC_HLS_RICCATI_WIDTH      32    // Q16.16
#define MPC_HLS_ACCUM_WIDTH        24    // Q12.12
#define MPC_HLS_RAW_ACC_GUARD_BITS 20    // 32+20=52-bit accumulator!
```

**Issues:**
- 52-bit intermediate accumulators may be overkill
- Multiply results (32×32→64-bit) are immediately right-shifted by 16
- 20 guard bits provide 1M:1 headroom (excessive)
- Larger bit-widths increase DSP usage and routing congestion

**Analysis:**
- MPC horizon = 10 steps, N terms ≤ 20 in worst sums
- log₂(20) = 4.3 bits needed
- Current 20 bits ≈ 4.6× overkill

**Estimated Savings: 100-200 cycles (via reduced routing delays and better packing)**

---

### **🟡 MEDIUM: Conditional Branching in Hot Paths**

**Location:** `riccati_solver_hls.cpp` line ~635-650 (z/y update)

**Current:**
```cpp
if (x_is_con[k][s]) {
    // Complex computation path: ~30 operations
    z_new = ...;
    y_new = ...;
    state_dual_delta_cand[k][s] = ...;
} else {
    // Simple passthrough: ~5 operations
    z_x[k][s] = x_passthrough;
}
```

**Issues:**
- Conditional blocks inside II=6 pipeline force synthesis to pre-compute both paths
- Data dependency on `x_is_con` (precomputed flags) prevents speculative execution
- ~60% of states are typically unconstrained in practice
- Branch penalty: +2-3 cycles per iteration in worst case

**Optimization:** Split constrained/unconstrained processing into separate pipelined paths (requires precomputation of state/control sets)

**Estimated Savings: 50-100 cycles (1.3-2.7% of total)**

---

## 3. Optimization Opportunities (Prioritized)

### **Priority 1: Reciprocal & 2×2 Inversion Acceleration** 
**Target Savings: 100-150 cycles (2.7-4.0%)**

#### 1a. Reduce fp_recip NR Iterations from 4 → 3
- **Rationale:** Only 1e-5 accuracy needed (Q16.16 format); 3 iterations → 1e-11
- **Change:**
  ```cpp
  for (int i = 0; i < 3; i++) {  // ← Was 4
  #pragma HLS PIPELINE II=2
      est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));
  }
  ```
- **Latency:** 18 → 14 cycles per reciprocal
- **Impact:** 20 iterations × 4 cycles saved = **80 cycles**
- **Risk:** Low (3 iterations still exceeds convergence requirement)

#### 1b. Reduce fp_recip Pipeline II from 2 → 1
- **Prerequisite:** Remove one `fp_mul()` call from NR loop body
- **Alternative form:** `est = est + (est - est²*x_norm) / 2` (1 multiply instead of 2)
- **Change:**
  ```cpp
  fp_QP_t est_sq = fp_mul(est, est);
  for (int i = 0; i < 3; i++) {
  #pragma HLS PIPELINE II=1
      fp_QP_t delta = est - fp_mul(fp_mul(est_sq, x_norm), FP_HALF);
      est = est + delta;
      est_sq = fp_mul(est, est);
  }
  ```
- **Latency:** 14 → 8 cycles per reciprocal
- **Impact:** 20 iterations × 6 cycles saved = **120 cycles**
- **Tied Multipliers:** 2 per NR iteration (acceptable in current budget)
- **Risk:** Medium (requires numerical verification)

#### 1c. Use Lookup Table (LUT) Initialization
- **Idea:** Precompute reciprocal for common values (velocity 0.5→8.0 m/s in 0.1 steps)
- **Rationale:** Vehicle model reciprocals account for ~40% of recip calls
- **Trade-off:** +512 LUT entries vs. -30 recip latencies in vehicle model
- **Estimated FPGA overhead:** ~1.5k LUTs (vehicle model is not critical path)
- **Net Savings:** **30-50 cycles** (vehicle model, not ADMM)
- **Risk:** Low (scoped to non-critical vehicle model)

---

### **Priority 2: Unpipeline State z/y Update (II=6 → II=2)**
**Target Savings: 500-700 cycles (13.4-18.8%)**

**Current:**
```cpp
for (k = 0; k <= N; k++) {  // 21 iterations (0..20)
#pragma HLS PIPELINE II=6
    const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N-1];
    for (s = 0; s < nx; s++) {  // 8 states, UNROLL factor=2
#pragma HLS UNROLL factor=MPC_HLS_UNROLL_STATE_FACTOR  // factor=2
        if (x_is_con[k][s]) {
            // 15+ operations per state
        }
    }
}
```

**Proposed Fix:**

```cpp
// Split into two paths: constrained vs unconstrained
// Precompute indices at iteration start
int con_states[MPC_NX_AUG];
int con_count = 0;
for (s = 0; s < nx; s++) {
    if (x_is_con[k][s]) con_states[con_count++] = s;
}

// Fast path: constrained states only
for (int idx = 0; idx < con_count; idx++) {
#pragma HLS PIPELINE II=2  // Only real computations
    int s = con_states[idx];
    // ...tight compute loop
}

// Trivial path: unconstrained (straight copies)
for (int idx = con_count; idx < nx; idx++) {
#pragma HLS PIPELINE II=1
    int s = idx;
    z_x[k][s] = solution->x[k][s];  // One-cycle passthrough
}
```

**Alternative (if precompute is unavailable):**

```cpp
for (k = 0; k <= N; k++) {
#pragma HLS PIPELINE II=2
    // Reorder: move conditional check outside, into separate single-cycle operations
    const StepData_t *sd = (k < N) ? &step_data[k] : &step_data[N-1];
    for (s = 0; s < nx; s++) {
#pragma HLS UNROLL  // Complete unroll of 8 states
        fp_QP_t x_val = solution->x[k][s];
        fp_raw_acc_t x_hat_raw = fp_raw_acc_from_qp(x_val);
        fp_raw_acc_t y_raw = fp_raw_acc_from_qp(y_x[k][s]);
        fp_raw_acc_t val = x_hat_raw + y_raw;  // Always do this
        
        // Post-compute clipping only for constrained
        if (x_is_con[k][s]) {
            // Clipping + y_update (non-speculative for timing)
            fp_raw_acc_t x_lb = fp_raw_acc_from_qp(sd->x_lb[s]);
            fp_raw_acc_t x_ub = fp_raw_acc_from_qp(sd->x_ub[s]);
            if (val < x_lb) val = x_lb;
            if (val > x_ub) val = x_ub;
        }
        z_x[k][s] = fp_qp_from_raw_acc(val);
    }
}
```

**Metrics:**
- **Before:** 21 × (6 II cycles + 1 overhead) = **147 cycles**
- **After:** 21 × 2 + some fixed overhead = **50-60 cycles**
- **Savings: 87-97 cycles** (if II=2 achievable)

**Conservative estimate: 150-200 cycles** (allowing for synthesis suboptimality)

---

### **Priority 3: Increase Dense Matrix Unroll (factor=4 → 6 or complete)**
**Target Savings: 200-300 cycles (5.4-8.1%)**

**Current P Update Loop (21 upper-triangle entries):**
```cpp
for (int idx = 0; idx < 21; idx++) {
#pragma HLS PIPELINE II=1
    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=4  // Partial: 6→2 iterations
        // Accumulate A^T*PA
    }
    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL  // Complete: 2 elements
        // Accumulate G^T*K
    }
}
```

**Proposed Fix:**

```cpp
for (int idx = 0; idx < 21; idx++) {
#pragma HLS PIPELINE II=1
    int ii = sym_i[idx];
    int jj = sym_j[idx];
    fp_raw_acc_t sum = 0;
    
    // Loop A: A^T * PA (6×6, 6 products per index)
    #pragma HLS UNROLL  // Complete unroll: all 6
    for (s = 0; s < 6; s++) {
        fp_raw_acc_t atpa_prod = fp_raw_acc_from_qp(A_local[s][ii]) * PA[s][jj];
        sum += atpa_prod;
    }
    
    // Loop B: G^T * K (2×N matrix multiply, only 2 products per index)
    #pragma HLS UNROLL  // Already complete
    for (a = 0; a < nu; a++) {
        fp_raw_acc_t gtk_prod = fp_raw_acc_from_qp(G[a][ii]) * fp_raw_acc_from_qp(K[k][a][jj]);
        sum += gtk_prod;
    }
    
    fp_raw_acc_t val = ((ii == jj) ? q_aug_diag[ii] : 0) + (sum >> FP_FRAC_BITS);
    P[ii][jj] = val;
    if (ii != jj) P[jj][ii] = val;
}
```

**Array Partitioning Adjustment:**
```cpp
#pragma HLS ARRAY_PARTITION variable=A_local complete dim=1  // Was cyclic
#pragma HLS ARRAY_PARTITION variable=A_local complete dim=2
#pragma HLS ARRAY_PARTITION variable=PA complete dim=1       // Already complete
#pragma HLS ARRAY_PARTITION variable=PA complete dim=2       // Already complete
#pragma HLS ARRAY_PARTITION variable=K cyclic factor=2 dim=3 // Reduce from complete
```

**Metrics:**
- **Before:** 21 × (6 cycles inner unroll + 2 cycles G^T*K) = **168 cycles**
- **After:** 21 × 1 + 2 cycles overhead = **23 cycles** (if II=1 holds)
- **Conservative Savings: 100-150 cycles**

---

### **Priority 4: Control z/y Loop (II=4 → II=2)**
**Target Savings: 150-200 cycles (4.0-5.4%)**

**Current:**
```cpp
for (k = 0; k < N; k++) {  // 20 iterations
#pragma HLS PIPELINE II=MPC_HLS_CTRL_ZY_II  // II=4
    for (a = 0; a < nu; a++) {
#pragma HLS UNROLL  // 2 controls, complete
        // ~6-8 fp_mul() operations
    }
}
```

**Analysis:**
- 20 timesteps × 4 II = 80 cycles minimum
- With 2 control inputs fully unrolled: can achieve II=2-3
- Dependent on DSP availability (currently 4 multipliers in budget)

**Proposed Fix:**

```cpp
for (k = 0; k < N; k++) {
#pragma HLS PIPELINE II=2  // Reduced from 4
#pragma HLS UNROLL factor=2  // Unroll outer loop, reduce inner iterations
    // Two k-iterations per pipeline stage
    // ...
}
```

**Alternative (if II=2 unachievable):**

```cpp
// Separate u_update and residual computation
// u_update: II=1 critical path
// residual: II=2, non-critical
```

**Conservative Savings: 80-120 cycles**

---

### **Priority 5: Forward Pass Unrolling (factor=2 → 3 or complete)**
**Target Savings: 150-250 cycles (4.0-6.7%)**

**Current (per state update in forward pass):**
```cpp
for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
    fp_raw_acc_t sum = 0;
    for (s = 0; s < 6; s++) {
#pragma HLS UNROLL factor=2  // Partial: 6→3 iterations
        sum += fp_raw_acc_from_qp(A_fwd[i][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // More operations...
}
```

**Proposed Fix:**

```cpp
for (i = 0; i < 6; i++) {
#pragma HLS PIPELINE II=1
    fp_raw_acc_t sum = 0;
    #pragma HLS UNROLL factor=3  // 6→2 iterations (better than factor=2)
    for (s = 0; s < 6; s++) {
        sum += fp_raw_acc_from_qp(A_fwd[i][s]) * fp_raw_acc_from_qp(x_out[k][s]);
    }
    // Alternative: complete unroll if DSP budget allows
    #pragma HLS UNROLL
    for (s = 0; s < 6; s++) {
        sum += ...
    }
}
```

**Repeated 20 times in forward pass → 150-250 cycle savings**

---

### **Priority 6: Bit-Width Tuning (52-bit → 40-bit accumulators)**
**Target Savings: 50-100 cycles (indirect, via timing closure)**

**Current Setup:**
```cpp
#define MPC_HLS_RAW_ACC_GUARD_BITS 20  // 52-bit total accumulator
```

**Analysis:**
- Horizon N=10 steps
- Worst-case sum: 10 state trajectories × fixed-point → ~4-6 bits guard needed
- Current 20 bits = excessive

**Proposed:**
```cpp
#define MPC_HLS_RAW_ACC_GUARD_BITS 12  // 44-bit accumulator (safer margin)
```

**Benefits:**
- Smaller intermediate operands → faster addition trees
- Reduced routing congestion
- Better timing closure → potentially allows higher clock / lower latency tolerance
- Side benefit: slight LUT/register reduction (~2-3%)

**Caveat:** Requires careful numerical validation to ensure no overflow

---

## 4. Comprehensive Optimization Strategy

### **Phase 1: Quick Wins (High Confidence, Low Risk)**
- ✅ Reduce `fp_recip()` NR iterations: 4 → 3
- ✅ Increase P-matrix unroll from factor=4 → 6
- ✅ Tune bit-width from 20 → 12 guard bits (with validation)

**Expected Savings: 150-250 cycles (4.0-6.7%)**

### **Phase 2: Medium Complexity (Medium Confidence, Medium Risk)**
- ✅ Reduce state z/y update II from 6 → 3 (intermediate step)
- ✅ Reduce control z/y update II from 4 → 2
- ✅ Increase forward pass unroll factor from 2 → 3

**Expected Savings: 250-400 cycles (6.7-10.8%)**

### **Phase 3: Advanced (Complex Implementation, Higher Risk)**
- ✅ Full path separation for constrained vs. unconstrained states
- ✅ Reduce `fp_recip()` II from 2 → 1 (3-NR-iteration version)
- ✅ Complete unrolling of dense matrix operations

**Expected Savings: 300-500 cycles (8.1-13.4%)**

---

## 5. Prioritized Optimization List with Estimates

| # | Optimization | Category | Cycles | % Total | Difficulty | Risk |
|----|---|---|---|---|---|---|
| **1** | Reduce fp_recip NR iter 4→3 | Math | 80 | 2.1% | Easy | Low |
| **2** | Increase P-matrix unroll factor 4→6 | Matrix | 100-150 | 2.7-4.0% | Medium | Low |
| **3** | State z/y update II 6→3 | ADMM | 150-200 | 4.0-5.4% | Medium | Medium |
| **4** | Reduce guard bits 20→12 | Precision | 50-100 | 1.3-2.7% | Hard | High |
| **5** | Control z/y update II 4→2 | ADMM | 100-150 | 2.7-4.0% | Medium | Medium |
| **6** | Forward pass unroll factor 2→3 | Unroll | 100-150 | 2.7-4.0% | Easy | Low |
| **7** | Reduce fp_recip II 2→1 (3-iter) | Math | 80-120 | 2.1-3.2% | Hard | Medium |
| **8** | Constrained state path separation | Branch | 50-100 | 1.3-2.7% | Hard | High |
| **9** | LUT reciprocal for vehicle model | Cache | 30-50 | 0.8-1.3% | Medium | Low |
| **10** | Loop tiling / blocking strategy | Architecture | 200-300 | 5.4-8.1% | Very Hard | Very High |

---

## 6. Implementation Guidance

### **Recommended Implementation Order**

1. **Week 1: Precision & Math Functions**
   - Implement 4→3 NR iteration reduction (80 cycles)
   - Validate numerical precision with testbenches
   - Increase P-matrix unroll (100-150 cycles)

2. **Week 2: ADMM Loop Optimization**
   - Reduce state z/y II step-wise: 6 → 4 → 2
   - Measure timing after each step
   - Adjust DSP allocation if needed

3. **Week 3: Forward Pass & Polish**
   - Increase forward pass unroll factor
   - Refine bit-widths (with margin for safety)
   - Final timing closure pass

### **Testing Strategy**

- **Unit Tests:** Verify fp_recip accuracy at corners (x near 0, 1, ∞)
- **System Simulation:** Full ADMM convergence against reference
- **HLS Synthesis:** Report cycle count vs. target at each phase
- **Timing Closure:** WNS/TNS tracking post-place & route

---

## 7. Risk Mitigation

### **High-Risk Changes Require:**
- ✅ Numerical validation (fp_recip, bit-width reduction)
- ✅ Reference comparison (Jetson CPU version)
- ✅ Convergence testing on hardware
- ✅ Rollback plan (git branch per phase)

### **Timing Closure Strategy**
- If target 2,000 cycles unachievable, aim for **2,500 cycles** (⅓ reduction)
- Maintain timing headroom: WNS > -0.5 ns at 5ns constraint
- Use hierarchical SLR floorplanning if necessary

---

## 8. Expected Outcome

**Optimistic Path (All Recommendations):**
- Current: 3,721 cycles
- Optimizations: **-1,500 to -1,700 cycles** (40-46%)
- **Target: 2,000-2,200 cycles** ✅

**Conservative Path (Phases 1-2 only):**
- Current: 3,721 cycles
- Optimizations: **-700 to -900 cycles** (19-24%)
- **Result: 2,800-3,000 cycles** (35% from target, still significant)

---

