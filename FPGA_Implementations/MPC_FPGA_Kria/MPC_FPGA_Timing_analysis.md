**MPC FPGA Timing Closure**

**Investigation, Root Causes & Fix Plan**

| **Target device**    | Xilinx KV260 - xck26-sfvc784-2LV-c                                                                                                                         |
| -------------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Target frequency** | 200 MHz                                                                                                                                                    |
| **HLS tool**         | Vitis HLS 2025.1                                                                                                                                           |
| **Document scope**   | All source files reviewed: fp_math_hls.cpp, vehicle_model_hls.cpp, riccati_solver_hls.cpp, mpc_riccati_hls.cpp, mpc_fpga_top.cpp, run_hls.tcl, all headers |
| **Document date**    | Tue Apr 14 2026                                                                                                                                            |

**Executive Summary**

HLS reports timing met at 200 MHz because its wire-delay model is idealised. Vivado implementation adds 0.3-0.8 ns of routing delay that HLS does not model, and the design currently carries no margin. A second class of issues - variable-length while loops, a 64-bit software CLZ intrinsic, contradictory array-partitioning pragmas, and a TCL double-assignment bug - create longer critical paths and higher resource congestion than HLS estimates. This document catalogues every root cause found across all source files and provides a prioritised fix plan.

Total issues found: **15** (5 critical, 5 high, 5 medium)

# 1\. Problem Statement

HLS synthesis completes and reports the design meeting the 5 ns (200 MHz) clock constraint. Post-implementation timing fails. The gap is structural, not random, and is caused by several overlapping issues detailed in this document.

## 1.1 The HLS-Implementation timing gap

Vitis HLS schedules logic against a clock period minus the clock uncertainty margin. With the current TCL settings:

set clock_period_ns "5.0" ; # 200 MHz target

set clock_uncertainty "5%" ; # => 0.25 ns consumed by HLS

Effective HLS budget: 4.75 ns

Typical routing delay: 0.3 - 0.8 ns on KV260 for this design density

Implementation slack: 4.75 - 0.8 = 3.95 ns - already negative

HLS uses a zero-wire-delay model: every operator result is assumed to arrive at the next pipeline register in zero routing time. Physical routing between DSP58E2 slices, BRAM outputs, and LUT chains adds delay HLS never sees. The minimum fix is to target HLS at 250-285 MHz and let the implementation tool use the headroom to close at 200 MHz.

## 1.2 Compounding factors

- Variable-length loops in fp_recip create long combinational feedback paths.
- A 64-bit software CLZ intrinsic synthesises to a slow priority encoder chain.
- Contradictory BRAM-storage + complete-partition pragmas on K, z_u, and y_u arrays.
- Multiplier resource limits set too low force the Riccati P-update loop to use slow LUT multipliers.
- A TCL variable double-assignment silently prevents the intended riccati_mul_limit value from reaching HLS.
- Newton-Raphson in fp_recip has an II=2 pragma on a loop whose carried-dependency latency is ~8 cycles - HLS silently degrades it.
- Up to 11 separate fp_recip module instances are created by the vehicle model with no allocation ceiling.

# 2\. Issue Catalogue

Issues are ordered by timing impact. Severity ratings: **CRITICAL** - directly causes timing failure; **HIGH** - significant degradation; **MEDIUM** - correctness or efficiency risk.

## Issue 1 - HLS clock target leaves no implementation margin

| **Severity** | **File**    | **Lines** | **Summary**                                                        |
| ------------ | ----------- | --------- | ------------------------------------------------------------------ |
| **CRITICAL** | run_hls.tcl | 27-38     | 5.0 ns HLS target leaves zero routing headroom for implementation. |

**Root cause:** HLS synthesises to 4.75 ns (5 ns minus 5% uncertainty). KV260 implementation adds 0.3-0.8 ns of routing delay to every cross-fabric path. The budget is exhausted before the router runs.

**Current TCL:**

set clock_period_ns "5.0"

set clock_uncertainty_in "5%" ; # 0.25 ns consumed - leaves 4.75 ns

**Fix:**

set clock_period_ns "4.0" ; # Target HLS at 250 MHz

set clock_uncertainty_in "12%" ; # 0.48 ns - models interconnect pessimism

\# Also add to force HLS to try harder on register balancing:

config_bind -effort high

config_schedule -effort high

## Issue 2 - TCL double-assignment bug: riccati_mul_limit always 4

| **Severity** | **File**    | **Lines** | **Summary**                                                                       |
| ------------ | ----------- | --------- | --------------------------------------------------------------------------------- |
| **CRITICAL** | run_hls.tcl | 120, 261  | riccati_mul_limit is assigned twice; second assignment silently overwrites first. |

**Root cause:** Line 120 sets riccati_mul_limit to 3. Line 261 resets it to 4. The macro -DMPC_HLS_RICCATI_MUL_LIMIT=4 is always passed to the compiler. Any environment sweep varying HLS_RICCATI_MUL_LIMIT is honoured after line 261, but the "default" profile is wrong. No past sweep result that relied on the default can be trusted.

**Current TCL:**

set riccati_mul_limit "3" ; # line 120 - intended tighter profile

... (~140 lines later) ...

set riccati_mul_limit "4" ; # line 261 - silently overwrites

**Fix:** Delete the duplicate assignment. Decide on one value and keep it. If the intent is mul_limit=4, remove line 120. If the intent is mul_limit=3, remove line 261. Also see Issue 6 regarding the correct target value.

## Issue 3 - fp_recip: variable-length while loops cannot be statically scheduled

| **Severity** | **File**        | **Lines** | **Summary**                                                                              |
| ------------ | --------------- | --------- | ---------------------------------------------------------------------------------------- |
| **CRITICAL** | fp_math_hls.cpp | 115-124   | Two while loops with data-dependent exit conditions create unschedulable feedback paths. |

**Root cause:** HLS cannot statically assign clock cycles to a loop whose trip count is determined at runtime. It generates a state-machine feedback loop (comparator → enable → shift → comparator) that is a multi-level combinational path through the clock period. LOOP_TRIPCOUNT hints affect only latency estimation in the report - they do not change what logic is synthesised.

// CURRENT - unschedulable feedback loop

while (x_norm > FP_ONE && shift < (MPC_HLS_RICCATI_WIDTH - 2)) {

# pragma HLS LOOP_TRIPCOUNT min=0 max=30 // affects reports only

x_norm >>= 1;

shift++;

}

while (x_norm &lt; FP_HALF && shift &gt; -(MPC_HLS_RICCATI_WIDTH - 2)) {

# pragma HLS LOOP_TRIPCOUNT min=0 max=30

x_norm <<= 1;

shift--;

}

**Fix:** Replace with a static leading-zero count on the ap_fixed bit pattern. The shift becomes a constant selected by a MUX tree (O(log W) LUT depth) rather than a feedback loop.

// FIXED - static mux tree, schedulable

ap_uint&lt;MPC_HLS_RICCATI_WIDTH&gt; abs_bits;

abs_bits.range(MPC_HLS_RICCATI_WIDTH-1, 0) =

abs_x.range(MPC_HLS_RICCATI_WIDTH-1, 0);

int lz = abs_bits.leading_zeros();

int shift = lz - MPC_HLS_RICCATI_INT_BITS; // known at schedule time

fp_QP_t x_norm = (shift >= 0) ? (abs_x >> shift) : (abs_x << (-shift));

// Fully unroll Newton iterations - no loop-carried dependency issues

fp_QP_t est = FP_QP_CONST(1.5);

est = fp_mul(est, FP_TWO - fp_mul(x_norm, est));

est = fp_mul(est, FP_TWO - fp_mul(x_norm, est));

est = fp_mul(est, FP_TWO - fp_mul(x_norm, est));

est = fp_mul(est, FP_TWO - fp_mul(x_norm, est));

## Issue 4 - fp_recip Newton loop: II=2 pragma is unachievable

| **Severity** | **File**        | **Lines** | **Summary**                                                                                                     |
| ------------ | --------------- | --------- | --------------------------------------------------------------------------------------------------------------- |
| **CRITICAL** | fp_math_hls.cpp | 127-131   | PIPELINE II=2 on a loop with ~8-cycle carried dependency. HLS silently degrades. Latency is ~32 cycles, not ~8. |

**Root cause:** Each loop iteration is:

est = fp_mul(est, (FP_TWO - fp_mul(x_norm, est)));

// ^--- 4 cycle latency

// ^--- another 4 cycles, dependent on inner result

// est updated: loop-carried dependency = 8+ cycles

With two serial fp_mul calls (each latency=4) and a loop-carried dependency through est, the minimum achievable II is 8-9, not 2. HLS will report this as an II violation in csynth.rpt. The four-iteration loop therefore takes approximately 32 cycles. Multiplied across 11+ invocations in the vehicle model and 10 horizon steps, this is a substantial latency contribution.

**Fix:** Fully unroll the Newton iterations as shown in Issue 3's fix. With unrolling, HLS can overlap the four multiply chains across cycles using multiple DSPs since there is no inter-iteration dependency. Latency reduces to approximately 4+4+4+4 = 16 cycles (pipelined through the chain) rather than 4 sequential 8-cycle blocks.

## Issue 5 - reciprocal_raw: \_\_builtin_clzll on a 64-bit cast

| **Severity** | **File**        | **Lines** | **Summary**                                                                                                |
| ------------ | --------------- | --------- | ---------------------------------------------------------------------------------------------------------- |
| **CRITICAL** | fp_math_hls.cpp | 30-31     | \_\_builtin_clzll forces a 64-bit priority encoder. fp_raw_acc_t is only 42 bits - wastes logic and depth. |

// CURRENT

unsigned long long abs_u = (unsigned long long)abs_det;

int lead_zeros = \_\_builtin_clzll(abs_u) // 64-bit priority encoder

\+ (2 \* FP_FRAC_BITS) - 64;

A 64-bit CLZ synthesises to a ripple-chain priority encoder approximately 6 LUT levels deep. Since fp_raw_acc_t = ap_int&lt;42&gt; (32 + 10 guard bits per TCL defaults), padding to 64 bits adds 22 zeroes then computes CLZ across all 64. Using ap_int::leading_zeros() on the native 42-bit type generates a 42-bit priority encoder roughly 5 LUT levels deep - shorter critical path, no wasted logic.

**Fix:**

// FIXED

typedef ap_uint&lt;MPC_HLS_RICCATI_WIDTH + MPC_HLS_RAW_ACC_GUARD_BITS&gt; raw_u_t;

raw_u_t abs_u = (raw_u_t)abs_det;

int lead_zeros = (int)abs_u.leading_zeros()

\+ (2 \* FP_FRAC_BITS)

\- (MPC_HLS_RICCATI_WIDTH + MPC_HLS_RAW_ACC_GUARD_BITS);

if (lead_zeros < 0) lead_zeros = 0;

if (lead_zeros > 62) lead_zeros = 62;

## Issue 6 - P-update loop demands 8 multipliers; only 4 budgeted

| **Severity** | **File**               | **Lines** | **Summary**                                                                                             |
| ------------ | ---------------------- | --------- | ------------------------------------------------------------------------------------------------------- |
| **HIGH**     | riccati_solver_hls.cpp | 276-296   | Step 7 backward pass needs 8 DSP muls per II=1 iteration. mul_limit=4 forces LUT fallback on 4 of them. |

The upper-triangle P update runs 21 times per backward step. Each iteration unrolls 6 products for A^T\*PA (unroll factor=4 → effectively 6/4 rounds of 3 muls each iteration, needing 6 DSPs at II=1) plus 2 products for G^T\*K (nu=2). Total: 8 parallel DSPs required. With MPC_HLS_RICCATI_MUL_LIMIT=4, HLS uses 4 DSPs and serialises the rest through LUT-based multipliers. LUT multipliers on UltraScale+ are approximately 3× slower than DSP58E2 at the same width, adding 1.5-2 ns to the path within those iterations. Across 21 iterations × 10 backward steps × 50 ADMM iterations, this is the dominant latency contributor.

**Fix:** Increase the multiplier limit for the Riccati hot path. The backward pass is the compute bottleneck and deserves a higher DSP budget:

// In riccati_solver_hls.cpp, replace line 58:

# pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_RICCATI_MUL_LIMIT

// With a per-scope override at the P-update section:

// (or set MPC_HLS_RICCATI_MUL_LIMIT=12 in TCL after fixing the

// double-assignment bug in Issue 2)

// In TCL (after fixing the bug):

set riccati_mul_limit "12"

## Issue 7 - K, z_u, y_u arrays: BIND_STORAGE(bram) contradicts ARRAY_PARTITION(complete)

| **Severity** | **File**               | **Lines**      | **Summary**                                                                                                                                                  |
| ------------ | ---------------------- | -------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **HIGH**     | riccati_solver_hls.cpp | 48-51, 442-446 | Complete partitioning + BRAM binding are contradictory. HLS creates tiny depth-10/depth-11 BRAMs with 2-cycle latency where registers or LUTRAM would serve. |

**K\[10\]\[2\]\[8\]:**

# pragma HLS ARRAY_PARTITION variable=K complete dim=3 // unfolds 8 → separate arrays

# pragma HLS ARRAY_PARTITION variable=K complete dim=2 // unfolds 2 → 16 arrays of depth 10

# pragma HLS BIND_STORAGE variable=K type=ram_2p impl=bram // CONTRADICTS above

// Result: 16 BRAMs of depth 10 - wasteful, 2-cycle latency, poor fit

**z_u\[10\]\[2\] and y_u\[10\]\[2\]:**

# pragma HLS BIND_STORAGE variable=z_u type=ram_2p impl=bram

# pragma HLS ARRAY_PARTITION variable=z_u complete dim=2

# pragma HLS BIND_STORAGE variable=y_u type=ram_2p impl=bram

# pragma HLS ARRAY_PARTITION variable=y_u complete dim=2

// Same contradiction - depth-10 BRAMs instead of registers

**Fix:** Remove the BIND_STORAGE pragmas from all three arrays. With complete partition on the inner dimensions, HLS will use registers or LUTRAM for the depth-10 slices, achieving 1-cycle read latency and eliminating the BRAM port-sharing bottleneck.

// Remove these three lines:

// #pragma HLS BIND_STORAGE variable=K type=ram_2p impl=bram

// #pragma HLS BIND_STORAGE variable=z_u type=ram_2p impl=bram

// #pragma HLS BIND_STORAGE variable=y_u type=ram_2p impl=bram

## Issue 8 - fp_atan INLINE expansion creates up to 11 fp_recip instances with no allocation limit

| **Severity** | **File**                                | **Lines**                  | **Summary**                                                                                                                                                   |
| ------------ | --------------------------------------- | -------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **HIGH**     | fp_math_hls.cpp / vehicle_model_hls.cpp | 228-263, 127-128, 133, 151 | fp_atan is INLINE with 3 branches, each calling fp_recip 0-2 times. 4 fp_atan calls in the vehicle model yields up to 11 fp_recip instances - no ceiling set. |

Because fp_atan is marked INLINE, HLS expands it at every call site. Each fp_atan invocation has three data-dependent branches synthesised as hardware with a result MUX:

- Branch 1 (|x| ≤ 0.5): no fp_recip call
- Branch 2 (0.5 < |x| ≤ 1.0): 1 fp_recip call
- Branch 3 (|x| > 1.0): up to 2 fp_recip calls (inv_x + optionally inv_den)

The vehicle model calls fp_atan four times (front_ratio, rear_ratio, Ba_f, Ba_r) plus three direct fp_recip calls (vx_safe, D_f, D_r). Without an instance limit, HLS may instantiate up to 4×2+3 = 11 separate fp_recip modules. Each module contains the variable while-loops (Issue 3) and the slow CLZ path (Issue 5), multiplying the critical-path logic by 11.

**Fix - add an allocation ceiling:**

// In compute_frenet_AB_hls():

# pragma HLS ALLOCATION function instances=fp_recip limit=4

// Forces resource sharing between branch paths - increases latency

// but eliminates 7+ redundant modules and drastically reduces

// LUT count and routing congestion.

## Issue 9 - riccati_pass_hls called twice without instance limit (cold-start duplication)

| **Severity** | **File**               | **Lines**        | **Summary**                                                                                                                                                    |
| ------------ | ---------------------- | ---------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **HIGH**     | riccati_solver_hls.cpp | 496-502, 553-558 | riccati_pass_hls is invoked in the cold-start branch (line 496) and the main ADMM loop (line 553). HLS may instantiate two full copies of this large function. |

The cold-start path is gated by if (!admm_state->initialized) - a data-dependent branch. HLS cannot statically determine mutual exclusivity with the ADMM-loop call. Without an explicit instance limit, two separate copies of riccati_pass_hls may be synthesised, approximately doubling DSP, LUT, and BRAM utilisation for the solver core. High resource utilisation leads to routing congestion, which is a major cause of timing failures even on paths that are individually short.

**Fix:**

// Add inside riccati_admm_solve_hls, before the cold-start branch:

# pragma HLS ALLOCATION function instances=riccati_pass_hls limit=1

// Forces resource sharing. HLS will time-multiplex the single instance

// between the cold-start and ADMM-loop call sites.

## Issue 10 - Warm-start copy loops cannot achieve II=1 against 2-port BRAM

| **Severity** | **File**               | **Lines** | **Summary**                                                                                            |
| ------------ | ---------------------- | --------- | ------------------------------------------------------------------------------------------------------ |
| **HIGH**     | riccati_solver_hls.cpp | 463-477   | II=1 warm-start copy with 8 reads per iteration (nx=8) from a 2-port BRAM. Minimum achievable II is 4. |

for (k = 0; k <= N; k++) {

# pragma HLS PIPELINE II=1

for (s = 0; s < nx; s++) { // nx=8 - fully unrolled

z_x\[k\]\[s\] = admm_state->z_x\[k\]\[s\]; // 8 reads from BRAM

y_x\[k\]\[s\] = admm_state->y_x\[k\]\[s\]; // 8 more reads

}

}

// 2-port BRAM: only 2 reads per cycle → minimum II = 8 not 1

The target local arrays (z_x, y_x) are completely partitioned on dim=2 and can accept all 8 writes in one cycle. The bottleneck is the source: admm_state->z_x is stored in the top-level static BRAM (bound via #pragma HLS BIND_STORAGE variable=admm_state type=ram_2p), providing only 2 read ports. HLS will silently degrade II to 4+ and emit a violation warning.

**Fix - partition the persistent ADMM state arrays on their inner dimension:**

// In mpc_fpga_compute_core(), add after the static declaration:

static AdmmState_t admm_state;

# pragma HLS ARRAY_PARTITION variable=admm_state.z_x complete dim=2

# pragma HLS ARRAY_PARTITION variable=admm_state.y_x complete dim=2

# pragma HLS ARRAY_PARTITION variable=admm_state.z_u complete dim=2

# pragma HLS ARRAY_PARTITION variable=admm_state.y_u complete dim=2

// This creates 8 parallel read ports for z_x/y_x, enabling II=1

## Issue 11 - fp_raw_acc_t at 42 bits forces DSP cascade; reducing guard bits would help

| **Severity** | **File**                       | **Lines**    | **Summary**                                                                                                                                                |
| ------------ | ------------------------------ | ------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **MEDIUM**   | fp_types_hls.hpp / run_hls.tcl | TCL defaults | 42-bit × 42-bit products require two cascaded DSP58E2 slices. Reducing RAW_ACC_GUARD_BITS from 10 to 4 gives 36-bit products with shorter cascade routing. |

DSP58E2 on UltraScale+ supports 27×18 natively (= 45 bits output). A 42×42 multiply requires two DSP slices in cascade with carry propagation between them, adding 0.2-0.4 ns compared to a single DSP. Reducing MPC_HLS_RAW_ACC_GUARD_BITS from 10 to 4 yields ap_int&lt;36&gt; - still requiring a 2-DSP cascade but with a shorter 36-bit carry chain. **Validate numeric range before changing.** The guard bits exist to absorb accumulation overflow in the inner products of the P-update.

**Fix (validate first):**

\# In run_hls.tcl:

set raw_acc_guard_bits "4" ; # down from 10 - verify no overflow

## Issue 12 - static initialized flag creates high-fanout reset net

| **Severity** | **File**         | **Lines** | **Summary**                                                                                                        |
| ------------ | ---------------- | --------- | ------------------------------------------------------------------------------------------------------------------ |
| **MEDIUM**   | mpc_fpga_top.cpp | ~70-82    | static int initialized=0 fans out to reset every downstream register. High-fanout nets degrade routing and timing. |

In HLS, a static variable that gates downstream state initialisation generates a single-bit register whose output fans out across all the logic it controls. With AdmmState_t containing 176 elements and MpcPersistState_t containing 6 elements, the initialized flag drives reset muxes on all of them. Vivado will automatically insert fanout buffers but these add 0.1-0.2 ns per level.

**Fix:** Move the one-time initialisation into a separate startup function called explicitly by the host after reset, or unconditionally initialise at synthesis time using ap_none reset directives.

## Issue 13 - fp_atan_tire_vm is defined but never called (dead code)

| **Severity** | **File**              | **Lines** | **Summary**                                                                                                                                               |
| ------------ | --------------------- | --------- | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **MEDIUM**   | vehicle_model_hls.cpp | 24-28     | fp_atan_tire_vm is defined with INLINE off + PIPELINE II=1, but no call site exists in the synthesis path. Dead code may confuse HLS resource allocation. |

The wrapper is declared but all actual atan calls in the vehicle model use fp_atan() (the full piecewise implementation). Because it has #pragma HLS INLINE off, HLS may attempt to synthesise a module for it even if unused, consuming scheduling budget. Remove or mark as simulation-only.

## Issue 14 - sdiv allocation pragma in invert_2x2_hls references non-existent operator

| **Severity** | **File**        | **Lines** | **Summary**                                                                                                                     |
| ------------ | --------------- | --------- | ------------------------------------------------------------------------------------------------------------------------------- |
| **MEDIUM**   | fp_math_hls.cpp | 55        | #pragma HLS ALLOCATION operation instances=sdiv limit=1 - no integer division exists in this function. Harmless but misleading. |

# pragma HLS ALLOCATION operation instances=sdiv limit=1

// sdiv = signed integer division. invert_2x2_hls uses reciprocal_raw()

// - no / operator present. Pragma has no effect but wastes a

// scheduling directive slot and suggests a misunderstanding of the

// operator taxonomy. Remove it.

# 3\. Prioritised Fix Plan

Execute in order. Each phase builds on the previous. Validate timing closure after each phase before continuing.

## Phase 1 - No-code changes (do immediately)

| **#** | **Issue**              | **File / Line**   | **Action**                                                     |
| ----- | ---------------------- | ----------------- | -------------------------------------------------------------- |
| 1     | Issue 1 - clock target | run_hls.tcl : 27  | Change clock_period_ns from 5.0 to 4.0; set uncertainty to 12% |
| 2     | Issue 1 - HLS effort   | run_hls.tcl       | Add config_bind -effort high and config_schedule -effort high  |
| 3     | Issue 2 - TCL bug      | run_hls.tcl : 261 | Delete the second set riccati_mul_limit assignment             |

_Expected result: HLS synthesis now has 0.8 ns of routing headroom. Implementation should close on all paths that were within 0.8 ns of the constraint._

## Phase 2 - Critical math library fixes

| **#** | **Issue**             | **File / Line**       | **Action**                                                                    |
| ----- | --------------------- | --------------------- | ----------------------------------------------------------------------------- |
| 4     | Issue 3 - while loops | fp_math_hls.cpp : 115 | Replace variable while-loops with ap_uint::leading_zeros() + static shift mux |
| 5     | Issue 4 - II=2 loop   | fp_math_hls.cpp : 127 | Fully unroll the 4 Newton iterations - remove the for loop                    |
| 6     | Issue 5 - CLZ         | fp_math_hls.cpp : 30  | Replace \_\_builtin_clzll with ap_uint::leading_zeros() on native type        |

_Expected result: fp_recip critical path drops from ~6 LUT feedback levels to ~3 LUT levels for the shift selection, and the Newton chain becomes a schedulable pipelined MAC sequence._

## Phase 3 - Resource and pragma fixes

| **#** | **Issue**                    | **File / Line**                         | **Action**                                                                  |
| ----- | ---------------------------- | --------------------------------------- | --------------------------------------------------------------------------- |
| 7     | Issue 7 - array pragmas      | riccati_solver_hls.cpp : 48-51, 442-446 | Remove BIND_STORAGE(bram) from K, z_u, y_u - keep ARRAY_PARTITION(complete) |
| 8     | Issue 6 - mul limit          | run_hls.tcl + riccati_solver_hls.cpp    | Set riccati_mul_limit to 12 in TCL; verify DSP budget on KV260 (<80%)       |
| 9     | Issue 8 - fp_recip instances | vehicle_model_hls.cpp                   | Add #pragma HLS ALLOCATION function instances=fp_recip limit=4              |
| 10    | Issue 9 - riccati_pass_hls   | riccati_solver_hls.cpp                  | Add #pragma HLS ALLOCATION function instances=riccati_pass_hls limit=1      |
| 11    | Issue 10 - warm-start II     | mpc_fpga_top.cpp                        | Add ARRAY_PARTITION complete dim=2 on admm_state.z_x/y_x/z_u/y_u            |

## Phase 5 - Medium-priority cleanup (after timing closes)

| **#** | **Issue**              | **File / Line**            | **Action**                                                          |
| ----- | ---------------------- | -------------------------- | ------------------------------------------------------------------- |
| 12    | Issue 11 - acc width   | run_hls.tcl                | Reduce raw_acc_guard_bits from 10 to 4 - validate no overflow first |
| 13    | Issue 12 - fanout      | mpc_fpga_top.cpp / XDC     | Add set_max_fanout 32 on initialized_reg\* in XDC                   |
| 14    | Issue 13 - dead code   | vehicle_model_hls.cpp : 24 | Remove fp_atan_tire_vm or move to simulation-only guard             |
| 15    | Issue 14 - sdiv pragma | fp_math_hls.cpp : 55       | Remove the sdiv ALLOCATION pragma                                   |
| 16    | Issue 15 - <<1 macros  | mpc_fpga_types.h : ~175    | Replace <<1 with fp_mul(x,FP_TWO) or add overflow static_asserts    |

# 4\. Diagnostic Tests

Run these before and after each phase to measure progress.

## Test 1 - Identify the actual critical path in Vivado

After implementation run, open the project in Vivado and run in the Tcl console:

report_timing_summary -delay_type min_max -max_paths 100 \\

\-input_pins -report_unconstrained \\

\-file timing_failures.rpt

Look for the worst negative slack (WNS) path. The LOGIC LEVELS count tells you which issue you're hitting:

- 8+ LUT levels in a loop-back path → Issue 3 (variable while loops)
- DSP48E2 → DSP48E2 → FDRE with no registers between → Issue 11 (42-bit cascade)
- RAMB36E2 → LUT6 → FDRE → timing report shows BRAM path dominant → Issue 7 or Issue 10
- High fanout nets (fanout > 200) on control signals → Issue 12

## Test 2 - Check II compliance in HLS synthesis report

After each HLS synthesis run, grep the schedule report:

grep -i 'ii violation\\|achieved ii\\|target ii' \\

mpc_fpga_hls/kria_kv260/syn/report/csynth.rpt

\# Key patterns to look for:

\# 'Achieved II: 8, Target II: 2' → Issue 4 (fp_recip Newton loop)

\# 'Achieved II: 4, Target II: 1' → Issue 10 (warm-start copy)

\# 'Achieved II: 2, Target II: 1' → mul_limit or BRAM port issue

## Test 3 - Validate the TCL mul_limit fix

After fixing Issue 2, run two HLS synth builds back to back:

HLS_RICCATI_MUL_LIMIT=4 vitis-run --mode hls --tcl scripts/run_hls.tcl

HLS_RICCATI_MUL_LIMIT=12 vitis-run --mode hls --tcl scripts/run_hls.tcl

\# Compare DSP count and estimated clock period in csynth.rpt

\# If period improves with higher limit, LUT fallback was occurring at 4

## Test 4 - Measure solver cycle count in testbench

Add a cycle counter to the cosim testbench to track per-solve latency:

// In testbench (simulation / host-side validation only):

uint64_t t0 = \_\_builtin_readcyclecounter();

mpc_fpga_top_scalar(ey, epsi, vx, vy, omega, steer,

ref_vx, ref_kappa, ref_left, ref_right,

MPC_HORIZON,

&out_s, &out_a, &status, &iters);

uint64_t dt = \_\_builtin_readcyclecounter() - t0;

printf("cycles: %llu iters: %d\\n", dt, iters);

// Target at 200 MHz for a 1 ms control loop budget:

// 200000 cycles total for the full solve (200 MHz × 1 ms)

## Test 5 - Resource utilisation headroom check

After Phase 3 changes, check DSP and LUT utilisation in the implementation report. If either exceeds 80%, routing congestion will cause timing failures independent of the critical path length:

\# In Vivado Tcl console after implementation:

report_utilization -file util_report.rpt

\# KV260 (xck26) resources:

\# DSP58E2: 1248 total - target < 1000 (80%)

\# LUT: 117504 total - target < 94000 (80%)

\# BRAM36: 144 total - target < 115 (80%)

# 5\. Quick Reference

## 5.1 Issue severity summary

| **#** | **Severity** | **File**               | **Title**                                  |
| ----- | ------------ | ---------------------- | ------------------------------------------ |
| 1     | **CRITICAL** | run_hls.tcl            | HLS clock leaves no routing margin         |
| 2     | **CRITICAL** | run_hls.tcl            | riccati_mul_limit double-assignment bug    |
| 3     | **CRITICAL** | fp_math_hls.cpp        | fp_recip variable while-loops              |
| 4     | **CRITICAL** | fp_math_hls.cpp        | fp_recip Newton II=2 unachievable          |
| 5     | **CRITICAL** | fp_math_hls.cpp        | \_\_builtin_clzll 64-bit priority encoder  |
| 6     | **HIGH**     | riccati_solver_hls.cpp | P-update needs 8 muls; only 4 budgeted     |
| 7     | **HIGH**     | riccati_solver_hls.cpp | K/z_u/y_u BRAM+partition contradiction     |
| 8     | **HIGH**     | vehicle_model_hls.cpp  | 11 fp_recip instances without ceiling      |
| 9     | **HIGH**     | riccati_solver_hls.cpp | riccati_pass_hls instantiated twice        |
| 10    | **HIGH**     | riccati_solver_hls.cpp | Warm-start copy II=1 vs 2-port BRAM        |
| 11    | **MEDIUM**   | run_hls.tcl            | 42-bit accum forces DSP cascade routing    |
| 12    | **MEDIUM**   | mpc_fpga_top.cpp       | initialized flag high fanout               |
| 13    | **MEDIUM**   | vehicle_model_hls.cpp  | fp_atan_tire_vm dead code                  |
| 14    | **MEDIUM**   | fp_math_hls.cpp        | sdiv pragma on function with no division   |
| 15    | **MEDIUM**   | mpc_fpga_types.h       | <<1 on ap_fixed with AP_WRAP overflow risk |

## 5.2 Key TCL parameter reference

| **Parameter**        | **Current default** | **Recommended** | **Notes**                   |
| -------------------- | ------------------- | --------------- | --------------------------- |
| clock_period_ns      | 5.0                 | 4.0             | Target 250 MHz in HLS       |
| clock_uncertainty_in | 5%                  | 12%             | Model routing delay         |
| riccati_mul_limit    | 4 (bug: set twice)  | 12              | Fix double-assign first     |
| vehicle_mul_limit    | 8                   | 8               | Adequate; leave             |
| raw_acc_guard_bits   | 10                  | 4 (validate)    | Shorter DSP cascade         |
| pipeline_loops       | 16                  | 16              | Leave; appropriate          |
| recip_ii             | 4                   | N/A after fix   | Redundant after Issue 3 fix |

## 5.3 Critical path expected latency - before and after fixes

| **Block**                    | **Before (est.)** | **After (est.)** | **Dominant factor**      |
| ---------------------------- | ----------------- | ---------------- | ------------------------ |
| fp_recip (one call)          | 30-50 cycles      | 12-16 cycles     | Variable while + Newton  |
| fp_atan (one call, branch 3) | 90-120 cycles     | 30-40 cycles     | 2× fp_recip              |
| compute_frenet_AB_hls        | 400-600 cycles    | 150-250 cycles   | 4× fp_atan + 3× fp_recip |
| Backward pass (one step)     | 140-200 cycles    | 80-120 cycles    | P-update mul count       |
| Full ADMM solve (50 iter)    | ~300k cycles      | ~120k cycles     | All above combined       |

_Estimates based on code analysis. Measure actual cycle counts using Test 4 to confirm._