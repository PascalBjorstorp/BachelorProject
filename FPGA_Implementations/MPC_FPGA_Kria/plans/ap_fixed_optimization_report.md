# ap_fixed Optimization Report — MPC Riccati-ADMM HLS Controller

---

## Executive Summary

After reading all 17 files, there are **two root causes** behind the resource and timing explosion, followed by several compounding issues. None of them are "ap_fixed used wrongly in isolation" — they are structural problems in how the type system is organized and how HLS pragmas interact with it.

**Root Cause A**: The `ap_fixed` types in `fp_types_hls.hpp` / `mpc_variable_types.hpp` are a **parallel type system that is entirely disconnected from the HLS compute core**. Every function that actually gets synthesized (`vehicle_model_hls.cpp`, `riccati_solver_hls.cpp`, `mpc_riccati_hls.cpp`, `fp_math_hls.cpp`) uses `fp_QP_t = int32_t`. When you introduced `ap_fixed` variables alongside these, HLS had to synthesize *two different operator families* (integer and ap_fixed) with no sharing possible between them.

**Root Cause B**: The inline `fp_mul` function carries `#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=4`. Because it is `static inline`, **every single multiplication in the entire design is forced to an independent 4-cycle DSP instance**. This eliminates all multiplier sharing and bloats both DSP count and latency simultaneously.

---

## Issue List (Priority Order)

---

### Issue 1 — CRITICAL: `#pragma HLS BIND_OP` Inside an Inline Function

**File**: `include/fp_math_hls.h`, function `fp_mul`

**Current code**:
```cpp
static inline fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b)
{
    int64_t product = (int64_t)a * (int64_t)b;
#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=4
    return (fp_QP_t)(product >> FP_FRAC_BITS);
}
```

**Problem**:
A `#pragma HLS BIND_OP` inside a function marked `inline` is applied at **every call site** after inlining. This means:
- Every `fp_mul(...)` call in `vehicle_model_hls.cpp` spawns an independent DSP with latency=4.
- Every `fp_mul(...)` call in `fp_math_hls.cpp` (sin, cos, atan, recip) spawns another.
- Every `fp_mul(...)` call in `mpc_riccati_hls.cpp` spawns another.
- HLS cannot share any of these because each is pinned to a specific DSP instance.
- The latency=4 constraint forces every pipeline stage that uses a multiply to be at least 4 cycles deep, propagating throughout the entire design's schedule.

**Fix**:
Remove the `#pragma HLS BIND_OP` from the inline. Apply DSP binding **only** in dedicated non-inline wrappers in hot-path files, which you already have in `vehicle_model_hls.cpp` (`fp_mul_vm`). That pattern is correct — the problem is the inline pragma polluting everything else.

```cpp
// FIXED: fp_math_hls.h
static inline fp_QP_t fp_mul(fp_QP_t a, fp_QP_t b)
{
    // No pragma here. Let HLS decide based on context and allocation pragmas.
    int64_t product = (int64_t)a * (int64_t)b;
    return (fp_QP_t)(product >> FP_FRAC_BITS);
}
```

The `fp_mul_vm` in `vehicle_model_hls.cpp` already correctly applies DSP binding in a non-inline wrapper. That wrapper is the right pattern to replicate for other hot paths.

---

### Issue 2 — CRITICAL: The ap_fixed Type System Is Disconnected from HLS Synthesis

**Files**: `include/fp_types_hls.hpp`, `include/mpc_variable_types.hpp`

**Problem**:
The types defined in these files (`fp_state_t`, `fp_acc_t`, `fp_K_t`, `fp_A_t`, etc.) are **never used** in any file that HLS actually synthesizes. Trace the include chain:

- `mpc_variable_types.hpp` → used by `mpc_runtime_tune.cpp` and `mpc_runtime_tune.h` (host-side only)
- The HLS files (`riccati_solver_hls.cpp`, `vehicle_model_hls.cpp`, `mpc_riccati_hls.cpp`) only include `fp_math_hls.h` and `mpc_fpga_types.h`, both of which use `fp_QP_t = int32_t`

When you introduced `ap_fixed` variables alongside the existing `int32_t` variables in the HLS flow, HLS sees two different data types for what should be the same arithmetic:
- `fp_QP_t` (int32_t) operators: 32-bit integer multiply/add units
- `ap_fixed<...>` operators: ap_fixed-specific multiply/add units

These **cannot share hardware**. HLS creates separate operator instances for each distinct type. The result is double the operators it should need, plus type-conversion logic at every boundary.

**Fix**:
Unify the type system by replacing `typedef int32_t fp_QP_t` in `fp_math_hls.h` with the ap_fixed type. This makes the entire HLS compute path use one type family.

```cpp
// FIXED: fp_math_hls.h — replace the typedef at the top

#ifdef __SYNTHESIS__
  #include <ap_fixed.h>
  // Single unified compute type for all HLS synthesis
  typedef ap_fixed<32, 16, AP_TRN, AP_WRAP> fp_QP_t;
#else
  // Software simulation: keep int32_t for host-side compatibility
  typedef int32_t fp_QP_t;
#endif
```

With this change, `fp_math_hls.h`, `mpc_fpga_types.h`, and all HLS compute files use one ap_fixed type. The manual `>> FP_FRAC_BITS` in `fp_mul` becomes unnecessary since ap_fixed handles truncation automatically on assignment, but it is harmless to keep it for the software path.

**Important**: The `FP_CONST`, `FP_MUL`, `FP_DIV` macros in `fp_math_hls.h` use `int64_t` casts. These work correctly for compile-time constant computation in `mpc_fpga_types.h` because the constants are computed as integer arithmetic first and then stored into the ap_fixed type. No changes needed to the macros themselves.

---

### Issue 3 — CRITICAL: `fp_A_t = ap_fixed<20, 4>` — The Fragmentation Type

**File**: `include/fp_types_hls.hpp`

**Current code**:
```cpp
typedef ap_fixed<MPC_HLS_A_WIDTH, MPC_HLS_A_INT_BITS, AP_TRN, AP_WRAP> fp_A_t;
// where MPC_HLS_A_WIDTH=20, MPC_HLS_A_INT_BITS=4
```

**Problem**:
This is the primary type-fragmentation issue I described in the previous discussion. `ap_fixed<20,4>` is a **unique operator width** that:
1. Cannot share any hardware operator with `ap_fixed<32,16>` operations (different widths = different DSPs/LUT configs)
2. Has only 4 integer bits — the A matrix contains entries like `dt * vx` where vx can be 20 m/s, giving 0.03 * 20 = 0.6 (fine), but also direct entries of 1.0 (the identity diagonal entries, encoded as 65536 in Q16.16) — wait, these need 17 bits to represent. 4 integer bits in ap_fixed<20,4> means values representable are only in [-8, 8), which cannot represent a Q16.16 value of 65536 (= 1.0). **This type is too narrow for A matrix entries and causes silent overflow.**
3. Generates 20×32 cross-width multiplies wherever `fp_A_t` interacts with `fp_QP_t`, which HLS maps to wide LUT chains instead of DSPs.

**Fix**:
Delete `fp_A_t` entirely. A matrix entries should use the standard `fp_core_t` (= the unified 32-bit core type). The A matrix values are explicitly computed in `vehicle_model_hls.cpp` and stored as `fp_QP_t A_fr[...]`, which is correct.

```cpp
// FIXED: fp_types_hls.hpp — DELETE these lines entirely:
// #ifndef MPC_HLS_A_WIDTH
// #define MPC_HLS_A_WIDTH 20
// #endif
// #ifndef MPC_HLS_A_INT_BITS
// #define MPC_HLS_A_INT_BITS 4
// #endif
// typedef ap_fixed<MPC_HLS_A_WIDTH, MPC_HLS_A_INT_BITS, AP_TRN, AP_WRAP> fp_A_t;
```

---

### Issue 4 — HIGH: `fp_acc_t` Has No Headroom Over the Core Type

**Files**: `include/fp_types_hls.hpp`, `include/mpc_variable_types.hpp`

**Current code**:
```cpp
typedef ap_fixed<MPC_HLS_ACC_WIDTH, MPC_HLS_ACC_INT_BITS, AP_TRN, AP_WRAP> fp_acc_t;
// defaults: MPC_HLS_ACC_WIDTH=32, MPC_HLS_ACC_INT_BITS=16
```

**Problem**:
`fp_acc_t` is the same width (32 bits) as the core type `fp_state_t`. An accumulator that is the same width as its inputs provides **zero overflow headroom**. In the vehicle model, terms like `D_f = vx² + front_num²` where vx can be 20 m/s (encoded as ~1.3×10⁶ in Q16.16) means vx² ≈ 1.7×10¹² before the `>> 16`. Storing this in a 32-bit accumulator silently saturates.

Additionally, in `mpc_variable_types.hpp`:
```cpp
typedef mpc_acc_t mpc_solver_acc_t;  // Same as core — wrong
```
The Riccati solver accumulations (`sum` variables in the P, G, K computations) are already correctly done in `int64_t` — but `mpc_solver_acc_t` being the same width as the core type would cause overflow if it were actually used.

**Fix**:
Change `fp_acc_t` to a genuinely wider type with 4 extra integer bits to cover accumulation of up to 8 products safely:

```cpp
// FIXED: fp_types_hls.hpp
#ifndef MPC_HLS_ACC_WIDTH
#define MPC_HLS_ACC_WIDTH 40     // Was 32 — 8 extra bits for accumulation headroom
#endif

#ifndef MPC_HLS_ACC_INT_BITS
#define MPC_HLS_ACC_INT_BITS 20  // Was 16 — 4 extra integer bits for sum overflow safety
#endif

typedef ap_fixed<MPC_HLS_ACC_WIDTH, MPC_HLS_ACC_INT_BITS, AP_TRN, AP_WRAP> fp_acc_t;
```

Update the static_assert to match:
```cpp
static_assert(MPC_HLS_ACC_WIDTH >= 32, ...);  // Was >= 24
```

Use `fp_acc_t` (or `mpc_acc_t`) explicitly in the vehicle model for intermediate sums (D_f, D_r denominators, Pacejka stiffness accumulations), then assign the result back to `fp_QP_t` after the accumulation is complete.

---

### Issue 5 — HIGH: `fp_recip_acc_t` at 48 Bits Creates Wide Datapaths

**File**: `include/fp_types_hls.hpp`

**Current code**:
```cpp
typedef ap_fixed<MPC_HLS_RECIP_ACC_WIDTH, MPC_HLS_RECIP_ACC_INT_BITS,
                 AP_TRN, AP_SAT> fp_recip_acc_t;
// defaults: 48 bits wide, 20 integer bits
```

**Problem**:
A 48-bit type creates 48-bit-wide datapaths in the Newton-Raphson reciprocal loop. On Xilinx/AMD FPGAs, 48-bit operations require 2 DSP48 slices each (DSPs are natively 27×18 or 18×27). In `fp_recip`, the 3-iteration Newton-Raphson loop with 48-bit types means **6 DSP pairs** just for the reciprocal. Additionally, `AP_SAT` on the accumulator adds comparator logic on what is already a critical timing path.

This type is also unused in synthesis (per Issue 2) — but was likely introduced to handle the reciprocal's precision. The 40-bit `fp_acc_t` from Issue 4's fix is sufficient.

**Fix**:
Delete `fp_recip_acc_t`. Replace its purpose with the widened `fp_acc_t` (40 bits) from Issue 4:

```cpp
// FIXED: fp_types_hls.hpp — DELETE these lines:
// #ifndef MPC_HLS_RECIP_ACC_WIDTH
// #define MPC_HLS_RECIP_ACC_WIDTH 48
// #endif
// #ifndef MPC_HLS_RECIP_ACC_INT_BITS
// #define MPC_HLS_RECIP_ACC_INT_BITS 20
// #endif
// typedef ap_fixed<...> fp_recip_acc_t;
// static_assert(...);
```

In `fp_math_hls.cpp`'s `fp_recip`, use `fp_acc_t` for the Newton-Raphson intermediate values instead, cast back to `fp_QP_t` on return.

---

### Issue 6 — HIGH: `AP_SAT` Overused — Adds Critical-Path Logic Everywhere

**File**: `include/fp_types_hls.hpp`

**Current code**:
```cpp
typedef ap_fixed<MPC_HLS_RECIP_WIDTH, MPC_HLS_RECIP_INT_BITS, AP_TRN, AP_SAT> fp_recip_t;
typedef ap_fixed<MPC_HLS_RECIP_ACC_WIDTH, MPC_HLS_RECIP_ACC_INT_BITS, AP_TRN, AP_SAT> fp_recip_acc_t;
```

**Problem**:
`AP_SAT` (saturation on overflow) adds a comparator and mux on **every assignment** to that type. For values on the critical timing path (like intermediate Newton-Raphson values), this adds 1-2 extra LUT levels per operation and can break timing closure.

`AP_SAT` is appropriate **only** at the output of the reciprocal function — as a final guard before returning to the caller — not for every intermediate computation inside.

**Fix**:
Change the Newton-Raphson internal accumulator to `AP_WRAP` (free, no extra logic). Apply `AP_SAT` only in a single cast at the return value of `fp_recip`:

```cpp
// FIXED: fp_types_hls.hpp — change recip type
typedef ap_fixed<MPC_HLS_RECIP_WIDTH, MPC_HLS_RECIP_INT_BITS,
                 AP_TRN, AP_WRAP> fp_recip_t;  // Changed SAT -> WRAP

// Internal NR accumulator uses the wider fp_acc_t (AP_WRAP)
```

```cpp
// FIXED: fp_math_hls.cpp — fp_recip return
fp_QP_t result = (fp_QP_t)est; // ap_fixed truncates, no SAT overhead
// Apply SAT only once as a clamp if needed:
if (result > FP_CONST(1000.0)) result = FP_CONST(1000.0);
if (result < FP_CONST(-1000.0)) result = FP_CONST(-1000.0);
return (sign < 0) ? fp_neg(result) : result;
```

---

### Issue 7 — HIGH: `reciprocal_64` and `invert_2x2_hls` Are INLINE in the Backward Pass

**File**: `src/riccati_solver_hls.cpp`

**Current code**:
```cpp
static int64_t reciprocal_64(int64_t det)
{
#pragma HLS INLINE          // <-- inlined into backward pass loop body
    // 3-iteration Newton-Raphson with 64-bit multiplies
    for (i = 0; i < 3; i++) {
#pragma HLS PIPELINE II=6
        int64_t prod = abs_det * est;   // 64x64 multiply
        ...
        int64_t adj  = est * corr;      // 64x64 multiply
        ...
    }
}

static int invert_2x2_hls(int64_t S[2][2], int64_t Si[2][2])
{
#pragma HLS INLINE          // <-- also inlined
    // Calls reciprocal_64, which is also inline
    int64_t inv_det = reciprocal_64(det);
    ...
}
```

**Problem**:
Both functions are `#pragma HLS INLINE`, meaning their logic is merged into the body of the `riccati_pass_hls` backward-pass loop. This causes:
1. The 64-bit multiplies from Newton-Raphson (`abs_det * est`, `est * corr`) become visible to the parent function's scheduler, competing for DSPs with the P, G, K, M matrix computations in the same backward-pass iteration.
2. The backward loop body now contains: P*A multiplies + A^T*PA multiplies + G computation multiplies + S inversion multiplies + K computation multiplies — all at once in one scheduling window.
3. With `#pragma HLS ALLOCATION operation instances=mul limit=MPC_HLS_RICCATI_MUL_LIMIT` (=4) in `riccati_pass_hls`, HLS must serialize all of these through 4 multiplier slots, creating enormous latency.
4. A 64-bit × 64-bit multiply requires 2–4 DSP48 slices. With 3 NR iterations inlined and unrolled, that is up to 12 DSPs purely for the reciprocal, before the matrix computations get any.

**Fix**:
Change both to `#pragma HLS INLINE off`. This makes them separate scheduled sub-functions with their own hardware, not competing with the matrix computation:

```cpp
// FIXED: riccati_solver_hls.cpp
static int64_t reciprocal_64(int64_t det)
{
#pragma HLS INLINE off          // Changed from INLINE
#pragma HLS PIPELINE II=1       // Pipeline the whole function call
    ...
}

static int invert_2x2_hls(int64_t S[2][2], int64_t Si[2][2])
{
#pragma HLS INLINE off          // Changed from INLINE
    ...
}
```

With `INLINE off`, HLS allocates the DSPs for these functions separately from the backward pass matrix operations. The backward pass scheduler sees only a latency number for the call, not the internal multiplies.

---

### Issue 8 — MEDIUM: Top-Level Multiplier Limit=2 May Cascade to Sub-Functions

**File**: `src/mpc_fpga_top.cpp`

**Current code**:
```cpp
void mpc_fpga_top(...)
{
#pragma HLS ALLOCATION operation instances=mul limit=2
```

**Problem**:
In Vitis HLS, an `ALLOCATION` pragma in a parent function applies to the function's own scope. Child functions that are *inlined* inherit the parent's allocation. Since `mpc_fpga_compute_core` is called (not inlined) from `mpc_fpga_top`, it gets its own scope. However, if HLS decides to inline `mpc_fpga_compute_core` (due to its `static` linkage and being called once), the limit=2 constraint cascades to the entire design.

Even if it does not cascade, the `#pragma HLS ALLOCATION operation instances=div limit=0` (unlimited div) alongside `limit=2` for mul creates a scheduling paradox: the 2x2 matrix inversion (`invert_2x2_hls`) uses both division (via `reciprocal_64`) and multiplication, and the scheduler tries to satisfy both constraints simultaneously.

**Fix**:
Remove the multiplier allocation pragma from `mpc_fpga_top` entirely. The sub-functions (`riccati_pass_hls`, `compute_frenet_AB_hls`) already carry their own, more precisely placed allocation pragmas. Let the top-level be unconstrained and rely on the sub-function pragmas:

```cpp
// FIXED: mpc_fpga_top.cpp — remove this line:
// #pragma HLS ALLOCATION operation instances=mul limit=2

// Keep only:
#pragma HLS ALLOCATION operation instances=div limit=0
```

---

### Issue 9 — MEDIUM: `fp_K_t` Is Redundant with `fp_state_t`

**File**: `include/fp_types_hls.hpp`

**Current code**:
```cpp
typedef ap_fixed<MPC_HLS_K_WIDTH, MPC_HLS_K_INT_BITS, AP_TRN, AP_WRAP> fp_K_t;
// defaults: 32 bits, 16 integer bits — identical to fp_state_t
```

**Problem**:
`fp_K_t` is bit-for-bit identical to `fp_state_t`. Having two types with the same underlying representation but different names means:
1. At any interface where `fp_K_t` meets `fp_state_t`, HLS may insert a conversion (even if it's a no-op).
2. Semantic aliases that collapse to the same type prevent HLS from sharing operators because each unique type gets its own operator binding.

**Fix**:
Delete `fp_K_t` and use `fp_state_t` (or the unified `fp_core_t` / `fp_QP_t` after the fix from Issue 2) for Riccati gain values. The Riccati solver already correctly stores K as `fp_QP_t K[MPC_HORIZON][MPC_NU][MPC_NX_AUG]`.

```cpp
// FIXED: fp_types_hls.hpp — DELETE:
// typedef ap_fixed<MPC_HLS_K_WIDTH, MPC_HLS_K_INT_BITS, AP_TRN, AP_WRAP> fp_K_t;
// static_assert(MPC_HLS_K_WIDTH > MPC_HLS_K_INT_BITS, ...);
```

---

### Issue 10 — MEDIUM: `mpc_variable_types.hpp` — All Semantic Types Collapse to One

**File**: `include/mpc_variable_types.hpp`

**Current code**:
```cpp
typedef fp_state_t mpc_core_t;
typedef fp_acc_t   mpc_acc_t;     // But fp_acc_t == fp_state_t (same 32 bits!)

typedef mpc_core_t mpc_P_t;       // All the same
typedef mpc_core_t mpc_K_t;
typedef mpc_core_t mpc_M_t;
typedef mpc_acc_t  mpc_solver_acc_t;   // Still same 32 bits
```

**Problem**:
All 20+ semantic types collapse to the same `ap_fixed<32,16>`. The differentiation is purely nominal. More critically, `mpc_acc_t` and `mpc_solver_acc_t` are supposed to be accumulator types with extra headroom but are the same width as the core type.

This is also where the disconnection from Issue 2 manifests: these types are defined but never appear in the HLS synthesis files' local variable declarations.

**Fix**:
After applying Issue 2's fix (unified `fp_QP_t = ap_fixed<32,16>`), simplify this file to its actual contract — two meaningful types:

```cpp
// FIXED: mpc_variable_types.hpp
#ifndef MPC_VARIABLE_TYPES_HPP
#define MPC_VARIABLE_TYPES_HPP

#include "fp_types_hls.hpp"

#ifdef __cplusplus

// Core arithmetic type: all states, parameters, I/O (32-bit Q16.16)
typedef fp_state_t  mpc_core_t;

// Accumulator type: for dot products and matrix multiply sums (40-bit Q20.20)
// Use this for any sum of more than 2 products before casting back to mpc_core_t.
typedef fp_acc_t    mpc_acc_t;

// All semantic aliases share the core type (operator sharing is the goal)
typedef mpc_core_t mpc_small_t;
typedef mpc_core_t mpc_angle_t;
typedef mpc_core_t mpc_vel_t;
typedef mpc_core_t mpc_bound_t;
typedef mpc_core_t mpc_cost_mid_t;
typedef mpc_core_t mpc_cost_large_t;
typedef mpc_core_t mpc_force_t;
typedef mpc_core_t mpc_jac_t;
typedef mpc_core_t mpc_rho_t;
typedef mpc_core_t mpc_residual_t;
typedef mpc_core_t mpc_state_var_t;
typedef mpc_acc_t  mpc_solver_acc_t;   // Correct: wider than core

// ... (keep remaining aliases pointing to mpc_core_t)

#endif
#endif
```

---

### Issue 11 — LOW: The `fp_math_hls.cpp` Math Functions — Inline vs Not

**File**: `src/fp_math_hls.cpp`

**Current code**:
```cpp
fp_QP_t fp_normalize_angle(fp_QP_t angle) { #pragma HLS INLINE ... }
fp_QP_t fp_sin(fp_QP_t angle)              { #pragma HLS INLINE ... }
fp_QP_t fp_cos(fp_QP_t angle)              { #pragma HLS INLINE ... }
fp_QP_t fp_atan(fp_QP_t x)                 { #pragma HLS INLINE ... }
fp_QP_t fp_atan_tire_approx(fp_QP_t x)     { #pragma HLS INLINE ... }
```

**Problem**:
All transcendental functions are `#pragma HLS INLINE`. When called from `vehicle_model_hls.cpp` (which calls `fp_atan_tire_vm` and `fp_sin`/`fp_cos` for steering), all their multiplications are inlined into the vehicle model function body. Combined with the BIND_OP issue (Issue 1), this means the vehicle model's scheduling sees: 20+ multiplications from trig expansions + 20+ from tire/load calculations + 5 from A-matrix rows + 4 from B-matrix rows, all at once, all competing for the 4-multiplier allocation limit.

**Fix**:
- Keep `fp_normalize_angle` and `fp_atan_small` as INLINE (they are pure arithmetic chains with no loops).
- Change `fp_sin`, `fp_cos`, and `fp_atan` to `#pragma HLS INLINE off`. These are complex enough to benefit from being sub-functions.
- Keep `fp_atan_tire_approx` as INLINE — it is a 3-term polynomial and is already called via `fp_atan_tire_vm` (which is already non-inline).
- Keep `fp_recip` as `#pragma HLS INLINE off` (it already is).

```cpp
// FIXED: fp_math_hls.cpp
fp_QP_t fp_sin(fp_QP_t angle)
{
#pragma HLS INLINE off    // Changed: was INLINE
#pragma HLS PIPELINE II=1
    ...
}

fp_QP_t fp_cos(fp_QP_t angle)
{
#pragma HLS INLINE off    // Changed: was INLINE
#pragma HLS PIPELINE II=1
    ...
}

fp_QP_t fp_atan(fp_QP_t x)
{
#pragma HLS INLINE off    // Changed: was INLINE
#pragma HLS PIPELINE II=1
    ...
}
```

---

## Consolidated Change Plan (Apply in This Order)

Apply these in order — each step can be validated in C simulation before moving to the next.

### Step 1: Fix `include/fp_math_hls.h`

1. Add `#ifdef __SYNTHESIS__` guard and change `fp_QP_t` to `ap_fixed<32, 16, AP_TRN, AP_WRAP>`.
2. Remove `#pragma HLS BIND_OP variable=product op=mul impl=dsp latency=4` from `fp_mul`.
3. Keep all macros (`FP_CONST`, `FP_MUL`, `FP_DIV`) as-is — they work with both `int32_t` and `ap_fixed` because they operate on literal constants, not type-dependent expressions.

```cpp
// --- BEFORE ---
typedef int32_t fp_QP_t;

// --- AFTER ---
#if defined(__SYNTHESIS__) && defined(MPC_HLS_BUILD)
  #include <ap_fixed.h>
  typedef ap_fixed<32, 16, AP_TRN, AP_WRAP> fp_QP_t;
#else
  typedef int32_t fp_QP_t;
#endif
```

### Step 2: Clean Up `include/fp_types_hls.hpp`

Delete: `fp_A_t`, `fp_recip_acc_t`, `fp_K_t` and their associated `#define` blocks and `static_assert` lines.

Change: `fp_acc_t` width from 32→40 bits, integer bits from 16→20.

Change: `fp_recip_t` overflow from `AP_SAT` → `AP_WRAP`.

Keep: `fp_state_t` (32, 16, TRN, WRAP), the new wider `fp_acc_t` (40, 20, TRN, WRAP), and `fp_stream_raw_t`.

The `static_assert` for `MPC_HLS_ACC_WIDTH >= 24` should be updated to `>= 32`.

### Step 3: Update `include/mpc_variable_types.hpp`

No structural changes needed beyond what flows from Step 2. Verify that `mpc_acc_t = fp_acc_t` now refers to the wider 40-bit type. Remove any `mpc_solver_acc_t` uses that were using the same-width type — they can stay as `mpc_acc_t` (now correctly wider).

### Step 4: Fix `src/fp_math_hls.cpp`

1. Change `fp_sin`, `fp_cos`, `fp_atan` from `#pragma HLS INLINE` to `#pragma HLS INLINE off`.
2. Add `#pragma HLS PIPELINE II=1` to each of those three functions.
3. In `fp_recip`: change the Newton-Raphson intermediate type from whatever it is to `fp_acc_t` (40-bit). Specifically, `est` and `adj` should be `fp_acc_t`, with a cast back to `fp_QP_t` at the return.

### Step 5: Fix `src/riccati_solver_hls.cpp`

1. Change `reciprocal_64` from `#pragma HLS INLINE` to `#pragma HLS INLINE off`, add `#pragma HLS PIPELINE II=1`.
2. Change `invert_2x2_hls` from `#pragma HLS INLINE` to `#pragma HLS INLINE off`.
3. No changes to the int64_t arithmetic inside these functions — it is correct and necessary given the cost weight magnitudes.

### Step 6: Fix `src/mpc_fpga_top.cpp`

Remove `#pragma HLS ALLOCATION operation instances=mul limit=2`. Keep only the div allocation pragma.

### Step 7: Fix `src/fp_math_hls.cpp` — `fp_recip` (detailed)

Current `fp_recip` uses `fp_QP_t est` internally. Change this internal type to the wider accumulator to improve Newton-Raphson convergence precision:

```cpp
fp_QP_t fp_recip(fp_QP_t x)
{
#pragma HLS INLINE off
    if (x == 0) return 0;

    int32_t sign = (x < 0) ? -1 : 1;
    fp_QP_t abs_x = fp_abs(x);

    int lead_zeros = __builtin_clz((unsigned int)(int32_t)abs_x);
    // Use wider type for Newton-Raphson accumulation
    fp_acc_t est = (fp_acc_t)(1 << lead_zeros);

    for (int i = 0; i < RECIP_ITERATIONS; i++) {
#pragma HLS PIPELINE II=2
#pragma HLS LOOP_TRIPCOUNT min=3 max=3
        fp_acc_t prod = (fp_acc_t)abs_x * est;
        fp_acc_t corr = (fp_acc_t)FP_ONE - prod;
        fp_acc_t adj  = est * corr;
        est = est + adj;
    }

    // Cast back to core type on return
    fp_QP_t result = (fp_QP_t)est;
    return (sign < 0) ? fp_neg(result) : result;
}
```

Note: `fp_acc_t` will need to be included in `fp_math_hls.h` or the corresponding include chain. Add `#include "fp_types_hls.hpp"` at the top of `fp_math_hls.h` (inside `__cplusplus` guard if needed, or unconditionally since both files are HLS-targeted).

---

## What You Should NOT Change

- **The int64_t P matrix in `riccati_solver_hls.cpp`**: P values can exceed INT32_MAX given the cost weight magnitudes (`W_LAT_ERROR = 15500` in Q16.16 ≈ 10⁹; after one backward step P[0][0] can approach 2×10⁹). The int64_t P is necessary for correctness. Do not attempt to change it.
- **The `#pragma HLS LOOP_TRIPCOUNT` annotations**: these are correct and help HLS schedule the bounded loops properly.
- **The `fp_mul_vm` non-inline wrapper in `vehicle_model_hls.cpp`**: this is the correct pattern for DSP-binding in a hot path. Keep it and its `#pragma HLS BIND_OP`.
- **The `#pragma HLS ARRAY_PARTITION` directives in the Riccati solver**: these enable parallel access to K, P, G arrays and are critical for achieving reasonable II on the inner loops.
- **The `B` matrix sparsity exploitation in the Riccati solver**: the hand-coded sparse accumulations for M, S, G are correct and important for reducing the number of multiplications per backward step.

---

## Summary Table

| # | File | Change | Impact |
|---|------|--------|--------|
| 1 | `fp_math_hls.h` | Remove `BIND_OP` from inline `fp_mul` | Eliminates DSP instance explosion; enables sharing |
| 2 | `fp_math_hls.h` | `fp_QP_t` → `ap_fixed<32,16,TRN,WRAP>` | Connects HLS compute to ap_fixed operator family |
| 3 | `fp_types_hls.hpp` | Delete `fp_A_t` (20-bit) | Eliminates operator fragmentation |
| 4 | `fp_types_hls.hpp` | Widen `fp_acc_t` to 40-bit | Real accumulation headroom |
| 5 | `fp_types_hls.hpp` | Delete `fp_recip_acc_t` (48-bit) | Removes wide datapath from critical timing path |
| 6 | `fp_types_hls.hpp` | `fp_recip_t`: SAT → WRAP | Removes comparator logic from critical path |
| 7 | `fp_types_hls.hpp` | Delete `fp_K_t` | Eliminates redundant type causing conversion overhead |
| 8 | `riccati_solver_hls.cpp` | `reciprocal_64`: INLINE → INLINE off | Isolates 64-bit NR multiplies from main scheduler |
| 9 | `riccati_solver_hls.cpp` | `invert_2x2_hls`: INLINE → INLINE off | Same: isolates 2x2 inversion compute |
| 10 | `mpc_fpga_top.cpp` | Remove `mul limit=2` | Removes top-level cascade that may starve sub-functions |
| 11 | `fp_math_hls.cpp` | `fp_sin/cos/atan`: INLINE → INLINE off | Reduces vehicle model's scheduling window |
| 12 | `mpc_variable_types.hpp` | `mpc_acc_t` → uses new 40-bit `fp_acc_t` | Fixes accumulator semantic alias |

Changes 1, 2, and 3 are the most impactful and should be applied first. Validating C simulation results match the int32_t baseline after Change 2 is the most important checkpoint.
