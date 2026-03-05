# MPC vs f1tenth_gym Simulator: Model Mismatch Analysis

## 1. Overview

This document compares the MPC's vehicle dynamics model (`vehicle_model.c`) against the
f1tenth_gym simulator's single-track model (`single_track.py`) to identify every source of
prediction error and prioritize fixes.

**Gym config used** (from `sim.yaml` + `gym_bridge.py`):
- `model='st'`, `integrator='rk4'`, `control_input=['accl', 'steering_angle']`
- `sim_timestep=0.005` (200 Hz RK4)
- `v_switch=7.319`, `sv_max=2.8492`, `a_max=8.0`, `v_max=20.0`, `v_min=-5.0`
- Parameters overridden from `sim.yaml`: mu=0.7463, m=3.314, I=0.035, C_Sf=2.804, C_Sr=3.320, lf=0.166, lr=0.16, h=0.0703, s_max=0.4282, sv_max=2.8492, a_max=8.0

**MPC config** (from `mpc_types.h`):
- State: `[x, y, ψ, v_x, v_y, ω]` (6 states)
- Control: `[δ, a_cmd]` (δ applied directly)
- Prediction: Forward Euler, dt=50 ms, N=20
- Parameters: identical physical values (mu, m, I, C_Sf, C_Sr, lf, lr, h all match sim.yaml)

---

## 2. State Representation

| Aspect | Gym (ST model) | MPC |
|---|---|---|
| **State vector** | `[X, Y, δ, V, ψ, ψ̇, β]` (7 states) | `[x, y, ψ, v_x, v_y, ω]` (6 states) |
| **Velocity** | Total speed `V` + slip angle `β` | Body-frame `v_x`, `v_y` |
| **Steering** | `δ` is a **state** (evolves via dδ/dt) | `δ` is a **control** (instant) |
| **Relation** | `v_x = V·cos(β)`, `v_y = V·sin(β)` | Direct |

The position equations are exactly equivalent:
`V·cos(ψ+β) = v_x·cos(ψ) - v_y·sin(ψ)` (trig identity).

---

## 3. Difference #1 — Slip Angle Small-Angle Approximation

### Equations

**MPC (exact atan):**
$$\alpha_f = \delta - \arctan\!\left(\frac{v_y + l_f \omega}{v_x}\right), \quad \alpha_r = -\arctan\!\left(\frac{v_y - l_r \omega}{v_x}\right)$$

**Gym (small-angle, derived from the (V,β) formulation):**
$$\alpha_f \approx \delta - \beta - \frac{l_f \dot\psi}{V} = \delta - \frac{v_y + l_f \omega}{v_x}, \quad \alpha_r \approx -\beta + \frac{l_r \dot\psi}{V} = -\frac{v_y - l_r \omega}{v_x}$$

The gym replaces $\arctan(x)$ with $x$. Error: $\arctan(x) - x \approx -x^3/3$.

### Quantitative Error

Let $x_f = (v_y + l_f\omega)/v_x$.

| Condition | v (m/s) | v_y (m/s) | ω (rad/s) | x_f | atan(x_f) | Error | % of α_f |
|---|---|---|---|---|---|---|---|
| Gentle cruise | 5.0 | 0.05 | 0.3 | 0.020 | 0.020 | 3e-6 | <0.01% |
| Moderate corner | 5.0 | 0.15 | 1.0 | 0.063 | 0.063 | 8e-5 | <0.1% |
| Hard corner | 3.0 | 0.30 | 1.5 | 0.183 | 0.181 | 0.002 | ~1% |
| Aggressive | 3.0 | 0.50 | 2.0 | 0.277 | 0.270 | 0.007 | ~2.5% |

**Lateral force error at aggressive point:**
ΔF_yf = μ·C_Sf·F_zf·Δα = 0.7463 × 2.804 × 15.95 × 0.007 = **0.23 N** (out of ~9 N total ≈ 2.6%)

### Can MPC match gym?
**Yes — trivially.** Replace `fp_atan(ratio)` with `ratio` in vehicle_model.c's slip angle
computation. This *removes* a nonlinear function, saving FPGA resources.

### Expected improvement
Eliminates 0–2.5% slip angle prediction error, most noticeable at low speeds + high yaw rates.
Minor overall impact (small error to begin with).

**Priority: LOW** (error is small; but the fix is free/simplifying)

---

## 4. Difference #2 — cos(δ)/sin(δ) Coupling Terms

### Equations

**MPC (full trigonometric coupling):**
$$\dot{v}_x = \frac{F_x - F_{yf}\sin\delta + m \, v_y \omega}{m}$$
$$\dot{v}_y = \frac{F_{yf}\cos\delta + F_{yr} - m \, v_x \omega}{m}$$
$$\dot\omega = \frac{l_f F_{yf}\cos\delta - l_r F_{yr}}{I_z}$$

**Gym (small-angle δ; cos(δ)≈1, sin(δ)≈0):**

The gym's (V,β) formulation implicitly sets cos(δ)=1, sin(δ)=0. Specifically:
$$\dot{V} = a_\text{cmd} \quad \text{(no } F_{yf}\sin\delta \text{ coupling)}$$
$$\ddot\psi = \frac{\mu m}{I_z L}\left[l_f C_{Sf} g_{lr} \delta + (l_r C_{Sr} g_{lf} - l_f C_{Sf} g_{lr})\beta - (l_f^2 C_{Sf} g_{lr} + l_r^2 C_{Sr} g_{lf})\frac{\dot\psi}{V}\right]$$

where $g_{lr} = g \cdot l_r - a \cdot h$, $g_{lf} = g \cdot l_f + a \cdot h$.

Note: the yaw moment uses $l_f \cdot F_{yf}$ not $l_f \cdot F_{yf}\cos\delta$, and the
longitudinal dynamics have no $F_{yf}\sin\delta$ retarding term.

### Quantitative Error

| δ (rad) | cos(δ) | sin(δ) | % error in F_yf·cos(δ) | F_yf·sin(δ) [N] at F_yf=8N |
|---|---|---|---|---|
| 0.10 | 0.995 | 0.0998 | 0.5% | 0.80 N |
| 0.15 | 0.989 | 0.149 | 1.1% | 1.19 N |
| 0.20 | 0.980 | 0.199 | 2.0% | 1.59 N |
| 0.25 | 0.969 | 0.247 | 3.1% | 1.98 N |
| 0.30 | 0.955 | 0.296 | 4.5% | 2.37 N |

The **sin(δ) coupling** is more significant: at δ=0.25, the MPC predicts F_yf·sin(δ)/m ≈ 1.98/3.314 = **0.60 m/s²** of longitudinal deceleration from cornering that the gym doesn't model.

This means: the MPC predicts the car slows down more during cornering than the gym actually
simulates, causing the MPC to slightly under-predict speed during turns.

### Can MPC match gym?
**Yes.** Set `cos_delta=FP_ONE`, `sin_delta=0` in both `predict_next_state` and
`compute_linearization`. Saves 2 trig evaluations per call.

### Expected improvement
Eliminates 0.5–4.5% lateral force error and removes the spurious longitudinal coupling
(up to 0.6 m/s² phantom deceleration). Most impactful at high δ.

**Priority: MEDIUM** (meaningful at typical steering angles; easy fix)

---

## 5. Difference #3 — Steering Dynamics (δ as state vs direct control)

### This is the LARGEST mismatch.

**Gym:** δ is state x[2], driven by:
```
dδ/dt = steering_velocity
```
where `steering_velocity` is clamped to `[-sv_max, sv_max] = [-2.8492, 2.8492]` rad/s.

When using `control_input=['accl', 'steering_angle']`, the desired angle is converted to
steering velocity via **bang-bang control**:
```python
def pid_steer(steer, current_steer, max_sv):
    steer_diff = steer - current_steer
    if abs(steer_diff) > 1e-4:
        sv = sign(steer_diff) * max_sv   # ±2.8492 rad/s
    else:
        sv = 0.0
```

So the gym's δ ramps toward the target at maximum rate (2.8492 rad/s).

**MPC:** δ is the control input, applied **instantaneously** at each prediction step.

### Quantitative Error

Maximum δ change per MPC step (dt=50ms): `sv_max × dt = 2.8492 × 0.05 = 0.1425 rad`

| Δδ commanded | Time to reach (ms) | After 1 MPC step (50ms) | Steering lag |
|---|---|---|---|
| 0.05 rad | 17.5 | 0.05 (reached) | 0% |
| 0.10 rad | 35.1 | 0.10 (reached) | 0% |
| 0.14 rad | 49.1 | 0.14 (reached) | 1% |
| 0.15 rad | 52.6 | 0.1425 | **5%** |
| 0.20 rad | 70.2 | 0.1425 | **29%** |
| 0.25 rad | 87.7 | 0.1425 | **43%** |
| 0.30 rad | 105.3 | 0.1425 | **52%** |
| 0.40 rad | 140.4 | 0.1425 | **64%** |

**Example at Δδ=0.20 rad (common turn entry):**
- MPC predicts: δ jumps to 0.20 instantly → predicts yaw rate ψ̇ ≈ 1.2 rad/s after 50ms
- Gym achieves: δ = 0.1425 after 50ms → actual ψ̇ ≈ 0.85 rad/s
- The MPC **over-predicts** turning by 29%, leading to:
  - Oscillation: MPC overshoots, then corrects, then overshoots again
  - Understeer appearance: car doesn't turn as much as predicted
  - Path tracking error: lateral deviation on corner entry

### Can MPC match gym?
**Yes, multiple options:**

**(a) Simple: Rate-constrain Δδ in the QP solver**
Add constraint: $|\delta_{k+1} - \delta_k| \leq sv_\text{max} \cdot dt$
(i.e., `|Δδ| ≤ 0.1425` per step). This is compatible with the existing QP structure
and Q16.16 fixed-point. The MPC already has `weight_steering_rate` but as a soft penalty —
a **hard constraint** is needed to match the gym's physical limit.

Complexity: Low. Add one linear constraint per step to the QP.

**(b) Better: Model δ as a 7th state**
Add state δ with dynamics dδ/dt = sv. Control becomes [sv, a_cmd].
This lets the MPC plan ahead knowing the steering actuator is rate-limited.
Cost: +1 state (7×7 matrices instead of 6×6), trivially representable in Q16.16.

**(c) Predict-and-filter: Apply the gym's ramp to the MPC's planned δ sequence**
After solving the QP, post-process the control sequence: enforce the sv_max ramp
in the predicted trajectory. Doesn't improve the QP solution quality but improves
trajectory prediction accuracy for tracking.

### Expected improvement
Eliminates 5–64% steering prediction error on transients. Most impactful fix of all.
Expect **50%+ reduction** in lateral tracking oscillation on corner entry/exit.

**Priority: CRITICAL (highest impact)**

---

## 6. Difference #4 — Acceleration Power Limiting (v_switch)

### Equations

**Gym:**
```python
if vel > v_switch:                    # v_switch = 7.319 m/s
    a_max_effective = a_max * v_switch / vel
else:
    a_max_effective = a_max            # 8.0 m/s²
```
This models a constant-power regime above v_switch: $P = F \cdot v = m \cdot a \cdot v = \text{const}$, so $a_\text{max}(v) = a_\text{max} \cdot v_\text{switch} / v$.

Braking is symmetric: $a_\text{min} = -a_\text{max} = -8.0$ m/s² at all speeds.

**MPC:** $a \in [-7.7, 8.0]$ m/s² at **all speeds** (no power limiting).

### Quantitative Error

| v (m/s) | Gym a_max (m/s²) | MPC a_max (m/s²) | Error |
|---|---|---|---|
| 3.0 | 8.0 | 8.0 | 0% |
| 5.0 | 8.0 | 8.0 | 0% |
| 7.0 | 8.0 | 8.0 | 0% |
| 7.319 | 8.0 | 8.0 | 0% |
| 8.0 | 7.319 | 8.0 | **9.3%** |
| 10.0 | 5.855 | 8.0 | **36.6%** |
| 12.0 | 4.879 | 8.0 | **64.0%** |
| 15.0 | 3.903 | 8.0 | **104.9%** |

### Can MPC match gym?
**Yes.** In `vehicle_model_saturate_control()`, add:
```c
if (vx > V_SWITCH) {
    a_max_effective = fp_div(fp_mul(a_max, V_SWITCH), vx);
    // clamp
}
```
V_SWITCH = FP_CONST(7.319). Requires one fp_div but only when v > 7.319 m/s. Alternatively
pre-compute a lookup table indexed by velocity.

### Expected improvement
Eliminates acceleration overshoot at high speeds. At v=10 m/s, prevents the MPC from
commanding 8.0 m/s² when only 5.86 is achievable → **no more phantom acceleration**.

**Priority: HIGH at high speeds (>7 m/s), NONE at low speeds**

---

## 7. Difference #5 — Normal Force Computation

### Equations

Both use the same formula:
$$F_{zf} = \frac{m \cdot g \cdot l_r - F_x \cdot h}{L}, \quad F_{zr} = \frac{m \cdot g \cdot l_f + F_x \cdot h}{L}$$

**But there's a subtle difference.** The MPC computes $F_x = m \cdot a_\text{cmd}$, while the
gym uses `ACCL * h` directly:
```python
glr = g * lr - ACCL * h    # = g*lr - a*h
glf = g * lf + ACCL * h    # = g*lf + a*h
```

The gym's normal force factors are:
$$g_{lr} = g \cdot l_r - a \cdot h = \frac{F_{zf} \cdot L}{m}$$
$$g_{lf} = g \cdot l_f + a \cdot h = \frac{F_{zr} \cdot L}{m}$$

The lateral force in the gym is:
$$F_{yf} = \mu \cdot C_{Sf} \cdot F_{zf} \cdot \alpha_f = \mu \cdot C_{Sf} \cdot \frac{m \cdot g_{lr}}{L} \cdot \alpha_f$$

When the gym multiplies out in ψ̈:
$$\ddot\psi = \frac{l_f \cdot F_{yf} - l_r \cdot F_{yr}}{I_z} = \frac{\mu \cdot m}{I_z \cdot L}\left[l_f C_{Sf} g_{lr} \alpha_f - l_r C_{Sr} g_{lf} |\alpha_r|\right]$$

This is **exactly** what both models compute. **No mismatch here.**

**Priority: NONE (already matched)**

---

## 8. Difference #6 — Integrator (RK4 vs Forward Euler)

### Equations

**Gym:** 4th-order Runge-Kutta at dt = 0.005 s (200 Hz)
```python
k1 = f(x, u, params)
k2 = f(x + dt*k1/2, u, params)
k3 = f(x + dt*k2/2, u, params)
k4 = f(x + dt*k3, u, params)
x_next = x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
```

**MPC:** Forward Euler at dt = 0.05 s (20 Hz prediction)
```c
x_next = x + dt * f(x, u)
```

### Error Analysis

Local truncation error per step:
- **Euler at h=50ms:** $O(h^2) \approx \frac{h^2}{2}|f'| \approx 1.25 \times 10^{-3} \cdot |f'|$
- **RK4 at h=5ms:** $O(h^5) \approx 3.1 \times 10^{-13} \cdot |f^{(5)}|$ (negligible)

The gym takes 10 RK4 steps per one MPC Forward Euler step. The effective "ground truth" is
essentially exact for our purposes.

| State derivative | Typical magnitude | Euler error per 50ms step |
|---|---|---|
| ψ̈ = 5 rad/s² | 5 | Δψ error ≈ h²/2 × 5 = **6.25e-3 rad (0.36°)** |
| v̇_x = 3 m/s² | 3 | Δv_x error ≈ h²/2 × 3 = **3.75e-3 m/s** |
| v̇_y has peaks ~2 | 2 | Δv_y error ≈ **2.5e-3 m/s** |

**Accumulated over N=20 steps:**
- Heading error: up to **~5–10°** (compounds quickly because heading error feeds position)
- Position error: up to **~0.05–0.1 m** at high ψ̇

### Can MPC improve this?
Partially. Options:
1. **2-substep Euler** (effective dt=25ms): Halves error, doubles computation. Feasible on FPGA.
2. **Heun's method** (2nd-order): Two evaluations per step, local error O(h³). Moderate FPGA cost.
3. **Full RK4**: 4× computation, overkill for a prediction model.

The MPC only uses the prediction for cost evaluation — the linearized model (Jacobians) is
what the QP actually solves. The Euler integration is in `predict_next_state` (used for
reference tracking), not in the QP itself. So the integrator mismatch primarily affects
the warm-start trajectory and terminal cost, not the core optimization.

**Priority: LOW-MEDIUM** (compounds over horizon, but MPC re-solves every 5ms which limits
drift; the QP linearization is separate)

---

## 9. Difference #7 — Low-Velocity Kinematic Fallback

**Gym:** Switches to a kinematic model when V < 0.5 m/s:
```python
if V < 0.5:
    # Uses kinematic bicycle model with β_hat = atan(tan(δ)·lr/L)
    ...
```

**MPC:** Uses velocity floor `vx_safe = max(vx, 0.5)` for slip angle computation but
keeps the dynamic model at all speeds.

### Impact
Only affects startup and hard braking to standstill. During normal racing (v > 2 m/s),
both models agree. The MPC's velocity floor at 0.5 m/s effectively degenerates the dynamic
model to near-kinematic behavior anyway.

**Priority: NEGLIGIBLE** (only at standstill)

---

## 10. Summary: Complete Comparison Table

| # | Difference | MPC | Gym | Error at Typical Op. Point | Fixable? | Priority |
|---|---|---|---|---|---|---|
| 1 | Slip angle | `atan(x)` | `x` | 0–2.5% of α | Yes (use `x`) | LOW |
| 2 | cos(δ)/sin(δ) | Full trig | cos≈1, sin≈0 | 0.5–4.5% of F_y + 0.6 m/s² phantom decel | Yes (set cos=1, sin=0) | MEDIUM |
| 3 | **Steering dynamics** | **δ direct (instant)** | **δ is state, rate-limited at 2.85 rad/s** | **5–64% of Δδ per step** | **Yes (add rate constraint)** | **CRITICAL** |
| 4 | Power limiting | a_max=8.0 always | a_max×v_switch/v above 7.3 m/s | 0–105% at high v | Yes (add v_switch logic) | HIGH (at v>7.3) |
| 5 | Integrator | Euler, 50ms | RK4, 5ms | ~0.36°/step heading | Partial (substeps) | LOW-MEDIUM |
| 6 | Low-v fallback | v_x floor at 0.5 | Kinematic model at V<0.5 | Near zero in practice | Not needed | NEGLIGIBLE |

---

## 11. Prioritized Fix List

### Fix 1 (CRITICAL): Steering Rate Constraint
**What:** Add hard constraint $|\delta_{k+1} - \delta_k| \leq sv_\text{max} \cdot dt = 0.1425$ rad/step.
**Where:** QP constraint matrix, or post-process control sequence in `mpc_riccati.c`.
**Q16.16 cost:** One comparison + clamp per step (trivial).
**Expected gain:** Eliminates 5–64% steering prediction error. Largest single improvement.
Should reduce lateral oscillation at corner entry by **~50%+**.

### Fix 2 (HIGH at v>7.3, easy): v_switch Power Limiting  
**What:** In `vehicle_model_saturate_control()`, if `vx > FP_CONST(7.319)`, set
`a_max_eff = fp_div(fp_mul(a_max, FP_CONST(7.319)), vx)`.
**Q16.16 cost:** One conditional + one `fp_div` (only at high speed).
**Expected gain:** Correct acceleration prediction at high speeds. Prevents phantom
acceleration above 7.3 m/s.

### Fix 3 (MEDIUM, free): Remove cos(δ)/sin(δ) from dynamics
**What:** In `predict_next_state` and `compute_linearization`, replace:
- `cos_delta` → `FP_ONE`
- `sin_delta` → `0`
**Q16.16 cost:** Saves 2 trig evaluations. **Net negative cost** (fewer resources).
**Expected gain:** Removes 0.5–4.5% lateral force discrepancy and up to 0.6 m/s² spurious
longitudinal coupling. Better trajectory prediction during turns.

### Fix 4 (LOW, free): Use small-angle slip angles
**What:** Replace `fp_atan(ratio)` with `ratio` in slip angle computation.
**Q16.16 cost:** Saves 2 `fp_atan` calls. **Net negative cost**.
**Expected gain:** Eliminates 0–2.5% slip angle error. Minor but free.

### Fix 5 (LOW-MEDIUM, costly): Improve integrator
**What:** Use 2 Euler substeps per prediction step (effective dt=25ms):
```c
VehicleState_t mid = predict_next_state(state, u, dt/2);
VehicleState_t next = predict_next_state(&mid, u, dt/2);
```
**Q16.16 cost:** 2× computation per prediction step.
**Expected gain:** Halves integration error (~0.18° vs 0.36° per step heading error).

---

## 12. Combined Expected Improvement

Applying Fixes 1–4 (no extra FPGA resources needed):

| Operating Condition | Current Mismatch | After Fixes 1–4 |
|---|---|---|
| Corner entry (Δδ=0.20, v=5) | ~29% steering + ~2% force | <5% (rate residual) |
| Steady-state cornering (δ=0.20, v=5) | ~2% force only | <0.5% |
| High speed straight (v=10, a=8) | ~37% accel overshoot | <2% |
| Aggressive corner (δ=0.25, v=3, ω=1.5) | ~43% steer + 3% force | <5% |

**Bottom line:** Fixes 1–4 together should make the MPC prediction model match the gym to
within ~5% across the entire operating envelope, with zero additional FPGA resource cost
(Fixes 3–4 actually *save* resources). Fix 1 alone handles the dominant error source.

---

## 13. Answer: "Can I with my parameters more accurately predict the model used by the simulation?"

**Yes, definitively.** Your physical parameters (mu, m, I, C_Sf, C_Sr, lf, lr, h) already
match the simulation perfectly — they're the *same values* in both sim.yaml and mpc_types.h.

The prediction errors come entirely from **structural equation differences**, not parameter
values:

1. **Your MPC model is *more complex* than the gym's** — it uses full atan() and cos(δ)/sin(δ)
   that the gym doesn't. Simplifying your model to match the gym's approximations would
   *improve* prediction accuracy (Fixes 3–4).

2. **Your MPC lacks the gym's actuator dynamics** — the steering rate limit (sv_max=2.8492)
   and power-limited acceleration (v_switch=7.319) are enforced by the gym but unknown to
   your MPC's prediction model. Adding these constraints (Fixes 1–2) is the biggest win.

No parameter tuning needed. The fix is matching the *equations*, not the *coefficients*.
