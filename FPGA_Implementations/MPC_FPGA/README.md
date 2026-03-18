# MPC FPGA — Riccati-ADMM Solver for Ultra96-V2

## Overview

HLS-compatible implementation of the Model Predictive Controller (MPC) using
Riccati recursion with ADMM (Alternating Direction Method of Multipliers)
for the F1/10th autonomous racing platform. Targets the Xilinx Zynq
UltraScale+ ZU3EG on the Avnet Ultra96-V2 board.

## Architecture

```
CPU (ARM Cortex-A53)               FPGA (Programmable Logic)
┌──────────────────────┐          ┌────────────────────────────┐
│ ROS2 MPC Node        │  AXI-   │  mpc_fpga_top              │
│                      │  Lite   │  ┌────────────────────────┐ │
│ 1. Localization      │◄───────►│  │ Waypoint BRAM (32 KB)  │ │
│ 2. Closest waypoint  │         │  │ ADMM warm-start state  │ │
│ 3. Write state regs  │         │  ├────────────────────────┤ │
│ 4. Trigger compute   │         │  │ Frenet conversion      │ │
│ 5. Poll/IRQ done     │         │  │ Vehicle linearization  │ │
│ 6. Read result regs  │         │  │ QP setup (8×8 augment) │ │
│                      │         │  │ Riccati-ADMM solver    │ │
│                      │         │  │ Control extraction     │ │
└──────────────────────┘          └────────────────────────────┘
```

### Key Design Decisions

| Feature | Choice | Rationale |
|---------|--------|-----------|
| Arithmetic | Q16.16 fixed-point | No FPU on PL, deterministic latency |
| Vehicle params | Compile-time `#define` | Constant propagation, zero BRAM overhead |
| Solver | Riccati-ADMM | O(N·nx³) per iter vs O(N³) for dense QP |
| Warm-start | Static BRAM | Persists across calls, reduces iterations |
| State dimension | 8 (5 Frenet + 3 augmented) | Rate penalties + steering dynamics |
| Horizon | 20 steps × 50 ms = 1 s | Sufficient look-ahead for racing |

## File Structure

```
MPC_FPGA/
├── include/
│   ├── mpc_fpga_types.h        # Types, constants, vehicle params
│   ├── fp_math_hls.h           # Fixed-point arithmetic (inline + decl)
│   ├── riccati_solver_hls.h    # Solver API declaration
│   └── mpc_fpga_interface.h    # AXI-Lite register map (documentation)
├── src/
│   ├── fp_math_hls.c           # Trig functions (sin, cos, atan, recip)
│   ├── vehicle_model_hls.c     # Frenet linearization (Pacejka + full physics)
│   ├── riccati_solver_hls.c    # Riccati-ADMM solver core (~700 lines)
│   ├── mpc_riccati_hls.c       # MPC QP construction + control extraction
│   └── mpc_fpga_top.c          # Top-level AXI-Lite wrapper
├── testbench/
│   └── tb_mpc_fpga.c           # C testbench for csim/cosim
├── scripts/
│   └── run_hls.tcl             # Vitis HLS automation script
└── README.md                   # This file
```

## Building

### Prerequisites

- Vitis 2025.1+ (uses `vitis-run --mode hls`)
- Target part: `xczu3eg-sbva484-1-e` (Ultra96-V2)

### Environment Setup

```bash
source /tools/Xilinx/2025.1/Vitis/settings64.sh
```

### Synthesis (default run mode)

```bash
cd FPGA_Implementations/MPC_FPGA
vitis-run --mode hls --tcl scripts/run_hls.tcl
```

Notes:
- In `scripts/run_hls.tcl`, default mode is `synth`.
- This command was re-validated on this repo after latest changes.

### Full Flow (csim → synth → cosim → export)

```bash
cd FPGA_Implementations/MPC_FPGA
HLS_RUN_MODE=all vitis-run --mode hls --tcl scripts/run_hls.tcl
```

### Step-by-Step (Vitis 2025.1+)

```bash
# C simulation only
HLS_RUN_MODE=csim vitis-run --mode hls --tcl scripts/run_hls.tcl

# Synthesis only
HLS_RUN_MODE=synth vitis-run --mode hls --tcl scripts/run_hls.tcl

# Co-simulation only (requires prior synthesis)
HLS_RUN_MODE=cosim vitis-run --mode hls --tcl scripts/run_hls.tcl

# Export IP catalog (requires prior synthesis)
HLS_RUN_MODE=export vitis-run --mode hls --tcl scripts/run_hls.tcl
```

The exported IP can then be imported into Vivado IP Catalog.

### GCC Testbench (quick validation)

```bash
cd FPGA_Implementations/MPC_FPGA
gcc -O2 -I include -Wno-unknown-pragmas \
    -o tb_mpc \
    testbench/tb_mpc_fpga.c \
    src/mpc_fpga_top.c \
    src/mpc_riccati_hls.c \
    src/riccati_solver_hls.c \
    src/vehicle_model_hls.c \
    src/fp_math_hls.c \
    -lm
./tb_mpc
```

### Vivado Implementation and Bitstream

1. Generate the HLS IP package:

```bash
cd FPGA_Implementations/MPC_FPGA
HLS_RUN_MODE=export vitis-run --mode hls --tcl scripts/run_hls.tcl
```

2. Open Vivado GUI and add this IP repository:
    `FPGA_Implementations/MPC_FPGA/mpc_fpga_hls/ultra96v2/impl/ip`
3. Add the packaged core (`mpc_fpga_top`) to your block design and connect AXI,
    clocks, resets, and interrupt as needed.
4. Run Vivado flow: Generate Output Products → Synthesis → Implementation →
    Generate Bitstream.

## Interface

### Operating Modes

| Mode | Description |
|------|-------------|
| 0 | **Compute**: Convert state to Frenet, run MPC, return controls |
| 1 | **Load waypoint**: Store one waypoint into BRAM at `wp_index` |
| 2 | **Finalize**: Set trajectory size, reset solver warm-start |

### Mode 1: Trajectory Loading (called N_total times)

Write registers: `wp_index`, `wp_x_fp` .. `wp_right_bound_fp`, `mode=1`, trigger.

### Mode 0: MPC Compute (called at control rate, ~200 Hz)

1. Write vehicle state registers (`state_x_fp` .. `state_wp_idx`)
2. Write `mode=0`, trigger `ap_start`
3. Wait for `ap_done` (poll or interrupt)
4. Read `out_steering_fp`, `out_accel_fp`, `out_status`, `out_iterations`

### AXI-Lite Register Map

Registers are auto-generated by Vitis HLS from the `s_axilite` pragmas.
See `include/mpc_fpga_interface.h` for approximate offsets (exact addresses
are in the generated driver header after synthesis).

## Resource Estimates

| Resource | Estimated Usage | Available (ZU3EG) | Utilization |
|----------|----------------|--------------------|----|
| LUT | ~25,000 | 70,560 | ~35% |
| FF | ~15,000 | 141,120 | ~11% |
| DSP48E2 | ~80–120 | 360 | ~22–33% |
| BRAM (36Kb) | ~25 | 216 | ~12% |

*Estimates based on solver complexity. Actual numbers after synthesis.*

### BRAM Breakdown

| Array | Size | BRAMs (36Kb) |
|-------|------|------|
| trajectory[1024] | 32 KB | 8 |
| step_data[20] | 11 KB | 3 |
| ADMM state | 2 KB | 1 |
| K gains + misc | 2 KB | 1 |
| **Total** | **~47 KB** | **~13** |

## Latency Estimates (100 MHz)

| Case | ADMM Iters | Latency |
|------|-----------|---------|
| Warm-start (typical) | 5–10 | 0.8–1.6 ms |
| Cold-start | 20–30 | 3.2–4.8 ms |
| Worst case | 50 (max) | ~8 ms |

All cases well within the 5 ms control period (200 Hz).

## Conversion Notes

### Changes from MPC_experimental

1. **No dynamic memory**: All arrays statically sized
2. **No globals**: Persistent state via `static` locals in top function
3. **No stdio/stdlib/string.h**: Removed all C library deps from synthesized code
4. **Compile-time vehicle params**: F1/10th constants enable propagation
5. **Real-hardware physics**: Full atan slip angles, Pacejka tire saturation,
   cos(δ)/sin(δ) force resolution. No simulation-matching simplifications.
6. **Frenet on FPGA**: Global→Frenet conversion inside top function
7. **Sparsity preserved**: 6×6 dense block + identity rows 6–7
8. **Division-free critical path**: fp_atan uses fp_recip (Newton-Raphson)

### HLS Optimizations Applied

- `INTERFACE s_axilite` on all top-function ports
- `PIPELINE II=1` on inner matrix-operation loops
- `LOOP_TRIPCOUNT` on all loops for report accuracy
- `ARRAY_PARTITION` on K gains, P matrix (dim=1 + dim=2), M, G, PA arrays
- `BIND_OP impl=dsp` on critical multiplications
- `INLINE` on small helper functions and vehicle model
- **z_old elimination**: Dual residual computed inline during z-update,
  saving ~4KB BRAM and eliminating copy loop per ADMM iteration
- **memset elimination**: All zero-fills use explicit HLS-friendly loops
- **fp_atan division-free**: All divisions replaced with fp_recip (multiply-only)

## Tuning

MPC weights are `#define` constants in `include/mpc_fpga_types.h`. To retune:

1. Edit weight values (e.g., `MPC_W_LAT_ERROR`, `MPC_W_HEADING`)
2. Re-run synthesis: `HLS_RUN_MODE=synth vitis-run --mode hls --tcl scripts/run_hls.tcl`
3. Re-export IP

For runtime-tunable weights, add AXI-Lite registers (increases resource usage).
