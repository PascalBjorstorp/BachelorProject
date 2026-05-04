# MPC FPGA — Riccati-ADMM Solver for Kria K26

## Overview

HLS-compatible implementation of the Model Predictive Controller (MPC) using
Riccati recursion with ADMM (Alternating Direction Method of Multipliers)
for the F1/10th autonomous racing platform. Targets the AMD/Xilinx Kria K26
device family (`xck26-sfvc784-2LV-c`) used on Kria carrier boards.

## Architecture

```
CPU (ARM Cortex-A53)               FPGA (Programmable Logic)
┌──────────────────────┐          ┌────────────────────────────┐
│ ROS2 MPC Node        │  AXI-   │  mpc_fpga_top              │
│                      │  Lite   │  ┌────────────────────────┐ │
│ 1. Localization      │◄───────►│  │ Waypoint BRAM (32 KB)  │ │
│ 2. Frenet error      │         │  │ ADMM warm-start state  │ │
│ 3. Write state regs  │         │  ├────────────────────────┤ │
│ 4. Write ref buffers │         │  │ Vehicle linearization  │ │
│ 5. Trigger compute   │         │  │ QP setup (8×8 augment) │ │
│ 6. Read result regs  │         │  │ QP setup (8×8 augment) │ │
│                      │         │  │ Riccati-ADMM solver    │ │
│                      │         │  │ Control extraction     │ │
└──────────────────────┘          └────────────────────────────┘
```

### Key Design Decisions

| Feature | Choice | Rationale |
|---------|--------|-----------|
| Arithmetic | ap_fixed<32,16> internal math | No FPU on PL, deterministic latency |
| Vehicle params | Compile-time `#define` | Constant propagation, zero BRAM overhead |
| Solver | Riccati-ADMM | O(N·nx³) per iter vs O(N³) for dense QP |
| Warm-start | Static BRAM | Persists across calls, reduces iterations |
| State dimension | 8 (5 Frenet + 3 augmented) | Rate penalties + steering dynamics |
| Horizon | 19 steps × 40 ms = 0.76 s | Balances look-ahead, latency, and resources |

## File Structure

```
MPC_FPGA_Kria/
├── include/
│   ├── mpc_fpga_types.h        # Types, constants, vehicle params
│   ├── fp_math_hls.h           # Fixed-point arithmetic (inline + decl)
│   ├── riccati_solver_hls.h    # Solver API declaration
│   └── mpc_fpga_interface.h    # AXI-Lite register map (documentation)
├── src/
│   ├── fp_math_hls.cpp         # Trig functions (sin, cos, atan, recip)
│   ├── vehicle_model_hls.cpp   # Frenet linearization (Pacejka + full physics)
│   ├── riccati_solver_hls.cpp  # Riccati-ADMM solver core
│   ├── mpc_riccati_hls.cpp     # MPC QP construction + control extraction
│   └── mpc_fpga_top.cpp        # AXI-stream top + scalar simulation wrapper
├── testbench/
│   └── test_fpga_sim_drive.c   # Closed-loop FPGA simulation test
├── scripts/
│   └── run_hls.tcl             # Vitis HLS automation script
└── README.md                   # This file
```

## Building

### Prerequisites

- Vitis 2025.1+ (uses `vitis-run --mode hls`)
- Target part: `xck26-sfvc784-2LV-c` (Kria K26)

### Environment Setup

```bash
source /tools/Xilinx/2025.1/Vitis/settings64.sh
```

or 

```bash
vivado_env
```

### Synthesis (default run mode)

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
vitis-run --mode hls --tcl scripts/run_hls.tcl
```

Notes:
- In `scripts/run_hls.tcl`, default mode is `all`.
- This command was re-validated on this repo after latest changes.

### Full Flow (csim → synth → cosim → export)

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
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

### Best Low-Latency Kria Profile (Validated)

Use this profile to reproduce the current best latency/frequency point found in this workspace:

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
source /tools/Xilinx/2025.1/Vitis/settings64.sh

HLS_RUN_MODE=synth \
vitis-run --mode hls --tcl scripts/run_hls.tcl
```

Validated synthesis result for this profile:
- Estimated Fmax: 204.79 MHz
- Top latency: 19,935 cycles
- Estimated runtime from cycles/Fmax: 97.34 us

Targeted exploration note:
- Additional sweeps were run to balance max frequency and cycle count for minimum end-to-end latency.
- Structural refinements include packed state-constraint masking (`x_con_mask`) and AP-fixed profile tuning.
- This profile is now reflected in `scripts/run_hls.tcl` default synthesis settings.
- Sweep logs:
    - `logs/hls_targeted_sweep.csv`
    - `logs/hls_targeted_sweep_phase2.csv`
    - `logs/hls_targeted_sweep_phase3.csv`

Export the same profile for Vivado IP integration:

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
source /tools/Xilinx/2025.1/Vitis/settings64.sh

HLS_RUN_MODE=export \
vitis-run --mode hls --tcl scripts/run_hls.tcl
```

Exported IP package (latest run):
- `mpc_fpga_hls/kria_kv260/impl/ip/f1tenth_mpc_mpc_fpga_top_1_0.zip`

### GCC Testbench (quick validation)

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
gcc -O2 -I include -Wno-unknown-pragmas -c testbench/test_fpga_sim_drive.c -o test_fpga_sim_drive.o
g++ -O2 -I include -Wno-unknown-pragmas -x c++ -c src/riccati_solver_hls.cpp -o riccati_solver_hls.o
g++ -O2 -I include -Wno-unknown-pragmas -x c++ -c src/mpc_riccati_hls.cpp -o mpc_riccati_hls.o
g++ -O2 -I include -Wno-unknown-pragmas -x c++ -c src/vehicle_model_hls.cpp -o vehicle_model_hls.o
g++ -O2 -I include -Wno-unknown-pragmas -x c++ -c src/fp_math_hls.cpp -o fp_math_hls.o
g++ -O2 -I include -Wno-unknown-pragmas -x c++ -c src/mpc_fpga_top.cpp -o mpc_fpga_top.o
g++ -o test_fpga_sim \
    test_fpga_sim_drive.o \
    mpc_fpga_top.o \
    mpc_riccati_hls.o \
    riccati_solver_hls.o \
    vehicle_model_hls.o \
    fp_math_hls.o \
    -lm
./test_fpga_sim
```

All core sources are compiled as C++ for AP-native type support and HLS compatibility.

### Vivado Implementation and Bitstream

1. Generate the HLS IP package:

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
HLS_RUN_MODE=export vitis-run --mode hls --tcl scripts/run_hls.tcl
```

2. Open Vivado GUI and add this IP repository:
    `FPGA_Implementations/MPC_FPGA_Kria/mpc_fpga_hls/kria_kv260/impl/ip`
3. Add the packaged core (`mpc_fpga_top`) to your block design and connect AXI,
    clocks, resets, and interrupt as needed.
4. Run Vivado flow: Generate Output Products → Synthesis → Implementation →
    Generate Bitstream.

## Interface

### AXI-Stream Compute Interface

The HLS top consumes packed 128-bit stream words on `input_stream`. The
scalar wrapper `mpc_fpga_top_scalar` is only used by the standalone simulation
harness.

1. Pack beat 0 as `[e_y | e_psi | vx | vy]`
2. Pack beat 1 as `[omega | steering | horizon_length | prev_accel]`
3. Pack words 8.. as repeated per-step groups:
   `[ref_ey | ref_vx | ref_kappa | ref_left | ref_right]`
4. Trigger `ap_start` and read `out_steering_fp`, `out_accel_fp`,
   `out_status`, and `out_iterations`

**Note:** Frenet errors (`e_y`, `e_psi`) must be computed on the ARM/CPU side
using the vehicle's localized pose and the first reference waypoint. The FPGA
expects pre-computed Frenet frame errors, not world-frame coordinates.

### AXI-Lite Register Map

Registers are auto-generated by Vitis HLS from the `s_axilite` pragmas.
See `include/mpc_fpga_interface.h` for offsets (matches generated driver
`mpc_fpga_hls/kria_kv260/impl/misc/drivers/mpc_fpga_top_v1_0/src/xmpc_fpga_top_hw.h`).

## Resource Estimates

| Resource | Estimated Usage | Available (legacy ZU3EG ref) | Utilization |
|----------|----------------|--------------------|----|
| LUT | ~25,000 | 70,560 | ~35% |
| FF | ~15,000 | 141,120 | ~11% |
| DSP48E2 | ~80–120 | 360 | ~22–33% |
| BRAM (36Kb) | ~25 | 216 | ~12% |

*Estimates based on solver complexity. Re-run synthesis on the K26 target and
use generated reports as the source of truth.*

### BRAM Breakdown

| Array | Size | BRAMs (36Kb) |
|-------|------|------|
| trajectory[64] | 2 KB | 1 |
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

For runtime-tunable weights, add AXI-Lite registers (increases resource usage).
