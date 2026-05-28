# MPC FPGA — Riccati-ADMM Solver for Kria K26

HLS implementation of the Model Predictive Controller used in the SDU-APEX
thesis. The solver combines a Riccati recursion with ADMM (Alternating
Direction Method of Multipliers) and is built for the AMD/Xilinx Kria K26
SoM (`xck26-sfvc784-2LV-c`).

The kernel is packaged as an OpenCL kernel (`mpc_fpga_top_opencl`) and is
invoked from the Jetson-side host application through XRT (see
[../../f1tenth_communication/](../../f1tenth_communication/)).

## Architecture

```
Jetson (Host, XRT/OpenCL)              Kria K26 (PL — mpc_fpga_top_opencl)
┌──────────────────────────────┐      ┌────────────────────────────────────┐
│ state_publisher / receiver   │      │ Input buffer (gmem0, 512-bit AXI-MM)│
│                              │      │   Header (8×32b) + 20 ref steps    │
│ 1. Localization + Frenet     │      │                                    │
│ 2. Pack 8 header words +     │ XRT  │ Unpack → Frenet linearization      │
│    20 × 8 reference words    ├─────►│ → ADMM-Riccati solver (warm-start) │
│ 3. enqueueMigrate(input)     │      │ → Control extraction               │
│ 4. enqueueTask(kernel)       │      │                                    │
│ 5. enqueueMigrate(output)    │◄─────┤ Output buffer (gmem1, 128-bit)     │
│ 6. Read steering/accel       │      │   [steering | accel | status | it] │
└──────────────────────────────┘      └────────────────────────────────────┘
```

The kernel is launched once per control tick. Persistent solver state
(warm-start `z`/`y`, previous gains) lives in the kernel's BRAM and survives
across calls.

## Key Design Decisions

| Feature           | Choice                                          | Rationale |
|-------------------|-------------------------------------------------|-----------|
| Arithmetic        | `ap_fixed<32,14>` (Q14.18) internal math        | No FPU on the PL, deterministic latency |
| Vehicle params    | Compile-time `#define` in `mpc_fpga_constants.h`| Constant propagation, zero BRAM overhead |
| Solver            | Riccati-ADMM                                    | O(N·nx³) per iteration vs O(N³) for dense QP |
| Warm-start        | Persistent BRAM (`MpcAdmmWarmStart_t`)          | Cuts iteration count between solves |
| State dimension   | 5 Frenet + 3 augmented = 8                      | Carries rate penalties + steering dynamics |
| Horizon           | 20 steps × 30 ms = **0.60 s**                   | `MPC_FPGA_HORIZON_STEPS = 20`, `MPC_FPGA_PREDICTION_DT_S = 0.03 f` |
| Control rate      | 200 Hz (5 ms)                                   | `MPC_FPGA_CONTROL_RATE_HZ`; cross-call scaling via `MPC_FPGA_CROSS_CALL_SCALE` |
| Max ADMM iters    | 50                                              | `MPC_FPGA_MAX_ADMM_ITER` |

The Riccati pass is unrolled per stage and the ADMM iteration is the only
sequential loop bound; the augmented 8-state model is compiled with a
constant horizon to let HLS materialize per-stage `K` gain ROMs.

## File Layout

```
MPC_FPGA_Kria/
├── include/
│   ├── fp_hls_config.hpp        # Per-domain ap_fixed widths (selectable profile)
│   ├── fp_math_hls.h            # Fixed-point arithmetic API
│   ├── fp_trig_lut_1024.h       # sin/cos LUT (QP profile)
│   ├── fp_trig_lut_fn_1024.h    # sin/cos LUT (narrow profile, vehicle model)
│   ├── fp_types_hls.hpp         # fp_QP_t / fp_FN_t / fp_P_t / fp_K_t / fp_MG_t
│   ├── mpc_cpu_compat.h         # CPU-side replay shim
│   ├── mpc_fpga_constants.h     # Horizon, control rate, vehicle params, IO sizing
│   ├── mpc_fpga_interface.h     # Transport contract (lane order, flags, status)
│   ├── mpc_fpga_types.h         # Solver-internal structs and dimensions
│   ├── mpc_riccati_hls.h        # MPC core API
│   └── riccati_solver_hls.h     # Riccati-ADMM solver API
├── src/
│   ├── fp_math_hls.cpp          # Trig, reciprocal, atan primitives
│   ├── vehicle_model_hls.cpp    # Frenet linearization (Pacejka tire model)
│   ├── riccati_solver_hls.cpp   # Riccati backward/forward + ADMM updates
│   ├── mpc_riccati_hls.cpp      # QP construction + control extraction
│   └── mpc_fpga_top.cpp         # OpenCL kernel top + scalar replay wrapper
├── testbench/
│   ├── test_fpga_csim_drive.c   # Closed-loop C-sim (FPGA boundary)
│   ├── tb_top_min.cpp           # Minimal HLS cosim driver
│   ├── tb_top_replay.cpp        # Replay-driven cosim from CSV logs
│   ├── tb_scalar_min.cpp        # Scalar non-HLS driver
│   ├── test_recip_accuracy.cpp  # Per-primitive precision check
│   └── fpga_tune_weights_hls.py # Weight sweep (CPU compat path)
├── MPC_FPGA/
│   └── hls_config.cfg           # Vitis HLS config for mpc_fpga_top_opencl
├── MPC_system/
│   ├── CMakeLists.txt           # Vitis system project (HLS + hw_link)
│   ├── hw_link/                 # v++ link config (synth/impl directives)
│   └── latest_impl/             # Symlinks to the most recent build outputs
└── Kria_platform/               # Vitis platform (PS init, BIF, device tree)
```

## Building

The flow has two stages: an **HLS kernel build** (produces an `.xo`) and a
**Vitis system link** (produces `.xclbin` + `.bit` + `.dtbo`). Both are
driven by Vitis 2025.2.

### Environment

Source the Vitis 2025.2 environment. The repo expects a global Vitis
install (`/tools/Xilinx/2025.2/Vitis/settings64.sh`)

### HLS kernel build (csim → synth → cosim)

The HLS configuration lives in
[`MPC_FPGA/hls_config.cfg`](MPC_FPGA/hls_config.cfg). Top function:
`mpc_fpga_top_opencl`; target clock: 5 ns (200 MHz); part:
`xck26-sfvc784-2LV-c`.

Open `MPC_FPGA/vitis-comp.json` in the Vitis 2025.2 IDE and run C-sim,
synthesis, or cosim from the GUI. Reports are written to
`MPC_FPGA/mpc_fpga_top_opencl/reports/`:

| Report                  | Content                              |
|-------------------------|--------------------------------------|
| `hls_compile.rpt`       | C-synthesis estimates                |
| `hls_cosim.rpt`         | RTL cosim cycle counts (min/avg/max) |
| `hls_impl_syn.rpt`      | Vivado RTL-synth resources + timing  |
| `hls_impl_pnr.rpt`      | Post-PnR resources + timing          |

### Hardware link (xclbin + bitstream)

The system project lives in [`MPC_system/`](MPC_system/). It is a CMake
front-end that calls `v++` with
[`MPC_system/hw_link/binary_container_1-link.cfg`](MPC_system/hw_link/binary_container_1-link.cfg).
Strategy: `Flow_PerfOptimized_high` synthesis,
`AltSpreadLogic_high` placement, `AggressiveExplore` post-route phys-opt.

Open `MPC_system/vitis-sys.json` in the IDE and run *Build → hw* (or
configure CMake with `-DVITIS_TARGET=hw`). Final artifacts are symlinked
into [`MPC_system/latest_impl/`](MPC_system/latest_impl/):

- `mpc_fpga_top_opencl.xclbin` — XRT loadable binary
- `mpc_fpga_top_opencl.xsa` — system archive
- `system.bit` — raw bitstream
- `timing_summary_routed.rpt` — Vivado post-route timing

The pre-built copies used by the runtime are kept in
[`../../FPGA_flash/`](../../FPGA_flash/) (`MPC_FPGA.bit.bin`,
`MPC_FPGA.dtbo`, `mpc_fpga_top_opencl.xclbin`).

### CPU testbench (no Vitis required)

A closed-loop C++ test that exercises the same code paths as the HLS top is
compiled with g++. From `FPGA_Implementations/MPC_FPGA_Kria/testbench/`:

```bash
g++ -O2 -I ../include -I /home/akselmo/Vivado_program/2025.2/Vitis/include \
    -Wno-unknown-pragmas \
    test_fpga_csim_drive.cpp \
    ../src/fp_math_hls.cpp ../src/vehicle_model_hls.cpp \
    ../src/riccati_solver_hls.cpp ../src/mpc_riccati_hls.cpp \
    ../src/mpc_fpga_top.cpp \
    -lm -o test_fpga_csim_drive
./test_fpga_csim_drive
```

The `-I .../Vitis/include` path supplies the `ap_fixed.h` headers; replace
it with whichever Vitis 2025.2 include directory is available on the build
host.

## Transport Contract

The host packs the kernel input as a contiguous block of 32-bit words on
`gmem0` (512-bit AXI-MM), and the kernel writes one 128-bit beat to
`gmem1`. The layout is defined in
[`include/mpc_fpga_interface.h`](include/mpc_fpga_interface.h):

### Input — 8 header words + `MPC_HORIZON × 8` reference words

| Offset (32-bit words) | Field           | Encoding                       |
|-----------------------|-----------------|--------------------------------|
| 0                     | `e_y`           | raw Q14.18 (signed `int32`)    |
| 1                     | `e_psi`         | raw Q14.18                     |
| 2                     | `vx`            | raw Q14.18                     |
| 3                     | `vy`            | raw Q14.18                     |
| 4                     | `omega`         | raw Q14.18                     |
| 5                     | `steering`      | raw Q14.18                     |
| 6                     | `control_flags` | bitmask (see below)            |
| 7                     | `prev_accel`    | raw Q14.18                     |
| 8 + 8·i + 0..7        | step `i` refs   | `ref_ey, ref_epsi, ref_vx, ref_vy, ref_omega_ref, ref_kappa, ref_left, ref_right` |

Total payload: `8 + 20 × 8 = 168` 32-bit words = 672 B, transferred as
eleven 512-bit beats (the buffer is rounded up to a multiple of 16 lanes
per `INPUT_BUFFER_WORDS_512`).

Control flags (`mpc_fpga_interface.h`):

| Flag                                | Meaning                              |
|-------------------------------------|--------------------------------------|
| `MPC_FPGA_CTRL_FLAG_RESET_STATE`    | Clear persistent solver state        |
| `MPC_FPGA_CTRL_FLAG_FORCE_COLD_START`| Discard warm-start trajectory       |
| `MPC_FPGA_CTRL_FLAG_ZERO_DUALS`     | Zero ADMM duals before solve         |
| `MPC_FPGA_CTRL_FLAG_DEBUG_ECHO_INPUTS`| Bypass solver, echo header to gmem1|

### Output — single 128-bit beat on `gmem1`

| Lane (32-bit) | Field            | Encoding                          |
|---------------|------------------|-----------------------------------|
| 0             | `steering_fp`    | raw Q14.18                        |
| 1             | `accel_fp`       | raw Q14.18                        |
| 2             | `status`         | `MPC_FPGA_STATUS_{OK,MAX_ITER,NO_TRAJECTORY}` |
| 3             | `iterations`     | ADMM iterations used              |

### Fixed-point conversion

```c
/* float → raw QP (Q14.18) */
int32_t raw = (int32_t)(value * MPC_FPGA_QP_SCALE_F64);
/* raw QP → float */
float value = (float)raw / MPC_FPGA_QP_SCALE_F32;
```

`MPC_FPGA_QP_SCALE_I32 = 1 << 18 = 262 144`. Representable range:
±2¹³ ≈ ±8192; resolution: 1/262 144 ≈ 3.8·10⁻⁶.

Frenet errors `e_y` and `e_psi` are computed host-side before packing — the
kernel never sees a world-frame pose.

## Results

The numbers below are read directly from the synthesis and Vivado
implementation reports of the kernel in
[`MPC_FPGA/mpc_fpga_top_opencl/reports/`](MPC_FPGA/mpc_fpga_top_opencl/reports/)
and from
[`MPC_system/latest_impl/IMPLEMENTATION_VERIFICATION.md`](MPC_system/latest_impl/IMPLEMENTATION_VERIFICATION.md).

### Latency (RTL cosim, Verilog, `hls_cosim.rpt`)

| Case | Cycles | Wall-clock @ 200 MHz |
|------|--------|----------------------|
| min  | 5 007  | **25.0 µs**          |
| avg  | 6 838  | **34.2 µs**          |
| max  | 12 898 | **64.5 µs**          |

Comfortably within the 5 ms control period (200 Hz) and the 30 ms
prediction step.

### Timing closure (Vivado, post-route)

Target period: 5.000 ns (200 MHz). Post-route WNS = **+0.038 ns**, TNS =
0.000 ns, WHS = +0.010 ns → **timing met**.

### Resource utilization (Vivado, post-route, `hls_impl_pnr.rpt`)

| Resource | Used   | Available (K26) | Utilization |
|----------|--------|-----------------|-------------|
| LUT      | 53 901 | 117 120         | 46.0 %      |
| FF       | 63 915 | 234 240         | 27.3 %      |
| DSP      | 1 086  | 1 248           | 87.0 %      |
| BRAM_18K | 99     | 288             | 34.4 %      |
| URAM     | 0      | 64              | 0 %         |

The DSP utilization is the binding constraint and reflects the per-stage
Pacejka linearization and the `ap_fixed<32,14>` multiplies in the Riccati
pass.

## CPU/FPGA Parity

The CPU MPC ([../../MPC/](../../MPC/)) and the FPGA kernel share:

- the same horizon and prediction step (`PREDICTION_HORIZON = 20`,
  `PREDICTION_DT_SECONDS = 0.03 f`),
- the same vehicle parameters (`mpc_fpga_constants.h` mirrors the CPU
  defines), and
- the same Q14.18 transport format on the wire.

`include/mpc_cpu_compat.h` exposes the kernel's solver through a CPU-side
API so the test harnesses in
[`../../tools/mpc_replay/`](../../tools/mpc_replay/) can replay logged
trajectories through the FPGA solver without an actual FPGA in the loop.
