# SDU-APEX — FPGA-Accelerated MPC for the F1TENTH Platform

Bachelor's thesis project, *BSc in Robot Systems Engineering*, University of Southern Denmark (SDU), June 2026.

**Authors:** Aksel Munksgaard-Ottosen, Jonathan Dalsgaard, Pascal Nestås Bjørstorp

The full thesis is available under [Report/](Report/) (build `main.tex` with `latexmk` / `pdflatex` + `biber`).

---

## 1. Project Overview

This repository implements a complete autonomous-racing stack for the [F1TENTH](https://f1tenth.org/) 1/10-scale platform, with an emphasis on **real-time Model Predictive Control (MPC)** accelerated on an **AMD/Xilinx Kria K26 FPGA**.

The stack covers the full pipeline from sensors to actuators:

```
┌────────────────────────────────────────────────────────────────────────┐
│                            F1TENTH Vehicle                             │
│   LiDAR ──► Localization ──► Planner ──► Controller ──► VESC ──► Motor │
│                 ▲                            │                         │
│                 │                            ▼                         │
│             Odometry / IMU              FPGA (Kria K26)                │
└────────────────────────────────────────────────────────────────────────┘
```

Three controllers are implemented and compared in the thesis:

| Controller | Where it runs | Description |
|------------|---------------|-------------|
| **MPC (Riccati-ADMM)** | CPU (C) — Jetson | Path-tracking MPC, Frenet frame, dense Riccati-ADMM QP solver |
| **MPCC** | CPU (C) — Jetson | Contouring MPC, optimizes both tracking error and progress |
| **MPC-FPGA** | Kria K26 PL (HLS) | Same Riccati-ADMM core re-implemented in `ap_fixed<32,16>` for the FPGA |

The Jetson handles localization, planning, and high-level coordination; the Kria runs the FPGA MPC and is fed pre-computed Frenet errors and the reference horizon over a ROS2 UDP/AXI-Lite bridge.

---

## 2. Repository Layout

```
BachelorProject/
├── Report/                         # LaTeX thesis sources
│
├── MPC/                            # CPU MPC (Riccati-ADMM), C + ROS2
├── MPCC/                           # CPU MPCC, C + ROS2
├── FPGA_Implementations/
│   └── MPC_FPGA_Kria/              # Vitis HLS sources for the Kria K26 MPC kernel
├── FPGA_flash/                     # Pre-built .bit.bin / .dtbo / .xclbin for the Kria
│
├── f1tenth_communication/          # Jetson<->Kria ROS2 bridge (state_publisher / state_receiver)
├── f1tenth_control/                # Baseline controllers (pure-pursuit, Stanley, FTG)
├── f1tenth_localization/           # GPU AMCL + EKF + odometry fusion (CUDA / C++)
├── f1tenth_lateral_planner/        # Lateral obstacle-avoidance planner
├── f1tenth_planning/               # Raceline optimizer (TUM global_racetrajectory_optimization)
├── f1tenth_sim/                    # F1TENTH gym ROS2 bridge (simulation)
├── f1tenth_system/                 # Vehicle driver stack (VESC, LiDAR, joystick mux)
└──  f1tenth_parameters/             # Vehicle parameter identification + scripts
```

Per-package READMEs exist in [MPC/README.md](MPC/README.md), [FPGA_Implementations/MPC_FPGA_Kria/README.md](FPGA_Implementations/MPC_FPGA_Kria/README.md), [f1tenth_communication/README.md](f1tenth_communication/README.md), [f1tenth_sim/README.md](f1tenth_sim/README.md), [f1tenth_system/README.md](f1tenth_system/README.md), and [f1tenth_planning/README.md](f1tenth_planning/README.md).

---

## 3. Hardware

| Component | Role |
|-----------|------|
| F1TENTH chassis (1/10 scale) | Vehicle base |
| NVIDIA Jetson | Host CPU/GPU: localization, planning, ROS2 master |
| AMD/Xilinx Kria K26 (`xck26-sfvc784-2LV-c`) | FPGA accelerator for MPC |
| VESC 6-MkV | Motor controller (FOC) + IMU |
| Hokuyo LiDAR | 2D scan for localization |

The Jetson and Kria run as two separate ROS2 nodes that exchange Frenet state and horizon references via the `f1tenth_communication` package.

---

## 4. Software Prerequisites

- **OS:** Ubuntu 22.04
- **ROS2:** Humble
- **Python:** 3.10+
- **Vitis / Vivado:** 2025.2 (HLS + bitstream generation)
- **CUDA:** required for the GPU AMCL on Jetson
- **CasADi + IPOPT:** required only for the time-optimal raceline mode

```bash
# ROS2 + system packages
sudo apt install ros-jazzy-desktop ros-jazzy-joy ros-jazzy-joy-teleop ros-jazzy-urg-node

# Vitis (for FPGA builds)
source /tools/Xilinx/2025.1/Vitis/settings64.sh
```

---

## 5. Quick Start

### 5.1 Simulation (no hardware required)

```bash
# 1. Set up the gym bridge once
cd f1tenth_sim
./setup.sh                                  # creates venv, installs deps, builds the bridge

# 2. Build the rest of the workspace
cd ..
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# 3. Run the simulator
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# 4. In another terminal: launch the CPU MPC against the simulator
ros2 launch mpc_riccati mpc_launch.py \
    trajectory_file:=f1tenth_planning/trajectories/my_track_map_raceline.csv

# (or the MPCC)
ros2 launch mpcc mpcc_launch.py \
    trajectory_file:=f1tenth_planning/trajectories/my_track_map_raceline.csv
```

### 5.2 Generating a Raceline

```bash
# Minimum-curvature raceline (fast, default)
python3 f1tenth_planning/scripts/optimize_trajectory.py

# Time-optimal raceline (slower, needs CasADi)
python3 f1tenth_planning/scripts/optimize_trajectory.py --opt-type mintime
```

Output: `f1tenth_planning/trajectories/<map>_raceline.csv` with columns
`s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m`.

### 5.3 Hardware Run (Jetson + Kria + Car)

**On the Jetson** — bring up the vehicle, localization, and the FPGA state publisher:

```bash
ros2 launch f1tenth_stack bringup_launch.py
ros2 launch f1tenth_localization cpp_localization.launch.py
ros2 launch state_publisher state_publisher_launch.py
```

**On the Kria** — flash the bitstream and start the FPGA receiver/driver:

```bash
# Load bitstream (one-time per boot)
sudo xmutil unloadapp
sudo xmutil loadapp k26-starter-kits      # or your custom overlay from FPGA_flash/

# Start the receiver: reads /mpc_state, runs OpenCL kernel, publishes /drive
ros2 launch state_receiver mpc_launch.py
```

The CPU-only fallback path is the same launch chain, replacing
`state_publisher`/`state_receiver` with:

```bash
ros2 launch mpc_riccati mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv
```

---

## 6. FPGA Build Flow

```bash
cd FPGA_Implementations/MPC_FPGA_Kria
source /tools/Xilinx/2025.1/Vitis/settings64.sh

# Full Vitis HLS flow (csim → synth → cosim → export)
HLS_RUN_MODE=all vitis-run --mode hls --tcl scripts/run_hls.tcl

# Or only the synthesis step (fastest iteration loop)
HLS_RUN_MODE=synth vitis-run --mode hls --tcl scripts/run_hls.tcl
```

Validated synthesis point (best low-latency profile):

- Target part: `xck26-sfvc784-2LV-c` (Kria K26)
- Estimated Fmax: **204.79 MHz**
- Top latency: **19 935 cycles** ≈ **97.3 µs**
- Arithmetic: `ap_fixed<32,16>` throughout
- Horizon: **19 steps × 40 ms = 0.76 s**

The packaged IP is exported to
`FPGA_Implementations/MPC_FPGA_Kria/mpc_fpga_hls/kria_kv260/impl/ip/` and imported into Vivado for bitstream generation. Pre-built artifacts (`MPC_FPGA.bit.bin`, `MPC_FPGA.dtbo`, `mpc_fpga_top_opencl.xclbin`) are kept in [FPGA_flash/](FPGA_flash/) for direct loading on the Kria.

See [FPGA_Implementations/MPC_FPGA_Kria/README.md](FPGA_Implementations/MPC_FPGA_Kria/README.md) for the AXI-Lite register map, the AXI-stream packing format, and resource estimates.

---

## 7. Tuning and Analysis Tools

| Path | Purpose |
|------|---------|
| `MPC/test/test_sim_drive.c` | Closed-loop CPU MPC test harness |
| `MPC/test/tune_realistic_v2.py` | Grid/random parameter tuner for MPC weights |
| `MPCC/test/test_sim_drive` | Closed-loop CPU MPCC test harness |
| `FPGA_Implementations/MPC_FPGA_Kria/testbench/` | FPGA csim/cosim drivers |
| `tools/mpc_replay/` | Replay recorded runs through CPU vs FPGA MPC and produce comparison plots |
| `f1tenth_parameters/scripts/` | Vehicle parameter identification from logged data |

The `tools/mpc_replay/helper/` folder contains the scripts used to generate the
ablation, baseline-tracking, and iteration-count plots in the thesis.

---

## 8. Citing the Thesis

```bibtex
@thesis{sdu_apex_2026,
  author  = {Aksel Munksgaard-Ottosen and Jonathan Dalsgaard and Pascal Nest{\aa}s Bj{\o}rstorp},
  title   = {SDU-APEX: FPGA-Accelerated Model Predictive Control for the F1TENTH Platform},
  school  = {University of Southern Denmark},
  type    = {BSc thesis, Robot Systems Engineering},
  year    = {2026},
  month   = jun
}
```

---

## 9. Acknowledgements

This work builds on:

- The [F1TENTH](https://f1tenth.org/) open platform and gym simulator.
- The [TUM global trajectory optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization) used by the planner.
- The AMD/Xilinx Kria toolchain (Vitis HLS, Vivado, XRT).
