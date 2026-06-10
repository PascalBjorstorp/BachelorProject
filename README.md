# SDU-APEX — FPGA-Accelerated MPC for the F1TENTH Platform

Bachelor's thesis project, *BSc in Robot Systems Engineering*, University of Southern Denmark (SDU), June 2026.

**Authors:** Aksel Munksgaard-Ottosen, Jonathan Dalsgaard, Pascal Nestås Bjørstorp

The full thesis sources are under [Report/](Report/) (build `main.tex` with `latexmk` / `pdflatex` + `biber`).

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

| Controller          | Where it runs       | Description |
|---------------------|---------------------|-------------|
| **MPC (Riccati-ADMM)** | CPU (C) — Jetson    | Path-tracking MPC in the Frenet frame, Riccati-ADMM QP solver |
| **MPCC**            | CPU (C) — Jetson    | Contouring MPC, jointly optimizes tracking error and progress along the raceline |
| **MPC-FPGA**        | Kria K26 PL (HLS)   | Same Riccati-ADMM core re-implemented in `ap_fixed<32,14>` (Q14.18) on the FPGA |

The Jetson handles localization, planning, and high-level coordination. The Kria runs the FPGA MPC and is fed pre-computed Frenet errors and the reference horizon over a ROS2 transport (`f1tenth_communication/`), which the receiver hands off to the kernel through XRT/OpenCL.

---

## 2. Repository Layout

```
BachelorProject/
├── Report/                         # LaTeX thesis sources
│
├── MPC/                            # CPU MPC (Riccati-ADMM), C + ROS2 (mpc_riccati)
├── MPCC/                           # CPU MPCC, C + ROS2 (mpcc_f1_10th)
├── FPGA_Implementations/
│   └── MPC_FPGA_Kria/              # Vitis HLS sources for the Kria K26 MPC kernel
├── FPGA_flash/                     # Pre-built artifacts for the Kria (.bit.bin / .dtbo / .xclbin / shell.json)
│
├── f1tenth_communication/          # Jetson↔Kria ROS2 bridge (state_publisher / state_receiver / UDP variants)
├── f1tenth_control/                # Baseline controllers (pure-pursuit, Stanley, follow-the-gap)
├── f1tenth_localization/           # GPU AMCL + EKF + odometry fusion (CUDA / C++)
├── f1tenth_lateral_planner/        # Lateral obstacle-avoidance planner
├── f1tenth_planning/               # Raceline optimizer (TUM global_racetrajectory_optimization)
├── f1tenth_sim/                    # F1TENTH gym ROS2 bridge (simulation)
├── f1tenth_system/                 # Vehicle driver stack (VESC, LiDAR, joystick mux)
└──  f1tenth_parameters/             # Vehicle parameter identification + scripts
```

Per-package READMEs: [MPC/](MPC/README.md), [MPCC/](MPCC/), [FPGA_Implementations/MPC_FPGA_Kria/](FPGA_Implementations/MPC_FPGA_Kria/README.md), [f1tenth_communication/](f1tenth_communication/README.md), [f1tenth_sim/](f1tenth_sim/README.md), [f1tenth_system/](f1tenth_system/README.md), [f1tenth_planning/](f1tenth_planning/README.md).

---

## 3. Hardware

| Component                                  | Role |
|--------------------------------------------|------|
| F1TENTH chassis (1/10 scale)               | Vehicle base |
| NVIDIA Jetson                              | Host CPU/GPU: localization, planning, ROS2 master |
| AMD/Xilinx Kria K26 (`xck26-sfvc784-2LV-c`)| FPGA accelerator for MPC |
| VESC                                       | Motor controller (FOC) + onboard IMU |
| 2D LiDAR (Hokuyo-class)                    | Scan source for localization |

The Jetson and the Kria run as separate ROS2 nodes that exchange Frenet state and horizon references via the `f1tenth_communication` package. The Kria-side node loads the kernel through XRT and invokes it as an OpenCL task.

---

## 4. Software Prerequisites

- **OS:** Ubuntu 22.04
- **ROS2:** Humble
- **Python:** 3.10+
- **Vitis / Vivado:** 2025.2 (HLS + bitstream generation)
- **XRT:** matching the Vitis 2025.2 release (deployed on the Kria for OpenCL kernel loading)
- **CUDA:** required by the GPU AMCL on the Jetson
- **CasADi + IPOPT:** required only for the time-optimal raceline mode

```bash
# ROS2 + system packages (Jetson side)
sudo apt install ros-humble-desktop ros-humble-joy ros-humble-joy-teleop ros-humble-urg-node

# Vitis (FPGA host side)
source /tools/Xilinx/2025.2/Vitis/settings64.sh
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
    trajectory_file:=f1tenth_planning/trajectories/my_track_raceline.csv

# (or the MPCC)
ros2 launch mpcc_f1_10th mpcc_launch.py \
    trajectory_file:=f1tenth_planning/trajectories/my_track_raceline.csv
```

### 5.2 Generating a Raceline

```bash
# Minimum-curvature raceline (fast, default)
python3 f1tenth_planning/scripts/optimize_trajectory.py

# Time-optimal raceline (needs CasADi)
python3 f1tenth_planning/scripts/optimize_trajectory.py --opt-type mintime
```

Output: `f1tenth_planning/trajectories/<map>_raceline.csv` with columns
`s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m`.

### 5.3 Hardware Run (Jetson + Kria + Car)

**On the Jetson** — vehicle bring-up, localization, and the FPGA state publisher:

```bash
ros2 launch f1tenth_stack bringup_launch.py
ros2 launch f1tenth_localization cpp_localization.launch.py
ros2 launch state_publisher state_publisher_launch.py
```

**On the Kria** — deploy the firmware once, then start the receiver:

```bash
# One-time deploy: copy the artifacts to the firmware directory
sudo mkdir -p /lib/firmware/xilinx/MPC_FPGA
sudo cp FPGA_flash/MPC_FPGA.bit.bin \
        FPGA_flash/MPC_FPGA.dtbo \
        FPGA_flash/shell.json \
        FPGA_flash/mpc_fpga_top_opencl.xclbin \
        /lib/firmware/xilinx/MPC_FPGA/

# Load the bitstream
sudo xmutil unloadapp
sudo xmutil loadapp MPC_FPGA

# Start the receiver: subscribes /mpc_state, launches the OpenCL kernel, publishes /drive
ros2 launch state_receiver mpc_launch.py
```

The receiver's `xclbin_path` defaults to `/lib/firmware/xilinx/MPC_FPGA/mpc_fpga_top_opencl.xclbin`; override it with the `xclbin_path` launch argument if the file lives elsewhere.

**CPU-only path on the car** — replace `state_publisher` / `state_receiver` with:

```bash
ros2 launch mpc_riccati mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv
```

---

## 6. FPGA Kernel

The Vitis HLS sources, OpenCL kernel top, and Vivado link configuration live in [FPGA_Implementations/MPC_FPGA_Kria/](FPGA_Implementations/MPC_FPGA_Kria/). See the package README for the full build flow, transport contract, and report locations.

Key facts (read directly from the Vitis 2025.2 reports of the latest build):

| Item                | Value                                                  |
|---------------------|--------------------------------------------------------|
| Target part         | `xck26-sfvc784-2LV-c` (Kria K26)                       |
| Top function        | `mpc_fpga_top_opencl` (AXI-MM `gmem0/gmem1` + AXI-Lite control) |
| Target clock        | 5.000 ns (200 MHz)                                     |
| Post-route timing   | WNS = +0.038 ns → **met at 200 MHz**                   |
| Cosim latency       | 5 007 / 6 838 / - cycles (min / avg)        |
| Wall-clock latency  | **25.0 / 34.2 / 64.5 µs** at 200 MHz                   |
| Horizon             | 20 steps × 30 ms = 0.60 s                              |
| Arithmetic          | `ap_fixed<32,14>` (Q14.18), per-domain narrower types  |
| Resources (PnR)     | 53 901 LUT (46 %), 63 915 FF (27 %), 1 086 DSP (87 %), 99 BRAM_18K (34 %) |

Pre-built artifacts (`MPC_FPGA.bit.bin`, `MPC_FPGA.dtbo`, `mpc_fpga_top_opencl.xclbin`, `shell.json`) are kept in [FPGA_flash/](FPGA_flash/) for direct loading on the Kria.

---

## 7. Tuning and Analysis Tools

| Path                                                | Purpose |
|-----------------------------------------------------|---------|
| `MPC/test/test_sim_drive.c`                         | Closed-loop CPU MPC test harness |
| `MPC/test/tune_realistic_v2.py`                     | Grid / random parameter tuner for MPC weights |
| `MPCC/test/test_sim_drive`                          | Closed-loop CPU MPCC test harness |
| `FPGA_Implementations/MPC_FPGA_Kria/testbench/`     | FPGA csim/cosim drivers and the recip-precision check |
| `tools/mpc_replay/`                                 | Replay recorded runs through CPU vs FPGA solvers and produce comparison plots |
| `f1tenth_parameters/scripts/`                       | Vehicle parameter identification from logged data |

The `tools/mpc_replay/helper/` folder contains the scripts used to generate the ablation, baseline-tracking, and iteration-count plots in the thesis.

---

## 8. Citing the Thesis

```bibtex
@thesis{sdu_apex_2026,
  author = {Aksel Munksgaard-Ottosen and Jonathan Dalsgaard and Pascal Nest{\aa}s Bj{\o}rstorp},
  title  = {SDU-APEX: FPGA-Accelerated Model Predictive Control for the F1TENTH Platform},
  school = {University of Southern Denmark},
  type   = {BSc thesis, Robot Systems Engineering},
  year   = {2026},
  month  = jun
}
```

---

## 9. Acknowledgements

This work builds on:

- The [F1TENTH](https://f1tenth.org/) open platform and gym simulator.
- The [TUM global trajectory optimizer](https://github.com/TUMFTM/global_racetrajectory_optimization) used by the planner.
- The AMD/Xilinx Kria toolchain (Vitis HLS, Vivado, XRT).
