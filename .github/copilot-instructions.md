# Copilot Instructions for F1Tenth Autonomous Racing Project

## Build Commands

```bash
# Build entire ROS2 workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Build specific package
colcon build --packages-select f1tenth_control

# Run tests for a package
colcon test --packages-select f1tenth_control
colcon test-result --verbose

# Run single C++ test (after colcon build)
./build/f1tenth_control/test_math_utils

# Run Python tests
python3 -m pytest f1tenth_parameters/tests/test_steering.py -v

# Docker build and run
cd humbleDocker && docker compose build && docker compose up
```

### FPGA HLS Synthesis

```bash
cd FPGA_Implementations/MPC_FPGA
source /tools/Xilinx/2025.1/Vitis/settings64.sh

# Full flow (csim → synth → cosim → export)
HLS_RUN_MODE=all vitis-run --mode hls --tcl scripts/run_hls.tcl

# Individual steps: csim | synth | cosim | export
HLS_RUN_MODE=synth vitis-run --mode hls --tcl scripts/run_hls.tcl

# Quick GCC testbench (no Vitis needed)
gcc -O2 -Iinclude -c src/*.c testbench/test_fpga_sim_drive.c
g++ -O2 -Iinclude -c src/mpc_fpga_top.cpp
g++ -o test_fpga_sim *.o -lm && ./test_fpga_sim
```

## Architecture Overview

This is a ROS2 Humble autonomous racing stack for F1Tenth cars with FPGA acceleration.

### Control Flow

```
LiDAR (40Hz) → Scan Splitter → [walls → GPU AMCL → pose]
                              → [obstacles → Lateral Planner]
                                              ↓
Trajectory CSV → MPC/MPCC Controller → /drive → VESC → Motors
                 (Jetson or FPGA)
```

### Key Packages

| Package | Purpose |
|---------|---------|
| `f1tenth_control` | Racing algorithms (FTG, Pure Pursuit, Stanley) - C++17 |
| `f1tenth_planning` | Trajectory optimization from SLAM maps - Python |
| `f1tenth_localization` | GPU-accelerated AMCL + EKF fusion |
| `MPC` | Model Predictive Control - C99 for Jetson |
| `MPCC` | Model Predictive Contouring Control - C99 |
| `FPGA_Implementations/MPC_FPGA` | HLS MPC solver for Ultra96-V2 |
| `f1tenth_sim` | F1Tenth Gym ROS2 bridge |
| `f1tenth_communication` | Jetson↔Ultra96 state transfer |

### MPC Architecture

- **Solver**: Riccati-ADMM (O(N·nx³) per iteration)
- **State**: 8-dimensional augmented Frenet `[e_y, e_psi, vx, vy, ω, δ, δ̇, a_prev]`
- **Horizon**: 19 steps × 40ms = 0.76s lookahead
- **Arithmetic**: Q16.16 fixed-point (FPGA), float32 (Jetson)

### FPGA Design

- **Target**: Xilinx Zynq UltraScale+ ZU3EG (Ultra96-V2)
- **Interface**: AXI-Stream input, AXI-Lite control registers
- **Latency**: 0.8-1.6ms warm-start, fits 200Hz control loop
- **Code**: Pure C99 in `src/`, C++ wrapper in `mpc_fpga_top.cpp`

## Key Conventions

### Code Organization

- Pure C/C++ algorithm libraries + separate ROS2 node wrappers
- Example: `mpc_riccati_core.c` (portable) + `mpc_ros2_node.c` (ROS2 adapter)

### Naming

- **Packages**: `snake_case` (f1tenth_control)
- **C++ classes**: `PascalCase` (FollowTheGap)
- **C functions**: `snake_case` (mpc_compute_optimal_control)
- **Fixed-point vars**: `_fp` suffix (x_fp, theta_fp)
- **HLS defines**: `MPC_HLS_BUILD`, `MPC_HLS_TARGET`

### HLS-Compatible C Code

When editing FPGA code in `FPGA_Implementations/`:

```c
// DO: Static arrays, compile-time sizes
static fix16_t trajectory[MAX_HORIZON][4];

// DON'T: malloc, printf, string.h in synthesized code
// DON'T: Division in critical path (use fp_recip)

// Required pragmas for performance:
#pragma HLS PIPELINE II=1
#pragma HLS ARRAY_PARTITION variable=K dim=1
```

### ROS2 Patterns

- Best Effort QoS for low-latency control topics
- Composable nodes with component registration
- Launch parameters in `config/*.yaml`, overridable via launch args

### Trajectory Format

CSV columns: `s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2, d_left_m, d_right_m`

## Common Tasks

### Add a New Control Algorithm

1. Create algorithm class in `f1tenth_control/include/f1tenth_control/algorithms/`
2. Implement in `f1tenth_control/src/algorithms/`
3. Create ROS2 node wrapper in `f1tenth_control/src/nodes/`
4. Add to `CMakeLists.txt` (library + executable)
5. Create launch file in `f1tenth_control/launch/`

### Modify MPC Weights

**Jetson (runtime):**
```bash
export MPC_W_LAT_ERROR=100.0
export MPC_W_HEADING=1000.0
ros2 launch mpc_riccati mpc_hardware.launch.py
```

**FPGA (compile-time):**
Edit `FPGA_Implementations/MPC_FPGA/include/mpc_fpga_types.h`, re-synthesize.

### Generate Racing Line

```bash
python3 f1tenth_planning/scripts/optimize_trajectory.py \
    --map f1tenth_sim/maps/my_track_map.yaml \
    --opt-type mincurv \
    --max-speed 8.0
```

## Hardware Targets

- **Jetson Orin/Xavier NX**: CPU control @ 200Hz
- **Ultra96-V2**: FPGA MPC acceleration
- **LiDAR**: Hokuyo UST-10LX (40Hz, 1080 beams, 270° FOV)
- **Motor**: VESC with Ackermann steering

## References

- `f1tenth_command_reference.md` - Full command reference for real robot
- Package READMEs in each directory for detailed usage
