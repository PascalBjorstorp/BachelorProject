# F1Tenth FPGA Control Implementation

This directory contains the FPGA implementation of path tracking controllers for the F1Tenth platform, targeting the **Ultra96-V2** with Zynq UltraScale+ ZU3EG.

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              FULL SYSTEM LOOP                               │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐         UDP           ┌───────────────────────────────┐  │
│  │   JETSON     │ ────────────────────► │         ULTRA96               │  │
│  │ (Simulator)  │    MpcState msg       │   ┌─────────────────────┐     │  │
│  │              │    - x, y, theta      │   │    mpc_receiver     │     │  │
│  │  f1tenth_sim │    - velocity         │   │    (ARM CPU)        │     │  │
│  │              │    - waypoint_idx     │   │                     │     │  │
│  └──────────────┘                       │   │  1. Receive state   │     │  │
│                                         │   │  2. Prepare input   │     │  │
│                                         │   │  3. Write to FPGA   │     │  │
│                                         │   │  4. Read output     │     │  │
│                                         │   │  5. Publish /drive  │     │  │
│                                         │   └─────────┬───────────┘     │  │
│                                         │             │ AXI             │  │
│                                         │             ▼                 │  │
│                                         │   ┌─────────────────────┐     │  │
│  ┌──────────────┐                       │   │   FPGA (PL)         │     │  │
│  │   JETSON     │         UDP           │   │                     │     │  │
│  │ (Simulator)  │ ◄──────────────────── │   │  Pure Pursuit       │     │  │
│  │              │  AckermannDriveStamped│   │  Controller         │     │  │
│  │  /drive sub  │    - steering_angle   │   │                     │     │  │
│  │              │    - speed            │   │  Fixed-point math   │     │  │
│  │  Car moves!  │                       │   │                     │     │  │
│  └──────────────┘                       │   └─────────────────────┘     │  │
│                                         └───────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Directory Structure

```
FPGA/
├── include/
│   ├── fpga_interface.h      # CPU<->FPGA shared data structures
│   ├── fp_math_hls.h         # Fixed-point math for HLS
│   └── pure_pursuit_fpga.h   # Pure Pursuit function declarations
├── src/
│   └── pure_pursuit_fpga.c   # Pure Pursuit HLS implementation
├── test/
│   └── test_pure_pursuit.c   # x86 testbench
├── Makefile                  # Build system
├── run_hls.tcl              # Vitis HLS synthesis script
└── README.md                # This file
```

## Quick Start

### 1. Test on x86 (Validate Algorithm)

```bash
cd FPGA
make test
```

This compiles the Pure Pursuit algorithm and runs the testbench on your development machine.

### 2. Synthesize for FPGA

Requires Vitis HLS 2022.1 or later:

```bash
source /tools/Xilinx/Vitis_HLS/2022.1/settings64.sh
make hls
```

This generates:
- RTL (Verilog/VHDL) in `pure_pursuit_fpga_hls/solution1/syn/`
- IP package in `pure_pursuit_fpga_hls/solution1/impl/ip/`

### 3. Integrate with Vivado

1. Open Vivado and create a block design for Ultra96-V2
2. Add the Zynq UltraScale+ PS
3. Import the generated IP: `pure_pursuit_fpga_hls/solution1/impl/ip`
4. Connect AXI interfaces between PS and PL
5. Generate bitstream
6. Export hardware (`.xsa` file)

### 4. Deploy to Ultra96

1. Copy bitstream and device tree overlay to Ultra96
2. Load FPGA configuration:
   ```bash
   sudo fpgautil -b pure_pursuit.bin -o pure_pursuit.dtbo
   ```
3. Run the modified mpc_receiver:
   ```bash
   ros2 run mpc_receiver mpc_receiver_fpga_node \
       --ros-args -p use_fpga:=true -p trajectory_file:=/path/to/raceline.csv
   ```

## Data Flow

### Input (CPU → FPGA)

The `FpgaInputBlock_t` structure contains:
- **Vehicle State**: Position (x, y), heading (θ), velocity
- **Control Parameters**: Lookahead distances, wheelbase, max steering
- **Waypoints**: Next N points from trajectory (lookahead horizon)

All values are in **Q16.16 fixed-point** format.

### Output (FPGA → CPU)

The `FpgaOutputBlock_t` structure contains:
- **Steering Angle**: Computed steering command [rad]
- **Velocity**: Target velocity [m/s]
- **Debug Info**: Cross-track error, heading error, lookahead distance
- **Status**: Error codes if any

## Algorithm: Pure Pursuit

The FPGA implements the Pure Pursuit path tracking algorithm:

1. **Compute Lookahead Distance**
   ```
   L = L_min + K × |velocity|
   ```

2. **Find Target Waypoint**
   - Search waypoints for first point at distance ≥ L ahead of vehicle

3. **Transform to Vehicle Frame**
   ```
   x_v = cos(θ)×Δx + sin(θ)×Δy
   y_v = -sin(θ)×Δx + cos(θ)×Δy
   ```

4. **Compute Steering**
   ```
   κ = 2 × y_v / L²         (curvature)
   δ = atan(κ × wheelbase)  (steering angle)
   ```

## Fixed-Point Arithmetic

All computations use **Q16.16** format:
- 16 bits for integer part (signed)
- 16 bits for fractional part
- Range: -32768 to +32767.99998
- Precision: ~0.000015

Example conversions:
```c
int32_t fp_value = (int32_t)(float_value * 65536);  // float → Q16.16
float float_value = (float)fp_value / 65536.0f;     // Q16.16 → float
```

## Configuration Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_lookahead` | 0.5 m | Minimum lookahead distance |
| `max_lookahead` | 2.0 m | Maximum lookahead distance |
| `lookahead_gain` | 0.3 | Velocity-based lookahead scaling |
| `wheelbase` | 0.324 m | Axle-to-axle distance |
| `max_steering` | 0.42 rad | Maximum steering angle (~24°) |
| `max_velocity` | 6.0 m/s | Maximum commanded velocity |

## Performance

Expected FPGA performance on Ultra96-V2:
- **Latency**: < 1 μs (single computation)
- **Throughput**: > 1 MHz (pipelined)
- **Resource Usage**: ~5% LUTs, ~2% FFs

## Debugging

### Software Fallback

If FPGA initialization fails, the system automatically falls back to software Pure Pursuit running on the ARM CPU.

### Logging

Enable verbose logging:
```bash
ros2 run mpc_receiver mpc_receiver_fpga_node --ros-args --log-level debug
```

### Test Without FPGA

Force software mode:
```bash
ros2 run mpc_receiver mpc_receiver_fpga_node --ros-args -p use_fpga:=false
```

## Extending to Other Algorithms

The interface is designed to support multiple algorithms:

1. **Stanley Controller**: Set `algorithm = ALGO_STANLEY`
2. **MPC**: Set `algorithm = ALGO_MPC` (future)

To add a new algorithm:
1. Implement in `src/your_algorithm_fpga.c`
2. Add case to top function dispatch
3. Regenerate IP with HLS

## Troubleshooting

### FPGA Not Responding
- Check bitstream is loaded: `fpgautil -i`
- Verify memory address matches design: check `/sys/class/uio/`
- Check permission for `/dev/mem`

### Poor Tracking Performance
- Verify trajectory waypoints are correctly loaded
- Check vehicle parameters match actual car
- Tune lookahead parameters for track

### Build Errors (HLS)
- Ensure Vitis HLS is properly sourced
- Check for integer overflow in fixed-point calculations
- Verify all array accesses are bounded

## References

- [Pure Pursuit Algorithm](https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html)
- [F1Tenth Platform](https://f1tenth.org/)
- [Vitis HLS User Guide](https://docs.xilinx.com/r/en-US/ug1399-vitis-hls)
- [Ultra96-V2 Documentation](https://www.avnet.com/wps/portal/us/products/avnet-boards/avnet-board-families/ultra96-v2/)
