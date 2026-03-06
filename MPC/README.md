# MPC Hardware — F1/10th Real-Time Controller

ROS2 C package implementing Model Predictive Control for the F1TENTH car,
targeting the Jetson Xavier NX at 200 Hz.

## Architecture

The package uses the same Riccati-ADMM MPC core library as the simulation
(`MPC_experimental`), but with the ROS2 node adapted for real hardware:

- **MPC Core** (copied from `MPC_experimental`, unmodified):
  - `fp_math.c/h` — Q16.16 fixed-point arithmetic
  - `vehicle_model.c/h` — Dynamic bicycle model + Frenet linearization
  - `riccati_solver.c/h` — Riccati-ADMM constrained LQR solver
  - `mpc_riccati.c` — MPC controller using 8-state augmented Frenet model
  - `mpc.h` — Public API
  - `mpc_types.h` — Type definitions and default parameters

- **Hardware Node** (`mpc_hardware_node.c`):
  - Subscribes to `/odom` (VESC odometry via `nav_msgs/Odometry`)
  - Publishes to `/drive` (via `ackermann_msgs/AckermannDriveStamped`)
  - No simulation-specific code (collision, ground truth, visualization)
  - Real-time optimized: `SCHED_FIFO`, CPU affinity, Best Effort QoS
  - `CLOCK_MONOTONIC_RAW` for precise timing

## 8-State Model

Augmented state: `[e_y, e_psi, vx, vy, omega, δ_actual, δ̇_prev, a_prev]`

Controls: `[δ̇ (steering rate), a (acceleration)]`

The servo rate limit (±2.849 rad/s) is a direct box constraint on `u[0]`,
handled natively by the ADMM projection step.

## Building

```bash
cd /path/to/workspace
colcon build --packages-select mpc_hardware
source install/setup.bash
```

## Running

```bash
# With a trajectory file:
ros2 launch mpc_hardware mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv

# With custom topics:
ros2 launch mpc_hardware mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv \
    odom_topic:=/odom \
    drive_topic:=/drive \
    speed_gain:=0.8

# With verbose logging for debugging:
ros2 launch mpc_hardware mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv \
    verbose:=1
```

## Hardware Integration

This node is designed to work with `bringup_launch.py` from `f1tenth_stack`:

1. VESC driver publishes odometry on `/odom`
2. This node subscribes to `/odom` and publishes to `/drive`
3. `ackermann_mux` merges autonomous commands with joystick

## Tuning

MPC weights can be tuned at runtime via environment variables:

```bash
export MPC_W_LAT_ERROR=100.0    # Lateral error weight
export MPC_W_HEADING=1000.0     # Heading error weight
export MPC_W_VELOCITY=10.0      # Velocity tracking weight
export MPC_W_STEER_RATE=3.0     # Steering jerk weight
export MPC_W_STEER_EFFORT=0.35  # Steering rate effort weight
export MPC_SPEED_GAIN=0.8       # Scale trajectory velocities
```

## Real-Time Performance

For best real-time performance on Jetson:

1. Run as root (or set `CAP_SYS_NICE`) for `SCHED_FIFO`
2. Disable CPU frequency scaling: `sudo jetson_clocks`
3. Set performance governor: `sudo nvpmodel -m 0`
4. The node pins to CPU core 5 by default (big ARM core)

Typical solve time: ~200-500 µs per MPC call on Xavier NX.

## Trajectory Format

CSV with columns: `s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2[, left_bound, right_bound]`

Lines starting with `#` are treated as comments.
