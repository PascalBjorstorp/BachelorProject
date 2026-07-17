# mpc_riccati — CPU MPC for the F1TENTH Stack

Riccati-ADMM Model Predictive Controller in C. Provides the simulation and hardware ROS2 nodes for the path-tracking MPC compared against the FPGA implementation in [../FPGA_Implementations/MPC_FPGA_Kria/](../FPGA_Implementations/MPC_FPGA_Kria/).

ROS2 package name: **`mpc_riccati`**.

## CPU Prediction Model

The optimizer uses the nine-state vector

```text
[e_y, e_psi, v_x, v_y, yaw_rate,
 steering_command, steering_effective, previous_steering_rate, previous_acceleration]
```

`steering_effective` is a recorded-data-selected, zero-dead-time first-order
pole with a fixed 25 ms time constant. It represents the steering that acts on
the vehicle model; it is not claimed to be a direct servo-shaft measurement.
The estimate advances once per 5 ms CPU control call, while the horizon uses
the exact 30 ms command-ramp/pole transition and interval-average effective
steering. The VESC servo-position-command topic is treated as a command echo.

There is no lag feature flag or dead-time queue. Changing the model requires an
explicit source change and recorded-data regression test.

### Recorded-data validation

The retained 25 ms pole was checked on two independent recorded drives using
5 ms command-history reconstruction and 20 prediction stages at 30 ms. Negative
changes below mean lower RMSE than instantaneous steering:

| Prediction range | Recording | Position | Heading | Yaw rate |
|------------------|-----------|---------:|--------:|---------:|
| 30–300 ms | 530 windows | −2.56% | −3.02% | −3.87% |
| 30–300 ms | 484 windows | −1.95% | −3.54% | −4.37% |
| 30–600 ms | 530 windows | −2.06% | −0.48% | −2.39% |
| 30–600 ms | 484 windows | −1.15% | +0.78% | −1.13% |

The model is therefore a modest overall improvement, especially over the
near horizon that matters to a controller replanning every 5 ms. It is not a
uniform 10% gain: one recording has a small late-horizon heading regression.
Odometry is also not independent ground truth, so hardware A/B testing remains
the final closed-loop check.

A deterministic replay over 111,185 recorded solver calls changed mean
iterations from 2.877 to 2.857, kept p95/p99 at 7/15, and produced no solver
errors. Max-iteration results changed from 79 to 81. An interleaved pinned replay
including CSV I/O measured 33.6 µs per row versus 27.6 µs for the eight-state
baseline; this remains below 0.7% of the 5 ms control period.

## Package Layout

```
MPC/
├── include/                       # Public headers, including steering_dynamics.h
├── src/
│   ├── mpc.c                      # MPC orchestration: env config, QP setup, status reporting
│   ├── riccati_solver.c           # Riccati backward/forward + ADMM iterations
│   ├── vehicle_model.c            # Pacejka-based vehicle linearization (Frenet)
│   ├── util_math.c                # Math helpers shared by solver and node
│   └── mpc_hardware_node.c        # Hardware ROS2 node (executable: mpc_hardware_node)
├── sim/
│   └── mpc_ros2_node.c            # Simulator ROS2 node (executable: mpc_node)
├── launch/
│   ├── mpc_launch.py              # Simulation launch
│   └── mpc_hardware.launch.py     # Hardware launch
├── test/
│   ├── test_riccati_solver.c      # Sparse-solver and dense-reference regressions
│   ├── test_frenet_linearization.c # Analytic-vs-finite-difference model test
│   ├── test_steering_dynamics.c   # Exact pole/cadence/reset regressions
│   ├── test_sim_drive.c           # Closed-loop test harness (no ROS dependency)
│   └── tune_realistic_v2.py       # Weight / horizon tuner
└── trajectories/                  # Sample raceline CSVs
```

## Build

From the workspace root:

```bash
colcon build --packages-select mpc_riccati
source install/setup.bash
```

## Run

Simulation (with `f1tenth_gym_ros` bridge already running):

```bash
ros2 launch mpc_riccati mpc_launch.py \
    trajectory_file:=/path/to/raceline.csv
```

Hardware:

```bash
ros2 launch mpc_riccati mpc_hardware.launch.py \
    trajectory_file:=/path/to/raceline.csv
```

Hardware launch arguments (declared in `launch/mpc_hardware.launch.py`):

| Argument                          | Purpose |
|-----------------------------------|---------|
| `trajectory_file`                 | Raceline CSV (positional) |
| `use_local_raceline`              | Subscribe to a Path topic instead of the static CSV |
| `local_raceline_topic`            | Topic name when `use_local_raceline = true` |
| `odom_topic`, `pose_topic`        | Vehicle state inputs |
| `drive_topic`, `servo_topic`      | Outputs |
| `verbose`, `watchdog_timeout`     | Debug / safety knobs |
| `low_speed_brake_inhibit_vx`, `low_speed_min_accel` | Low-speed throttle handling |
| `recovery_epsi`, `recovery_ey`, `recovery_vref_cap` | Recovery thresholds |
| `drive_republish_period`          | Auto-republish for keep-alive |
| `log_local_raceline_snapshots`    | Periodic raceline dumps for offline analysis |
| `solver_csv_log`                  | Per-tick solver log (timestamps, status, iterations) |

## Runtime Configuration (environment variables)

Tuning values are read from environment variables in `mpc.c` and `riccati_solver.c` at startup. Defaults come from `include/mpc_types.h`.

| Group           | Variables (verified in `src/mpc.c`) |
|-----------------|-------------------------------------|
| Stage weights   | `MPC_W_LAT_ERROR`, `MPC_W_HEADING`, `MPC_W_VELOCITY`, `MPC_W_LAT_VEL`, `MPC_W_YAW_RATE` |
| Effort weights  | `MPC_W_STEER_EFFORT`, `MPC_W_ACCEL_EFFORT`, `MPC_W_EFFECTIVE_STEERING` |
| Rate weights    | `MPC_W_STEER_RATE`, `MPC_W_ACCEL_RATE` |
| Solver          | `RHO`, `RHO_U`, `TOL`, `MAX_ITER`, `MPC_SHARED_RHO`, `MPC_ADAPTIVE_RHO` |
| Timing          | `PRED_DT`, `MPC_CROSS_CALL_SCALE` |
| Wall handling   | `MPC_WALL_MARGIN` (or `WALL_MARGIN`), `MPC_WALL_BIAS_CLEAR_M`, `MPC_WALL_BIAS_MAX_M`, `MPC_WALL_REF_CLEAR_M`, `MPC_WALL_BOUND_WINDOW` |
| Misc            | `MPC_AFFINE_SCALE` |

Compile-time bounds (from `include/mpc_types.h`):

- `PREDICTION_HORIZON = 20` — fixed at compile time; raise the define to lengthen the horizon.
- `PREDICTION_DT_SECONDS = 0.03f` — nominal CPU model prediction step.
- `CONTROL_RATE_HZ = 200.0f` — control sample rate; `CONTROL_DT_SECONDS = 1 / CONTROL_RATE_HZ`.
- `STEERING_EFFECTIVE_TIME_CONSTANT_SECONDS = 0.025f` — fixed effective-steering pole.

## Tests and Tuning

Core solver/model regression tests:

```bash
cmake -S MPC -B /tmp/mpc-build -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/mpc-build
ctest --test-dir /tmp/mpc-build --output-on-failure
```

For controller changes, replay the same exported recorded-state CSV through a
frozen baseline binary and the candidate binary, then compare the aligned
outputs and convergence statistics:

```bash
gcc -O3 -I MPC/include \
    -I FPGA_Implementations/MPC_FPGA_Kria/include \
    tools/mpc_replay/helper/replay_cpu_mpc.c \
    MPC/src/mpc.c MPC/src/riccati_solver.c \
    MPC/src/vehicle_model.c MPC/src/util_math.c \
    -o /tmp/replay_cpu_mpc -lm

/tmp/replay_cpu_mpc /tmp/recorded_state.csv /tmp/replay_candidate.csv

python3 tools/mpc_replay/helper/compare_replay_outputs.py \
    /tmp/replay_baseline.csv /tmp/replay_candidate.csv
```

This replay is a deterministic regression gate; it does not replace a
closed-loop hardware A/B test for changes that intentionally alter commands.

Standalone closed-loop test (no ROS dependency):

```bash
gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
    -Wno-unused-variable -Wno-unused-but-set-variable \
    -Iinclude test/test_sim_drive.c \
        src/mpc.c src/riccati_solver.c \
        src/vehicle_model.c src/util_math.c \
    -o test_sim_drive -lm
./test_sim_drive
```

Weight tuner (uses the standalone binary):

```bash
python3 test/tune_realistic_v2.py --objective tracker
python3 test/tune_realistic_v2.py --objective fastest --jobs 8
```

Tuning output is written next to `tune_realistic_v2.py` as timestamped CSVs (e.g. `tuning_hardware_base_<timestamp>.csv`).
