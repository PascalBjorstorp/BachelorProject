# f1tenth_communication

ROS2 bridge that streams the current vehicle state and the planner's local raceline from the Jetson host to the Kria FPGA, and returns the FPGA-computed drive command. The Kria side invokes the [`mpc_fpga_top_opencl`](../FPGA_Implementations/MPC_FPGA_Kria/) kernel through XRT/OpenCL.

## Architecture

```
Jetson (state_publisher)                         Kria (state_receiver)
┌────────────────────────────────────┐          ┌────────────────────────────────────┐
│ Sub: /ego_racecar/odom (or /odom)  │          │ Sub: /mpc_state                    │
│ Sub: /ekf_pose                     │          │ XRT init: load .xclbin,            │
│ Sub: /local_raceline (nav_msgs/Path)│         │           create CommandQueue      │
│ Sub: /sensors/servo_position_command│         │                                    │
│                                    │ MpcState │ Pack header + horizon into gmem0   │
│ Compute Frenet error vs raceline   ├─────────►│ enqueueTask(kernel)                │
│ Sample horizon along raceline      │          │ Read steering/accel from gmem1     │
│ Convert to raw Q14.18 (int32)      │          │                                    │
│ Pub: /mpc_state                    │          │ Pub: /drive (ackermann_msgs)       │
└────────────────────────────────────┘          └────────────────────────────────────┘
```

## Packages

### `f1tenth_msgs`

Defines [`MpcState.msg`](f1tenth_msgs/msg/MpcState.msg) — the fixed-point transport message. All numeric fields ending in `_fp` are raw Q14.18 (`int32`); `horizon_length` is `uint32`.

| Section       | Fields |
|---------------|--------|
| Header        | `std_msgs/Header header` |
| Frenet state  | `e_y_fp, e_psi_fp, velocity_fp, vy_fp, omega_fp, steering_angle_fp` |
| Horizon       | `horizon_length`, `ref_ey_fp[]`, `ref_epsi_fp[]`, `ref_vx_fp[]`, `ref_vy_fp[]`, `ref_omega_ref_fp[]`, `ref_kappa_fp[]`, `ref_left_bound_fp[]`, `ref_right_bound_fp[]` |

Decoded value = `raw_int32 / MPC_FPGA_QP_SCALE_F64` (= `raw / 262 144`).

### `state_publisher` (Jetson)

Executable: `state_publisher_node`. Computes the Frenet error against the published local raceline and forwards the horizon to the FPGA receiver.

Parameters (declared in `state_publisher/src/state_publisher_node.cpp`):

| Parameter                      | Default                              | Description |
|--------------------------------|--------------------------------------|-------------|
| `odom_topic`                   | `/ego_racecar/odom`                  | Source of body-frame velocities and yaw rate |
| `pose_topic`                   | `/ekf_pose`                          | Source of the map-frame pose (`PoseWithCovarianceStamped`) |
| `raceline_topic`               | `/local_raceline`                    | Local raceline path (`nav_msgs/Path`) |
| `servo_topic`                  | `/sensors/servo_position_command`    | Last commanded steering angle |
| `output_topic`                 | `/mpc_state`                         | Output `MpcState` topic |
| `drop_stale_raceline`          | `false`                              | Drop frames where the raceline lags the pose |
| `max_pose_raceline_age_ms`     | `150.0`                              | Maximum allowed pose/raceline age skew before drop |

The publish trigger is the arrival of a new pose message; velocities are taken from the most recent odometry sample, and the steering angle is taken from the most recent servo command. Horizon length matches the FPGA kernel's `MPC_HORIZON` (= 20).

### `state_receiver` (Kria)

Executable: `mpc_receiver_node`. Subscribes to `/mpc_state`, packs the header + horizon into the kernel's `gmem0` input buffer, enqueues one `mpc_fpga_top_opencl` task per message, reads `gmem1` for the steering / acceleration result, and publishes an `ackermann_msgs/AckermannDriveStamped` on `/drive`.

Launch arguments (see [`state_receiver/launch/mpc_launch.py`](state_receiver/launch/mpc_launch.py)):

| Argument        | Default                                                          |
|-----------------|------------------------------------------------------------------|
| `input_topic`   | `/mpc_state`                                                     |
| `drive_topic`   | `/drive`                                                         |
| `xclbin_path`   | `/lib/firmware/xilinx/MPC_FPGA/mpc_fpga_top_opencl.xclbin`       |
| `kernel_name`   | `mpc_fpga_top_opencl`                                            |
| `device_index`  | `0`                                                              |
| `debug_gdb`     | `false` — run the node under `gdb` for debugging                 |

The receiver also measures per-stage timing: host-side input pack, `enqueueMigrate(input)`, `enqueueTask`, `enqueueMigrate(output)`, `waitForEvents`, and output unpack, plus the OpenCL device-side `COMMAND_END - COMMAND_START` profile.

### `state_publisher_udp` / `state_receiver_udp` / `state_transport_udp`

UDP-transport variants used in the throughput / latency experiments where a pure ROS2 link was not desirable (different subnets, recorded replay, etc.). They use the same Q14.18 payload contract as the ROS2 path and call the same OpenCL kernel.

## Build

From the workspace root:

```bash
colcon build --packages-select f1tenth_msgs state_publisher state_receiver
source install/setup.bash
```

Add `state_publisher_udp state_receiver_udp state_transport_udp` to the package list when building the UDP variants.

## Run

```bash
# Jetson
ros2 launch state_publisher state_publisher_launch.py

# Kria (after deploying the firmware to /lib/firmware/xilinx/MPC_FPGA/ and
# loading it with `sudo xmutil loadapp MPC_FPGA`)
ros2 launch state_receiver mpc_launch.py
```

## Raw Q14.18 Reference

```cpp
// float -> raw QP
int32_t fp = static_cast<int32_t>(value * MPC_FPGA_QP_SCALE_F64);
// raw QP -> float
float value = static_cast<float>(fp) / MPC_FPGA_QP_SCALE_F32;
```

`MPC_FPGA_QP_SCALE_I32 = 1 << 18 = 262 144`. Representable range ≈ ±8192; resolution ≈ 3.8 · 10⁻⁶.

## Latency Notes

End-to-end latency is measured from the publisher's `MpcState` header timestamp through the receiver's drive command publish. The dominant terms are (in typical decreasing order) localization rate, XRT memory migration, kernel compute (see the FPGA README for verified cosim numbers), and ROS2 transport.
