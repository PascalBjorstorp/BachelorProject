# F1Tenth Communication

ROS2 communication stack for streaming vehicle state and MPC references from Jetson to Kria.

## Architecture

```
Jetson (state_publisher)                              Kria (state_receiver)
┌──────────────────────────────────────┐             ┌──────────────────────────────────────┐
│ Sub: /ego_racecar/odom, /ekf_pose    │             │ Sub: /mpc_state (Best Effort QoS)    │
│ Optional sub: /sensors/servo...      │             │ Init: OpenCL context + kernel buffers │
│                                      │             │                                      │
│ KD-tree nearest + forward bias       │  MpcState   │ Pack state/horizon into OpenCL input │
│ Horizon extraction from trajectory   │────────────>│ Launch OpenCL MPC kernel             │
│ Q16.16 fixed-point conversion        │             │ Read steering/accel outputs          │
│ Pub: /mpc_state                      │             │ Pub: /drive                          │
└──────────────────────────────────────┘             └──────────────────────────────────────┘
```

## Packages

### f1tenth_msgs

Defines `MpcState.msg` for fixed-point transport.

Main payload groups:

- Vehicle state (`x_fp`, `y_fp`, `theta_fp`, `velocity_fp`, `vy_fp`, `omega_fp`, `steering_angle_fp`)
- First reference point for Frenet error (`ref_x_0_fp`, `ref_y_0_fp`, `ref_psi_0_fp`)
- Horizon arrays (`horizon_length`, `ref_vx_fp`, `ref_kappa_fp`, `ref_left_bound_fp`, `ref_right_bound_fp`)

All numeric payload fields are Q16.16 fixed-point (`int32`) except `horizon_length` (`uint32`).

### state_publisher (Jetson)

`state_publisher_node`:

1. Loads a trajectory CSV.
2. Caches odometry dynamics from `/ego_racecar/odom`.
3. Triggers publish on each `/ekf_pose` message.
4. Uses KD-tree nearest search plus fixed forward lookahead (compile-time define).
5. Streams only the active MPC horizon in each `MpcState` message.

### state_receiver (Kria)

`mpc_receiver_node`:

1. Subscribes to `/mpc_state` with Best Effort QoS.
2. Validates horizon payload lengths.
3. Transfers packed data to FPGA via OpenCL buffer writes.
4. Reads MPC outputs from OpenCL kernel output buffer.
5. Publishes `ackermann_msgs/AckermannDriveStamped` on `/drive`.

### state_transport_udp

UDP bridge package for transport experiments and non-ROS links.
It now builds as a normal ROS2 package (`state_transport_udp`) and uses the
same OpenCL-backed MPC interface contract as `state_receiver`.

## Build

```bash
cd ~/BachelorProject
colcon build --packages-select f1tenth_msgs state_publisher state_receiver
source install/setup.bash
```

## Run

Start Jetson publisher:

```bash
ros2 launch state_publisher state_publisher_launch.py
```

Start Kria receiver:

```bash
ros2 launch state_receiver mpc_launch.py
```

## Key Runtime Parameters

`state_publisher` defaults (see `state_publisher/config/params.yaml`):

- `trajectory_file`: raceline CSV path
- `odom_topic`: `/ego_racecar/odom`
- `pose_topic`: `/ekf_pose`
- `output_topic`: `/mpc_state`

`horizon` and `forward_lookahead` are fixed by FPGA-aligned compile-time constants.

`state_receiver` launch arguments (see `state_receiver/launch/mpc_launch.py`):

- `input_topic` (default `/mpc_state`)
- `drive_topic` (default `/drive`)

OpenCL payload sizing and command-limit constants are taken directly from compile-time defines in `mpc_fpga_interface.h`.

## Q16.16 Fixed-Point Reference

```cpp
// Float -> Q16.16
int32_t fp = static_cast<int32_t>(value * 65536.0);

// Q16.16 -> Float
float value = static_cast<float>(fp) / 65536.0f;
```

Approximate range: +/-32768 with step size 1/65536.

## Latency Notes

Receiver-side latency is measured from ROS header timestamps (`msg->header.stamp`) rather than a dedicated timestamp payload field.

Practical latency depends on localization rate, network load, and FPGA compute time.
