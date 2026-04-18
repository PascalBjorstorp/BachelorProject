# state_transport_udp

UDP transport package for Jetson <-> Kria MPC streaming.

## Binaries

- ros2_udp_sender
  - ROS2 node (Jetson side)
  - Subscribes to odom + pose, builds streamed horizon from trajectory, and sends fixed-size UDP packets.
- kria_udp_receiver
  - Kria executable (no ROS topics)
  - Receives UDP state packets, computes Frenet errors, runs OpenCL MPC kernel compute, and sends UDP control response.
- udp_control_bridge
  - ROS2 node (Jetson side)
  - Receives control UDP packets and publishes Ackermann commands to /drive.

## Packet Contract

StatePacket mirrors the active MpcState contract used by the ROS transport path:

- Vehicle state: x/y/theta/vx/vy/omega/steering (Q16.16)
- First reference point for Frenet error: ref_x_0/ref_y_0/ref_psi_0 (Q16.16)
- Horizon arrays: ref_vx/ref_kappa/ref_left_bound/ref_right_bound

ControlPacket returns steering/speed/accel plus solver status and timing.

## Build

```bash
colcon build --packages-select state_transport_udp
source install/setup.bash
```

## Run

Jetson sender:

```bash
ros2 run state_transport_udp ros2_udp_sender
```

Jetson control bridge:

```bash
ros2 run state_transport_udp udp_control_bridge
```

Kria receiver:

```bash
UDP_STATE_PORT=49000 \
UDP_CONTROL_PORT=49001 \
MPC_XCLBIN=/lib/firmware/mpc_fpga_top_opencl.xclbin \
MPC_KERNEL_NAME=mpc_fpga_top_opencl \
MPC_DEVICE_INDEX=0 \
ros2 run state_transport_udp kria_udp_receiver
```

Optional overrides without YAML:

```bash
ros2 run state_transport_udp ros2_udp_sender --ros-args \
  -p trajectory_file:=/home/f1tenth/BachelorProject/f1tenth_planning/trajectories/my_track_raceline.csv \
  -p dest_ip:=10.23.0.120
```

## End-to-End Loop

1. Jetson ros2_udp_sender sends StatePacket to Kria:49000.
2. Kria receiver validates CRC, runs OpenCL-backed MPC solve, sends ControlPacket to Jetson:49001.
3. Jetson udp_control_bridge validates CRC and publishes /drive.

## Timing Fields

- sender_mono_ns: Jetson monotonic timestamp echoed through control response.
- ultra_process_us: Kria processing time from packet receive to control send.
- udp_control_bridge computes RTT from sender_mono_ns on Jetson.
