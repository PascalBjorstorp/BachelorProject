# state_transport_udp

Standalone UDP transport package for Jetson <-> Ultra96 state streaming.

## Binaries

- ros2_udp_sender
  - ROS2 node (Jetson side)
  - Subscribes to odom, builds MPC horizon from trajectory, and sends fixed-size UDP packets
- ultra96_udp_receiver
  - Non-ROS process (Ultra96 side)
  - Receives UDP packets, validates CRC, runs FPGA compute, and sends control UDP response
- udp_control_bridge
  - ROS2 node (Jetson side)
  - Receives control UDP packets and publishes Ackermann commands to /drive

## Build

colcon build --packages-select state_transport_udp

## Run

Jetson:
ros2 run state_transport_udp ros2_udp_sender --ros-args --params-file <path_to_sender.yaml>

Jetson (control return path):
ros2 run state_transport_udp udp_control_bridge --ros-args --params-file <path_to_control_bridge.yaml>

Ultra96:
ros2 run state_transport_udp ultra96_udp_receiver

The Ultra96 process does not publish ROS2 topics.

## End-to-end loop

1. Jetson ros2_udp_sender subscribes to odom, computes nearest waypoint + horizon from trajectory, and sends StatePacket to Ultra96:49000.
   - The sender uses KD-tree nearest lookup plus heading-based forward bias.
   - This keeps lookup latency stable with larger trajectories while avoiding
     behind-vehicle waypoint picks during fast motion.
2. Ultra96 ultra96_udp_receiver validates CRC, loads horizon to FPGA, computes control, sends ControlPacket to Jetson:49001.
3. Jetson udp_control_bridge validates CRC and publishes /drive.

## Timing instrumentation

- UDP loop timing:
  - sender_mono_ns: Jetson monotonic timestamp set when state packet is sent
  - ultra_process_us: Ultra96 processing time from packet receive to control packet send
  - udp_control_bridge computes end-to-end RTT on Jetson using sender_mono_ns echo

- ROS2 loop timing:
  - Existing ROS2 path timing is in state_receiver/src/mpc_receiver.cpp (latency and compute logs).
