#!/usr/bin/env bash
set -euo pipefail

# One-shot Kria helper for UDP transport + OpenCL MPC:
#  1) Run kria_udp_receiver with UDP + OpenCL runtime env
#
# Pure OpenCL/XRT flow: application loads xclbin at runtime.

ROS_SETUP="${ROS_SETUP:-/home/xilinx/ros2_humble/install/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/xilinx/ros2_ws/install/setup.bash}"

UDP_STATE_PORT="${UDP_STATE_PORT:-49000}"
UDP_CONTROL_PORT="${UDP_CONTROL_PORT:-49001}"
UDP_CONTROL_DT_S="${UDP_CONTROL_DT_S:-0.04}"
UDP_MAX_VEL_MPS="${UDP_MAX_VEL_MPS:-20.0}"

MPC_XCLBIN="${MPC_XCLBIN:-/lib/firmware/mpc_fpga_top_opencl.xclbin}"
MPC_KERNEL_NAME="${MPC_KERNEL_NAME:-mpc_fpga_top_opencl}"
MPC_DEVICE_INDEX="${MPC_DEVICE_INDEX:-0}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found: $ROS_SETUP"
  exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
  echo "ERROR: Workspace setup not found: $WS_SETUP"
  exit 1
fi

echo "[1/1] Launching kria_udp_receiver (OpenCL mode)"
set +u
source "$ROS_SETUP"
source "$WS_SETUP"
set -u

exec sudo PYTHONPATH="${PYTHONPATH:-}" \
          AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
          LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
          ROS_DISTRO="${ROS_DISTRO:-}" \
          PATH="$PATH" \
          UDP_STATE_PORT="$UDP_STATE_PORT" \
          UDP_CONTROL_PORT="$UDP_CONTROL_PORT" \
          UDP_CONTROL_DT_S="$UDP_CONTROL_DT_S" \
          UDP_MAX_VEL_MPS="$UDP_MAX_VEL_MPS" \
          MPC_XCLBIN="$MPC_XCLBIN" \
          MPC_KERNEL_NAME="$MPC_KERNEL_NAME" \
          MPC_DEVICE_INDEX="$MPC_DEVICE_INDEX" \
  ros2 run state_transport_udp kria_udp_receiver
