#!/usr/bin/env bash
set -euo pipefail

# One-shot Kria helper for OpenCL MPC receiver:
#  1) Run mpc_receiver_node with OpenCL/XRT runtime env
#
# Pure OpenCL/XRT flow: application loads xclbin at runtime.

ROS_SETUP="${ROS_SETUP:-/home/xilinx/ros2_humble/install/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/xilinx/ros2_ws/install/setup.bash}"

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

echo "[1/1] Launching mpc_receiver_node (OpenCL mode)"
set +u
source "$ROS_SETUP"
source "$WS_SETUP"
set -u

exec sudo PYTHONPATH="${PYTHONPATH:-}" \
          AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}" \
          LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
          ROS_DISTRO="${ROS_DISTRO:-}" \
          PATH="$PATH" \
          MPC_XCLBIN="$MPC_XCLBIN" \
          MPC_KERNEL_NAME="$MPC_KERNEL_NAME" \
          MPC_DEVICE_INDEX="$MPC_DEVICE_INDEX" \
  ros2 run state_receiver mpc_receiver_node
