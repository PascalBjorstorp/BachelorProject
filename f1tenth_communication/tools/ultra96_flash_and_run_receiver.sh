#!/usr/bin/env bash
set -euo pipefail

# One-shot Ultra96 helper for AXI-Stream DMA-based MPC:
#  1) Apply DDR reserved-memory DT overlay (for DMA buffer)
#  2) Program PL from XSA
#  3) Run mpc_receiver_node with DMA parameters

XSA_PATH="${XSA_PATH:-/home/xilinx/MPC_FPGA/mpc_design_wrapper.xsa}"
DTBO_PATH="${DTBO_PATH:-/home/xilinx/mpc_ref_buffers.dtbo}"
DMA_BUFFER="${DMA_BUFFER:-0x70000000}"

ROS_SETUP="${ROS_SETUP:-/home/xilinx/ros2_humble/install/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/xilinx/ros2_ws/install/setup.bash}"

if [[ ! -f "$XSA_PATH" ]]; then
  echo "ERROR: XSA not found: $XSA_PATH"
  exit 1
fi

if [[ ! -f "$DTBO_PATH" ]]; then
  echo "ERROR: DTBO not found: $DTBO_PATH"
  exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found: $ROS_SETUP"
  exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
  echo "ERROR: Workspace setup not found: $WS_SETUP"
  exit 1
fi

echo "[1/3] Applying reserved-memory overlay from $DTBO_PATH"
echo "      (DMA buffer at $DMA_BUFFER needs physically contiguous memory)"
sudo mount -t configfs none /sys/kernel/config 2>/dev/null || true
sudo rmdir /sys/kernel/config/device-tree/overlays/mpc_ref_buffers 2>/dev/null || true
sudo mkdir -p /sys/kernel/config/device-tree/overlays/mpc_ref_buffers
sudo sh -c "cat '$DTBO_PATH' > /sys/kernel/config/device-tree/overlays/mpc_ref_buffers/dtbo"
cat /sys/kernel/config/device-tree/overlays/mpc_ref_buffers/status
ls /sys/firmware/devicetree/base/reserved-memory | grep mpc_ref_buffers

echo "[2/3] Programming PL from $XSA_PATH"
sudo -E python3 - <<PY
from pynq import Overlay
Overlay("$XSA_PATH", download=True)
print("PL programmed from XSA")
PY

if [[ -f /sys/class/fpga_manager/fpga0/state ]]; then
  echo "fpga0 state: $(cat /sys/class/fpga_manager/fpga0/state)"
fi

echo "[3/3] Launching mpc_receiver_node (AXI-Stream DMA mode)"
set +u  # Temporarily allow unbound variables for ROS setup scripts
source "$ROS_SETUP"
source "$WS_SETUP"
set -u  # Re-enable unbound variable check

# sudo -E doesn't preserve env reliably; pass all ROS2 env vars explicitly
exec sudo PYTHONPATH="$PYTHONPATH" \
          AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH" \
          LD_LIBRARY_PATH="$LD_LIBRARY_PATH" \
          ROS_DISTRO="$ROS_DISTRO" \
          PATH="$PATH" \
  ros2 run state_receiver mpc_receiver_node
