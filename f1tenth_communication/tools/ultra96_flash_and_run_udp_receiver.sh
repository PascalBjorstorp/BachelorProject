#!/usr/bin/env bash
set -euo pipefail

# One-shot Ultra96 helper for UDP transport + AXI-Stream DMA MPC:
#  1) Apply DDR reserved-memory DT overlay (for DMA buffer)
#  2) Program PL from XSA
#  3) Run ultra96_udp_receiver with UDP + DMA runtime env

XSA_PATH="${XSA_PATH:-/home/xilinx/MPC_FPGA/mpc_design_wrapper.xsa}"
DTBO_PATH="${DTBO_PATH:-/home/xilinx/mpc_ref_buffers.dtbo}"

MPC_BASE_ADDR="${MPC_BASE_ADDR:-0xA0010000}"
DMA_BASE_ADDR="${DMA_BASE_ADDR:-0xA0000000}"
DMA_BUFFER_PHYS_ADDR="${DMA_BUFFER_PHYS_ADDR:-0x70000000}"

UDP_STATE_PORT="${UDP_STATE_PORT:-49000}"
UDP_CONTROL_PORT="${UDP_CONTROL_PORT:-49001}"
UDP_CONTROL_DT_S="${UDP_CONTROL_DT_S:-0.04}"
UDP_MAX_VEL_MPS="${UDP_MAX_VEL_MPS:-20.0}"

ROS_SETUP="${ROS_SETUP:-/home/xilinx/ros2_humble/install/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/xilinx/ros2_ws/install/setup.bash}"
SKIP_FPGA_PROGRAM="${SKIP_FPGA_PROGRAM:-0}"

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ERROR: ROS setup not found: $ROS_SETUP"
  exit 1
fi

if [[ ! -f "$WS_SETUP" ]]; then
  echo "ERROR: Workspace setup not found: $WS_SETUP"
  exit 1
fi

if [[ "$SKIP_FPGA_PROGRAM" != "1" ]]; then
  if [[ ! -f "$XSA_PATH" ]]; then
    echo "ERROR: XSA not found: $XSA_PATH"
    exit 1
  fi

  if [[ ! -f "$DTBO_PATH" ]]; then
    echo "ERROR: DTBO not found: $DTBO_PATH"
    exit 1
  fi

  echo "[1/3] Applying reserved-memory overlay from $DTBO_PATH"
  sudo mount -t configfs none /sys/kernel/config 2>/dev/null || true
  sudo rmdir /sys/kernel/config/device-tree/overlays/mpc_ref_buffers 2>/dev/null || true
  sudo mkdir -p /sys/kernel/config/device-tree/overlays/mpc_ref_buffers
  sudo sh -c "cat '$DTBO_PATH' > /sys/kernel/config/device-tree/overlays/mpc_ref_buffers/dtbo"
  cat /sys/kernel/config/device-tree/overlays/mpc_ref_buffers/status

  echo "[2/3] Programming PL from $XSA_PATH"
  sudo -E python3 - <<PY
from pynq import Overlay
Overlay("$XSA_PATH", download=True)
print("PL programmed from XSA")
PY

  if [[ -f /sys/class/fpga_manager/fpga0/state ]]; then
    echo "fpga0 state: $(cat /sys/class/fpga_manager/fpga0/state)"
  fi
else
  echo "[1/3] SKIP_FPGA_PROGRAM=1, skipping overlay/programming"
  echo "[2/3] Reusing current PL + overlay"
fi

echo "[3/3] Launching ultra96_udp_receiver (UDP + AXI-Stream DMA mode)"
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
          MPC_BASE_ADDR="$MPC_BASE_ADDR" \
          DMA_BASE_ADDR="$DMA_BASE_ADDR" \
          DMA_BUFFER_PHYS_ADDR="$DMA_BUFFER_PHYS_ADDR" \
  ros2 run state_transport_udp ultra96_udp_receiver
