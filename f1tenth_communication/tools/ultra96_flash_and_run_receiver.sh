#!/usr/bin/env bash
set -euo pipefail

# One-shot Ultra96 helper:
#  1) Configure eth0 address
#  2) Apply DDR reserved-memory DT overlay
#  3) Program PL from XSA
#  4) Run mpc_receiver_node with required addresses

ETH_IFACE="${ETH_IFACE:-eth0}"
ULTRA_IP_CIDR="${ULTRA_IP_CIDR:-10.23.0.148/24}"
XSA_PATH="${XSA_PATH:-/home/xilinx/MPC_FPGA/mpc_design_wrapper.xsa}"
DTBO_PATH="${DTBO_PATH:-/home/xilinx/mpc_ref_buffers.dtbo}"

ROS_SETUP="${ROS_SETUP:-/home/xilinx/ros2_humble/install/setup.bash}"
WS_SETUP="${WS_SETUP:-/home/xilinx/ros2_ws/install/setup.bash}"

FPGA_BASE="${FPGA_BASE:-0xA0000000}"
REF_VX="${REF_VX:-0x70000000}"
REF_KAPPA="${REF_KAPPA:-0x70010000}"
REF_LEFT="${REF_LEFT:-0x70020000}"
REF_RIGHT="${REF_RIGHT:-0x70030000}"
REF_CAPACITY="${REF_CAPACITY:-64}"
MAX_VEL="${MAX_VEL:-20.0}"

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

echo "[1/4] Configuring $ETH_IFACE address -> $ULTRA_IP_CIDR"
sudo ip addr flush dev "$ETH_IFACE"
sudo ip addr add "$ULTRA_IP_CIDR" dev "$ETH_IFACE"
sudo ip link set "$ETH_IFACE" up
ip -4 addr show dev "$ETH_IFACE" || true

echo "[2/4] Applying reserved-memory overlay from $DTBO_PATH"
sudo mount -t configfs none /sys/kernel/config 2>/dev/null || true
sudo rmdir /sys/kernel/config/device-tree/overlays/mpc_ref_buffers 2>/dev/null || true
sudo mkdir -p /sys/kernel/config/device-tree/overlays/mpc_ref_buffers
sudo sh -c "cat '$DTBO_PATH' > /sys/kernel/config/device-tree/overlays/mpc_ref_buffers/dtbo"
cat /sys/kernel/config/device-tree/overlays/mpc_ref_buffers/status
ls /sys/firmware/devicetree/base/reserved-memory | grep mpc_ref_buffers

echo "[3/4] Programming PL from $XSA_PATH"
sudo -E python3 - <<PY
from pynq import Overlay
Overlay("$XSA_PATH", download=True)
print("PL programmed from XSA")
PY

if [[ -f /sys/class/fpga_manager/fpga0/state ]]; then
  echo "fpga0 state: $(cat /sys/class/fpga_manager/fpga0/state)"
fi

echo "[4/4] Launching mpc_receiver_node"
source "$ROS_SETUP"
source "$WS_SETUP"
exec sudo -E ros2 run state_receiver mpc_receiver_node --ros-args \
  -p fpga_base_address:="$FPGA_BASE" \
  -p ref_vx_phys_addr:="$REF_VX" \
  -p ref_kappa_phys_addr:="$REF_KAPPA" \
  -p ref_left_phys_addr:="$REF_LEFT" \
  -p ref_right_phys_addr:="$REF_RIGHT" \
  -p ref_buffer_capacity:="$REF_CAPACITY" \
  -p max_velocity:="$MAX_VEL"
