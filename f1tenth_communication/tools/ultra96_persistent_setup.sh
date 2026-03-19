#!/usr/bin/env bash
set -euo pipefail

# Persistent Ultra96 setup:
# - Static eth0 IP for direct PC link
# - Boot-time DT overlay apply (reserved DDR buffers)
# - systemd service for state_receiver mpc launch

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo bash $0"
  exit 1
fi

ULTRA_USER="xilinx"
ETH_IFACE="eth0"
ULTRA_IP_CIDR="10.23.0.120/24"

ROS_SETUP="/home/${ULTRA_USER}/ros2_humble/install/setup.bash"
WS_SETUP="/home/${ULTRA_USER}/ros2_ws/install/setup.bash"
DTBO_PATH="/home/${ULTRA_USER}/pynq/overlays/mpc_ref_buffers/mpc_ref_buffers.dtbo"

echo "[1/5] Configuring static IPv4 on ${ETH_IFACE} (${ULTRA_IP_CIDR})"
if [[ -d /etc/netplan ]] && systemctl is-enabled systemd-networkd >/dev/null 2>&1; then
  cat > /etc/netplan/99-ultra96-eth0.yaml <<EOF
network:
  version: 2
  renderer: networkd
  ethernets:
    ${ETH_IFACE}:
      dhcp4: false
      addresses:
        - ${ULTRA_IP_CIDR}
EOF
  netplan generate
  netplan apply || true
else
  echo "Using /etc/network/interfaces fallback for static IPv4"
  if ! grep -q "^auto ${ETH_IFACE}" /etc/network/interfaces 2>/dev/null; then
    cat >> /etc/network/interfaces <<EOF

auto ${ETH_IFACE}
iface ${ETH_IFACE} inet static
  address 10.23.0.120
  netmask 255.255.255.0
EOF
  fi
  ip addr flush dev "${ETH_IFACE}" || true
  ip addr add "${ULTRA_IP_CIDR}" dev "${ETH_IFACE}" || true
  ip link set "${ETH_IFACE}" up || true
fi

mkdir -p /usr/local/sbin
cat > /usr/local/sbin/apply_mpc_overlay.sh <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
DTBO_PATH="/home/xilinx/pynq/overlays/mpc_ref_buffers/mpc_ref_buffers.dtbo"
OVERLAY_DIR="/sys/kernel/config/device-tree/overlays/mpc_ref_buffers"

mount -t configfs none /sys/kernel/config 2>/dev/null || true

if [[ ! -f "${DTBO_PATH}" ]]; then
  echo "DTBO not found: ${DTBO_PATH}" >&2
  exit 1
fi

if [[ -d "${OVERLAY_DIR}" ]]; then
  rmdir "${OVERLAY_DIR}" 2>/dev/null || true
fi

mkdir -p "${OVERLAY_DIR}"
cat "${DTBO_PATH}" > "${OVERLAY_DIR}/dtbo"
EOF
chmod +x /usr/local/sbin/apply_mpc_overlay.sh

echo "[2/5] Installing mpc-overlay.service"
cat > /etc/systemd/system/mpc-overlay.service <<'EOF'
[Unit]
Description=Apply MPC reserved-memory DT overlay
After=local-fs.target
Before=network.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/apply_mpc_overlay.sh
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

echo "[3/5] Installing mpc-receiver.service"
cat > /etc/systemd/system/mpc-receiver.service <<EOF
[Unit]
Description=F1TENTH MPC Receiver (Ultra96)
After=network-online.target mpc-overlay.service
Wants=network-online.target
Requires=mpc-overlay.service

[Service]
Type=simple
User=root
Environment=HOME=/root
ExecStart=/bin/bash -lc 'source ${ROS_SETUP} && source ${WS_SETUP} && ros2 launch state_receiver mpc_launch.py'
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "[4/5] Reloading systemd and enabling services"
systemctl daemon-reload
systemctl enable mpc-overlay.service
systemctl enable mpc-receiver.service

echo "[5/5] Starting services now"
systemctl start mpc-overlay.service
systemctl start mpc-receiver.service

echo
ip -brief addr show "${ETH_IFACE}" || true
systemctl --no-pager --full status mpc-overlay.service || true
systemctl --no-pager --full status mpc-receiver.service || true

echo
echo "Done. Services enabled for boot persistence."
