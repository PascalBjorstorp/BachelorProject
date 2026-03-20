#!/usr/bin/env bash
set -euo pipefail

# Configure direct PC <-> Ultra96 USB-Ethernet link on a dedicated subnet.
# Must be run on PC with sudo.

IFACE="${1:-enx806d97376383}"
PC_IP_CIDR="10.23.0.1/24"
ULTRA_MAC="00:e0:4c:53:44:58"
ULTRA_IP="10.23.0.120"

if [[ ${EUID:-$(id -u)} -ne 0 ]]; then
  echo "Run as root: sudo bash $0 [iface]"
  exit 1
fi

ip addr flush dev "$IFACE" || true
ip addr add "$PC_IP_CIDR" dev "$IFACE"
ip link set "$IFACE" up
ip route del default dev "$IFACE" 2>/dev/null || true

pkill dnsmasq 2>/dev/null || true

dnsmasq --interface="$IFACE" --bind-interfaces \
  --dhcp-range=10.23.0.100,10.23.0.150,12h \
  --dhcp-host=${ULTRA_MAC},${ULTRA_IP}

echo "PC link ready on ${IFACE}: ${PC_IP_CIDR}"
echo "Expected Ultra96 IP: ${ULTRA_IP}"
