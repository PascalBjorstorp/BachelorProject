#!/bin/bash
# =================================================
# Hokuyo UST-10LX Network Setup Script
# =================================================
# Run this script once after connecting the LiDAR
# to configure the Jetson's network interface.
#
# Usage:
#   sudo ./setup_lidar_network.sh [interface] [jetson_ip] [lidar_ip]
#
# Defaults:
#   interface: eth0
#   jetson_ip: 192.168.10.15
#   lidar_ip:  192.168.10.10 (after UST-10LX reconfiguration)

INTERFACE=${1:-eth0}
JETSON_IP=${2:-192.168.10.15}
LIDAR_IP=${3:-192.168.10.10}

echo "==================================="
echo "Hokuyo UST-10LX Network Setup"
echo "==================================="
echo "Interface: $INTERFACE"
echo "Jetson IP: $JETSON_IP"
echo "LiDAR IP:  $LIDAR_IP"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: Please run with sudo"
    exit 1
fi

# Check if interface exists
if ! ip link show "$INTERFACE" &> /dev/null; then
    echo "Error: Interface $INTERFACE not found"
    echo "Available interfaces:"
    ip link show | grep -E "^[0-9]+:" | awk '{print $2}' | tr -d ':'
    exit 1
fi

# Bring up interface
echo "Bringing up $INTERFACE..."
ip link set "$INTERFACE" up

# Add IP address (remove existing first to avoid conflicts)
echo "Configuring IP address..."
ip addr flush dev "$INTERFACE"
ip addr add "$JETSON_IP/24" dev "$INTERFACE"

# Verify configuration
echo ""
echo "Current configuration:"
ip addr show "$INTERFACE"

# Test connection
echo ""
echo "Testing connection to LiDAR at $LIDAR_IP..."
if ping -c 3 -W 1 "$LIDAR_IP" &> /dev/null; then
    echo "✓ LiDAR is reachable!"
    echo ""
    echo "You can now launch the driver:"
    echo "  ros2 launch f1tenth_stack bringup_launch.py"
else
    echo "✗ Cannot reach LiDAR at $LIDAR_IP"
    echo ""
    echo "Troubleshooting:"
    echo "1. Check Ethernet cable connection"
    echo "2. Verify LiDAR power (should show activity LEDs)"
    echo "3. Check LiDAR IP setting (expected: 192.168.10.10; factory default: 192.168.0.10)"
    echo "4. Try: arp -a | grep $INTERFACE"
fi
