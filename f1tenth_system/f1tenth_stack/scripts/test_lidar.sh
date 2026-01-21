#!/bin/bash
# =================================================
# Test LiDAR Connection Script
# =================================================
# Quick test to verify LiDAR is working before full launch
#
# Usage:
#   ./test_lidar.sh [lidar_ip]

LIDAR_IP=${1:-192.168.0.10}
LIDAR_PORT=10940

echo "==================================="
echo "Hokuyo UST-10LX Connection Test"
echo "==================================="
echo "LiDAR IP: $LIDAR_IP"
echo "Port:     $LIDAR_PORT"
echo ""

# Test ping
echo "1. Testing ping..."
if ping -c 1 -W 1 "$LIDAR_IP" &> /dev/null; then
    echo "   ✓ Ping successful"
else
    echo "   ✗ Ping failed - check network configuration"
    exit 1
fi

# Test TCP port
echo "2. Testing TCP port $LIDAR_PORT..."
if timeout 2 bash -c "echo > /dev/tcp/$LIDAR_IP/$LIDAR_PORT" 2>/dev/null; then
    echo "   ✓ Port is open"
else
    echo "   ✗ Port not responding - LiDAR may not be ready"
    exit 1
fi

# Quick ROS topic test
echo "3. Checking if urg_node is available..."
if ros2 pkg list 2>/dev/null | grep -q "urg_node"; then
    echo "   ✓ urg_node package found"
else
    echo "   ✗ urg_node not installed - run: sudo apt install ros-humble-urg-node"
    exit 1
fi

echo ""
echo "==================================="
echo "All tests passed! LiDAR is ready."
echo "==================================="
echo ""
echo "To launch:"
echo "  ros2 launch f1tenth_stack bringup_launch.py"
echo ""
echo "To view scan data:"
echo "  ros2 topic echo /scan"
echo ""
echo "To visualize in RViz:"
echo "  rviz2 (add LaserScan display, topic: /scan)"
