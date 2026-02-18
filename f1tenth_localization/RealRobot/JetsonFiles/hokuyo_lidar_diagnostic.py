#!/usr/bin/env python3
"""
Hokuyo UST-10LX LiDAR Diagnostic Tool

Tests ethernet connectivity, monitors scan data quality, and helps debug issues.
Can run standalone (without ROS) or as a ROS2 node.

Usage:
  # Test network connectivity only (no ROS needed)
  python3 hokuyo_lidar_diagnostic.py --test-network
  
  # Monitor scan data (requires ROS2 and urg_node running)
  ros2 run f1tenth_localization hokuyo_lidar_diagnostic.py
  
  # Monitor with custom settings
  ros2 run f1tenth_localization hokuyo_lidar_diagnostic.py --ros-args -p ip_address:=192.168.1.10
"""

import argparse
import os
import subprocess
import sys
import time
import socket
from datetime import datetime

# Default LiDAR settings
DEFAULT_IP = "192.168.0.10"
DEFAULT_PORT = 10940


def test_network_connectivity(ip_address: str = DEFAULT_IP, port: int = DEFAULT_PORT):
    """Test network connectivity to the Hokuyo LiDAR."""
    print("=" * 60)
    print("Hokuyo UST-10LX Network Diagnostic")
    print("=" * 60)
    print(f"Target: {ip_address}:{port}")
    print()
    
    # Test 1: Ping
    print("[1/4] Testing ping connectivity...")
    try:
        result = subprocess.run(
            ['ping', '-c', '3', '-W', '2', ip_address],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0:
            print(f"  ✓ Ping successful to {ip_address}")
            # Extract latency
            for line in result.stdout.split('\n'):
                if 'avg' in line or 'rtt' in line:
                    print(f"    {line.strip()}")
        else:
            print(f"  ✗ Ping FAILED to {ip_address}")
            print(f"    {result.stderr.strip()}")
            print()
            print("  Troubleshooting:")
            print(f"    1. Check Ethernet cable connection")
            print(f"    2. Configure Jetson's IP on same subnet:")
            print(f"       sudo ip addr add 192.168.0.15/24 dev eth0")
            print(f"    3. Verify LiDAR has power (LED should be on)")
            return False
    except subprocess.TimeoutExpired:
        print(f"  ✗ Ping timed out")
        return False
    except FileNotFoundError:
        print(f"  ✗ 'ping' command not found")
        return False
    print()
    
    # Test 2: TCP Socket Connection
    print("[2/4] Testing TCP connection to LiDAR port...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5)
        result = sock.connect_ex((ip_address, port))
        sock.close()
        
        if result == 0:
            print(f"  ✓ TCP connection successful to port {port}")
        else:
            print(f"  ✗ TCP connection FAILED to port {port} (error code: {result})")
            print()
            print("  Troubleshooting:")
            print("    1. Verify LiDAR is powered on and initialized")
            print("    2. Check if another application is using the LiDAR")
            print("    3. Try rebooting the LiDAR")
            return False
    except socket.timeout:
        print(f"  ✗ TCP connection timed out")
        return False
    except Exception as e:
        print(f"  ✗ TCP connection error: {e}")
        return False
    print()
    
    # Test 3: Check network interface
    print("[3/4] Checking network interfaces...")
    try:
        result = subprocess.run(['ip', 'addr'], capture_output=True, text=True)
        print("  Network interfaces with IP addresses:")
        for line in result.stdout.split('\n'):
            if 'inet ' in line and '127.0.0.1' not in line:
                print(f"    {line.strip()}")
    except Exception as e:
        print(f"  Warning: Could not list interfaces: {e}")
    print()
    
    # Test 4: Check route
    print("[4/4] Checking route to LiDAR...")
    try:
        result = subprocess.run(['ip', 'route', 'get', ip_address], capture_output=True, text=True)
        if result.returncode == 0:
            print(f"  Route: {result.stdout.strip()}")
        else:
            print(f"  Warning: No route to {ip_address}")
    except Exception as e:
        print(f"  Warning: Could not check route: {e}")
    print()
    
    print("=" * 60)
    print("Network connectivity: OK")
    print("=" * 60)
    return True


def run_ros_diagnostic():
    """Run as a ROS2 node to monitor LiDAR data."""
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import LaserScan
        import numpy as np
    except ImportError:
        print("Error: ROS2 libraries not available.")
        print("Run with --test-network for network-only diagnostics.")
        sys.exit(1)
    
    class LidarDiagnosticNode(Node):
        def __init__(self):
            super().__init__('hokuyo_lidar_diagnostic')
            
            # Parameters
            self.declare_parameter('ip_address', DEFAULT_IP)
            self.declare_parameter('scan_topic', '/scan')
            self.declare_parameter('expected_rate_hz', 40.0)
            
            self.ip_address = self.get_parameter('ip_address').value
            scan_topic = self.get_parameter('scan_topic').value
            self.expected_rate = self.get_parameter('expected_rate_hz').value
            
            # Stats
            self.scan_count = 0
            self.last_scan_time = None
            self.scan_intervals = []
            self.range_stats = {'min': float('inf'), 'max': 0, 'valid_count': 0, 'invalid_count': 0}
            self.start_time = time.time()
            
            # Subscriber — use SensorDataQoS to match urg_node's publisher
            from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
            sensor_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            self.subscription = self.create_subscription(
                LaserScan,
                scan_topic,
                self.scan_callback,
                sensor_qos
            )
            
            # Status timer (every 2 seconds)
            self.status_timer = self.create_timer(2.0, self.print_status)
            
            self.get_logger().info(f'Hokuyo LiDAR Diagnostic Node Started')
            self.get_logger().info(f'  LiDAR IP: {self.ip_address}')
            self.get_logger().info(f'  Scan topic: {scan_topic}')
            self.get_logger().info(f'  Expected rate: {self.expected_rate} Hz')
            self.get_logger().info('Waiting for scan data...')
        
        def scan_callback(self, msg: LaserScan):
            now = time.time()
            self.scan_count += 1
            
            # Calculate interval
            if self.last_scan_time is not None:
                interval = now - self.last_scan_time
                self.scan_intervals.append(interval)
                if len(self.scan_intervals) > 100:
                    self.scan_intervals.pop(0)
            self.last_scan_time = now
            
            # Analyze ranges
            ranges = np.array(msg.ranges)
            valid_mask = np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)
            valid_ranges = ranges[valid_mask]
            
            if len(valid_ranges) > 0:
                self.range_stats['min'] = min(self.range_stats['min'], float(np.min(valid_ranges)))
                self.range_stats['max'] = max(self.range_stats['max'], float(np.max(valid_ranges)))
            
            self.range_stats['valid_count'] += np.sum(valid_mask)
            self.range_stats['invalid_count'] += np.sum(~valid_mask)
        
        def print_status(self):
            if self.scan_count == 0:
                self.get_logger().warn('No scan data received yet!')
                self.get_logger().warn(f'  Check if urg_node is running')
                self.get_logger().warn(f'  Check topic: ros2 topic list | grep scan')
                return
            
            elapsed = time.time() - self.start_time
            avg_rate = self.scan_count / elapsed if elapsed > 0 else 0
            
            # Calculate instantaneous rate
            if len(self.scan_intervals) > 1:
                avg_interval = sum(self.scan_intervals) / len(self.scan_intervals)
                inst_rate = 1.0 / avg_interval if avg_interval > 0 else 0
            else:
                inst_rate = 0
            
            # Rate health check
            rate_status = "OK" if abs(inst_rate - self.expected_rate) < 5 else "LOW"
            
            total_points = self.range_stats['valid_count'] + self.range_stats['invalid_count']
            valid_pct = (self.range_stats['valid_count'] / total_points * 100) if total_points > 0 else 0
            
            self.get_logger().info('─' * 50)
            self.get_logger().info(f'Scans received: {self.scan_count} ({elapsed:.1f}s elapsed)')
            self.get_logger().info(f'Scan rate: {inst_rate:.1f} Hz (expected: {self.expected_rate} Hz) [{rate_status}]')
            self.get_logger().info(f'Range: {self.range_stats["min"]:.2f}m - {self.range_stats["max"]:.2f}m')
            self.get_logger().info(f'Valid points: {valid_pct:.1f}%')
    
    rclpy.init()
    node = LidarDiagnosticNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description='Hokuyo UST-10LX LiDAR Diagnostic Tool',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test network only (no ROS needed)
  python3 hokuyo_lidar_diagnostic.py --test-network
  
  # Test with custom IP
  python3 hokuyo_lidar_diagnostic.py --test-network --ip 192.168.1.10
  
  # Run as ROS2 node (monitor scan data)
  ros2 run f1tenth_localization hokuyo_lidar_diagnostic.py
"""
    )
    parser.add_argument(
        '--test-network', '-t',
        action='store_true',
        help='Run network connectivity test only (no ROS needed)'
    )
    parser.add_argument(
        '--ip',
        type=str,
        default=DEFAULT_IP,
        help=f'LiDAR IP address (default: {DEFAULT_IP})'
    )
    parser.add_argument(
        '--port',
        type=int,
        default=DEFAULT_PORT,
        help=f'LiDAR port (default: {DEFAULT_PORT})'
    )
    
    # Handle ROS2 arguments
    # ROS2 passes arguments like --ros-args, filter them out for our parser
    args_to_parse = []
    skip_next = False
    for i, arg in enumerate(sys.argv[1:]):
        if skip_next:
            skip_next = False
            continue
        if arg == '--ros-args':
            break  # Stop parsing, rest are ROS args
        args_to_parse.append(arg)
    
    args = parser.parse_args(args_to_parse)
    
    if args.test_network:
        success = test_network_connectivity(args.ip, args.port)
        sys.exit(0 if success else 1)
    else:
        # Run as ROS2 node
        run_ros_diagnostic()


if __name__ == '__main__':
    main()
