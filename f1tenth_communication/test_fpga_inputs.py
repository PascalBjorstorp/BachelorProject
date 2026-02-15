#!/usr/bin/env python3
"""
FPGA Pure Pursuit Test Script
=============================
Publishes simulated MpcState messages to test the FPGA pure pursuit
controller running on the Ultra96.

Usage:
  1. On Ultra96: Load bitstream & start mpc_receiver_fpga node
  2. Run this script (on Ultra96 or any networked machine with ROS2):

     ros2 run f1tenth_msgs test_fpga_inputs.py
     # or just:
     python3 test_fpga_inputs.py

  3. Monitor output:
     ros2 topic echo /drive

Scenarios:
  1. Static position test - same state repeatedly
  2. Straight line drive - moving along X axis
  3. Circular track - following a circle
  4. Step input - sudden position change
  5. Varying speed - different velocities
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
import struct

# Q16.16 conversion
def float_to_fp(val: float) -> int:
    """Convert float to Q16.16 fixed-point (signed int32)."""
    raw = int(val * 65536.0)
    # Clamp to int32 range
    raw = max(-2147483648, min(2147483647, raw))
    return raw

def fp_to_float(val: int) -> float:
    """Convert Q16.16 fixed-point to float."""
    # Handle signed int32
    if val > 0x7FFFFFFF:
        val -= 0x100000000
    return val / 65536.0


class FpgaTestPublisher(Node):
    def __init__(self):
        super().__init__('fpga_test_publisher')
        
        # Import after rclpy.init
        from f1tenth_msgs.msg import MpcState
        self.MpcState = MpcState
        
        # QoS matching mpc_receiver_fpga (Best Effort)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        
        self.pub = self.create_publisher(MpcState, '/mpc_state', qos)
        self.get_logger().info('FPGA Test Publisher started. Publishing to /mpc_state')
        self.get_logger().info('Monitor output with: ros2 topic echo /drive')
        
    def publish_state(self, x, y, theta, vel, wp_idx):
        """Publish a single MpcState message."""
        msg = self.MpcState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x_fp = float_to_fp(x)
        msg.y_fp = float_to_fp(y)
        msg.theta_fp = float_to_fp(theta)
        msg.velocity_fp = float_to_fp(vel)
        msg.waypoint_index = wp_idx
        msg.timestamp_ms = int(time.time() * 1000) & 0xFFFFFFFF
        self.pub.publish(msg)
        
    def run_test_suite(self):
        """Run all test scenarios."""
        self.get_logger().info('='*50)
        self.get_logger().info('FPGA PURE PURSUIT TEST SUITE')
        self.get_logger().info('='*50)
        
        tests = [
            ('Static Position', self.test_static),
            ('Straight Line', self.test_straight_line),
            ('Circular Track', self.test_circular),
            ('Step Input', self.test_step_input),
            ('Varying Speed', self.test_varying_speed),
            ('Stress Test (100Hz)', self.test_stress),
        ]
        
        for name, test_fn in tests:
            self.get_logger().info(f'\n--- Test: {name} ---')
            input(f'Press Enter to start "{name}" test...')
            test_fn()
            self.get_logger().info(f'--- {name} complete ---\n')
            time.sleep(0.5)
        
        self.get_logger().info('='*50)
        self.get_logger().info('ALL TESTS COMPLETE')
        self.get_logger().info('='*50)
    
    def test_static(self):
        """Send the same position 10 times at 10 Hz."""
        self.get_logger().info('Sending static position (0, 0, 0) at 10 Hz for 1 second')
        for i in range(10):
            self.publish_state(x=0.0, y=0.0, theta=0.0, vel=2.0, wp_idx=0)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        self.get_logger().info('Static test done - steering should converge to a consistent value')
    
    def test_straight_line(self):
        """Move along X axis, should produce minimal steering."""
        self.get_logger().info('Moving along X axis at 3 m/s, 20 steps')
        for i in range(20):
            x = i * 0.3  # 3 m/s * 0.1s = 0.3m per step
            self.publish_state(x=x, y=0.0, theta=0.0, vel=3.0, wp_idx=i % 100)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self.get_logger().info('Straight line done - steering should be near zero (if trajectory is straight)')
    
    def test_circular(self):
        """Follow a circular path, should produce consistent steering."""
        R = 10.0  # 10m radius circle
        N = 50    # 50 steps = ~half circle
        self.get_logger().info(f'Circular path: R={R}m, {N} steps at 10 Hz')
        for i in range(N):
            angle = (i / N) * math.pi  # half circle
            x = R * math.cos(angle)
            y = R * math.sin(angle)
            theta = angle + math.pi / 2.0  # tangent direction
            self.publish_state(x=x, y=y, theta=theta, vel=2.0, wp_idx=i % 100)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)
        self.get_logger().info(f'Circular done - steering should be ~{math.atan(0.324/R):.4f} rad')
    
    def test_step_input(self):
        """Sudden position changes to test response."""
        self.get_logger().info('Step input: alternating positions')
        positions = [
            (0.0, 0.0, 0.0),
            (5.0, 0.0, 0.0),
            (5.0, 5.0, math.pi/2),
            (0.0, 5.0, math.pi),
            (0.0, 0.0, -math.pi/2),
        ]
        for x, y, theta in positions:
            self.get_logger().info(f'  Position: ({x:.1f}, {y:.1f}, {theta:.2f})')
            for _ in range(5):  # 5 messages per position
                self.publish_state(x=x, y=y, theta=theta, vel=2.0, wp_idx=0)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.1)
        self.get_logger().info('Step input done')
    
    def test_varying_speed(self):
        """Different velocities affect lookahead distance."""
        self.get_logger().info('Varying speed: 0.5 to 6.0 m/s')
        speeds = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        for vel in speeds:
            self.get_logger().info(f'  Velocity: {vel:.1f} m/s')
            for _ in range(5):
                self.publish_state(x=0.0, y=0.0, theta=0.0, vel=vel, wp_idx=0)
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.1)
        self.get_logger().info('Varying speed done - higher speed = longer lookahead = gentler steering')
    
    def test_stress(self):
        """100 Hz for 5 seconds = 500 messages."""
        N = 500
        self.get_logger().info(f'Stress test: {N} messages at 100 Hz')
        t_start = time.monotonic()
        sent = 0
        for i in range(N):
            angle = (i / 100.0) * 2.0 * math.pi  # full circle every 100 steps
            x = 10.0 * math.cos(angle)
            y = 10.0 * math.sin(angle)
            theta = angle + math.pi / 2.0
            self.publish_state(x=x, y=y, theta=theta, vel=3.0, wp_idx=i % 100)
            sent += 1
            
            # Process callbacks
            rclpy.spin_once(self, timeout_sec=0.001)
            
            # Rate limit to ~100 Hz
            elapsed = time.monotonic() - t_start
            expected = (i + 1) / 100.0
            if elapsed < expected:
                time.sleep(expected - elapsed)
        
        elapsed = time.monotonic() - t_start
        self.get_logger().info(f'Stress test done: {sent} msgs in {elapsed:.2f}s = {sent/elapsed:.0f} Hz')


def main():
    rclpy.init()
    node = FpgaTestPublisher()
    
    try:
        node.run_test_suite()
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
