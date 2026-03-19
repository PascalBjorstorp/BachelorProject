#!/usr/bin/env python3
"""
FPGA Closed-Loop Input Test Script
==================================
Publishes simulated MpcState messages along a raceline trajectory to validate
the full control path: state input -> FPGA compute -> /drive output.

The script exercises geometric tracking behavior (cross-track error,
heading error, and lookahead-dependent steering response), not only message I/O.

Usage:
  Terminal 1 (receiver):
    sudo bash
    export ROS_DOMAIN_ID=42
    source /home/xilinx/ros2_humble/install/setup.bash
    source /home/xilinx/ros2_ws/install/setup.bash
    ros2 run state_receiver mpc_receiver_node \
        --ros-args -p trajectory_file:=/home/xilinx/trajectories/Spielberg_raceline.csv

  Terminal 2 (test):
    sudo bash
    export ROS_DOMAIN_ID=42
    source /home/xilinx/ros2_humble/install/setup.bash
    source /home/xilinx/ros2_ws/install/setup.bash
    python3 /home/xilinx/test_fpga_inputs.py /home/xilinx/trajectories/Spielberg_raceline.csv

  Terminal 3 (monitor):
    sudo bash
    export ROS_DOMAIN_ID=42
    source /home/xilinx/ros2_humble/install/setup.bash
    ros2 topic echo /drive
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time
import sys
import csv


# Q16.16 fixed-point scale (2^16) for deterministic FPGA arithmetic.
FP_SCALE = 65536


def current_time_ms_u32() -> int:
    """Return wall-clock milliseconds wrapped to uint32 for wire compatibility."""
    return int(time.time() * 1000) & 0xFFFFFFFF

def float_to_fp(val: float) -> int:
    """Convert float to Q16.16 fixed-point (signed int32)."""
    raw = int(val * FP_SCALE)
    return max(-2147483648, min(2147483647, raw))


class Waypoint:
    """A single trajectory waypoint."""
    def __init__(self, s, x, y, psi, kappa, vx, ax):
        self.s = s          # arc length [m]
        self.x = x          # position x [m]
        self.y = y          # position y [m]
        self.psi = psi      # heading [rad]
        self.kappa = kappa  # curvature [1/m]
        self.vx = vx        # velocity [m/s]
        self.ax = ax        # acceleration [m/s2]


def load_raceline(filepath: str) -> list:
    """Load raceline from CSV used by the controller reference path.
    Format: s_m,x_m,y_m,psi_rad,kappa_radpm,vx_mps,ax_mps2
    """
    waypoints = []
    with open(filepath, 'r') as f:
        reader = csv.reader(f)
        for row in reader:
            if row[0].startswith('#') or row[0].startswith('s_m'):
                continue  # skip header/comments
            wp = Waypoint(
                s=float(row[0]),
                x=float(row[1]),
                y=float(row[2]),
                psi=float(row[3]),
                kappa=float(row[4]),
                vx=float(row[5]),
                ax=float(row[6])
            )
            waypoints.append(wp)
    return waypoints


class FpgaTestPublisher(Node):
    def __init__(self, raceline_path: str):
        super().__init__('fpga_test_publisher')

        from f1tenth_msgs.msg import MpcState
        from ackermann_msgs.msg import AckermannDriveStamped
        self.MpcState = MpcState

        # Use the same path parameterization as the control stack.
        self.raceline = load_raceline(raceline_path)
        self.get_logger().info(f'Loaded {len(self.raceline)} waypoints from {raceline_path}')

        # Best-effort QoS favors low latency over retransmission of stale commands.
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(MpcState, '/mpc_state', qos)

        # Subscribe to /drive to verify output
        self.drive_sub = self.create_subscription(
            AckermannDriveStamped, '/drive',
            self.drive_callback, 10
        )
        self.last_drive = None
        self.drive_count = 0

        self.get_logger().info('Publishing to /mpc_state, monitoring /drive')

    def drive_callback(self, msg):
        self.last_drive = msg
        self.drive_count += 1

    def publish_state(self, x, y, theta, vel, wp_idx):
        """Publish a single MpcState message."""
        msg = self.MpcState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x_fp = float_to_fp(x)
        msg.y_fp = float_to_fp(y)
        msg.theta_fp = float_to_fp(theta)
        msg.velocity_fp = float_to_fp(vel)
        msg.waypoint_index = wp_idx
        msg.timestamp_ms = current_time_ms_u32()
        self.pub.publish(msg)

    def wait_and_report(self, dt=0.1):
        """Spin briefly and report any drive output."""
        rclpy.spin_once(self, timeout_sec=dt)
        if self.last_drive:
            d = self.last_drive.drive
            self.get_logger().info(
                f'  -> /drive: steer={d.steering_angle:.4f} rad '
                f'({math.degrees(d.steering_angle):.1f} deg) '
                f'speed={d.speed:.2f} m/s'
            )
            self.last_drive = None

    def run_test_suite(self):
        """Run all test scenarios."""
        self.get_logger().info('=' * 60)
        self.get_logger().info('FPGA PURE PURSUIT TEST SUITE')
        self.get_logger().info(f'Raceline: {len(self.raceline)} waypoints')
        first = self.raceline[0]
        self.get_logger().info(f'First waypoint: ({first.x:.2f}, {first.y:.2f}), heading={first.psi:.3f}')
        self.get_logger().info('=' * 60)

        # Scenario progression: nominal tracking -> perturbations -> stress rate.
        tests = [
            ('1. On-track at waypoint 0', self.test_on_track_start),
            ('2. Drive along raceline (10 Hz)', self.test_follow_raceline_10hz),
            ('3. Offset from raceline (CTE test)', self.test_offset),
            ('4. Wrong heading (heading error test)', self.test_wrong_heading),
            ('5. Different velocities', self.test_velocities),
            ('6. Full lap at 200 Hz', self.test_full_lap_200hz),
        ]

        for name, test_fn in tests:
            self.get_logger().info(f'\n--- {name} ---')
            input(f'Press Enter to start "{name}"...')
            self.drive_count = 0
            test_fn()
            self.get_logger().info(f'--- Done ({self.drive_count} drive messages received) ---\n')
            time.sleep(0.5)

        self.get_logger().info('=' * 60)
        self.get_logger().info('ALL TESTS COMPLETE')
        self.get_logger().info('=' * 60)

    def test_on_track_start(self):
        """Send the car's position exactly on waypoint 0.
        Expect: small CTE, small steering, velocity = raceline speed.
        """
        wp = self.raceline[0]
        self.get_logger().info(f'Position: ({wp.x:.2f}, {wp.y:.2f})')
        self.get_logger().info(f'Heading:  {wp.psi:.3f} rad ({math.degrees(wp.psi):.1f} deg)')
        self.get_logger().info(f'Velocity: {wp.vx:.1f} m/s')
        self.get_logger().info('Expected: small steering, CTE near zero')
        for i in range(20):
            self.publish_state(wp.x, wp.y, wp.psi, wp.vx, 0)
            self.wait_and_report(0.1)

    def test_follow_raceline_10hz(self):
        """Simulate driving along the raceline at 10 Hz for 5 seconds.
        The waypoint index anchors local horizon context while pose stays on path.
        Expect: consistent small steering following the track curvature.
        """
        n = len(self.raceline)
        steps = 50
        stride = max(1, n // steps)
        self.get_logger().info(f'Following raceline at 10 Hz, {steps} steps, stride={stride}')

        for i in range(steps):
            idx = (i * stride) % n
            wp = self.raceline[idx]
            self.publish_state(wp.x, wp.y, wp.psi, wp.vx, idx)
            self.wait_and_report(0.1)
            if (i + 1) % 10 == 0:
                self.get_logger().info(
                    f'  Step {i+1}/{steps}: wp_idx={idx}, '
                    f'pos=({wp.x:.1f}, {wp.y:.1f}), '
                    f'heading={math.degrees(wp.psi):.0f} deg'
                )

    def test_offset(self):
        """Send positions offset 1m perpendicular from the raceline.
        Expect: CTE ≈ 1m, steering correcting toward the track.
        """
        offset_m = 1.0
        self.get_logger().info(f'Sending positions {offset_m}m offset perpendicular to raceline')
        self.get_logger().info(f'Expected CTE near {offset_m}m, steering corrects toward track')

        for i in range(20):
            idx = (i * 50) % len(self.raceline)
            wp = self.raceline[idx]
            # Offset 1m perpendicular (left of heading)
            x_off = wp.x + offset_m * math.cos(wp.psi + math.pi / 2)
            y_off = wp.y + offset_m * math.sin(wp.psi + math.pi / 2)
            self.publish_state(x_off, y_off, wp.psi, wp.vx, idx)
            self.wait_and_report(0.1)

    def test_wrong_heading(self):
        """Send positions on track but with heading 45 degrees off.
        Expect: CTE near zero, but large heading error -> large steering correction.
        """
        heading_offset = math.pi / 4  # 45 degrees
        self.get_logger().info(f'On track but heading {math.degrees(heading_offset):.0f} deg off')
        self.get_logger().info('Expected: CTE near zero, large steering correction')

        for i in range(20):
            idx = (i * 50) % len(self.raceline)
            wp = self.raceline[idx]
            self.publish_state(wp.x, wp.y, wp.psi + heading_offset, wp.vx, idx)
            self.wait_and_report(0.1)

    def test_velocities(self):
        """Test speed-dependent lookahead behavior.
        Expect: higher speed produces gentler steering (larger look-ahead).
        """
        wp = self.raceline[100]  # pick a waypoint with some curvature
        speeds = [0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        self.get_logger().info(f'Testing speeds at waypoint 100: ({wp.x:.1f}, {wp.y:.1f})')
        self.get_logger().info('Higher speed -> longer lookahead -> gentler steering')

        for speed in speeds:
            self.get_logger().info(f'  Speed: {speed:.1f} m/s')
            for _ in range(5):
                self.publish_state(wp.x, wp.y, wp.psi, speed, 100)
                self.wait_and_report(0.1)

    def test_full_lap_200hz(self):
        """Simulate a full lap at 200 Hz, following every waypoint.
        This approximates runtime update rate and message timing pressure.
        """
        n = len(self.raceline)
        rate_hz = 200
        steps = min(n, 1000)
        self.get_logger().info(f'Full speed lap: {steps} waypoints at {rate_hz} Hz')

        t_start = time.monotonic()
        for i in range(steps):
            wp = self.raceline[i % n]
            self.publish_state(wp.x, wp.y, wp.psi, wp.vx, i % n)

            # Process callbacks
            rclpy.spin_once(self, timeout_sec=0.0005)

            # Rate limit to 200 Hz
            elapsed = time.monotonic() - t_start
            expected = (i + 1) / rate_hz
            if elapsed < expected:
                time.sleep(expected - elapsed)

        elapsed = time.monotonic() - t_start
        actual_hz = steps / elapsed
        self.get_logger().info(f'Done: {steps} msgs in {elapsed:.2f}s = {actual_hz:.0f} Hz')
        self.get_logger().info(f'Received {self.drive_count} drive responses')


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_fpga_inputs.py <raceline.csv>")
        print("Example: python3 test_fpga_inputs.py /home/xilinx/trajectories/Spielberg_raceline.csv")
        sys.exit(1)

    raceline_path = sys.argv[1]

    rclpy.init()
    node = FpgaTestPublisher(raceline_path)

    try:
        node.run_test_suite()
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
