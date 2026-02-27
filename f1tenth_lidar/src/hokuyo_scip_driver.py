#!/usr/bin/env python3
"""
Hokuyo UST-10LX SCIP 2.0 Direct Driver for ROS 2

Uses the SCIP 2.0 MD command for continuous streaming at the full 40 Hz
sensor rate, bypassing urg_node's synchronous request/reply limitation.

Protocol reference: Hokuyo SCIP 2.0 specification

Usage:
  ros2 run f1tenth_localization hokuyo_scip_driver.py
  ros2 run f1tenth_localization hokuyo_scip_driver.py --ros-args -p ip_address:=192.168.0.10
"""

import math
import socket
import struct
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan


def scip_decode(data: str) -> list[int]:
    """Decode SCIP 2.0 character-encoded data (3-character encoding)."""
    values = []
    # Remove line feeds within data block and checksum characters
    clean = ""
    for line in data.split("\n"):
        if len(line) > 0:
            # Last char of each line is checksum, strip it
            clean += line[:-1]

    for i in range(0, len(clean), 3):
        if i + 3 <= len(clean):
            # 3-character encoding: each char encodes 6 bits
            val = 0
            for j in range(3):
                val = (val << 6) + (ord(clean[i + j]) - 0x30)
            values.append(val)
    return values


class HokuyoScipDriver(Node):
    """Direct SCIP 2.0 driver for Hokuyo UST-10LX at full 40 Hz."""

    def __init__(self):
        super().__init__("hokuyo_scip_driver")

        # All parameters are loaded from config/hokuyo_ust10lx.yaml via the
        # launch file.  Defaults here are a safety-net mirror of the YAML.
        self.declare_parameter("ip_address", "192.168.0.10")
        self.declare_parameter("ip_port", 10940)
        self.declare_parameter("laser_frame_id", "ego_racecar/laser")
        self.declare_parameter("angle_min", -2.356194)  # -135 deg
        self.declare_parameter("angle_max", 2.356194)   # +135 deg
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cluster", 4)   # 4 → 270 beams at 1° resolution
        self.declare_parameter("skip", 0)      # 0 → every scan (full 40 Hz)

        self.ip_address = self.get_parameter("ip_address").value
        self.ip_port = self.get_parameter("ip_port").value
        self.frame_id = self.get_parameter("laser_frame_id").value
        self.angle_min = self.get_parameter("angle_min").value
        self.angle_max = self.get_parameter("angle_max").value
        self.range_min = self.get_parameter("range_min").value
        self.range_max = self.get_parameter("range_max").value
        scan_topic = self.get_parameter("scan_topic").value
        self.cluster = self.get_parameter("cluster").value
        self.skip = self.get_parameter("skip").value

        # UST-10LX sensor specs
        self.total_steps = 1080  # Total measurement steps
        self.step_min = 0
        self.step_max = 1080
        self.angular_resolution = math.radians(0.25)  # 0.25 degrees per step
        self.scan_time = 0.100  # 25ms = 40 Hz

        # Publisher with SensorDataQoS (best-effort for real-time sensor data)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )
        self.scan_pub = self.create_publisher(LaserScan, scan_topic, sensor_qos)

        # Connection state
        self.sock: Optional[socket.socket] = None
        self.running = False
        self.recv_thread: Optional[threading.Thread] = None

        self.get_logger().info(f"Hokuyo SCIP 2.0 Direct Driver")
        self.get_logger().info(f"  Target: {self.ip_address}:{self.ip_port}")
        self.get_logger().info(f"  Frame: {self.frame_id}")
        self.get_logger().info(f"  Topic: {scan_topic}")
        self.get_logger().info(f"  Cluster: {self.cluster}, Skip: {self.skip}")

        # Connect and start streaming
        if self.connect():
            self.start_streaming()

    def connect(self) -> bool:
        """Connect to the sensor via TCP."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.ip_address, self.ip_port))
            self.get_logger().info("Connected to sensor")

            # Switch to SCIP 2.0 mode
            self._send_command("SCIP2.0\n")
            time.sleep(0.1)
            self._read_response()

            # Query sensor info
            self._send_command("VV\n")
            time.sleep(0.1)
            response = self._read_response()
            for line in response.split("\n"):
                if line.startswith("PROD:") or line.startswith("FIRM:") or line.startswith("SERI:"):
                    self.get_logger().info(f"  {line.strip()}")

            # Query sensor parameters
            self._send_command("PP\n")
            time.sleep(0.1)
            pp_response = self._read_response()
            self._parse_sensor_params(pp_response)

            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return False

    def _parse_sensor_params(self, response: str):
        """Parse PP response for sensor parameters."""
        for line in response.split("\n"):
            line = line.strip()
            if line.startswith("AMIN:"):
                self.step_min = int(line.split(":")[1].split(";")[0])
            elif line.startswith("AMAX:"):
                self.step_max = int(line.split(":")[1].split(";")[0])
            elif line.startswith("ARES:"):
                total_steps = int(line.split(":")[1].split(";")[0])
                if total_steps > 0:
                    self.angular_resolution = (2.0 * math.pi) / total_steps
            elif line.startswith("SCAN:"):
                scan_rpm = int(line.split(":")[1].split(";")[0])
                if scan_rpm > 0:
                    self.scan_time = 60.0 / scan_rpm

        num_steps = self.step_max - self.step_min
        self.get_logger().info(
            f"  Sensor: steps {self.step_min}-{self.step_max} "
            f"({num_steps} points), scan time {self.scan_time * 1000:.1f}ms"
        )

    def _send_command(self, cmd: str):
        """Send a SCIP command."""
        if self.sock:
            self.sock.sendall(cmd.encode("ascii"))

    def _read_response(self) -> str:
        """Read a complete SCIP response (terminated by double LF)."""
        buf = b""
        self.sock.settimeout(2.0)
        try:
            while True:
                chunk = self.sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                # SCIP responses end with \n\n
                if b"\n\n" in buf:
                    break
        except socket.timeout:
            pass
        return buf.decode("ascii", errors="replace")

    def start_streaming(self):
        """Start continuous distance measurement using MD command."""
        # MD command format: MDsssseeeecc0ii\n
        #   ssss = start step (4 digits)
        #   eeee = end step (4 digits)
        #   cc   = cluster count (2 digits)
        #   0    = scan interval (0 = every scan)
        #   ii   = number of scans (00 = infinite)
        start = self.step_min
        end = self.step_max
        cluster = self.cluster
        skip = self.skip

        cmd = f"MD{start:04d}{end:04d}{cluster:02d}{skip:01d}00\n"
        self.get_logger().info(f"Starting continuous stream: {cmd.strip()}")

        self._send_command(cmd)

        # Read the echo/status response
        time.sleep(0.1)
        status = self._read_response()
        status_lines = [l for l in status.split("\n") if l.strip()]
        if len(status_lines) >= 2:
            status_code = status_lines[1][:2] if len(status_lines[1]) >= 2 else "??"
            if status_code == "00" or status_code == "99":
                self.get_logger().info("Continuous streaming started (40 Hz)")
            else:
                self.get_logger().warn(f"MD status: {status_code} — {status}")

        # Start receive thread
        self.running = True
        self.sock.settimeout(1.0)
        self.recv_thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.recv_thread.start()

    def _receive_loop(self):
        """Continuously receive and parse streaming scan data."""
        buf = b""
        while self.running and rclpy.ok():
            try:
                chunk = self.sock.recv(8192)
                if not chunk:
                    self.get_logger().error("Connection lost")
                    break
                buf += chunk

                # Process complete messages (separated by \n\n)
                while b"\n\n" in buf:
                    idx = buf.index(b"\n\n")
                    message = buf[:idx].decode("ascii", errors="replace")
                    buf = buf[idx + 2 :]
                    self._process_md_response(message)

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self.get_logger().error(f"Receive error: {e}")
                break

    def _process_md_response(self, message: str):
        """Parse an MD streaming response and publish LaserScan."""
        lines = message.split("\n")
        if len(lines) < 3:
            return

        # First line is the echo of the MD command
        header = lines[0]
        if not header.startswith("MD"):
            return

        # Second line is status + timestamp
        status_line = lines[1]
        if len(status_line) < 2:
            return
        status = status_line[:2]
        if status != "99":
            # 99 = data follows, 00 = command accepted (initial response)
            return

        # Third line is timestamp (4-byte encoded)
        if len(lines) < 4:
            return
        timestamp_line = lines[2]

        # Remaining lines are distance data (3-char encoded, checksum at end of each line)
        data_str = "\n".join(lines[3:])
        if not data_str.strip():
            return

        try:
            distances_mm = scip_decode(data_str)
        except Exception as e:
            self.get_logger().debug(f"Decode error: {e}")
            return

        if len(distances_mm) == 0:
            return

        # Build LaserScan message
        scan = LaserScan()
        now = self.get_clock().now()
        scan.header.stamp = now.to_msg()
        scan.header.frame_id = self.frame_id

        num_points = len(distances_mm)
        effective_resolution = self.angular_resolution * self.cluster

        # Calculate angle range based on actual step range
        total_angle = effective_resolution * (num_points - 1)
        center_step = (self.step_max + self.step_min) / 2.0
        half_steps = (self.step_max - self.step_min) / 2.0
        
        scan.angle_min = -half_steps * self.angular_resolution
        scan.angle_max = half_steps * self.angular_resolution
        scan.angle_increment = effective_resolution
        scan.time_increment = self.scan_time / num_points
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # Convert mm to meters, apply range filtering
        scan.ranges = []
        for d in distances_mm:
            if d <= 20:  # Invalid reading (< 20mm typically means error)
                scan.ranges.append(float("inf"))
            else:
                r = d / 1000.0  # mm to meters
                if r < self.range_min or r > self.range_max:
                    scan.ranges.append(float("inf"))
                else:
                    scan.ranges.append(r)

        self.scan_pub.publish(scan)

    def stop_streaming(self):
        """Stop continuous measurement."""
        self.running = False
        if self.sock:
            try:
                # QT command stops streaming
                self._send_command("QT\n")
                time.sleep(0.1)
            except Exception:
                pass

    def destroy_node(self):
        """Clean shutdown."""
        self.stop_streaming()
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HokuyoScipDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
