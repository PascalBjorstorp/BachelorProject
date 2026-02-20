"""
Common utilities for F1/10th parameter identification tests.

Provides:
- DataRecorder: Records timestamped data from ROS2 topics to CSV
- SafetyMonitor: Monitors battery voltage, speed limits, timeouts
- TestNode: Base ROS2 node with command publishing and data recording

DATA PIPELINE NOTES (f1tenth_system VESC stack):

  Command path (NO smoothing in default bringup_launch.py):
    /drive  →  ackermann_mux  →  /ackermann_cmd  →  ackermann_to_vesc
    →  /commands/motor/speed + /commands/servo/position  →  VESC hardware
    
    The throttle_interpolator node is NOT launched by default.
    Commands go directly to the VESC without rate limiting.
    
    ackermann_to_vesc quirks (VEL_TO_ERPM mode, default):
    - Slow-start: when current_vel < 1.0 and commanded > 1.0 m/s, it
      commands (current_vel + 0.4) instead of the full speed. This limits
      initial acceleration and affects max dynamics tests from standstill.
    - Sigmoid braking: with speed_to_braking_gain=0.0 (default), the brake
      force is constant at speed_to_braking_max / 2 = 10000 whenever
      decelerating. Not proportional to speed error.
    - Servo: servo = steering_angle_to_servo_gain * angle + offset
      (no smoothing, no rate limit)

  Sensor path:
    VESC hardware (polled at 200Hz by vesc_driver)
    →  /sensors/core      (VescStateStamped: RPM, voltage, current)
    →  /sensors/imu/raw   (sensor_msgs/Imu: SI units, NO filtering)
    →  /sensors/servo_position_command  (last commanded servo value)
    
    /sensors/core + /sensors/imu/raw  →  vesc_to_odom  →  /odom
    
    IMPORTANT - Odom filtering/quirks:
    - Velocity: speed = (RPM - offset) / gain
      *** DEADZONE: speeds below 0.05 m/s are zeroed ***
    - Heading (yaw): from IMU quaternion directly (NO filtering)
    - Angular velocity (twist.angular.z):
      *** LOW-PASS EMA FILTERED with alpha=0.3 ***
      This means odom angular velocity is SMOOTHED and DELAYED.
      For accurate yaw rate measurements, use /sensors/imu/raw instead!
    - Position: Euler integration of velocity * heading (drifts over time)

  Recommendation for parameter tests:
    - Use /sensors/imu/raw for yaw rate (angular_velocity.z) — unfiltered
    - Use /odom for velocity (twist.linear.x) — unfiltered but has deadzone
    - Use /odom for position (pose.position.x/y) — integrated, drifts slowly
    - Use /sensors/core for raw RPM and battery voltage
"""

import csv
import os
import signal
import sys
import time
from datetime import datetime

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# ============================================================================
# Constants
# ============================================================================

# Default safety limits
DEFAULT_MAX_SPEED = 12.0        # m/s
DEFAULT_MAX_TEST_TIME = 60.0   # seconds
DEFAULT_MIN_BATTERY_V = 10.5   # volts (3S LiPo cutoff ~3.5V/cell)

# VESC conversion defaults (from vesc.yaml)
DEFAULT_ERPM_GAIN = 4600.0
DEFAULT_ERPM_OFFSET = 0.0
DEFAULT_ERPM_QUADRATIC = 0.0
DEFAULT_SERVO_GAIN = -0.6960
DEFAULT_SERVO_OFFSET = 0.5460
DEFAULT_SERVO_MIN = 0.106
DEFAULT_SERVO_MAX = 1.0
DEFAULT_WHEELBASE = 0.324

# Compute max steering angle from servo limits
# servo = gain * angle + offset
# angle = (servo - offset) / gain
# For gain < 0: max angle at servo_min, min angle at servo_max
# Use the minimum of both sides for a safe symmetric limit
_steer_at_servo_min = abs((DEFAULT_SERVO_MIN - DEFAULT_SERVO_OFFSET) / DEFAULT_SERVO_GAIN)
_steer_at_servo_max = abs((DEFAULT_SERVO_MAX - DEFAULT_SERVO_OFFSET) / DEFAULT_SERVO_GAIN)
DEFAULT_MAX_STEER = min(_steer_at_servo_min, _steer_at_servo_max)

DATA_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


# ============================================================================
# Data Recording
# ============================================================================

class DataRecorder:
    """Records timestamped data to CSV files."""
    
    def __init__(self, test_name: str, columns: list):
        """
        Args:
            test_name: Name of the test (used in filename)
            columns: List of column names
        """
        os.makedirs(DATA_DIR, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.test_name = f'{test_name}_{timestamp}'
        self.filename = os.path.join(DATA_DIR, f'{self.test_name}.csv')
        self.columns = ['timestamp_s'] + columns
        self.rows = []
        self.start_time = None
        
    def start(self):
        """Mark the start time."""
        self.start_time = time.monotonic()
    
    def record(self, **kwargs):
        """Record a row of data. All column values should be provided as kwargs."""
        if self.start_time is None:
            self.start()
        t = time.monotonic() - self.start_time
        row = {'timestamp_s': t}
        row.update(kwargs)
        self.rows.append(row)
    
    def save(self):
        """Save recorded data to CSV file."""
        if not self.rows:
            print(f"No data to save for {self.filename}")
            return self.filename
        
        with open(self.filename, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.columns)
            writer.writeheader()
            for row in self.rows:
                writer.writerow(row)
        
        print(f"Saved {len(self.rows)} rows to {self.filename}")
        return self.filename
    
    def get_array(self, column: str) -> np.ndarray:
        """Get a column as a numpy array."""
        return np.array([row.get(column, np.nan) for row in self.rows])


# ============================================================================
# Safety Monitor
# ============================================================================

class SafetyMonitor:
    """Monitors safety conditions during tests."""
    
    def __init__(self, max_speed: float = DEFAULT_MAX_SPEED,
                 max_time: float = DEFAULT_MAX_TEST_TIME,
                 min_battery_v: float = DEFAULT_MIN_BATTERY_V):
        self.max_speed = max_speed
        self.max_time = max_time
        self.min_battery_v = min_battery_v
        self.start_time = None
        self.current_speed = 0.0
        self.battery_voltage = 12.0
        self.abort_reason = None
    
    def start(self):
        self.start_time = time.monotonic()
    
    def update_speed(self, speed: float):
        self.current_speed = abs(speed)
    
    def update_battery(self, voltage: float):
        self.battery_voltage = voltage
    
    def check(self) -> bool:
        """Returns True if safe to continue, False if test should abort."""
        if self.start_time is None:
            self.start()
        
        elapsed = time.monotonic() - self.start_time
        
        if elapsed > self.max_time:
            self.abort_reason = f"Test timeout ({self.max_time:.1f}s)"
            return False
        
        if self.current_speed > self.max_speed * 1.2:  # 20% margin
            self.abort_reason = f"Speed limit exceeded ({self.current_speed:.2f} > {self.max_speed:.2f} m/s)"
            return False
        
        if self.battery_voltage < self.min_battery_v:
            self.abort_reason = f"Low battery ({self.battery_voltage:.2f}V < {self.min_battery_v:.2f}V)"
            return False
        
        return True


# ============================================================================
# Base Test Node
# ============================================================================

class TestNode(Node):
    """
    Base ROS2 node for parameter identification tests.
    
    Provides:
    - Publishing AckermannDriveStamped commands to /drive
    - Subscribing to /odom, /sensors/imu/raw, /sensors/core
    - Data recording to CSV
    - Safety monitoring
    - Clean shutdown on Ctrl+C
    """
    
    def __init__(self, node_name: str, test_name: str, 
                 data_columns: list,
                 max_speed: float = DEFAULT_MAX_SPEED,
                 max_time: float = DEFAULT_MAX_TEST_TIME):
        super().__init__(node_name)
        
        # Data recording
        self.recorder = DataRecorder(test_name, data_columns)
        
        # Safety
        self.safety = SafetyMonitor(max_speed=max_speed, max_time=max_time)
        
        # State
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0
        self.odom_vx = 0.0
        self.odom_vy = 0.0
        self.odom_omega = 0.0       # WARNING: this is LOW-PASS FILTERED (alpha=0.3)
        self.imu_ax = 0.0
        self.imu_ay = 0.0
        self.imu_az = 0.0
        self.imu_gx = 0.0
        self.imu_gy = 0.0
        self.imu_gz = 0.0
        self.battery_voltage = 12.0
        self.motor_rpm = 0.0
        self.odom_received = False
        self.imu_received = False
        self.test_running = False
        self.test_complete = False
        
        # Command state
        self.cmd_speed = 0.0
        self.cmd_steering = 0.0
        
        # Publishers
        # Publish to /drive (mux navigation input). The mux forwards the
        # highest-priority active topic to /ackermann_cmd.
        # No joystick needed — when /teleop is inactive the mux passes /drive through.
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, 'drive', 10)
        
        # Subscribers
        # NOTE: vesc_to_odom publishes on 'ego_racecar/odom' (hardcoded in vesc_ackermann)
        self.odom_sub = self.create_subscription(
            Odometry, 'ego_racecar/odom', self._odom_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, 'sensors/imu/raw', self._imu_callback, 10)
        
        # VESC state subscriber (for battery voltage, RPM)
        # VescStateStamped may not be available as a Python msg, use raw subscription
        # We'll try to import it, fall back to skipping if unavailable
        try:
            from vesc_msgs.msg import VescStateStamped
            self.vesc_sub = self.create_subscription(
                VescStateStamped, 'sensors/core', self._vesc_callback, 10)
            self.has_vesc_msgs = True
        except ImportError:
            self.get_logger().warn(
                "vesc_msgs not available, battery monitoring disabled")
            self.has_vesc_msgs = False
        
        # Shutdown handler
        self._original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._sigint_handler)
        
        self.get_logger().info(f"Test node '{node_name}' initialized")
        self.get_logger().info(f"Max speed: {max_speed:.1f} m/s, Max time: {max_time:.0f}s")
    
    # --- Callbacks ---
    
    def _odom_callback(self, msg: Odometry):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion (unfiltered — comes from IMU quaternion)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.odom_yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        # Velocity from ERPM conversion (unfiltered, but 0.05 m/s deadzone)
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_vy = msg.twist.twist.linear.y
        # WARNING: odom angular velocity is LOW-PASS FILTERED (EMA alpha=0.3)
        # For accurate yaw rate, use self.imu_gz from /sensors/imu/raw instead!
        self.odom_omega = msg.twist.twist.angular.z
        
        self.safety.update_speed(self.odom_vx)
        self.odom_received = True
    
    def _imu_callback(self, msg: Imu):
        # VESC IMU (MkV, mounted z-down on car).
        # IMPORTANT: Set "Imu Rotation Roll" to 180° in VESC Tool so the
        # firmware compensates for z-down mounting. With that setting, raw
        # messages are in standard vehicle frame:
        #   x = forward, y = left, z = up, gz = yaw rate (CCW positive)
        # All tests also use abs() for magnitudes as a safety net.
        self.imu_ax = msg.linear_acceleration.x
        self.imu_ay = msg.linear_acceleration.y
        self.imu_az = msg.linear_acceleration.z
        self.imu_gx = msg.angular_velocity.x
        self.imu_gy = msg.angular_velocity.y
        self.imu_gz = msg.angular_velocity.z
        self.imu_received = True
    
    def _vesc_callback(self, msg):
        self.battery_voltage = msg.state.voltage_input
        self.motor_rpm = msg.state.speed
        self.safety.update_battery(self.battery_voltage)
    
    def _sigint_handler(self, sig, frame):
        self.get_logger().warn("Ctrl+C detected, stopping car...")
        self.send_command(0.0, 0.0)
        time.sleep(0.1)
        self.send_command(0.0, 0.0)
        self.test_running = False
        self.test_complete = True
        self.recorder.save()
        # Restore original handler and re-raise
        signal.signal(signal.SIGINT, self._original_sigint)
        sys.exit(0)
    
    # --- Command Publishing ---
    
    def send_command(self, speed: float, steering_angle: float):
        """
        Publish an AckermannDriveStamped command to /drive.
        
        NOTE: ackermann_to_vesc has a slow-start behavior in VEL_TO_ERPM mode:
        when current velocity < 1.0 m/s and commanded speed > 1.0 m/s, it
        limits the ERPM command to (current_vel + 0.4) / gain. This means
        acceleration from standstill is slower than expected. If you need
        precise acceleration measurements, start from >= 1.0 m/s.
        
        Args:
            speed: Desired speed in m/s
            steering_angle: Desired steering angle in radians
        """
        # Clamp speed for safety
        speed = np.clip(speed, -self.safety.max_speed, self.safety.max_speed)
        
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steering_angle)
        self.drive_pub.publish(msg)
        
        self.cmd_speed = speed
        self.cmd_steering = steering_angle
    
    def stop_car(self):
        """Send zero command to stop the car."""
        self.send_command(0.0, 0.0)
    
    # --- Utilities ---
    
    def wait_for_odom(self, timeout: float = 5.0) -> bool:
        """Wait until odometry messages are received."""
        self.get_logger().info("Waiting for odometry...")
        start = time.monotonic()
        while not self.odom_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                self.get_logger().error("Timeout waiting for odometry!")
                return False
        self.get_logger().info("Odometry received.")
        return True
    
    def wait_for_sensors(self, timeout: float = 5.0) -> bool:
        """Wait until all sensor messages are received."""
        self.get_logger().info("Waiting for sensors...")
        start = time.monotonic()
        while not (self.odom_received and self.imu_received):
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                missing = []
                if not self.odom_received:
                    missing.append("odom")
                if not self.imu_received:
                    missing.append("IMU")
                self.get_logger().error(
                    f"Timeout waiting for sensors: {', '.join(missing)}")
                return False
        self.get_logger().info("All sensors active.")
        return True
    
    def spin_for(self, duration: float, rate_hz: float = 50.0):
        """Spin the node for a given duration while processing callbacks.
        
        Re-publishes the current command at ~rate_hz to keep the ackermann_mux
        alive (mux timeout is typically 0.2s).
        """
        dt = 1.0 / rate_hz
        start = time.monotonic()
        while time.monotonic() - start < duration:
            # Re-publish current command to prevent mux timeout
            self.send_command(self.cmd_speed, self.cmd_steering)
            rclpy.spin_once(self, timeout_sec=dt)
            if not self.safety.check():
                self.get_logger().error(
                    f"Safety abort: {self.safety.abort_reason}")
                self.stop_car()
                return False
        return True
    
    def countdown(self, seconds: int = 3):
        """Print a countdown before starting a test.
        
        Keeps spinning during the countdown so odom/IMU callbacks stay fresh.
        """
        for i in range(seconds, 0, -1):
            self.get_logger().info(f"Starting in {i}...")
            # Spin at ~50Hz during the 1-second wait
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().info("GO!")


# ============================================================================
# Analysis Utilities
# ============================================================================

def fit_circle(x: np.ndarray, y: np.ndarray):
    """
    Fit a circle to 2D points using algebraic least squares.
    
    Returns:
        (cx, cy, r): Center coordinates and radius
        residual: RMS residual of fit
    """
    # Kasa method: minimize sum of (x^2 + y^2 - 2*cx*x - 2*cy*y - (r^2 - cx^2 - cy^2))^2
    A = np.column_stack([x, y, np.ones_like(x)])
    b = x**2 + y**2
    result = np.linalg.lstsq(A, b, rcond=None)
    params = result[0]
    
    cx = params[0] / 2.0
    cy = params[1] / 2.0
    r = np.sqrt(params[2] + cx**2 + cy**2)
    
    # Compute residual
    distances = np.sqrt((x - cx)**2 + (y - cy)**2)
    residual = np.sqrt(np.mean((distances - r)**2))
    
    return cx, cy, r, residual


def steering_angle_from_radius(radius: float, wheelbase: float) -> float:
    """
    Compute steering angle from turning radius using the Ackermann kinematic
    bicycle model (consistent with compute_half_circle_diameter).

    Forward model:  beta = arctan(0.5 * tan(delta))
                    R = L / (2 * sin(beta))

    Inverse:  delta = arctan(2 * tan(arcsin(L / (2*R))))
    """
    sin_beta = wheelbase / (2.0 * radius)
    sin_beta = np.clip(sin_beta, -1.0, 1.0)  # numerical safety
    beta = np.arcsin(sin_beta)
    return np.arctan(2.0 * np.tan(beta))


def radius_from_steering_angle(angle: float, wheelbase: float) -> float:
    """
    Compute turning radius from steering angle using the Ackermann kinematic
    bicycle model.

    beta = arctan(0.5 * tan(delta))
    R = L / (2 * sin(beta))
    """
    if abs(angle) < 1e-6:
        return float('inf')
    beta = np.arctan(0.5 * np.tan(angle))
    return wheelbase / (2.0 * np.sin(beta))


def servo_to_angle(servo_value: float, 
                   gain: float = DEFAULT_SERVO_GAIN,
                   offset: float = DEFAULT_SERVO_OFFSET) -> float:
    """Convert servo position to steering angle."""
    return (servo_value - offset) / gain


def angle_to_servo(angle: float,
                   gain: float = DEFAULT_SERVO_GAIN,
                   offset: float = DEFAULT_SERVO_OFFSET) -> float:
    """Convert steering angle to servo position."""
    return gain * angle + offset


def load_csv(filepath: str) -> dict:
    """Load a CSV file into a dict of numpy arrays."""
    data = {}
    with open(filepath, 'r') as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    
    if not rows:
        return data
    
    for key in rows[0].keys():
        try:
            data[key] = np.array([float(row[key]) for row in rows])
        except (ValueError, TypeError):
            data[key] = np.array([row[key] for row in rows])
    
    return data
