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
    - Use /scan for LiDAR scan-matching body velocity — drift-free, independent
      of wheel state and accelerometer bias. Best source for slip ratio and
      braking deceleration measurements.
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64

# ============================================================================
# Constants
# ============================================================================

# Default safety limits
DEFAULT_MAX_SPEED = 12.0        # m/s
DEFAULT_MAX_TEST_TIME = 60.0   # seconds
DEFAULT_MIN_BATTERY_V = 10.5   # volts (3S LiPo cutoff ~3.5V/cell)

# VESC conversion defaults (from vesc.yaml)
DEFAULT_ERPM_GAIN = 4550.0
DEFAULT_ERPM_OFFSET = 0.0
DEFAULT_SERVO_GAIN = -0.7940
DEFAULT_SERVO_OFFSET = 0.5500
DEFAULT_SERVO_MIN = 0.202
DEFAULT_SERVO_MAX = 0.890
DEFAULT_WHEELBASE = 0.324
DEFAULT_LASER_X_OFFSET = 0.275  # LiDAR (ego_racecar/laser) to base_link x offset (m)

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
                 min_battery_v: float = DEFAULT_MIN_BATTERY_V,
                 max_distance: float = 0.0,
                 geofence_consecutive_samples: int = 20):
        """
        Args:
            max_distance: Maximum distance from origin before abort.
                          0 = disabled (default). Set > 0 to enable geofence.
            geofence_consecutive_samples: Number of consecutive out-of-bounds
                          samples required before geofence abort (debounce).
        """
        self.max_speed = max_speed
        self.max_time = max_time
        self.min_battery_v = min_battery_v
        self.max_distance = max_distance
        self.geofence_consecutive_samples = max(1, int(geofence_consecutive_samples))
        self.start_time = None
        self.current_speed = 0.0
        self.battery_voltage = 12.0
        self.origin_x = None
        self.origin_y = None
        self.current_x = 0.0
        self.current_y = 0.0
        self._geofence_violation_count = 0
        self.abort_reason = None
    
    def start(self):
        self.start_time = time.monotonic()
    
    def set_origin(self, x: float, y: float):
        """Set the geofence origin (call once at test start)."""
        self.origin_x = x
        self.origin_y = y
    
    def update_speed(self, speed: float):
        self.current_speed = abs(speed)
    
    def update_position(self, x: float, y: float):
        self.current_x = x
        self.current_y = y
    
    def update_battery(self, voltage: float):
        self.battery_voltage = voltage
    
    def check(self) -> bool:
        """Returns True if safe to continue, False if test should abort."""
        if self.start_time is None:
            self.start()
        
        elapsed = time.monotonic() - self.start_time
        
        if self.max_time > 0 and elapsed > self.max_time:
            self.abort_reason = f"Test timeout ({self.max_time:.1f}s)"
            return False
        
        if self.current_speed > self.max_speed * 1.2:  # 20% margin
            self.abort_reason = f"Speed limit exceeded ({self.current_speed:.2f} > {self.max_speed:.2f} m/s)"
            return False
        
        if self.battery_voltage < self.min_battery_v:
            self.abort_reason = f"Low battery ({self.battery_voltage:.2f}V < {self.min_battery_v:.2f}V)"
            return False
        
        # Geofence: check distance from origin
        if self.max_distance > 0 and self.origin_x is not None:
            dx = self.current_x - self.origin_x
            dy = self.current_y - self.origin_y
            dist = (dx * dx + dy * dy) ** 0.5
            if dist > self.max_distance:
                self._geofence_violation_count += 1
                if self._geofence_violation_count >= self.geofence_consecutive_samples:
                    self.abort_reason = (
                        f"Geofence exceeded ({dist:.2f}m > {self.max_distance:.1f}m from origin, "
                        f"{self._geofence_violation_count} consecutive samples)")
                    return False
            else:
                self._geofence_violation_count = 0
        
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
                 max_time: float = DEFAULT_MAX_TEST_TIME,
                 max_distance: float = 0.0):
        super().__init__(node_name)
        
        # Data recording
        self.recorder = DataRecorder(test_name, data_columns)
        
        # Safety
        self.safety = SafetyMonitor(
            max_speed=max_speed, max_time=max_time,
            max_distance=max_distance)
        
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
        self.imu_bias_ax = 0.0
        self.imu_bias_ay = 0.0
        self.imu_bias_az = 9.81  # default gravity reference
        self.imu_bias_gz = 0.0
        self.battery_voltage = 12.0
        self.motor_rpm = 0.0
        self.motor_current = 0.0
        self.input_current = 0.0
        self.temp_fet = 0.0
        self.temp_motor = 0.0
        self.odom_received = False
        self.imu_received = False
        self.vesc_received = False
        self.lidar_received = False
        self.lidar_vx = 0.0
        self.lidar_vy = 0.0
        self.lidar_omega = 0.0
        self.lidar_vx_raw = 0.0
        self.has_vesc_msgs = False
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
        
        # VESC state subscriber (for battery voltage, RPM, current, temperature)
        # Try both reliable and best-effort QoS to match whatever the driver uses
        try:
            from vesc_msgs.msg import VescStateStamped
            self._VescStateStamped = VescStateStamped
            qos_reliable = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10)
            qos_best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=10)
            self.vesc_sub = self.create_subscription(
                VescStateStamped, 'sensors/core',
                self._vesc_callback, qos_reliable)
            self.vesc_sub_be = self.create_subscription(
                VescStateStamped, 'sensors/core',
                self._vesc_callback, qos_best_effort)
            self.has_vesc_msgs = True
            self.get_logger().info("VESC subscription created (reliable + best-effort)")
        except (ImportError, Exception) as e:
            self.get_logger().error(
                f"VESC MSGS NOT AVAILABLE: {e}")
            self.get_logger().error(
                "Source your workspace first: source install/setup.bash")
            self.get_logger().error(
                "Battery monitoring, current, and temperature disabled!")
            self.has_vesc_msgs = False
        
        # LiDAR subscription for scan-matching velocity estimation
        # Hokuyo UST-10LX publishes LaserScan on /scan at 40 Hz.
        # Provides drift-free body velocity via ICP between consecutive scans.
        self._lidar_vel = LidarVelocityEstimator(
            laser_x_offset=DEFAULT_LASER_X_OFFSET)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self._scan_callback, 10)
        
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
        self.safety.update_position(self.odom_x, self.odom_y)
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
        self.motor_current = msg.state.current_motor
        self.input_current = msg.state.current_input
        # Use temp_fet if available, otherwise fall back to max of ntc_temp_mos
        # (the driver now populates temp_fet, but older builds may not)
        fet_temp = msg.state.temp_fet
        if fet_temp < 0.1:
            fet_temp = max(
                msg.state.ntc_temp_mos1,
                msg.state.ntc_temp_mos2,
                msg.state.ntc_temp_mos3)
        self.temp_fet = fet_temp
        # Negative temp_motor means no sensor connected
        self.temp_motor = msg.state.temp_motor if msg.state.temp_motor > -40.0 else 0.0
        self.safety.update_battery(self.battery_voltage)
        if not self.vesc_received:
            self.vesc_received = True
            self.get_logger().info(
                f"VESC data received: {self.battery_voltage:.1f}V, "
                f"FET={self.temp_fet:.1f}°C")
    
    def _scan_callback(self, msg: LaserScan):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        result = self._lidar_vel.update(
            msg.ranges, msg.angle_min, msg.angle_increment, timestamp,
            odom_vx=self.odom_vx, imu_gz=self.imu_gz)
        if result[0] is not None:
            self.lidar_vx = result[0]
            self.lidar_vy = result[1]
            self.lidar_omega = self._lidar_vel.omega
            self.lidar_vx_raw = self._lidar_vel._raw_vx
            if not self.lidar_received:
                self.lidar_received = True
                hz = 1.0 / max(self._lidar_vel._dt, 0.001)
                self.get_logger().info(
                    f"LiDAR velocity active (scan-matching, ~{hz:.0f} Hz)")
    
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

    def calibrate_imu_bias(self, duration: float = 1.5):
        """Estimate IMU biases while the car is stationary."""
        self.get_logger().info(f"Calibrating IMU bias for {duration:.1f}s (keep car still)...")
        ax_samples = []
        ay_samples = []
        az_samples = []
        gz_samples = []
        start = time.monotonic()
        while time.monotonic() - start < duration:
            self.send_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.005)
            if self.imu_received:
                ax_samples.append(self.imu_ax)
                ay_samples.append(self.imu_ay)
                az_samples.append(self.imu_az)
                gz_samples.append(self.imu_gz)

        if len(ax_samples) < 20:
            self.get_logger().warn("IMU bias calibration had few samples; keeping previous biases")
            return

        self.imu_bias_ax = float(np.mean(ax_samples))
        self.imu_bias_ay = float(np.mean(ay_samples))
        self.imu_bias_az = float(np.mean(az_samples))  # ≈ g when level
        self.imu_bias_gz = float(np.mean(gz_samples))
        self.get_logger().info(
            f"IMU bias: ax={self.imu_bias_ax:+.4f}, ay={self.imu_bias_ay:+.4f} m/s², "
            f"az={self.imu_bias_az:+.4f} m/s² (gravity ref), "
            f"gz={self.imu_bias_gz:+.4f} rad/s")
    
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
    
    def wait_for_sensors(self, timeout: float = 5.0,
                          require_vesc: bool = False,
                          require_lidar: bool = False) -> bool:
        """Wait until all sensor messages are received.
        
        Args:
            timeout: Seconds to wait before giving up.
            require_vesc: If True, also wait for VESC state data
                          (battery, current, temperature).
            require_lidar: If True, also wait for LiDAR scan-matching
                           velocity (requires two consecutive /scan msgs).
        """
        self.get_logger().info("Waiting for sensors...")
        start = time.monotonic()
        while True:
            ready = self.odom_received and self.imu_received
            if require_vesc:
                ready = ready and self.vesc_received
            if require_lidar:
                ready = ready and self.lidar_received
            if ready:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                missing = []
                if not self.odom_received:
                    missing.append("odom (ego_racecar/odom)")
                if not self.imu_received:
                    missing.append("IMU (sensors/imu/raw)")
                if require_vesc and not self.vesc_received:
                    missing.append("VESC state (sensors/core)")
                if require_lidar and not self.lidar_received:
                    missing.append("LiDAR velocity (scan on /scan)")
                self.get_logger().error(
                    f"Timeout waiting for sensors: {', '.join(missing)}")
                if require_vesc and not self.vesc_received:
                    self.get_logger().error(
                        "VESC state not received! Check that:")
                    self.get_logger().error(
                        "  1. The VESC stack is running: "
                        "ros2 launch f1tenth_stack bringup_launch.py")
                    self.get_logger().error(
                        "  2. The workspace is sourced: "
                        "source install/setup.bash")
                    self.get_logger().error(
                        "  3. Run 'ros2 topic echo /sensors/core' to verify")
                if require_lidar and not self.lidar_received:
                    self.get_logger().error(
                        "LiDAR not received! Check that:")
                    self.get_logger().error(
                        "  1. The LiDAR is running: "
                        "ros2 launch f1tenth_lidar hokuyo_lidar.launch.py")
                    self.get_logger().error(
                        "  2. Run 'ros2 topic echo /scan --once' to verify")
                return False
        status = "All sensors active"
        if self.vesc_received:
            status += f" (battery={self.battery_voltage:.1f}V, FET={self.temp_fet:.1f}°C)"
        if self.lidar_received:
            status += " (LiDAR velocity OK)"
        self.get_logger().info(status)
        return True
    
    def spin_for(self, duration: float, rate_hz: float = 200.0):
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
# IMU-Based Velocity and Radius Estimation
# ============================================================================

class ImuVelocityEstimator:
    """
    Integrates IMU longitudinal acceleration to estimate velocity independently
    of wheel (ERPM-based) odometry.

    The VESC odometry derives speed from motor ERPM (back-EMF). During
    braking the wheels can lock or slip, causing ERPM to drop faster than
    the actual vehicle speed. During aggressive cornering the driven wheels
    may spin faster than the ground speed. In both cases the ERPM-based
    velocity is wrong.

    The IMU accelerometer measures actual body acceleration regardless of
    wheel state, so integrating it gives a slip-independent velocity
    estimate for short durations. Over longer periods the integrated
    velocity drifts due to bias, so it should be periodically re-anchored
    to odom when the car is in a known-good regime (e.g. steady straight
    driving).

    Usage:
        estimator = ImuVelocityEstimator()
        # While stationary, collect imu_ax samples:
        estimator.calibrate_bias(stationary_ax_samples)
        # Start from known velocity
        estimator.reset(initial_velocity=odom_vx)
        # In the control / recording loop:
        estimator.update(imu_ax, dt)
        v = estimator.velocity
    """

    def __init__(self, bias: float = 0.0):
        self.velocity = 0.0
        self.bias = bias

    def reset(self, initial_velocity: float = 0.0, bias: float = None):
        """Reset velocity; optionally update bias."""
        self.velocity = initial_velocity
        if bias is not None:
            self.bias = bias

    def calibrate_bias(self, samples) -> float:
        """Compute accelerometer bias from stationary samples (list or array)."""
        self.bias = float(np.mean(samples)) if len(samples) > 0 else 0.0
        return self.bias

    def update(self, imu_ax: float, dt: float) -> float:
        """Integrate corrected acceleration; returns updated velocity."""
        self.velocity += (imu_ax - self.bias) * dt
        self.velocity = max(self.velocity, 0.0)  # forward-only assumption
        return self.velocity


class LidarVelocityEstimator:
    """
    Estimates body velocity from consecutive LiDAR scans using 2D
    point-to-line ICP with odometry-seeded initial guess.

    Unlike IMU integration, this provides drift-free velocity independent
    of both wheel encoders and accelerometer bias. The LiDAR measures the
    true motion of the car relative to static surroundings.

    KEY IMPROVEMENTS over basic point-to-point ICP:
      1. **Point-to-line ICP** — minimizes point-to-line distance instead of
         point-to-point. This dramatically improves accuracy along smooth
         surfaces (walls, floors) where point-to-point ICP suffers from the
         "aperture problem" (can't determine tangential displacement along
         featureless surfaces). Point-to-line constrains motion along surface
         normals, giving correct displacement even along smooth walls.
      2. **Odometry-seeded initial guess** — uses wheel odometry (vx) and
         IMU yaw rate (gz) to predict the inter-scan displacement. This
         provides a good starting point for ICP, improving convergence speed
         and avoiding local minima at higher speeds.
      3. **Local normal estimation** — computes surface normals from k
         nearest neighbors in the target scan for the point-to-line error.

    Requires: scipy (for KDTree nearest-neighbor queries in ICP).

    Usage:
        estimator = LidarVelocityEstimator(laser_x_offset=0.275)
        # In scan callback:
        vx, vy = estimator.update(ranges, angle_min, angle_increment,
                                  timestamp, odom_vx=..., imu_gz=...)
        # vx = forward velocity, vy = lateral velocity (body frame)
    """

    def __init__(self, laser_x_offset: float = 0.275,
                 max_range: float = 10.0, min_range: float = 0.05,
                 icp_max_iter: int = 30, icp_tolerance: float = 1e-6,
                 max_correspondence_dist: float = 0.5,
                 velocity_limit: float = 15.0,
                 normal_k: int = 7,
                 downsample: int = 2,
                 ema_alpha: float = 0.3):
        from scipy.spatial import KDTree
        self._KDTree = KDTree

        self.laser_x_offset = laser_x_offset
        self.max_range = max_range
        self.min_range = min_range
        self.icp_max_iter = icp_max_iter
        self.icp_tolerance = icp_tolerance
        self.max_correspondence_dist = max_correspondence_dist
        self.velocity_limit = velocity_limit
        self.normal_k = normal_k  # neighbors for normal estimation
        self.downsample = downsample  # keep every Nth point

        # EMA low-pass filter for per-scan ICP noise
        # At 40 Hz with alpha=0.4: ~60ms time constant, cuts noise by ~2x
        # while tracking real velocity changes (accel/braking) with <1 scan lag
        self.ema_alpha = ema_alpha

        self._prev_points = None
        self._prev_time = None
        self._dt = 0.025  # default 40 Hz

        self.velocity_x = 0.0   # body-frame forward velocity (m/s)
        self.velocity_y = 0.0   # body-frame lateral velocity (m/s)
        self.omega = 0.0        # yaw rate from scan matching (rad/s)
        self._raw_vx = 0.0      # unfiltered ICP velocity (for debugging)
        self._raw_vy = 0.0
        self._initialized = False

    def _scan_to_points(self, ranges, angle_min, angle_increment,
                        deskew_vx=0.0, deskew_vy=0.0, deskew_omega=0.0):
        """Convert polar laser scan to 2D Cartesian points in laser frame.

        Applies motion deskewing (undistortion) to correct for the car's
        motion during the scan acquisition period (~25 ms for Hokuyo UST-10LX).
        At 5 m/s the car moves 12.5 cm during one scan, which causes
        systematic ICP underestimation if uncorrected.

        Each beam at angular index i is acquired at time fraction (i/N) of the
        scan period. We correct the measured point by subtracting the estimated
        car displacement from the scan start time.

        Also applies downsampling to reduce point count for faster ICP.
        """
        n = len(ranges)
        angles = angle_min + np.arange(n) * angle_increment
        ranges = np.asarray(ranges, dtype=np.float64)

        valid = ((ranges > self.min_range) & (ranges < self.max_range)
                 & np.isfinite(ranges))
        angles = angles[valid]
        ranges = ranges[valid]

        # Downsample to reduce computational cost
        if self.downsample > 1 and len(ranges) > 100:
            angles = angles[::self.downsample]
            ranges = ranges[::self.downsample]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        # --- Motion deskewing ---
        # Correct for car movement during scan acquisition.
        # Beam i is acquired at time t_i relative to the scan midpoint.
        # At time t_i, the car's pose offset from midpoint is:
        #   position: v * dt_i (in laser frame)
        #   rotation: omega * dt_i
        # The measured point p_meas is in the laser frame at t_i.
        # To project to the midpoint frame:
        #   p_mid = R(omega*dt_i) @ p_meas + v * dt_i
        # Linearized for small angles:
        #   x_mid = x_meas + v_x*dt_i - omega*dt_i * y_meas
        #   y_mid = y_meas + v_y*dt_i + omega*dt_i * x_meas
        n_pts = len(x)
        if n_pts > 1 and (abs(deskew_vx) > 0.1 or abs(deskew_omega) > 0.01):
            # Fractional time offset for each beam: 0 at start, 1 at end
            frac = np.linspace(0.0, 1.0, n_pts)
            # Time relative to midpoint (so deskewing is symmetric)
            dt_frac = frac - 0.5  # ranges from -0.5 to +0.5
            # Estimated scan period (Hokuyo UST-10LX scans at ~40-50 Hz)
            scan_period = self._dt if self._dt > 0.005 else 0.025
            dt_beam = dt_frac * scan_period

            # Per-beam displacement relative to midpoint
            dx = deskew_vx * dt_beam
            dy = deskew_vy * dt_beam
            dtheta = deskew_omega * dt_beam

            # Project measured points to midpoint frame
            x_corr = x + dx - y * dtheta
            y_corr = y + dy + x * dtheta
            x, y = x_corr, y_corr

        return np.column_stack([x, y])

    def _compute_normals(self, points, tree=None):
        """
        Compute local surface normals for each point using finite differences.

        Since LiDAR scan points are naturally ordered by beam angle, the
        surface tangent at point i is approximated by (p[i+w] - p[i-w]).
        The normal is the 90-degree rotation of the tangent direction.

        This is much faster than PCA-based normals (no KDTree query, fully
        vectorized) and works well for 2D scan data.

        For points near range discontinuities (where neighboring points are
        far apart), the normal may be unreliable, but ICP's correspondence
        filtering handles this.

        Returns:
            normals: (N, 2) unit normal vectors
        """
        n = len(points)
        w = min(3, n // 4)  # window half-width for finite difference

        # Tangent vectors via central differences
        # tangent[i] = points[i+w] - points[i-w]
        tangent = np.zeros_like(points)
        tangent[w:-w] = points[2*w:] - points[:-2*w]
        # Handle edges: use one-sided differences
        tangent[:w] = points[w:2*w] - points[:w]
        tangent[-w:] = points[-w:] - points[-2*w:-w]

        # Normal = 90-degree rotation of tangent: (tx, ty) → (-ty, tx)
        normals = np.column_stack([-tangent[:, 1], tangent[:, 0]])

        # Normalize
        norms = np.sqrt(normals[:, 0]**2 + normals[:, 1]**2)
        norms = np.maximum(norms, 1e-10)  # avoid division by zero
        normals[:, 0] /= norms
        normals[:, 1] /= norms

        return normals

    def _icp_point_to_line(self, source, target, initial_R=None, initial_t=None):
        """
        Point-to-line 2D ICP: find R, t such that target ≈ R @ source + t.

        Minimizes the point-to-line distance:
            sum_i |n_i . (R @ s_i + t - t_i)|^2

        where n_i is the surface normal at target point t_i. This is much
        more accurate than point-to-point ICP for environments with smooth
        surfaces (walls), because it correctly handles sliding along walls.

        Args:
            source: (N, 2) points from previous scan
            target: (M, 2) points from current scan
            initial_R: 2x2 initial rotation guess (or None for identity)
            initial_t: 2-vector initial translation guess (or None for zero)

        Returns:
            (R, t, converged): 2x2 rotation, 2-vector translation, bool
        """
        tree = self._KDTree(target)
        normals = self._compute_normals(target)

        # Apply initial guess
        src = source.copy()
        if initial_R is not None:
            R_total = initial_R.copy()
            t_total = initial_t.copy() if initial_t is not None else np.zeros(2)
            src = (R_total @ src.T).T + t_total
        else:
            R_total = np.eye(2)
            t_total = np.zeros(2)

        for iteration in range(self.icp_max_iter):
            dists, indices = tree.query(src)

            # Filter by max correspondence distance
            mask = dists < self.max_correspondence_dist
            n_matched = int(np.sum(mask))
            if n_matched < 10:
                break

            src_m = src[mask]
            tgt_m = target[indices[mask]]
            nrm_m = normals[indices[mask]]

            # --- Point-to-line linearized solution ---
            # For small rotation dtheta and translation (dx, dy):
            #   transformed source point: [x_s - y_s*dtheta + dx,
            #                              y_s + x_s*dtheta + dy]
            #   error_i = n_i . (transformed_s_i - t_i) = 0
            #
            # This gives a 3x3 linear system: A @ [dx, dy, dtheta]^T = b

            n = len(src_m)
            A = np.zeros((n, 3))
            b_vec = np.zeros(n)

            # Residual vector: src_m - tgt_m
            diff = src_m - tgt_m

            # A[:, 0] = nx (translation in x)
            A[:, 0] = nrm_m[:, 0]
            # A[:, 1] = ny (translation in y)
            A[:, 1] = nrm_m[:, 1]
            # A[:, 2] = nx * (-y_s) + ny * (x_s)  (rotation about origin)
            A[:, 2] = nrm_m[:, 0] * (-src_m[:, 1]) + nrm_m[:, 1] * src_m[:, 0]
            # b = -n . (src - tgt) = n . (tgt - src)
            b_vec = -(nrm_m[:, 0] * diff[:, 0] + nrm_m[:, 1] * diff[:, 1])

            # Solve least-squares: A^T A x = A^T b
            ATA = A.T @ A
            ATb = A.T @ b_vec

            try:
                params = np.linalg.solve(ATA, ATb)
            except np.linalg.LinAlgError:
                break

            dx, dy, dtheta = params

            # Build incremental R, t
            cos_dt = np.cos(dtheta)
            sin_dt = np.sin(dtheta)
            R_inc = np.array([[cos_dt, -sin_dt],
                              [sin_dt,  cos_dt]])
            t_inc = np.array([dx, dy])

            # Apply increment
            src = (R_inc @ src.T).T + t_inc
            R_total = R_inc @ R_total
            t_total = R_inc @ t_total + t_inc

            # Convergence check
            trans_norm = np.sqrt(dx**2 + dy**2)
            rot_norm = abs(dtheta)
            if trans_norm < self.icp_tolerance and rot_norm < self.icp_tolerance:
                return R_total, t_total, True

        return R_total, t_total, True

    def update(self, ranges, angle_min, angle_increment, timestamp,
               odom_vx=0.0, imu_gz=0.0):
        """
        Process a new laser scan and estimate body-frame velocity.

        Uses point-to-line ICP with an odometry-seeded initial guess for
        robust and accurate velocity estimation.

        The ICP transform maps previous-scan points to current-scan points:
            p_current ≈ R_icp @ p_prev + t_icp

        From this the robot's displacement in the current laser frame is:
            d_laser = -t_icp   (environment shifts opposite to robot motion)
            dtheta  = -arctan2(R_icp[1,0], R_icp[0,0])

        Velocity is then d_laser / dt, transformed from laser frame to
        base_link (accounting for the x-offset of the laser).

        Args:
            ranges: Array of range measurements from LaserScan
            angle_min: Start angle of scan (rad)
            angle_increment: Angular step between beams (rad)
            timestamp: Scan timestamp in seconds
            odom_vx: Current wheel odometry forward velocity (m/s),
                     used as initial guess for ICP translation
            imu_gz: Current IMU yaw rate (rad/s), used as initial
                    guess for ICP rotation

        Returns:
            (vx, vy): Body-frame velocities (m/s), or (None, None) if
                      not enough data yet (first scan).
        """
        # Deskew the scan using odometry velocity.
        # We always use odom_vx (wheel-based) for deskewing because:
        # 1. It's unbiased (validated cruise ratio = 1.000 at 3 m/s)
        # 2. Using ICP velocity for deskew creates a feedback loop:
        #    biased velocity → insufficient deskew → biased velocity
        # 3. Odom has negligible noise at the scan period timescale
        deskew_vx = odom_vx
        deskew_omega = imu_gz
        points = self._scan_to_points(
            ranges, angle_min, angle_increment,
            deskew_vx=deskew_vx, deskew_vy=0.0, deskew_omega=deskew_omega)

        if len(points) < 30:
            return None, None

        if self._prev_points is None:
            self._prev_points = points
            self._prev_time = timestamp
            self._initialized = True
            return None, None

        dt = timestamp - self._prev_time
        if dt < 1e-6:
            return self.velocity_x, self.velocity_y

        self._dt = dt

        # --- Odometry-seeded initial guess ---
        # Predict how much the environment moved in the laser frame
        # between scans. The robot moves forward by odom_vx * dt and
        # rotates by imu_gz * dt. The environment appears to shift in
        # the opposite direction.
        #
        # For ICP: p_current ≈ R_guess @ p_prev + t_guess
        # Environment shift = opposite of robot motion
        dtheta_guess = -imu_gz * dt   # environment rotates opposite
        cos_g = np.cos(dtheta_guess)
        sin_g = np.sin(dtheta_guess)
        R_guess = np.array([[cos_g, -sin_g],
                            [sin_g,  cos_g]])
        # Robot moves forward by odom_vx*dt in laser frame → env shifts back
        t_guess = np.array([-odom_vx * dt, 0.0])
        # Apply rotation to translation
        t_guess = R_guess @ np.zeros(2) + t_guess  # simplified: just t_guess

        # Run point-to-line ICP with initial guess
        R, t, converged = self._icp_point_to_line(
            self._prev_points, points, R_guess, t_guess)

        self._prev_points = points
        self._prev_time = timestamp

        if not converged:
            return self.velocity_x, self.velocity_y

        # Robot displacement in current laser frame
        # Robot moves forward → environment shifts backward → t_icp is negative
        d_laser_x = -t[0]
        d_laser_y = -t[1]
        dtheta = -np.arctan2(R[1, 0], R[0, 0])

        # Velocity in laser frame
        vx_laser = d_laser_x / dt
        vy_laser = d_laser_y / dt
        omega = dtheta / dt

        # Transform to base_link frame
        # Laser is at (offset, 0) in base frame.
        # Rigid body: v_laser = v_base + omega × r_{base→laser}
        # omega × (offset, 0) = (0, omega*offset)
        # So: v_base = v_laser - (0, omega*offset)
        vx_base = vx_laser
        vy_base = vy_laser - omega * self.laser_x_offset

        # Sanity check: reject physically impossible velocities
        speed = np.sqrt(vx_base**2 + vy_base**2)
        if speed > self.velocity_limit:
            return self.velocity_x, self.velocity_y

        # Store raw (unfiltered) values for debugging
        self._raw_vx = vx_base
        self._raw_vy = vy_base

        # Apply EMA low-pass filter to smooth per-scan ICP noise
        # v_filtered = alpha * v_new + (1 - alpha) * v_prev
        a = self.ema_alpha
        self.velocity_x = a * vx_base + (1 - a) * self.velocity_x
        self.velocity_y = a * vy_base + (1 - a) * self.velocity_y
        self.omega = a * omega + (1 - a) * self.omega

        return self.velocity_x, self.velocity_y

    def reset(self):
        """Reset the estimator (clears previous scan data)."""
        self._prev_points = None
        self._prev_time = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.omega = 0.0
        self._raw_vx = 0.0
        self._raw_vy = 0.0
        self._initialized = False

    @property
    def velocity(self):
        """Forward velocity (compatible interface with ImuVelocityEstimator)."""
        return self.velocity_x


def radius_from_imu(speed: float, yaw_rate: float) -> float:
    """
    Compute turning radius from speed and IMU yaw rate.

        R = |v| / |ω|

    This is independent of wheel odometry position integration and
    therefore unaffected by tire slip or ERPM estimation errors.

    Args:
        speed: Forward speed in m/s (can be from odom or IMU-integrated).
        yaw_rate: Yaw rate in rad/s (from /sensors/imu/raw angular_velocity.z).

    Returns:
        Turning radius in meters, or float('inf') if yaw rate ≈ 0.
    """
    if abs(yaw_rate) < 0.01:
        return float('inf')
    return abs(speed) / abs(yaw_rate)


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


# ============================================================================
# Vehicle Parameter Auto-Update
# ============================================================================

VEHICLE_PARAMS_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), 'vehicle_params.yaml')

# Dependency map: which tests need re-running when a parameter changes
PARAM_DEPENDENCIES = {
    'mass': ['test_cornering_stiffness.py', 'test_longitudinal_stiffness.py',
             'test_motor_torque.py'],
    'l_f': ['test_cornering_stiffness.py'],
    'l_r': ['test_cornering_stiffness.py'],
    'r_eff': ['test_longitudinal_stiffness.py', 'test_motor_torque.py'],
    'gear_ratio': ['test_motor_torque.py'],
    'wheelbase': ['test_cornering_stiffness.py', 'test_steering_rate.py'],
}


def update_vehicle_param(key: str, value, status: str = 'TESTED',
                         logger=None):
    """
    Update a single parameter in vehicle_params.yaml.
    
    Replaces the value on a line like:
        key: <old_value>    # ...
    with:
        key: <new_value>    # [STATUS] ...
    
    Args:
        key:    YAML key name (e.g., 'mu', 'max_velocity', 'C_alpha_f')
        value:  New value (float, int, or str)
        status: Tag like 'TESTED', 'CALC', 'MEASURED'
        logger: Optional ROS2 logger for output
    """
    if not os.path.isfile(VEHICLE_PARAMS_FILE):
        if logger:
            logger.warn(f'vehicle_params.yaml not found, skipping auto-update')
        return False

    with open(VEHICLE_PARAMS_FILE, 'r') as f:
        lines = f.readlines()

    # Format the value
    if isinstance(value, float):
        if abs(value) < 0.001 and value != 0:
            val_str = f'{value:.6f}'
        elif abs(value) < 1:
            val_str = f'{value:.4f}'
        else:
            val_str = f'{value:.4f}'
        # Strip trailing zeros but keep at least one decimal
        val_str = val_str.rstrip('0').rstrip('.')
        if '.' not in val_str:
            val_str += '.0'
    else:
        val_str = str(value)

    # Find the line with this key
    import re as _re
    pattern = _re.compile(
        r'^(\s*)' + _re.escape(key) + r':\s*'
        r'([^#\n]*?)'          # current value (group 2)
        r'(\s*#\s*)'           # comment start (group 3)
        r'(\[.*?\]\s*)?'       # optional [TAG] (group 4)
        r'(.*?)$'              # rest of comment (group 5)
    )

    updated = False
    for i, line in enumerate(lines):
        m = pattern.match(line)
        if m:
            indent = m.group(1)
            comment_sep = m.group(3)
            rest_comment = m.group(5) or ''
            # Reconstruct line with new value and status tag
            new_line = (f'{indent}{key}: {val_str}'
                        f'{comment_sep}[{status}] {rest_comment}\n')
            lines[i] = new_line
            updated = True
            break

    if not updated:
        # Try simpler pattern (key with ~ or no comment)
        simple = _re.compile(
            r'^(\s*)' + _re.escape(key) + r':\s*~?\s*(#.*)?$')
        for i, line in enumerate(lines):
            m = simple.match(line)
            if m:
                indent = m.group(1)
                comment = m.group(2) or ''
                # Insert [STATUS] into comment if present
                if comment:
                    comment = _re.sub(r'\[.*?\]\s*', '', comment)
                    comment = comment.replace('# ', f'# [{status}] ', 1)
                else:
                    comment = f'# [{status}]'
                lines[i] = f'{indent}{key}: {val_str}  {comment}\n'
                updated = True
                break

    if updated:
        with open(VEHICLE_PARAMS_FILE, 'w') as f:
            f.writelines(lines)
        if logger:
            logger.info(f'  Updated vehicle_params.yaml: {key} = {val_str}')
    else:
        if logger:
            logger.warn(f'  Could not find "{key}" in vehicle_params.yaml')

    return updated


def update_vehicle_params(params: dict, status: str = 'TESTED',
                          logger=None):
    """
    Update multiple parameters in vehicle_params.yaml at once.
    
    Args:
        params: Dict of {key: value} pairs
        status: Tag like 'TESTED', 'CALC'
        logger: Optional ROS2 logger
    """
    for key, value in params.items():
        if value is not None:
            update_vehicle_param(key, value, status=status, logger=logger)

