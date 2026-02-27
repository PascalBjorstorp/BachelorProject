"""
Slip Detector - IMU-based wheel slip detection for high-speed driving.

Compares expected acceleration (from wheel odometry) with measured acceleration
(from IMU) to detect when the wheels are slipping (e.g., during aggressive
cornering or hard braking).

When slip is detected, the particle filter can:
1. Increase motion model noise (be more uncertain)
2. Trust the sensor model more (rely on LiDAR)
3. Prevent overconfident localization during aggressive maneuvers

Usage:
    detector = SlipDetector(threshold=0.5)
    is_slipping = detector.update(odom_velocity, odom_acceleration, imu_linear_accel)
"""

import numpy as np
from typing import Tuple, Optional
from collections import deque


class SlipDetectorConfig:
    """Configuration for slip detection."""
    
    def __init__(
        self,
        # Slip detection threshold
        slip_threshold: float = 0.5,  # m/s² difference to trigger slip
        # Filtering
        filter_window: int = 5,       # Number of samples to average
        # Response
        slip_noise_multiplier: float = 2.0,  # Multiply motion noise when slipping
        slip_sensor_boost: float = 1.5,      # Boost sensor model weight when slipping
        # Tuning
        lateral_threshold: float = 0.3,      # Lateral acceleration slip detection (m/s²)
    ):
        self.slip_threshold = slip_threshold
        self.filter_window = filter_window
        self.slip_noise_multiplier = slip_noise_multiplier
        self.slip_sensor_boost = slip_sensor_boost
        self.lateral_threshold = lateral_threshold


class SlipDetector:
    """
    Detect wheel slip using IMU accelerometer data.
    
    The idea is simple:
    - From wheel odometry, we can estimate expected acceleration
    - From IMU, we measure actual acceleration
    - If they differ significantly, wheels are slipping
    
    Slip indicators:
    1. Forward slip: Expected forward accel ≠ measured forward accel (wheel spin/lockup)
    2. Lateral slip: High lateral acceleration (drifting/sliding)
    3. Rotation slip: Gyro shows rotation but wheels don't (oversteer)
    """
    
    def __init__(self, config: SlipDetectorConfig = None):
        """
        Initialize slip detector.
        
        Args:
            config: Detection parameters
        """
        self.config = config or SlipDetectorConfig()
        
        # History for filtering
        self.accel_diff_history = deque(maxlen=self.config.filter_window)
        self.lateral_accel_history = deque(maxlen=self.config.filter_window)
        
        # State
        self.is_slipping = False
        self.slip_confidence = 0.0  # 0.0 = no slip, 1.0 = definite slip
        self.last_velocity = 0.0
        self.last_time = None
        
        # Statistics
        self.slip_count = 0
        self.total_updates = 0
    
    def update(
        self,
        velocity: float,
        dt: float,
        imu_linear_accel: Tuple[float, float, float],
        imu_angular_velocity: float
    ) -> bool:
        """
        Update slip detection with new measurements.
        
        Args:
            velocity: Current velocity from wheel odometry (m/s)
            dt: Time delta since last update (seconds)
            imu_linear_accel: (ax, ay, az) acceleration from IMU (m/s²)
            imu_angular_velocity: Angular velocity from gyro (rad/s)
        
        Returns:
            True if slip is detected
        """
        self.total_updates += 1
        
        # Skip if dt is invalid
        if dt <= 0 or dt > 1.0:
            self.last_velocity = velocity
            return self.is_slipping
        
        # Expected acceleration from odometry
        expected_accel = (velocity - self.last_velocity) / dt
        self.last_velocity = velocity
        
        # Measured accelerations from IMU (x = forward, y = lateral)
        measured_forward_accel = imu_linear_accel[0]
        measured_lateral_accel = imu_linear_accel[1]
        
        # Compute differences
        forward_diff = abs(expected_accel - measured_forward_accel)
        lateral_mag = abs(measured_lateral_accel)
        
        # Add to history for filtering
        self.accel_diff_history.append(forward_diff)
        self.lateral_accel_history.append(lateral_mag)
        
        # Compute filtered values
        avg_forward_diff = np.mean(self.accel_diff_history)
        avg_lateral_accel = np.mean(self.lateral_accel_history)
        
        # Slip detection logic
        forward_slip = avg_forward_diff > self.config.slip_threshold
        lateral_slip = avg_lateral_accel > self.config.lateral_threshold
        
        # Combined slip detection
        was_slipping = self.is_slipping
        self.is_slipping = forward_slip or lateral_slip
        
        # Compute confidence (how severe is the slip)
        forward_confidence = min(1.0, avg_forward_diff / (self.config.slip_threshold * 2))
        lateral_confidence = min(1.0, avg_lateral_accel / (self.config.lateral_threshold * 2))
        self.slip_confidence = max(forward_confidence, lateral_confidence)
        
        # Track slip events
        if self.is_slipping and not was_slipping:
            self.slip_count += 1
        
        return self.is_slipping
    
    def get_noise_multiplier(self) -> float:
        """
        Get motion model noise multiplier based on slip state.
        
        Returns:
            Multiplier for motion model alpha parameters (1.0 if no slip)
        """
        if not self.is_slipping:
            return 1.0
        
        # Gradually increase noise based on slip confidence
        max_mult = self.config.slip_noise_multiplier
        return 1.0 + (max_mult - 1.0) * self.slip_confidence
    
    def get_sensor_boost(self) -> float:
        """
        Get sensor model weight boost based on slip state.
        
        When slipping, we should trust LiDAR more and odometry less.
        
        Returns:
            Multiplier for sensor model weight (1.0 if no slip)
        """
        if not self.is_slipping:
            return 1.0
        
        # Gradually boost sensor trust based on slip confidence
        max_boost = self.config.slip_sensor_boost
        return 1.0 + (max_boost - 1.0) * self.slip_confidence
    
    def get_stats(self) -> dict:
        """Get slip detection statistics."""
        slip_rate = self.slip_count / max(1, self.total_updates) * 100
        return {
            'is_slipping': self.is_slipping,
            'slip_confidence': self.slip_confidence,
            'slip_count': self.slip_count,
            'total_updates': self.total_updates,
            'slip_rate_percent': slip_rate,
            'noise_multiplier': self.get_noise_multiplier(),
            'sensor_boost': self.get_sensor_boost()
        }
    
    def reset(self):
        """Reset slip detector state."""
        self.accel_diff_history.clear()
        self.lateral_accel_history.clear()
        self.is_slipping = False
        self.slip_confidence = 0.0
        self.last_velocity = 0.0
        self.slip_count = 0
        self.total_updates = 0
