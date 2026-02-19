#!/usr/bin/env python3
"""
Slip Detector Unit Tests

Tests the IMU-based slip detection logic, noise multiplier, and sensor boost.

Usage:
    python3 -m pytest gpu_amcl/test/test_slip_detector.py -v
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpu_amcl.core.slip_detector import SlipDetector, SlipDetectorConfig


class TestSlipDetectorBasic:
    """Basic slip detector functionality."""

    def test_no_slip_at_rest(self):
        detector = SlipDetector()
        is_slipping = detector.update(
            velocity=0.0, dt=0.01,
            imu_linear_accel=(0.0, 0.0, 9.81),
            imu_angular_velocity=0.0
        )
        assert not is_slipping

    def test_no_slip_constant_velocity(self):
        detector = SlipDetector()
        # Drive at constant 2 m/s — no acceleration difference
        for _ in range(10):
            is_slipping = detector.update(
                velocity=2.0, dt=0.01,
                imu_linear_accel=(0.0, 0.0, 9.81),
                imu_angular_velocity=0.0
            )
        assert not is_slipping

    def test_forward_slip_detected(self):
        config = SlipDetectorConfig(slip_threshold=0.3, filter_window=2)
        detector = SlipDetector(config)
        # Velocity changing → expected accel = 100 m/s²
        # But IMU says 0 → big mismatch
        detector.update(velocity=0.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        is_slipping = detector.update(
            velocity=1.0, dt=0.01,
            imu_linear_accel=(0.0, 0.0, 9.81),
            imu_angular_velocity=0.0
        )
        assert is_slipping

    def test_lateral_slip_detected(self):
        config = SlipDetectorConfig(lateral_threshold=0.2, filter_window=2)
        detector = SlipDetector(config)
        for _ in range(3):
            is_slipping = detector.update(
                velocity=3.0, dt=0.01,
                imu_linear_accel=(0.0, 5.0, 9.81),  # High lateral accel
                imu_angular_velocity=1.0
            )
        assert is_slipping

    def test_invalid_dt_ignored(self):
        detector = SlipDetector()
        # dt=0 should not crash
        detector.update(velocity=1.0, dt=0.0,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        # dt > 1.0 should not crash
        detector.update(velocity=1.0, dt=2.0,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)


class TestNoiseMultiplier:
    """Tests for noise multiplier and sensor boost."""

    def test_no_slip_multiplier_is_one(self):
        detector = SlipDetector()
        detector.update(velocity=0.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        assert detector.get_noise_multiplier() == 1.0

    def test_slip_multiplier_above_one(self):
        config = SlipDetectorConfig(slip_threshold=0.1, filter_window=1, slip_noise_multiplier=3.0)
        detector = SlipDetector(config)
        detector.update(velocity=0.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        detector.update(velocity=5.0, dt=0.01,  # Huge accel mismatch
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        mult = detector.get_noise_multiplier()
        assert mult > 1.0
        assert mult <= 3.0

    def test_sensor_boost_when_slipping(self):
        config = SlipDetectorConfig(slip_threshold=0.1, filter_window=1, slip_sensor_boost=2.0)
        detector = SlipDetector(config)
        detector.update(velocity=0.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        detector.update(velocity=5.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        boost = detector.get_sensor_boost()
        assert boost > 1.0
        assert boost <= 2.0


class TestSlipStats:
    """Test statistics and reset."""

    def test_stats_dict_keys(self):
        detector = SlipDetector()
        stats = detector.get_stats()
        expected_keys = {'is_slipping', 'slip_confidence', 'slip_count',
                         'total_updates', 'slip_rate_percent',
                         'noise_multiplier', 'sensor_boost'}
        assert set(stats.keys()) == expected_keys

    def test_reset_clears_state(self):
        detector = SlipDetector()
        for _ in range(5):
            detector.update(velocity=1.0, dt=0.01,
                            imu_linear_accel=(10.0, 10.0, 9.81),
                            imu_angular_velocity=1.0)
        detector.reset()
        assert detector.total_updates == 0
        assert detector.slip_count == 0
        assert detector.slip_confidence == 0.0

    def test_slip_count_increments_on_transitions(self):
        config = SlipDetectorConfig(slip_threshold=0.1, filter_window=1)
        detector = SlipDetector(config)
        # No slip
        detector.update(velocity=0.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        # Trigger slip
        detector.update(velocity=5.0, dt=0.01,
                        imu_linear_accel=(0.0, 0.0, 9.81),
                        imu_angular_velocity=0.0)
        assert detector.slip_count >= 1


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
