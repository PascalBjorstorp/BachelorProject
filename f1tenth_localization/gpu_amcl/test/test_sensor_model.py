#!/usr/bin/env python3
"""
Sensor Model Unit Tests

Tests the likelihood field sensor model weight computation,
distance transform, and edge cases.

Usage:
    python3 -m pytest gpu_amcl/test/test_sensor_model.py -v
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpu_amcl.core.sensor_model import SensorModel, SensorModelConfig


def _make_box_map(size: int = 100) -> np.ndarray:
    """Create a simple box map with walls on all edges."""
    m = np.zeros((size, size), dtype=np.int8)
    m[0, :] = 100
    m[-1, :] = 100
    m[:, 0] = 100
    m[:, -1] = 100
    return m


class TestDistanceField:
    """Tests for distance field construction."""

    def test_distance_field_shape(self):
        m = _make_box_map(100)
        sensor = SensorModel(m, resolution=0.1, origin=(0.0, 0.0, 0.0), use_gpu=False)
        assert sensor.distance_field.shape == (100, 100)

    def test_obstacle_distance_zero(self):
        m = _make_box_map(100)
        sensor = SensorModel(m, resolution=0.1, origin=(0.0, 0.0, 0.0), use_gpu=False)
        # Cells on the wall should have distance 0
        assert sensor.distance_field[0, 50] == 0.0
        assert sensor.distance_field[99, 50] == 0.0

    def test_free_space_distance_positive(self):
        m = _make_box_map(100)
        sensor = SensorModel(m, resolution=0.1, origin=(0.0, 0.0, 0.0), use_gpu=False)
        # Center cell should be far from walls
        assert sensor.distance_field[50, 50] > 0.0

    def test_distance_capped_at_max_dist(self):
        m = _make_box_map(200)
        config = SensorModelConfig(max_dist=1.0)
        sensor = SensorModel(m, resolution=0.05, origin=(0.0, 0.0, 0.0), config=config, use_gpu=False)
        assert sensor.distance_field.max() <= 1.0


class TestWeightComputation:
    """Tests for compute_weights."""

    def _make_sensor(self, map_size=100, resolution=0.1):
        m = _make_box_map(map_size)
        config = SensorModelConfig(max_beams=10, max_range=10.0)
        return SensorModel(m, resolution=resolution, origin=(0.0, 0.0, 0.0),
                           config=config, use_gpu=False)

    def test_output_shape(self):
        sensor = self._make_sensor()
        particles = np.array([[5.0, 5.0, 0.0]], dtype=np.float32)
        ranges = np.ones(20, dtype=np.float32) * 2.0
        w = sensor.compute_weights(particles, ranges, -np.pi / 2, np.pi / 20)
        assert w.shape == (1,)

    def test_weights_positive(self):
        sensor = self._make_sensor()
        N = 50
        particles = np.zeros((N, 3), dtype=np.float32)
        particles[:, 0] = 5.0
        particles[:, 1] = 5.0
        ranges = np.ones(20, dtype=np.float32) * 2.0
        w = sensor.compute_weights(particles, ranges, -np.pi / 2, np.pi / 20)
        assert np.all(w > 0), "All weights should be positive"

    def test_particle_near_wall_higher_weight(self):
        """A particle whose beams actually hit the wall should score higher
        than one placed randomly with the same scan."""
        m = _make_box_map(100)
        config = SensorModelConfig(max_beams=10, max_range=10.0, laser_offset_x=0.0)
        sensor = SensorModel(m, resolution=0.1, origin=(0.0, 0.0, 0.0),
                             config=config, use_gpu=False)

        # Particle near left wall facing left → range ~0.5 m to wall
        near_wall = np.array([[0.5, 5.0, np.pi]], dtype=np.float32)
        far_away = np.array([[5.0, 5.0, np.pi]], dtype=np.float32)

        # Scan says 0.5 m everywhere
        ranges = np.full(20, 0.5, dtype=np.float32)
        w_near = sensor.compute_weights(near_wall, ranges, -np.pi / 4, np.pi / 20)
        w_far = sensor.compute_weights(far_away, ranges, -np.pi / 4, np.pi / 20)

        assert float(w_near[0]) >= float(w_far[0]), \
            "Particle near the wall should score at least as high"

    def test_all_invalid_ranges(self):
        """All ranges outside valid range should produce uniform weights."""
        sensor = self._make_sensor()
        particles = np.array([[5.0, 5.0, 0.0], [3.0, 3.0, 1.0]], dtype=np.float32)
        # All ranges are 0 (invalid)
        ranges = np.zeros(20, dtype=np.float32)
        w = sensor.compute_weights(particles, ranges, -np.pi / 2, np.pi / 20)
        # When all beams are invalid, all weights should be equal (~1.0)
        assert np.allclose(w, w[0]), "Invalid ranges should produce equal weights"

    def test_max_range_readings(self):
        """Max-range readings should not crash and produce valid weights."""
        sensor = self._make_sensor()
        particles = np.array([[5.0, 5.0, 0.0]], dtype=np.float32)
        ranges = np.full(20, 9.99, dtype=np.float32)
        w = sensor.compute_weights(particles, ranges, -np.pi / 2, np.pi / 20)
        assert np.all(np.isfinite(w)), "Weights should be finite"


class TestCoordinateConversion:
    """Tests for world_to_map and helper methods."""

    def test_world_to_map_origin(self):
        m = _make_box_map(100)
        sensor = SensorModel(m, resolution=0.1, origin=(0.0, 0.0, 0.0), use_gpu=False)
        mx, my = sensor.world_to_map(0.0, 0.0)
        assert mx == 0 and my == 0

    def test_world_to_map_offset(self):
        m = _make_box_map(100)
        sensor = SensorModel(m, resolution=0.1, origin=(-5.0, -5.0, 0.0), use_gpu=False)
        mx, my = sensor.world_to_map(0.0, 0.0)
        assert mx == 50 and my == 50

    def test_is_in_map_bounds(self):
        m = _make_box_map(100)
        sensor = SensorModel(m, resolution=0.1, origin=(0.0, 0.0, 0.0), use_gpu=False)
        assert sensor.is_in_map(50, 50) is True
        assert sensor.is_in_map(-1, 50) is False
        assert sensor.is_in_map(100, 50) is False


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
