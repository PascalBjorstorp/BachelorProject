#!/usr/bin/env python3
"""
Map Utilities Unit Tests

Tests map loading, coordinate transforms, and occupancy grid conversion.

Usage:
    python3 -m pytest gpu_amcl/test/test_map_utils.py -v
"""

import numpy as np
import sys
import os
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpu_amcl.utils.map_utils import MapProcessor


class TestMapProcessor:
    """Tests for MapProcessor."""

    def test_initial_state(self):
        mp = MapProcessor()
        assert mp.map_data is None
        assert mp.map_loaded is False

    def test_world_to_map_identity_origin(self):
        mp = MapProcessor()
        mp.resolution = 0.1
        mp.origin = (0.0, 0.0, 0.0)
        mp.map_data = np.zeros((100, 100), dtype=np.int8)
        mx, my = mp.world_to_map(1.0, 2.0)
        assert mx == 10
        assert my == 20

    def test_world_to_map_with_offset(self):
        mp = MapProcessor()
        mp.resolution = 0.05
        mp.origin = (-5.0, -5.0, 0.0)
        mp.map_data = np.zeros((200, 200), dtype=np.int8)
        mx, my = mp.world_to_map(0.0, 0.0)
        assert mx == 100
        assert my == 100

    def test_map_to_world_roundtrip(self):
        mp = MapProcessor()
        mp.resolution = 0.1
        mp.origin = (-5.0, -3.0, 0.0)
        mp.map_data = np.zeros((100, 100), dtype=np.int8)
        # world -> map -> world (with center-of-cell offset)
        wx, wy = 1.0, 2.0
        mx, my = mp.world_to_map(wx, wy)
        wx2, wy2 = mp.map_to_world(mx, my)
        assert abs(wx2 - wx) < mp.resolution
        assert abs(wy2 - wy) < mp.resolution

    def test_is_in_bounds(self):
        mp = MapProcessor()
        mp.map_data = np.zeros((50, 80), dtype=np.int8)
        assert mp.is_in_bounds(0, 0) is True
        assert mp.is_in_bounds(79, 49) is True
        assert mp.is_in_bounds(80, 0) is False
        assert mp.is_in_bounds(0, 50) is False
        assert mp.is_in_bounds(-1, 0) is False

    def test_is_free_and_occupied(self):
        mp = MapProcessor()
        mp.resolution = 0.1
        mp.origin = (0.0, 0.0, 0.0)
        mp.map_data = np.zeros((100, 100), dtype=np.int8)
        mp.map_data[50, 50] = 100  # occupied cell

        assert mp.is_free(2.0, 2.0)            # cell (20,20) = free
        assert not mp.is_free(5.0, 5.0)         # cell (50,50) = occupied
        assert mp.is_occupied(5.0, 5.0)

    def test_is_occupied_outside_map(self):
        mp = MapProcessor()
        mp.resolution = 0.1
        mp.origin = (0.0, 0.0, 0.0)
        mp.map_data = np.zeros((100, 100), dtype=np.int8)
        # Outside map should be treated as occupied
        assert mp.is_occupied(-1.0, -1.0) is True

    def test_get_free_cells_returns_coordinates(self):
        mp = MapProcessor()
        mp.resolution = 1.0
        mp.origin = (0.0, 0.0, 0.0)
        # Small map: all free except a wall
        mp.map_data = np.zeros((5, 5), dtype=np.int8)
        mp.map_data[0, :] = 100
        mp.map_loaded = True
        free = mp.get_free_cells()
        # 5x5 grid, top row occupied → 20 free cells
        assert len(free) == 20
        assert free.shape[1] == 2

    def test_get_free_cells_empty_map(self):
        mp = MapProcessor()
        assert len(mp.get_free_cells()) == 0


class TestImageToOccupancy:
    """Tests for image-to-occupancy conversion."""

    def test_white_is_free(self):
        mp = MapProcessor()
        img = np.full((10, 10), 255, dtype=np.uint8)  # all white
        occ = mp._image_to_occupancy(img, negate=0, occupied_thresh=0.65, free_thresh=0.196)
        assert np.all(occ == 0), "White should be free"

    def test_black_is_occupied(self):
        mp = MapProcessor()
        img = np.zeros((10, 10), dtype=np.uint8)  # all black
        occ = mp._image_to_occupancy(img, negate=0, occupied_thresh=0.65, free_thresh=0.196)
        assert np.all(occ == 100), "Black should be occupied"

    def test_gray_is_unknown(self):
        mp = MapProcessor()
        img = np.full((10, 10), 128, dtype=np.uint8)  # mid gray
        occ = mp._image_to_occupancy(img, negate=0, occupied_thresh=0.65, free_thresh=0.196)
        assert np.all(occ == -1), "Gray should be unknown"

    def test_negate_inverts(self):
        mp = MapProcessor()
        img_white = np.full((10, 10), 255, dtype=np.uint8)
        occ_normal = mp._image_to_occupancy(img_white, negate=0, occupied_thresh=0.65, free_thresh=0.196)
        occ_negated = mp._image_to_occupancy(img_white, negate=1, occupied_thresh=0.65, free_thresh=0.196)
        # White + negate → should be occupied (inverted)
        assert np.all(occ_normal == 0)
        assert np.all(occ_negated == 100)


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
