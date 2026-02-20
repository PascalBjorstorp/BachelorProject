#!/usr/bin/env python3
"""
KLD Sampling & Recovery Unit Tests

Tests the adaptive particle count (KLD sampling) and
kidnapped-robot recovery features.

Usage:
    python3 -m pytest gpu_amcl/test/test_kld_and_recovery.py -v
"""

import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpu_amcl.core.particle_filter import ParticleFilter, ParticleFilterConfig


def _make_box_map(size=100):
    m = np.zeros((size, size), dtype=np.int8)
    m[0, :] = 100
    m[-1, :] = 100
    m[:, 0] = 100
    m[:, -1] = 100
    return m


class TestKLDSampling:
    """Tests for KLD adaptive particle count."""

    def test_kld_returns_bounded_count(self):
        config = ParticleFilterConfig(
            num_particles=500,
            min_particles=50,
            max_particles=2000,
            use_gpu=False,
            use_kld_sampling=True,
            kld_epsilon=0.05,
            kld_z=2.33,
        )
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        n = pf._compute_kld_particle_count()
        assert config.min_particles <= n <= config.max_particles

    def test_kld_low_spread_fewer_particles(self):
        """Tightly clustered particles should need fewer bins → fewer particles."""
        config = ParticleFilterConfig(
            num_particles=1000,
            min_particles=50,
            max_particles=5000,
            use_gpu=False,
            use_kld_sampling=True,
            initial_cov=(0.01, 0.01, 0.01),  # Very tight cluster
        )
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        n_tight = pf._compute_kld_particle_count()

        config2 = ParticleFilterConfig(
            num_particles=1000,
            min_particles=50,
            max_particles=5000,
            use_gpu=False,
            use_kld_sampling=True,
            initial_cov=(2.0, 2.0, 1.0),  # Very spread out
        )
        pf2 = ParticleFilter(config2)
        pf2.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        n_spread = pf2._compute_kld_particle_count()

        assert n_tight <= n_spread, \
            f"Tight cluster ({n_tight}) should need <= particles than spread ({n_spread})"


class TestRecovery:
    """Tests for kidnapped-robot recovery / random particle injection."""

    def test_inject_random_particles(self):
        config = ParticleFilterConfig(
            num_particles=200,
            use_gpu=False,
            recovery_alpha_slow=0.001,
            recovery_alpha_fast=0.1,
        )
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        # Record original particles
        orig_particles = pf.particles.copy()

        # Inject 50% random particles
        pf._inject_random_particles(0.5)

        n_changed = np.sum(
            np.any(pf.particles != orig_particles, axis=1)
        )
        # At least some particles should have changed
        assert n_changed >= 50, f"Expected >=50 changed particles, got {n_changed}"

    def test_recovery_disabled_when_alpha_zero(self):
        """When recovery_alpha_slow=0, no recovery tracker should activate."""
        config = ParticleFilterConfig(
            num_particles=100,
            use_gpu=False,
            recovery_alpha_slow=0.0,
        )
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        assert pf._w_slow == 0.0
        assert pf._w_fast == 0.0

    def test_free_cells_precomputed(self):
        config = ParticleFilterConfig(num_particles=100, use_gpu=False)
        pf = ParticleFilter(config)
        m = _make_box_map(50)
        pf.initialize(
            initial_pose=(2.5, 2.5, 0.0),
            map_data=m,
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        assert pf._free_cells is not None
        # 50x50 map with 4 walls (1 pixel each), inner area = 48*48 = 2304
        assert len(pf._free_cells) == 48 * 48


class TestEdgeCases:
    """Edge-case tests for particle filter."""

    def test_zero_motion_no_crash(self):
        config = ParticleFilterConfig(num_particles=100, use_gpu=False)
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        # Zero motion should not crash
        pf.predict((0.0, 0.0, 0.0))
        est = pf.get_estimate()
        assert abs(est.x - 5.0) < 1.0

    def test_get_estimate_covariance_shape(self):
        config = ParticleFilterConfig(num_particles=100, use_gpu=False)
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        est = pf.get_estimate()
        assert est.covariance.shape == (3, 3)
        # Covariance should be symmetric
        assert np.allclose(est.covariance, est.covariance.T, atol=1e-6)

    def test_empty_scan_handled(self):
        """An empty scan array shouldn't crash."""
        config = ParticleFilterConfig(num_particles=50, use_gpu=False)
        pf = ParticleFilter(config)
        pf.initialize(
            initial_pose=(5.0, 5.0, 0.0),
            map_data=_make_box_map(100),
            map_resolution=0.1,
            map_origin=(0.0, 0.0, 0.0),
        )
        try:
            pf.update(np.array([], dtype=np.float32), -np.pi / 2, np.pi / 180)
        except Exception:
            pass  # Acceptable to raise, just shouldn't segfault


if __name__ == '__main__':
    import pytest
    pytest.main([__file__, '-v'])
