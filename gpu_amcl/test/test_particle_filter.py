#!/usr/bin/env python3
"""
GPU AMCL Unit Tests

Tests the core particle filter components without ROS dependencies.
Useful for verifying GPU acceleration is working correctly.

Usage:
    python3 -m pytest test/test_particle_filter.py -v
    
Or run directly:
    python3 test/test_particle_filter.py
"""

import numpy as np
import time
import sys
import os

# Add package to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from gpu_amcl.core.particle_filter import ParticleFilter, ParticleFilterConfig
from gpu_amcl.core.motion_model import MotionModel, MotionModelConfig
from gpu_amcl.core.resampling import Resampler


def test_motion_model():
    """Test motion model applies noise correctly."""
    print("\n=== Testing Motion Model ===")
    
    for use_gpu in [False, True]:
        try:
            motion = MotionModel(use_gpu=use_gpu)
            mode = "GPU" if motion.use_gpu else "CPU"
            
            # Create test particles
            N = 1000
            particles = np.zeros((N, 3), dtype=np.float32)
            particles[:, 0] = 5.0  # All at x=5
            particles[:, 1] = 3.0  # All at y=3
            particles[:, 2] = 0.0  # All facing forward
            
            if use_gpu and motion.use_gpu:
                import cupy as cp
                particles = cp.asarray(particles)
            
            # Apply motion
            odom_delta = (1.0, 0.0, 0.1)  # Move 1m forward, rotate 0.1 rad
            
            t0 = time.perf_counter()
            for _ in range(100):
                new_particles = motion.apply(particles, odom_delta)
            t1 = time.perf_counter()
            
            # Transfer back to CPU for analysis
            if motion.use_gpu:
                new_particles = new_particles.get()
            
            # Check particles moved approximately correctly
            mean_x = np.mean(new_particles[:, 0])
            mean_y = np.mean(new_particles[:, 1])
            
            # Should be roughly at x=6 (started at 5, moved 1)
            assert abs(mean_x - 6.0) < 0.5, f"Mean X should be ~6, got {mean_x}"
            
            # Check there's variance (noise was added)
            std_x = np.std(new_particles[:, 0])
            assert std_x > 0.01, f"Particles should have variance, got std={std_x}"
            
            print(f"  [{mode}] Motion model: PASSED ({(t1-t0)*10:.1f}ms per 100 updates)")
            print(f"       Mean pos: ({mean_x:.2f}, {mean_y:.2f}), std: ({std_x:.3f})")
            
        except ImportError:
            if use_gpu:
                print(f"  [GPU] Skipped (CuPy not available)")
    
    print("  Motion model tests passed!")


def test_resampler():
    """Test low-variance resampling."""
    print("\n=== Testing Resampler ===")
    
    for use_gpu in [False, True]:
        try:
            resampler = Resampler(use_gpu=use_gpu)
            mode = "GPU" if resampler.use_gpu else "CPU"
            
            # Create test particles with skewed weights
            N = 1000
            particles = np.random.randn(N, 3).astype(np.float32)
            weights = np.random.rand(N).astype(np.float32)
            weights[0] = 10.0  # Make first particle dominant
            weights /= weights.sum()
            
            if use_gpu and resampler.use_gpu:
                import cupy as cp
                particles = cp.asarray(particles)
                weights = cp.asarray(weights)
            
            # Resample
            t0 = time.perf_counter()
            for _ in range(100):
                new_particles, new_weights = resampler.resample(particles, weights)
            t1 = time.perf_counter()
            
            # Check ESS improved
            ess = resampler.compute_effective_sample_size(new_weights)
            assert ess > 0.9 * N, f"ESS after resample should be ~N, got {ess}"
            
            print(f"  [{mode}] Resampler: PASSED ({(t1-t0)*10:.1f}ms per 100 resamples)")
            print(f"       ESS after resample: {ess:.0f}/{N}")
            
        except ImportError:
            if use_gpu:
                print(f"  [GPU] Skipped (CuPy not available)")
    
    print("  Resampler tests passed!")


def test_particle_filter_initialization():
    """Test particle filter initialization."""
    print("\n=== Testing Particle Filter ===")
    
    # Create simple test map (10x10 grid, all free)
    map_data = np.zeros((100, 100), dtype=np.int8)  # Free space
    map_data[0, :] = 100  # Top wall
    map_data[-1, :] = 100  # Bottom wall
    map_data[:, 0] = 100  # Left wall
    map_data[:, -1] = 100  # Right wall
    
    for use_gpu in [False, True]:
        try:
            config = ParticleFilterConfig(
                num_particles=500,
                use_gpu=use_gpu
            )
            pf = ParticleFilter(config)
            mode = "GPU" if pf.use_gpu else "CPU"
            
            # Initialize
            initial_pose = (5.0, 5.0, 0.0)
            pf.initialize(
                initial_pose=initial_pose,
                map_data=map_data,
                map_resolution=0.1,
                map_origin=(0.0, 0.0, 0.0)
            )
            
            # Check initialization
            assert pf.initialized, "PF should be initialized"
            assert len(pf.particles) == 500, f"Should have 500 particles, got {len(pf.particles)}"
            
            # Check mean is near initial pose
            estimate = pf.get_estimate()
            assert abs(estimate.x - 5.0) < 1.0, f"Mean X should be ~5, got {estimate.x}"
            assert abs(estimate.y - 5.0) < 1.0, f"Mean Y should be ~5, got {estimate.y}"
            
            print(f"  [{mode}] Initialization: PASSED")
            print(f"       Initial estimate: ({estimate.x:.2f}, {estimate.y:.2f}, {estimate.theta:.2f})")
            
        except ImportError:
            if use_gpu:
                print(f"  [GPU] Skipped (CuPy not available)")
    
    print("  Particle filter initialization tests passed!")


def benchmark_particle_filter():
    """Benchmark particle filter performance."""
    print("\n=== Performance Benchmark ===")
    
    # Create test map
    map_data = np.zeros((500, 500), dtype=np.int8)
    map_data[0, :] = 100
    map_data[-1, :] = 100
    map_data[:, 0] = 100
    map_data[:, -1] = 100
    
    # Add some obstacles
    for _ in range(20):
        x, y = np.random.randint(50, 450, 2)
        map_data[y-5:y+5, x-5:x+5] = 100
    
    # Create fake laser scan
    num_beams = 270
    ranges = np.random.uniform(0.5, 10.0, num_beams).astype(np.float32)
    angle_min = -np.pi * 0.75
    angle_increment = (np.pi * 1.5) / num_beams
    
    for use_gpu in [False, True]:
        try:
            for num_particles in [500, 1000, 2000]:
                config = ParticleFilterConfig(
                    num_particles=num_particles,
                    use_gpu=use_gpu
                )
                pf = ParticleFilter(config)
                mode = "GPU" if pf.use_gpu else "CPU"
                
                pf.initialize(
                    initial_pose=(25.0, 25.0, 0.0),
                    map_data=map_data,
                    map_resolution=0.1,
                    map_origin=(0.0, 0.0, 0.0)
                )
                
                # Warmup
                for _ in range(5):
                    pf.predict((0.01, 0.0, 0.01))
                    pf.update(ranges, angle_min, angle_increment)
                
                # Benchmark
                t0 = time.perf_counter()
                num_updates = 50
                for _ in range(num_updates):
                    pf.predict((0.01, 0.0, 0.01))
                    pf.update(ranges, angle_min, angle_increment)
                t1 = time.perf_counter()
                
                avg_time = (t1 - t0) / num_updates * 1000
                timing = pf.get_timing_stats()
                
                print(f"  [{mode}] {num_particles} particles: {avg_time:.1f}ms/update")
                print(f"       Predict: {timing['predict_ms']:.1f}ms, "
                      f"Update: {timing['update_ms']:.1f}ms, "
                      f"Resample: {timing['resample_ms']:.1f}ms")
                
        except ImportError:
            if use_gpu:
                print(f"  [GPU] Skipped (CuPy not available)")


def main():
    """Run all tests."""
    print("=" * 60)
    print("GPU AMCL Unit Tests")
    print("=" * 60)
    
    test_motion_model()
    test_resampler()
    test_particle_filter_initialization()
    benchmark_particle_filter()
    
    print("\n" + "=" * 60)
    print("All tests passed!")
    print("=" * 60)


if __name__ == '__main__':
    main()
