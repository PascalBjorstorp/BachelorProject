"""
Resampling - Particle Selection Strategies

This module implements particle resampling algorithms for the particle filter.
Resampling is necessary to prevent particle deprivation (all weight on few particles).

Implements low-variance resampling which produces more diverse particle sets
than naive multinomial resampling.

Reference:
    Probabilistic Robotics, Chapter 4.3.4 - Low Variance Sampler
"""

from typing import Tuple
import numpy as np

# Try to import CuPy for GPU acceleration
try:
    import cupy as cp
    GPU_AVAILABLE = True
except ImportError:
    cp = None
    GPU_AVAILABLE = False


class Resampler:
    """
    GPU-accelerated Particle Resampler.
    
    Implements low-variance resampling (also called systematic resampling).
    This method produces less variance in the resampled population compared
    to standard multinomial resampling.
    
    Algorithm:
        1. Draw single random number r in [0, 1/N)
        2. Select particle i where cumsum(weights)[i] >= r + n/N
        3. This "walks" through the cumulative distribution evenly
    
    Example:
        >>> resampler = Resampler(use_gpu=True)
        >>> new_particles, new_weights = resampler.resample(particles, weights)
    """
    
    def __init__(self, use_gpu: bool = True):
        """
        Initialize resampler.
        
        Args:
            use_gpu: Whether to use GPU acceleration
        """
        self.use_gpu = use_gpu and GPU_AVAILABLE
        self.xp = cp if self.use_gpu else np
    
    def resample(
        self,
        particles: np.ndarray,
        weights: np.ndarray,
        target_n: int = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Resample particles using low-variance resampling.
        
        Args:
            particles: (N, 3) array of [x, y, theta] particle poses
            weights: (N,) array of normalized weights (sum to 1)
            target_n: Target number of particles after resampling (for KLD).
                      If None, keeps the same number of particles.
        
        Returns:
            Tuple of:
                - new_particles: (target_n, 3) resampled particles
                - new_weights: (target_n,) uniform weights (1/target_n each)
        """
        xp = self.xp
        N = len(particles)
        M = target_n if target_n is not None else N  # Output particle count
        
        # Ensure weights are normalized
        weight_sum = xp.sum(weights)
        if weight_sum > 0:
            weights = weights / weight_sum
        else:
            weights = xp.ones(N, dtype=xp.float32) / N
        
        # Compute cumulative sum of weights
        cumsum = xp.cumsum(weights)
        
        # Low-variance resampling
        # Generate evenly-spaced samples: r, r + 1/M, r + 2/M, ...
        if self.use_gpu:
            r = xp.random.uniform(0, 1.0 / M, dtype=xp.float32)
        else:
            r = np.float32(np.random.uniform(0, 1.0 / M))
        
        positions = r + xp.arange(M, dtype=xp.float32) / M
        
        # Find indices using searchsorted
        # indices[i] = smallest j such that cumsum[j] >= positions[i]
        if self.use_gpu:
            # CuPy searchsorted
            indices = xp.searchsorted(cumsum, positions, side='left')
        else:
            indices = np.searchsorted(cumsum, positions, side='left')
        
        # Clamp indices to valid range (source particles)
        indices = xp.clip(indices, 0, N - 1)
        
        # Select particles at these indices
        new_particles = particles[indices].copy()
        
        # Reset to uniform weights
        new_weights = xp.ones(M, dtype=xp.float32) / M
        
        return new_particles, new_weights
    
    def compute_effective_sample_size(self, weights: np.ndarray) -> float:
        """
        Compute effective sample size (ESS) from weights.
        
        ESS = 1 / sum(w_i^2)
        
        When all weights are equal: ESS = N
        When one weight dominates: ESS approaches 1
        
        Args:
            weights: (N,) array of normalized weights
        
        Returns:
            Effective sample size (float)
        """
        xp = self.xp
        weights_squared = weights ** 2
        ess = 1.0 / xp.sum(weights_squared)
        
        if self.use_gpu:
            return float(ess.get())
        return float(ess)
    
    def multinomial_resample(
        self,
        particles: np.ndarray,
        weights: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Alternative: Multinomial resampling.
        
        Simpler but higher variance than low-variance resampling.
        Included for comparison/testing.
        """
        xp = self.xp
        N = len(particles)
        
        # Normalize weights
        weights = weights / xp.sum(weights)
        
        # Sample indices with probability proportional to weights
        if self.use_gpu:
            # CuPy multinomial sampling
            indices = xp.random.choice(N, size=N, replace=True, p=weights)
        else:
            indices = np.random.choice(N, size=N, replace=True, p=weights)
        
        new_particles = particles[indices].copy()
        new_weights = xp.ones(N, dtype=xp.float32) / N
        
        return new_particles, new_weights
