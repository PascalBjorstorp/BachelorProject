"""
Particle Filter - Core AMCL Algorithm

This module implements the main particle filter logic for Monte Carlo Localization.
It orchestrates the predict-update-resample cycle and maintains the particle cloud.

The implementation supports both GPU (CuPy) and CPU (NumPy) backends.
"""

from typing import Optional, Tuple, List
import numpy as np
import time

# Try to import CuPy for GPU acceleration
try:
    import cupy as cp
    GPU_AVAILABLE = True
except ImportError:
    cp = None
    GPU_AVAILABLE = False

from .motion_model import MotionModel
from .sensor_model import SensorModel
from .resampling import Resampler
from ..utils.math_utils import normalize_angle


class ParticleFilterConfig:
    """Configuration for particle filter."""
    
    def __init__(
        self,
        num_particles: int = 2000,
        min_particles: int = 100,
        max_particles: int = 5000,
        use_gpu: bool = True,
        resample_threshold: float = 0.5,
        initial_cov: Tuple[float, float, float] = (0.5, 0.5, 0.2),
        # KLD Sampling (Adaptive Particle Count)
        use_kld_sampling: bool = False,
        kld_epsilon: float = 0.05,   # Maximum error bound
        kld_z: float = 2.33,         # Upper quantile (99% confidence = 2.33, 95% = 1.96)
        kld_bin_size_xy: float = 0.5,  # Spatial bin size in meters
        kld_bin_size_theta: float = 0.2,  # Angular bin size in radians
        # Recovery / kidnapped-robot detection (nav2_amcl-style)
        recovery_alpha_slow: float = 0.001,  # Slow average weight decay rate (0 = disabled)
        recovery_alpha_fast: float = 0.1,    # Fast average weight decay rate
        recovery_random_fraction_max: float = 0.1,  # Max fraction of particles to randomize
    ):
        # Particle count
        self.num_particles = num_particles
        self.min_particles = min_particles
        self.max_particles = max_particles
        
        # GPU settings
        self.use_gpu = use_gpu
        
        # Resampling: Resample when Neff < threshold * N
        self.resample_threshold = resample_threshold
        
        # Initial distribution: x, y, theta standard deviations
        self.initial_cov = initial_cov
        
        # KLD Sampling
        self.use_kld_sampling = use_kld_sampling
        self.kld_epsilon = kld_epsilon
        self.kld_z = kld_z
        self.kld_bin_size_xy = kld_bin_size_xy
        self.kld_bin_size_theta = kld_bin_size_theta

        # Recovery
        self.recovery_alpha_slow = recovery_alpha_slow
        self.recovery_alpha_fast = recovery_alpha_fast
        self.recovery_random_fraction_max = recovery_random_fraction_max

        # Validate
        self._validate()

    def _validate(self):
        """Validate configuration parameters."""
        if self.num_particles < 1:
            raise ValueError(f"num_particles must be >= 1, got {self.num_particles}")
        if self.min_particles < 1:
            raise ValueError(f"min_particles must be >= 1, got {self.min_particles}")
        if self.max_particles < self.min_particles:
            raise ValueError(
                f"max_particles ({self.max_particles}) must be >= min_particles ({self.min_particles})"
            )
        if not (0.0 < self.resample_threshold <= 1.0):
            raise ValueError(f"resample_threshold must be in (0, 1], got {self.resample_threshold}")
        if any(c < 0 for c in self.initial_cov):
            raise ValueError(f"initial_cov values must be >= 0, got {self.initial_cov}")
        if self.kld_epsilon <= 0:
            raise ValueError(f"kld_epsilon must be > 0, got {self.kld_epsilon}")
        if self.recovery_alpha_slow < 0 or self.recovery_alpha_fast < 0:
            raise ValueError("recovery_alpha_slow and recovery_alpha_fast must be >= 0")
        if not (0.0 <= self.recovery_random_fraction_max <= 1.0):
            raise ValueError(f"recovery_random_fraction_max must be in [0, 1], got {self.recovery_random_fraction_max}")


class Particle:
    """Single particle representation (for debugging/visualization)."""
    
    def __init__(self, x: float, y: float, theta: float, weight: float = 1.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = weight


class PoseEstimate:
    """Estimated pose with covariance."""
    
    def __init__(
        self,
        x: float,
        y: float,
        theta: float,
        covariance: np.ndarray,
        timestamp: float = 0.0
    ):
        self.x = x
        self.y = y
        self.theta = theta
        self.covariance = covariance  # 3x3 covariance matrix
        self.timestamp = timestamp


class ParticleFilter:
    """
    GPU-Accelerated Particle Filter for AMCL.
    
    This class maintains a set of particles representing possible robot poses
    and updates them based on odometry (motion model) and laser scans (sensor model).
    
    Attributes:
        config: ParticleFilterConfig with algorithm parameters
        particles: (N, 3) array of [x, y, theta] for each particle
        weights: (N,) array of particle weights
        motion_model: Handles prediction step
        sensor_model: Handles measurement update
        resampler: Handles particle resampling
    
    Example:
        >>> pf = ParticleFilter(config)
        >>> pf.initialize(initial_pose, map_data)
        >>> 
        >>> # Main loop
        >>> pf.predict(odom_delta)
        >>> pf.update(laser_scan)
        >>> pose = pf.get_estimate()
    """
    
    def __init__(self, config: ParticleFilterConfig = None):
        """Initialize particle filter with configuration."""
        self.config = config or ParticleFilterConfig()
        
        # Select compute backend
        self.use_gpu = self.config.use_gpu and GPU_AVAILABLE
        self.xp = cp if self.use_gpu else np  # Array library (CuPy or NumPy)
        
        if self.config.use_gpu and not GPU_AVAILABLE:
            print("[ParticleFilter] Warning: GPU requested but CuPy not available. Using CPU.")
        
        # Particle state: (N, 3) array of [x, y, theta]
        self.particles: Optional[np.ndarray] = None
        self.weights: Optional[np.ndarray] = None
        
        # Sub-modules (initialized when map is set)
        self.motion_model: Optional[MotionModel] = None
        self.sensor_model: Optional[SensorModel] = None
        self.resampler: Optional[Resampler] = None
        
        # State tracking
        self.initialized = False
        self.last_odom = None
        self.last_update_time = 0.0

        # Recovery: exponential-average weight trackers (nav2_amcl algorithm)
        self._w_slow = 0.0  # Slow-decaying average of mean weight
        self._w_fast = 0.0  # Fast-decaying average of mean weight

        # Map metadata (stored for random particle injection)
        self._map_data: Optional[np.ndarray] = None
        self._map_resolution: float = 0.0
        self._map_origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._free_cells: Optional[np.ndarray] = None  # (M, 2) free-space pixel coords

        # Performance metrics
        self.timing = {
            'predict': 0.0,
            'update': 0.0,
            'resample': 0.0,
        }
    
    def initialize(
        self,
        initial_pose: Tuple[float, float, float],
        map_data: np.ndarray,
        map_resolution: float,
        map_origin: Tuple[float, float, float],
    ) -> None:
        """
        Initialize particle filter with map and initial pose estimate.
        
        Args:
            initial_pose: (x, y, theta) initial pose estimate
            map_data: 2D occupancy grid (0=free, 100=occupied, -1=unknown)
            map_resolution: Map resolution in meters/pixel
            map_origin: (x, y, theta) of map origin in world frame
        """
        xp = self.xp
        N = self.config.num_particles
        
        # Initialize particles around initial pose with Gaussian distribution
        x0, y0, theta0 = initial_pose
        std_x, std_y, std_theta = self.config.initial_cov
        
        # Generate random particles (on GPU if available)
        if self.use_gpu:
            particles = xp.zeros((N, 3), dtype=xp.float32)
            particles[:, 0] = x0 + std_x * xp.random.randn(N, dtype=xp.float32)
            particles[:, 1] = y0 + std_y * xp.random.randn(N, dtype=xp.float32)
            particles[:, 2] = theta0 + std_theta * xp.random.randn(N, dtype=xp.float32)
        else:
            particles = np.zeros((N, 3), dtype=np.float32)
            particles[:, 0] = x0 + std_x * np.random.randn(N).astype(np.float32)
            particles[:, 1] = y0 + std_y * np.random.randn(N).astype(np.float32)
            particles[:, 2] = theta0 + std_theta * np.random.randn(N).astype(np.float32)
        
        # Normalize angles to [-pi, pi]
        particles[:, 2] = self._normalize_angles(particles[:, 2])
        
        # Uniform initial weights
        weights = xp.ones(N, dtype=xp.float32) / N
        
        self.particles = particles
        self.weights = weights
        
        # Initialize sub-modules
        self.motion_model = MotionModel(use_gpu=self.use_gpu)
        self.sensor_model = SensorModel(
            map_data=map_data,
            resolution=map_resolution,
            origin=map_origin,
            use_gpu=self.use_gpu
        )
        self.resampler = Resampler(use_gpu=self.use_gpu)
        
        # Store map metadata for recovery (random particle injection)
        self._map_data = map_data
        self._map_resolution = map_resolution
        self._map_origin = map_origin
        # Pre-compute free-space cell coordinates (row, col)
        self._free_cells = np.argwhere(map_data == 0)  # (M, 2) of [row, col]
        
        # Reset recovery trackers
        self._w_slow = 0.0
        self._w_fast = 0.0
        
        self.initialized = True
        print(f"[ParticleFilter] Initialized with {N} particles on {'GPU' if self.use_gpu else 'CPU'}")
    
    def predict(self, odom_delta: Tuple[float, float, float], imu_dtheta: Optional[float] = None) -> None:
        """
        Prediction step: propagate particles through motion model.
        
        Args:
            odom_delta: (dx, dy, dtheta) odometry change in robot frame
            imu_dtheta: Optional rotation from IMU gyroscope for better accuracy at high speed
        """
        if not self.initialized:
            raise RuntimeError("ParticleFilter not initialized. Call initialize() first.")
        
        t0 = time.perf_counter()
        
        # Apply motion model to all particles (GPU parallel)
        self.particles = self.motion_model.apply(self.particles, odom_delta, imu_dtheta=imu_dtheta)
        
        # Normalize angles
        self.particles[:, 2] = self._normalize_angles(self.particles[:, 2])
        
        self.timing['predict'] = time.perf_counter() - t0
    
    def update(self, ranges: np.ndarray, angle_min: float, angle_increment: float) -> None:
        """
        Update step: compute particle weights from sensor model.
        
        Args:
            ranges: Laser scan ranges (N_beams,)
            angle_min: Minimum scan angle (radians)
            angle_increment: Angle between consecutive beams (radians)
        """
        if not self.initialized:
            raise RuntimeError("ParticleFilter not initialized. Call initialize() first.")
        
        t0 = time.perf_counter()
        
        # Transfer scan to GPU if needed
        xp = self.xp
        if self.use_gpu:
            ranges_gpu = cp.asarray(ranges, dtype=cp.float32)
        else:
            ranges_gpu = np.asarray(ranges, dtype=np.float32)
        
        # Compute weights for all particles (GPU parallel)
        new_weights = self.sensor_model.compute_weights(
            self.particles,
            ranges_gpu,
            angle_min,
            angle_increment
        )
        
        # Update weights (multiply with previous, then normalize)
        self.weights = self.weights * new_weights
        
        # Normalize weights
        weight_sum = xp.sum(self.weights)
        if weight_sum > 0:
            self.weights = self.weights / weight_sum
        else:
            # All weights zero - particle deprivation, reinitialize uniformly
            print("[ParticleFilter] Warning: All weights zero, reinitializing weights")
            self.weights = xp.ones_like(self.weights) / len(self.weights)
        
        # Update recovery trackers (nav2_amcl-style exponential averages)
        # w_avg is the mean unnormalized weight this iteration
        N = len(self.weights)
        w_avg = float(weight_sum.get() if self.use_gpu else weight_sum) / N
        alpha_slow = self.config.recovery_alpha_slow
        alpha_fast = self.config.recovery_alpha_fast
        if alpha_slow > 0.0:
            if self._w_slow == 0.0:
                self._w_slow = w_avg
            else:
                self._w_slow += alpha_slow * (w_avg - self._w_slow)
            if self._w_fast == 0.0:
                self._w_fast = w_avg
            else:
                self._w_fast += alpha_fast * (w_avg - self._w_fast)
        
        self.timing['update'] = time.perf_counter() - t0
        
        # Check if resampling is needed
        self._check_resample()
    
    def _check_resample(self) -> None:
        """Check effective particle count and resample if needed."""
        xp = self.xp
        
        # Compute effective sample size: Neff = 1 / sum(w^2)
        weights_squared = self.weights ** 2
        n_eff = 1.0 / xp.sum(weights_squared)
        
        # Convert to Python float if on GPU
        if self.use_gpu:
            n_eff = float(n_eff.get())
        
        threshold = self.config.resample_threshold * len(self.weights)
        
        if n_eff < threshold:
            self._resample()
    
    def _resample(self) -> None:
        """Resample particles using low-variance resampling with optional KLD and recovery."""
        t0 = time.perf_counter()
        
        if self.config.use_kld_sampling:
            # KLD sampling: determine number of particles based on histogram bins
            target_n = self._compute_kld_particle_count()
        else:
            # Fixed particle count
            target_n = len(self.particles)
        
        self.particles, self.weights = self.resampler.resample(
            self.particles,
            self.weights,
            target_n=target_n
        )
        
        # Recovery: inject random particles in free space when filter may be lost
        # Uses nav2_amcl algorithm: when w_fast diverges below w_slow, the filter
        # is performing worse than its long-term average → inject random particles.
        if (self.config.recovery_alpha_slow > 0.0
                and self._w_slow > 0.0
                and self._free_cells is not None
                and len(self._free_cells) > 0):
            ratio = 1.0 - (self._w_fast / self._w_slow)
            random_fraction = max(0.0, min(ratio, self.config.recovery_random_fraction_max))
            if random_fraction > 0.01:  # Only inject if meaningful
                self._inject_random_particles(random_fraction)
                print(f"[ParticleFilter] Recovery: injected {random_fraction*100:.1f}% random particles")
        
        self.timing['resample'] = time.perf_counter() - t0
    
    def _inject_random_particles(self, fraction: float) -> None:
        """
        Replace a fraction of particles with random poses in free space.
        
        Args:
            fraction: Fraction of particles to replace (0.0 to 1.0)
        """
        xp = self.xp
        N = len(self.particles)
        n_random = max(1, int(fraction * N))
        
        # Sample random free-space cells
        indices = np.random.choice(len(self._free_cells), size=n_random, replace=True)
        cells = self._free_cells[indices]  # (n_random, 2) of [row, col]
        
        # Convert pixel coords to world coords
        ox, oy, _ = self._map_origin
        world_x = ox + (cells[:, 1].astype(np.float32) + 0.5) * self._map_resolution
        world_y = oy + (cells[:, 0].astype(np.float32) + 0.5) * self._map_resolution
        world_theta = np.random.uniform(-np.pi, np.pi, n_random).astype(np.float32)
        
        # Build random particle array
        random_particles = np.stack([world_x, world_y, world_theta], axis=1)
        
        if self.use_gpu:
            random_particles = cp.asarray(random_particles)
        
        # Replace the lowest-weight particles
        self.particles[-n_random:] = random_particles
        
        # Re-normalize weights to uniform
        self.weights = xp.ones(N, dtype=xp.float32) / N
    
    def _compute_kld_particle_count(self) -> int:
        """
        Compute required particle count using KLD sampling.
        
        Based on the number of occupied histogram bins (k), compute the minimum
        number of particles needed to ensure the KL-divergence between the
        true and sample distributions is below epsilon with probability 1-delta.
        
        Formula from Fox et al. "Adapting the Sample Size in Particle Filters":
            n = (k-1) / (2*epsilon) * (1 - 2/(9*(k-1)) + sqrt(2/(9*(k-1))) * z)^3
        
        Returns:
            Target number of particles bounded by [min_particles, max_particles]
        """
        xp = self.xp
        
        # Get particles on CPU for binning
        if self.use_gpu:
            particles_cpu = self.particles.get()
        else:
            particles_cpu = self.particles
        
        # Bin sizes
        bin_xy = self.config.kld_bin_size_xy
        bin_theta = self.config.kld_bin_size_theta
        
        # Compute bin indices for each particle
        bin_x = np.floor(particles_cpu[:, 0] / bin_xy).astype(np.int32)
        bin_y = np.floor(particles_cpu[:, 1] / bin_xy).astype(np.int32)
        bin_theta = np.floor(particles_cpu[:, 2] / bin_theta).astype(np.int32)
        
        # Count unique bins (occupied histogram bins)
        bins = np.stack([bin_x, bin_y, bin_theta], axis=1)
        unique_bins = np.unique(bins, axis=0)
        k = len(unique_bins)
        
        # KLD formula
        if k <= 1:
            return self.config.min_particles
        
        epsilon = self.config.kld_epsilon
        z = self.config.kld_z
        
        # Wilson-Hilferty approximation
        term = 1.0 - 2.0 / (9.0 * (k - 1)) + np.sqrt(2.0 / (9.0 * (k - 1))) * z
        n = int((k - 1) / (2.0 * epsilon) * (term ** 3))
        
        # Bound by min/max
        n = max(self.config.min_particles, min(n, self.config.max_particles))
        
        return n
    
    def get_estimate(self) -> PoseEstimate:
        """
        Compute weighted mean pose estimate from particle cloud.
        
        Returns:
            PoseEstimate with mean pose and covariance
        """
        xp = self.xp
        
        # Weighted mean for x, y
        mean_x = float(xp.sum(self.weights * self.particles[:, 0]))
        mean_y = float(xp.sum(self.weights * self.particles[:, 1]))
        
        # Circular mean for theta (handles wraparound)
        sin_sum = float(xp.sum(self.weights * xp.sin(self.particles[:, 2])))
        cos_sum = float(xp.sum(self.weights * xp.cos(self.particles[:, 2])))
        mean_theta = float(np.arctan2(sin_sum, cos_sum))
        
        # Compute covariance (vectorized, GPU-accelerated when available)
        # Center particles on the current backend (GPU or CPU)
        centered = self.particles.copy()
        centered[:, 0] -= mean_x
        centered[:, 1] -= mean_y
        centered[:, 2] = self._normalize_angles(centered[:, 2] - mean_theta)
        
        # Weighted covariance: (centered.T * weights) @ centered
        # weighted shape: (N, 3) with each row scaled by its weight
        weighted = centered * self.weights[:, None]  # broadcast (N,) -> (N, 3)
        cov = xp.dot(weighted.T, centered)  # (3, 3)
        
        # Transfer to CPU if on GPU
        if self.use_gpu:
            cov = cov.get()
        cov = np.asarray(cov, dtype=np.float32)
        
        return PoseEstimate(
            x=mean_x,
            y=mean_y,
            theta=mean_theta,
            covariance=cov,
            timestamp=time.time()
        )
    
    def get_particles_numpy(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get particles as NumPy arrays (transfers from GPU if needed)."""
        if self.use_gpu:
            return self.particles.get(), self.weights.get()
        return self.particles, self.weights
    
    def _normalize_angles(self, angles):
        """Normalize angles to [-pi, pi] (works with CuPy or NumPy)."""
        xp = self.xp
        return xp.arctan2(xp.sin(angles), xp.cos(angles))
    
    def _normalize_angles_cpu(self, angles: np.ndarray) -> np.ndarray:
        """Normalize angles to [-pi, pi] (NumPy only)."""
        return np.arctan2(np.sin(angles), np.cos(angles))
    
    def get_timing_stats(self) -> dict:
        """Get timing statistics for performance analysis."""
        total = sum(self.timing.values())
        return {
            'predict_ms': self.timing['predict'] * 1000,
            'update_ms': self.timing['update'] * 1000,
            'resample_ms': self.timing['resample'] * 1000,
            'total_ms': total * 1000,
        }
