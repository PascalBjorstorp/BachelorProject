"""
Sensor Model - Likelihood Field for Laser Scans

This module implements the likelihood field sensor model from Probabilistic Robotics.
It computes how well each particle's pose explains the observed laser scan.

The GPU implementation computes all particle weights in parallel, which is the
primary performance bottleneck in AMCL (O(N_particles × N_beams)).

Key optimization: Pre-compute distance transform of obstacles for O(1) lookups.

Reference:
    Probabilistic Robotics, Chapter 6.4 - Likelihood Fields for Range Finders
"""

from typing import Tuple, Optional
import numpy as np

# Try to import CuPy for GPU acceleration
try:
    import cupy as cp
    GPU_AVAILABLE = True
except ImportError:
    cp = None
    GPU_AVAILABLE = False

# Try to import scipy for distance transform (CPU fallback)
try:
    from scipy.ndimage import distance_transform_edt
    SCIPY_AVAILABLE = True
except ImportError:
    SCIPY_AVAILABLE = False


class SensorModelConfig:
    """
    Likelihood field sensor model parameters.
    
    The sensor model mixes multiple probability distributions:
        - z_hit: Correct reading (Gaussian centered at true distance)
        - z_short: Unexpected short readings (reflections, etc.)
        - z_max: Max-range readings (no return)
        - z_rand: Random readings (noise)
    
    These should sum to 1.0.
    """
    
    def __init__(
        self,
        # Mixing weights (must sum to 1.0)
        z_hit: float = 0.95,      # Weight for hit probability
        z_short: float = 0.0,     # Weight for short readings
        z_max: float = 0.025,     # Weight for max-range readings
        z_rand: float = 0.025,    # Weight for random readings
        # Hit model parameters
        sigma_hit: float = 0.2,   # Std dev for Gaussian hit model (meters)
        # Sensor parameters
        max_beams: int = 60,      # Number of beams to use (subsampled from full scan)
        max_range: float = 10.0,  # Maximum valid range (meters)
        # Likelihood field parameters
        max_dist: float = 2.0,    # Max distance in likelihood field lookup (meters)
        # Laser offset from base_link (set to match your robot)
        laser_offset_x: float = 0.265,  # Forward offset (meters)
        laser_offset_y: float = 0.0     # Lateral offset (meters)
    ):
        self.z_hit = z_hit
        self.z_short = z_short
        self.z_max = z_max
        self.z_rand = z_rand
        self.sigma_hit = sigma_hit
        self.max_beams = max_beams
        self.max_range = max_range
        self.max_dist = max_dist
        self.laser_offset_x = laser_offset_x
        self.laser_offset_y = laser_offset_y

        # Validate mixing weights
        weight_sum = z_hit + z_short + z_max + z_rand
        if abs(weight_sum - 1.0) > 0.01:
            raise ValueError(
                f"Sensor model mixing weights must sum to 1.0, got "
                f"z_hit={z_hit} + z_short={z_short} + z_max={z_max} + z_rand={z_rand} = {weight_sum}"
            )
        if sigma_hit <= 0:
            raise ValueError(f"sigma_hit must be > 0, got {sigma_hit}")
        if max_beams < 1:
            raise ValueError(f"max_beams must be >= 1, got {max_beams}")
        if max_range <= 0:
            raise ValueError(f"max_range must be > 0, got {max_range}")


class SensorModel:
    """
    GPU-accelerated Likelihood Field Sensor Model.
    
    Computes particle weights based on how well each particle's pose explains
    the observed laser scan. Uses a pre-computed distance transform for fast
    lookup of distance-to-nearest-obstacle.
    
    Algorithm:
        1. For each particle, transform laser beams to map frame
        2. Look up distance to nearest obstacle at each beam endpoint
        3. Compute probability using Gaussian likelihood
        4. Multiply probabilities across beams
    
    GPU Parallelization:
        - All particles computed in parallel
        - Within each particle, all beams computed in parallel
        - Memory-bound by distance field texture lookups
    
    Example:
        >>> sensor = SensorModel(map_data, resolution, origin, use_gpu=True)
        >>> weights = sensor.compute_weights(particles, ranges, angle_min, angle_inc)
    """
    
    def __init__(
        self,
        map_data: np.ndarray,
        resolution: float,
        origin: Tuple[float, float, float],
        config: SensorModelConfig = None,
        use_gpu: bool = True
    ):
        """
        Initialize sensor model with map.
        
        Args:
            map_data: 2D occupancy grid (0=free, 100=occupied, -1=unknown)
            resolution: Map resolution in meters/pixel
            origin: (x, y, theta) of map origin in world frame
            config: Sensor model parameters
            use_gpu: Whether to use GPU acceleration
        """
        self.config = config or SensorModelConfig()
        self.use_gpu = use_gpu and GPU_AVAILABLE
        self.xp = cp if self.use_gpu else np
        
        self.resolution = resolution
        self.origin = origin  # (x, y, theta)
        
        # Pre-compute distance transform (expensive but done once)
        self._build_distance_field(map_data)
        
        # Pre-compute Gaussian normalization constant
        self._precompute_gaussian()
    
    def _build_distance_field(self, map_data: np.ndarray) -> None:
        """
        Build distance transform of obstacle map.
        
        The distance field stores the distance to the nearest obstacle
        at every cell. This allows O(1) lookup during weight computation.
        """
        # Create binary obstacle map (1 = free, 0 = occupied)
        # Treat unknown (-1) as free for distance computation
        free_space = (map_data == 0) | (map_data == -1)
        
        if SCIPY_AVAILABLE:
            # Compute Euclidean distance transform (in pixels)
            dist_pixels = distance_transform_edt(free_space)
        else:
            # Fallback: Chebyshev distance (less accurate but no scipy needed)
            dist_pixels = self._simple_distance_transform(free_space)
        
        # Convert to meters
        self.distance_field = dist_pixels.astype(np.float32) * self.resolution
        
        # Cap at max_dist for memory efficiency
        self.distance_field = np.clip(self.distance_field, 0, self.config.max_dist)
        
        # Store map dimensions
        self.map_height, self.map_width = self.distance_field.shape
        
        # Transfer to GPU if available
        if self.use_gpu:
            self.distance_field_gpu = cp.asarray(self.distance_field)
        else:
            self.distance_field_gpu = self.distance_field
        
        print(f"[SensorModel] Built distance field: {self.map_width}x{self.map_height}")
    
    def _simple_distance_transform(self, free_space: np.ndarray) -> np.ndarray:
        """
        Approximate Euclidean distance transform without scipy.
        
        Uses 8-connected BFS with proper diagonal distance (sqrt(2) ≈ 1.414)
        for a closer approximation to the true Euclidean distance transform.
        """
        from collections import deque
        
        SQRT2 = np.float32(1.4142135)
        
        h, w = free_space.shape
        dist = np.full((h, w), np.inf, dtype=np.float32)
        
        # Initialize obstacles to 0
        obstacles = np.where(~free_space)
        for y, x in zip(obstacles[0], obstacles[1]):
            dist[y, x] = 0
        
        # 8-connected BFS from obstacles (cardinal + diagonal)
        # Diagonal moves cost sqrt(2), cardinal moves cost 1
        queue = deque()
        for y, x in zip(obstacles[0], obstacles[1]):
            queue.append((y, x))
        
        # (dy, dx, cost) for 8-connected neighborhood
        neighbors = [
            (-1,  0, 1.0),     ( 1,  0, 1.0),
            ( 0, -1, 1.0),     ( 0,  1, 1.0),
            (-1, -1, SQRT2),   (-1,  1, SQRT2),
            ( 1, -1, SQRT2),   ( 1,  1, SQRT2),
        ]
        
        while queue:
            y, x = queue.popleft()
            for dy, dx, cost in neighbors:
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w:
                    new_dist = dist[y, x] + cost
                    if new_dist < dist[ny, nx]:
                        dist[ny, nx] = new_dist
                        queue.append((ny, nx))
        
        return dist
    
    def _precompute_gaussian(self) -> None:
        """Pre-compute Gaussian normalization for likelihood."""
        sigma = self.config.sigma_hit
        self.gaussian_norm = 1.0 / (np.sqrt(2 * np.pi) * sigma)
        self.gaussian_coeff = -0.5 / (sigma ** 2)
    
    def compute_weights(
        self,
        particles: np.ndarray,
        ranges: np.ndarray,
        angle_min: float,
        angle_increment: float
    ) -> np.ndarray:
        """
        Compute particle weights from laser scan.
        
        This is the most compute-intensive function and is fully GPU-parallelized.
        
        Args:
            particles: (N, 3) array of [x, y, theta] particle poses
            ranges: Laser scan ranges (M,) - will be subsampled
            angle_min: Minimum scan angle (radians)
            angle_increment: Angle between consecutive beams (radians)
        
        Returns:
            (N,) array of unnormalized particle weights
        """
        xp = self.xp
        N = len(particles)
        
        # Subsample beams if needed
        num_beams = len(ranges)
        if num_beams > self.config.max_beams:
            step = num_beams // self.config.max_beams
            beam_indices = xp.arange(0, num_beams, step, dtype=xp.int32)[:self.config.max_beams]
        else:
            beam_indices = xp.arange(num_beams, dtype=xp.int32)
        
        M = len(beam_indices)  # Number of beams to use
        
        # Get selected beam angles and ranges
        if self.use_gpu:
            # Ensure ranges is on GPU
            if not isinstance(ranges, cp.ndarray):
                ranges = cp.asarray(ranges, dtype=cp.float32)
            beam_indices_cpu = beam_indices.get()
        else:
            beam_indices_cpu = beam_indices
        
        angles = angle_min + beam_indices.astype(xp.float32) * angle_increment
        selected_ranges = ranges[beam_indices]
        
        # Filter out invalid ranges
        valid_mask = (selected_ranges > 0.1) & (selected_ranges < self.config.max_range)
        
        # Get particle poses
        px = particles[:, 0]  # (N,)
        py = particles[:, 1]  # (N,)
        ptheta = particles[:, 2]  # (N,)
        
        # Laser offset in robot frame
        laser_x = self.config.laser_offset_x
        laser_y = self.config.laser_offset_y
        
        # Transform laser origin to world frame for each particle
        cos_theta = xp.cos(ptheta)
        sin_theta = xp.sin(ptheta)
        laser_world_x = px + laser_x * cos_theta - laser_y * sin_theta  # (N,)
        laser_world_y = py + laser_x * sin_theta + laser_y * cos_theta  # (N,)
        
        # Compute beam endpoints for all particles and beams
        # Shape: (N, M) for each coordinate
        beam_angles_world = ptheta[:, None] + angles[None, :]  # (N, M)
        
        # Beam endpoints in world frame
        endpoints_x = laser_world_x[:, None] + selected_ranges[None, :] * xp.cos(beam_angles_world)
        endpoints_y = laser_world_y[:, None] + selected_ranges[None, :] * xp.sin(beam_angles_world)
        
        # Convert endpoints to map coordinates
        ox, oy, _ = self.origin
        map_x = ((endpoints_x - ox) / self.resolution).astype(xp.int32)
        map_y = ((endpoints_y - oy) / self.resolution).astype(xp.int32)
        
        # Clamp to map bounds
        map_x = xp.clip(map_x, 0, self.map_width - 1)
        map_y = xp.clip(map_y, 0, self.map_height - 1)
        
        # Look up distances in distance field
        # This is the key GPU parallel operation
        if self.use_gpu:
            # Flatten indices for GPU lookup
            flat_indices = map_y * self.map_width + map_x
            distances = self.distance_field_gpu.ravel()[flat_indices.ravel()].reshape(N, M)
        else:
            distances = self.distance_field_gpu[map_y, map_x]
        
        # Compute Gaussian likelihood for each beam
        # p(z | x) = z_hit * N(d, 0, sigma) + z_short * p_short(z) + z_max * p_max(z) + z_rand / max_range
        gaussian_probs = self.gaussian_norm * xp.exp(self.gaussian_coeff * distances ** 2)
        
        # Mixing weights
        z_hit = self.config.z_hit
        z_short = self.config.z_short
        z_max = self.config.z_max
        z_rand = self.config.z_rand
        max_range = self.config.max_range
        
        beam_probs = z_hit * gaussian_probs + z_rand / max_range
        
        # Short reading model: exponential distribution p(z) = lambda * exp(-lambda * z)
        # where lambda = 1 / expected_range.  We approximate expected_range with the
        # distance-field value (distance to nearest obstacle along the beam direction).
        # For beams where distance ≈ 0 (right at a wall) we skip to avoid division by zero.
        if z_short > 0.0:
            # Use distances as proxy for expected range; clamp away from 0
            expected_range = xp.clip(distances, 0.1, max_range)
            # lambda = 1 / expected_range; normaliser so integral 0..expected = 1
            eta = 1.0 / (1.0 - xp.exp(-1.0))  # normalisation for unit exponential
            p_short = eta / expected_range * xp.exp(-selected_ranges[None, :] / expected_range)
            # Only apply to beams shorter than the expected range
            p_short = xp.where(selected_ranges[None, :] <= expected_range, p_short, xp.zeros_like(p_short))
            beam_probs = beam_probs + z_short * p_short
        
        # Max-range readings: small probability spike at max range
        if z_max > 0.0:
            near_max = (selected_ranges[None, :] > (max_range - 0.1))
            beam_probs = beam_probs + z_max * xp.where(near_max, xp.ones_like(beam_probs), xp.zeros_like(beam_probs))
        
        # Apply valid mask (invalid beams don't contribute)
        beam_probs = xp.where(valid_mask[None, :], beam_probs, xp.ones_like(beam_probs))
        
        # Multiply probabilities across beams (product over M dimension)
        # Use log-sum for numerical stability
        log_probs = xp.log(beam_probs + 1e-10)
        log_weights = xp.sum(log_probs, axis=1)
        
        # Convert back to probability
        weights = xp.exp(log_weights - xp.max(log_weights))  # Normalize for stability
        
        return weights
    
    def world_to_map(self, x: float, y: float) -> Tuple[int, int]:
        """Convert world coordinates to map pixel coordinates."""
        ox, oy, _ = self.origin
        mx = int((x - ox) / self.resolution)
        my = int((y - oy) / self.resolution)
        return mx, my
    
    def is_in_map(self, mx: int, my: int) -> bool:
        """Check if map coordinates are within bounds."""
        return 0 <= mx < self.map_width and 0 <= my < self.map_height
    
    def get_distance_at(self, x: float, y: float) -> float:
        """Get distance to nearest obstacle at world coordinates."""
        mx, my = self.world_to_map(x, y)
        if not self.is_in_map(mx, my):
            return 0.0  # Outside map = obstacle
        return float(self.distance_field[my, mx])
