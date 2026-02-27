"""
Motion Model - Odometry-based Particle Propagation with Optional IMU Fusion

This module implements the odometry motion model from Probabilistic Robotics (Thrun et al.).
It propagates particles based on odometry changes with added noise to model uncertainty.

Optional IMU fusion uses gyroscope data for more accurate rotation estimates,
which is especially useful at high speeds where wheel slip causes odometry drift.

The GPU implementation applies noise to all particles in parallel.

Reference:
    Probabilistic Robotics, Chapter 5.4 - Odometry Motion Model
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


class MotionModelConfig:
    """
    Odometry motion model noise parameters.
    
    These alpha parameters control how much noise is added during motion.
    Higher values = more uncertainty.
    
    From Probabilistic Robotics (Thrun et al.):
        alpha1: Rotation noise from rotation
        alpha2: Rotation noise from translation
        alpha3: Translation noise from translation
        alpha4: Translation noise from rotation
    """
    
    def __init__(
        self,
        alpha1: float = 0.2,
        alpha2: float = 0.2,
        alpha3: float = 0.2,
        alpha4: float = 0.2,
        use_imu_rotation: bool = False,
        imu_gyro_weight: float = 0.8  # How much to trust IMU vs odometry for rotation
    ):
        self.alpha1 = alpha1  # Rotation noise from rotation
        self.alpha2 = alpha2  # Rotation noise from translation
        self.alpha3 = alpha3  # Translation noise from translation
        self.alpha4 = alpha4  # Translation noise from rotation
        self.use_imu_rotation = use_imu_rotation
        self.imu_gyro_weight = imu_gyro_weight

        # Validate
        for name, val in [('alpha1', alpha1), ('alpha2', alpha2),
                          ('alpha3', alpha3), ('alpha4', alpha4)]:
            if val < 0:
                raise ValueError(f"{name} must be >= 0, got {val}")
        if not (0.0 <= imu_gyro_weight <= 1.0):
            raise ValueError(f"imu_gyro_weight must be in [0, 1], got {imu_gyro_weight}")


class MotionModel:
    """
    GPU-accelerated Odometry Motion Model.
    
    Implements the sample_motion_model_odometry algorithm from Probabilistic Robotics.
    Propagates all particles through odometry change with Gaussian noise.
    
    The noise is modeled as:
        - Rotation is noisy based on how much we rotated AND how far we moved
        - Translation is noisy based on how far we moved AND how much we rotated
    
    Example:
        >>> motion = MotionModel(use_gpu=True)
        >>> odom_delta = (0.1, 0.0, 0.05)  # dx, dy, dtheta
        >>> particles_new = motion.apply(particles, odom_delta)
    """
    
    def __init__(self, config: MotionModelConfig = None, use_gpu: bool = True):
        """
        Initialize motion model.
        
        Args:
            config: Motion model noise parameters
            use_gpu: Whether to use GPU acceleration
        """
        self.config = config or MotionModelConfig()
        self.use_gpu = use_gpu and GPU_AVAILABLE
        self.xp = cp if self.use_gpu else np
    
    def apply(
        self,
        particles: np.ndarray,
        odom_delta: Tuple[float, float, float],
        imu_dtheta: Optional[float] = None
    ) -> np.ndarray:
        """
        Apply motion model to all particles.
        
        This implements the sample_motion_model_odometry algorithm.
        Each particle is moved by the odometry delta plus sampled noise.
        
        If IMU data is provided and enabled, rotation estimates are fused
        with gyroscope data for better accuracy at high speeds.
        
        Args:
            particles: (N, 3) array of [x, y, theta] particle poses
            odom_delta: (dx, dy, dtheta) odometry change in robot frame
                       dx: forward motion
                       dy: lateral motion (usually 0 for differential drive)
                       dtheta: rotation (from odometry)
            imu_dtheta: Rotation change from IMU gyroscope (optional).
                        If provided and use_imu_rotation is True, this is
                        blended with odom dtheta using imu_gyro_weight.
        
        Returns:
            Updated (N, 3) particle array
        """
        xp = self.xp
        N = len(particles)
        
        dx, dy, dtheta = odom_delta
        
        # Fuse IMU rotation if available and enabled
        if imu_dtheta is not None and self.config.use_imu_rotation:
            # Weighted average: trust IMU more for rotation (less wheel slip)
            w = self.config.imu_gyro_weight
            dtheta = w * imu_dtheta + (1.0 - w) * dtheta
        
        # Compute translation magnitude
        trans = float(xp.sqrt(dx * dx + dy * dy))
        
        # Skip if no motion
        if trans < 1e-6 and abs(dtheta) < 1e-6:
            return particles
        
        # Sample noise for each particle (GPU parallel)
        alpha1, alpha2, alpha3, alpha4 = (
            self.config.alpha1, self.config.alpha2,
            self.config.alpha3, self.config.alpha4
        )
        
        # Noise standard deviations
        rot_noise_std = xp.sqrt(alpha1 * dtheta**2 + alpha2 * trans**2)
        trans_noise_std = xp.sqrt(alpha3 * trans**2 + alpha4 * dtheta**2)
        
        # Make sure noise isn't zero (add small minimum)
        rot_noise_std = max(float(rot_noise_std), 0.001)
        trans_noise_std = max(float(trans_noise_std), 0.001)
        
        # Generate random noise for all particles (N samples each)
        if self.use_gpu:
            rot_noise = rot_noise_std * xp.random.randn(N, dtype=xp.float32)
            trans_x_noise = trans_noise_std * xp.random.randn(N, dtype=xp.float32)
            trans_y_noise = trans_noise_std * xp.random.randn(N, dtype=xp.float32) * 0.1  # Less lateral noise
            rot2_noise = rot_noise_std * xp.random.randn(N, dtype=xp.float32)
        else:
            rot_noise = rot_noise_std * np.random.randn(N).astype(np.float32)
            trans_x_noise = trans_noise_std * np.random.randn(N).astype(np.float32)
            trans_y_noise = trans_noise_std * np.random.randn(N).astype(np.float32) * 0.1
            rot2_noise = rot_noise_std * np.random.randn(N).astype(np.float32)
        
        # Apply motion in robot frame, then transform to world frame
        # This is the key GPU-parallel operation
        
        theta = particles[:, 2]
        
        # Noisy odometry in robot frame
        noisy_dx = dx + trans_x_noise
        noisy_dy = dy + trans_y_noise
        noisy_dtheta = dtheta + rot_noise + rot2_noise
        
        # Transform to world frame and update poses
        cos_theta = xp.cos(theta)
        sin_theta = xp.sin(theta)
        
        # New positions (parallel broadcast)
        new_x = particles[:, 0] + noisy_dx * cos_theta - noisy_dy * sin_theta
        new_y = particles[:, 1] + noisy_dx * sin_theta + noisy_dy * cos_theta
        new_theta = theta + noisy_dtheta
        
        # Stack into new particle array
        if self.use_gpu:
            new_particles = xp.stack([new_x, new_y, new_theta], axis=1)
        else:
            new_particles = np.stack([new_x, new_y, new_theta], axis=1)
        
        return new_particles
    
    def set_noise_params(self, alpha1: float, alpha2: float, alpha3: float, alpha4: float):
        """Update noise parameters at runtime."""
        self.config.alpha1 = alpha1
        self.config.alpha2 = alpha2
        self.config.alpha3 = alpha3
        self.config.alpha4 = alpha4
