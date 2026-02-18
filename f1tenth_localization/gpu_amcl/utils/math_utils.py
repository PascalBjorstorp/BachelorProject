"""
Math Utilities for GPU AMCL

Common mathematical operations used throughout the particle filter.
All functions support both NumPy and CuPy arrays.
"""

from typing import Tuple, Union
import numpy as np

# Try to import CuPy for GPU acceleration
try:
    import cupy as cp
    GPU_AVAILABLE = True
except ImportError:
    cp = None
    GPU_AVAILABLE = False


def normalize_angle(angle: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
    """
    Normalize angle(s) to [-pi, pi].
    
    Args:
        angle: Single angle or array of angles (radians)
    
    Returns:
        Normalized angle(s) in [-pi, pi]
    """
    if isinstance(angle, (int, float)):
        return float(np.arctan2(np.sin(angle), np.cos(angle)))
    
    # Detect if CuPy array
    if GPU_AVAILABLE and isinstance(angle, cp.ndarray):
        return cp.arctan2(cp.sin(angle), cp.cos(angle))
    
    return np.arctan2(np.sin(angle), np.cos(angle))


def angle_diff(a: float, b: float) -> float:
    """
    Compute shortest difference between two angles.
    
    Args:
        a: First angle (radians)
        b: Second angle (radians)
    
    Returns:
        Shortest angular difference in [-pi, pi]
    """
    diff = a - b
    while diff > np.pi:
        diff -= 2 * np.pi
    while diff < -np.pi:
        diff += 2 * np.pi
    return diff


def pose_to_array(pose) -> np.ndarray:
    """
    Convert ROS Pose message to numpy array [x, y, theta].
    
    Args:
        pose: geometry_msgs/Pose or geometry_msgs/PoseStamped
    
    Returns:
        numpy array [x, y, theta]
    """
    # Handle PoseStamped
    if hasattr(pose, 'pose'):
        pose = pose.pose
    
    x = pose.position.x
    y = pose.position.y
    
    # Extract yaw from quaternion
    q = pose.orientation
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    theta = np.arctan2(siny_cosp, cosy_cosp)
    
    return np.array([x, y, theta], dtype=np.float32)


def array_to_pose(arr: np.ndarray):
    """
    Convert [x, y, theta] array to ROS Pose message.
    
    Args:
        arr: numpy array [x, y, theta]
    
    Returns:
        geometry_msgs/Pose message
    """
    from geometry_msgs.msg import Pose, Point, Quaternion
    
    x, y, theta = float(arr[0]), float(arr[1]), float(arr[2])
    
    # Convert yaw to quaternion (only rotation about Z axis)
    qz = np.sin(theta / 2)
    qw = np.cos(theta / 2)
    
    pose = Pose()
    pose.position = Point(x=x, y=y, z=0.0)
    pose.orientation = Quaternion(x=0.0, y=0.0, z=float(qz), w=float(qw))
    
    return pose


def quaternion_to_yaw(q) -> float:
    """
    Extract yaw angle from quaternion.
    
    Args:
        q: Quaternion (object with x, y, z, w attributes)
    
    Returns:
        Yaw angle in radians
    """
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """
    Convert yaw angle to quaternion (rotation about Z axis only).
    
    Args:
        yaw: Yaw angle in radians
    
    Returns:
        Tuple of (x, y, z, w) quaternion components
    """
    return (0.0, 0.0, float(np.sin(yaw / 2)), float(np.cos(yaw / 2)))


def transform_pose(pose: np.ndarray, dx: float, dy: float, dtheta: float) -> np.ndarray:
    """
    Apply a relative transform to a pose.
    
    Args:
        pose: [x, y, theta] current pose
        dx: Forward motion in robot frame
        dy: Lateral motion in robot frame
        dtheta: Rotation
    
    Returns:
        New [x, y, theta] pose
    """
    x, y, theta = pose[0], pose[1], pose[2]
    
    # Transform motion to world frame
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    
    new_x = x + dx * cos_theta - dy * sin_theta
    new_y = y + dx * sin_theta + dy * cos_theta
    new_theta = normalize_angle(theta + dtheta)
    
    return np.array([new_x, new_y, new_theta], dtype=np.float32)


def compute_pose_covariance(particles: np.ndarray, weights: np.ndarray) -> np.ndarray:
    """
    Compute 3x3 pose covariance from weighted particles.
    
    Args:
        particles: (N, 3) array of [x, y, theta]
        weights: (N,) array of normalized weights
    
    Returns:
        3x3 covariance matrix
    """
    # Compute weighted mean
    mean_x = np.sum(weights * particles[:, 0])
    mean_y = np.sum(weights * particles[:, 1])
    
    # Circular mean for theta
    sin_sum = np.sum(weights * np.sin(particles[:, 2]))
    cos_sum = np.sum(weights * np.cos(particles[:, 2]))
    mean_theta = np.arctan2(sin_sum, cos_sum)
    
    # Center particles
    centered = particles.copy()
    centered[:, 0] -= mean_x
    centered[:, 1] -= mean_y
    centered[:, 2] = normalize_angle(particles[:, 2] - mean_theta)
    
    # Weighted covariance
    cov = np.zeros((3, 3), dtype=np.float32)
    for i in range(3):
        for j in range(3):
            cov[i, j] = np.sum(weights * centered[:, i] * centered[:, j])
    
    return cov
