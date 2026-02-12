"""
GPU AMCL ROS2 Node

Main ROS2 node that wraps the GPU-accelerated particle filter.
Subscribes to laser scans and odometry, publishes pose estimates.

This node follows the same interface as nav2_amcl for drop-in replacement.
"""

import numpy as np
import time
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import Header
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import (
    Pose, PoseStamped, PoseWithCovarianceStamped,
    PoseArray, TransformStamped
)
from visualization_msgs.msg import MarkerArray, Marker

import tf2_ros
from tf2_ros import TransformBroadcaster

from ..core import ParticleFilter, MotionModel, SensorModel, Resampler, SlipDetector
from ..core.particle_filter import ParticleFilterConfig
from ..core.motion_model import MotionModelConfig
from ..core.sensor_model import SensorModelConfig
from ..core.slip_detector import SlipDetectorConfig
from ..utils.math_utils import pose_to_array, quaternion_to_yaw, yaw_to_quaternion
from ..utils.map_utils import MapProcessor


class GPUAMCLNode(Node):
    """
    GPU-accelerated AMCL ROS2 Node.
    
    This node provides Monte Carlo Localization using GPU acceleration.
    It is designed as a drop-in replacement for nav2_amcl with improved
    performance on NVIDIA Jetson platforms.
    
    Subscriptions:
        /scan (sensor_msgs/LaserScan): Laser scan data
        /odom (nav_msgs/Odometry): Odometry data
        /map (nav_msgs/OccupancyGrid): Occupancy grid map
        /initialpose (geometry_msgs/PoseWithCovarianceStamped): Initial pose
    
    Publishers:
        /amcl_pose (geometry_msgs/PoseWithCovarianceStamped): Estimated pose
        /particlecloud (geometry_msgs/PoseArray): Particle visualization
    
    TF:
        Publishes: map -> odom transform
    
    Parameters:
        See config/amcl_params.yaml for full list
    """
    
    def __init__(self):
        super().__init__('gpu_amcl')
        
        # Declare and load parameters
        self._declare_parameters()
        self._load_parameters()
        
        # Initialize state
        self.pf: Optional[ParticleFilter] = None
        self.map_processor = MapProcessor()
        self.last_odom_pose: Optional[np.ndarray] = None
        self.last_odom_time: Optional[Time] = None
        self.initialized = False
        
        # IMU tracking for gyro-based rotation
        self.last_imu_time: Optional[Time] = None
        self.imu_dtheta_accumulated = 0.0  # Accumulated rotation since last PF update
        self.current_imu_linear_accel = (0.0, 0.0, 0.0)  # For slip detection
        self.current_imu_angular_velocity = 0.0
        
        # Slip detector
        if self.use_slip_detection:
            slip_config = SlipDetectorConfig(
                slip_threshold=self.slip_threshold,
                lateral_threshold=self.slip_lateral_threshold,
                slip_noise_multiplier=self.slip_noise_multiplier
            )
            self.slip_detector = SlipDetector(slip_config)
        
        # TF buffer and broadcaster
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS profiles
        sensor_qos = QoSProfile(depth=10)
        map_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        # Use ReentrantCallbackGroup so MultiThreadedExecutor can run
        # scan_callback and publish_callback truly in parallel
        self._cb_group = ReentrantCallbackGroup()
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, self.scan_topic, self.scan_callback, sensor_qos,
            callback_group=self._cb_group
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self.odom_callback, sensor_qos,
            callback_group=self._cb_group
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos,
            callback_group=self._cb_group
        )
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10,
            callback_group=self._cb_group
        )
        
        # IMU subscription for gyro-based rotation (optional)
        if self.use_imu_rotation:
            self.imu_sub = self.create_subscription(
                Imu, self.imu_topic, self.imu_callback, sensor_qos,
                callback_group=self._cb_group
            )
            self.get_logger().info(f'IMU fusion enabled: {self.imu_topic}')
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', 10
        )
        self.particle_pub = self.create_publisher(
            PoseArray, '/particlecloud', 10
        )
        
        # Timers - same reentrant group allows parallel execution with scan_callback
        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_callback,
            callback_group=self._cb_group
        )
        
        # Performance tracking
        self.update_count = 0
        self.total_update_time = 0.0
        
        # Thread safety for MultiThreadedExecutor
        self._pf_lock = threading.Lock()
        
        # Track last scan timestamp for proper pose stamping
        self._last_scan_stamp = None
        
        # Cached estimate for lock-free publishing
        self._cached_estimate = None
        self._cached_scan_stamp = None
        
        # Scan dropping: skip scans that arrive while processing
        self._processing_scan = False
        
        self.get_logger().info(f'GPU AMCL Node started')
        self.get_logger().info(f'  Particles: {self.num_particles}')
        self.get_logger().info(f'  GPU: {"enabled" if self.use_gpu else "disabled"}')
        self.get_logger().info(f'  Waiting for map...')
    
    def _declare_parameters(self):
        """Declare all ROS parameters."""
        # General
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('num_particles', 2000)
        self.declare_parameter('min_particles', 100)
        self.declare_parameter('max_particles', 5000)
        
        # KLD Sampling (Adaptive Particle Count)
        self.declare_parameter('use_kld_sampling', False)
        self.declare_parameter('kld_epsilon', 0.05)
        self.declare_parameter('kld_z', 2.33)  # 99% confidence
        
        # Topics and frames
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('base_frame_id', 'ego_racecar/base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('global_frame_id', 'map')
        
        # Update thresholds
        self.declare_parameter('update_min_d', 0.01)
        self.declare_parameter('update_min_a', 0.02)
        
        # Motion model
        self.declare_parameter('alpha1', 0.2)
        self.declare_parameter('alpha2', 0.2)
        self.declare_parameter('alpha3', 0.2)
        self.declare_parameter('alpha4', 0.2)
        
        # IMU fusion for motion model
        self.declare_parameter('imu_topic', '/sensors/imu/raw')
        self.declare_parameter('use_imu_rotation', True)
        self.declare_parameter('imu_gyro_weight', 0.8)  # Trust IMU 80%, odom 20% for rotation
        
        # Slip detection
        self.declare_parameter('use_slip_detection', True)
        self.declare_parameter('slip_threshold', 0.5)        # m/s² difference to trigger
        self.declare_parameter('slip_lateral_threshold', 0.3)  # Lateral accel threshold
        self.declare_parameter('slip_noise_multiplier', 2.0)  # Motion noise boost when slipping
        
        # Sensor model
        self.declare_parameter('max_beams', 60)
        self.declare_parameter('z_hit', 0.95)
        self.declare_parameter('z_rand', 0.05)
        self.declare_parameter('sigma_hit', 0.2)
        self.declare_parameter('laser_max_range', 10.0)
        
        # Laser offset
        self.declare_parameter('laser_offset_x', 0.275)
        self.declare_parameter('laser_offset_y', 0.0)
        
        # Resampling
        self.declare_parameter('resample_threshold', 0.5)
        
        # Initial pose
        self.declare_parameter('initial_pose_x', 0.0)
        self.declare_parameter('initial_pose_y', 0.0)
        self.declare_parameter('initial_pose_a', 0.0)
        self.declare_parameter('initial_cov_xx', 0.5)
        self.declare_parameter('initial_cov_yy', 0.5)
        self.declare_parameter('initial_cov_aa', 0.2)
        
        # Publishing
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('transform_tolerance', 0.1)
    
    def _load_parameters(self):
        """Load parameters into instance variables."""
        # General
        self.use_gpu = self.get_parameter('use_gpu').value
        self.num_particles = self.get_parameter('num_particles').value
        
        # Topics and frames
        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.global_frame_id = self.get_parameter('global_frame_id').value
        
        # Update thresholds
        self.update_min_d = self.get_parameter('update_min_d').value
        self.update_min_a = self.get_parameter('update_min_a').value
        
        # IMU fusion
        self.imu_topic = self.get_parameter('imu_topic').value
        self.use_imu_rotation = self.get_parameter('use_imu_rotation').value
        self.imu_gyro_weight = self.get_parameter('imu_gyro_weight').value
        
        # Slip detection
        self.use_slip_detection = self.get_parameter('use_slip_detection').value
        self.slip_threshold = self.get_parameter('slip_threshold').value
        self.slip_lateral_threshold = self.get_parameter('slip_lateral_threshold').value
        self.slip_noise_multiplier = self.get_parameter('slip_noise_multiplier').value
        
        # Publishing
        self.publish_rate = self.get_parameter('publish_rate').value
        self.transform_tolerance = self.get_parameter('transform_tolerance').value
    
    def map_callback(self, msg: OccupancyGrid):
        """Handle map message - initialize particle filter."""
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}')
        
        # Load map
        self.map_processor.load_from_nav_msgs(msg)
        
        # Build configuration
        pf_config = ParticleFilterConfig(
            num_particles=self.num_particles,
            min_particles=self.get_parameter('min_particles').value,
            max_particles=self.get_parameter('max_particles').value,
            use_gpu=self.use_gpu,
            resample_threshold=self.get_parameter('resample_threshold').value,
            initial_cov=(
                self.get_parameter('initial_cov_xx').value,
                self.get_parameter('initial_cov_yy').value,
                self.get_parameter('initial_cov_aa').value,
            ),
            # KLD Sampling
            use_kld_sampling=self.get_parameter('use_kld_sampling').value,
            kld_epsilon=self.get_parameter('kld_epsilon').value,
            kld_z=self.get_parameter('kld_z').value,
        )
        
        kld_status = "enabled" if pf_config.use_kld_sampling else "disabled"
        self.get_logger().info(f'KLD sampling: {kld_status}')
        
        # Create particle filter
        self.pf = ParticleFilter(pf_config)
        
        # Configure motion model
        self.pf.motion_model = MotionModel(
            config=MotionModelConfig(
                alpha1=self.get_parameter('alpha1').value,
                alpha2=self.get_parameter('alpha2').value,
                alpha3=self.get_parameter('alpha3').value,
                alpha4=self.get_parameter('alpha4').value,
            ),
            use_gpu=self.use_gpu
        )
        
        # Store base alpha values for slip detection (to restore after boosting)
        self.alpha1_base = self.get_parameter('alpha1').value
        self.alpha2_base = self.get_parameter('alpha2').value
        self.alpha3_base = self.get_parameter('alpha3').value
        self.alpha4_base = self.get_parameter('alpha4').value
        
        # Initial pose
        initial_pose = (
            self.get_parameter('initial_pose_x').value,
            self.get_parameter('initial_pose_y').value,
            self.get_parameter('initial_pose_a').value,
        )
        
        # Initialize particle filter
        self.pf.initialize(
            initial_pose=initial_pose,
            map_data=self.map_processor.map_data,
            map_resolution=self.map_processor.resolution,
            map_origin=self.map_processor.origin
        )
        
        # Configure sensor model
        self.pf.sensor_model.config.max_beams = self.get_parameter('max_beams').value
        self.pf.sensor_model.config.z_hit = self.get_parameter('z_hit').value
        self.pf.sensor_model.config.z_rand = self.get_parameter('z_rand').value
        self.pf.sensor_model.config.sigma_hit = self.get_parameter('sigma_hit').value
        self.pf.sensor_model.config.max_range = self.get_parameter('laser_max_range').value
        self.pf.sensor_model.config.laser_offset_x = self.get_parameter('laser_offset_x').value
        self.pf.sensor_model.config.laser_offset_y = self.get_parameter('laser_offset_y').value
        
        self.initialized = True
        self.get_logger().info('Particle filter initialized')
    
    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Handle initial pose estimate from RViz."""
        if self.pf is None:
            self.get_logger().warn('Cannot set initial pose: map not loaded')
            return
        
        pose = msg.pose.pose
        x = pose.position.x
        y = pose.position.y
        theta = quaternion_to_yaw(pose.orientation)
        
        self.get_logger().info(f'Reinitializing with pose: ({x:.2f}, {y:.2f}, {theta:.2f})')
        
        # Reinitialize particles around new pose
        self.pf.initialize(
            initial_pose=(x, y, theta),
            map_data=self.map_processor.map_data,
            map_resolution=self.map_processor.resolution,
            map_origin=self.map_processor.origin
        )
        
        self.last_odom_pose = None
    
    def odom_callback(self, msg: Odometry):
        """Handle odometry message - store for motion model."""
        pose = msg.pose.pose
        self.current_odom_pose = pose_to_array(pose)
        self.current_odom_time = Time.from_msg(msg.header.stamp)
    
    def imu_callback(self, msg: Imu):
        """
        Handle IMU message - accumulate gyro rotation for motion model.
        
        The VESC IMU provides angular velocity (gyro) and we integrate it
        to get rotation change. This is more accurate than wheel odometry
        at high speeds where wheel slip occurs.
        
        Also tracks linear acceleration for slip detection.
        """
        current_time = Time.from_msg(msg.header.stamp)
        
        if self.last_imu_time is not None:
            # Compute dt in seconds
            dt = (current_time - self.last_imu_time).nanoseconds * 1e-9
            
            # Integrate angular velocity (z-axis) to get rotation change
            # Assuming planar motion, only yaw matters
            omega_z = msg.angular_velocity.z  # rad/s
            self.imu_dtheta_accumulated += omega_z * dt
        
        # Store for slip detection
        self.current_imu_linear_accel = (
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        )
        self.current_imu_angular_velocity = msg.angular_velocity.z
        
        self.last_imu_time = current_time
    
    def scan_callback(self, msg: LaserScan):
        """Handle laser scan - main update loop."""
        if not self.initialized:
            return
        
        # Drop scans that arrive while we're still processing the previous one
        if self._processing_scan:
            return
        self._processing_scan = True
        
        # Track scan timestamp for pose stamping
        self._last_scan_stamp = msg.header.stamp
        
        t_start = time.perf_counter()
        
        # Compute odometry delta
        if self.last_odom_pose is not None and hasattr(self, 'current_odom_pose'):
            dx = self.current_odom_pose[0] - self.last_odom_pose[0]
            dy = self.current_odom_pose[1] - self.last_odom_pose[1]
            dtheta = self.current_odom_pose[2] - self.last_odom_pose[2]
            
            # Normalize angle
            while dtheta > np.pi:
                dtheta -= 2 * np.pi
            while dtheta < -np.pi:
                dtheta += 2 * np.pi
            
            # Transform to robot frame
            cos_theta = np.cos(-self.last_odom_pose[2])
            sin_theta = np.sin(-self.last_odom_pose[2])
            dx_robot = dx * cos_theta - dy * sin_theta
            dy_robot = dx * sin_theta + dy * cos_theta
            
            # Check if moved enough to update
            dist = np.sqrt(dx_robot**2 + dy_robot**2)
            if dist < self.update_min_d and abs(dtheta) < self.update_min_a:
                self._processing_scan = False
                return  # Haven't moved enough
            
            # Slip detection (if enabled)
            noise_multiplier = 1.0
            if self.use_slip_detection and hasattr(self, 'slip_detector'):
                # Estimate velocity from odometry delta
                if hasattr(self, 'current_odom_time') and hasattr(self, 'last_odom_time_for_slip'):
                    dt = (self.current_odom_time - self.last_odom_time_for_slip).nanoseconds * 1e-9
                    if dt > 0:
                        velocity = dist / dt
                        is_slipping = self.slip_detector.update(
                            velocity, dt,
                            self.current_imu_linear_accel,
                            self.current_imu_angular_velocity
                        )
                        if is_slipping:
                            noise_multiplier = self.slip_detector.get_noise_multiplier()
                            self.get_logger().debug(
                                f'Slip detected! Confidence: {self.slip_detector.slip_confidence:.2f}, '
                                f'noise mult: {noise_multiplier:.1f}'
                            )
                self.last_odom_time_for_slip = self.current_odom_time
            
            # Prediction step with optional IMU fusion
            imu_dtheta = None
            if self.use_imu_rotation and self.imu_dtheta_accumulated != 0.0:
                imu_dtheta = self.imu_dtheta_accumulated
                self.imu_dtheta_accumulated = 0.0  # Reset accumulator
            
            # Apply noise multiplier if slipping
            if noise_multiplier > 1.0:
                self.pf.motion_model.set_noise_params(
                    self.alpha1_base * noise_multiplier,
                    self.alpha2_base * noise_multiplier,
                    self.alpha3_base * noise_multiplier,
                    self.alpha4_base * noise_multiplier
                )
            
            with self._pf_lock:
                self.pf.predict((dx_robot, dy_robot, dtheta), imu_dtheta=imu_dtheta)
            
            # Restore normal noise after prediction
            if noise_multiplier > 1.0:
                self.pf.motion_model.set_noise_params(
                    self.alpha1_base, self.alpha2_base,
                    self.alpha3_base, self.alpha4_base
                )
        
        # Update odometry reference
        if hasattr(self, 'current_odom_pose'):
            self.last_odom_pose = self.current_odom_pose.copy()
        
        # Measurement update (thread-safe)
        ranges = np.array(msg.ranges, dtype=np.float32)
        with self._pf_lock:
            self.pf.update(ranges, msg.angle_min, msg.angle_increment)
        
            # Cache the estimate so publish_callback can use it without locking
            self._cached_estimate = self.pf.get_estimate()
            self._cached_scan_stamp = self._last_scan_stamp
        
        # Timing
        t_elapsed = time.perf_counter() - t_start
        self.update_count += 1
        self.total_update_time += t_elapsed
        
        # Ready for next scan
        self._processing_scan = False
        
        # Log performance periodically
        if self.update_count % 100 == 0:
            avg_time = self.total_update_time / self.update_count * 1000
            timing = self.pf.get_timing_stats()
            self.get_logger().info(
                f'AMCL update: {avg_time:.1f}ms avg '
                f'(predict:{timing["predict_ms"]:.1f}, '
                f'update:{timing["update_ms"]:.1f}, '
                f'resample:{timing["resample_ms"]:.1f})'
            )
    
    def publish_callback(self):
        """Periodically publish pose estimate and TF."""
        if not self.initialized or self.pf is None:
            return
        
        # Use cached estimate - no lock needed, avoids blocking on scan processing
        estimate = self._cached_estimate
        if estimate is None:
            return
        
        now = self.get_clock().now()
        
        # Use the scan timestamp that produced this estimate for proper latency tracking
        pose_stamp = self._cached_scan_stamp
        if pose_stamp is None:
            pose_stamp = now.to_msg()
        
        # Publish PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = pose_stamp
        pose_msg.header.frame_id = self.global_frame_id
        
        pose_msg.pose.pose.position.x = estimate.x
        pose_msg.pose.pose.position.y = estimate.y
        pose_msg.pose.pose.position.z = 0.0
        
        qx, qy, qz, qw = yaw_to_quaternion(estimate.theta)
        pose_msg.pose.pose.orientation.x = qx
        pose_msg.pose.pose.orientation.y = qy
        pose_msg.pose.pose.orientation.z = qz
        pose_msg.pose.pose.orientation.w = qw
        
        # Covariance (6x6, we only use x, y, yaw)
        cov_flat = [0.0] * 36
        cov_flat[0] = float(estimate.covariance[0, 0])   # xx
        cov_flat[1] = float(estimate.covariance[0, 1])   # xy
        cov_flat[5] = float(estimate.covariance[0, 2])   # x-yaw
        cov_flat[6] = float(estimate.covariance[1, 0])   # yx
        cov_flat[7] = float(estimate.covariance[1, 1])   # yy
        cov_flat[11] = float(estimate.covariance[1, 2])  # y-yaw
        cov_flat[30] = float(estimate.covariance[2, 0])  # yaw-x
        cov_flat[31] = float(estimate.covariance[2, 1])  # yaw-y
        cov_flat[35] = float(estimate.covariance[2, 2])  # yaw-yaw
        pose_msg.pose.covariance = cov_flat
        
        self.pose_pub.publish(pose_msg)
        
        # Publish TF: map -> odom
        self._publish_transform(estimate, now)
        
        # Publish particle cloud
        self._publish_particles(now)
    
    def _publish_transform(self, estimate, stamp):
        """Publish map -> odom transform."""
        # We need to compute map->odom from map->base_link and odom->base_link
        # map_T_odom = map_T_base * inv(odom_T_base)
        
        if not hasattr(self, 'current_odom_pose'):
            return
        
        # AMCL estimate is map->base_link
        map_x, map_y, map_theta = estimate.x, estimate.y, estimate.theta
        
        # Current odom is odom->base_link
        odom_x, odom_y, odom_theta = self.current_odom_pose
        
        # Compute map->odom: first translate by -(odom->base), then rotate
        cos_odom = np.cos(-odom_theta)
        sin_odom = np.sin(-odom_theta)
        
        # Inverse of odom->base in odom frame
        inv_odom_x = -(odom_x * cos_odom - odom_y * sin_odom)
        inv_odom_y = -(odom_x * sin_odom + odom_y * cos_odom)
        inv_odom_theta = -odom_theta
        
        # Compose: map->base + inv(odom->base)
        cos_map = np.cos(map_theta)
        sin_map = np.sin(map_theta)
        
        tf_x = map_x + inv_odom_x * cos_map - inv_odom_y * sin_map
        tf_y = map_y + inv_odom_x * sin_map + inv_odom_y * cos_map
        tf_theta = map_theta + inv_odom_theta
        
        # Normalize angle
        tf_theta = np.arctan2(np.sin(tf_theta), np.cos(tf_theta))
        
        # Create transform message
        t = TransformStamped()
        t.header.stamp = stamp.to_msg()
        t.header.frame_id = self.global_frame_id
        t.child_frame_id = self.odom_frame_id
        
        t.transform.translation.x = tf_x
        t.transform.translation.y = tf_y
        t.transform.translation.z = 0.0
        
        qx, qy, qz, qw = yaw_to_quaternion(tf_theta)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        
        self.tf_broadcaster.sendTransform(t)
    
    def _publish_particles(self, stamp):
        """Publish particle cloud for visualization."""
        # Non-blocking: skip if scan processing holds the lock
        if not self._pf_lock.acquire(blocking=False):
            return
        try:
            particles, weights = self.pf.get_particles_numpy()
        except Exception:
            return
        finally:
            self._pf_lock.release()
        
        msg = PoseArray()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.global_frame_id
        
        # Subsample particles for visualization (max 100)
        step = max(1, len(particles) // 100)
        
        for i in range(0, len(particles), step):
            pose = Pose()
            pose.position.x = float(particles[i, 0])
            pose.position.y = float(particles[i, 1])
            pose.position.z = 0.0
            
            qx, qy, qz, qw = yaw_to_quaternion(float(particles[i, 2]))
            pose.orientation.x = qx
            pose.orientation.y = qy
            pose.orientation.z = qz
            pose.orientation.w = qw
            
            msg.poses.append(pose)
        
        self.particle_pub.publish(msg)
