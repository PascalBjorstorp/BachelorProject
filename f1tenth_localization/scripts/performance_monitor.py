#!/usr/bin/env python3
"""
Performance Monitor for F1TENTH Localization

Monitors CPU, GPU (Jetson), memory usage, and localization latency.
Tracks AMCL process CPU usage specifically for accurate benchmarking.
Outputs data to CSV for analysis.

Usage:
    ros2 run f1tenth_localization performance_monitor.py
    ros2 run f1tenth_localization performance_monitor.py --ros-args -p output_dir:=/path/to/output
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import csv
import os
import time
import subprocess
import threading
from datetime import datetime


class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        
        # Parameters
        self.declare_parameter('output_dir', '/tmp/f1tenth_performance')
        self.declare_parameter('sample_rate_hz', 100.0)  # Higher rate to catch spikes
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        
        # Benchmark configuration parameters (for CSV metadata)
        self.declare_parameter('amcl_type', 'nav2_amcl')  # e.g., 'nav2_amcl', 'gpu_amcl'
        self.declare_parameter('min_particles', 500)
        self.declare_parameter('max_particles', 2000)
        self.declare_parameter('max_beams', 60)
        
        self.output_dir = self.get_parameter('output_dir').value
        self.sample_rate = self.get_parameter('sample_rate_hz').value
        scan_topic = self.get_parameter('scan_topic').value
        amcl_topic = self.get_parameter('amcl_pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        
        # Store benchmark config
        self.amcl_type = self.get_parameter('amcl_type').value
        self.min_particles = self.get_parameter('min_particles').value
        self.max_particles = self.get_parameter('max_particles').value
        self.max_beams = self.get_parameter('max_beams').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Detect platform (Jetson vs regular PC)
        self.is_jetson = self._detect_jetson()
        platform = "Jetson" if self.is_jetson else "PC"
        self.get_logger().info(f'Detected platform: {platform}')
        self.get_logger().info(f'Benchmark config: {self.amcl_type}, particles={self.min_particles}-{self.max_particles}, beams={self.max_beams}')
        
        # CSV file for logging - include config in filename
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        config_str = f'{self.amcl_type}_p{self.min_particles}-{self.max_particles}_b{self.max_beams}'
        self.csv_filename = os.path.join(self.output_dir, f'performance_{config_str}_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        
        # CSV headers - include benchmark config and metrics
        headers = [
            'timestamp_sec', 'timestamp_nsec',
            'amcl_type', 'min_particles', 'max_particles', 'max_beams',
            'system_cpu_percent', 'amcl_cpu_percent', 'amcl_memory_mb',
            'memory_percent', 'memory_used_mb',
            'scan_to_pose_latency_ms', 'scan_rate_hz', 'pose_rate_hz',
            'num_cores',
            # Accuracy metrics (ground truth comparison)
            'position_error_m', 'orientation_error_rad',
            'position_error_mean_m', 'orientation_error_mean_rad',
        ]
        
        # Add per-core CPU headers
        try:
            import psutil
            self._temp_num_cores = psutil.cpu_count()
            for i in range(self._temp_num_cores):
                headers.append(f'cpu_core_{i}_percent')
        except ImportError:
            self._temp_num_cores = 0
            
        if self.is_jetson:
            headers.extend(['gpu_percent', 'gpu_freq_mhz', 'emc_percent'])
        
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=headers)
        self.csv_writer.writeheader()
        
        # State for latency calculation
        self.last_scan_time = None
        self.last_scan_header_time = None  # Timestamp from scan message header  
        self.last_scan_wall_time = None    # Wall clock time when scan was received
        self.last_pose_time = None
        self.scan_timestamps = []  # Last N scan timestamps for rate calculation
        self.pose_timestamps = []  # Last N pose timestamps for rate calculation
        self.scan_pose_latencies = []  # Last N latencies (wall clock: scan rx → pose rx)
        self.scan_receipt_times = {}  # Map: scan_header_stamp_ns -> wall_time_ns when received
        
        # Ground truth tracking for accuracy measurement
        self.ground_truth_pose = None  # Latest ground truth from simulator odom
        self.ground_truth_history = []  # List of (timestamp_ns, pose) for temporal matching
        self.GT_HISTORY_SIZE = 200  # Keep last N ground truth poses
        self.position_errors = []  # Recent position errors (meters)
        self.orientation_errors = []  # Recent orientation errors (radians)
        
        # Frame alignment: capture initial offset between map frame (AMCL) and odom frame (ground truth)
        self.frame_alignment_offset = None  # (dx, dy, dyaw) to align odom frame to map frame
        self.first_amcl_pose = None
        self.first_gt_pose = None
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic, self.pose_callback, 10
        )
        # Ground truth subscription (simulator provides true pose via odom)
        self.ground_truth_sub = self.create_subscription(
            Odometry, '/ego_racecar/odom', self.ground_truth_callback, 10
        )
        
        # Timer for periodic sampling
        period = 1.0 / self.sample_rate
        self.timer = self.create_timer(period, self.sample_callback)
        
        # CPU tracking
        self.cpu_percent = 0.0
        self.per_core_cpu = []  # Per-core CPU percentages
        self.memory_percent = 0.0
        self.memory_used_mb = 0.0
        self.amcl_process = None
        self.amcl_cpu_percent = 0.0
        self.amcl_memory_mb = 0.0
        
        # Peak tracking for spikes
        self.amcl_cpu_peak = 0.0
        self.amcl_cpu_samples = []  # Recent samples for running average
        self.peak_reset_counter = 0
        
        # Try to import psutil
        try:
            import psutil
            self.psutil = psutil
            self.num_cores = psutil.cpu_count()
            self.get_logger().info(f'psutil available - {self.num_cores} CPU cores detected')
        except ImportError:
            self.psutil = None
            self.num_cores = 1
            self.get_logger().warn('psutil not available - install with: pip install psutil')
        
        self.get_logger().info(f'Performance Monitor started')
        self.get_logger().info(f'  Output: {self.csv_filename}')
        self.get_logger().info(f'  Sample rate: {self.sample_rate} Hz')
        
    def _detect_jetson(self):
        """Detect if running on Jetson by checking for tegra files"""
        return os.path.exists('/sys/devices/gpu.0') or \
               os.path.exists('/sys/class/thermal/thermal_zone0/type')
    
    def _find_amcl_process(self):
        """Find the AMCL process by name"""
        if self.psutil is None:
            return None
        
        if self.amcl_process is not None:
            try:
                # Check if process still exists
                if self.amcl_process.is_running():
                    return self.amcl_process
            except:
                pass
            self.amcl_process = None
        
        # Search for AMCL process - look for the nav2_amcl executable or gpu_amcl Python node
        # IMPORTANT: Skip 'ros2 launch' processes - they also contain 'gpu_amcl' in their args
        for proc in self.psutil.process_iter(['pid', 'name', 'cmdline', 'exe']):
            try:
                exe = proc.info.get('exe', '') or ''
                name = proc.info.get('name', '') or ''
                cmdline = proc.info.get('cmdline', []) or []
                cmdline_str = ' '.join(str(c) for c in cmdline if c is not None)
                
                # Skip ros2 launch processes - they contain amcl in args but aren't the node
                if 'ros2' in cmdline_str and 'launch' in cmdline_str:
                    continue
                
                # Match AMCL executable (nav2_amcl binary or gpu_amcl Python script)
                is_amcl = (
                    name == 'amcl' or
                    exe.endswith('/amcl') or
                    'nav2_amcl' in cmdline_str or
                    'gpu_amcl_node' in cmdline_str or
                    '__node:=gpu_amcl' in cmdline_str or
                    '__node:=amcl' in cmdline_str or
                    any(str(c).endswith('/amcl') for c in cmdline if c is not None)
                )
                
                if is_amcl:
                    self.amcl_process = proc
                    self.get_logger().info(
                        f'Found AMCL process: PID {proc.pid}, name={name}, '
                        f'exe={exe}, cmdline={cmdline_str[:120]}'
                    )
                    # Initialize CPU measurement (first call returns 0, subsequent calls track delta)
                    proc.cpu_percent(interval=None)
                    return proc
            except (self.psutil.NoSuchProcess, self.psutil.AccessDenied,
                    self.psutil.ZombieProcess, AttributeError, TypeError):
                continue
        
        return None
    
    def _get_amcl_cpu_usage(self):
        """Get AMCL process CPU and memory usage"""
        amcl_cpu = 0.0
        amcl_mem = 0.0
        
        proc = self._find_amcl_process()
        if proc is not None:
            try:
                # CPU percent - this returns the CPU usage since last call
                # Returns single-core equivalent (100% = 1 full core)
                amcl_cpu = proc.cpu_percent(interval=None)
                
                # Memory in MB
                mem_info = proc.memory_info()
                amcl_mem = mem_info.rss / (1024 * 1024)
                
                # Track peak CPU for this session
                if amcl_cpu > self.amcl_cpu_peak:
                    self.amcl_cpu_peak = amcl_cpu
                    self.get_logger().debug(f'New AMCL CPU peak: {amcl_cpu:.1f}%')
                    
            except (self.psutil.NoSuchProcess, self.psutil.AccessDenied, self.psutil.ZombieProcess) as e:
                self.get_logger().debug(f'AMCL process access error: {e}')
                self.amcl_process = None
        
        return amcl_cpu, amcl_mem
    
    def _get_jetson_gpu_usage(self):
        """Read GPU usage from Jetson tegrastats or sysfs"""
        gpu_percent = 0.0
        gpu_freq = 0.0
        emc_percent = 0.0
        
        # Use cached tegrastats result if available and recent
        if hasattr(self, '_tegrastats_cache'):
            cache_time, cached_result = self._tegrastats_cache
            if time.time() - cache_time < 0.5:  # Use cache if less than 0.5s old
                return cached_result
        
        try:
            # Method 1: Try tegrastats (most reliable across all Jetson models)
            # Run with timeout and capture first line
            result = subprocess.run(
                ['timeout', '0.5', 'tegrastats', '--interval', '100'],
                capture_output=True, text=True, timeout=1
            )
            output = result.stdout + result.stderr
            if output:
                # Parse GPU usage: GR3D_FREQ 0%@76 or GR3D 45%@921
                import re
                gpu_match = re.search(r'GR3D[_FREQ]*\s+(\d+)%', output)
                if gpu_match:
                    gpu_percent = float(gpu_match.group(1))
                
                # Parse GPU frequency
                freq_match = re.search(r'GR3D[_FREQ]*\s+\d+%@(\d+)', output)
                if freq_match:
                    gpu_freq = float(freq_match.group(1))
                
                # Parse EMC usage
                emc_match = re.search(r'EMC_FREQ\s+(\d+)%', output)
                if emc_match:
                    emc_percent = float(emc_match.group(1))
                
                result = (gpu_percent, gpu_freq, emc_percent)
                self._tegrastats_cache = (time.time(), result)
                return result
                
        except (subprocess.TimeoutExpired, FileNotFoundError):
            pass
        except Exception as e:
            self.get_logger().debug(f'tegrastats failed: {e}')
        
        # Method 2: Fallback to sysfs paths
        try:
            # Try reading GPU load from sysfs (Jetson)
            # Check multiple possible paths for different Jetson models
            gpu_load_paths = [
                '/sys/devices/gpu.0/load',
                '/sys/devices/platform/gpu.0/load',
                '/sys/devices/17000000.ga10b/load',
                '/sys/devices/17000000.gv11b/load',
                '/sys/devices/platform/17000000.ga10b/load',
                '/sys/devices/platform/17000000.gv11b/load',
            ]
            
            for gpu_load_path in gpu_load_paths:
                if os.path.exists(gpu_load_path):
                    with open(gpu_load_path, 'r') as f:
                        gpu_percent = float(f.read().strip()) / 10.0  # Value is in 0.1%
                    break
            
            # Try reading GPU frequency from multiple possible paths
            import glob
            gpu_freq_patterns = [
                '/sys/devices/gpu.0/devfreq/*/cur_freq',
                '/sys/devices/platform/gpu.0/devfreq/*/cur_freq',
                '/sys/devices/17000000.ga10b/devfreq/*/cur_freq',
                '/sys/devices/17000000.gv11b/devfreq/*/cur_freq',
                '/sys/devices/platform/*/devfreq/*/cur_freq',
            ]
            
            for pattern in gpu_freq_patterns:
                matches = glob.glob(pattern)
                if matches:
                    with open(matches[0], 'r') as f:
                        gpu_freq = float(f.read().strip()) / 1e6  # Convert to MHz
                    break
            
            # EMC (memory controller) usage - indicates memory bandwidth
            emc_paths = [
                '/sys/kernel/debug/clk/emc/clk_rate',
                '/sys/kernel/debug/bpmp/debug/clk/emc/rate',
            ]
            for emc_path in emc_paths:
                if os.path.exists(emc_path):
                    try:
                        with open(emc_path, 'r') as f:
                            emc_percent = float(f.read().strip()) / 1e6
                    except:
                        pass
                    break
                    
        except Exception as e:
            self.get_logger().debug(f'Could not read Jetson GPU stats: {e}')
        
        return gpu_percent, gpu_freq, emc_percent
    
    def scan_callback(self, msg: LaserScan):
        """Record scan timestamp for latency calculation"""
        now = self.get_clock().now()
        self.last_scan_time = now
        
        # Store scan header timestamp (when the scan was taken by sensor)
        # We'll use this to correlate with pose output
        scan_header_stamp = Time.from_msg(msg.header.stamp)
        self.last_scan_header_time = scan_header_stamp
        self.last_scan_wall_time = now  # Wall clock when we received this scan
        
        # Store scan reception time indexed by header timestamp for proper correlation
        # This allows us to find when we received a specific scan that AMCL processed
        scan_stamp_key = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.scan_receipt_times[scan_stamp_key] = now.nanoseconds
        
        # Cleanup old entries (keep last 200 scans)
        if len(self.scan_receipt_times) > 200:
            oldest_key = min(self.scan_receipt_times.keys())
            del self.scan_receipt_times[oldest_key]
        
        # Track scan rate
        self.scan_timestamps.append(now.nanoseconds)
        if len(self.scan_timestamps) > 100:
            self.scan_timestamps.pop(0)
    
    def ground_truth_callback(self, msg: Odometry):
        """Store ground truth pose from simulator with timestamp for temporal matching"""
        self.ground_truth_pose = msg.pose.pose
        
        # Store in history with timestamp for temporal matching against AMCL poses
        gt_stamp_ns = msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec
        self.ground_truth_history.append((gt_stamp_ns, msg.pose.pose))
        if len(self.ground_truth_history) > self.GT_HISTORY_SIZE:
            self.ground_truth_history.pop(0)
        
        # Capture first ground truth pose for alignment
        if self.first_gt_pose is None:
            self.first_gt_pose = msg.pose.pose
            self._try_compute_alignment()
    
    def _find_closest_gt_pose(self, target_stamp_ns):
        """Find the ground truth pose closest in time to the given timestamp."""
        if not self.ground_truth_history:
            return self.ground_truth_pose
        
        best_pose = None
        best_diff = float('inf')
        for ts_ns, pose in self.ground_truth_history:
            diff = abs(ts_ns - target_stamp_ns)
            if diff < best_diff:
                best_diff = diff
                best_pose = pose
        
        return best_pose
    
    def _try_compute_alignment(self):
        """Compute frame alignment offset when both first poses are available"""
        import math
        if self.first_amcl_pose is not None and self.first_gt_pose is not None and self.frame_alignment_offset is None:
            # Calculate offset: how much to add to GT pose to get AMCL pose
            amcl_yaw = self._quaternion_to_yaw(self.first_amcl_pose.orientation)
            gt_yaw = self._quaternion_to_yaw(self.first_gt_pose.orientation)
            
            # Offset = AMCL - GT (we'll add this to GT to align with AMCL)
            dx = self.first_amcl_pose.position.x - self.first_gt_pose.position.x
            dy = self.first_amcl_pose.position.y - self.first_gt_pose.position.y
            dyaw = amcl_yaw - gt_yaw
            
            # Normalize dyaw
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            
            self.frame_alignment_offset = (dx, dy, dyaw)
            self.get_logger().info(f'Frame alignment computed: dx={dx:.3f}m, dy={dy:.3f}m, dyaw={math.degrees(dyaw):.1f}°')
    
    def _quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        import math
        # yaw (z-axis rotation)
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def _angle_diff(self, a, b):
        """Calculate smallest difference between two angles (handles wraparound)"""
        import math
        diff = a - b
        while diff > math.pi:
            diff -= 2 * math.pi
        while diff < -math.pi:
            diff += 2 * math.pi
        return abs(diff)
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Record pose timestamp and calculate latency by correlating with the scan that produced it"""
        import math
        now = self.get_clock().now()
        self.last_pose_time = now
        
        amcl_pose = msg.pose.pose
        
        # Capture first AMCL pose for alignment
        if self.first_amcl_pose is None:
            self.first_amcl_pose = amcl_pose
            self._try_compute_alignment()
        
        # AMCL stamps its pose with the same timestamp as the scan it processed
        # Use this to look up when we actually received that specific scan
        pose_stamp = msg.header.stamp
        pose_stamp_key = pose_stamp.sec * 1000000000 + pose_stamp.nanosec
        
        if pose_stamp_key in self.scan_receipt_times:
            # Found the matching scan - calculate true processing latency
            scan_receipt_ns = self.scan_receipt_times[pose_stamp_key]
            latency_ns = now.nanoseconds - scan_receipt_ns
            latency_ms = latency_ns / 1e6
            
            # Record latency if reasonable (0 to 5 seconds)
            # Use >= 0 to include near-zero latencies
            if 0 <= latency_ms < 5000:
                self.scan_pose_latencies.append(latency_ms)
                if len(self.scan_pose_latencies) > 100:
                    self.scan_pose_latencies.pop(0)
        else:
            # Fallback: use wall-clock difference between last scan receipt and pose receipt
            # This is less accurate but better than reporting 0
            if self.last_scan_wall_time is not None:
                fallback_latency_ns = now.nanoseconds - self.last_scan_wall_time.nanoseconds
                fallback_latency_ms = fallback_latency_ns / 1e6
                if 0 <= fallback_latency_ms < 5000:
                    self.scan_pose_latencies.append(fallback_latency_ms)
                    if len(self.scan_pose_latencies) > 100:
                        self.scan_pose_latencies.pop(0)
        
        # Calculate accuracy compared to ground truth (with frame alignment)
        # Use temporally matched ground truth - find GT pose closest to the AMCL pose timestamp
        # This prevents errors from temporal misalignment (AMCL pose may be based on an older scan)
        if self.ground_truth_history and self.frame_alignment_offset is not None:
            gt_pose = self._find_closest_gt_pose(pose_stamp_key)
            if gt_pose is None:
                gt_pose = self.ground_truth_pose
            
            dx_offset, dy_offset, dyaw_offset = self.frame_alignment_offset
            
            # Aligned ground truth position (in map frame)
            gt_x_aligned = gt_pose.position.x + dx_offset
            gt_y_aligned = gt_pose.position.y + dy_offset
            gt_yaw = self._quaternion_to_yaw(gt_pose.orientation)
            gt_yaw_aligned = gt_yaw + dyaw_offset
            
            # Position error (Euclidean distance in x-y plane)
            dx = amcl_pose.position.x - gt_x_aligned
            dy = amcl_pose.position.y - gt_y_aligned
            position_error = math.sqrt(dx * dx + dy * dy)
            
            # Orientation error (yaw difference)
            amcl_yaw = self._quaternion_to_yaw(amcl_pose.orientation)
            orientation_error = self._angle_diff(amcl_yaw, gt_yaw_aligned)
            
            # Sanity check: cap position error at reasonable max (e.g., 100m = map size)
            # Errors larger than this indicate AMCL is completely lost (kidnapped)
            MAX_REASONABLE_ERROR = 100.0  # meters
            if position_error > MAX_REASONABLE_ERROR:
                position_error = MAX_REASONABLE_ERROR  # Cap for stats, but record as "lost"
            
            # Store errors
            self.position_errors.append(position_error)
            self.orientation_errors.append(orientation_error)
            if len(self.position_errors) > 100:
                self.position_errors.pop(0)
            if len(self.orientation_errors) > 100:
                self.orientation_errors.pop(0)
        
        # Track pose rate
        self.pose_timestamps.append(now.nanoseconds)
        if len(self.pose_timestamps) > 100:
            self.pose_timestamps.pop(0)
    
    def _calculate_rate(self, timestamps):
        """Calculate rate from list of timestamps (in nanoseconds)"""
        if len(timestamps) < 2:
            return 0.0
        
        duration_ns = timestamps[-1] - timestamps[0]
        if duration_ns <= 0:
            return 0.0
        
        duration_sec = duration_ns / 1e9
        return (len(timestamps) - 1) / duration_sec
    
    def sample_callback(self):
        """Periodic sampling of performance metrics"""
        now = self.get_clock().now()
        
        # Get system-wide CPU and memory usage
        if self.psutil:
            self.cpu_percent = self.psutil.cpu_percent(interval=None)
            self.per_core_cpu = self.psutil.cpu_percent(interval=None, percpu=True)
            mem = self.psutil.virtual_memory()
            self.memory_percent = mem.percent
            self.memory_used_mb = mem.used / (1024 * 1024)
            
            # Get AMCL-specific usage
            self.amcl_cpu_percent, self.amcl_memory_mb = self._get_amcl_cpu_usage()
            
            # Track peaks and running average
            self.amcl_cpu_samples.append(self.amcl_cpu_percent)
            if len(self.amcl_cpu_samples) > 50:  # ~1 sec window at 50 Hz
                self.amcl_cpu_samples.pop(0)
            
            if self.amcl_cpu_percent > self.amcl_cpu_peak:
                self.amcl_cpu_peak = self.amcl_cpu_percent
            
            # Reset peak every 10 seconds
            self.peak_reset_counter += 1
            if self.peak_reset_counter >= int(self.sample_rate * 10):
                self.amcl_cpu_peak = max(self.amcl_cpu_samples) if self.amcl_cpu_samples else 0.0
                self.peak_reset_counter = 0
        
        # Calculate rates
        scan_rate = self._calculate_rate(self.scan_timestamps)
        pose_rate = self._calculate_rate(self.pose_timestamps)
        
        # Calculate average latency
        avg_latency = 0.0
        if self.scan_pose_latencies:
            avg_latency = sum(self.scan_pose_latencies) / len(self.scan_pose_latencies)
        
        # Calculate accuracy metrics
        latest_position_error = 0.0
        latest_orientation_error = 0.0
        mean_position_error = 0.0
        mean_orientation_error = 0.0
        if self.position_errors:
            latest_position_error = self.position_errors[-1]
            mean_position_error = sum(self.position_errors) / len(self.position_errors)
        if self.orientation_errors:
            latest_orientation_error = self.orientation_errors[-1]
            mean_orientation_error = sum(self.orientation_errors) / len(self.orientation_errors)
        
        # Prepare data row
        data = {
            'timestamp_sec': now.seconds_nanoseconds()[0],
            'timestamp_nsec': now.seconds_nanoseconds()[1],
            'amcl_type': self.amcl_type,
            'min_particles': self.min_particles,
            'max_particles': self.max_particles,
            'max_beams': self.max_beams,
            'system_cpu_percent': round(self.cpu_percent, 1),
            'amcl_cpu_percent': round(self.amcl_cpu_percent, 1),
            'amcl_memory_mb': round(self.amcl_memory_mb, 1),
            'memory_percent': round(self.memory_percent, 1),
            'memory_used_mb': round(self.memory_used_mb, 1),
            'scan_to_pose_latency_ms': round(avg_latency, 2),
            'scan_rate_hz': round(scan_rate, 1),
            'pose_rate_hz': round(pose_rate, 1),
            'num_cores': self.num_cores,
            'position_error_m': round(latest_position_error, 4),
            'orientation_error_rad': round(latest_orientation_error, 4),
            'position_error_mean_m': round(mean_position_error, 4),
            'orientation_error_mean_rad': round(mean_orientation_error, 4),
        }
        
        # Add per-core CPU percentages
        for i, core_pct in enumerate(self.per_core_cpu):
            data[f'cpu_core_{i}_percent'] = round(core_pct, 1)
        
        # Add Jetson-specific metrics
        if self.is_jetson:
            gpu_percent, gpu_freq, emc_percent = self._get_jetson_gpu_usage()
            data['gpu_percent'] = round(gpu_percent, 1)
            data['gpu_freq_mhz'] = round(gpu_freq, 0)
            data['emc_percent'] = round(emc_percent, 1)
        
        # Write to CSV
        self.csv_writer.writerow(data)
        self.csv_file.flush()  # Ensure data is written
        
        # Calculate running average
        avg_amcl_cpu = sum(self.amcl_cpu_samples) / len(self.amcl_cpu_samples) if self.amcl_cpu_samples else 0.0
        
        # Log every 1 second (more frequent for spike visibility)
        if int(now.nanoseconds / 1e9) % 1 == 0 and hasattr(self, '_last_log_sec'):
            current_sec = int(now.nanoseconds / 1e9)
            if current_sec != self._last_log_sec:
                self._last_log_sec = current_sec
                
                # Main stats line
                msg = f'AMCL CPU: {self.amcl_cpu_percent:.1f}% now, {avg_amcl_cpu:.1f}% avg, {self.amcl_cpu_peak:.1f}% peak'
                self.get_logger().info(msg)
                
                # System stats line
                msg = f'System: {self.cpu_percent:.1f}% total ({self.num_cores} cores) | Mem: {self.memory_percent:.1f}%'
                if self.is_jetson:
                    msg += f' | GPU: {data.get("gpu_percent", 0):.1f}%'
                self.get_logger().info(msg)
                
                # Per-core CPU usage - each core on its own line
                if self.per_core_cpu:
                    for i, core_pct in enumerate(self.per_core_cpu):
                        bar_len = int(core_pct / 5)  # Scale to 20 chars max
                        bar = '█' * bar_len + '░' * (20 - bar_len)
                        self.get_logger().info(f'  Core {i}: [{bar}] {core_pct:5.1f}%')
                
                # Latency/rate line
                msg = f'Pose: {pose_rate:.1f} Hz | Scan: {scan_rate:.1f} Hz | Latency: {avg_latency:.1f} ms'
                self.get_logger().info(msg)
                
                # Accuracy line (if ground truth available)
                if self.position_errors:
                    import math
                    msg = f'Accuracy: pos_err={mean_position_error*100:.1f}cm | orient_err={math.degrees(mean_orientation_error):.1f}°'
                    self.get_logger().info(msg)
                
                self.get_logger().info('---')
        elif not hasattr(self, '_last_log_sec'):
            self._last_log_sec = int(now.nanoseconds / 1e9)
    
    def destroy_node(self):
        """Clean up on shutdown"""
        self.csv_file.close()
        self.get_logger().info(f'Performance data saved to: {self.csv_filename}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        pass  # Handle external shutdown gracefully
    finally:
        try:
            node.destroy_node()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
