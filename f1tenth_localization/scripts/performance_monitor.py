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
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, scan_topic, self.scan_callback, 10
        )
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, amcl_topic, self.pose_callback, 10
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
        
        # Search for AMCL process - look for the nav2_amcl executable specifically
        for proc in self.psutil.process_iter(['pid', 'name', 'cmdline', 'exe']):
            try:
                # Check executable name first (most reliable)
                exe = proc.info.get('exe', '') or ''
                name = proc.info.get('name', '') or ''
                cmdline = proc.info.get('cmdline', []) or []
                
                # Match AMCL executable (could be named 'amcl' or path ending in /amcl)
                is_amcl = (
                    name == 'amcl' or
                    exe.endswith('/amcl') or
                    (cmdline and any('nav2_amcl' in str(c) or c.endswith('/amcl') for c in cmdline))
                )
                
                if is_amcl:
                    self.amcl_process = proc
                    self.get_logger().info(f'Found AMCL process: PID {proc.pid}, name={name}, exe={exe}')
                    # Initialize CPU measurement (first call returns 0, subsequent calls track delta)
                    proc.cpu_percent(interval=None)
                    return proc
            except (self.psutil.NoSuchProcess, self.psutil.AccessDenied, self.psutil.ZombieProcess):
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
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Record pose timestamp and calculate latency by correlating with the scan that produced it"""
        now = self.get_clock().now()
        self.last_pose_time = now
        
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
            if 0 < latency_ms < 5000:
                self.scan_pose_latencies.append(latency_ms)
                if len(self.scan_pose_latencies) > 100:
                    self.scan_pose_latencies.pop(0)
        
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
