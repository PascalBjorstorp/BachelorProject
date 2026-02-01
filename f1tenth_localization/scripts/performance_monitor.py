#!/usr/bin/env python3
"""
Performance Monitor for F1TENTH Localization

Monitors CPU, GPU (Jetson), memory usage, and localization latency.
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
        self.declare_parameter('sample_rate_hz', 10.0)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('odom_topic', '/odom')
        
        self.output_dir = self.get_parameter('output_dir').value
        self.sample_rate = self.get_parameter('sample_rate_hz').value
        scan_topic = self.get_parameter('scan_topic').value
        amcl_topic = self.get_parameter('amcl_pose_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Detect platform (Jetson vs regular PC)
        self.is_jetson = self._detect_jetson()
        platform = "Jetson" if self.is_jetson else "PC"
        self.get_logger().info(f'Detected platform: {platform}')
        
        # CSV file for logging
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(self.output_dir, f'performance_{timestamp}.csv')
        self.csv_file = open(self.csv_filename, 'w', newline='')
        
        # CSV headers
        headers = [
            'timestamp_sec', 'timestamp_nsec',
            'cpu_percent', 'memory_percent', 'memory_used_mb',
            'scan_to_pose_latency_ms', 'scan_rate_hz', 'pose_rate_hz',
        ]
        if self.is_jetson:
            headers.extend(['gpu_percent', 'gpu_freq_mhz', 'emc_percent'])
        
        self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=headers)
        self.csv_writer.writeheader()
        
        # State for latency calculation
        self.last_scan_time = None
        self.last_pose_time = None
        self.scan_timestamps = []  # Last N scan timestamps for rate calculation
        self.pose_timestamps = []  # Last N pose timestamps for rate calculation
        self.scan_pose_latencies = []  # Last N latencies
        
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
        
        # CPU tracking (requires psutil)
        self.cpu_percent = 0.0
        self.memory_percent = 0.0
        self.memory_used_mb = 0.0
        
        # Try to import psutil
        try:
            import psutil
            self.psutil = psutil
            self.get_logger().info('psutil available for CPU/memory monitoring')
        except ImportError:
            self.psutil = None
            self.get_logger().warn('psutil not available - install with: pip install psutil')
        
        self.get_logger().info(f'Performance Monitor started')
        self.get_logger().info(f'  Output: {self.csv_filename}')
        self.get_logger().info(f'  Sample rate: {self.sample_rate} Hz')
        
    def _detect_jetson(self):
        """Detect if running on Jetson by checking for tegra files"""
        return os.path.exists('/sys/devices/gpu.0') or \
               os.path.exists('/sys/class/thermal/thermal_zone0/type')
    
    def _get_jetson_gpu_usage(self):
        """Read GPU usage from Jetson tegrastats or sysfs"""
        gpu_percent = 0.0
        gpu_freq = 0.0
        emc_percent = 0.0
        
        try:
            # Try reading GPU load from sysfs (Jetson)
            gpu_load_path = '/sys/devices/gpu.0/load'
            if os.path.exists(gpu_load_path):
                with open(gpu_load_path, 'r') as f:
                    gpu_percent = float(f.read().strip()) / 10.0  # Value is in 0.1%
            
            # Try reading GPU frequency
            gpu_freq_path = '/sys/devices/gpu.0/devfreq/17000000.gv11b/cur_freq'
            if os.path.exists(gpu_freq_path):
                with open(gpu_freq_path, 'r') as f:
                    gpu_freq = float(f.read().strip()) / 1e6  # Convert to MHz
            
            # EMC (memory controller) usage - indicates memory bandwidth
            emc_path = '/sys/kernel/debug/clk/emc/clk_rate'
            if os.path.exists(emc_path):
                try:
                    with open(emc_path, 'r') as f:
                        emc_percent = float(f.read().strip()) / 1e6
                except:
                    pass
                    
        except Exception as e:
            self.get_logger().debug(f'Could not read Jetson GPU stats: {e}')
        
        return gpu_percent, gpu_freq, emc_percent
    
    def scan_callback(self, msg: LaserScan):
        """Record scan timestamp for latency calculation"""
        now = self.get_clock().now()
        self.last_scan_time = now
        
        # Track scan rate
        self.scan_timestamps.append(now.nanoseconds)
        if len(self.scan_timestamps) > 100:
            self.scan_timestamps.pop(0)
    
    def pose_callback(self, msg: PoseWithCovarianceStamped):
        """Record pose timestamp and calculate latency"""
        now = self.get_clock().now()
        self.last_pose_time = now
        
        # Calculate scan-to-pose latency
        if self.last_scan_time is not None:
            latency_ns = now.nanoseconds - self.last_scan_time.nanoseconds
            latency_ms = latency_ns / 1e6
            
            # Only record reasonable latencies (< 1 second)
            if 0 < latency_ms < 1000:
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
        
        # Get CPU and memory usage
        if self.psutil:
            self.cpu_percent = self.psutil.cpu_percent(interval=None)
            mem = self.psutil.virtual_memory()
            self.memory_percent = mem.percent
            self.memory_used_mb = mem.used / (1024 * 1024)
        
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
            'cpu_percent': round(self.cpu_percent, 1),
            'memory_percent': round(self.memory_percent, 1),
            'memory_used_mb': round(self.memory_used_mb, 1),
            'scan_to_pose_latency_ms': round(avg_latency, 2),
            'scan_rate_hz': round(scan_rate, 1),
            'pose_rate_hz': round(pose_rate, 1),
        }
        
        # Add Jetson-specific metrics
        if self.is_jetson:
            gpu_percent, gpu_freq, emc_percent = self._get_jetson_gpu_usage()
            data['gpu_percent'] = round(gpu_percent, 1)
            data['gpu_freq_mhz'] = round(gpu_freq, 0)
            data['emc_percent'] = round(emc_percent, 1)
        
        # Write to CSV
        self.csv_writer.writerow(data)
        self.csv_file.flush()  # Ensure data is written
        
        # Log periodically (every 5 seconds)
        if int(now.nanoseconds / 1e9) % 5 == 0:
            msg = f'CPU: {data["cpu_percent"]:.1f}%, Mem: {data["memory_percent"]:.1f}%, '
            msg += f'Scan rate: {scan_rate:.1f} Hz, Latency: {avg_latency:.1f} ms'
            if self.is_jetson:
                msg += f', GPU: {data.get("gpu_percent", 0):.1f}%'
            self.get_logger().info(msg)
    
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
