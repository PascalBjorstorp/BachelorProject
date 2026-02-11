#!/usr/bin/env python3
"""
Scan Filter Node for F1Tenth LiDAR

Preprocesses LaserScan data before sending to AMCL:
  - Range clipping (removes too-close or too-far readings)
  - Downsampling (reduces point count for faster processing)
  - Optional median filtering for noise reduction

Subscribes: /scan_raw (raw LiDAR data)
Publishes:  /scan (filtered data for AMCL)

Usage:
  ros2 run f1tenth_localization scan_filter.py
  
  # With custom settings
  ros2 run f1tenth_localization scan_filter.py --ros-args \
    -p range_min:=0.15 -p range_max:=8.0 -p downsample_factor:=4

Launch file usage:
  See scan_filter.launch.py for integration with LiDAR driver
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from typing import List


class ScanFilterNode(Node):
    def __init__(self):
        super().__init__('scan_filter')
        
        # =====================================================================
        # Parameters
        # =====================================================================
        
        # Range clipping
        self.declare_parameter('range_min', 0.1)  # Ignore readings closer than this (meters)
        self.declare_parameter('range_max', 10.0)  # Ignore readings farther than this (meters)
        
        # Downsampling: keep every Nth point (1 = no downsampling, 4 = 1080→270 points)
        self.declare_parameter('downsample_factor', 4)
        
        # Shadow filter: removes edge artifacts at obstacle boundaries
        # When enabled, detects sudden range jumps between adjacent beams and marks them invalid
        self.declare_parameter('shadow_filter_enabled', True)
        self.declare_parameter('shadow_filter_threshold', 0.3)  # Min range jump to trigger (meters)
        self.declare_parameter('shadow_filter_window', 1)  # Number of points to invalidate around edges
        
        # Topics
        self.declare_parameter('input_topic', '/scan_raw')
        self.declare_parameter('output_topic', '/scan')
        
        # Get parameters
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.downsample_factor = max(1, self.get_parameter('downsample_factor').value)
        self.shadow_enabled = self.get_parameter('shadow_filter_enabled').value
        self.shadow_threshold = self.get_parameter('shadow_filter_threshold').value
        self.shadow_window = self.get_parameter('shadow_filter_window').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # =====================================================================
        # Publishers and Subscribers
        # =====================================================================
        self.scan_pub = self.create_publisher(LaserScan, output_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, input_topic, self.scan_callback, 10
        )
        
        # Stats for logging
        self.scan_count = 0
        self.clipped_count = 0
        self.shadow_count = 0
        
        # Log configuration
        self.get_logger().info('Scan Filter Node Started')
        self.get_logger().info(f'  Input:  {input_topic}')
        self.get_logger().info(f'  Output: {output_topic}')
        self.get_logger().info(f'  Range:  {self.range_min}m - {self.range_max}m')
        self.get_logger().info(f'  Downsample: {self.downsample_factor}x (1080 → {1080 // self.downsample_factor} points)')
        if self.shadow_enabled:
            self.get_logger().info(f'  Shadow filter: enabled (threshold={self.shadow_threshold}m, window={self.shadow_window})')
        else:
            self.get_logger().info(f'  Shadow filter: disabled')
    
    def apply_shadow_filter(self, ranges: np.ndarray) -> np.ndarray:
        """
        Shadow filter: Removes edge artifacts at obstacle boundaries.
        
        When a laser beam grazes the edge of an obstacle, it can produce a false
        reading somewhere between the obstacle distance and the background distance.
        This filter detects large range discontinuities between adjacent beams and
        invalidates the points near those edges.
        
        Args:
            ranges: Array of range values
            
        Returns:
            Filtered ranges with shadow points set to inf
        """
        if not self.shadow_enabled:
            return ranges
        
        filtered = np.copy(ranges)
        n = len(ranges)
        
        # Find large discontinuities (edges)
        for i in range(1, n):
            prev_r = ranges[i - 1]
            curr_r = ranges[i]
            
            # Skip if either reading is already invalid
            if not np.isfinite(prev_r) or not np.isfinite(curr_r):
                continue
            
            # Check for sudden range jump (edge detected)
            jump = abs(curr_r - prev_r)
            if jump > self.shadow_threshold:
                # The shorter reading is likely the foreground object (valid)
                # The longer reading near the edge might be a shadow artifact
                # Invalidate points near the edge on the "far" side
                
                # Determine which side is the background (larger range)
                if curr_r > prev_r:
                    # Current point is further - it might be shadow, invalidate it
                    for w in range(self.shadow_window + 1):
                        if i + w < n:
                            filtered[i + w] = float('inf')
                            self.shadow_count += 1
                else:
                    # Previous point is further - invalidate points before
                    for w in range(self.shadow_window + 1):
                        if i - 1 - w >= 0:
                            filtered[i - 1 - w] = float('inf')
                            self.shadow_count += 1
        
        return filtered
    
    def apply_range_clipping(self, ranges: np.ndarray) -> np.ndarray:
        """Clip ranges outside valid range to inf (invalid)."""
        clipped = np.copy(ranges)
        
        # Mark out-of-range readings as invalid
        invalid_mask = (ranges < self.range_min) | (ranges > self.range_max)
        clipped[invalid_mask] = float('inf')
        
        # Track stats
        self.clipped_count += np.sum(invalid_mask)
        
        return clipped
    
    def apply_downsampling(self, ranges: np.ndarray, intensities: List[float] = None) -> tuple:
        """Downsample by taking every Nth point."""
        if self.downsample_factor <= 1:
            return ranges, intensities
        
        downsampled_ranges = ranges[::self.downsample_factor]
        
        if intensities is not None and len(intensities) > 0:
            intensities_array = np.array(intensities)
            downsampled_intensities = intensities_array[::self.downsample_factor].tolist()
        else:
            downsampled_intensities = []
        
        return downsampled_ranges, downsampled_intensities
    
    def scan_callback(self, msg: LaserScan):
        """Process incoming scan and publish filtered version."""
        self.scan_count += 1
        
        # Convert to numpy
        ranges = np.array(msg.ranges, dtype=np.float32)
        
        # Step 1: Shadow filter (remove edge artifacts)
        ranges = self.apply_shadow_filter(ranges)
        
        # Step 2: Range clipping
        ranges = self.apply_range_clipping(ranges)
        
        # Step 3: Downsampling
        ranges, intensities = self.apply_downsampling(ranges, list(msg.intensities))
        
        # Create output message
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        
        # Update scan geometry for downsampled data
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment * self.downsample_factor
        filtered_msg.time_increment = msg.time_increment * self.downsample_factor
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = self.range_min
        filtered_msg.range_max = self.range_max
        filtered_msg.ranges = ranges.tolist()
        filtered_msg.intensities = intensities
        
        # Publish
        self.scan_pub.publish(filtered_msg)
        
        # Log stats periodically (every 100 scans = ~2.5 seconds at 40Hz)
        if self.scan_count % 100 == 0:
            avg_clipped = self.clipped_count / 100
            avg_shadow = self.shadow_count / 100
            self.get_logger().info(
                f'Processed {self.scan_count} scans | '
                f'Clipped: {avg_clipped:.0f}/scan | '
                f'Shadow: {avg_shadow:.0f}/scan | '
                f'Output: {len(ranges)} pts'
            )
            self.clipped_count = 0
            self.shadow_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScanFilterNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
