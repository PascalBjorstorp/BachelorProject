#!/usr/bin/env python3
"""
GPU AMCL Node Entry Point

This script is the main executable for the GPU AMCL ROS2 node.
It can be run directly or through the launch file.

Usage:
    ros2 run gpu_amcl gpu_amcl_node.py
    ros2 run gpu_amcl gpu_amcl_node.py --ros-args -p num_particles:=1000
"""

import rclpy
from gpu_amcl.ros import GPUAMCLNode


def main(args=None):
    rclpy.init(args=args)
    
    node = GPUAMCLNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
