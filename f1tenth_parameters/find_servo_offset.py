#!/usr/bin/env python3
"""
Interactive servo center finder.

Drives slowly forward while letting you adjust the servo offset
with keyboard input until the car goes straight.

Usage:
    python3 find_servo_center.py [--speed 0.5]
    
Controls:
    + / =   : increase servo center by 0.005
    - / _   : decrease servo center by 0.005
    ] / }   : increase by 0.001 (fine)
    [ / {   : decrease by 0.001 (fine)
    s       : start/stop driving
    q       : quit (and print final value)
"""

import argparse
import sys
import termios
import tty
import time
import threading

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

# Current default from vesc.yaml
DEFAULT_OFFSET = 0.5390
DEFAULT_GAIN = -0.915


class ServoCenterFinder(Node):
    def __init__(self, speed):
        super().__init__('servo_center_finder')
        
        self.speed = speed
        self.servo_offset = DEFAULT_OFFSET
        self.driving = False
        
        self.pub = self.create_publisher(
            AckermannDriveStamped, 'drive', 10)
        
        self.odom_sub = self.create_subscription(
            Odometry, 'ego_racecar/odom', self._odom_cb, 10)
        
        self.odom_vx = 0.0
        self.odom_yaw_rate = 0.0
        
        # Timer to publish commands at 20Hz
        self.timer = self.create_timer(0.05, self._timer_cb)
    
    def _odom_cb(self, msg):
        self.odom_vx = msg.twist.twist.linear.x
        self.odom_yaw_rate = msg.twist.twist.angular.z
    
    def _timer_cb(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        if self.driving:
            msg.drive.speed = self.speed
        else:
            msg.drive.speed = 0.0
        # Command steering angle = 0 (straight). The servo value sent
        # to the VESC will be: gain * 0 + offset = offset
        msg.drive.steering_angle = 0.0
        self.pub.publish(msg)
    
    def adjust_offset(self, delta):
        self.servo_offset += delta
        # Clamp to valid servo range
        self.servo_offset = max(0.0, min(1.0, self.servo_offset))


def get_key():
    """Read a single keypress."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def main():
    parser = argparse.ArgumentParser(description='Find servo center interactively')
    parser.add_argument('--speed', type=float, default=4.0,
                        help='Driving speed in m/s (default: 1.5, needs >1.0 for sensorless VESC)')
    args = parser.parse_args()
    
    rclpy.init()
    node = ServoCenterFinder(args.speed)
    
    # Spin ROS in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    print("=" * 60)
    print("SERVO CENTER FINDER")
    print("=" * 60)
    print(f"Starting offset: {node.servo_offset:.4f}")
    print(f"Speed: {args.speed} m/s")
    print()
    print("NOTE: This adjusts steering_angle_to_servo_offset in vesc.yaml.")
    print("      The ackermann_to_vesc node converts angle -> servo using:")
    print(f"      servo = {DEFAULT_GAIN} * angle + offset")
    print()
    print("IMPORTANT: Changes here only affect the offset parameter.")
    print("           The ackermann_to_vesc node uses the LOADED parameter,")
    print("           so we send servo commands directly after finding center.")
    print()
    print("Controls:")
    print("  s     : start/stop driving")
    print("  a     : offset +0.005 (coarse right)")
    print("  d     : offset -0.005 (coarse left)")
    print("  ]/}   : offset +0.001 (fine right)")
    print("  [/{   : offset -0.001 (fine left)")
    print("  q     : quit and print final value")
    print("=" * 60)
    
    # Since the ackermann_to_vesc node has its own copy of the offset,
    # we need to dynamically set the parameter on the node OR compute
    # the equivalent steering_angle that produces our desired servo value.
    # Approach: command a small steering angle that moves the servo to
    # our desired center position:
    #   desired_servo = our_offset
    #   ackermann_to_vesc computes: servo = gain * angle + loaded_offset
    #   So: angle = (desired_servo - loaded_offset) / gain
    
    loaded_offset = DEFAULT_OFFSET  # What ackermann_to_vesc has loaded
    
    try:
        while True:
            key = get_key()
            
            if key == 'q':
                break
            elif key == 's':
                node.driving = not node.driving
                state = "DRIVING" if node.driving else "STOPPED"
                print(f"\r{state}  offset={node.servo_offset:.4f}  "
                      f"v={node.odom_vx:.2f}m/s  yaw_rate={node.odom_yaw_rate:.3f}rad/s    ")
            elif key in ('d'):
                node.adjust_offset(0.005)
            elif key in ('a'):
                node.adjust_offset(-0.005)
            elif key in ('z'):
                node.adjust_offset(0.001)
            elif key in ('c'):
                node.adjust_offset(-0.001)
            else:
                continue
            
            # Compute the steering angle that produces our desired servo center
            # servo = gain * angle + loaded_offset
            # angle = (servo - loaded_offset) / gain
            correction_angle = (node.servo_offset - loaded_offset) / DEFAULT_GAIN
            
            # Update the command to use this correction angle instead of 0
            # We override the timer callback's angle
            node._correction_angle = correction_angle
            
            print(f"\roffset={node.servo_offset:.4f}  "
                  f"correction_angle={correction_angle:.4f}rad  "
                  f"{'DRIVING' if node.driving else 'STOPPED'}  "
                  f"v={node.odom_vx:.2f}  yaw={node.odom_yaw_rate:.3f}    ", end='')
    
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the car
        node.driving = False
        time.sleep(0.2)
        
        print()
        print()
        print("=" * 60)
        print(f"FINAL servo offset: {node.servo_offset:.4f}")
        print(f"(was: {DEFAULT_OFFSET:.4f}, delta: {node.servo_offset - DEFAULT_OFFSET:+.4f})")
        print()
        print("Update vesc.yaml with:")
        print(f"  steering_angle_to_servo_offset: {node.servo_offset:.4f}")
        print("=" * 60)
        
        node.destroy_node()
        rclpy.shutdown()


# Patch the timer callback to use correction angle
_orig_timer_cb = ServoCenterFinder._timer_cb

def _patched_timer_cb(self):
    msg = AckermannDriveStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    if self.driving:
        msg.drive.speed = self.speed
    else:
        msg.drive.speed = 0.0
    # Use correction angle to achieve desired servo position
    msg.drive.steering_angle = getattr(self, '_correction_angle', 0.0)
    self.pub.publish(msg)

ServoCenterFinder._timer_cb = _patched_timer_cb


if __name__ == '__main__':
    main()
