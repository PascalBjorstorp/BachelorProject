#!/usr/bin/env python3
"""
Interactive Servo Limit Finder for F1/10th Car

Finds the physical servo_min and servo_max values by letting you
sweep the servo from center outward and mark where the steering
linkage hits its mechanical stops.

IMPORTANT: Put the car on a stand with wheels OFF the ground before running.
           No motor commands are sent — only servo position.

The script publishes directly to /commands/servo/position (Float64),
bypassing ackermann_to_vesc, so it controls the servo duty cycle directly.

Usage:
    python3 find_servo_limits.py
    python3 find_servo_limits.py --center 0.546   # custom center position

Controls:
    a / d   : move servo left / right by 0.01 (coarse)
    z / c   : move servo left / right by 0.002 (fine)
    1       : mark current position as LEFT limit (servo_max)
    2       : mark current position as RIGHT limit (servo_min)
    r       : reset to center
    q       : quit and print results
"""

import argparse
import sys
import termios
import tty
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

# Defaults from vesc.yaml
DEFAULT_SERVO_OFFSET = 0.5460
DEFAULT_SERVO_GAIN = -0.6960


class ServoLimitFinder(Node):
    def __init__(self, center):
        super().__init__('servo_limit_finder')

        self.center = center
        self.servo_pos = center
        self.left_limit = None   # servo_max (high servo value = left with negative gain)
        self.right_limit = None  # servo_min (low servo value = right with negative gain)
        self.paused = False
        self.saved_pos = None    # position saved when pausing

        self.pub = self.create_publisher(
            Float64, 'commands/servo/position', 10)

        # Publish at 20 Hz
        self.timer = self.create_timer(0.05, self._timer_cb)

    def _timer_cb(self):
        msg = Float64()
        # When paused, publish center to relax the servo
        msg.data = self.center if self.paused else self.servo_pos
        self.pub.publish(msg)

    def adjust(self, delta):
        self.servo_pos += delta
        self.servo_pos = max(0.0, min(1.0, self.servo_pos))

    def back_off(self, amount=0.005):
        """Move servo_pos toward center by `amount`."""
        if self.servo_pos > self.center:
            self.servo_pos = max(self.center, self.servo_pos - amount)
        else:
            self.servo_pos = min(self.center, self.servo_pos + amount)

    def toggle_pause(self):
        """Toggle pause. When pausing, save position. When resuming, restore backed off."""
        if not self.paused:
            # Pause: save current position, publish center to relax servo
            self.saved_pos = self.servo_pos
            self.paused = True
        else:
            # Resume: restore position backed off 0.01 toward center
            if self.saved_pos is not None:
                self.servo_pos = self.saved_pos
                self.back_off(0.01)
                self.saved_pos = None
            self.paused = False

    def servo_to_angle_deg(self, servo_val):
        """Convert servo value to steering angle (degrees)."""
        angle_rad = (servo_val - self.center) / DEFAULT_SERVO_GAIN
        return angle_rad * 180.0 / 3.14159265

    def reset(self):
        self.servo_pos = self.center


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


def print_status(node):
    """Print current status line."""
    angle = node.servo_to_angle_deg(node.servo_pos)

    left_str = f'{node.left_limit:.3f}' if node.left_limit else '---'
    right_str = f'{node.right_limit:.3f}' if node.right_limit else '---'
    pause_str = ' [PAUSED]' if node.paused else ''
    saved_str = f' (saved={node.saved_pos:.4f})' if node.saved_pos else ''

    print(f'\rservo={node.servo_pos:.4f}  angle={angle:+6.1f}deg  '
          f'LEFT={left_str}  RIGHT={right_str}{pause_str}{saved_str}    ', end='')


def main():
    parser = argparse.ArgumentParser(
        description='Find servo min/max limits interactively',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Put the car on a stand with wheels off the ground.\n'
            'Sweep the servo outward from center and mark where the\n'
            'steering linkage hits its mechanical stops.\n'
        ))
    parser.add_argument('--center', type=float, default=DEFAULT_SERVO_OFFSET,
                        help=f'Servo center position (default: {DEFAULT_SERVO_OFFSET})')
    args = parser.parse_args()

    rclpy.init()
    node = ServoLimitFinder(args.center)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print('=' * 65)
    print('  SERVO LIMIT FINDER')
    print('=' * 65)
    print(f'  Center (offset): {args.center:.4f}')
    print(f'  Servo gain:      {DEFAULT_SERVO_GAIN}')
    print()
    print('  PUT THE CAR ON A STAND — wheels off the ground!')
    print()
    print('  Controls:')
    print('    a / d   : servo -0.01 / +0.01 (coarse)')
    print('    z / c   : servo -0.002 / +0.002 (fine)')
    print('    x / v   : servo -0.0005 / +0.0005 (ultra-fine)')
    print('    SPACE   : pause/resume (relaxes servo at center, saves position)')
    print('    b       : back off 0.005 toward center (without pausing)')
    print('    1       : mark LEFT limit  (the max servo value before lock)')
    print('    2       : mark RIGHT limit (the min servo value before lock)')
    print('    r       : reset to center')
    print('    q       : quit and show results')
    print()
    print('  PROCEDURE:')
    print('    1. Sweep slowly outward from center with a/d or z/c')
    print('    2. If the servo buzzes, press SPACE to pause (relaxes servo)')
    print('    3. Press SPACE again to resume (backed off slightly)')
    print('    4. Use x/v for ultra-fine adjustment near the limit')
    print('    5. Mark the last non-buzzing position with 1 or 2')
    print('=' * 65)
    print()

    print_status(node)

    try:
        while True:
            key = get_key()

            if key == 'q':
                break
            elif key == 'a':
                node.adjust(-0.01)
            elif key == 'd':
                node.adjust(0.01)
            elif key == 'z':
                node.adjust(-0.002)
            elif key == 'c':
                node.adjust(0.002)
            elif key == 'x':
                node.adjust(-0.0005)
            elif key == 'v':
                node.adjust(0.0005)
            elif key == ' ':
                node.toggle_pause()
                state = 'PAUSED — servo at center' if node.paused else 'RESUMED'
                print(f'\n  >> {state}')
            elif key == 'b':
                node.back_off(0.005)
                print(f'\n  >> Backed off toward center')
            elif key == '1':
                # Left limit = higher servo value (with negative gain, higher servo = more left)
                node.left_limit = node.servo_pos
                print(f'\n  >> LEFT limit marked at servo={node.servo_pos:.4f} '
                      f'({node.servo_to_angle_deg(node.servo_pos):+.1f}deg)')
            elif key == '2':
                # Right limit = lower servo value
                node.right_limit = node.servo_pos
                print(f'\n  >> RIGHT limit marked at servo={node.servo_pos:.4f} '
                      f'({node.servo_to_angle_deg(node.servo_pos):+.1f}deg)')
            elif key == 'r':
                node.reset()
            else:
                continue

            print_status(node)

    except KeyboardInterrupt:
        pass
    finally:
        # Return to center
        node.reset()
        time.sleep(0.3)

        print()
        print()
        print('=' * 65)
        print('  RESULTS')
        print('=' * 65)

        if node.left_limit and node.right_limit:
            # With negative gain: left = high servo, right = low servo
            servo_max = max(node.left_limit, node.right_limit)
            servo_min = min(node.left_limit, node.right_limit)

            # Add a small safety margin (0.02)
            margin = 0.02
            safe_max = servo_max - margin
            safe_min = servo_min + margin

            angle_left = node.servo_to_angle_deg(servo_max)
            angle_right = node.servo_to_angle_deg(servo_min)
            safe_angle_left = node.servo_to_angle_deg(safe_max)
            safe_angle_right = node.servo_to_angle_deg(safe_min)

            print(f'  Lock-to-lock servo range: [{servo_min:.4f}, {servo_max:.4f}]')
            print(f'  Lock-to-lock angle range: [{angle_right:+.1f}deg, {angle_left:+.1f}deg]')
            print()
            print(f'  With {margin} safety margin:')
            print(f'    servo_min: {safe_min:.3f}')
            print(f'    servo_max: {safe_max:.3f}')
            print(f'    angle range: [{safe_angle_right:+.1f}deg, {safe_angle_left:+.1f}deg]')
            print()
            print(f'  Update vesc.yaml:')
            print(f'    servo_min: {safe_min:.3f}')
            print(f'    servo_max: {safe_max:.3f}')
        elif node.left_limit:
            print(f'  LEFT limit:  servo={node.left_limit:.4f} '
                  f'({node.servo_to_angle_deg(node.left_limit):+.1f}deg)')
            print(f'  RIGHT limit: not marked')
        elif node.right_limit:
            print(f'  LEFT limit:  not marked')
            print(f'  RIGHT limit: servo={node.right_limit:.4f} '
                  f'({node.servo_to_angle_deg(node.right_limit):+.1f}deg)')
        else:
            print(f'  No limits marked.')

        print('=' * 65)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
