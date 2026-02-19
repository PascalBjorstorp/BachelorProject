#!/usr/bin/env python3
"""
Keyboard teleop for F1TENTH car.
Publishes AckermannDriveStamped to /teleop topic (high priority on mux).

Controls:
    W / Up    : increase speed
    S / Down  : decrease speed
    A / Left  : steer left
    D / Right : steer right
    Space     : emergency stop (zero speed + zero steering)
    Q         : quit
"""

import sys
import termios
import tty
import select
import time

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped


SPEED_STEP = 0.5      # m/s per keypress
STEER_STEP = 0.05     # rad per keypress
MAX_SPEED = 4.0       # m/s
MAX_STEER = 0.40      # rad (~23 deg)
PUBLISH_RATE = 200.0   # Hz

HELP_TEXT = """
╔══════════════════════════════════════════╗
║       F1TENTH Keyboard Teleop            ║
╠══════════════════════════════════════════╣
║  W / ↑    : increase speed               ║
║  S / ↓    : decrease speed               ║
║  A / ←    : steer left                   ║
║  D / →    : steer right                  ║
║  Space    : STOP (zero speed+steering)   ║
║  Q / Esc  : quit                         ║
╠══════════════════════════════════════════╣
║  Speed step : {speed_step:.1f} m/s                  ║
║  Steer step : {steer_step:.2f} rad                 ║
║  Max speed  : {max_speed:.1f} m/s                  ║
║  Max steer  : {max_steer:.2f} rad ({max_steer_deg:.0f}°)            ║
╚══════════════════════════════════════════╝
"""


def get_key(timeout=0.05):
    """Read a single keypress (non-blocking)."""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Handle escape sequences (arrow keys)
            if key == '\x1b':
                key2 = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                key3 = sys.stdin.read(1) if select.select([sys.stdin], [], [], 0.01)[0] else ''
                if key2 == '[':
                    if key3 == 'A': return 'UP'
                    elif key3 == 'B': return 'DOWN'
                    elif key3 == 'C': return 'RIGHT'
                    elif key3 == 'D': return 'LEFT'
                return 'ESC'
            return key
        return None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('speed_step', SPEED_STEP)
        self.declare_parameter('steer_step', STEER_STEP)
        self.declare_parameter('max_speed', MAX_SPEED)
        self.declare_parameter('max_steer', MAX_STEER)
        self.declare_parameter('topic', 'teleop')

        self.speed_step = self.get_parameter('speed_step').value
        self.steer_step = self.get_parameter('steer_step').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steer = self.get_parameter('max_steer').value
        topic = self.get_parameter('topic').value

        self.pub = self.create_publisher(AckermannDriveStamped, topic, 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.publish_cmd)

        self.speed = 0.0
        self.steering = 0.0

    def publish_cmd(self):
        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering
        self.pub.publish(msg)

    def run(self):
        import math
        print(HELP_TEXT.format(
            speed_step=self.speed_step,
            steer_step=self.steer_step,
            max_speed=self.max_speed,
            max_steer=self.max_steer,
            max_steer_deg=math.degrees(self.max_steer)
        ))

        try:
            while rclpy.ok():
                key = get_key(timeout=1.0 / PUBLISH_RATE)
                rclpy.spin_once(self, timeout_sec=0)

                if key is None:
                    continue

                if key in ('w', 'W', 'UP'):
                    self.speed = min(self.speed + self.speed_step, self.max_speed)
                elif key in ('s', 'S', 'DOWN'):
                    self.speed = max(self.speed - self.speed_step, -self.max_speed)
                elif key in ('a', 'A', 'LEFT'):
                    self.steering = min(self.steering + self.steer_step, self.max_steer)
                elif key in ('d', 'D', 'RIGHT'):
                    self.steering = max(self.steering - self.steer_step, -self.max_steer)
                elif key == ' ':
                    self.speed = 0.0
                    self.steering = 0.0
                elif key in ('q', 'Q', 'ESC', '\x03'):  # q, Q, Esc, Ctrl+C
                    self.speed = 0.0
                    self.steering = 0.0
                    self.publish_cmd()
                    time.sleep(0.1)
                    break

                # Status line
                direction = "FWD" if self.speed > 0 else ("REV" if self.speed < 0 else "STOP")
                steer_dir = "LEFT" if self.steering > 0 else ("RIGHT" if self.steering < 0 else "CENTER")
                sys.stdout.write(
                    f'\r  Speed: {self.speed:+5.1f} m/s ({direction})  |  '
                    f'Steer: {self.steering:+5.2f} rad ({steer_dir})    '
                )
                sys.stdout.flush()

        except KeyboardInterrupt:
            pass
        finally:
            # Send stop command
            self.speed = 0.0
            self.steering = 0.0
            self.publish_cmd()
            print('\n\nStopped.')


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
