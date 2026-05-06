import time

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from vesc_msgs.msg import VescStateStamped


FAULT_NAMES = {
    0: 'none',
    1: 'over_voltage',
    2: 'under_voltage',
    3: 'drv8302',
    4: 'abs_over_current',
    5: 'over_temp_fet',
    6: 'over_temp_motor',
}


class SystemMonitor(Node):
    def __init__(self):
        super().__init__('system_monitor')

        self.declare_parameter('vesc_topic', '/sensors/core')
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('monitor_vesc', True)
        self.declare_parameter('monitor_drive', True)
        self.declare_parameter('drive_arm_on_first_message', True)
        self.declare_parameter('vesc_timeout_sec', 0.5)
        self.declare_parameter('drive_timeout_sec', 0.15)
        self.declare_parameter('startup_grace_sec', 5.0)
        self.declare_parameter('check_period_sec', 0.05)
        self.declare_parameter('error_repeat_sec', 2.0)

        self.vesc_topic = self.get_parameter('vesc_topic').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.monitor_vesc = bool(self.get_parameter('monitor_vesc').value)
        self.monitor_drive = bool(self.get_parameter('monitor_drive').value)
        self.drive_arm_on_first_message = bool(
            self.get_parameter('drive_arm_on_first_message').value)
        self.vesc_timeout_sec = float(self.get_parameter('vesc_timeout_sec').value)
        self.drive_timeout_sec = float(self.get_parameter('drive_timeout_sec').value)
        self.startup_grace_sec = float(self.get_parameter('startup_grace_sec').value)
        self.error_repeat_sec = float(self.get_parameter('error_repeat_sec').value)
        check_period_sec = float(self.get_parameter('check_period_sec').value)

        self.start_time = time.monotonic()
        self.last_vesc_time = None
        self.last_drive_time = None
        self.vesc_ok = None
        self.drive_ok = None
        self.last_vesc_error_time = 0.0
        self.last_drive_error_time = 0.0
        self.last_fault_code = 0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self.monitor_vesc:
            self.vesc_sub = self.create_subscription(
                VescStateStamped,
                self.vesc_topic,
                self._vesc_callback,
                qos,
            )

        if self.monitor_drive:
            self.drive_sub = self.create_subscription(
                AckermannDriveStamped,
                self.drive_topic,
                self._drive_callback,
                qos,
            )

        self.timer = self.create_timer(check_period_sec, self._check_health)
        self.get_logger().info(
            'System monitor started: '
            f'vesc={self.vesc_topic} timeout={self.vesc_timeout_sec:.2f}s, '
            f'drive={self.drive_topic} timeout={self.drive_timeout_sec:.2f}s'
        )

    def _vesc_callback(self, msg):
        self.last_vesc_time = time.monotonic()
        fault_code = int(msg.state.fault_code)

        if self.vesc_ok is not True:
            self.get_logger().info(f'Vesc telemetry active on {self.vesc_topic}')
        self.vesc_ok = True

        if fault_code != 0 and fault_code != self.last_fault_code:
            fault_name = FAULT_NAMES.get(fault_code, 'unknown')
            self.get_logger().error(
                f'Vesc fault reported: code={fault_code} ({fault_name})'
            )
        elif fault_code == 0 and self.last_fault_code != 0:
            self.get_logger().info('Vesc fault cleared')

        self.last_fault_code = fault_code

    def _drive_callback(self, _msg):
        self.last_drive_time = time.monotonic()

        if self.drive_ok is not True:
            self.get_logger().info(f'Drive commands active on {self.drive_topic}')
        self.drive_ok = True

    def _check_health(self):
        now = time.monotonic()

        if self.monitor_vesc:
            self._check_topic(
                now=now,
                label='VESC telemetry',
                topic=self.vesc_topic,
                last_time=self.last_vesc_time,
                timeout_sec=self.vesc_timeout_sec,
                startup_missing_msg='no VESC telemetry received',
                stale_msg='VESC telemetry stopped',
                ok_attr='vesc_ok',
                last_error_attr='last_vesc_error_time',
            )

        if self.monitor_drive:
            self._check_topic(
                now=now,
                label='drive command',
                topic=self.drive_topic,
                last_time=self.last_drive_time,
                timeout_sec=self.drive_timeout_sec,
                startup_missing_msg='no /drive command received',
                stale_msg='/drive command stream stopped',
                ok_attr='drive_ok',
                last_error_attr='last_drive_error_time',
                arm_on_first_message=self.drive_arm_on_first_message,
            )

    def _check_topic(
        self,
        *,
        now,
        label,
        topic,
        last_time,
        timeout_sec,
        startup_missing_msg,
        stale_msg,
        ok_attr,
        last_error_attr,
        arm_on_first_message=False,
    ):
        ok_state = getattr(self, ok_attr)
        last_error_time = getattr(self, last_error_attr)

        if last_time is None:
            if arm_on_first_message:
                return
            age = now - self.start_time
            if age < self.startup_grace_sec:
                return
            message = startup_missing_msg
        else:
            age = now - last_time
            if age <= timeout_sec:
                if ok_state is False:
                    self.get_logger().info(
                        f'{label} recovered after gap; latest age={age:.3f}s'
                    )
                setattr(self, ok_attr, True)
                return
            message = stale_msg

        if ok_state is not False or (now - last_error_time) >= self.error_repeat_sec:
            publishers = self.count_publishers(topic)
            self.get_logger().error(
                f'{message}: age={age:.3f}s > timeout={timeout_sec:.3f}s, '
                f'publishers={publishers}, topic={topic}'
            )
            setattr(self, last_error_attr, now)
        setattr(self, ok_attr, False)


def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
