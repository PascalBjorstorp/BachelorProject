#!/usr/bin/env python3
"""
Motor Current & Thermal Characterization Test for F1/10th Car

Helps determine safe motor current limits by driving the car while
monitoring FET and motor temperatures.

TWO MODES:
    Straight-line (default): Drives straight, pauses between runs so you
        can reposition the car. Works in a hallway (needs ~10m for 4 m/s).
    Circle mode (--circle):  Drives in circles, stays in one area.
        Needs ~3×3 m open space.

USE THIS TO FIND WHAT TO SET IN VESC TOOL:
    1. Set a starting motor current limit in VESC Tool (e.g., 40A)
    2. Run this test
    3. Check the temperature report
    4. If temps are fine (< 70°C), increase the limit in VESC Tool and re-run
    5. If temps are too high (> 80°C), decrease the limit

SAFE TEMPERATURE GUIDELINES:
    FET temp < 70°C  : ✓ Fine, plenty of margin
    FET temp 70-85°C : ⚠ At the limit — adequate for racing
    FET temp > 85°C  : ✗ Reduce motor current limit
    Motor temp < 80°C: ✓ Fine for most brushless motors
    Motor temp > 100°C: ✗ Risk of demagnetization

Usage:
    python3 test_current_limits.py                         # straight, 4 m/s
    python3 test_current_limits.py --circle                # circle mode
    python3 test_current_limits.py --max-speed 3.0 --runs 5
    python3 test_current_limits.py --circle --steering 0.35
"""

import argparse
import time

import numpy as np
import rclpy

from common import TestNode


class CurrentLimitNode(TestNode):

    def __init__(self, args):
        columns = [
            'odom_vx', 'imu_ax',
            'motor_rpm', 'motor_current', 'input_current',
            'battery_voltage', 'temp_fet', 'temp_motor',
            'cmd_speed', 'phase'
        ]

        super().__init__(
            'current_limit_test',
            'current_limits',
            columns,
            max_speed=args.max_speed * 1.3,
            max_time=args.runs * (args.run_time + args.cool_time) + 300.0
        )

        self.max_speed = args.max_speed
        self.circle_mode = args.circle
        self.steering_angle = args.steering if args.circle else 0.0
        self.num_runs = args.runs
        self.run_time = args.run_time
        self.cool_time = args.cool_time

        # Temperature tracking (from VESC state)
        self.temp_fet = 0.0
        self.temp_motor = 0.0

        # Results
        self.run_results = []

    def _vesc_callback(self, msg):
        """Extended callback that also reads temperature."""
        self.battery_voltage = msg.state.voltage_input
        self.motor_rpm = msg.state.speed
        self.motor_current = msg.state.current_motor
        self.input_current = msg.state.current_input
        self.temp_fet = msg.state.temp_fet
        self.temp_motor = msg.state.temp_motor
        self.safety.update_battery(self.battery_voltage)

    def _estimate_travel_distance(self):
        """Rough estimate of forward travel for straight-line mode."""
        # Assume ~1.5s to reach target speed (linear accel), then constant
        accel_time = min(1.5, self.run_time)
        coast_time = max(0, self.run_time - accel_time)
        # Distance during acceleration (triangle) + coast (rectangle)
        dist = 0.5 * self.max_speed * accel_time + self.max_speed * coast_time
        # Add braking distance (~1s at half speed)
        dist += self.max_speed * 0.5
        return dist

    def run_test(self):
        """Execute the current limit characterization test."""
        if not self.wait_for_sensors():
            return False

        self.get_logger().info("=" * 65)
        self.get_logger().info("MOTOR CURRENT & THERMAL CHARACTERIZATION TEST")
        self.get_logger().info("=" * 65)
        self.get_logger().info(f"Speed command: {self.max_speed:.1f} m/s")

        if self.circle_mode:
            radius = 0.326 / abs(np.tan(self.steering_angle))
            self.get_logger().info(
                f"Mode: CIRCLE (steering {self.steering_angle:.2f} rad, "
                f"radius ≈ {radius:.2f} m)")
            self.get_logger().info(f"Space needed: ~{2 * radius + 0.5:.1f} × "
                                   f"{2 * radius + 0.5:.1f} m")
        else:
            est_dist = self._estimate_travel_distance()
            self.get_logger().info(f"Mode: STRAIGHT LINE")
            self.get_logger().info(
                f"Estimated travel per run: ~{est_dist:.1f} m "
                f"(including braking)")
            self.get_logger().info(
                "You will be prompted to reposition the car between runs.")

        self.get_logger().info(f"Runs: {self.num_runs}, Duration: {self.run_time:.1f}s each")
        self.get_logger().info(f"Cooling time: {self.cool_time:.1f}s")
        self.get_logger().info("=" * 65)

        # Check initial temperatures
        self.get_logger().info(f"\nInitial state:")
        self.get_logger().info(f"  FET:     {self.temp_fet:.1f}°C")
        self.get_logger().info(f"  Motor:   {self.temp_motor:.1f}°C")
        self.get_logger().info(f"  Battery: {self.battery_voltage:.1f}V")

        if self.temp_fet > 60:
            self.get_logger().warn("FET already warm! Let the VESC cool down first.")

        self.countdown(5)
        self.recorder.start()
        self.safety.start()
        self.test_running = True

        for run_idx in range(self.num_runs):
            if not self.test_running:
                break

            self.get_logger().info(f"\n--- Run {run_idx + 1}/{self.num_runs} ---")
            self.get_logger().info(
                f"FET: {self.temp_fet:.1f}°C, Motor: {self.temp_motor:.1f}°C")

            # Safety: abort if temperatures are too high
            if self.temp_fet > 85:
                self.get_logger().error(
                    "FET temperature > 85°C! Aborting for safety.")
                self.get_logger().error(
                    "Reduce motor current limit in VESC Tool.")
                break

            # ---- Drive phase ----
            current_samples = []
            input_current_samples = []
            ax_samples = []
            vx_samples = []
            temp_fet_samples = []
            temp_motor_samples = []

            phase_start = time.monotonic()
            while time.monotonic() - phase_start < self.run_time:
                rclpy.spin_once(self, timeout_sec=0.005)
                if not self.safety.check():
                    self.get_logger().error(
                        f"Safety abort: {self.safety.abort_reason}")
                    self.stop_car()
                    self.test_running = False
                    break

                self.send_command(self.max_speed, self.steering_angle)

                current_samples.append(self.motor_current)
                input_current_samples.append(self.input_current)
                ax_samples.append(self.imu_ax)
                vx_samples.append(self.odom_vx)
                temp_fet_samples.append(self.temp_fet)
                temp_motor_samples.append(self.temp_motor)

                self.recorder.record(
                    odom_vx=self.odom_vx, imu_ax=self.imu_ax,
                    motor_rpm=self.motor_rpm,
                    motor_current=self.motor_current,
                    input_current=self.input_current,
                    battery_voltage=self.battery_voltage,
                    temp_fet=self.temp_fet,
                    temp_motor=self.temp_motor,
                    cmd_speed=self.max_speed,
                    phase=f'run_{run_idx + 1}'
                )

            if not self.test_running:
                break

            # ---- Stop and record results ----
            self.stop_car()

            if current_samples:
                result = {
                    'run': run_idx + 1,
                    'peak_motor_I': max(current_samples),
                    'avg_motor_I': np.mean(current_samples),
                    'peak_battery_I': max(input_current_samples),
                    'avg_battery_I': np.mean(input_current_samples),
                    'peak_speed': max(vx_samples),
                    'temp_fet_start': temp_fet_samples[0],
                    'temp_fet_end': temp_fet_samples[-1],
                    'temp_motor_start': temp_motor_samples[0],
                    'temp_motor_end': temp_motor_samples[-1],
                    'temp_fet_rise': temp_fet_samples[-1] - temp_fet_samples[0],
                }
                self.run_results.append(result)

                self.get_logger().info(
                    f"  Peak I_motor: {result['peak_motor_I']:.1f}A, "
                    f"Avg: {result['avg_motor_I']:.1f}A")
                self.get_logger().info(
                    f"  Peak I_batt:  {result['peak_battery_I']:.1f}A, "
                    f"Avg: {result['avg_battery_I']:.1f}A")
                self.get_logger().info(
                    f"  FET temp: {result['temp_fet_start']:.1f} → "
                    f"{result['temp_fet_end']:.1f}°C "
                    f"(+{result['temp_fet_rise']:.1f}°C)")
                self.get_logger().info(
                    f"  Peak speed: {result['peak_speed']:.2f} m/s")

            # ---- Between-run pause ----
            if run_idx < self.num_runs - 1:
                # Active braking for 2s
                brake_start = time.monotonic()
                while time.monotonic() - brake_start < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.005)
                    self.send_command(0.0, 0.0)
                    self.recorder.record(
                        odom_vx=self.odom_vx, imu_ax=self.imu_ax,
                        motor_rpm=self.motor_rpm,
                        motor_current=self.motor_current,
                        input_current=self.input_current,
                        battery_voltage=self.battery_voltage,
                        temp_fet=self.temp_fet,
                        temp_motor=self.temp_motor,
                        cmd_speed=0.0,
                        phase=f'brake_{run_idx + 1}'
                    )

                if self.circle_mode:
                    # Circle mode: automatic cooling period
                    self.get_logger().info(
                        f"  Cooling for {self.cool_time:.0f}s...")
                    remaining = self.cool_time - 2.0
                    if remaining > 0:
                        cool_start = time.monotonic()
                        while time.monotonic() - cool_start < remaining:
                            rclpy.spin_once(self, timeout_sec=0.05)
                else:
                    # Straight-line mode: wait for user to reposition
                    self.get_logger().info("")
                    self.get_logger().info(
                        "  >>> Pick up the car, turn it around, and place "
                        "it at the start.")
                    self.get_logger().info(
                        f"  >>> FET: {self.temp_fet:.1f}°C  |  "
                        f"Let it cool if temps are high.")
                    input("  >>> Press ENTER when ready for the next run...")
                    # Keep subscriptions alive after user input
                    for _ in range(20):
                        rclpy.spin_once(self, timeout_sec=0.005)

        self.stop_car()
        time.sleep(0.5)
        self.stop_car()

        self.recorder.save()
        self.analyze()
        return True

    def analyze(self):
        """Analyze thermal characterization results."""
        if not self.run_results:
            self.get_logger().warn("No run data to analyze")
            return

        self.get_logger().info("")
        self.get_logger().info("=" * 65)
        self.get_logger().info("THERMAL CHARACTERIZATION RESULTS")
        self.get_logger().info("=" * 65)

        # Table header
        self.get_logger().info(
            f"\n{'Run':>4} {'I_peak':>7} {'I_avg':>6} {'I_batt':>7} "
            f"{'v_max':>6} {'FET°C':>10} {'Mot°C':>10} {'ΔFET':>5}")

        for r in self.run_results:
            self.get_logger().info(
                f"{r['run']:4d} {r['peak_motor_I']:7.1f} "
                f"{r['avg_motor_I']:6.1f} "
                f"{r['avg_battery_I']:7.1f} {r['peak_speed']:6.2f} "
                f"{r['temp_fet_start']:4.1f}→{r['temp_fet_end']:4.1f} "
                f"{r['temp_motor_start']:4.1f}→{r['temp_motor_end']:4.1f} "
                f"{r['temp_fet_rise']:+5.1f}")

        # Overall stats
        max_fet = max(r['temp_fet_end'] for r in self.run_results)
        max_mot = max(r['temp_motor_end'] for r in self.run_results)
        peak_I = max(r['peak_motor_I'] for r in self.run_results)
        avg_I = np.mean([r['avg_motor_I'] for r in self.run_results])
        total_fet_rise = (self.run_results[-1]['temp_fet_end'] -
                          self.run_results[0]['temp_fet_start'])

        self.get_logger().info(f"\n  Peak motor current:   {peak_I:.1f} A")
        self.get_logger().info(f"  Average motor current: {avg_I:.1f} A")
        self.get_logger().info(f"  Max FET temperature:   {max_fet:.1f}°C")
        self.get_logger().info(f"  Max motor temperature: {max_mot:.1f}°C")
        self.get_logger().info(
            f"  Total FET temp rise: {total_fet_rise:+.1f}°C "
            f"over {self.num_runs} runs")

        # Assessment
        self.get_logger().info(f"\n  ASSESSMENT:")
        if max_fet < 60:
            self.get_logger().info(
                f"  ✓ FET well within limits ({max_fet:.0f}°C < 60°C)")
            self.get_logger().info(
                f"    You can safely INCREASE the motor current limit.")
        elif max_fet < 75:
            self.get_logger().info(
                f"  ✓ FET temperature OK ({max_fet:.0f}°C < 75°C)")
            self.get_logger().info(
                f"    Current limit is reasonable. Small increases okay.")
        elif max_fet < 85:
            self.get_logger().info(
                f"  ⚠ FET getting warm ({max_fet:.0f}°C)")
            self.get_logger().info(
                f"    At the edge. Don't increase further.")
            self.get_logger().info(
                f"    For sustained racing, consider DECREASING.")
        else:
            self.get_logger().info(
                f"  ✗ FET TOO HIGH ({max_fet:.0f}°C > 85°C)")
            self.get_logger().info(
                f"    DECREASE the motor current limit in VESC Tool!")

        if total_fet_rise > 15:
            self.get_logger().info(
                f"  ⚠ Temperature still climbing between runs.")
            self.get_logger().info(
                f"    Sustained driving will go higher. Add margin.")

        # Battery analysis
        peak_batt = max(r['peak_battery_I'] for r in self.run_results)
        self.get_logger().info(f"\n  Battery peak current: {peak_batt:.1f} A")
        self.get_logger().info(
            f"  Battery voltage at end: {self.battery_voltage:.1f}V")
        if self.battery_voltage < 10.5:
            self.get_logger().warn(
                "  ⚠ Battery voltage low! Charge before further tests.")

        self.get_logger().info(f"\n  NEXT STEPS:")
        self.get_logger().info(
            "  1. If temps fine: increase motor current in VESC Tool, re-run")
        self.get_logger().info(
            "  2. If temps high: decrease motor current, re-run")
        self.get_logger().info(
            "  3. Once satisfied: note the limits for vehicle_params.yaml")
        self.get_logger().info(
            f"     - max_motor_current: {peak_I:.1f} A (measured peak)")
        self.get_logger().info(
            "     - Motor Current Max in VESC Tool: [your setting]")

        # Auto-save to vehicle_params.yaml
        from common import update_vehicle_params
        update_vehicle_params({
            'max_motor_current': float(peak_I),
        }, status='TESTED', logger=self.get_logger())
        self.get_logger().info("=" * 65)


def main():
    parser = argparse.ArgumentParser(
        description='F1/10th Motor Current & Thermal Characterization',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'Straight-line mode (default): needs ~10m, pauses between\n'
            'runs so you can reposition the car.\n'
            'Circle mode (--circle): needs ~3×3 m open area, automatic.\n\n'
            'Start conservative (30-40A) and increase gradually.\n'
        ))
    parser.add_argument('--max-speed', type=float, default=4.0,
                        help='Speed to command (m/s, default: 4.0)')
    parser.add_argument('--circle', action='store_true',
                        help='Drive in circles instead of straight')
    parser.add_argument('--steering', type=float, default=0.30,
                        help='Steering angle in circle mode (rad, '
                             'default: 0.30)')
    parser.add_argument('--runs', type=int, default=5,
                        help='Number of driving runs (default: 5)')
    parser.add_argument('--run-time', type=float, default=3.0,
                        help='Duration of each run (s, default: 3.0)')
    parser.add_argument('--cool-time', type=float, default=10.0,
                        help='Cooling time in circle mode (s, default: 10.0)')
    args = parser.parse_args()

    rclpy.init()
    node = CurrentLimitNode(args)

    try:
        node.run_test()
    finally:
        node.stop_car()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
