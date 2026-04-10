# VESC 6 MKVI IMU Calibration and Stabilization Guide

## Goal

This guide explains how to recalibrate and tune the VESC 6 MKVI IMU so ROS2 odometry and localization become stable enough for racing use.

It is written for the current F1Tenth stack where IMU data is consumed from:
- /sensors/imu/raw
- /ego_racecar/odom

## Symptoms This Guide Addresses

- Car drives with yaw drift when standing still
- Slip mode toggles too often (chatter)
- Lateral acceleration looks unrealistic (large spikes)
- Localization quality drops during acceleration or aggressive maneuvers

## Safety First

- Put the car on a stand for motor-command checks.
- Remove nearby metal clutter when calibrating IMU.
- Keep battery voltage healthy and stable during calibration.
- Do not calibrate while touching the car.

## Prerequisites

- Latest stable VESC Tool installed
- USB connection to VESC 6 MKVI
- Existing config backup saved before changes
- ROS2 workspace built and sourced

## Step 1: Backup Existing VESC Configuration

In VESC Tool:
1. Connect to the controller.
2. Read motor config from VESC.
3. Read app config from VESC.
4. Save both to files with a timestamp.

Use names like:
- motor_config_before_imu_recal.xml
- app_config_before_imu_recal.xml

## Step 2: Mechanical and Electrical Sanity Check

Before calibrating software, verify hardware basics:
1. IMU board is firmly mounted (no loose screws).
2. Mount has light vibration isolation, not soft wobble.
3. Sensor cable and power wiring are secure.
4. Motor phase wires are routed away from IMU wiring where possible.
5. No connector intermittency when tapping chassis lightly.

## Step 3: Gyroscope Bias Calibration

In VESC Tool IMU calibration section:
1. Place car on a perfectly still surface.
2. Keep wheels and chassis motionless for at least 20 to 30 seconds.
3. Run gyro calibration.
4. Save/apply calibration values.

Important:
- If calibration is run while vibrating, gyro bias will be wrong.
- Repeat if results look inconsistent between runs.

## Step 4: Accelerometer Calibration

Run accelerometer calibration wizard (6-point if available):
1. Level orientation
2. Nose up
3. Nose down
4. Left side down
5. Right side down
6. Upside down (if required by wizard)

For each position:
- Hold still until the tool confirms sample accepted.
- Do not bump the chassis during capture.

Expected result:
- Acceleration magnitude near 1 g at rest.

## Step 5: Verify IMU Orientation and Axis Signs

After calibration, check axis alignment in live data:
1. Yaw left by hand slowly. Yaw rate sign should match expected positive direction.
2. Pitch forward slowly. Pitch sign should be consistent.
3. Roll right slowly. Roll sign should be consistent.

If any axis is inverted or swapped:
- Correct IMU orientation or axis mapping in VESC Tool.
- Re-test signs before moving on.

## Step 6: Configure IMU Filtering in VESC Tool

Menu names differ by firmware, but set low-pass filters to conservative starting points:

- Gyro LPF: 20 to 30 Hz
- Accel LPF: 10 to 20 Hz

Start conservative, then relax only if latency is too high.

Why:
- Lower filter cutoff removes vibration noise that causes false slip detection.
- Excessively high cutoff passes motor/chassis vibration into odometry.

## Step 7: Write Configuration and Reboot

1. Write motor config.
2. Write app config.
3. Power cycle VESC.
4. Reconnect and verify values persisted.

## Step 8: ROS2 Validation Procedure

Run this minimal validation workflow:

1. Launch minimal stack:
ros2 launch f1tenth_stack bringup_launch.py use_lidar:=false use_teleop:=false

2. Record bag:
ros2 bag record /sensors/imu/raw /ego_racecar/odom /sensors/servo_position_command /tf /tf_static -o imu_health_after_cal

3. Execute short test:
- stand still 20 s
- one straight burst
- stand still 20 s

4. Analyze:
/usr/bin/python3 f1tenth_parameters/scripts/analyze_imu_health_bag.py bags/imu_health_after_cal

## Step 9: Acceptance Targets

Use these practical targets for this project:

- Static gyro z std: aim <= 0.05 rad/s (good), <= 0.10 rad/s (acceptable)
- Static gyro z mean: close to 0 rad/s
- Static accel norm: near 9.81 m/s^2 (gravity present is normal)
- Moving lateral accel spikes: rare above 12 m/s^2 in simple straight tests

Notes:
- Linear acceleration includes gravity in many IMU pipelines. That is normal.
- Do not use raw accel directly for slip triggers unless gravity-compensated and filtered.

## Step 10: Recommended ROS Parameters After Calibration

In vesc.yaml (starting point):

- slip_use_lateral_accel: false
- imu_angular_velocity_alpha: 0.20 to 0.30
- slip_enter_hold_sec: 0.08
- slip_exit_hold_sec: 0.18 to 0.22
- use_dynamic_bicycle_model: true only after IMU quality is acceptable

## Troubleshooting

If gyro noise remains high after calibration:
1. Repeat gyro calibration with absolutely no vibration.
2. Reduce gyro LPF cutoff further.
3. Improve mounting isolation and wiring routing.
4. Check if chassis resonance occurs near motor startup RPM.

If acceleration still has large spikes:
1. Lower accel LPF cutoff.
2. Check for mechanical shocks from drivetrain slack.
3. Keep accel out of slip decision logic (recommended).

If car does not drive cleanly at short burst commands:
1. This can be startup/cogging behavior, separate from IMU.
2. Use a brief low-speed warm-up pulse before high-speed pulse.
3. Verify steering center trim for straight-line behavior.

## Change Log Template (Recommended)

Record each calibration pass with:
- Date/time
- VESC firmware version
- LPF settings
- Calibration method used
- Bag path
- Analyzer output summary
- Drive impression

Keeping this log makes regression tracking and tuning much faster.
