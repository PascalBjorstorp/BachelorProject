# F1/10th Parameter Identification Suite

Automated test scripts for identifying vehicle parameters on the real F1/10th car.

**Vehicle:** Traxxas Slash 4x4, Traxxas 3351R motor, VESC VI mk5 (with IMU), LiDAR

## Prerequisites

- ROS2 Jazzy with the f1tenth_stack running (`ros2 launch f1tenth_stack bringup_launch.py`)
- Python 3 with `rclpy`, `numpy`, `scipy`
- Open space (~3m x 3m minimum for circle tests, ~5m straight for speed tests)
- Fully charged battery (11.1V LiPo)
- **Joystick connected** — the joystick has higher mux priority and acts as a deadman switch / emergency override

## Safety

All tests have built-in safety features:
- **Maximum speed limit** (configurable, default 3.0 m/s)
- **Test duration timeout** (configurable per test)
- **Battery voltage monitoring** (warns below 10.5V)
- **Ctrl+C** stops the car immediately (publishes zero command on shutdown)
- **Joystick override** — pressing the joystick deadman switch takes priority over test commands

**Always keep the joystick in hand during tests.** If anything goes wrong, press the joystick
deadman switch and steer away, or simply let go (commands timeout in 0.5s).

## Test Sequence

Run the tests in this order:

### 1. Steering Calibration (`test_steering_calibration.py`)
Verifies/calibrates the servo-to-steering-angle mapping.
```bash
python3 test_steering_calibration.py --speed 1.0 --hold-time 3.0
```
- Drives slowly forward while sweeping servo positions
- Records turning radius at each servo position
- Outputs corrected `steering_angle_to_servo_gain` and `steering_angle_to_servo_offset`

### 2. Speed Calibration (`test_speed_calibration.py`)
Verifies the ERPM-to-speed conversion gain.
```bash
python3 test_speed_calibration.py --distance 5.0
```
- Drives over a known distance at constant speed
- Times the run and compares reported vs actual velocity
- Outputs corrected `speed_to_erpm_gain`

### 3. Circle Test (`test_circle.py`)
Extracts wheelbase, max steering angle, and (at higher speeds) cornering stiffness.
```bash
python3 test_circle.py --speed 1.5 --steering 0.3 --laps 2
```
- Drives circles at fixed steering angle and speed
- Fits circle to trajectory → extracts radius
- Computes effective wheelbase and steering angle
- At multiple speeds: extracts understeer gradient → cornering stiffness ratio

### 4. Maximum Dynamics (`test_max_dynamics.py`)
Measures maximum velocity, acceleration, and deceleration.
```bash
python3 test_max_dynamics.py --max-speed 3.0
```
- Full throttle from standstill → max acceleration and velocity
- Full brake → max deceleration
- **Requires ~5m of straight, clear space**

### 5. Steering Rate (`test_steering_rate.py`)
Measures the steering actuator speed.
```bash
python3 test_steering_rate.py --speed 1.5
```
- Drives straight, then commands step steering changes
- Measures yaw rate response time from IMU
- Extracts effective steering rate limit

### 6. Friction Limit (`test_friction.py`)
Measures the maximum tire grip (friction coefficient).
```bash
python3 test_friction.py --steering 0.3 --max-speed 4.0
```
- Drives circles at increasing speed
- Records lateral acceleration from IMU
- When tires saturate → friction coefficient μ = a_lat_max / g

## Output

All tests save timestamped CSV files in `f1tenth_parameters/data/`. After running tests,
view a summary and export parameters:

```bash
python3 analyze_results.py
```

This prints identified parameters and generates a YAML snippet for `vesc.yaml` and
a C header snippet for `mpc_types.h`.

## Data Pipeline Notes

Understanding what processing happens to sensor data and commands is critical for
accurate parameter identification.

### Command Path (no smoothing by default)
```
/drive (test scripts)
  → ackermann_mux (priority selection, joystick overrides)
  → /ackermann_cmd
  → ackermann_to_vesc (angle→servo, speed→ERPM conversion)
  → /commands/motor/speed + /commands/servo/position
  → vesc_driver → VESC hardware
```

**No throttle interpolator** in the default `bringup_launch.py`. Commands go directly
to the VESC without rate limiting or smoothing.

**Quirks in ackermann_to_vesc (VEL_TO_ERPM mode):**
- **Slow-start**: When `current_vel < 1.0` m/s and `commanded > 1.0` m/s, the driver
  commands `(current_vel + 0.4)` instead of the full speed. This limits initial
  acceleration from standstill.
- **Constant braking**: With default `speed_to_braking_gain=0.0`, deceleration uses
  a constant brake force (`max/2`), not proportional to speed error.

### Sensor Path
```
VESC hardware (polled at 200Hz)
  → /sensors/core       (RPM, voltage, current — raw)
  → /sensors/imu/raw    (accel in m/s², gyro in rad/s — NO filtering)
  → /sensors/servo_position_command (last commanded servo value)

/sensors/core + /sensors/imu/raw → vesc_to_odom → /odom
```

**What's filtered/modified in `/odom`:**
| Field | Source | Filtering |
|-------|--------|-----------|
| `pose.position.x/y` | Integrated from velocity+heading | Drifts over time |
| `pose.orientation` | IMU quaternion | **None** (raw from IMU) |
| `twist.linear.x` | ERPM conversion | **0.05 m/s deadzone** (below → 0) |
| `twist.angular.z` | IMU gyroscope | **LOW-PASS EMA (α=0.3)** ⚠️ |

**For accurate yaw rate measurements** (steering rate test, friction test), the scripts
use `/sensors/imu/raw` (angular_velocity.z) directly, NOT the filtered `odom.twist.angular.z`.

### IMU Axis Convention

The VESC 6 MkV IMU is physically mounted **z-down** (board upside down on car).

**Required VESC Tool setting:** Set `Imu Rotation Roll` to **180°** in
App Settings → IMU → Rotation. This makes the firmware compensate for the
z-down mounting, so raw messages on `/sensors/imu/raw` are in the standard
vehicle frame:
- **x** = forward (longitudinal acceleration)
- **y** = left (lateral acceleration, positive during left turns)
- **z** = up (gravity reads as ~-9.8 m/s²)
- **gz** = yaw rate (positive = counter-clockwise)

All test scripts also use `abs()` for magnitude-based measurements as a safety net.

### Steering Rate Limit

The measured steering rate is the **physical servo speed** — there is no throttle
interpolator rate-limiting the servo commands in the default `bringup_launch.py`.

## File Structure

```
f1tenth_parameters/
├── README.md                      # This file
├── common.py                      # Shared utilities (safety, recording, commands)
├── test_steering_calibration.py   # Steering servo calibration
├── test_speed_calibration.py      # ERPM-to-velocity calibration
├── test_circle.py                 # Circle test (wheelbase, cornering stiffness)
├── test_max_dynamics.py           # Max velocity, acceleration, deceleration
├── test_steering_rate.py          # Steering actuator bandwidth
├── test_friction.py               # Tire friction coefficient
├── analyze_results.py             # Combined analysis and parameter export
└── data/                          # Saved test data (CSV files)
```
