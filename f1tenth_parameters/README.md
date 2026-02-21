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

## Odometry Limitations & IMU Corrections

The VESC derives speed from motor ERPM (back-EMF). This is accurate during
steady-state driving but becomes unreliable in two key situations:

1. **Braking / deceleration**: Wheels may lock or slip, causing ERPM to drop
   faster than the actual vehicle speed. The odom reports the car has stopped
   before it physically has.
2. **High-speed cornering**: Lateral tire slip means the wheels rotate at a
   different rate than the ground speed. The odom trajectory drifts.

The VESC IMU (`/sensors/imu/raw`) provides direct body-frame measurements
that are **unaffected by wheel slip**:
- **Accelerometer (ax)**: Integrate for velocity during braking
- **Gyroscope (gz)**: Yaw rate → `R_imu = v / |ω|` for slip-independent turning radius
- **Lateral accel (ay)**: Direct friction coefficient measurement

All test scripts use IMU-based crosschecks where relevant and flag
discrepancies between odom and IMU values.

## Test Sequence

### 1. VESC PID Tuning (`vesc_pid_test.py`)
Test and analyse the VESC speed PID controller.
```bash
# Mode 1: Direct serial test (VESC Tool must be closed)
python3 vesc_pid_test.py --target-rpm 5000

# Mode 2: Analyse a VESC Tool CSV log
python3 vesc_pid_test.py --analyze /path/to/vesc_tool_log.csv
```

### 2. Find Servo Offset (`find_servo_offset.py`)
Interactive tool to find the servo center (straight-driving) offset.
```bash
python3 find_servo_offset.py --speed 1.0
```

### 2b. Find Servo Limits (`find_servo_limits.py`)
Interactive tool to find `servo_min` and `servo_max` (physical lock-to-lock range).
Put the car on a stand with wheels off the ground, then sweep the servo outward
from center and mark where the steering hits its mechanical stops.
```bash
python3 find_servo_limits.py
```

### 3. Steering Gain (`test_steering_gain.py`) ✓ validated at 1 m/s
Calibrates `steering_angle_to_servo_gain` by driving a half-circle at max steering.
```bash
python3 test_steering_gain.py --speed 1.0 --runs 3
```

### 4. Steering Calibration (`test_steering_calibration.py`)
Sweeps servo positions to build a full servo → steering angle map.
```bash
python3 test_steering_calibration.py --speed 1.0 --hold-time 3.0
```
⚠ **Run at ≤ 1 m/s only.** At higher speeds tire slip corrupts the odom-based
radius measurement. See odometry limitations above.

### 5. Speed Calibration Sweep (`test_speed_sweep.py`)
Drives at multiple speeds, measuring actual distance with a tape measure.
Fits a quadratic ERPM model.  Uses IMU-assisted braking distance to handle
tire slide during deceleration.
```bash
python3 test_speed_sweep.py --distance 5.0 --min-speed 1 --max-speed 10
```

### 6. Wheelbase Verification (`test_wheelbase.py`)
Drives circles at low speed to extract the effective wheelbase.
```bash
python3 test_wheelbase.py --steering 0.3 --speeds 1.0,1.5,2.0 --laps 2
```
Reports both odom-based and IMU-based (R = v/ω) radius estimates.
The wheelbase estimate is most reliable at ≤ 2 m/s.

### 7. Maximum Dynamics (`test_max_dynamics.py`)
Measures maximum velocity, acceleration, and deceleration.
```bash
python3 test_max_dynamics.py --max-speed 3.0
```
⚠ Uses IMU-integrated velocity for accurate deceleration measurement.
Odom-based deceleration is unreliable due to wheel slip during braking.

### 8. Steering Rate (`test_steering_rate.py`) ⚠ not yet tested
Measures the steering actuator speed.
```bash
python3 test_steering_rate.py --speed 1.5
```
Uses IMU yaw rate (not filtered odom angular velocity) for accurate timing.

### 9. Friction Limit (`test_friction.py`)
Measures maximum tire grip (friction coefficient μ = a_y_max / g).
```bash
python3 test_friction.py --steering 0.3 --max-speed 4.0
```
The friction coefficient comes directly from the **IMU lateral acceleration**
and is not affected by odometry errors. Odom-based and IMU-based radii are
reported side-by-side for slip detection.

### 10. Cornering Stiffness (`test_cornering_stiffness.py`)
Identifies front and rear tire cornering stiffness (Cα_f, Cα_r) from steady-state
circle driving at increasing speeds. Requires vehicle mass and CG position.
```bash
python3 test_cornering_stiffness.py --steering 0.3 --max-speed 3.5 --mass 3.314 --l-f 0.1679 --l-r 0.158
```
Also computes the understeer gradient.

### 11. Longitudinal Tire Stiffness (`test_longitudinal_stiffness.py`)
Measures longitudinal tire stiffness (C_x) by comparing wheel speed (ERPM) to
body speed (IMU integration) during acceleration and braking.
```bash
python3 test_longitudinal_stiffness.py --max-speed 3.0 --mass 3.314
```
The slip ratio κ = (v_wheel − v_body) / max(|v_wheel|, |v_body|).

### 12. Motor Torque (`test_motor_torque.py`)
Maps motor current to wheel force at different speed levels. Measures max
drive and braking torque.
```bash
python3 test_motor_torque.py --speeds 1.5,2.0,3.0,4.0 --mass 3.314 --r-tire 0.05
```
⚠ Update `--r-tire` with your measured effective (loaded) tire radius!

### 13. Current Limit Characterization (`test_current_limits.py`)
Helps determine safe motor current limits by driving the car while
monitoring FET and motor temperatures. Two modes:
- **Straight-line** (default): drives straight, pauses between runs to reposition (~10m needed)
- **Circle mode** (`--circle`): drives in circles, stays in one area (~3×3 m needed)
```bash
python3 test_current_limits.py                    # straight, 4 m/s, 5 runs
python3 test_current_limits.py --circle           # circle mode
python3 test_current_limits.py --max-speed 3.0 --runs 7
```
Iterative process: set a limit in VESC Tool → run test → check temps → adjust → repeat.

## Parameter Dependencies

Test results are **automatically saved** to `vehicle_params.yaml` after each test.
If you change a fundamental measurement, re-run the affected tests:

| If you change... | Re-run these tests |
|---|---|
| `mass` | cornering stiffness, longitudinal stiffness, motor torque |
| `l_f` or `l_r` | cornering stiffness |
| `r_eff` | longitudinal stiffness, motor torque |
| `gear_ratio` | motor torque |
| `wheelbase` | cornering stiffness, steering rate |

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

## Master Test Runner

Once all VESC calibration prerequisites are done, use the master test runner to
walk through all vehicle model parameter tests interactively:

```bash
python3 run_all_tests.py
python3 run_all_tests.py --skip-confirmed   # Skip tests that already have data
```

Each test gets a y/n prompt, lets you customize arguments, and provides a
summary at the end with all identified parameters.

## File Structure

```
f1tenth_parameters/
├── README.md                      # This file
├── common.py                      # Shared utilities (safety, recording, commands, IMU helpers)
├── vehicle_params.yaml            # Vehicle parameter document (fill in with measured values)
├── run_all_tests.py               # Master test runner for vehicle model parameters
├── vesc_pid_test.py               # VESC PID step response test + VESC Tool CSV analyzer
├── find_servo_offset.py           # Interactive servo center/offset finder
├── find_servo_limits.py           # Interactive servo min/max limit finder
├── test_steering_gain.py          # Steering gain calibration (half-circle) ✓ validated at 1 m/s
├── test_steering_calibration.py   # Full steering servo calibration
├── test_speed_sweep.py            # Speed calibration sweep (tape measure + IMU braking)
├── test_wheelbase.py              # Wheelbase verification (circle test + IMU cross-check)
├── test_max_dynamics.py           # Max velocity, acceleration, deceleration (IMU-assisted)
├── test_steering_rate.py          # Steering actuator bandwidth + servo constant ⚠ not yet tested
├── test_friction.py               # Tire friction coefficient (IMU-based)
├── test_cornering_stiffness.py    # Front/rear cornering stiffness (Cα_f, Cα_r)
├── test_longitudinal_stiffness.py # Longitudinal tire stiffness (C_x)
├── test_motor_torque.py           # Motor current → wheel force mapping
├── test_current_limits.py         # Thermal characterization for VESC current limits
└── data/                          # Saved test data (CSV files)
```
