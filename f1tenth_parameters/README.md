# F1/10th Vehicle Parameter Identification

Identify physical vehicle model parameters for MPC / path planning.

## Structure

```
f1tenth_parameters/
├── common.py               Shared library (TestNode, DataRecorder, etc.)
├── vehicle_params.yaml     Identified parameters (auto-updated by tests)
├── run_all_tests.py        Master runner — walks through all model tests
├── prerequisites/          Calibration scripts (run FIRST)
├── tests/                  Core parameter identification tests
├── report/                 LaTeX report outline + plot generator
├── data/                   CSV output from tests
└── figures/                PDF plots from generate_plots.py
```

## Prerequisites

- ROS 2 with f1tenth_stack running: `ros2 launch f1tenth_stack bringup_launch.py`
- Python 3 with `numpy`, `matplotlib`, `rclpy`
- Joystick connected (emergency override)

## Step 1 — Calibration (prerequisites/)

Run in order, car on a stand for servo tests:

```bash
cd f1tenth_parameters
python3 prerequisites/vesc_pid_test.py
python3 prerequisites/find_servo_offset.py --speed 1.0
python3 prerequisites/find_servo_limits.py
python3 prerequisites/test_steering_gain.py --speed 1.0
python3 prerequisites/test_steering_calibration.py --speed 1.0
python3 prerequisites/test_speed_sweep.py --distance 5.0
```

## Step 2 — Parameter tests (tests/)

Run all interactively:

```bash
python3 run_all_tests.py
```

Or run individually:

```bash
python3 tests/test_max_dynamics.py --max-speed 3.0
python3 tests/test_steering_rate.py
python3 tests/test_friction.py --max-speed 4.0
python3 tests/test_cornering_stiffness.py
python3 tests/test_longitudinal_stiffness.py
python3 tests/test_motor_torque.py
python3 tests/test_current_limits.py
```

Each test saves CSVs to `data/` and auto-updates `vehicle_params.yaml`.
Use `--runs N` for repeated measurements.

## Step 3 — Plots & Report

```bash
python3 report/generate_plots.py
cd report && pdflatex report_outline.tex && pdflatex report_outline.tex
```
