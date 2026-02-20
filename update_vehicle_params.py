#!/usr/bin/env python3
"""
Vehicle Parameter Updater for F1Tenth
======================================
This script prompts you for real measured values from your assembled car,
then updates ALL config files, launch files, headers, and URDF/xacro files
throughout the workspace to use consistent values.

Usage:
    python3 update_vehicle_params.py

It will show current defaults and ask for each measurement.
Press Enter to keep the current default value.
"""

import os
import re
import sys

# ============================================================
# Workspace root (where this script lives)
# ============================================================
WORKSPACE = os.path.dirname(os.path.abspath(__file__))


def ask_float(prompt, default, unit="m"):
    """Ask user for a float value with a default."""
    while True:
        raw = input(f"  {prompt} [current: {default} {unit}]: ").strip()
        if raw == "":
            return default
        try:
            val = float(raw)
            return val
        except ValueError:
            print("    ⚠ Please enter a number (e.g. 0.325)")


def ask_yes_no(prompt, default=True):
    """Ask a yes/no question."""
    suffix = "[Y/n]" if default else "[y/N]"
    raw = input(f"  {prompt} {suffix}: ").strip().lower()
    if raw == "":
        return default
    return raw in ("y", "yes")


def replace_in_file(filepath, pattern, replacement, count=0):
    """Replace regex pattern in a file. Returns number of replacements made."""
    full_path = os.path.join(WORKSPACE, filepath)
    if not os.path.exists(full_path):
        print(f"    ⚠ File not found: {filepath}")
        return 0

    with open(full_path, "r") as f:
        content = f.read()

    new_content, n = re.subn(pattern, replacement, content, count=count)

    if n > 0:
        with open(full_path, "w") as f:
            f.write(new_content)
        print(f"    ✓ {filepath} ({n} replacement{'s' if n > 1 else ''})")
    else:
        print(f"    - {filepath} (no match found)")

    return n


def main():
    print("=" * 60)
    print("  F1Tenth Vehicle Parameter Updater")
    print("=" * 60)
    print()
    print("Measure your assembled car and enter the values below.")
    print("Press Enter to keep the current default value.")
    print("All distances are in meters, angles in radians, mass in kg.")
    print()

    # ============================================================
    # 1. Core measurements
    # ============================================================
    print("─── Core Measurements ───")
    print("  (base_link = rear axle center on the ground)")
    print()

    wheelbase = ask_float(
        "Wheelbase (rear axle center → front axle center)", 0.3302)

    lidar_x = ask_float(
        "LiDAR X offset (rear axle → LiDAR, forward positive)", 0.275)

    lidar_y = ask_float(
        "LiDAR Y offset (rear axle → LiDAR, left positive)", 0.0)

    lidar_z = ask_float(
        "LiDAR Z offset (ground → LiDAR center height)", 0.10)

    car_width = ask_float(
        "Total car width (widest point)", 0.31)

    car_length = ask_float(
        "Total car length", 0.58)

    wheel_radius = ask_float(
        "Wheel radius", 0.0508)

    print()

    # ============================================================
    # 2. Center of gravity
    # ============================================================
    print("─── Center of Gravity ───")
    print("  lf + lr should equal your wheelbase.")
    print(f"  (wheelbase = {wheelbase})")
    print()

    default_lf = round(wheelbase * 0.15875 / 0.3302, 5)  # scale from default ratio
    default_lr = round(wheelbase - default_lf, 5)

    lf = ask_float(
        "Distance from CG to front axle (lf)", default_lf)

    lr = ask_float(
        "Distance from CG to rear axle (lr)", round(wheelbase - lf, 5))

    cg_height = ask_float(
        "Center of gravity height above ground", 0.074)

    mass = ask_float(
        "Total vehicle mass (with battery)", 3.74, unit="kg")

    inertia = ask_float(
        "Yaw moment of inertia (Iz)", 0.04712, unit="kg·m²")

    print()

    # ============================================================
    # 3. Steering
    # ============================================================
    print("─── Steering ───")
    max_steering = ask_float(
        "Maximum steering angle", 0.4189, unit="rad")

    max_steering_rate = ask_float(
        "Maximum steering rate", 3.2, unit="rad/s")

    print()

    # ============================================================
    # 4. IMU orientation (VESC upside down)
    # ============================================================
    print("─── IMU / VESC Orientation ───")
    print("  Your VESC is mounted upside-down with Y pointing forwards.")
    print()
    vesc_imu_fixed_in_firmware = ask_yes_no(
        "Did you fix the IMU orientation in the VESC Tool firmware?", default=True)

    if vesc_imu_fixed_in_firmware:
        imu_roll = 0.0
        imu_pitch = 0.0
        imu_yaw = 0.0
        print("    → Static TF base_link→imu will use identity (no rotation)")
    else:
        print("    → Will set 180° roll in the base_link→imu static transform")
        print("    ⚠ WARNING: The odometry node reads IMU quaternions directly.")
        print("      The static TF alone may NOT fix yaw estimation.")
        print("      Strongly recommend fixing orientation in VESC Tool instead.")
        imu_roll = 3.14159
        imu_pitch = 0.0
        imu_yaw = 0.0

    print()

    # ============================================================
    # 5. Also update simulation?
    # ============================================================
    print("─── Simulation ───")
    update_sim = ask_yes_no(
        "Also update simulation files to match real car?", default=True)

    print()
    print("=" * 60)
    print("  Applying changes...")
    print("=" * 60)
    print()

    total = 0

    # ============================================================
    # WHEELBASE
    # ============================================================
    print("── Wheelbase ──")

    # vesc.yaml: "    wheelbase: 0.324"
    total += replace_in_file(
        "f1tenth_system/f1tenth_stack/config/vesc.yaml",
        r"(wheelbase:\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # ftg_params.yaml: "    wheelbase: 0.324"
    total += replace_in_file(
        "f1tenth_control/config/ftg_params.yaml",
        r"(wheelbase:\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # vehicle_params.yaml: "  wheelbase: 0.3302"
    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(wheelbase:\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # fpga_params.yaml: "    wheelbase: 0.324"
    total += replace_in_file(
        "f1tenth_communication/mpc_receiver/config/fpga_params.yaml",
        r"(wheelbase:\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # pure_pursuit_launch.py: "'wheelbase': 0.3302,"
    total += replace_in_file(
        "f1tenth_control/launch/pure_pursuit_launch.py",
        r"('wheelbase':\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # stanley_launch.py: "'wheelbase': 0.3302,"
    total += replace_in_file(
        "f1tenth_control/launch/stanley_launch.py",
        r"('wheelbase':\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # ekf_launch.py: "'wheelbase': 0.3302,"
    total += replace_in_file(
        "f1tenth_control/launch/ekf_launch.py",
        r"('wheelbase':\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    # MPC mpc_types.h: FP_CONST(0.3302) after F110_DEFAULT_WHEELBASE_METERS
    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_DEFAULT_WHEELBASE_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{wheelbase}\2")

    print()

    # ============================================================
    # LIDAR TRANSFORM (base_link → laser)
    # ============================================================
    print("── LiDAR Transform ──")

    # bringup_launch.py: '0.27', '0.0', '0.11' → laser xyz
    total += replace_in_file(
        "f1tenth_system/f1tenth_stack/launch/bringup_launch.py",
        r"'[\d.]+',\s*'[\d.]+',\s*'[\d.]+',\s*#\s*x,\s*y,\s*z\s+translation",
        f"'{lidar_x}', '{lidar_y}', '{lidar_z}',   # x, y, z translation")

    # hokuyo_lidar.launch.py: '--x', '0.275', '--y', '0.0', '--z', '0.05'
    total += replace_in_file(
        "f1tenth_localization/RealRobot/JetsonFiles/hokuyo_lidar.launch.py",
        r"('--x',\s*')[\d.]+(.*?'--y',\s*')[\d.]+(.*?'--z',\s*')[\d.]+",
        rf"\g<1>{lidar_x}\g<2>{lidar_y}\g<3>{lidar_z}")

    print()

    # ============================================================
    # IMU TRANSFORM (base_link → imu)
    # ============================================================
    print("── IMU Transform ──")

    # bringup_launch.py: static_tf_base_to_imu - replace the roll, pitch, yaw line
    # The IMU xyz is always (0,0,0) since the VESC is at base_link origin
    # The rotation needs to change if VESC is upside down
    total += replace_in_file(
        "f1tenth_system/f1tenth_stack/launch/bringup_launch.py",
        r"'[\d.]+',\s*'[\d.]+',\s*'[\d.]+',\s*#\s*roll,\s*pitch,\s*yaw\s*\n\s*'base_link',\s*'imu'",
        f"'{imu_roll}', '{imu_pitch}', '{imu_yaw}',      # roll, pitch, yaw\n"
        f"            'base_link', 'imu'")

    print()

    # ============================================================
    # CAR WIDTH
    # ============================================================
    print("── Car Width ──")

    # ftg_params.yaml: "    car_width: 0.30"
    total += replace_in_file(
        "f1tenth_control/config/ftg_params.yaml",
        r"(car_width:\s*)\d+\.\d+",
        rf"\g<1>{car_width}")

    # vehicle_params.yaml: "  width: 0.31"
    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(width:\s*)\d+\.\d+",
        rf"\g<1>{car_width}")

    # mpc_types.h: F110_VEHICLE_WIDTH_METERS
    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_VEHICLE_WIDTH_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{car_width}\2")

    print()

    # ============================================================
    # CAR LENGTH
    # ============================================================
    print("── Car Length ──")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(length:\s*)\d+\.\d+",
        rf"\g<1>{car_length}")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_VEHICLE_LENGTH_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{car_length}\2")

    print()

    # ============================================================
    # CG DISTANCES
    # ============================================================
    print("── CG Distances ──")

    # vehicle_params.yaml
    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(lf:\s*)\d+\.\d+",
        rf"\g<1>{lf}")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(lr:\s*)\d+\.\d+",
        rf"\g<1>{lr}")

    # mpc_types.h
    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_DIST_CG_TO_FRONT_AXLE_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{lf}\2")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_DIST_CG_TO_REAR_AXLE_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{lr}\2")

    print()

    # ============================================================
    # MASS & INERTIA
    # ============================================================
    print("── Mass & Inertia ──")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(mass:\s*)\d+\.\d+",
        rf"\g<1>{mass}")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(inertia:\s*)\d+\.\d+",
        rf"\g<1>{inertia}")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(cg_height:\s*)\d+\.\d+",
        rf"\g<1>{cg_height}")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_VEHICLE_MASS_KG\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{mass}\2")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_YAW_INERTIA_KGM2\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{inertia}\2")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_CG_HEIGHT_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{cg_height}\2")

    print()

    # ============================================================
    # WHEEL RADIUS
    # ============================================================
    print("── Wheel Radius ──")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_WHEEL_RADIUS_METERS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{wheel_radius}\2")

    print()

    # ============================================================
    # MAX STEERING
    # ============================================================
    print("── Max Steering ──")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(max_steering_angle:\s*)\d+\.\d+",
        rf"\g<1>{max_steering}")

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(max_steering_rate:\s*)\d+\.\d+",
        rf"\g<1>{max_steering_rate}")

    total += replace_in_file(
        "f1tenth_control/launch/pure_pursuit_launch.py",
        r"('max_steering':\s*)\d+\.\d+",
        rf"\g<1>{max_steering}")

    total += replace_in_file(
        "f1tenth_control/launch/stanley_launch.py",
        r"('max_steering':\s*)\d+\.\d+",
        rf"\g<1>{max_steering}")

    total += replace_in_file(
        "f1tenth_communication/mpc_receiver/config/fpga_params.yaml",
        r"(max_steering:\s*)\d+\.\d+",
        rf"\g<1>{max_steering}")

    total += replace_in_file(
        "MPC/include/mpc_types.h",
        r"(F110_DEFAULT_MAXIMUM_STEERING_RADIANS\s*\\\s*\n\s*FP_CONST\()\d+\.\d+(\))",
        rf"\g<1>{max_steering}\2")

    print()

    # ============================================================
    # SAFETY MARGIN (half car width)
    # ============================================================
    print("── Safety Margin ──")
    safety_margin = round(car_width / 2.0, 4)

    total += replace_in_file(
        "f1tenth_planning/config/vehicle_params.yaml",
        r"(safety_margin:\s*)\d+\.\d+",
        rf"\g<1>{safety_margin}")

    print()

    # ============================================================
    # TEST SCRIPT DEFAULTS (f1tenth_parameters/common.py)
    # ============================================================
    print("── Test Script Defaults ──")

    total += replace_in_file(
        "f1tenth_parameters/common.py",
        r"(DEFAULT_WHEELBASE\s*=\s*)\d+\.\d+",
        rf"\g<1>{wheelbase}")

    print()

    # ============================================================
    # SIMULATION FILES (optional)
    # ============================================================
    if update_sim:
        print("── Simulation Files ──")

        # ego_racecar.xacro
        total += replace_in_file(
            "f1tenth_sim/launch/ego_racecar.xacro",
            r'(name="wheelbase"\s+value=")\d+\.\d+(")',
            rf"\g<1>{wheelbase}\2")

        total += replace_in_file(
            "f1tenth_sim/launch/ego_racecar.xacro",
            r'(name="width"\s+value=")\d+\.\d+(")',
            rf"\g<1>{car_width}\2")

        total += replace_in_file(
            "f1tenth_sim/launch/ego_racecar.xacro",
            r'(name="wheel_radius"\s+value=")\d+\.\d+(")',
            rf"\g<1>{wheel_radius}\2")

        total += replace_in_file(
            "f1tenth_sim/launch/ego_racecar.xacro",
            r'(name="laser_distance_from_base_link"\s+value=")\d+\.\d+(")',
            rf"\g<1>{lidar_x}\2")

        # sim.yaml: scan_distance_to_base_link
        total += replace_in_file(
            "f1tenth_sim/config/sim.yaml",
            r"(scan_distance_to_base_link:\s*)\d+\.\d+",
            rf"\g<1>{lidar_x}")

        # opp_racecar.xacro
        total += replace_in_file(
            "f1tenth_sim/launch/opp_racecar.xacro",
            r'(name="wheelbase"\s+value=")\d+\.\d+(")',
            rf"\g<1>{wheelbase}\2")

        total += replace_in_file(
            "f1tenth_sim/launch/opp_racecar.xacro",
            r'(name="laser_distance_from_base_link"\s+value=")\d+\.\d+(")',
            rf"\g<1>{lidar_x}\2")

        total += replace_in_file(
            "f1tenth_sim/launch/opp_racecar.xacro",
            r'(name="wheel_radius"\s+value=")\d+\.\d+(")',
            rf"\g<1>{wheel_radius}\2")

        print()

    # ============================================================
    # Summary
    # ============================================================
    print("=" * 60)
    print(f"  Done! {total} total replacements across all files.")
    print("=" * 60)
    print()
    print("Summary of your values:")
    print(f"  Wheelbase:        {wheelbase} m")
    print(f"  LiDAR offset:     x={lidar_x}, y={lidar_y}, z={lidar_z} m")
    print(f"  Car width:        {car_width} m")
    print(f"  Car length:       {car_length} m")
    print(f"  Wheel radius:     {wheel_radius} m")
    print(f"  CG→front (lf):    {lf} m")
    print(f"  CG→rear  (lr):    {lr} m")
    print(f"  CG height:        {cg_height} m")
    print(f"  Mass:             {mass} kg")
    print(f"  Inertia (Iz):     {inertia} kg·m²")
    print(f"  Max steering:     {max_steering} rad")
    print(f"  Max steer rate:   {max_steering_rate} rad/s")
    print(f"  Safety margin:    {safety_margin} m")
    print(f"  IMU TF roll:      {imu_roll} rad")
    print()

    if not vesc_imu_fixed_in_firmware:
        print("⚠ IMPORTANT: You indicated the VESC IMU orientation is NOT fixed in firmware.")
        print("  The static TF has been set to 180° roll, but the odometry node")
        print("  (vesc_to_odom) reads the IMU quaternion directly.")
        print("  You should fix the orientation in VESC Tool → IMU → General → Rotation.")
        print()

    print("Next steps:")
    print("  1. Rebuild the workspace: cd ~/f1tenth_ws && colcon build")
    print("  2. Verify with: ros2 run tf2_ros tf2_echo base_link laser")
    print("  3. Test odometry: ros2 topic echo /vesc/odom")
    print()


if __name__ == "__main__":
    main()
