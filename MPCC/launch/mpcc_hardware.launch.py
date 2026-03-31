"""
MPCC hardware launch file for F1/10th real-car deployment.

Launches the MPCC hardware node with explicit hardware topics and
MPCC-specific environment variables.

Usage:
  ros2 launch mpcc_f1_10th mpcc_hardware.launch.py

Override trajectory and topics:
  ros2 launch mpcc_f1_10th mpcc_hardware.launch.py \
      trajectory_file:=/path/to/hardware_raceline.csv \
      odom_topic:=/ego_racecar/odom \
      pose_topic:=/ekf_pose \
      imu_topic:=/imu/filtered_angular_velocity
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _resolve_default_trajectory() -> str:
    candidates = []

    try:
        from ament_index_python.packages import get_package_share_directory

        planning_share = get_package_share_directory("f1tenth_planning")
        candidates.extend(
            [
                os.path.join(planning_share, "trajectories", "hardware_raceline.csv"),
                os.path.join(planning_share, "trajectories", "my_track_raceline.csv"),
                os.path.join(planning_share, "trajectories", "Spielberg_raceline.csv"),
            ]
        )
    except Exception:
        pass

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    workspace_root = os.path.dirname(os.path.dirname(launch_dir))

    candidates.extend(
        [
            os.path.join(workspace_root, "f1tenth_planning", "trajectories", "hardware_raceline.csv"),
            os.path.join(workspace_root, "f1tenth_planning", "trajectories", "my_track_raceline.csv"),
            "/ros2_ws/src/f1tenth_planning/trajectories/hardware_raceline.csv",
            "/ros2_ws/src/f1tenth_planning/trajectories/my_track_raceline.csv",
        ]
    )

    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            return candidate

    return ""


def generate_launch_description() -> LaunchDescription:
    trajectory_arg = DeclareLaunchArgument(
        "trajectory_file",
        default_value=_resolve_default_trajectory(),
        description="Path to MPCC trajectory CSV file",
    )

    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/ego_racecar/odom",
        description="Odometry topic from VESC or fused odometry",
    )

    pose_topic_arg = DeclareLaunchArgument(
        "pose_topic",
        default_value="/ekf_pose",
        description="Map-frame pose topic (EKF/AMCL)",
    )

    imu_topic_arg = DeclareLaunchArgument(
        "imu_topic",
        default_value="/imu/filtered_angular_velocity",
        description="Filtered IMU yaw-rate topic",
    )

    drive_topic_arg = DeclareLaunchArgument(
        "drive_topic",
        default_value="/drive",
        description="Ackermann drive output topic",
    )

    control_period_arg = DeclareLaunchArgument(
        "control_period_ms",
        default_value="0",
        description="Control timer period in milliseconds (0 uses MPCC DT)",
    )

    watchdog_arg = DeclareLaunchArgument(
        "watchdog_timeout",
        default_value="0.5",
        description="Odometry watchdog timeout in seconds",
    )

    verbose_arg = DeclareLaunchArgument(
        "verbose",
        default_value="0",
        description="Verbose solver logging (0/1)",
    )

    set_trajectory = SetEnvironmentVariable(
        "MPCC_TRAJECTORY_FILE", LaunchConfiguration("trajectory_file")
    )
    set_odom = SetEnvironmentVariable(
        "MPCC_ODOM_TOPIC", LaunchConfiguration("odom_topic")
    )
    set_pose = SetEnvironmentVariable(
        "MPCC_POSE_TOPIC", LaunchConfiguration("pose_topic")
    )
    set_imu = SetEnvironmentVariable(
        "MPCC_IMU_TOPIC", LaunchConfiguration("imu_topic")
    )
    set_drive = SetEnvironmentVariable(
        "MPCC_DRIVE_TOPIC", LaunchConfiguration("drive_topic")
    )
    set_control_period = SetEnvironmentVariable(
        "MPCC_CONTROL_PERIOD_MS", LaunchConfiguration("control_period_ms")
    )
    set_watchdog = SetEnvironmentVariable(
        "MPCC_WATCHDOG_TIMEOUT", LaunchConfiguration("watchdog_timeout")
    )
    set_verbose = SetEnvironmentVariable(
        "MPCC_VERBOSE", LaunchConfiguration("verbose")
    )

    mpcc_hardware_node = Node(
        package="mpcc_f1_10th",
        executable="mpcc_hardware_node",
        name="mpcc_hardware_node",
        output="screen",
        emulate_tty=True,
        arguments=[LaunchConfiguration("trajectory_file")],
    )

    return LaunchDescription(
        [
            trajectory_arg,
            odom_topic_arg,
            pose_topic_arg,
            imu_topic_arg,
            drive_topic_arg,
            control_period_arg,
            watchdog_arg,
            verbose_arg,
            set_trajectory,
            set_odom,
            set_pose,
            set_imu,
            set_drive,
            set_control_period,
            set_watchdog,
            set_verbose,
            mpcc_hardware_node,
        ]
    )
