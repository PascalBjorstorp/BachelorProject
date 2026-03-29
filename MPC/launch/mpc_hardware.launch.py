"""
MPC Hardware Launch File for F1/10th

Launches the MPC Riccati-ADMM controller node for real hardware.
Designed to integrate with bringup_launch.py from f1tenth_stack.

Usage:
  # Standalone:
  ros2 launch mpc_hardware mpc_hardware.launch.py trajectory_file:=/path/to/raceline.csv

  # With bringup (include in your composite launch):
  ros2 launch f1tenth_stack bringup_launch.py
  ros2 launch mpc_hardware mpc_hardware.launch.py

Environment variable overrides for MPC tuning:
  MPC_W_LAT_ERROR, MPC_W_HEADING, MPC_W_VELOCITY, MPC_W_STEER_RATE, etc.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Build the launch graph for the MPC hardware node.

    Resolves the default trajectory path from f1tenth_planning package share,
    declares configurable launch arguments for all runtime-tunable parameters,
    and maps them to environment variables consumed by the C MPC node process.

    Returns:
        LaunchDescription containing all argument declarations, environment
        variable setters, and the mpc_hardware_node action.
    """

    # Default trajectory: look in f1tenth_planning's installed share directory
    default_trajectory = ''
    try:
        f1tenth_planning_share = get_package_share_directory('f1tenth_planning')
        candidate = os.path.join(
            f1tenth_planning_share, 'trajectories', 'my_track_map.csv'
        )
        if os.path.isfile(candidate):
            default_trajectory = candidate
    except Exception:
        pass

    # Fallback: try workspace source directory relative to this launch file
    if not default_trajectory:
        launch_dir = os.path.dirname(os.path.abspath(__file__))
        workspace_root = os.path.dirname(os.path.dirname(launch_dir))
        candidate = os.path.join(
            workspace_root, 'f1tenth_planning', 'trajectories', 'my_track_map.csv'
        )
        if os.path.isfile(candidate):
            default_trajectory = candidate

    # Declare launch arguments
    trajectory_file_arg = DeclareLaunchArgument(
        'trajectory_file',
        default_value=default_trajectory,
        description='Path to the racing trajectory CSV file'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ego_racecar/odom',
        description='Odometry topic name (from VESC driver)'
    )

    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value='/drive',
        description='Drive command topic name (to ackermann_mux)'
    )

    servo_topic_arg = DeclareLaunchArgument(
        'servo_topic',
        default_value='/sensors/servo_position_command',
        description='VESC servo position feedback topic'
    )

    imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value='/imu/filtered_angular_velocity',
        description='Filtered IMU angular velocity topic'
    )

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/ekf_pose',
        description='Map-frame pose topic (EKF-fused or raw AMCL)'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='0',
        description='Enable verbose logging (0=off, 1=on). Disable for real-time.'
    )

    watchdog_timeout_arg = DeclareLaunchArgument(
        'watchdog_timeout',
        default_value='0.5',
        description='Safety watchdog timeout in seconds'
    )

    # Set environment variables for the MPC node
    set_trajectory = SetEnvironmentVariable(
        'MPC_TRAJECTORY_FILE',
        LaunchConfiguration('trajectory_file')
    )
    set_odom = SetEnvironmentVariable(
        'MPC_ODOM_TOPIC',
        LaunchConfiguration('odom_topic')
    )
    set_drive = SetEnvironmentVariable(
        'MPC_DRIVE_TOPIC',
        LaunchConfiguration('drive_topic')
    )
    set_servo = SetEnvironmentVariable(
        'MPC_SERVO_TOPIC',
        LaunchConfiguration('servo_topic')
    )
    set_imu = SetEnvironmentVariable(
        'MPC_IMU_TOPIC',
        LaunchConfiguration('imu_topic')
    )
    set_pose = SetEnvironmentVariable(
        'MPC_AMCL_TOPIC',
        LaunchConfiguration('pose_topic')
    )
    set_verbose = SetEnvironmentVariable(
        'MPC_VERBOSE',
        LaunchConfiguration('verbose')
    )
    set_watchdog = SetEnvironmentVariable(
        'MPC_WATCHDOG_TIMEOUT',
        LaunchConfiguration('watchdog_timeout')
    )
    # MPC hardware node
    mpc_node = Node(
        package='mpc_riccati',
        executable='mpc_hardware_node',
        name='mpc_hardware_node',
        output='screen',
        emulate_tty=True,
        # Node arguments: trajectory file path passed as first arg
        arguments=[LaunchConfiguration('trajectory_file')],
    )

    return LaunchDescription([
        trajectory_file_arg,
        odom_topic_arg,
        drive_topic_arg,
        servo_topic_arg,
        imu_topic_arg,
        pose_topic_arg,
        verbose_arg,
        watchdog_timeout_arg,
        set_trajectory,
        set_odom,
        set_drive,
        set_servo,
        set_imu,
        set_pose,
        set_verbose,
        set_watchdog,
        mpc_node,
    ])
