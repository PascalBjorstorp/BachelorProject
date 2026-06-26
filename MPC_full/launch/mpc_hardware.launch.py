"""
MPC Hardware Launch File for F1/10th

Launches the MPC Riccati-ADMM controller node for real hardware.
Designed to integrate with bringup_launch.py from f1tenth_stack.

Usage:
  # Standalone:
        ros2 launch mpc_full mpc_hardware.launch.py

  # With bringup (include in your composite launch):
  ros2 launch f1tenth_stack bringup_launch.py
    ros2 launch mpc_full mpc_hardware.launch.py

Environment variable overrides for MPC tuning:
  MPC_W_LAT_ERROR, MPC_W_HEADING, MPC_W_VELOCITY, MPC_W_STEER_RATE, etc.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Build the launch graph for the MPC hardware node.

    Declares configurable launch arguments for all runtime-tunable parameters,
    and maps them to environment variables consumed by the C MPC node process.

    Returns:
        LaunchDescription containing all argument declarations, environment
        variable setters, and the mpc_hardware_node action.
    """

    # Declare launch arguments
    use_local_raceline_arg = DeclareLaunchArgument(
        'use_local_raceline',
        default_value='true',
        description='Use /local_raceline topic as MPC reference trajectory'
    )

    local_raceline_topic_arg = DeclareLaunchArgument(
        'local_raceline_topic',
        default_value='/local_raceline',
        description='Local raceline topic published by lateral planner'
    )

    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/ego_racecar/odom',
        description='Odometry topic (velocity feedback)'
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

    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/ekf_pose',
        description='EKF pose topic'
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

    low_speed_brake_inhibit_vx_arg = DeclareLaunchArgument(
        'low_speed_brake_inhibit_vx',
        default_value='0.7',
        description='Disable braking below this speed (m/s). Set 0 to disable.'
    )

    low_speed_min_accel_arg = DeclareLaunchArgument(
        'low_speed_min_accel',
        default_value='0.0',
        description='Minimum allowed accel when low-speed brake inhibit is active (m/s^2).'
    )

    recovery_epsi_arg = DeclareLaunchArgument(
        'recovery_epsi_rad',
        default_value='0.45',
        description='Recovery trigger heading error threshold |e_psi| (rad). Set <=0 to disable.'
    )

    recovery_ey_arg = DeclareLaunchArgument(
        'recovery_ey_m',
        default_value='0.20',
        description='Recovery trigger lateral error threshold |e_y| (m). Set <=0 to disable.'
    )

    recovery_vref_cap_arg = DeclareLaunchArgument(
        'recovery_vref_max',
        default_value='1.50',
        description='Max v_ref during recovery (m/s). Set <=0 to disable.'
    )

    drive_republish_period_arg = DeclareLaunchArgument(
        'drive_republish_period_ms',
        default_value='5',
        description='Republish latest MPC /drive command at this fixed period in ms.'
    )

    log_local_raceline_snapshots_arg = DeclareLaunchArgument(
        'log_local_raceline_snapshots',
        default_value='0',
        description='Write full /local_raceline CSV snapshots from MPC node (0/1). Off for real-time driving.'
    )

    solver_csv_log_arg = DeclareLaunchArgument(
        'solver_csv_log',
        default_value='0',
        description='Write per-cycle solver CSV from MPC node (0/1). Keep off for real-time driving.'
    )

    # Set environment variables for the MPC node
    set_use_local_raceline = SetEnvironmentVariable(
        'MPC_USE_LOCAL_RACELINE',
        LaunchConfiguration('use_local_raceline')
    )
    set_local_raceline_topic = SetEnvironmentVariable(
        'MPC_LOCAL_RACELINE_TOPIC',
        LaunchConfiguration('local_raceline_topic')
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
    set_pose = SetEnvironmentVariable(
        'MPC_EKF_TOPIC',
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
    set_low_speed_brake_inhibit_vx = SetEnvironmentVariable(
        'MPC_LOW_SPEED_BRAKE_INHIBIT_VX',
        LaunchConfiguration('low_speed_brake_inhibit_vx')
    )
    set_low_speed_min_accel = SetEnvironmentVariable(
        'MPC_LOW_SPEED_MIN_ACCEL',
        LaunchConfiguration('low_speed_min_accel')
    )
    set_recovery_epsi = SetEnvironmentVariable(
        'MPC_RECOVERY_EPSI_RAD',
        LaunchConfiguration('recovery_epsi_rad')
    )
    set_recovery_ey = SetEnvironmentVariable(
        'MPC_RECOVERY_EY_M',
        LaunchConfiguration('recovery_ey_m')
    )
    set_recovery_vref = SetEnvironmentVariable(
        'MPC_RECOVERY_VREF_MAX',
        LaunchConfiguration('recovery_vref_max')
    )
    set_drive_republish_period = SetEnvironmentVariable(
        'MPC_DRIVE_REPUBLISH_PERIOD_MS',
        LaunchConfiguration('drive_republish_period_ms')
    )
    set_log_local_raceline_snapshots = SetEnvironmentVariable(
        'MPC_LOG_LOCAL_RACELINE_SNAPSHOTS',
        LaunchConfiguration('log_local_raceline_snapshots')
    )
    set_solver_csv_log = SetEnvironmentVariable(
        'MPC_SOLVER_CSV_LOG',
        LaunchConfiguration('solver_csv_log')
    )
    # MPC hardware node
    mpc_node = Node(
        package='mpc_full',
        executable='mpc_hardware_node',
        name='mpc_full_hardware_node',
        output='screen',
        emulate_tty=True,
        respawn=True,
        respawn_delay=0.5,
    )

    return LaunchDescription([
        use_local_raceline_arg,
        local_raceline_topic_arg,
        odom_topic_arg,
        drive_topic_arg,
        servo_topic_arg,
        pose_topic_arg,
        verbose_arg,
        watchdog_timeout_arg,
        low_speed_brake_inhibit_vx_arg,
        low_speed_min_accel_arg,
        recovery_epsi_arg,
        recovery_ey_arg,
        recovery_vref_cap_arg,
        drive_republish_period_arg,
        log_local_raceline_snapshots_arg,
        solver_csv_log_arg,
        set_use_local_raceline,
        set_local_raceline_topic,
        set_odom,
        set_drive,
        set_servo,
        set_pose,
        set_verbose,
        set_watchdog,
        set_low_speed_brake_inhibit_vx,
        set_low_speed_min_accel,
        set_recovery_epsi,
        set_recovery_ey,
        set_recovery_vref,
        set_drive_republish_period,
        set_log_local_raceline_snapshots,
        set_solver_csv_log,
        mpc_node,
    ])
