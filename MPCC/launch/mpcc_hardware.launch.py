"""
MPCC hardware launch file for F1/10th real-car deployment.

Launches the MPCC hardware node with explicit hardware topics and
MPCC-specific environment variables.

Usage:
  ros2 launch mpcc_f1_10th mpcc_hardware.launch.py

Override trajectory and topics:
  ros2 launch mpcc_f1_10th mpcc_hardware.launch.py \
      trajectory_file:=/path/to/my_track_centerline.csv \
      odom_topic:=/ego_racecar/odom \
      pose_topic:=/ekf_pose \
      imu_topic:=/imu/filtered_angular_velocity

This launch file uses the current Jetson MPCC baseline by default,
while retaining the extra hardware guardrails needed on the real car.
Override any launch argument if you need to run a different tune.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


HARDWARE_TUNING_DEFAULTS = [
    ("horizon", "HORIZON", "60", "Prediction horizon steps"),
    ("dt", "DT", "0.025", "Prediction time step in seconds"),
    ("q_contouring", "Q_CONTOURING", "16.0", "Contouring weight"),
    ("q_lag", "Q_LAG", "13.4", "Lag weight"),
    ("q_heading", "Q_HEADING", "5.8", "Heading alignment weight"),
    ("q_wall_clearance", "Q_WALL_CLEARANCE", "450.0", "Wall-clearance weight"),
    ("wall_clearance_margin", "WALL_CLEARANCE_MARGIN", "0.12", "Soft wall-clearance margin in meters"),
    ("track_buffer", "MPCC_TRACK_BUFFER", "0.00", "Hard buffer subtracted from each track bound in meters"),
    ("q_progress", "Q_PROGRESS", "0.9", "Progress reward"),
    ("q_physical_progress", "Q_PHYSICAL_PROGRESS", "1.78", "Physical path-progress reward"),
    ("s_qp_window", "MPCC_S_QP_WINDOW", "1.0", "Per-stage s bound window in meters"),
    ("q_vx", "Q_VX", "0.0", "Longitudinal velocity tracking weight"),
    ("vx_ref", "VX_REF", "0.0", "Reference longitudinal velocity"),
    (
        "use_raceline_vx_ref",
        "MPCC_USE_RACELINE_VX_REF",
        "0",
        "Use CSV velocity as per-stage vx target (0/1)",
    ),
    (
        "use_raceline_vx_limit",
        "MPCC_USE_RACELINE_VX_LIMIT",
        "0",
        "Use CSV velocity in speed limiter (0/1)",
    ),
    (
        "raceline_vx_limit_scale",
        "MPCC_RACELINE_VX_LIMIT_SCALE",
        "0.0",
        "Multiplier for CSV velocity speed limit when enabled",
    ),
    ("q_vy", "Q_VY", "0.0", "Lateral velocity regularization weight"),
    ("q_omega", "Q_OMEGA", "1.0", "Yaw-rate regularization weight"),
    ("r_delta", "R_DELTA", "0.95", "Steering effort weight"),
    ("r_ax", "R_AX", "0.42", "Acceleration effort weight"),
    ("ax_max", "AX_MAX", "10.25", "Maximum solver acceleration in m/s^2"),
    ("ax_min", "AX_MIN", "-10.25", "Minimum solver acceleration in m/s^2"),
    ("ax_min_hardware", "MPCC_AX_MIN_HARDWARE", "-10.25", "Hardware braking acceleration clamp in m/s^2"),
    ("r_vtheta", "R_VTHETA", "0.09", "Virtual progress effort weight"),
    ("w_delta_rate", "W_DELTA_RATE", "54.0", "Steering rate weight"),
    ("w_ax_rate", "W_AX_RATE", "3.6", "Acceleration rate weight"),
    ("w_vtheta_rate", "W_VTHETA_RATE", "0.42", "Virtual progress rate weight"),
    (
        "w_vtheta_physical",
        "W_VTHETA_PHYSICAL",
        "130.0",
        "Penalty on v_theta minus physical path velocity",
    ),
    ("q_contouring_term", "Q_CONTOURING_TERM", "100.0", "Terminal contouring weight"),
    ("q_lag_term", "Q_LAG_TERM", "50.0", "Terminal lag weight"),
    ("q_heading_term", "Q_HEADING_TERM", "18.0", "Terminal heading alignment weight"),
    ("q_progress_term", "Q_PROGRESS_TERM", "32.0", "Terminal progress reward"),
    ("admm_rho", "ADMM_RHO", "60.0", "ADMM penalty parameter"),
    ("admm_rho_u", "ADMM_RHO_U", "4.0", "Optional control ADMM penalty (0 uses rho)"),
    ("admm_max_iter", "ADMM_MAX_ITER", "125", "ADMM maximum iterations"),
    ("admm_tol", "ADMM_TOL", "0.02", "ADMM convergence tolerance"),
    ("admm_adaptive_rho", "ADMM_ADAPTIVE_RHO", "1", "Enable ADMM adaptive rho updates (0/1)"),
    ("admm_alpha_relax", "ADMM_ALPHA_RELAX", "1.0", "ADMM over-relaxation factor"),
    ("accept_max_iter", "MPCC_ACCEPT_MAX_ITER", "1", "Accept near-converged max-iteration solves (0/1)"),
    ("max_iter_primal_tol", "MPCC_MAX_ITER_PRIMAL_TOL", "0.04", "Primal residual tolerance for accepted max-iteration solves"),
    ("max_iter_dual_tol", "MPCC_MAX_ITER_DUAL_TOL", "0.04", "Dual residual tolerance for accepted max-iteration solves"),
    ("max_iter_track_tol", "MPCC_MAX_ITER_TRACK_TOL", "0.05", "Track-violation tolerance for accepted max-iteration solves"),
    ("warm_start_max_s_error", "MPCC_WARM_START_MAX_S_ERROR", "1.5", "Maximum warm-start geometry s error before reset"),
    ("v_theta_max", "V_THETA_MAX", "20.0", "Maximum virtual progress speed"),
    ("delta_rate_max", "DELTA_RATE_MAX", "2.849", "Maximum modeled steering rate in rad/s"),
    (
        "cross_call_scale",
        "MPCC_CROSS_CALL_SCALE",
        "1.0",
        "Rate-scaling factor for a 25 ms control period with DT=0.025",
    ),
    (
        "adapt_cross_call_scale",
        "MPCC_ADAPT_CROSS_CALL_SCALE",
        "0",
        "Enable runtime adaptation of cross-call scaling (0/1)",
    ),
    (
        "vx_min_cmd",
        "MPCC_VX_MIN_CMD",
        "0.1",
        "Minimum positive speed command in m/s",
    ),
]


def _env_or_default(env_name: str, default_value: str) -> str:
    return os.environ.get(env_name, default_value)


def _resolve_default_trajectory() -> str:
    candidates = []

    try:
        from ament_index_python.packages import get_package_share_directory

        planning_share = get_package_share_directory("f1tenth_planning")
        candidates.extend(
            [
                os.path.join(planning_share, "trajectories", "my_track_centerline.csv"),
            ]
        )
    except Exception:
        pass

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    search_roots = []
    current = launch_dir
    for _ in range(6):
        current = os.path.dirname(current)
        search_roots.append(current)

    for root in search_roots:
        candidates.extend(
            [
                os.path.join(root, "f1tenth_planning", "trajectories", "my_track_centerline.csv"),
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
        default_value="25",
        description="Nominal control period in milliseconds for rate scaling",
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

    publish_speed_arg = DeclareLaunchArgument(
        "publish_speed_command",
        default_value="0",
        description="Publish Ackermann speed along with acceleration (0/1)",
    )

    use_local_raceline_arg = DeclareLaunchArgument(
        "use_local_raceline",
        default_value="0",
        description="Allow /local_raceline to override the startup CSV trajectory (0/1)",
    )

    tuning_args = [
        DeclareLaunchArgument(
            arg_name,
            default_value=_env_or_default(env_name, default_value),
            description=description,
        )
        for arg_name, env_name, default_value, description in HARDWARE_TUNING_DEFAULTS
    ]

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
    set_publish_speed = SetEnvironmentVariable(
        "MPCC_PUBLISH_SPEED_COMMAND", LaunchConfiguration("publish_speed_command")
    )
    set_use_local_raceline = SetEnvironmentVariable(
        "MPCC_USE_LOCAL_RACELINE", LaunchConfiguration("use_local_raceline")
    )

    tuning_env = [
        SetEnvironmentVariable(env_name, LaunchConfiguration(arg_name))
        for arg_name, env_name, _default_value, _description in HARDWARE_TUNING_DEFAULTS
    ]

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
            publish_speed_arg,
            use_local_raceline_arg,
            *tuning_args,
            set_trajectory,
            set_odom,
            set_pose,
            set_imu,
            set_drive,
            set_control_period,
            set_watchdog,
            set_verbose,
            set_publish_speed,
            set_use_local_raceline,
            *tuning_env,
            mpcc_hardware_node,
        ]
    )
