# Copyright 2020 Hongrui Zheng
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import pathlib
import time
from typing import Any, Dict, List, Tuple

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
import f1tenth_gym  # noqa: F401 - Register environment
from f1tenth_gym.envs.f110_env import F110Env
from f1tenth_gym.envs.track import Track
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
import gymnasium as gym
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from transforms3d import euler
from vesc_msgs.msg import VescStateStamped


def _source_maps_dir() -> pathlib.Path:
    return pathlib.Path(__file__).resolve().parents[1] / 'maps'


def _resolve_map_yaml_path(map_path: str) -> pathlib.Path:
    """Resolve map_path as absolute YAML/path stem, installed map, or source map."""
    expanded = pathlib.Path(map_path).expanduser()

    candidates = []
    if expanded.suffix in ('.yaml', '.yml'):
        candidates.append(expanded)
    elif expanded.is_absolute() or expanded.parent != pathlib.Path('.'):
        candidates.append(expanded.with_suffix('.yaml'))
    else:
        share_maps = pathlib.Path(get_package_share_directory('f1tenth_gym_ros')) / 'maps'
        candidates.append(share_maps / f'{map_path}.yaml')
        candidates.append(_source_maps_dir() / f'{map_path}.yaml')

    for candidate in candidates:
        if candidate.exists():
            return candidate.resolve()

    return candidates[0]


# Constants for timer periods (in seconds)
PUBLISH_TIMER_PERIOD: float = 0.005  # 200 Hz for sensor data publishing (matches sim_timestep)
SYNC_MODE_TIMER_PERIOD: float = 1.0  # 1 Hz heartbeat in sync mode

# Identity quaternion (w, x, y, z) for no rotation
IDENTITY_QUAT: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class GymBridge(Node):
    """
    ROS2 bridge node for F1Tenth gym environment.

    This node bridges the F1Tenth gym simulation environment with ROS2,
    publishing sensor data (laser scans, odometry) and subscribing to
    drive commands. Includes performance optimizations such as pre-allocated
    messages, batched transform publishing, and static transforms for fixed
    frames.
    """

    def __init__(self) -> None:
        super().__init__('gym_bridge')

        # Declare all parameters
        self._declare_parameters()

        # Validate and get parameters
        num_agents = self._validate_num_agents()
        self.vehicle_params = self._get_vehicle_params()
        scale = self.get_parameter('scale').value

        # Get noise parameters for realistic simulation
        self._odom_noise_enabled = self.get_parameter('odom_noise_enabled').value
        self._odom_pos_noise_std = self.get_parameter('odom_pos_noise_std').value
        self._odom_vel_noise_std = self.get_parameter('odom_vel_noise_std').value
        self._odom_yaw_noise_std = self.get_parameter('odom_yaw_noise_std').value

        # Get TF frame configuration
        self._tf_frame_id = self.get_parameter('tf_frame_id').value
        self._odom_frame_id = self.get_parameter('odom_frame_id').value
        self._publish_odom_topic = bool(self.get_parameter('publish_odom').value)
        self._publish_base_tf = bool(self.get_parameter('publish_base_tf').value)
        self._publish_sim_vesc_sensors = bool(
            self.get_parameter('publish_sim_vesc_sensors').value)
        self._drive_input_mode = str(
            self.get_parameter('drive_input_mode').value).strip().lower()
        if self._drive_input_mode not in ('ackermann', 'vesc'):
            raise ValueError("drive_input_mode must be 'ackermann' or 'vesc'")
        self._sim_vesc_current_topic = self.get_parameter(
            'sim_vesc_current_topic').value
        self._sim_vesc_brake_topic = self.get_parameter(
            'sim_vesc_brake_topic').value
        self._sim_vesc_motor_speed_topic = self.get_parameter(
            'sim_vesc_motor_speed_topic').value
        self._sim_vesc_servo_command_topic = self.get_parameter(
            'sim_vesc_servo_command_topic').value
        self._sim_vesc_accel_to_current_gain = self.get_parameter(
            'sim_vesc_accel_to_current_gain').value
        self._sim_vesc_accel_to_brake_gain = self.get_parameter(
            'sim_vesc_accel_to_brake_gain').value
        self._sim_vesc_current_accel_scale = self.get_parameter(
            'sim_vesc_current_accel_scale').value
        self._sim_vesc_brake_accel_scale = self.get_parameter(
            'sim_vesc_brake_accel_scale').value
        self._sim_vesc_erpm_speed_tau = self.get_parameter(
            'sim_vesc_erpm_speed_tau').value
        self._sim_vesc_motor_accel_limit = self.get_parameter(
            'sim_vesc_motor_accel_limit').value
        self._sim_vesc_command_timeout_sec = self.get_parameter(
            'sim_vesc_command_timeout_sec').value
        self._sim_vesc_brake_stop_speed = max(
            0.0, float(self.get_parameter('sim_vesc_brake_stop_speed').value))
        if self._sim_vesc_accel_to_current_gain == 0.0:
            raise ValueError('sim_vesc_accel_to_current_gain must be non-zero')
        if self._sim_vesc_accel_to_brake_gain == 0.0:
            raise ValueError('sim_vesc_accel_to_brake_gain must be non-zero')
        if self._sim_vesc_erpm_speed_tau <= 0.0:
            self._sim_vesc_erpm_speed_tau = 0.15
        if self._sim_vesc_motor_accel_limit <= 0.0:
            self._sim_vesc_motor_accel_limit = self.vehicle_params.get(
                'a_max', 7.31)
        self._sim_vesc_last_command_wall_sec = None
        self._sim_vesc_last_motor_current = 0.0
        self._sim_vesc_last_brake_current = 0.0
        self._sim_vesc_last_input_current = 0.0
        self._sim_vesc_state_topic = self.get_parameter('sim_vesc_state_topic').value
        self._sim_vesc_imu_topic = self.get_parameter('sim_vesc_imu_topic').value
        self._sim_vesc_servo_topic = self.get_parameter('sim_vesc_servo_topic').value
        self._sim_vesc_speed_to_erpm_gain = self.get_parameter(
            'sim_vesc_speed_to_erpm_gain').value
        self._sim_vesc_speed_to_erpm_offset = self.get_parameter(
            'sim_vesc_speed_to_erpm_offset').value
        if self._sim_vesc_speed_to_erpm_gain == 0.0:
            raise ValueError('sim_vesc_speed_to_erpm_gain must be non-zero')
        self._sim_vesc_speed_scale = self.get_parameter('sim_vesc_speed_scale').value
        self._sim_vesc_speed_bias = self.get_parameter('sim_vesc_speed_bias').value
        self._sim_vesc_speed_noise_std = self.get_parameter(
            'sim_vesc_speed_noise_std').value
        self._sim_vesc_yaw_rate_bias = self.get_parameter(
            'sim_vesc_yaw_rate_bias').value
        self._sim_vesc_yaw_rate_noise_std = self.get_parameter(
            'sim_vesc_yaw_rate_noise_std').value
        self._sim_vesc_accel_noise_std = self.get_parameter(
            'sim_vesc_accel_noise_std').value
        self._sim_vesc_servo_gain = self.get_parameter(
            'sim_vesc_servo_gain').value
        self._sim_vesc_servo_offset = self.get_parameter(
            'sim_vesc_servo_offset').value
        if self._sim_vesc_servo_gain == 0.0:
            raise ValueError('sim_vesc_servo_gain must be non-zero')
        self._sim_vesc_steering_correction_c2 = self.get_parameter(
            'sim_vesc_steering_correction_c2').value
        self._sim_vesc_steering_correction_c1 = self.get_parameter(
            'sim_vesc_steering_correction_c1').value
        self._sim_vesc_steering_correction_c0 = self.get_parameter(
            'sim_vesc_steering_correction_c0').value
        self._sim_vesc_last_time_sec = None
        self._sim_vesc_last_vx = 0.0
        self._sim_vesc_last_vy = 0.0

        # Get scan rate limiting configuration
        self._scan_publish_rate = self.get_parameter('scan_publish_rate').value
        self._last_scan_time = 0.0
        if self._scan_publish_rate > 0:
            self._scan_period = 1.0 / self._scan_publish_rate
            self.get_logger().info(
                f'Scan publish rate limited to {self._scan_publish_rate:.1f} Hz')
        else:
            self._scan_period = 0.0

        if self._odom_noise_enabled:
            self.get_logger().info(
                f'Odometry noise ENABLED: pos_std={self._odom_pos_noise_std:.3f}m, '
                f'vel_std={self._odom_vel_noise_std:.3f}m/s, '
                f'yaw_std={self._odom_yaw_noise_std:.4f}rad')

        if self._tf_frame_id != 'map' or self._odom_frame_id != 'map':
            self.get_logger().info(
                f'TF frames configured: tf_frame={self._tf_frame_id}, '
                f'odom_frame={self._odom_frame_id}')
        if self._publish_sim_vesc_sensors:
            self.get_logger().info(
                'Simulated VESC sensors enabled: publishing wheel speed, '
                'IMU yaw-rate, and servo command.')
        if self._drive_input_mode == 'vesc':
            self.get_logger().info(
                'Simulated VESC command input enabled: consuming current, '
                'brake, ERPM, and servo commands.')

        # Load map and create environment
        self.env = self._create_environment(num_agents, scale)

        # Initialize agent state
        self._init_ego_state()
        if num_agents == 2:
            self._init_opponent_state()

        self._scan_downsample_idx = None
        self._scan_downsample_source = None
        self._scan_downsample_warned = False

        # Reset environment with initial poses
        self._reset_environment(num_agents)

        # Pre-allocate messages for hot path
        self._preallocate_messages(num_agents)

        # Setup timers based on mode
        self._setup_timers(num_agents)

        # Setup publishers and subscribers
        self._setup_publishers(num_agents)
        self._setup_subscribers(num_agents)

        # Performance metrics
        self._enable_perf_metrics = False
        self._loop_times: List[float] = []

        # Real-time throttle: 0 = run as fast as possible, 1.0 = real-time
        self._real_time_factor = self.get_parameter('real_time_factor').value
        self._rt_wall_start = time.perf_counter()
        self._rt_sim_start = 0.0
        self._rt_initialized = False
        if self._real_time_factor > 0:
            self.get_logger().info(
                f'Real-time factor: {self._real_time_factor}x '
                f'(sim will run at {self._real_time_factor}x wall-clock speed)')
        else:
            self.get_logger().info('Real-time factor: unlimited (run as fast as possible)')

        # QoS profile for reliable communication
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

    def _declare_parameters(self) -> None:
        """Declare all ROS2 parameters."""
        self.declare_parameter('ego_namespace', 'ego_racecar')
        self.declare_parameter('ego_odom_topic', 'odom')
        self.declare_parameter('ego_opp_odom_topic', 'opp_odom')
        self.declare_parameter('ego_scan_topic', 'scan')
        self.declare_parameter('ego_drive_topic', 'drive')
        self.declare_parameter('opp_namespace', 'opp_racecar')
        self.declare_parameter('opp_odom_topic', 'odom')
        self.declare_parameter('opp_ego_odom_topic', 'opp_odom')
        self.declare_parameter('opp_scan_topic', 'opp_scan')
        self.declare_parameter('opp_drive_topic', 'opp_drive')
        self.declare_parameter('scan_distance_to_base_link', 0.265)
        self.declare_parameter('scan_fov', 4.7)
        self.declare_parameter('scan_beams', 1080)
        self.declare_parameter('scan_range_max', 30.0)
        self.declare_parameter('map_path', 'levine')
        self.declare_parameter('map_img_ext', '.png')
        self.declare_parameter('num_agent', 1)
        self.declare_parameter('sx', 0.0)
        self.declare_parameter('sy', 0.0)
        self.declare_parameter('stheta', 0.0)
        self.declare_parameter('sx1', 2.0)
        self.declare_parameter('sy1', 0.5)
        self.declare_parameter('stheta1', 0.0)
        self.declare_parameter('kb_teleop', True)
        self.declare_parameter('teleop_steering_angle', 0.3)
        self.declare_parameter('scale', 1.0)
        self.declare_parameter('sim_timestep', 0.01)
        self.declare_parameter('vehicle_params', 'f1tenth')
        self.declare_parameter('async_mode', True)
        self.declare_parameter('use_sim_time_bridge', False)
        self.declare_parameter('scan_noise_std', 0.0)
        self.declare_parameter('headless', False)  # Disable rendering for headless systems
        # 0 = unlimited, 1.0 = real-time, 0.5 = half speed.
        self.declare_parameter('real_time_factor', 0.0)
        self.declare_parameter('drive_uses_acceleration_field', False)
        self.declare_parameter('drive_input_mode', 'ackermann')
        self.declare_parameter('sim_vesc_current_topic', 'commands/motor/current')
        self.declare_parameter('sim_vesc_brake_topic', 'commands/motor/brake')
        self.declare_parameter('sim_vesc_motor_speed_topic', 'commands/motor/speed')
        self.declare_parameter('sim_vesc_servo_command_topic', 'commands/servo/position')
        self.declare_parameter('sim_vesc_accel_to_current_gain', 5.82)
        self.declare_parameter('sim_vesc_accel_to_brake_gain', 7.38)
        self.declare_parameter('sim_vesc_current_accel_scale', 1.0)
        self.declare_parameter('sim_vesc_brake_accel_scale', 1.0)
        self.declare_parameter('sim_vesc_erpm_speed_tau', 0.15)
        self.declare_parameter('sim_vesc_motor_accel_limit', 7.31)
        self.declare_parameter('sim_vesc_command_timeout_sec', 0.20)
        self.declare_parameter('sim_vesc_brake_stop_speed', 0.05)

        # Sensor noise parameters for realistic simulation
        self.declare_parameter('odom_noise_enabled', False)  # Enable odometry noise
        self.declare_parameter('odom_pos_noise_std', 0.01)   # Position noise std dev (m)
        self.declare_parameter('odom_vel_noise_std', 0.05)   # Velocity noise std dev (m/s)
        self.declare_parameter('odom_yaw_noise_std', 0.005)  # Yaw noise std dev (rad)

        # TF frame configuration
        self.declare_parameter('tf_frame_id', 'map')         # Parent frame for TF (map or odom)
        self.declare_parameter('odom_frame_id', 'map')       # Parent frame for odom topic
        self.declare_parameter('publish_odom', True)
        self.declare_parameter('publish_base_tf', True)

        # Optional simulated VESC sensor outputs for dead-reckoned odometry.
        self.declare_parameter('publish_sim_vesc_sensors', False)
        self.declare_parameter('sim_vesc_state_topic', 'sensors/core')
        self.declare_parameter('sim_vesc_imu_topic', 'sensors/imu/raw')
        self.declare_parameter('sim_vesc_servo_topic', 'sensors/servo_position_command')
        self.declare_parameter('sim_vesc_speed_to_erpm_gain', 4550.0)
        self.declare_parameter('sim_vesc_speed_to_erpm_offset', 0.0)
        self.declare_parameter('sim_vesc_speed_scale', 1.0)
        self.declare_parameter('sim_vesc_speed_bias', 0.0)
        self.declare_parameter('sim_vesc_speed_noise_std', 0.04)
        self.declare_parameter('sim_vesc_yaw_rate_bias', 0.0)
        self.declare_parameter('sim_vesc_yaw_rate_noise_std', 0.05)
        self.declare_parameter('sim_vesc_accel_noise_std', 0.20)
        self.declare_parameter('sim_vesc_servo_gain', -0.7284)
        self.declare_parameter('sim_vesc_servo_offset', 0.5500)
        self.declare_parameter('sim_vesc_steering_correction_c2', 0.589566)
        self.declare_parameter('sim_vesc_steering_correction_c1', 0.918061)
        self.declare_parameter('sim_vesc_steering_correction_c0', 0.001490)

        # Scan publish rate limiting (for realistic 40Hz LiDAR simulation)
        self.declare_parameter('scan_publish_rate', 0.0)     # 0 = no limit, >0 = Hz limit

        # Vehicle parameter overrides (0.0 = use default from f1tenth_gym)
        self.declare_parameter('vehicle_mu', 0.0)
        self.declare_parameter('vehicle_m', 0.0)
        self.declare_parameter('vehicle_I', 0.0)
        self.declare_parameter('vehicle_C_Sf', 0.0)
        self.declare_parameter('vehicle_C_Sr', 0.0)
        self.declare_parameter('vehicle_lf', 0.0)
        self.declare_parameter('vehicle_lr', 0.0)
        self.declare_parameter('vehicle_h', 0.0)
        self.declare_parameter('vehicle_s_max', 0.0)
        self.declare_parameter('vehicle_sv_max', 0.0)
        self.declare_parameter('vehicle_a_max', 0.0)
        self.declare_parameter('vehicle_v_max', 0.0)
        self.declare_parameter('vehicle_v_min', 0.0)
        self.declare_parameter('vehicle_v_switch', 0.0)
        self.declare_parameter('vehicle_width', 0.0)
        self.declare_parameter('vehicle_length', 0.0)
        self.declare_parameter('realistic_plant_enabled', False)
        self.declare_parameter('realistic_actuation_enabled', True)
        self.declare_parameter('vehicle_mu_front', 0.775687)
        self.declare_parameter('vehicle_mu_rear', 0.6565520426481404)
        self.declare_parameter('vehicle_C_Sf_high_slip', 2.4199490875105907)
        self.declare_parameter('vehicle_C_Sr_high_slip', 2.73123678240426)
        self.declare_parameter('vehicle_steer_gain', 1.0085301459687404)
        self.declare_parameter('vehicle_steer_gain_high_slip', 0.6541720766809247)
        self.declare_parameter('vehicle_roll_resistance_n', 2.79)
        self.declare_parameter('vehicle_drag_c0', 0.0)
        self.declare_parameter('vehicle_drag_c1', 0.05)
        self.declare_parameter('vehicle_drag_c2', 0.04)
        self.declare_parameter('vehicle_accel_tau_pos', 0.05)
        self.declare_parameter('vehicle_accel_tau_neg', 0.12)
        self.declare_parameter('vehicle_accel_gain_pos', 0.575)
        self.declare_parameter('vehicle_accel_gain_neg', 1.0)
        self.declare_parameter('vehicle_pacejka_c', 1.6041121492252324)
        self.declare_parameter('vehicle_pacejka_c_front', 1.8031639754063644)
        self.declare_parameter('vehicle_pacejka_c_rear', 1.7681655069132207)
        self.declare_parameter('vehicle_slip_blend_start_front', 0.1643527788471148)
        self.declare_parameter('vehicle_slip_blend_end_front', 0.5319307735091576)
        self.declare_parameter('vehicle_slip_blend_start_rear', 0.2502122916247753)
        self.declare_parameter('vehicle_slip_blend_end_rear', 0.47793678552502167)
        self.declare_parameter('vehicle_combined_slip_gain', 0.10359393575265835)
        self.declare_parameter('vehicle_front_peak_drop', 0.11804981810838257)
        self.declare_parameter('vehicle_front_peak_drop_start', 0.13813810031946996)
        self.declare_parameter('vehicle_front_peak_drop_end', 0.48938120479012814)
        self.declare_parameter('vehicle_front_peak_drop_pow', 1.03)
        self.declare_parameter('vehicle_front_combined_gain', 0.13366870620631957)
        self.declare_parameter('vehicle_front_peak_floor', 0.2708096984131235)
        self.declare_parameter('vehicle_clamp_velocity_state', True)

    def _validate_num_agents(self) -> int:
        """Validate and return number of agents."""
        num_agents = self.get_parameter('num_agent').value
        if not isinstance(num_agents, int):
            raise ValueError('num_agents should be an int.')
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        return num_agents

    def _get_vehicle_params(self) -> Dict[str, Any]:
        """Get vehicle parameters based on configuration, with YAML overrides."""
        vehicle_type = self.get_parameter('vehicle_params').value
        params_map = {
            'f1tenth': F110Env.f1tenth_vehicle_params,
            'fullscale': F110Env.fullscale_vehicle_params,
            'f1fifth': F110Env.f1fifth_vehicle_params,
        }
        if vehicle_type not in params_map:
            raise ValueError(
                f'vehicle_params should be one of: {list(params_map.keys())}'
            )
        params = params_map[vehicle_type]()

        # Apply YAML overrides for real car values
        overrides = {
            'mu': self.get_parameter('vehicle_mu').value,
            'm': self.get_parameter('vehicle_m').value,
            'I': self.get_parameter('vehicle_I').value,
            'C_Sf': self.get_parameter('vehicle_C_Sf').value,
            'C_Sr': self.get_parameter('vehicle_C_Sr').value,
            'lf': self.get_parameter('vehicle_lf').value,
            'lr': self.get_parameter('vehicle_lr').value,
            'h': self.get_parameter('vehicle_h').value,
            's_max': self.get_parameter('vehicle_s_max').value,
            'sv_max': self.get_parameter('vehicle_sv_max').value,
            'a_max': self.get_parameter('vehicle_a_max').value,
            'v_max': self.get_parameter('vehicle_v_max').value,
            'v_min': self.get_parameter('vehicle_v_min').value,
            'v_switch': self.get_parameter('vehicle_v_switch').value,
            'width': self.get_parameter('vehicle_width').value,
            'length': self.get_parameter('vehicle_length').value,
        }
        # Also mirror s_max → s_min and sv_max → sv_min
        s_max_val = self.get_parameter('vehicle_s_max').value
        sv_max_val = self.get_parameter('vehicle_sv_max').value
        for key, val in overrides.items():
            if val > 0.0:
                old_val = params[key]
                params[key] = val
                self.get_logger().info(
                    f'Vehicle param override: {key} = {val} (default was {old_val})'
                )
        # v_min is negative, so handle separately
        v_min_val = self.get_parameter('vehicle_v_min').value
        if v_min_val < 0.0:
            old_val = params.get('v_min', -5.0)
            params['v_min'] = v_min_val
            self.get_logger().info(
                f'Vehicle param override: v_min = {v_min_val} (default was {old_val})'
            )
        # Mirror symmetric limits
        if s_max_val > 0.0:
            params['s_min'] = -s_max_val
            self.get_logger().info(
                f'Vehicle param override: s_min = {-s_max_val} (mirrored from s_max)'
            )
        if sv_max_val > 0.0:
            params['sv_min'] = -sv_max_val
            self.get_logger().info(
                f'Vehicle param override: sv_min = {-sv_max_val} (mirrored from sv_max)'
            )

        realistic_enabled = self.get_parameter('realistic_plant_enabled').value
        params['realistic_plant_enabled'] = bool(realistic_enabled)
        if realistic_enabled:
            params.update({
                'realistic_actuation_enabled': self.get_parameter(
                    'realistic_actuation_enabled').value,
                'realistic_drive_enabled': True,
                'pacejka_tires_enabled': True,
                'mu_front': self.get_parameter('vehicle_mu_front').value,
                'mu_rear': self.get_parameter('vehicle_mu_rear').value,
                'C_Sf_high_slip': self.get_parameter('vehicle_C_Sf_high_slip').value,
                'C_Sr_high_slip': self.get_parameter('vehicle_C_Sr_high_slip').value,
                'steer_gain': self.get_parameter('vehicle_steer_gain').value,
                'steer_gain_high_slip': self.get_parameter('vehicle_steer_gain_high_slip').value,
                'roll_resistance_n': self.get_parameter('vehicle_roll_resistance_n').value,
                'drag_c0': self.get_parameter('vehicle_drag_c0').value,
                'drag_c1': self.get_parameter('vehicle_drag_c1').value,
                'drag_c2': self.get_parameter('vehicle_drag_c2').value,
                'accel_tau_pos': self.get_parameter('vehicle_accel_tau_pos').value,
                'accel_tau_neg': self.get_parameter('vehicle_accel_tau_neg').value,
                'accel_gain_pos': self.get_parameter('vehicle_accel_gain_pos').value,
                'accel_gain_neg': self.get_parameter('vehicle_accel_gain_neg').value,
                'pacejka_c': self.get_parameter('vehicle_pacejka_c').value,
                'pacejka_c_front': self.get_parameter('vehicle_pacejka_c_front').value,
                'pacejka_c_rear': self.get_parameter('vehicle_pacejka_c_rear').value,
                'slip_blend_start_front': self.get_parameter(
                    'vehicle_slip_blend_start_front').value,
                'slip_blend_end_front': self.get_parameter('vehicle_slip_blend_end_front').value,
                'slip_blend_start_rear': self.get_parameter('vehicle_slip_blend_start_rear').value,
                'slip_blend_end_rear': self.get_parameter('vehicle_slip_blend_end_rear').value,
                'combined_slip_gain': self.get_parameter('vehicle_combined_slip_gain').value,
                'front_peak_drop': self.get_parameter('vehicle_front_peak_drop').value,
                'front_peak_drop_start': self.get_parameter('vehicle_front_peak_drop_start').value,
                'front_peak_drop_end': self.get_parameter('vehicle_front_peak_drop_end').value,
                'front_peak_drop_pow': self.get_parameter('vehicle_front_peak_drop_pow').value,
                'front_combined_gain': self.get_parameter('vehicle_front_combined_gain').value,
                'front_peak_floor': self.get_parameter('vehicle_front_peak_floor').value,
                'clamp_velocity_state': self.get_parameter('vehicle_clamp_velocity_state').value,
            })
            self.get_logger().info(
                'Realistic plant enabled: Pacejka tires, steering compliance, drag, accel lag'
            )

        return params

    def _create_environment(self, num_agents: int, scale: float) -> gym.Env:
        """Create and configure the F1Tenth gym environment."""
        map_name = self.get_parameter('map_path').value
        map_yaml_path = _resolve_map_yaml_path(map_name)
        self.get_logger().info(f'Loading map: {map_name} from path: {map_yaml_path}')

        # Load the track
        try:
            loaded_map = Track.from_track_path(pathlib.Path(map_yaml_path), scale)
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            raise

        # Check if headless mode (no rendering)
        headless = self.get_parameter('headless').value
        render_mode = None if headless else 'rgb_array'
        if headless:
            self.get_logger().info('Running in HEADLESS mode (no rendering)')

        # Create environment
        sim_timestep = self.get_parameter('sim_timestep').value
        try:
            env = gym.make(
                'f1tenth-v0',
                config={
                    'map': loaded_map,
                    'num_agents': num_agents,
                    'timestep': sim_timestep,
                    'integrator': 'rk4',
                    'control_input': ['accl', 'steering_angle'],
                    'model': 'st',
                    'observation_config': {'type': 'original'},
                    'params': self.vehicle_params,
                    'reset_config': {'type': 'map_random_static'},
                    'scale': scale,
                    'lidar_dist': self.get_parameter('scan_distance_to_base_link').value
                },
                render_mode=render_mode,
            )
        except Exception as e:
            self.get_logger().error(f'Failed to create gym environment: {e}')
            raise
        return env

    def _init_ego_state(self) -> None:
        """Initialize ego agent state variables."""
        sx = self.get_parameter('sx').value
        sy = self.get_parameter('sy').value
        stheta = self.get_parameter('stheta').value

        self.ego_pose: List[float] = [sx, sy, stheta]
        self.ego_speed: List[float] = [0.0, 0.0, 0.0]
        self.ego_requested_speed: float = 0.0
        self.ego_steer: float = 0.0
        self.ego_collision: bool = False
        self.ego_scan: List[float] = []
        self.ego_drive_published = False

        # Scan parameters
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self._scan_beams = int(scan_beams)
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        # LaserScan samples include both angular endpoints. Match the gym scan
        # simulator and preserve angle_max = angle_min + (N-1)*increment.
        self.angle_inc = scan_fov / max(1, scan_beams - 1)
        self.scan_range_max = self.get_parameter('scan_range_max').value

        self.ego_namespace = self.get_parameter('ego_namespace').value
        self.scan_distance_to_base_link = self.get_parameter(
            'scan_distance_to_base_link').value

    def _init_opponent_state(self) -> None:
        """Initialize opponent agent state variables."""
        self.has_opp = True
        self.opp_namespace = self.get_parameter('opp_namespace').value

        sx1 = self.get_parameter('sx1').value
        sy1 = self.get_parameter('sy1').value
        stheta1 = self.get_parameter('stheta1').value

        self.opp_pose: List[float] = [sx1, sy1, stheta1]
        self.opp_speed: List[float] = [0.0, 0.0, 0.0]
        self.opp_requested_speed: float = 0.0
        self.opp_steer: float = 0.0
        self.opp_collision: bool = False
        self.opp_scan: List[float] = []
        self.opp_drive_published = False

    def _reset_environment(self, num_agents: int) -> None:
        """Reset environment with initial poses."""
        try:
            if num_agents == 2:
                self.has_opp = True
                poses = np.array([self.ego_pose, self.opp_pose])
                self.obs, _ = self.env.reset(options={'poses': poses})
                self.ego_scan = self.obs['scans'][0].tolist()
                self.opp_scan = self.obs['scans'][1].tolist()
            else:
                self.has_opp = False
                poses = np.array([self.ego_pose])
                self.obs, _ = self.env.reset(options={'poses': poses})
                self.ego_scan = self.obs['scans'][0].tolist()
        except Exception as e:
            self.get_logger().error(f'Failed to reset environment: {e}')
            raise

    def _preallocate_messages(self, num_agents: int) -> None:
        """Pre-allocate ROS messages to reduce allocation overhead in hot paths."""
        # Pre-allocate scan messages
        self._ego_scan_msg = LaserScan()
        self._ego_scan_msg.angle_min = self.angle_min
        self._ego_scan_msg.angle_max = self.angle_max
        self._ego_scan_msg.angle_increment = self.angle_inc
        self._ego_scan_msg.range_min = 0.0
        self._ego_scan_msg.range_max = self.scan_range_max
        self._ego_scan_msg.header.frame_id = f'{self.ego_namespace}/laser'

        # Pre-allocate odometry messages
        self._ego_odom_msg = Odometry()
        self._ego_odom_msg.header.frame_id = self._odom_frame_id
        self._ego_odom_msg.child_frame_id = f'{self.ego_namespace}/base_link'

        # Pre-allocate ground truth odometry message (noise-free)
        self._ego_gt_msg = Odometry()
        self._ego_gt_msg.header.frame_id = 'map'
        self._ego_gt_msg.child_frame_id = f'{self.ego_namespace}/base_link'

        # Pre-allocate transform messages
        self._ego_tf_msg = TransformStamped()
        self._ego_tf_msg.header.frame_id = self._tf_frame_id
        self._ego_tf_msg.child_frame_id = f'{self.ego_namespace}/base_link'

        # Pre-allocate wheel transforms
        self._ego_left_wheel_tf = TransformStamped()
        self._ego_left_wheel_tf.header.frame_id = f'{self.ego_namespace}/front_left_hinge'
        self._ego_left_wheel_tf.child_frame_id = f'{self.ego_namespace}/front_left_wheel'

        self._ego_right_wheel_tf = TransformStamped()
        self._ego_right_wheel_tf.header.frame_id = f'{self.ego_namespace}/front_right_hinge'
        self._ego_right_wheel_tf.child_frame_id = f'{self.ego_namespace}/front_right_wheel'

        # Pre-allocate clock message
        self._clock_msg = Clock()

        # Pre-allocate simulated VESC sensor messages.
        self._sim_vesc_state_msg = VescStateStamped()
        self._sim_vesc_state_msg.header.frame_id = f'{self.ego_namespace}/vesc'
        self._sim_vesc_imu_msg = Imu()
        self._sim_vesc_imu_msg.header.frame_id = f'{self.ego_namespace}/imu'
        self._sim_vesc_servo_msg = Float64()

        if num_agents == 2:
            self._opp_scan_msg = LaserScan()
            self._opp_scan_msg.angle_min = self.angle_min
            self._opp_scan_msg.angle_max = self.angle_max
            self._opp_scan_msg.angle_increment = self.angle_inc
            self._opp_scan_msg.range_min = 0.0
            self._opp_scan_msg.range_max = self.scan_range_max
            self._opp_scan_msg.header.frame_id = f'{self.opp_namespace}/laser'

            self._opp_odom_msg = Odometry()
            self._opp_odom_msg.header.frame_id = self._odom_frame_id
            self._opp_odom_msg.child_frame_id = f'{self.opp_namespace}/base_link'

            self._opp_tf_msg = TransformStamped()
            self._opp_tf_msg.header.frame_id = self._tf_frame_id
            self._opp_tf_msg.child_frame_id = f'{self.opp_namespace}/base_link'

            self._opp_left_wheel_tf = TransformStamped()
            self._opp_left_wheel_tf.header.frame_id = f'{self.opp_namespace}/front_left_hinge'
            self._opp_left_wheel_tf.child_frame_id = f'{self.opp_namespace}/front_left_wheel'

            self._opp_right_wheel_tf = TransformStamped()
            self._opp_right_wheel_tf.header.frame_id = f'{self.opp_namespace}/front_right_hinge'
            self._opp_right_wheel_tf.child_frame_id = f'{self.opp_namespace}/front_right_wheel'

    def _setup_timers(self, num_agents: int) -> None:
        """Set up simulation timers based on async/sync mode."""
        if not self.get_parameter('async_mode').value:
            self.get_logger().info(
                'Running in synchronous mode. '
                'Simulation will step only on new /drive messages.'
            )
            self.timer = self.create_timer(
                SYNC_MODE_TIMER_PERIOD, self.timer_callback)
        else:
            self.get_logger().info(
                'Running in asynchronous mode. '
                'Simulation will step using a timer callback.'
            )
            sim_timestep = self.get_parameter('sim_timestep').value
            self.drive_timer = self.create_timer(
                sim_timestep, self.drive_timer_callback)
            self.timer = self.create_timer(
                PUBLISH_TIMER_PERIOD, self.timer_callback)

        # Transform broadcasters
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        # Publish static laser transforms (these never change)
        self._publish_static_laser_transforms(num_agents)

        # Simulation state
        self.sim_paused = False
        self.done = False

    def _publish_static_laser_transforms(self, num_agents: int) -> None:
        """Publish static transforms for laser frames (relative to base_link)."""
        static_transforms = []

        # Ego laser transform
        ego_laser_tf = TransformStamped()
        ego_laser_tf.header.stamp = self.get_clock().now().to_msg()
        ego_laser_tf.header.frame_id = f'{self.ego_namespace}/base_link'
        ego_laser_tf.child_frame_id = f'{self.ego_namespace}/laser'
        ego_laser_tf.transform.translation.x = self.scan_distance_to_base_link
        ego_laser_tf.transform.translation.y = 0.0
        ego_laser_tf.transform.translation.z = 0.0
        ego_laser_tf.transform.rotation.w = IDENTITY_QUAT[0]
        ego_laser_tf.transform.rotation.x = IDENTITY_QUAT[1]
        ego_laser_tf.transform.rotation.y = IDENTITY_QUAT[2]
        ego_laser_tf.transform.rotation.z = IDENTITY_QUAT[3]
        static_transforms.append(ego_laser_tf)

        if num_agents == 2:
            opp_laser_tf = TransformStamped()
            opp_laser_tf.header.stamp = self.get_clock().now().to_msg()
            opp_laser_tf.header.frame_id = f'{self.opp_namespace}/base_link'
            opp_laser_tf.child_frame_id = f'{self.opp_namespace}/laser'
            opp_laser_tf.transform.translation.x = self.scan_distance_to_base_link
            opp_laser_tf.transform.translation.y = 0.0
            opp_laser_tf.transform.translation.z = 0.0
            opp_laser_tf.transform.rotation.w = IDENTITY_QUAT[0]
            opp_laser_tf.transform.rotation.x = IDENTITY_QUAT[1]
            opp_laser_tf.transform.rotation.y = IDENTITY_QUAT[2]
            opp_laser_tf.transform.rotation.z = IDENTITY_QUAT[3]
            static_transforms.append(opp_laser_tf)

        self.static_br.sendTransform(static_transforms)

    def _setup_publishers(self, num_agents: int) -> None:
        """Set up all publishers."""
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_odom_param = self.get_parameter('ego_odom_topic').value
        ego_odom_topic = f'{self.ego_namespace}/{ego_odom_param}'

        self.ego_scan_pub = self.create_publisher(
            LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_gt_pub = self.create_publisher(
            Odometry, f'{self.ego_namespace}/ground_truth', 10)
        self.ego_drive_published = False

        # Collision publishers for downstream nodes
        self.ego_collision_pub = self.create_publisher(
            Bool, f'{self.ego_namespace}/collision', 10
        )

        if self._publish_sim_vesc_sensors:
            self.sim_vesc_state_pub = self.create_publisher(
                VescStateStamped, self._sim_vesc_state_topic, 10)
            self.sim_vesc_imu_pub = self.create_publisher(
                Imu, self._sim_vesc_imu_topic, 10)
            self.sim_vesc_servo_pub = self.create_publisher(
                Float64, self._sim_vesc_servo_topic, 10)

        if num_agents == 2:
            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_param = self.get_parameter('opp_odom_topic').value
            opp_odom_topic = f'{self.opp_namespace}/{opp_odom_param}'
            ego_opp_param = self.get_parameter('ego_opp_odom_topic').value
            ego_opp_odom_topic = f'{self.ego_namespace}/{ego_opp_param}'
            opp_ego_param = self.get_parameter('opp_ego_odom_topic').value
            opp_ego_odom_topic = f'{self.opp_namespace}/{opp_ego_param}'

            self.opp_scan_pub = self.create_publisher(
                LaserScan, opp_scan_topic, 10)
            self.opp_odom_pub = self.create_publisher(
                Odometry, opp_odom_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(
                Odometry, ego_opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(
                Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

            self.opp_collision_pub = self.create_publisher(
                Bool, f'{self.opp_namespace}/collision', 10
            )

        if self.get_parameter('use_sim_time_bridge').value:
            self.get_logger().info('Using simulation time. Will publish /clock topic.')
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            if self.get_parameter('async_mode').value:
                self.drive_timer.timer_period_ns = 0
                self.timer.timer_period_ns = 0

    def _setup_subscribers(self, num_agents: int) -> None:
        """Set up all subscribers."""
        if self._drive_input_mode == 'vesc':
            self.ego_current_sub = self.create_subscription(
                Float64,
                self._sim_vesc_current_topic,
                self.sim_vesc_current_callback,
                10)
            self.ego_brake_sub = self.create_subscription(
                Float64,
                self._sim_vesc_brake_topic,
                self.sim_vesc_brake_callback,
                10)
            self.ego_motor_speed_sub = self.create_subscription(
                Float64,
                self._sim_vesc_motor_speed_topic,
                self.sim_vesc_motor_speed_callback,
                10)
            self.ego_servo_command_sub = self.create_subscription(
                Float64,
                self._sim_vesc_servo_command_topic,
                self.sim_vesc_servo_callback,
                10)
        else:
            ego_drive_topic = self.get_parameter('ego_drive_topic').value
            self.ego_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                ego_drive_topic,
                self.drive_callback,
                10)
        self.ego_reset_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/initialpose',
            self.ego_reset_callback,
            10)

        if num_agents == 2:
            opp_drive_topic = self.get_parameter('opp_drive_topic').value
            self.opp_drive_sub = self.create_subscription(
                AckermannDriveStamped,
                opp_drive_topic,
                self.opp_drive_callback,
                10)
            self.opp_reset_sub = self.create_subscription(
                PoseStamped,
                '/goal_pose',
                self.opp_reset_callback,
                10)

        if self.get_parameter('kb_teleop').value:
            self.teleop_sub = self.create_subscription(
                Twist,
                '/cmd_vel',
                self.teleop_callback,
                10)

        self.pause_subscriber = self.create_subscription(
            Bool,
            '/pause_sim',
            self.pause_callback,
            10)

    # === Callback Methods ===

    def pause_callback(self, msg: Bool) -> None:
        """Handle pause/resume simulation messages."""
        self.sim_paused = msg.data
        status = 'paused' if self.sim_paused else 'resumed'
        self.get_logger().info(f'Simulation {status}')

    def _mark_sim_vesc_command(self) -> None:
        self._sim_vesc_last_command_wall_sec = time.perf_counter()
        self.ego_drive_published = True

    def _servo_to_steering_angle(self, servo_position: float) -> float:
        corrected = (
            servo_position - self._sim_vesc_servo_offset
        ) / self._sim_vesc_servo_gain
        corrected_abs = abs(corrected)
        c2 = self._sim_vesc_steering_correction_c2
        c1 = self._sim_vesc_steering_correction_c1
        c0 = self._sim_vesc_steering_correction_c0

        if corrected_abs <= c0:
            steer_abs = 0.0
        elif abs(c2) < 1e-9:
            steer_abs = (corrected_abs - c0) / c1 if abs(c1) > 1e-9 else 0.0
        else:
            discriminant = max(c1 * c1 - 4.0 * c2 * (c0 - corrected_abs), 0.0)
            steer_abs = (-c1 + np.sqrt(discriminant)) / (2.0 * c2)

        return float(np.clip(
            np.copysign(max(steer_abs, 0.0), corrected),
            self.vehicle_params['s_min'],
            self.vehicle_params['s_max']))

    def sim_vesc_current_callback(self, msg: Float64) -> None:
        """Convert simulated VESC motor current to gym acceleration command."""
        if self.sim_paused:
            return

        self._sim_vesc_last_motor_current = float(msg.data)
        self._sim_vesc_last_brake_current = 0.0
        self._sim_vesc_last_input_current = max(float(msg.data), 0.0)
        requested_accel = (
            float(msg.data) /
            self._sim_vesc_accel_to_current_gain *
            self._sim_vesc_current_accel_scale)
        self.ego_requested_speed = float(np.clip(
            requested_accel,
            -self._sim_vesc_motor_accel_limit,
            self._sim_vesc_motor_accel_limit))
        self._mark_sim_vesc_command()

    def sim_vesc_brake_callback(self, msg: Float64) -> None:
        """Convert simulated VESC brake current to gym deceleration command."""
        if self.sim_paused:
            return

        self._sim_vesc_last_motor_current = 0.0
        self._sim_vesc_last_brake_current = max(float(msg.data), 0.0)
        self._sim_vesc_last_input_current = -self._sim_vesc_last_brake_current
        # A VESC brake-current command dissipates kinetic energy; it must not
        # become a reverse-throttle command after crossing zero speed. This is
        # especially important when AckermannToVesc emits its large emergency
        # brake value for a zero-speed request.
        brake_accel = (
            self._sim_vesc_last_brake_current /
            self._sim_vesc_accel_to_brake_gain *
            self._sim_vesc_brake_accel_scale)
        brake_accel = float(np.clip(
            brake_accel, 0.0, self._sim_vesc_motor_accel_limit))
        longitudinal_speed = float(self.ego_speed[0])
        if longitudinal_speed > self._sim_vesc_brake_stop_speed:
            self.ego_requested_speed = -brake_accel
        elif longitudinal_speed < -self._sim_vesc_brake_stop_speed:
            self.ego_requested_speed = brake_accel
        else:
            self.ego_requested_speed = 0.0
        self._mark_sim_vesc_command()

    def sim_vesc_motor_speed_callback(self, msg: Float64) -> None:
        """Convert simulated ERPM command to a bounded acceleration request."""
        if self.sim_paused:
            return

        target_speed = (
            float(msg.data) - self._sim_vesc_speed_to_erpm_offset
        ) / self._sim_vesc_speed_to_erpm_gain
        accel = (
            target_speed - float(self.ego_speed[0])
        ) / self._sim_vesc_erpm_speed_tau
        accel = float(np.clip(
            accel,
            -self._sim_vesc_motor_accel_limit,
            self._sim_vesc_motor_accel_limit))
        self._sim_vesc_last_motor_current = max(
            accel * self._sim_vesc_accel_to_current_gain, 0.0)
        self._sim_vesc_last_brake_current = max(
            -accel * self._sim_vesc_accel_to_brake_gain, 0.0)
        self._sim_vesc_last_input_current = (
            self._sim_vesc_last_motor_current -
            self._sim_vesc_last_brake_current)
        self.ego_requested_speed = accel
        self._mark_sim_vesc_command()

    def sim_vesc_servo_callback(self, msg: Float64) -> None:
        """Convert simulated servo position command back to steering angle."""
        if self.sim_paused:
            return

        self.ego_steer = self._servo_to_steering_angle(float(msg.data))

    def drive_callback(self, drive_msg: AckermannDriveStamped) -> None:
        """Handle ego drive commands."""
        if self.sim_paused:
            return

        if self.get_parameter('drive_uses_acceleration_field').value:
            self.ego_requested_speed = drive_msg.drive.acceleration
        else:
            self.ego_requested_speed = drive_msg.drive.speed
        self.ego_drive_published = True
        self.ego_steer = np.clip(
            drive_msg.drive.steering_angle,
            self.vehicle_params['s_min'],
            self.vehicle_params['s_max']
        )

        if not self.get_parameter('async_mode').value:
            self.drive_timer_callback()
            self.timer_callback()

    def opp_drive_callback(self, drive_msg: AckermannDriveStamped) -> None:
        """Handle opponent drive commands."""
        if self.sim_paused:
            return

        if self.get_parameter('drive_uses_acceleration_field').value:
            self.opp_requested_speed = drive_msg.drive.acceleration
        else:
            self.opp_requested_speed = drive_msg.drive.speed
        self.opp_drive_published = True
        self.opp_steer = np.clip(
            drive_msg.drive.steering_angle,
            self.vehicle_params['s_min'],
            self.vehicle_params['s_max']
        )

        if not self.get_parameter('async_mode').value:
            self.drive_timer_callback()
            self.timer_callback()

    def ego_reset_callback(self, pose_msg: PoseWithCovarianceStamped) -> None:
        """Handle ego reset pose from RViz 2D Pose Estimate."""
        if self.sim_paused:
            return

        rx, ry, rtheta = self._extract_pose_2d(
            pose_msg.pose.pose.position,
            pose_msg.pose.pose.orientation
        )

        try:
            if self.has_opp:
                opp_pose = [
                    self.obs['poses_x'][1],
                    self.obs['poses_y'][1],
                    self.obs['poses_theta'][1]
                ]
                self.obs, _ = self.env.reset(
                    options={'poses': np.array([[rx, ry, rtheta], opp_pose])})
            else:
                self.obs, _ = self.env.reset(
                    options={'poses': np.array([[rx, ry, rtheta]])})
        except Exception as e:
            self.get_logger().error(f'Failed to reset ego pose: {e}')

    def opp_reset_callback(self, pose_msg: PoseStamped) -> None:
        """Handle opponent reset pose from RViz 2D Goal Pose."""
        if self.sim_paused or not self.has_opp:
            return

        rx, ry, rtheta = self._extract_pose_2d(
            pose_msg.pose.position,
            pose_msg.pose.orientation
        )

        try:
            self.obs, _ = self.env.reset(
                options={'poses': np.array([self.ego_pose, [rx, ry, rtheta]])})
        except Exception as e:
            self.get_logger().error(f'Failed to reset opponent pose: {e}')

    def teleop_callback(self, twist_msg: Twist) -> None:
        """Handle keyboard teleop commands."""
        if self.sim_paused:
            return

        self.ego_requested_speed = twist_msg.linear.x
        self.ego_drive_published = True
        teleop_steer = self.get_parameter('teleop_steering_angle').value

        if twist_msg.angular.z > 0.0:
            self.ego_steer = teleop_steer
        elif twist_msg.angular.z < 0.0:
            self.ego_steer = -teleop_steer
        else:
            self.ego_steer = 0.0

    def drive_timer_callback(self) -> None:
        """Step the simulation forward."""
        if self.sim_paused:
            return
        if (self.vehicle_params.get('realistic_plant_enabled', False) and
                not self.ego_drive_published):
            return
        if (self._drive_input_mode == 'vesc' and
                self._sim_vesc_last_command_wall_sec is not None and
                self._sim_vesc_command_timeout_sec > 0.0 and
                time.perf_counter() - self._sim_vesc_last_command_wall_sec >
                self._sim_vesc_command_timeout_sec):
            self.ego_requested_speed = 0.0
            self._sim_vesc_last_motor_current = 0.0
            self._sim_vesc_last_brake_current = 0.0
            self._sim_vesc_last_input_current = 0.0

        start_time = time.perf_counter() if self._enable_perf_metrics else None

        try:
            if not self.has_opp:
                action = np.array([[self.ego_steer, self.ego_requested_speed]])
            else:
                action = np.array([
                    [self.ego_steer, self.ego_requested_speed],
                    [self.opp_steer, self.opp_requested_speed]
                ])
            self.obs, _, self.done, _, _ = self.env.step(action)
        except Exception as e:
            self.get_logger().error(f'Simulation step failed: {e}')
            return

        self._update_sim_state()
        self._publish_collisions()

        if self.get_parameter('use_sim_time_bridge').value:
            self._publish_clock()

        if self._enable_perf_metrics and start_time is not None:
            elapsed = (time.perf_counter() - start_time) * 1000  # ms
            self._loop_times.append(elapsed)
            if len(self._loop_times) >= 1000:
                avg = sum(self._loop_times) / len(self._loop_times)
                self.get_logger().info(f'Avg sim step time: {avg:.2f}ms')
                self._loop_times.clear()

        # Real-time throttle: sleep if sim is running ahead of wall clock
        if self._real_time_factor > 0:
            sim_time = self.env.unwrapped.current_time
            if not self._rt_initialized:
                self._rt_sim_start = sim_time
                self._rt_wall_start = time.perf_counter()
                self._rt_initialized = True
            else:
                sim_elapsed = (sim_time - self._rt_sim_start) / self._real_time_factor
                wall_elapsed = time.perf_counter() - self._rt_wall_start
                sleep_time = sim_elapsed - wall_elapsed
                if sleep_time > 0.0001:  # Only sleep if > 0.1ms ahead
                    time.sleep(sleep_time)

    def timer_callback(self) -> None:
        """Publish sensor data and transforms."""
        if self.sim_paused:
            return

        ts = self._get_timestamp()
        self._publish_scans(ts)
        self._publish_odom(ts)
        self._publish_sim_vesc_sensors_msg(ts)
        self._publish_transforms(ts)
        self._publish_wheel_transforms(ts)

    # === Helper Methods ===

    def _extract_pose_2d(
            self, position, orientation) -> Tuple[float, float, float]:
        """Extract 2D pose (x, y, theta) from position and orientation."""
        x = position.x
        y = position.y
        _, _, theta = euler.quat2euler(
            [orientation.w, orientation.x, orientation.y, orientation.z],
            axes='sxyz'
        )
        return x, y, theta

    def _get_timestamp(self):
        """Get current timestamp, using sim time if configured."""
        ts = self.get_clock().now().to_msg()
        if self.get_parameter('use_sim_time_bridge').value:
            ts.sec = int(self.env.unwrapped.current_time // 1.0)
            ts.nanosec = int((self.env.unwrapped.current_time % 1.0) * 1e9)
        return ts

    def _publish_clock(self) -> None:
        """Publish simulation clock using pre-allocated message."""
        self._clock_msg.clock.sec = int(self.env.unwrapped.current_time // 1.0)
        self._clock_msg.clock.nanosec = int(
            (self.env.unwrapped.current_time % 1.0) * 1e9)
        self.clock_pub.publish(self._clock_msg)

    def _update_sim_state(self) -> None:
        """Update internal state from simulation observations."""
        # Use tolist() for NumPy arrays - more efficient than list()
        self.ego_scan = self.obs['scans'][0].tolist()
        self.ego_pose[0] = float(self.obs['poses_x'][0])
        self.ego_pose[1] = float(self.obs['poses_y'][0])
        self.ego_pose[2] = float(self.obs['poses_theta'][0])
        self.ego_speed[0] = float(self.obs['linear_vels_x'][0])
        self.ego_speed[1] = float(self.obs['linear_vels_y'][0])
        self.ego_speed[2] = float(self.obs['ang_vels_z'][0])

        # Update collision state
        new_ego_collision = bool(self.obs.get('collisions', [False])[0])
        self._ego_collision_changed = (new_ego_collision != self.ego_collision)
        self.ego_collision = new_ego_collision

        if self.has_opp:
            self.opp_scan = self.obs['scans'][1].tolist()
            self.opp_pose[0] = float(self.obs['poses_x'][1])
            self.opp_pose[1] = float(self.obs['poses_y'][1])
            self.opp_pose[2] = float(self.obs['poses_theta'][1])
            self.opp_speed[0] = float(self.obs['linear_vels_x'][1])
            self.opp_speed[1] = float(self.obs['linear_vels_y'][1])
            self.opp_speed[2] = float(self.obs['ang_vels_z'][1])

            new_opp_collision = bool(
                self.obs.get(
                    'collisions', [
                        False, False])[1])
            self._opp_collision_changed = (
                new_opp_collision != self.opp_collision)
            self.opp_collision = new_opp_collision

    def _publish_collisions(self) -> None:
        """Publish collision events when state changes."""
        if hasattr(
                self,
                '_ego_collision_changed') and self._ego_collision_changed:
            collision_msg = Bool()
            collision_msg.data = self.ego_collision
            self.ego_collision_pub.publish(collision_msg)
            if self.ego_collision:
                self.get_logger().warn('Ego vehicle collision detected!')

        if self.has_opp and hasattr(
                self, '_opp_collision_changed') and self._opp_collision_changed:
            collision_msg = Bool()
            collision_msg.data = self.opp_collision
            self.opp_collision_pub.publish(collision_msg)
            if self.opp_collision:
                self.get_logger().warn('Opponent vehicle collision detected!')

    def _publish_scans(self, ts) -> None:
        """Publish laser scan messages using pre-allocated messages."""
        # Check rate limiting with accumulated time for more accurate rate
        if self._scan_period > 0:
            current_time = time.time()
            elapsed = current_time - self._last_scan_time
            if elapsed < self._scan_period:
                return  # Skip this publish to maintain target rate
            # Advance by period (not current time) to maintain steady rate
            # This compensates for jitter by catching up
            self._last_scan_time += self._scan_period
            # But don't let it drift too far behind (max 2 periods behind)
            if current_time - self._last_scan_time > self._scan_period * 2:
                self._last_scan_time = current_time

        # Get noise stddev from parameter (default to 0.0 if not set)
        scan_noise_std = (
            self.get_parameter('scan_noise_std').value
            if self.has_parameter('scan_noise_std') else 0.0
        )

        # Ego scan
        ego_scan = np.array(self.ego_scan)
        ego_scan = self._downsample_scan(ego_scan)
        if scan_noise_std > 0.0:
            noise = np.random.normal(0, scan_noise_std, ego_scan.shape)
            ego_scan = ego_scan + noise
            # Clamp to valid range
            ego_scan = np.clip(ego_scan, 0.0, self.scan_range_max)
        self._ego_scan_msg.header.stamp = ts
        self._ego_scan_msg.ranges = ego_scan.tolist()
        self.ego_scan_pub.publish(self._ego_scan_msg)

        # Opponent scan
        if self.has_opp:
            opp_scan = np.array(self.opp_scan)
            opp_scan = self._downsample_scan(opp_scan)
            if scan_noise_std > 0.0:
                noise = np.random.normal(0, scan_noise_std, opp_scan.shape)
                opp_scan = opp_scan + noise
                opp_scan = np.clip(opp_scan, 0.0, self.scan_range_max)
            self._opp_scan_msg.header.stamp = ts
            self._opp_scan_msg.ranges = opp_scan.tolist()
            self.opp_scan_pub.publish(self._opp_scan_msg)

    def _downsample_scan(self, scan: np.ndarray) -> np.ndarray:
        target_beams = self._scan_beams
        if target_beams <= 0 or scan.size == target_beams:
            return scan

        if not self._scan_downsample_warned:
            self.get_logger().info(
                f'Downsampling scan from {scan.size} to {target_beams} beams for ROS output.')
            self._scan_downsample_warned = True

        if self._scan_downsample_idx is None or self._scan_downsample_source != scan.size:
            if scan.size % target_beams == 0:
                stride = scan.size // target_beams
                self._scan_downsample_idx = np.arange(0, scan.size, stride)
            else:
                self._scan_downsample_idx = np.linspace(
                    0, scan.size - 1, num=target_beams).astype(int)
            self._scan_downsample_source = scan.size

        return scan[self._scan_downsample_idx]

    def _publish_odom(self, ts) -> None:
        """Publish odometry messages using pre-allocated messages."""
        if self._publish_odom_topic:
            # Ego odom (with noise if enabled)
            self._update_odom_msg(
                self._ego_odom_msg,
                ts,
                self.ego_pose,
                self.ego_speed)
            self.ego_odom_pub.publish(self._ego_odom_msg)

        # Ego ground truth (always noise-free)
        self._update_gt_msg(self._ego_gt_msg, ts, self.ego_pose, self.ego_speed)
        self.ego_gt_pub.publish(self._ego_gt_msg)

        # Opponent odom
        if self.has_opp and self._publish_odom_topic:
            self._update_odom_msg(
                self._opp_odom_msg,
                ts,
                self.opp_pose,
                self.opp_speed)
            self.opp_odom_pub.publish(self._opp_odom_msg)
            self.opp_ego_odom_pub.publish(self._ego_odom_msg)
            self.ego_opp_odom_pub.publish(self._opp_odom_msg)

    def _update_odom_msg(self, odom: Odometry, ts, pose: List[float],
                         speed: List[float]) -> None:
        """Update an Odometry message in-place with optional noise injection."""
        odom.header.stamp = ts

        # Apply position noise if enabled
        if self._odom_noise_enabled:
            noisy_x = pose[0] + np.random.normal(0, self._odom_pos_noise_std)
            noisy_y = pose[1] + np.random.normal(0, self._odom_pos_noise_std)
            noisy_yaw = pose[2] + np.random.normal(0, self._odom_yaw_noise_std)
            noisy_vx = speed[0] + np.random.normal(0, self._odom_vel_noise_std)
            noisy_vy = speed[1] + np.random.normal(0, self._odom_vel_noise_std)
            noisy_wz = speed[2] + np.random.normal(0, self._odom_yaw_noise_std * 10)
        else:
            noisy_x, noisy_y, noisy_yaw = pose[0], pose[1], pose[2]
            noisy_vx, noisy_vy, noisy_wz = speed[0], speed[1], speed[2]

        odom.pose.pose.position.x = noisy_x
        odom.pose.pose.position.y = noisy_y

        quat = euler.euler2quat(0.0, 0.0, noisy_yaw, axes='sxyz')
        odom.pose.pose.orientation.w = quat[0]
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]

        odom.twist.twist.linear.x = noisy_vx
        odom.twist.twist.linear.y = noisy_vy
        odom.twist.twist.angular.z = noisy_wz

    def _update_gt_msg(self, gt: 'Odometry', ts, pose: List[float],
                       speed: List[float]) -> None:
        """Update a ground truth Odometry message (noise-free)."""
        gt.header.stamp = ts
        gt.pose.pose.position.x = pose[0]
        gt.pose.pose.position.y = pose[1]

        quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
        gt.pose.pose.orientation.w = quat[0]
        gt.pose.pose.orientation.x = quat[1]
        gt.pose.pose.orientation.y = quat[2]
        gt.pose.pose.orientation.z = quat[3]

        gt.twist.twist.linear.x = speed[0]
        gt.twist.twist.linear.y = speed[1]
        gt.twist.twist.angular.z = speed[2]

    def _stamp_to_sec(self, ts) -> float:
        return float(ts.sec) + float(ts.nanosec) * 1e-9

    def _publish_sim_vesc_sensors_msg(self, ts) -> None:
        """Publish VESC-like wheel speed, IMU, and servo signals from sim state."""
        if not self._publish_sim_vesc_sensors:
            return

        stamp_sec = self._stamp_to_sec(ts)
        vx = float(self.ego_speed[0])
        vy = float(self.ego_speed[1])
        yaw_rate = float(self.ego_speed[2])

        accel_x = 0.0
        accel_y = 0.0
        if self._sim_vesc_last_time_sec is not None:
            dt = stamp_sec - self._sim_vesc_last_time_sec
            if 1e-6 < dt < 0.5:
                accel_x = (vx - self._sim_vesc_last_vx) / dt
                accel_y = (vy - self._sim_vesc_last_vy) / dt
        self._sim_vesc_last_time_sec = stamp_sec
        self._sim_vesc_last_vx = vx
        self._sim_vesc_last_vy = vy

        measured_vx = (
            self._sim_vesc_speed_scale * vx +
            self._sim_vesc_speed_bias +
            np.random.normal(0.0, self._sim_vesc_speed_noise_std))
        measured_yaw_rate = (
            yaw_rate +
            self._sim_vesc_yaw_rate_bias +
            np.random.normal(0.0, self._sim_vesc_yaw_rate_noise_std))
        measured_accel_x = accel_x + np.random.normal(
            0.0, self._sim_vesc_accel_noise_std)
        measured_accel_y = accel_y + np.random.normal(
            0.0, self._sim_vesc_accel_noise_std)

        abs_steer = abs(self.ego_steer)
        corrected_abs = (
            self._sim_vesc_steering_correction_c2 * abs_steer * abs_steer +
            self._sim_vesc_steering_correction_c1 * abs_steer +
            self._sim_vesc_steering_correction_c0)
        corrected_steer = np.copysign(corrected_abs, self.ego_steer)
        self._sim_vesc_servo_msg.data = (
            self._sim_vesc_servo_gain * corrected_steer +
            self._sim_vesc_servo_offset)

        self._sim_vesc_imu_msg.header.stamp = ts
        self._sim_vesc_imu_msg.angular_velocity.z = measured_yaw_rate
        self._sim_vesc_imu_msg.linear_acceleration.x = measured_accel_x
        self._sim_vesc_imu_msg.linear_acceleration.y = measured_accel_y

        self._sim_vesc_state_msg.header.stamp = ts
        self._sim_vesc_state_msg.state.speed = (
            self._sim_vesc_speed_to_erpm_gain * measured_vx +
            self._sim_vesc_speed_to_erpm_offset)
        self._sim_vesc_state_msg.state.current_motor = (
            self._sim_vesc_last_motor_current)
        self._sim_vesc_state_msg.state.current_input = (
            self._sim_vesc_last_input_current)
        self._sim_vesc_state_msg.state.voltage_input = 16.0
        self._sim_vesc_state_msg.state.fault_code = 0

        self.sim_vesc_servo_pub.publish(self._sim_vesc_servo_msg)
        self.sim_vesc_imu_pub.publish(self._sim_vesc_imu_msg)
        self.sim_vesc_state_pub.publish(self._sim_vesc_state_msg)

    def _publish_transforms(self, ts) -> None:
        """Publish base_link and wheel transforms in a single batched call."""
        transforms = []

        if self._publish_base_tf:
            # Ego base_link transform
            self._update_transform_msg(self._ego_tf_msg, ts, self.ego_pose)
            transforms.append(self._ego_tf_msg)

        # Ego wheel transforms
        ego_wheel_quat = euler.euler2quat(
            0.0, 0.0, self.ego_steer, axes='sxyz')
        self._update_wheel_tf(self._ego_left_wheel_tf, ts, ego_wheel_quat)
        self._update_wheel_tf(self._ego_right_wheel_tf, ts, ego_wheel_quat)
        transforms.append(self._ego_left_wheel_tf)
        transforms.append(self._ego_right_wheel_tf)

        # Opponent transforms
        if self.has_opp:
            if self._publish_base_tf:
                self._update_transform_msg(self._opp_tf_msg, ts, self.opp_pose)
                transforms.append(self._opp_tf_msg)

            opp_wheel_quat = euler.euler2quat(
                0.0, 0.0, self.opp_steer, axes='sxyz')
            self._update_wheel_tf(self._opp_left_wheel_tf, ts, opp_wheel_quat)
            self._update_wheel_tf(self._opp_right_wheel_tf, ts, opp_wheel_quat)
            transforms.append(self._opp_left_wheel_tf)
            transforms.append(self._opp_right_wheel_tf)

        # Batch publish all transforms
        try:
            if transforms:
                self.br.sendTransform(transforms)
        except Exception as e:
            self.get_logger().error(f'Failed to publish transforms: {e}')

    def _update_transform_msg(self, tf_msg: TransformStamped, ts,
                              pose: List[float]) -> None:
        """Update a TransformStamped message in-place."""
        tf_msg.header.stamp = ts
        tf_msg.transform.translation.x = pose[0]
        tf_msg.transform.translation.y = pose[1]
        tf_msg.transform.translation.z = 0.0

        quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
        tf_msg.transform.rotation.w = quat[0]
        tf_msg.transform.rotation.x = quat[1]
        tf_msg.transform.rotation.y = quat[2]
        tf_msg.transform.rotation.z = quat[3]

    def _update_wheel_tf(self, tf_msg: TransformStamped, ts,
                         quat: Tuple[float, float, float, float]) -> None:
        """Update a wheel transform message in-place."""
        tf_msg.header.stamp = ts
        tf_msg.transform.rotation.w = quat[0]
        tf_msg.transform.rotation.x = quat[1]
        tf_msg.transform.rotation.y = quat[2]
        tf_msg.transform.rotation.z = quat[3]

    def _publish_wheel_transforms(self, ts) -> None:
        """Legacy method - wheel transforms are now published in _publish_transforms."""
        pass


def main(args=None) -> None:
    """Run the gym bridge node for the gym bridge node."""
    rclpy.init(args=args)
    gym_bridge = None

    try:
        gym_bridge = GymBridge()
        rclpy.spin(gym_bridge)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if gym_bridge:
            gym_bridge.get_logger().error(f'Error: {e}')
        else:
            print(f'Failed to initialize GymBridge: {e}')
    finally:
        if gym_bridge:
            gym_bridge.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass  # Already shutdown


if __name__ == '__main__':
    main()
