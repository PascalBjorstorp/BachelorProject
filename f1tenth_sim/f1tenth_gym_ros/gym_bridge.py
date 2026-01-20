# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from typing import List, Optional, Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from ackermann_msgs.msg import AckermannDriveStamped
from tf2_ros import TransformBroadcaster

import gymnasium as gym
import numpy as np
from transforms3d import euler

import pathlib
import f1tenth_gym  # Import to register the environment
from f1tenth_gym.envs.f110_env import F110Env
from f1tenth_gym.envs.track import Track

class GymBridge(Node):
    """ROS2 bridge node for F1Tenth gym environment."""

    def __init__(self) -> None:
        super().__init__('gym_bridge')

        # Declare all parameters
        self._declare_parameters()
        
        # Validate and get parameters
        num_agents = self._validate_num_agents()
        self.vehicle_params = self._get_vehicle_params()
        scale = self.get_parameter('scale').value
        
        # Load map and create environment
        self.env = self._create_environment(num_agents, scale)
        
        # Initialize agent state
        self._init_ego_state()
        if num_agents == 2:
            self._init_opponent_state()
        
        # Reset environment with initial poses
        self._reset_environment(num_agents)
        
        # Setup timers based on mode
        self._setup_timers()
        
        # Setup publishers and subscribers
        self._setup_publishers(num_agents)
        self._setup_subscribers(num_agents)
        
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
        self.declare_parameter('scan_distance_to_base_link', 0.275)
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

    def _validate_num_agents(self) -> int:
        """Validate and return number of agents."""
        num_agents = self.get_parameter('num_agent').value
        if not isinstance(num_agents, int):
            raise ValueError('num_agents should be an int.')
        if num_agents < 1 or num_agents > 2:
            raise ValueError('num_agents should be either 1 or 2.')
        return num_agents

    def _get_vehicle_params(self) -> Dict[str, Any]:
        """Get vehicle parameters based on configuration."""
        vehicle_type = self.get_parameter('vehicle_params').value
        params_map = {
            'f1tenth': F110Env.f1tenth_vehicle_params,
            'fullscale': F110Env.fullscale_vehicle_params,
            'f1fifth': F110Env.f1fifth_vehicle_params,
        }
        if vehicle_type not in params_map:
            raise ValueError(f'vehicle_params should be one of: {list(params_map.keys())}')
        return params_map[vehicle_type]()

    def _create_environment(self, num_agents: int, scale: float) -> gym.Env:
        """Create and configure the F1Tenth gym environment."""
        # Parse map path
        path = self.get_parameter('map_path').value
        name = path.split('/')[-1].split('.')[0]
        path = path + '.yaml'
        self.get_logger().info(f'Loading map: {name} from path: {path}')

        # Load the track
        try:
            loaded_map = Track.from_track_path(pathlib.Path(path), scale)
        except Exception as e:
            self.get_logger().error(f'Failed to load map: {e}')
            raise

        # Create environment
        sim_timestep = self.get_parameter('sim_timestep').value
        try:
            env = gym.make(
                "f1tenth-v0",
                config={
                    "map": loaded_map,
                    "num_agents": num_agents,
                    "timestep": sim_timestep,
                    "integrator": "rk4",
                    "control_input": ["speed", "steering_angle"],
                    "model": "st",
                    "observation_config": {"type": "original"},
                    "params": self.vehicle_params,
                    "reset_config": {"type": "map_random_static"},
                    "scale": scale,
                    "lidar_dist": self.get_parameter("scan_distance_to_base_link").value
                },
                render_mode="rgb_array",
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
        
        # Scan parameters
        scan_fov = self.get_parameter('scan_fov').value
        scan_beams = self.get_parameter('scan_beams').value
        self.angle_min = -scan_fov / 2.0
        self.angle_max = scan_fov / 2.0
        self.angle_inc = scan_fov / scan_beams
        self.scan_range_max = self.get_parameter('scan_range_max').value
        
        self.ego_namespace = self.get_parameter('ego_namespace').value
        self.scan_distance_to_base_link = self.get_parameter('scan_distance_to_base_link').value

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

    def _reset_environment(self, num_agents: int) -> None:
        """Reset environment with initial poses."""
        try:
            if num_agents == 2:
                self.has_opp = True
                poses = np.array([self.ego_pose, self.opp_pose])
                self.obs, _ = self.env.reset(options={"poses": poses})
                self.ego_scan = list(self.obs['scans'][0])
                self.opp_scan = list(self.obs['scans'][1])
            else:
                self.has_opp = False
                poses = np.array([self.ego_pose])
                self.obs, _ = self.env.reset(options={"poses": poses})
                self.ego_scan = list(self.obs['scans'][0])
        except Exception as e:
            self.get_logger().error(f'Failed to reset environment: {e}')
            raise

    def _setup_timers(self) -> None:
        """Setup simulation timers based on async/sync mode."""
        if not self.get_parameter('async_mode').value:
            self.get_logger().info('Running in synchronous mode. Simulation will step only on new /drive messages.')
            self.timer = self.create_timer(1.0, self.timer_callback)
        else:
            self.get_logger().info('Running in asynchronous mode. Simulation will step using a timer callback.')
            sim_timestep = self.get_parameter('sim_timestep').value
            self.drive_timer = self.create_timer(sim_timestep, self.drive_timer_callback)
            self.timer = self.create_timer(0.004, self.timer_callback)

        # Transform broadcaster
        self.br = TransformBroadcaster(self)
        
        # Simulation state
        self.sim_paused = False
        self.done = False

    def _setup_publishers(self, num_agents: int) -> None:
        """Setup all publishers."""
        ego_scan_topic = self.get_parameter('ego_scan_topic').value
        ego_odom_topic = f"{self.ego_namespace}/{self.get_parameter('ego_odom_topic').value}"
        
        self.ego_scan_pub = self.create_publisher(LaserScan, ego_scan_topic, 10)
        self.ego_odom_pub = self.create_publisher(Odometry, ego_odom_topic, 10)
        self.ego_drive_published = False

        if num_agents == 2:
            opp_scan_topic = self.get_parameter('opp_scan_topic').value
            opp_odom_topic = f"{self.opp_namespace}/{self.get_parameter('opp_odom_topic').value}"
            ego_opp_odom_topic = f"{self.ego_namespace}/{self.get_parameter('ego_opp_odom_topic').value}"
            opp_ego_odom_topic = f"{self.opp_namespace}/{self.get_parameter('opp_ego_odom_topic').value}"

            self.opp_scan_pub = self.create_publisher(LaserScan, opp_scan_topic, 10)
            self.opp_odom_pub = self.create_publisher(Odometry, opp_odom_topic, 10)
            self.ego_opp_odom_pub = self.create_publisher(Odometry, ego_opp_odom_topic, 10)
            self.opp_ego_odom_pub = self.create_publisher(Odometry, opp_ego_odom_topic, 10)
            self.opp_drive_published = False

        if self.get_parameter('use_sim_time_bridge').value:
            self.get_logger().info('Using simulation time. Will publish /clock topic.')
            self.clock_pub = self.create_publisher(Clock, '/clock', 10)
            if self.get_parameter('async_mode').value:
                self.drive_timer.timer_period_ns = 0
                self.timer.timer_period_ns = 0

    def _setup_subscribers(self, num_agents: int) -> None:
        """Setup all subscribers."""
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
        self.get_logger().info(f"Simulation {'paused' if self.sim_paused else 'resumed'}")

    def drive_callback(self, drive_msg: AckermannDriveStamped) -> None:
        """Handle ego drive commands."""
        if self.sim_paused:
            return

        self.ego_requested_speed = drive_msg.drive.speed
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

        self.opp_requested_speed = drive_msg.drive.speed
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
                self.obs, _ = self.env.reset(options={"poses": np.array([[rx, ry, rtheta], opp_pose])})
            else:
                self.obs, _ = self.env.reset(options={"poses": np.array([[rx, ry, rtheta]])})
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
            self.obs, _ = self.env.reset(options={"poses": np.array([self.ego_pose, [rx, ry, rtheta]])})
        except Exception as e:
            self.get_logger().error(f'Failed to reset opponent pose: {e}')

    def teleop_callback(self, twist_msg: Twist) -> None:
        """Handle keyboard teleop commands."""
        if self.sim_paused:
            return

        self.ego_requested_speed = twist_msg.linear.x
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
        
        if self.get_parameter('use_sim_time_bridge').value:
            self._publish_clock()

    def timer_callback(self) -> None:
        """Publish sensor data and transforms."""
        if self.sim_paused:
            return

        ts = self._get_timestamp()
        self._publish_scans(ts)
        self._publish_odom(ts)
        self._publish_transforms(ts)
        self._publish_laser_transforms(ts)
        self._publish_wheel_transforms(ts)

    # === Helper Methods ===

    def _extract_pose_2d(self, position, orientation) -> tuple:
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
        """Publish simulation clock."""
        clock_msg = Clock()
        clock_msg.clock.sec = int(self.env.unwrapped.current_time // 1.0)
        clock_msg.clock.nanosec = int((self.env.unwrapped.current_time % 1.0) * 1e9)
        self.clock_pub.publish(clock_msg)

    def _update_sim_state(self) -> None:
        """Update internal state from simulation observations."""
        self.ego_scan = list(self.obs['scans'][0])
        self.ego_pose[0] = float(self.obs['poses_x'][0])
        self.ego_pose[1] = float(self.obs['poses_y'][0])
        self.ego_pose[2] = float(self.obs['poses_theta'][0])
        self.ego_speed[0] = float(self.obs['linear_vels_x'][0])
        self.ego_speed[1] = float(self.obs['linear_vels_y'][0])
        self.ego_speed[2] = float(self.obs['ang_vels_z'][0])

        if self.has_opp:
            self.opp_scan = list(self.obs['scans'][1])
            self.opp_pose[0] = float(self.obs['poses_x'][1])
            self.opp_pose[1] = float(self.obs['poses_y'][1])
            self.opp_pose[2] = float(self.obs['poses_theta'][1])
            self.opp_speed[0] = float(self.obs['linear_vels_x'][1])
            self.opp_speed[1] = float(self.obs['linear_vels_y'][1])
            self.opp_speed[2] = float(self.obs['ang_vels_z'][1])

    def _publish_scans(self, ts) -> None:
        """Publish laser scan messages."""
        # Ego scan
        scan = self._create_scan_msg(ts, self.ego_namespace, self.ego_scan)
        self.ego_scan_pub.publish(scan)

        # Opponent scan
        if self.has_opp:
            opp_scan = self._create_scan_msg(ts, self.opp_namespace, self.opp_scan)
            self.opp_scan_pub.publish(opp_scan)

    def _create_scan_msg(self, ts, namespace: str, scan_data: List[float]) -> LaserScan:
        """Create a LaserScan message."""
        scan = LaserScan()
        scan.header.stamp = ts
        scan.header.frame_id = f'{namespace}/laser'
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_inc
        scan.range_min = 0.0
        scan.range_max = self.scan_range_max
        scan.ranges = [float(x) for x in scan_data]
        return scan

    def _publish_odom(self, ts) -> None:
        """Publish odometry messages."""
        # Ego odom
        ego_odom = self._create_odom_msg(
            ts, 'map', f'{self.ego_namespace}/base_link',
            self.ego_pose, self.ego_speed
        )
        self.ego_odom_pub.publish(ego_odom)

        # Opponent odom
        if self.has_opp:
            opp_odom = self._create_odom_msg(
                ts, 'map', f'{self.opp_namespace}/base_link',
                self.opp_pose, self.opp_speed
            )
            self.opp_odom_pub.publish(opp_odom)
            self.opp_ego_odom_pub.publish(ego_odom)
            self.ego_opp_odom_pub.publish(opp_odom)

    def _create_odom_msg(self, ts, frame_id: str, child_frame_id: str,
                         pose: List[float], speed: List[float]) -> Odometry:
        """Create an Odometry message."""
        odom = Odometry()
        odom.header.stamp = ts
        odom.header.frame_id = frame_id
        odom.child_frame_id = child_frame_id
        odom.pose.pose.position.x = pose[0]
        odom.pose.pose.position.y = pose[1]
        
        quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
        odom.pose.pose.orientation.x = quat[1]
        odom.pose.pose.orientation.y = quat[2]
        odom.pose.pose.orientation.z = quat[3]
        odom.pose.pose.orientation.w = quat[0]
        
        odom.twist.twist.linear.x = speed[0]
        odom.twist.twist.linear.y = speed[1]
        odom.twist.twist.angular.z = speed[2]
        return odom

    def _publish_transforms(self, ts) -> None:
        """Publish base_link transforms."""
        # Ego transform
        ego_ts = self._create_transform(
            ts, 'map', f'{self.ego_namespace}/base_link', self.ego_pose
        )
        self.br.sendTransform(ego_ts)

        # Opponent transform
        if self.has_opp:
            opp_ts = self._create_transform(
                ts, 'map', f'{self.opp_namespace}/base_link', self.opp_pose
            )
            self.br.sendTransform(opp_ts)

    def _create_transform(self, ts, frame_id: str, child_frame_id: str,
                          pose: List[float]) -> TransformStamped:
        """Create a TransformStamped message."""
        t = Transform()
        t.translation.x = pose[0]
        t.translation.y = pose[1]
        t.translation.z = 0.0
        
        quat = euler.euler2quat(0.0, 0.0, pose[2], axes='sxyz')
        t.rotation.x = quat[1]
        t.rotation.y = quat[2]
        t.rotation.z = quat[3]
        t.rotation.w = quat[0]

        ts_msg = TransformStamped()
        ts_msg.transform = t
        ts_msg.header.stamp = ts
        ts_msg.header.frame_id = frame_id
        ts_msg.child_frame_id = child_frame_id
        return ts_msg

    def _publish_wheel_transforms(self, ts) -> None:
        """Publish wheel steering transforms."""
        # Ego wheels
        self._publish_wheel_pair(ts, self.ego_namespace, self.ego_steer)

        # Opponent wheels
        if self.has_opp:
            self._publish_wheel_pair(ts, self.opp_namespace, self.opp_steer)

    def _publish_wheel_pair(self, ts, namespace: str, steer: float) -> None:
        """Publish left and right wheel transforms for a vehicle."""
        quat = euler.euler2quat(0.0, 0.0, steer, axes='sxyz')
        
        wheel_ts = TransformStamped()
        wheel_ts.transform.rotation.x = quat[1]
        wheel_ts.transform.rotation.y = quat[2]
        wheel_ts.transform.rotation.z = quat[3]
        wheel_ts.transform.rotation.w = quat[0]
        wheel_ts.header.stamp = ts

        # Left wheel
        wheel_ts.header.frame_id = f'{namespace}/front_left_hinge'
        wheel_ts.child_frame_id = f'{namespace}/front_left_wheel'
        self.br.sendTransform(wheel_ts)

        # Right wheel
        wheel_ts.header.frame_id = f'{namespace}/front_right_hinge'
        wheel_ts.child_frame_id = f'{namespace}/front_right_wheel'
        self.br.sendTransform(wheel_ts)

    def _publish_laser_transforms(self, ts) -> None:
        """Publish laser frame transforms."""
        # Ego laser
        ego_scan_ts = self._create_laser_transform(ts, self.ego_namespace)
        self.br.sendTransform(ego_scan_ts)

        # Opponent laser
        if self.has_opp:
            opp_scan_ts = self._create_laser_transform(ts, self.opp_namespace)
            self.br.sendTransform(opp_scan_ts)

    def _create_laser_transform(self, ts, namespace: str) -> TransformStamped:
        """Create laser frame transform."""
        scan_ts = TransformStamped()
        scan_ts.transform.translation.x = self.scan_distance_to_base_link
        scan_ts.transform.rotation.w = 1.0
        scan_ts.header.stamp = ts
        scan_ts.header.frame_id = f'{namespace}/base_link'
        scan_ts.child_frame_id = f'{namespace}/laser'
        return scan_ts


def main(args=None) -> None:
    """Main entry point for the gym bridge node."""
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