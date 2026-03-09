"""
F1TENTH Real-Car Localization Benchmark with OptiTrack Ground Truth

Launches the localization + OptiTrack ground-truth publisher + benchmark
logger + bag recorder so you can drive the car and later analyse AMCL/EKF
accuracy against OptiTrack.

Prerequisites
─────────────
  • Car bringup already running  (``bringup_launch.py``)
  • VRPN-mocap client running    (``ros2 launch vrpn_mocap …``)
  • SLAM toolbox NOT running     (stop it first — AMCL provides map→odom)

What this launch file starts
────────────────────────────
  1. **OptiTrack ground truth node** — converts VRPN pose from ``world`` to
     ``map`` frame and publishes ``/ego_racecar/ground_truth`` (Odometry).
  2. **AMCL** (gpu_amcl or nav2_amcl) — localization under test.
  3. **Localization benchmark** — logs GT vs EKF vs AMCL to CSV.
  4. **Bag recorder** (optional) — records all relevant topics for replay.

Calibration
───────────
Before first use, calibrate the ``world → map`` offset:
  1. Place car at a known spot.
  2. Read OptiTrack pose:  ``ros2 topic echo /vrpn_mocap/car_pos/pose --once``
  3. Read map-frame pose:  ``ros2 run tf2_ros tf2_echo map ego_racecar/base_link``
  4. Compute offset and set ``offset_x``, ``offset_y``, ``offset_yaw`` below.

Usage
─────
::

    ros2 launch f1tenth_localization optitrack_benchmark.launch.py

    ros2 launch f1tenth_localization optitrack_benchmark.launch.py \\
        amcl_type:=nav2_amcl max_particles:=2000

    ros2 launch f1tenth_localization optitrack_benchmark.launch.py \\
        record_bag:=true
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode

# ── Workspace root (four levels up from installed share directory) ──
_pkg_share = get_package_share_directory('f1tenth_localization')
_workspace_root = os.path.dirname(
    os.path.dirname(os.path.dirname(os.path.dirname(_pkg_share)))
)

# ── Topics to record ──
RECORD_TOPICS = [
    '/scan',
    '/scan_walls',
    '/ego_racecar/odom',
    '/ego_racecar/ground_truth',
    '/amcl_pose',
    '/ekf_pose',
    '/map',
    '/tf',
    '/tf_static',
    '/vrpn_mocap/car_pos/pose',
]


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    pkg_dir = get_package_share_directory('f1tenth_localization')
    amcl_params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')
    gpu_amcl_params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_params.yaml')

    amcl_type = LaunchConfiguration('amcl_type').perform(context)
    min_particles = LaunchConfiguration('min_particles').perform(context)
    max_particles = LaunchConfiguration('max_particles').perform(context)
    max_beams = LaunchConfiguration('max_beams').perform(context)
    record_bag = LaunchConfiguration('record_bag').perform(context).lower() == 'true'

    # Calibration offsets (world → map)
    offset_x = LaunchConfiguration('offset_x').perform(context)
    offset_y = LaunchConfiguration('offset_y').perform(context)
    offset_yaw = LaunchConfiguration('offset_yaw').perform(context)

    nodes = []

    nodes.append(LogInfo(
        msg=f'Starting OptiTrack Benchmark — {amcl_type}, '
            f'particles=[{min_particles},{max_particles}], beams={max_beams}'
    ))

    # ──────────────────────────────────────────────
    # 1) OptiTrack Ground Truth Publisher
    # ──────────────────────────────────────────────
    # Find the script in the workspace source tree
    optitrack_script = os.path.join(
        _workspace_root, 'f1tenth_system', 'f1tenth_stack', 'scripts',
        'optitrack_tf_publisher.py'
    )
    vrpn_topic = LaunchConfiguration('vrpn_topic').perform(context)
    optitrack_node = ExecuteProcess(
        cmd=[
            'python3', optitrack_script,
            '--ros-args',
            '-p', f'vrpn_topic:={vrpn_topic}',
            '-p', 'ground_truth_topic:=/ego_racecar/ground_truth',
            '-p', 'map_frame:=map',
            '-p', 'base_frame:=ego_racecar/base_link',
            '-p', f'offset_x:={offset_x}',
            '-p', f'offset_y:={offset_y}',
            '-p', f'offset_yaw:={offset_yaw}',
        ],
        output='screen',
        name='optitrack_ground_truth',
    )
    nodes.append(optitrack_node)

    # ──────────────────────────────────────────────
    # 2) AMCL Localization (under test)
    # ──────────────────────────────────────────────
    lifecycle_node_names = []

    if amcl_type == 'gpu_amcl':
        amcl_node = Node(
            package='f1tenth_localization',
            executable='gpu_amcl_node.py',
            name='gpu_amcl',
            output='screen',
            parameters=[
                gpu_amcl_params_file,
                {
                    'use_sim_time': False,
                    'num_particles': int(max_particles),
                    'max_beams': int(max_beams),
                },
            ],
        )
    else:
        amcl_node = LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='/',
            output='screen',
            parameters=[
                amcl_params_file,
                {
                    'use_sim_time': False,
                    'min_particles': int(min_particles),
                    'max_particles': int(max_particles),
                    'max_beams': int(max_beams),
                },
            ],
        )
        lifecycle_node_names.append('amcl')

    nodes.append(amcl_node)

    # Lifecycle manager (only needed for nav2_amcl)
    if lifecycle_node_names:
        nodes.append(Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': lifecycle_node_names,
                'bond_timeout': 0.0,
            }],
        ))

    # ──────────────────────────────────────────────
    # 3) Localization Benchmark Logger
    # ──────────────────────────────────────────────
    benchmark_output_dir = os.path.join(
        _workspace_root, 'f1tenth_localization', 'Benchmark', 'optitrack_results'
    )
    os.makedirs(benchmark_output_dir, exist_ok=True)

    benchmark_node = Node(
        package='f1tenth_localization',
        executable='localization_benchmark',
        name='localization_benchmark',
        output='screen',
        parameters=[{
            'output_dir': benchmark_output_dir,
            'ground_truth_topic': '/ego_racecar/ground_truth',
            'ekf_topic': '/ekf_pose',
            'amcl_topic': '/amcl_pose',
            'scan_topic': '/scan_walls',
        }],
    )
    nodes.append(benchmark_node)

    # ──────────────────────────────────────────────
    # 4) Bag Recorder (optional)
    # ──────────────────────────────────────────────
    if record_bag:
        bag_dir = os.path.join(
            _workspace_root, 'bags', 'optitrack_benchmark'
        )
        os.makedirs(bag_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        bag_path = os.path.join(
            bag_dir,
            f'optitrack_{amcl_type}_p{max_particles}_b{max_beams}_{timestamp}'
        )
        nodes.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', bag_path] + RECORD_TOPICS,
            output='screen',
            name='bag_recorder',
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ── VRPN / OptiTrack ──
        DeclareLaunchArgument(
            'vrpn_topic',
            default_value='/vrpn_mocap/car_pos/pose',
            description='VRPN pose topic for the car rigid body',
        ),
        DeclareLaunchArgument(
            'offset_x', default_value='1.291',
            description='X position of map origin in world frame',
        ),
        DeclareLaunchArgument(
            'offset_y', default_value='-1.316',
            description='Y position of map origin in world frame',
        ),
        DeclareLaunchArgument(
            'offset_yaw', default_value='-0.108',
            description='Yaw of map frame relative to world frame (rad)',
        ),
        # ── AMCL ──
        DeclareLaunchArgument(
            'amcl_type', default_value='gpu_amcl',
            description="AMCL implementation: 'gpu_amcl' or 'nav2_amcl'",
        ),
        DeclareLaunchArgument(
            'min_particles', default_value='500',
            description='AMCL minimum particles',
        ),
        DeclareLaunchArgument(
            'max_particles', default_value='2000',
            description='AMCL maximum particles',
        ),
        DeclareLaunchArgument(
            'max_beams', default_value='120',
            description='AMCL max laser beams',
        ),
        # ── Recording ──
        DeclareLaunchArgument(
            'record_bag', default_value='false',
            description='Record a bag with all benchmark topics',
        ),
        OpaqueFunction(function=launch_setup),
    ])
