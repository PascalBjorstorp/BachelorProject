"""
Record Bag Launch File

Launches the headless F1TENTH simulation and records a bag of the relevant
topics for later AMCL benchmarking. The car drives autonomously along the
trajectory while all sensor and odometry data is captured.

Recorded topics:
  /scan              - LiDAR scan (sensor_msgs/LaserScan)
  /ego_racecar/odom  - Ground truth odometry (nav_msgs/Odometry)
  /map               - Occupancy grid (nav_msgs/OccupancyGrid)
  /tf                - Transform tree
  /tf_static         - Static transforms

Output bags are written to Benchmark/bags/lapBags/ by default.

Usage:
  ros2 launch f1tenth_localization record_bag.launch.py

  # Custom output directory
  ros2 launch f1tenth_localization record_bag.launch.py \
    output_dir:=<workspace>/f1tenth_localization/Benchmark/bags/lapBags

  # Custom duration (seconds, 0 = unlimited)
  ros2 launch f1tenth_localization record_bag.launch.py duration:=120
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


# Workspace root: 4 levels up from installed share directory
_pkg_share = get_package_share_directory('f1tenth_localization')
_workspace_root = os.path.abspath(os.path.join(_pkg_share, '..', '..', '..', '..'))

TOPICS_TO_RECORD = [
    '/scan',
    '/ego_racecar/odom',
    '/map',
    '/tf',
    '/tf_static',
]


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    output_dir = LaunchConfiguration('output_dir').perform(context)
    duration = LaunchConfiguration('duration').perform(context)
    output_dir = os.path.expanduser(output_dir)

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Generate bag name with timestamp
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(output_dir, f'lap_recording_{timestamp}')

    # ==================== Headless Simulation ====================
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_pkg_share, 'launch', 'sim_headless.launch.py')
        )
    )

    # ==================== Bag Recorder ====================
    # Build the ros2 bag record command
    record_cmd = [
        'ros2', 'bag', 'record',
        '--output', bag_path,
        '--use-sim-time',
    ]

    # Add duration limit if specified
    if int(duration) > 0:
        record_cmd.extend(['--max-duration', duration])

    # Add all topics
    record_cmd.extend(TOPICS_TO_RECORD)

    # Delay bag recording slightly to let sim start up
    bag_recorder = TimerAction(
        period=3.0,
        actions=[
            LogInfo(msg=f'Starting bag recording to: {bag_path}'),
            ExecuteProcess(
                cmd=record_cmd,
                output='screen',
                name='bag_recorder',
            ),
        ],
    )

    return [
        LogInfo(msg=f'Output directory: {output_dir}'),
        LogInfo(msg=f'Duration: {duration}s (0 = unlimited)'),
        sim_launch,
        bag_recorder,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_dir',
            default_value=os.path.join(
                _workspace_root, 'f1tenth_localization', 'Benchmark', 'bags', 'lapBags'
            ),
            description='Directory to save recorded bags',
        ),
        DeclareLaunchArgument(
            'duration',
            default_value='0',
            description='Recording duration in seconds (0 = unlimited, Ctrl+C to stop)',
        ),
        OpaqueFunction(function=launch_setup),
    ])
