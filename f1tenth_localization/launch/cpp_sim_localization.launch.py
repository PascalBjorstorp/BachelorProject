"""
Launch file for C++ GPU AMCL localization in SIMULATION.

This is the simulation-only variant of cpp_localization.launch.py.
It additionally launches:
  - scan_splitter  (f1tenth_lidar)   — splits /scan → /scan_walls + /scan_obstacles
  - lateral_planner (f1tenth_lateral_planner) — opponent avoidance planner

For real hardware, use cpp_localization.launch.py instead (the bringup
stack handles the scan splitter and lateral planner).

Nodes launched:
  1. scan_splitter        — /scan → /scan_walls, /scan_obstacles
  2. lateral_planner_node — opponent-avoidance local raceline
  3. map_server           — serves static map from YAML
  4. lifecycle_manager    — auto-activates map_server
  5. odom_fused           — IMU + wheel odom fusion (200 Hz)
  6. gpu_amcl_cpp         — GPU particle-filter AMCL (40 Hz)
  7. ekf_localization     — EKF sensor fusion + TF broadcast

Usage:
  ros2 launch f1tenth_localization cpp_sim_localization.launch.py
  ros2 launch f1tenth_localization cpp_sim_localization.launch.py use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    loc_pkg = get_package_share_directory('f1tenth_localization')
    lidar_pkg = get_package_share_directory('f1tenth_lidar')
    lateral_pkg = get_package_share_directory('f1tenth_lateral_planner')
    params_file = os.path.join(loc_pkg, 'config', 'gpu_amcl_cpp_params.yaml')
    lateral_config = os.path.join(lateral_pkg, 'config', 'lateral_planner.yaml')

    # Default map path (workspace-relative)
    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(loc_pkg)))
    )
    default_map = os.path.join(workspace_root, 'f1tenth_sim', 'maps', 'my_track_map.yaml')

    # Default trajectory for lateral planner
    try:
        planning_pkg = get_package_share_directory('f1tenth_planning')
        default_trajectory = os.path.join(
            planning_pkg, 'trajectories', 'my_track_raceline.csv'
        )
    except Exception:
        default_trajectory = ''

    return LaunchDescription([
        # ── Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the unified YAML parameter file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to the map YAML file for map_server'),

        DeclareLaunchArgument(
            'trajectory_file',
            default_value=default_trajectory,
            description='Path to global raceline CSV for lateral planner'),

        # ── Scan splitter (/scan → /scan_walls + /scan_obstacles) ─
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'scan_splitter.launch.py')
            ),
        ),

        # ── Lateral planner ───────────────────────────────────────
        Node(
            package='f1tenth_lateral_planner',
            executable='lateral_planner_node',
            name='lateral_planner_node',
            output='screen',
            parameters=[
                lateral_config,
                {'trajectory_file': LaunchConfiguration('trajectory_file')},
            ],
        ),

        # ── Map server ────────────────────────────────────────────
        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='/',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'yaml_filename': LaunchConfiguration('map_file'),
            }],
        ),

        # ── Lifecycle manager (auto-activates map_server) ─────────
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,
            }],
        ),

        # ── Odom fusion node (must start first) ───────────────────
        Node(
            package='f1tenth_localization',
            executable='odom_fused_node',
            name='odom_fused',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ── AMCL node ─────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='gpu_amcl_cpp_node',
            name='gpu_amcl_cpp',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # ── EKF node ──────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='ekf_localization_node',
            name='ekf_localization',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
