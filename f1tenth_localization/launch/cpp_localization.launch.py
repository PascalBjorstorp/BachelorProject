"""
Launch file for the C++ GPU AMCL localization stack.

Launches four nodes:
  1. map_server        — serves static map from YAML
  2. gpu_amcl_cpp      — GPU particle-filter AMCL (40 Hz, subscribes to /scan_walls)
  3. odom_fused        — IMU + wheel odom fusion (200 Hz)
  4. ekf_localization  — EKF sensor fusion + TF broadcast

The scan splitter (/scan → /scan_walls) is expected to be running
from the bringup_launch.py driver stack.

All parameters are loaded from config/gpu_amcl_cpp_params.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    params_file = os.path.join(pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')

    # Default map path (workspace-relative)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_dir))))
    default_map = os.path.join(workspace_root, 'f1tenth_sim', 'maps', 'my_track_map.yaml')

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
                'bond_timeout': 5.0,
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
