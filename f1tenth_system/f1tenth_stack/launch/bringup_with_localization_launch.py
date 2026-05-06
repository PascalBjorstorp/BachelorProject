"""
Combined launch file for localization + driver stack.

Startup order:
1) Launch localization stack first.
2) Launch bringup stack after a short delay.

Example:
  ros2 launch f1tenth_stack bringup_with_localization_launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stack_pkg_share = get_package_share_directory('f1tenth_stack')
    localization_pkg_share = get_package_share_directory('f1tenth_localization')

    bringup_launch_path = os.path.join(stack_pkg_share, 'launch', 'bringup_launch.py')
    localization_launch_path = os.path.join(
        localization_pkg_share, 'launch', 'cpp_localization.launch.py'
    )
    default_localization_params = os.path.join(
        localization_pkg_share, 'config', 'gpu_amcl_cpp_params.yaml'
    )

    workspace_root = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.dirname(stack_pkg_share)))
    )
    default_map = os.path.join(workspace_root, 'f1tenth_sim', 'maps', 'my_track_map.yaml')

    ld = LaunchDescription([
        # Shared time argument for both included launches.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time',
        ),

        # Delay to guarantee localization actions are started first.
        DeclareLaunchArgument(
            'bringup_delay_sec',
            default_value='2.0',
            description='Delay before bringup starts (seconds)',
        ),

        # Localization launch arguments.
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=default_localization_params,
            description='Path to f1tenth_localization parameter YAML',
        ),
        DeclareLaunchArgument(
            'amcl_global_initialization',
            default_value='true',
            description='Seed GPU AMCL particles globally along raceline with heading cone',
        ),

        # Bringup launch arguments (kept aligned with bringup_launch.py).
        DeclareLaunchArgument(
            'joy_config',
            default_value=os.path.join(stack_pkg_share, 'config', 'joy_teleop.yaml'),
            description='Path to joystick configuration file',
        ),
        DeclareLaunchArgument(
            'vesc_config',
            default_value=os.path.join(stack_pkg_share, 'config', 'vesc.yaml'),
            description='Path to VESC configuration file',
        ),
        DeclareLaunchArgument(
            'sensors_config',
            default_value=os.path.join(stack_pkg_share, 'config', 'sensors.yaml'),
            description='Path to sensors configuration file',
        ),
        DeclareLaunchArgument(
            'mux_config',
            default_value=os.path.join(stack_pkg_share, 'config', 'mux.yaml'),
            description='Path to ackermann_mux configuration file',
        ),
        DeclareLaunchArgument(
            'use_teleop',
            default_value='true',
            description='Launch joystick teleop and mux',
        ),
        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Launch LiDAR driver',
        ),
        DeclareLaunchArgument(
            'mapping_mode',
            default_value='false',
            description='Mapping mode: no scan splitter or lateral planner',
        ),
        DeclareLaunchArgument(
            'trajectory_file',
            default_value='__from_yaml__',
            description='Optional override for lateral planner trajectory_file',
        ),
        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to the map YAML file for map_server',
        ),
    ])

    localization_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={
            'params_file': LaunchConfiguration('localization_params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'amcl_global_initialization': LaunchConfiguration('amcl_global_initialization'),
        }.items(),
    )

    bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config'),
            'vesc_config': LaunchConfiguration('vesc_config'),
            'sensors_config': LaunchConfiguration('sensors_config'),
            'mux_config': LaunchConfiguration('mux_config'),
            'use_teleop': LaunchConfiguration('use_teleop'),
            'use_lidar': LaunchConfiguration('use_lidar'),
            'mapping_mode': LaunchConfiguration('mapping_mode'),
            'trajectory_file': LaunchConfiguration('trajectory_file'),
            'map_file': LaunchConfiguration('map_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    ld.add_action(localization_include)
    ld.add_action(
        TimerAction(
            period=LaunchConfiguration('bringup_delay_sec'),
            actions=[bringup_include],
        )
    )

    return ld
