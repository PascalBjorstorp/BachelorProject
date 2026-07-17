"""
Launch the F1TENTH stack with Nav2 AMCL, EKF fusion, and benchmark monitors.

This wrapper starts the normal system stack without the custom GPU AMCL, then
adds:
  - odom_fused_node for /ego_racecar/odom -> /odom_pose
  - ekf_localization_node for /odom_pose + /amcl_pose -> /ekf_pose
  - nav2_amcl for /scan -> /amcl_pose
  - pipeline_latency_monitor + performance_monitor_cpp

Nav2 AMCL is pinned to one CPU core and starts with global localization.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stack_pkg_dir = get_package_share_directory('f1tenth_stack')
    localization_pkg_dir = get_package_share_directory('f1tenth_localization')
    planner_pkg_dir = get_package_share_directory('f1tenth_planning')

    system_no_localization_launch = os.path.join(
        stack_pkg_dir, 'launch', 'System_no_localization.launch.py')
    joy_teleop_config = os.path.join(stack_pkg_dir, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(stack_pkg_dir, 'config', 'vesc.yaml')
    mux_config = os.path.join(stack_pkg_dir, 'config', 'mux.yaml')
    default_map = os.path.join(planner_pkg_dir, 'maps', 'my_track_map.yaml')
    localization_params_file = os.path.join(
        localization_pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')
    nav2_amcl_params_file = os.path.join(
        localization_pkg_dir, 'config', 'nav2_amcl_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config,
            description='Path to VESC configuration file'),

        DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config,
            description='Path to ackermann_mux configuration file'),

        DeclareLaunchArgument(
            'joy_config',
            default_value=joy_teleop_config,
            description='Path to joystick configuration file'),

        DeclareLaunchArgument(
            'use_teleop',
            default_value='false',
            description='Launch joystick teleop'),

        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Launch LiDAR driver'),

        DeclareLaunchArgument(
            'mapping_mode',
            default_value='false',
            description='Mapping mode: full-resolution LiDAR, no scan splitter or lateral planner'),

        DeclareLaunchArgument(
            'lidar_cluster',
            default_value='4',
            description='Racing /scan clustering; 4 keeps AMCL on 270 beams'),

        DeclareLaunchArgument(
            'lateral_planner_avoidance_enabled',
            default_value='false',
            description='Enable lateral planner obstacle avoidance'),

        DeclareLaunchArgument(
            'lateral_planner_delay_sec',
            default_value='2.0',
            description='Delay before starting lateral planner after bringup starts'),

        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Path to map YAML file for map_server'),

        DeclareLaunchArgument(
            'bringup_delay_sec',
            default_value='2.0',
            description='Delay before base bringup starts'),

        DeclareLaunchArgument(
            'use_dynamic_bicycle_model',
            default_value='true',
            description='Enable dynamic bicycle model inside vesc_to_odom node'),

        DeclareLaunchArgument(
            'oldOdom',
            default_value='false',
            description='Use legacy analytical vesc_to_odom implementation'),

        DeclareLaunchArgument(
            'localization_params_file',
            default_value=localization_params_file,
            description='Path to odom/EKF parameter file'),

        DeclareLaunchArgument(
            'nav2_amcl_params_file',
            default_value=nav2_amcl_params_file,
            description='Path to Nav2 AMCL parameter file'),

        DeclareLaunchArgument(
            'nav2_amcl_cpu_core',
            default_value='3',
            description='CPU core for Nav2 AMCL taskset pinning'),

        DeclareLaunchArgument(
            'nav2_min_particles',
            default_value='600',
            description='Nav2 AMCL minimum particle count'),

        DeclareLaunchArgument(
            'nav2_max_particles',
            default_value='2000',
            description='Nav2 AMCL maximum particle count'),

        DeclareLaunchArgument(
            'nav2_max_beams',
            default_value='270',
            description='Number of beams used by Nav2 AMCL from the reduced racing scan'),

        DeclareLaunchArgument(
            'nav2_update_min_d',
            default_value='0.05',
            description='Minimum translation before Nav2 AMCL filter update'),

        DeclareLaunchArgument(
            'nav2_update_min_a',
            default_value='0.05',
            description='Minimum rotation before Nav2 AMCL filter update'),

        DeclareLaunchArgument(
            'start_nav2_delay_sec',
            default_value='3.0',
            description='Delay before starting Nav2 AMCL and EKF nodes'),

        DeclareLaunchArgument(
            'nav2_global_init_delay_sec',
            default_value='2.0',
            description='Delay after Nav2 AMCL start before global localization service call'),

        DeclareLaunchArgument(
            'use_pipeline_monitor',
            default_value='true',
            description='Launch pipeline_latency_monitor and performance_monitor_cpp'),

        DeclareLaunchArgument(
            'monitor_print_every',
            default_value='40',
            description='Print latency summary every N completed scan cycles'),

        DeclareLaunchArgument(
            'monitor_csv_output_dir',
            default_value='f1tenth_localization/Benchmark/Matlab/csv',
            description='Directory for monitor CSV output'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_no_localization_launch),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vesc_config': LaunchConfiguration('vesc_config'),
                'mux_config': LaunchConfiguration('mux_config'),
                'joy_config': LaunchConfiguration('joy_config'),
                'use_teleop': LaunchConfiguration('use_teleop'),
                'use_lidar': LaunchConfiguration('use_lidar'),
                'mapping_mode': LaunchConfiguration('mapping_mode'),
                'lidar_cluster': LaunchConfiguration('lidar_cluster'),
                'lateral_planner_avoidance_enabled': LaunchConfiguration(
                    'lateral_planner_avoidance_enabled'),
                'lateral_planner_delay_sec': LaunchConfiguration('lateral_planner_delay_sec'),
                'map_file': LaunchConfiguration('map_file'),
                'bringup_delay_sec': LaunchConfiguration('bringup_delay_sec'),
                'use_dynamic_bicycle_model': LaunchConfiguration('use_dynamic_bicycle_model'),
                'oldOdom': LaunchConfiguration('oldOdom'),
            }.items(),
        ),

        TimerAction(
            period=LaunchConfiguration('start_nav2_delay_sec'),
            actions=[
                Node(
                    package='f1tenth_localization',
                    executable='odom_fused_node',
                    name='odom_fused',
                    output='screen',
                    parameters=[
                        LaunchConfiguration('localization_params_file'),
                        {'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool)},
                    ],
                ),

                LifecycleNode(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    namespace='/',
                    output='screen',
                    prefix=PythonExpression([
                        "'taskset -c ",
                        LaunchConfiguration('nav2_amcl_cpu_core'),
                        "' if '",
                        LaunchConfiguration('nav2_amcl_cpu_core'),
                        "' else ''",
                    ]),
                    parameters=[
                        LaunchConfiguration('nav2_amcl_params_file'),
                        {
                            'use_sim_time': ParameterValue(
                                LaunchConfiguration('use_sim_time'), value_type=bool),
                            'min_particles': ParameterValue(
                                LaunchConfiguration('nav2_min_particles'), value_type=int),
                            'max_particles': ParameterValue(
                                LaunchConfiguration('nav2_max_particles'), value_type=int),
                            'max_beams': ParameterValue(
                                LaunchConfiguration('nav2_max_beams'), value_type=int),
                            'update_min_d': ParameterValue(
                                LaunchConfiguration('nav2_update_min_d'), value_type=float),
                            'update_min_a': ParameterValue(
                                LaunchConfiguration('nav2_update_min_a'), value_type=float),
                            'tf_broadcast': False,
                            'always_reset_initial_pose': False,
                            'set_initial_pose': False,
                        },
                    ],
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_nav2_amcl',
                    output='screen',
                    parameters=[{
                        'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool),
                        'autostart': True,
                        'node_names': ['amcl'],
                        'bond_timeout': 0.0,
                    }],
                ),

                Node(
                    package='f1tenth_localization',
                    executable='ekf_localization_node',
                    name='ekf_localization',
                    output='screen',
                    parameters=[
                        LaunchConfiguration('localization_params_file'),
                        {'use_sim_time': ParameterValue(
                            LaunchConfiguration('use_sim_time'), value_type=bool)},
                    ],
                ),

                TimerAction(
                    period=LaunchConfiguration('nav2_global_init_delay_sec'),
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                'ros2',
                                'service',
                                'call',
                                '/reinitialize_global_localization',
                                'std_srvs/srv/Empty',
                                '{}',
                            ],
                            output='screen',
                        ),
                    ],
                ),
            ],
        ),

        Node(
            package='f1tenth_localization',
            executable='pipeline_latency_monitor',
            name='pipeline_latency_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_pipeline_monitor')),
            parameters=[{
                'scan_topic': '/scan',
                'amcl_topic': '/amcl_pose',
                'amcl_particle_count_topic': '/amcl_particle_count',
                'ekf_topic': '/ekf_pose',
                'drive_topic': '/drive',
                'ackermann_topic': '/ackermann_cmd',
                'stage_match_max_ms': 20.0,
                'strict_mode': True,
                'print_every': ParameterValue(
                    LaunchConfiguration('monitor_print_every'), value_type=int),
                'log_to_csv': True,
                'csv_output_dir': LaunchConfiguration('monitor_csv_output_dir'),
            }],
        ),

        Node(
            package='f1tenth_localization',
            executable='performance_monitor_cpp',
            name='performance_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_pipeline_monitor')),
        ),
    ])
