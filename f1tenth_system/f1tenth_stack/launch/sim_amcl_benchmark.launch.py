"""
Headless simulation benchmark for GPU AMCL versus Nav2 AMCL.

Starts one localization stack, MPC, scan splitter, lateral planner, ackermann
mux, pipeline monitor, system monitor, and ground-truth-at-EKF logger.
The logger exits after max_laps or collision; launch then shuts down.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def _bool_config(name):
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _float_config(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _int_config(name):
    return ParameterValue(LaunchConfiguration(name), value_type=int)


def _launch_setup(context, *args, **kwargs):
    localizer = LaunchConfiguration('localizer').perform(context).lower()
    if localizer not in ('gpu', 'nav2'):
        raise RuntimeError("localizer must be 'gpu' or 'nav2'")

    output_dir = LaunchConfiguration('output_dir').perform(context)
    pipeline_dir = os.path.join(output_dir, 'pipeline')
    system_dir = os.path.join(output_dir, 'system')
    mpc_dir = os.path.join(output_dir, 'mpc')

    loc_pkg = get_package_share_directory('f1tenth_localization')
    stack_pkg = get_package_share_directory('f1tenth_stack')
    sim_pkg = get_package_share_directory('f1tenth_gym_ros')
    lidar_pkg = get_package_share_directory('f1tenth_lidar')

    gpu_params = os.path.join(loc_pkg, 'config', 'gpu_amcl_cpp_params.yaml')
    nav2_params = os.path.join(loc_pkg, 'config', 'nav2_amcl_params.yaml')
    sim_config = os.path.join(sim_pkg, 'config', 'sim.yaml')
    mux_config = os.path.join(stack_pkg, 'config', 'mux.yaml')

    actions = [
        SetEnvironmentVariable('MPC_ODOM_TOPIC', '/ego_racecar/odom'),
        SetEnvironmentVariable('MPC_DRIVE_TOPIC', '/drive'),
        SetEnvironmentVariable('MPC_EKF_TOPIC', '/ekf_pose'),
        SetEnvironmentVariable('MPC_LOCAL_RACELINE_TOPIC', '/local_raceline'),
        SetEnvironmentVariable('MPC_VERBOSE', LaunchConfiguration('mpc_verbose')),
        SetEnvironmentVariable('MPC_SOLVER_LOG', os.path.join(mpc_dir, 'solver.csv')),
        SetEnvironmentVariable(
            'MPC_LOCAL_RACELINE_LOG',
            os.path.join(mpc_dir, 'local_raceline.csv')),

        LifecycleNode(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            namespace='/',
            output='screen',
            parameters=[{
                'use_sim_time': _bool_config('use_sim_time'),
                'yaml_filename': LaunchConfiguration('map_file'),
            }],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': _bool_config('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server'],
                'bond_timeout': 0.0,
            }],
        ),

        Node(
            package='f1tenth_gym_ros',
            executable='gym_bridge',
            name='bridge',
            output='screen',
            parameters=[
                sim_config,
                {
                    'use_sim_time': _bool_config('use_sim_time'),
                    'use_sim_time_bridge': _bool_config('use_sim_time'),
                    'headless': _bool_config('headless'),
                    'tf_frame_id': 'ego_racecar/odom',
                    'odom_frame_id': 'ego_racecar/odom',
                    'map_path': LaunchConfiguration('map_file'),
                    'ego_drive_topic': LaunchConfiguration('sim_drive_topic'),
                    'sx': _float_config('initial_pose_x'),
                    'sy': _float_config('initial_pose_y'),
                    'stheta': _float_config('initial_pose_yaw'),
                },
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(lidar_pkg, 'launch', 'scan_splitter.launch.py'))),

        Node(
            package='f1tenth_lateral_planner',
            executable='lateral_planner_node',
            name='lateral_planner_node',
            output='screen',
            parameters=[
                {
                    'trajectory_file': LaunchConfiguration('trajectory_file'),
                    'avoidance_enabled': _bool_config(
                        'lateral_planner_avoidance_enabled'),
                },
            ],
        ),

        Node(
            package='ackermann_mux',
            executable='ackermann_mux',
            name='ackermann_mux',
            output='screen',
            parameters=[mux_config],
        ),

        Node(
            package='f1tenth_localization',
            executable='odom_fused_node',
            name='odom_fused',
            output='screen',
            parameters=[gpu_params, {'use_sim_time': _bool_config('use_sim_time')}],
        ),

        Node(
            package='f1tenth_localization',
            executable='ekf_localization_node',
            name='ekf_localization',
            output='screen',
            parameters=[gpu_params, {'use_sim_time': _bool_config('use_sim_time')}],
        ),

        Node(
            package='mpc_riccati',
            executable='mpc_hardware_node',
            name='mpc_hardware_node',
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='f1tenth_localization',
            executable='pipeline_latency_monitor',
            name='pipeline_latency_monitor',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'amcl_topic': '/amcl_pose',
                'amcl_particle_count_topic': '/amcl_particle_count',
                'ekf_topic': '/ekf_pose',
                'drive_topic': '/drive',
                'ackermann_topic': '/ackermann_cmd',
                'stage_match_max_ms': _float_config('monitor_stage_match_max_ms'),
                'strict_mode': _bool_config('monitor_strict_mode'),
                'print_every': _int_config('monitor_print_every'),
                'log_to_csv': True,
                'csv_output_dir': pipeline_dir,
            }],
        ),

        Node(
            package='f1tenth_localization',
            executable='performance_monitor_cpp',
            name='performance_monitor',
            output='screen',
            parameters=[{'output_dir': system_dir}],
        ),
    ]

    if localizer == 'gpu':
        actions.append(
            Node(
                package='f1tenth_localization',
                executable='gpu_amcl_cpp_node',
                name='gpu_amcl_cpp',
                output='screen',
                parameters=[
                    gpu_params,
                    {
                        'use_sim_time': _bool_config('use_sim_time'),
                        'num_particles': _int_config('amcl_num_particles'),
                        'min_particles': _int_config('amcl_min_particles'),
                        'max_particles': _int_config('amcl_max_particles'),
                        'max_beams': _int_config('amcl_max_beams'),
                        'use_kld_sampling': _bool_config('amcl_use_kld'),
                        'global_initialization': _bool_config(
                            'amcl_global_initialization'),
                        'initial_pose_x': _float_config('initial_pose_x'),
                        'initial_pose_y': _float_config('initial_pose_y'),
                        'initial_pose_a': _float_config('initial_pose_yaw'),
                    },
                ],
            )
        )
    else:
        actions.extend([
            LifecycleNode(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                namespace='/',
                output='screen',
                parameters=[
                    nav2_params,
                    {
                        'use_sim_time': _bool_config('use_sim_time'),
                        'min_particles': _int_config('amcl_min_particles'),
                        'max_particles': _int_config('amcl_max_particles'),
                        'max_beams': _int_config('amcl_max_beams'),
                        'update_min_d': _float_config('amcl_update_min_d'),
                        'update_min_a': _float_config('amcl_update_min_a'),
                        'tf_broadcast': False,
                        'always_reset_initial_pose': True,
                        'set_initial_pose': True,
                        'initial_pose.x': _float_config('initial_pose_x'),
                        'initial_pose.y': _float_config('initial_pose_y'),
                        'initial_pose.z': 0.0,
                        'initial_pose.yaw': _float_config('initial_pose_yaw'),
                    },
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_nav2_amcl',
                output='screen',
                parameters=[{
                    'use_sim_time': _bool_config('use_sim_time'),
                    'autostart': True,
                    'node_names': ['amcl'],
                    'bond_timeout': 0.0,
                }],
            ),
        ])

    logger_node = Node(
        package='f1tenth_localization',
        executable='sim_benchmark_logger.py',
        name='sim_benchmark_logger',
        output='screen',
        arguments=[
            '--localizer', localizer,
            '--output-dir', output_dir,
            '--trajectory-file', LaunchConfiguration('trajectory_file'),
            '--max-laps', LaunchConfiguration('max_laps'),
            '--max-duration-sec', LaunchConfiguration('max_duration_sec'),
        ],
    )
    actions.append(logger_node)
    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=logger_node,
                on_exit=[Shutdown(reason='simulation benchmark complete')],
            )
        )
    )

    return actions


def generate_launch_description():
    planner_pkg = get_package_share_directory('f1tenth_planning')
    default_map = os.path.join(planner_pkg, 'maps', 'my_track_map.yaml')
    default_trajectory = os.path.join(
        planner_pkg, 'trajectories', 'my_track_raceline.csv')

    return LaunchDescription([
        DeclareLaunchArgument('localizer', default_value='gpu',
                              description='gpu or nav2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('trajectory_file', default_value=default_trajectory),
        DeclareLaunchArgument(
            'output_dir',
            default_value='f1tenth_localization/Benchmark/Matlab/sim_benchmark'),
        DeclareLaunchArgument('max_laps', default_value='10'),
        DeclareLaunchArgument('max_duration_sec', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_x', default_value='-0.79'),
        DeclareLaunchArgument('initial_pose_y', default_value='-4.88'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.641322'),
        DeclareLaunchArgument('sim_drive_topic', default_value='ackermann_cmd',
                              description='Topic consumed by simulator bridge'),
        DeclareLaunchArgument('lateral_planner_avoidance_enabled',
                              default_value='false'),
        DeclareLaunchArgument('mpc_verbose', default_value='0'),
        DeclareLaunchArgument('amcl_num_particles', default_value='1000'),
        DeclareLaunchArgument('amcl_min_particles', default_value='1000'),
        DeclareLaunchArgument('amcl_max_particles', default_value='1000'),
        DeclareLaunchArgument('amcl_max_beams', default_value='270'),
        DeclareLaunchArgument('amcl_use_kld', default_value='false'),
        DeclareLaunchArgument('amcl_global_initialization', default_value='false'),
        DeclareLaunchArgument('amcl_update_min_d', default_value='0.0'),
        DeclareLaunchArgument('amcl_update_min_a', default_value='0.0'),
        DeclareLaunchArgument('monitor_print_every', default_value='40'),
        DeclareLaunchArgument('monitor_stage_match_max_ms', default_value='20.0'),
        DeclareLaunchArgument('monitor_strict_mode', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
