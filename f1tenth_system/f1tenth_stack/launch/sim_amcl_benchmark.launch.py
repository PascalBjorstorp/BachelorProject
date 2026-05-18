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

    headless_value = LaunchConfiguration('headless').perform(context)
    headless = str(headless_value).strip().lower() in ('true', '1', 'yes')
    realistic_value = LaunchConfiguration('realistic_plant').perform(context)
    realistic_plant = str(realistic_value).strip().lower() in ('true', '1', 'yes')

    output_dir = LaunchConfiguration('output_dir').perform(context)
    pipeline_dir = os.path.join(output_dir, 'pipeline')
    system_dir = os.path.join(output_dir, 'system')
    mpc_dir = os.path.join(output_dir, 'mpc')

    loc_pkg = get_package_share_directory('f1tenth_localization')
    stack_pkg = get_package_share_directory('f1tenth_stack')
    sim_pkg = get_package_share_directory('f1tenth_gym_ros')
    lidar_pkg = get_package_share_directory('f1tenth_lidar')
    mpc_pkg = get_package_share_directory('mpc_riccati')

    gpu_params = os.path.join(loc_pkg, 'config', 'gpu_amcl_cpp_params.yaml')
    nav2_params = os.path.join(loc_pkg, 'config', 'nav2_amcl_params.yaml')
    rviz_config = os.path.join(loc_pkg, 'config', 'amcl_bag_visualization.rviz')
    sim_config = os.path.join(sim_pkg, 'config', 'sim.yaml')
    mux_config = os.path.join(stack_pkg, 'config', 'mux.yaml')
    realistic_vehicle_params = {'realistic_plant_enabled': realistic_plant}
    if realistic_plant:
        realistic_vehicle_params.update({
            # Defaults copied from MPC/test/test_sim_drive.c via tune_realistic_v2.py.
            'vehicle_mu': 0.6652002785524997,
            'vehicle_mu_front': 0.775687,
            'vehicle_mu_rear': 0.6565520426481404,
            'vehicle_m': 3.57912,
            'vehicle_I': 0.035,
            'vehicle_C_Sf': 4.78281642069513,
            'vehicle_C_Sr': 2.73123678240426,
            'vehicle_C_Sf_high_slip': 2.4199490875105907,
            'vehicle_C_Sr_high_slip': 2.73123678240426,
            'vehicle_lf': 0.166,
            'vehicle_lr': 0.16,
            'vehicle_h': 0.0703,
            'vehicle_s_max': 0.39,
            'vehicle_sv_max': 2.8492,
            'vehicle_steer_gain': 1.0085301459687404,
            'vehicle_steer_gain_high_slip': 0.6541720766809247,
            'vehicle_a_max': 7.31,
            'vehicle_v_switch': 7.319,
            'vehicle_v_min': 0.5,
            'vehicle_v_max': 21.6,
            'vehicle_roll_resistance_n': 2.79,
            'vehicle_drag_c0': 0.0,
            'vehicle_drag_c1': 0.05,
            'vehicle_drag_c2': 0.04,
            'vehicle_accel_tau_pos': 0.05,
            'vehicle_accel_tau_neg': 0.12,
            'vehicle_accel_gain_pos': 0.575,
            'vehicle_accel_gain_neg': 1.0,
            'vehicle_pacejka_c': 1.6041121492252324,
            'vehicle_pacejka_c_front': 1.8031639754063644,
            'vehicle_pacejka_c_rear': 1.7681655069132207,
            'vehicle_slip_blend_start_front': 0.1643527788471148,
            'vehicle_slip_blend_end_front': 0.5319307735091576,
            'vehicle_slip_blend_start_rear': 0.2502122916247753,
            'vehicle_slip_blend_end_rear': 0.47793678552502167,
            'vehicle_combined_slip_gain': 0.10359393575265835,
            'vehicle_front_peak_drop': 0.11804981810838257,
            'vehicle_front_peak_drop_start': 0.13813810031946996,
            'vehicle_front_peak_drop_end': 0.48938120479012814,
            'vehicle_front_peak_drop_pow': 1.03,
            'vehicle_front_combined_gain': 0.13366870620631957,
            'vehicle_front_peak_floor': 0.2708096984131235,
            'vehicle_clamp_velocity_state': True,
            'realistic_actuation_enabled': True,
        })

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
                    'drive_uses_acceleration_field': _bool_config(
                        'sim_drive_uses_acceleration_field'),
                    'sx': _float_config('initial_pose_x'),
                    'sy': _float_config('initial_pose_y'),
                    'stheta': _float_config('initial_pose_yaw'),
                },
                realistic_vehicle_params,
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
            parameters=[
                gpu_params,
                {
                    'use_sim_time': _bool_config('use_sim_time'),
                    'transform_tolerance': _float_config('ekf_transform_tolerance'),
                    'process_noise_scale': _float_config('ekf_process_noise_scale'),
                },
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mpc_pkg, 'launch', 'mpc_hardware.launch.py')),
            launch_arguments={
                'use_local_raceline': 'true',
                'local_raceline_topic': '/local_raceline',
                'odom_topic': '/ego_racecar/odom',
                'drive_topic': '/drive',
                'pose_topic': '/ekf_pose',
                'verbose': LaunchConfiguration('mpc_verbose'),
            }.items(),
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
        global_init_value = LaunchConfiguration('amcl_global_initialization').perform(context)
        gpu_global_initialization = str(global_init_value).strip().lower() in ('true', '1', 'yes')
        gpu_amcl_overrides = {
            'use_sim_time': _bool_config('use_sim_time'),
            'num_particles': _int_config('amcl_num_particles'),
            'min_particles': _int_config('amcl_min_particles'),
            'max_particles': _int_config('amcl_max_particles'),
            'max_beams': _int_config('amcl_max_beams'),
            'use_kld_sampling': _bool_config('amcl_use_kld'),
            'update_min_d': _float_config('amcl_update_min_d'),
            'update_min_a': _float_config('amcl_update_min_a'),
            'cloud_publish_rate': _float_config('amcl_cloud_publish_rate'),
            'alpha1': _float_config('amcl_alpha1'),
            'alpha2': _float_config('amcl_alpha2'),
            'alpha3': _float_config('amcl_alpha3'),
            'alpha4': _float_config('amcl_alpha4'),
            'z_hit': _float_config('amcl_z_hit'),
            'z_rand': _float_config('amcl_z_rand'),
            'sigma_hit': _float_config('amcl_sigma_hit'),
            'resample_threshold': _float_config('amcl_resample_threshold'),
            'normalize_likelihood_by_beams': _bool_config(
                'amcl_normalize_likelihood_by_beams'),
            'likelihood_scale': _float_config('amcl_likelihood_scale'),
            'use_cluster_estimate': _bool_config('amcl_use_cluster_estimate'),
            'cluster_xy_bin_m': _float_config('amcl_cluster_xy_bin_m'),
            'cluster_radius_m': _float_config('amcl_cluster_radius_m'),
            'cluster_iterations': _int_config('amcl_cluster_iterations'),
            'cluster_min_covariance': _float_config('amcl_cluster_min_covariance'),
            'cluster_publish_min_weight': _float_config('amcl_cluster_publish_min_weight'),
            'debug_pre_resample_particles': _bool_config(
                'amcl_debug_pre_resample_particles'),
            'global_initialization': _bool_config('amcl_global_initialization'),
            'global_heading_trajectory_file': LaunchConfiguration('trajectory_file'),
        }
        if not gpu_global_initialization:
            gpu_amcl_overrides.update({
                'initial_pose_x': _float_config('initial_pose_x'),
                'initial_pose_y': _float_config('initial_pose_y'),
                'initial_pose_a': _float_config('initial_pose_yaw'),
            })

        actions.append(
            Node(
                package='f1tenth_localization',
                executable='gpu_amcl_cpp_node',
                name='gpu_amcl_cpp',
                output='screen',
                parameters=[
                    gpu_params,
                    gpu_amcl_overrides,
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
            '--csv-name', LaunchConfiguration('csv_name'),
            '--status-name', LaunchConfiguration('status_name'),
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

    if not headless:
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': _bool_config('use_sim_time')}],
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
        DeclareLaunchArgument('realistic_plant', default_value='true',
                              description='Use hardware-calibrated plant from MPC tune_realistic_v2.py'),
        DeclareLaunchArgument('map_file', default_value=default_map),
        DeclareLaunchArgument('trajectory_file', default_value=default_trajectory),
        DeclareLaunchArgument(
            'output_dir',
            default_value='f1tenth_localization/Benchmark/Matlab/sim_benchmark'),
        DeclareLaunchArgument('max_laps', default_value='10'),
        DeclareLaunchArgument('max_duration_sec', default_value='0.0'),
        DeclareLaunchArgument('csv_name', default_value='groundtruth_at_ekf.csv'),
        DeclareLaunchArgument('status_name', default_value='run_status.json'),
        DeclareLaunchArgument('initial_pose_x', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0'),
        DeclareLaunchArgument('sim_drive_topic', default_value='ackermann_cmd',
                              description='Topic consumed by simulator bridge'),
        DeclareLaunchArgument(
            'sim_drive_uses_acceleration_field',
            default_value='true',
            description='Use AckermannDrive.acceleration as gym acceleration command'),
        DeclareLaunchArgument('lateral_planner_avoidance_enabled',
                              default_value='false'),
        DeclareLaunchArgument('mpc_verbose', default_value='0'),
        DeclareLaunchArgument('amcl_num_particles', default_value='1000'),
        DeclareLaunchArgument('amcl_min_particles', default_value='1000'),
        DeclareLaunchArgument('amcl_max_particles', default_value='1000'),
        DeclareLaunchArgument('amcl_max_beams', default_value='270'),
        DeclareLaunchArgument('amcl_use_kld', default_value='false'),
        DeclareLaunchArgument('amcl_cloud_publish_rate', default_value='0.1'),
        DeclareLaunchArgument('amcl_normalize_likelihood_by_beams', default_value='true'),
        DeclareLaunchArgument('amcl_likelihood_scale', default_value='0.75'),
        DeclareLaunchArgument('amcl_use_cluster_estimate', default_value='true'),
        DeclareLaunchArgument('amcl_cluster_xy_bin_m', default_value='0.25'),
        DeclareLaunchArgument('amcl_cluster_radius_m', default_value='0.75'),
        DeclareLaunchArgument('amcl_cluster_iterations', default_value='3'),
        DeclareLaunchArgument('amcl_cluster_min_covariance', default_value='0.0001'),
        DeclareLaunchArgument('amcl_cluster_publish_min_weight', default_value='0.60'),
        DeclareLaunchArgument('amcl_debug_pre_resample_particles', default_value='false'),
        DeclareLaunchArgument('amcl_global_initialization', default_value='true'),
        DeclareLaunchArgument('amcl_update_min_d', default_value='0.0'),
        DeclareLaunchArgument('amcl_update_min_a', default_value='0.0'),
        DeclareLaunchArgument('amcl_alpha1', default_value='0.4'),
        DeclareLaunchArgument('amcl_alpha2', default_value='0.4'),
        DeclareLaunchArgument('amcl_alpha3', default_value='0.2'),
        DeclareLaunchArgument('amcl_alpha4', default_value='0.2'),
        DeclareLaunchArgument('amcl_z_hit', default_value='0.95'),
        DeclareLaunchArgument('amcl_z_rand', default_value='0.05'),
        DeclareLaunchArgument('amcl_sigma_hit', default_value='0.06'),
        DeclareLaunchArgument('amcl_resample_threshold', default_value='0.3'),
        DeclareLaunchArgument('ekf_transform_tolerance', default_value='0.02'),
        DeclareLaunchArgument('ekf_process_noise_scale', default_value='5.0'),
        DeclareLaunchArgument('monitor_print_every', default_value='40'),
        DeclareLaunchArgument('monitor_stage_match_max_ms', default_value='20.0'),
        DeclareLaunchArgument('monitor_strict_mode', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
