"""
Real-car launch for GPU AMCL system measurements.

This starts the hardware stack, GPU AMCL localization, EKF, ackermann mux,
pipeline latency monitor, and performance monitor. It intentionally does not
start MPC; start the controller separately after localization is ready.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    stack_pkg_dir = get_package_share_directory('f1tenth_stack')
    planner_pkg_dir = get_package_share_directory('f1tenth_planning')
    localization_pkg_dir = get_package_share_directory('f1tenth_localization')

    system_launch = os.path.join(stack_pkg_dir, 'launch', 'System_launch.py')
    bag_record_script = os.path.join(
        stack_pkg_dir, 'scripts', 'record_lateral_planner_bag.sh')
    joy_teleop_config = os.path.join(stack_pkg_dir, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(stack_pkg_dir, 'config', 'vesc.yaml')
    mux_config = os.path.join(stack_pkg_dir, 'config', 'mux.yaml')
    localization_params_file = os.path.join(
        localization_pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')
    default_map = os.path.join(planner_pkg_dir, 'maps', 'my_track_map.yaml')
    default_output_root = '/home/f1tenth/BachelorProject/PleaseWorkTest'
    pipeline_output_dir = PathJoinSubstitution([
        LaunchConfiguration('monitor_output_dir'), 'pipeline'])
    system_output_dir = PathJoinSubstitution([
        LaunchConfiguration('monitor_output_dir'), 'system'])
    bag_output_path = PathJoinSubstitution([
        LaunchConfiguration('monitor_output_dir'), 'bag', 'lateral_planner_bag'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'monitor_output_dir',
            default_value=default_output_root,
            description='Root directory for bag, pipeline, and system monitor outputs'),

        DeclareLaunchArgument(
            'record_bag',
            default_value='true',
            description='Record a lateral-planner localization bag into monitor_output_dir/bag'),

        DeclareLaunchArgument(
            'localization_params_file',
            default_value=localization_params_file,
            description='GPU AMCL / odom / EKF parameter file'),

        DeclareLaunchArgument(
            'map_file',
            default_value=default_map,
            description='Map YAML used by map_server'),

        DeclareLaunchArgument(
            'vesc_config',
            default_value=vesc_config,
            description='VESC configuration file'),

        DeclareLaunchArgument(
            'mux_config',
            default_value=mux_config,
            description='ackermann_mux configuration file'),

        DeclareLaunchArgument(
            'joy_config',
            default_value=joy_teleop_config,
            description='Joystick configuration file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock instead of wall time'),

        DeclareLaunchArgument(
            'use_lidar',
            default_value='true',
            description='Launch Hokuyo LiDAR driver'),

        DeclareLaunchArgument(
            'lidar_cluster',
            default_value='4',
            description='LiDAR clustering in racing mode: 1=1080, 2=540, 4=270 beams'),

        DeclareLaunchArgument(
            'mapping_mode',
            default_value='false',
            description='Full-resolution LiDAR mode; disables scan splitter and lateral planner'),

        DeclareLaunchArgument(
            'use_teleop',
            default_value='false',
            description='Launch joystick teleop. Default false for controller measurements'),

        DeclareLaunchArgument(
            'use_ackermann_mux',
            default_value='true',
            description='Keep /drive -> /ackermann_cmd mux active for MPC'),

        DeclareLaunchArgument(
            'use_lateral_planner',
            default_value='true',
            description='Launch lateral planner so MPC receives /local_raceline'),

        DeclareLaunchArgument(
            'lateral_planner_avoidance_enabled',
            default_value='false',
            description='Enable lateral planner obstacle avoidance if use_lateral_planner=true'),

        DeclareLaunchArgument(
            'bringup_delay_sec',
            default_value='2.0',
            description='Delay before hardware bringup starts'),

        DeclareLaunchArgument(
            'lateral_planner_delay_sec',
            default_value='2.0',
            description='Delay before lateral planner starts if enabled'),

        DeclareLaunchArgument(
            'use_dynamic_bicycle_model',
            default_value='true',
            description='Use slip-aware dynamic odometry model'),

        DeclareLaunchArgument(
            'oldOdom',
            default_value='false',
            description='Use legacy odometry implementation'),

        DeclareLaunchArgument(
            'amcl_max_beams',
            default_value='270',
            description='GPU AMCL max beams sampled from each scan'),

        DeclareLaunchArgument(
            'use_system_monitor',
            default_value='true',
            description='Launch real-car VESC/drive heartbeat monitor'),

        DeclareLaunchArgument(
            'monitor_vesc_timeout_sec',
            default_value='0.30',
            description='VESC heartbeat timeout'),

        DeclareLaunchArgument(
            'monitor_drive_timeout_sec',
            default_value='0.15',
            description='/drive heartbeat timeout after first drive command'),

        DeclareLaunchArgument(
            'monitor_drive_arm_on_first_message',
            default_value='true',
            description='Only enforce /drive timeout after the first command arrives'),

        DeclareLaunchArgument(
            'monitor_startup_grace_sec',
            default_value='5.0',
            description='Startup grace period for real-car heartbeat monitor'),

        DeclareLaunchArgument(
            'use_pipeline_monitor',
            default_value='true',
            description='Launch scan->AMCL->EKF->drive->ackermann latency monitor'),

        DeclareLaunchArgument(
            'pipeline_print_every',
            default_value='40',
            description='Print one latency summary every N completed scan cycles'),

        DeclareLaunchArgument(
            'pipeline_stage_match_max_ms',
            default_value='20.0',
            description='Maximum stage matching window for latency monitor'),

        DeclareLaunchArgument(
            'pipeline_strict_mode',
            default_value='false',
            description='Strict latency-monitor mismatch warnings'),

        DeclareLaunchArgument(
            'use_performance_monitor',
            default_value='true',
            description='Launch CPU/GPU/cache/EMC CSV monitor'),

        DeclareLaunchArgument(
            'system_monitor_cpu_sample_hz',
            default_value='100.0',
            description='CPU utilization sample rate'),

        DeclareLaunchArgument(
            'system_monitor_gpu_sample_hz',
            default_value='50.0',
            description='GPU utilization sample rate'),

        DeclareLaunchArgument(
            'system_monitor_csv_log_hz',
            default_value='50.0',
            description='Short CPU/GPU CSV log rate'),

        DeclareLaunchArgument(
            'system_monitor_long_csv_log_hz',
            default_value='1.0',
            description='Long CPU/GPU CSV log rate'),

        DeclareLaunchArgument(
            'system_monitor_memory_log_hz',
            default_value='1.0',
            description='CPU memory and CPU cache CSV log rate'),

        DeclareLaunchArgument(
            'system_monitor_memory_controller_log_hz',
            default_value='1.0',
            description='Jetson EMC/RAM bandwidth CSV log rate'),

        DeclareLaunchArgument(
            'system_monitor_emc_peak_bandwidth_mib_s',
            default_value='97275.0',
            description='Jetson peak memory bandwidth in MiB/s; 97275 ~= 102 GB/s'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(system_launch),
            launch_arguments={
                'localization_params_file': LaunchConfiguration('localization_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'vesc_config': LaunchConfiguration('vesc_config'),
                'mux_config': LaunchConfiguration('mux_config'),
                'joy_config': LaunchConfiguration('joy_config'),
                'use_teleop': LaunchConfiguration('use_teleop'),
                'use_ackermann_mux': LaunchConfiguration('use_ackermann_mux'),
                'use_lidar': LaunchConfiguration('use_lidar'),
                'mapping_mode': LaunchConfiguration('mapping_mode'),
                'lidar_cluster': LaunchConfiguration('lidar_cluster'),
                'use_lateral_planner': LaunchConfiguration('use_lateral_planner'),
                'lateral_planner_avoidance_enabled': LaunchConfiguration(
                    'lateral_planner_avoidance_enabled'),
                'lateral_planner_delay_sec': LaunchConfiguration('lateral_planner_delay_sec'),
                'map_file': LaunchConfiguration('map_file'),
                'bringup_delay_sec': LaunchConfiguration('bringup_delay_sec'),
                'use_dynamic_bicycle_model': LaunchConfiguration('use_dynamic_bicycle_model'),
                'oldOdom': LaunchConfiguration('oldOdom'),
                'use_localization': 'true',
                'amcl_max_beams': LaunchConfiguration('amcl_max_beams'),
                'use_system_monitor': LaunchConfiguration('use_system_monitor'),
                'monitor_vesc_timeout_sec': LaunchConfiguration('monitor_vesc_timeout_sec'),
                'monitor_drive_timeout_sec': LaunchConfiguration('monitor_drive_timeout_sec'),
                'monitor_drive_arm_on_first_message': LaunchConfiguration(
                    'monitor_drive_arm_on_first_message'),
                'monitor_startup_grace_sec': LaunchConfiguration('monitor_startup_grace_sec'),
            }.items(),
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
                'amcl_timing_topic': '/amcl_timing',
                'amcl_gpu_timing_topic': '/amcl_gpu_timing',
                'ekf_topic': '/ekf_pose',
                'drive_topic': '/drive',
                'ackermann_topic': '/ackermann_cmd',
                'stage_match_max_ms': ParameterValue(
                    LaunchConfiguration('pipeline_stage_match_max_ms'), value_type=float),
                'strict_mode': ParameterValue(
                    LaunchConfiguration('pipeline_strict_mode'), value_type=bool),
                'print_every': ParameterValue(
                    LaunchConfiguration('pipeline_print_every'), value_type=int),
                'log_to_csv': True,
                'csv_output_dir': pipeline_output_dir,
            }],
        ),

        Node(
            package='f1tenth_localization',
            executable='performance_monitor_cpp',
            name='performance_monitor',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_performance_monitor')),
            parameters=[{
                'output_dir': system_output_dir,
                'cpu_sample_hz': ParameterValue(
                    LaunchConfiguration('system_monitor_cpu_sample_hz'), value_type=float),
                'gpu_sample_hz': ParameterValue(
                    LaunchConfiguration('system_monitor_gpu_sample_hz'), value_type=float),
                'csv_log_hz': ParameterValue(
                    LaunchConfiguration('system_monitor_csv_log_hz'), value_type=float),
                'long_csv_log_hz': ParameterValue(
                    LaunchConfiguration('system_monitor_long_csv_log_hz'), value_type=float),
                'memory_log_hz': ParameterValue(
                    LaunchConfiguration('system_monitor_memory_log_hz'), value_type=float),
                'memory_controller_log_hz': ParameterValue(
                    LaunchConfiguration('system_monitor_memory_controller_log_hz'),
                    value_type=float),
                'emc_peak_bandwidth_mib_s': ParameterValue(
                    LaunchConfiguration('system_monitor_emc_peak_bandwidth_mib_s'),
                    value_type=float),
            }],
        ),

        ExecuteProcess(
            cmd=['bash', bag_record_script, bag_output_path],
            name='real_system_bag_recorder',
            output='screen',
            condition=IfCondition(LaunchConfiguration('record_bag')),
            sigterm_timeout='5',
            sigkill_timeout='5',
        ),
    ])
