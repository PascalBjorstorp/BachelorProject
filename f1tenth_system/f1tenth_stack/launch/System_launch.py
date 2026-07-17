"""

"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode
from launch_ros.parameter_descriptions import ParameterValue


def _geometry_defaults(vesc_config: str) -> dict[str, str]:
    """Read the geometry promoted by the calibration campaign.

    A launch-file substitution cannot inspect an arbitrary ``vesc_config``
    override after launch has started.  The explicit arguments declared below
    remain available for that case, while the normal stack launch receives the
    measured defaults stored alongside the VESC calibration values.
    """
    defaults = {
        'laser_to_base_x_m': '0.265', 'laser_to_base_y_m': '0.0',
        'laser_to_base_z_m': '0.05', 'laser_to_base_yaw_rad': '0.0',
        'imu_to_base_x_m': '0.160', 'imu_to_base_y_m': '0.0',
        'imu_to_base_z_m': '0.0703', 'imu_to_base_yaw_rad': '0.0',
    }
    try:
        document = yaml.safe_load(Path(vesc_config).read_text(encoding='utf-8')) or {}
        geometry = document.get('vehicle_geometry', {}).get('ros__parameters', {})
        if not isinstance(geometry, dict):
            return defaults
        for key, fallback in defaults.items():
            defaults[key] = str(float(geometry.get(key, fallback)))
    except (OSError, TypeError, ValueError, yaml.YAMLError):
        pass
    return defaults



def generate_launch_description():
    # ── Package directories for included files ──
    localization_pkg_dir = get_package_share_directory('f1tenth_localization')
    lateral_planner_pkg_dir = get_package_share_directory('f1tenth_lateral_planner')
    lidar_pkg_dir = get_package_share_directory('f1tenth_lidar')
    stack_pkg_dir = get_package_share_directory('f1tenth_stack')
    planner_pkg_dir = get_package_share_directory('f1tenth_planning')

    # ── YAML file paths ──  
    localization_params_file = os.path.join(localization_pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')
    # ── Config file paths ──
    joy_teleop_config = os.path.join(stack_pkg_dir, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(stack_pkg_dir, 'config', 'vesc.yaml')
    mux_config = os.path.join(stack_pkg_dir, 'config', 'mux.yaml')
    hokuyo_config = os.path.join(lidar_pkg_dir, 'config', 'hokuyo_ust10lx.yaml')
    geometry = _geometry_defaults(vesc_config)
    # ── Default map path ──
    default_map = os.path.join(planner_pkg_dir, 'maps', 'my_track_map.yaml')

    # --------------- Launch argument variables ----------
    localization_params_file_arg = LaunchConfiguration('localization_params_file')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    vesc_config_arg = LaunchConfiguration('vesc_config')
    vesc_priority_prefix_arg = LaunchConfiguration('vesc_priority_prefix')
    mux_config_arg = LaunchConfiguration('mux_config')
    use_teleop_arg = LaunchConfiguration('use_teleop')
    use_ackermann_mux_arg = LaunchConfiguration('use_ackermann_mux')
    use_lidar_arg = LaunchConfiguration('use_lidar')
    mapping_mode_arg = LaunchConfiguration('mapping_mode')
    lidar_ip_address_arg = LaunchConfiguration('lidar_ip_address')
    lidar_cluster_arg = LaunchConfiguration('lidar_cluster')
    full_scan_arg = LaunchConfiguration('full_scan')
    use_lateral_planner_arg = LaunchConfiguration('use_lateral_planner')
    lateral_planner_avoidance_enabled_arg = LaunchConfiguration('lateral_planner_avoidance_enabled')
    lateral_planner_delay_sec_arg = LaunchConfiguration('lateral_planner_delay_sec')
    map_file_arg = LaunchConfiguration('map_file')
    bringup_delay_sec_arg = LaunchConfiguration('bringup_delay_sec')
    use_dynamic_bicycle_model_arg = LaunchConfiguration('use_dynamic_bicycle_model')
    old_odom_arg = LaunchConfiguration('oldOdom')
    use_localization_arg = LaunchConfiguration('use_localization')
    amcl_max_beams_arg = LaunchConfiguration('amcl_max_beams')
    amcl_cloud_publish_rate_arg = LaunchConfiguration('amcl_cloud_publish_rate')
    amcl_debug_pre_resample_particles_arg = LaunchConfiguration(
        'amcl_debug_pre_resample_particles')
    use_system_monitor_arg = LaunchConfiguration('use_system_monitor')
    monitor_vesc_timeout_sec_arg = LaunchConfiguration('monitor_vesc_timeout_sec')
    monitor_drive_timeout_sec_arg = LaunchConfiguration('monitor_drive_timeout_sec')
    monitor_drive_arm_on_first_message_arg = LaunchConfiguration(
        'monitor_drive_arm_on_first_message')
    monitor_startup_grace_sec_arg = LaunchConfiguration('monitor_startup_grace_sec')

    return LaunchDescription([

        # ── Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(  'localization_params_file',
                                default_value=localization_params_file,
                                description='Path to the unified YAML parameter file for the localization stack'),

        DeclareLaunchArgument(  'use_sim_time',
                                default_value='false',
                                description='Use /clock for simulation time'),

        DeclareLaunchArgument(  'vesc_config', 
                                default_value=vesc_config,
                                description='Path to VESC configuration file'),

        DeclareLaunchArgument(
            'laser_to_base_x_m', default_value=geometry['laser_to_base_x_m'],
            description='base_link-to-LiDAR x [m]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'laser_to_base_y_m', default_value=geometry['laser_to_base_y_m'],
            description='base_link-to-LiDAR y [m]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'laser_to_base_z_m', default_value=geometry['laser_to_base_z_m'],
            description='base_link-to-LiDAR z [m]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'laser_to_base_yaw_rad', default_value=geometry['laser_to_base_yaw_rad'],
            description='base_link-to-LiDAR yaw [rad]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'imu_to_base_x_m', default_value=geometry['imu_to_base_x_m'],
            description='base_link-to-IMU x [m]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'imu_to_base_y_m', default_value=geometry['imu_to_base_y_m'],
            description='base_link-to-IMU y [m]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'imu_to_base_z_m', default_value=geometry['imu_to_base_z_m'],
            description='base_link-to-IMU z [m]; default is promoted vehicle_geometry'),
        DeclareLaunchArgument(
            'imu_to_base_yaw_rad', default_value=geometry['imu_to_base_yaw_rad'],
            description='base_link-to-IMU yaw [rad]; default is promoted vehicle_geometry'),

        DeclareLaunchArgument(
            'vesc_priority_prefix',
            default_value='taskset -c 2',
            description=(
                'Command prefix for each VESC process. Default pins all VESC '
                'nodes to CPU 2. Use empty string to disable, or e.g. '
                '"sudo chrt -f 60 taskset -c 2" for RT scheduling.'
            )),
        
        DeclareLaunchArgument(  'mux_config', 
                                default_value=mux_config,
                                description='Path to ackermann_mux configuration file'),

        DeclareLaunchArgument(  'joy_config', 
                                default_value=joy_teleop_config,
                                description='Path to joystick configuration file'),

        DeclareLaunchArgument(  'use_teleop', 
                                default_value='false',
                                description='Launch joystick teleop'),

        DeclareLaunchArgument(  'use_ackermann_mux',
                                default_value='true',
                                description='Launch ackermann_mux for /drive -> /ackermann_cmd'),
        
        DeclareLaunchArgument(  'use_lidar', 
                                default_value='true',
                                description='Launch LiDAR driver (Hokuyo SCIP 2.0, 40 Hz)'),
        
        DeclareLaunchArgument(  'mapping_mode', 
                                default_value='false',
                                description='Mapping mode: all 1080 beams @ 40 Hz, no scan splitter or lateral planner'),

        DeclareLaunchArgument(  'lidar_cluster',
                                default_value='4',
                                description='Racing /scan clustering: 4=270 beams for AMCL; full_scan keeps 1080 on /scan_full'),

        DeclareLaunchArgument(  'lidar_ip_address',
                                default_value='192.168.10.10',
                                description='Hokuyo LiDAR IPv4 address'),

        DeclareLaunchArgument(  'full_scan',
                                default_value='false',
                                description='Record all 1080 beams on /scan_full while retaining the reduced racing /scan for AMCL'),

        DeclareLaunchArgument(  'lateral_planner_avoidance_enabled',
                    default_value='false',
                    description='Enable lateral planner obstacle avoidance (false publishes baseline raceline)'),

        DeclareLaunchArgument(  'use_lateral_planner',
                    default_value='true',
                    description='Launch lateral planner after bringup delay'),

        DeclareLaunchArgument(  'lateral_planner_delay_sec',
                    default_value='2.0',
                    description='Delay before starting the lateral planner after bringup starts (seconds)'),

        DeclareLaunchArgument(  'map_file', 
                                default_value=default_map,
                                description='Path to the map YAML file for map_server'),

        DeclareLaunchArgument(  'bringup_delay_sec',
                    default_value='2.0',
                    description='Delay before bringup starts (seconds)'),

        DeclareLaunchArgument(
                'use_dynamic_bicycle_model',
                default_value='true',
                description='Enable dynamic bicycle model inside vesc_to_odom node'),

        DeclareLaunchArgument(
            'oldOdom',
            default_value='false',
            description='Use legacy analytical vesc_to_odom implementation'),

        DeclareLaunchArgument(
            'use_localization',
            default_value='true',
            description='Launch the GPU AMCL localization stack'),

        DeclareLaunchArgument(
            'amcl_max_beams',
            default_value='270',
            description='GPU AMCL max beams sampled from the reduced racing scan'),

        DeclareLaunchArgument(
            'amcl_cloud_publish_rate',
            default_value='0.0',
            description='Particle cloud publish rate in Hz; 0 disables particle cloud downloads'),

        DeclareLaunchArgument(
            'amcl_debug_pre_resample_particles',
            default_value='false',
            description='Publish weighted pre-resample particle cloud debug topic'),

        DeclareLaunchArgument(
            'use_system_monitor',
            default_value='true',
            description='Monitor VESC telemetry and /drive heartbeat'),

        DeclareLaunchArgument(
            'monitor_vesc_timeout_sec',
            default_value='0.30',
            description='Seconds without /sensors/core before VESC error'),

        DeclareLaunchArgument(
            'monitor_drive_timeout_sec',
            default_value='0.15',
            description='Seconds without /drive before command error'),

        DeclareLaunchArgument(
            'monitor_drive_arm_on_first_message',
            default_value='true',
            description='Start /drive heartbeat only after first /drive message'),

        DeclareLaunchArgument(
            'monitor_startup_grace_sec',
            default_value='5.0',
            description='Startup grace period before missing-topic errors'),


        # ------------------------------- LOCALIZATION NODES -------------------------------

        # ── Odom fusion node ───────────────────
        Node(
            package='f1tenth_localization',
            executable='odom_fused_node',
            name='odom_fused',
            output='screen',
            parameters=[
                localization_params_file_arg,
                {'use_sim_time': use_sim_time_arg},
                {
                    'max_beams': ParameterValue(amcl_max_beams_arg, value_type=int),
                },
            ],
            condition=IfCondition(use_localization_arg),
        ),

        # ── AMCL node ─────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='gpu_amcl_cpp_node',
            name='gpu_amcl_cpp',
            output='screen',
            parameters=[
                localization_params_file_arg,
                {'use_sim_time': use_sim_time_arg},
                {
                    'max_beams': ParameterValue(amcl_max_beams_arg, value_type=int),
                    'cloud_publish_rate': ParameterValue(
                        amcl_cloud_publish_rate_arg, value_type=float),
                    'debug_pre_resample_particles': ParameterValue(
                        amcl_debug_pre_resample_particles_arg, value_type=bool),

                },
            ],
            condition=IfCondition(use_localization_arg),
        ),

        # ── EKF node ──────────────────────────────────────────────
        Node(
            package='f1tenth_localization',
            executable='ekf_localization_node',
            name='ekf_localization',
            output='screen',
            parameters=[
                localization_params_file_arg,
                {'use_sim_time': use_sim_time_arg},
            ],
            condition=IfCondition(use_localization_arg),
        ),



        # ------------------------------- Delay before bringup ------------------------
        TimerAction(
            period=bringup_delay_sec_arg,
            actions=[
                LogInfo(msg='Bringup delay elapsed.'),

                # ------------------------------- BRINGUP NODES -------------------------------

                # ══════════════════════
                #  Map Server (always)
                # ══════════════════════
                # Serves the static occupancy-grid map on /map with transient_local QoS.
                # Started early so the map is available before localization and scan_splitter.
                LifecycleNode(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    namespace='/',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time_arg,
                        'yaml_filename': map_file_arg,
                    }],
                ),

                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_map',
                    output='screen',
                    parameters=[{
                        'use_sim_time': use_sim_time_arg,
                        'autostart': True,
                        'node_names': ['map_server'],
                        'bond_timeout': 0.0,
                    }],
                ),
                # ══════════════════════
                #  VESC nodes (separate prioritized processes)
                # ══════════════════════
                Node(
                    package='vesc_driver',
                    executable='vesc_driver_node',
                    name='vesc_driver_node',
                    output='screen',
                    parameters=[vesc_config_arg],
                    prefix=vesc_priority_prefix_arg,
                    condition=UnlessCondition(old_odom_arg),
                ),

                Node(
                    package='vesc_ackermann',
                    executable='vesc_to_odom_node',
                    name='vesc_to_odom_node',
                    output='screen',
                    parameters=[
                        vesc_config_arg,
                        {'use_dynamic_bicycle_model': use_dynamic_bicycle_model_arg},
                    ],
                    prefix=vesc_priority_prefix_arg,
                    condition=UnlessCondition(old_odom_arg),
                ),

                Node(
                    package='vesc_ackermann',
                    executable='ackermann_to_vesc_node',
                    name='ackermann_to_vesc_node',
                    output='screen',
                    parameters=[vesc_config_arg],
                    prefix=vesc_priority_prefix_arg,
                    condition=UnlessCondition(old_odom_arg),
                ),

                Node(
                    package='vesc_driver',
                    executable='vesc_driver_node',
                    name='vesc_driver_node',
                    output='screen',
                    parameters=[vesc_config_arg],
                    prefix=vesc_priority_prefix_arg,
                    condition=IfCondition(old_odom_arg),
                ),

                Node(
                    package='vesc_ackermann',
                    executable='vesc_to_odom_old_node',
                    name='vesc_to_odom_old_node',
                    output='screen',
                    parameters=[vesc_config_arg],
                    prefix=vesc_priority_prefix_arg,
                    condition=IfCondition(old_odom_arg),
                ),

                Node(
                    package='vesc_ackermann',
                    executable='ackermann_to_vesc_node',
                    name='ackermann_to_vesc_node',
                    output='screen',
                    parameters=[vesc_config_arg],
                    prefix=vesc_priority_prefix_arg,
                    condition=IfCondition(old_odom_arg),
                ),

                Node(
                    package='f1tenth_stack',
                    executable='system_monitor',
                    name='system_monitor',
                    output='screen',
                    parameters=[{
                        'vesc_topic': '/sensors/core',
                        'drive_topic': '/drive',
                        'vesc_timeout_sec': ParameterValue(
                            monitor_vesc_timeout_sec_arg, value_type=float),
                        'drive_timeout_sec': ParameterValue(
                            monitor_drive_timeout_sec_arg, value_type=float),
                        'drive_arm_on_first_message': ParameterValue(
                            monitor_drive_arm_on_first_message_arg, value_type=bool),
                        'startup_grace_sec': ParameterValue(
                            monitor_startup_grace_sec_arg, value_type=float),
                    }],
                    condition=IfCondition(use_system_monitor_arg),
                ),

                # ══════════════════════
                #  Ackermann_Mux
                # ══════════════════════
                Node(
                    package='joy',
                    executable='joy_node',
                    name='joy',
                    parameters=[LaunchConfiguration('joy_config')],
                    condition=IfCondition(use_teleop_arg),
                ),

                Node(
                    package='joy_teleop',
                    executable='joy_teleop',
                    name='joy_teleop',
                    parameters=[LaunchConfiguration('joy_config')],
                    condition=IfCondition(use_teleop_arg),
                ),

                Node(
                    package='ackermann_mux',
                    executable='ackermann_mux',
                    name='ackermann_mux',
                    parameters=[mux_config_arg],
                    condition=IfCondition(use_ackermann_mux_arg),
                ),


                # ══════════════════════
                #  LiDAR — Custom SCIP 2.0 driver
                # ══════════════════════
                # Normal racing mode: 270 beams @ 40 Hz (cluster=4, skip=0).
                # Calibration launches bypass this path and request cluster=1.
                Node(
                    package='f1tenth_lidar',
                    executable='hokuyo_scip_driver_node',
                    name='hokuyo_scip_driver',
                    output='screen',
                    parameters=[hokuyo_config, {
                        'ip_address': lidar_ip_address_arg,
                        'skip': 0,
                        'cluster': lidar_cluster_arg,
                    }],
                    condition=IfCondition(PythonExpression([
                        "'", use_lidar_arg, "' == 'true' and '",
                        mapping_mode_arg, "' != 'true' and '",
                        full_scan_arg, "' != 'true'"
                    ])),
                ),
                # Full-recording mode: driver publishes all 1080 beams on
                # /scan_full while the relay keeps /scan reduced for AMCL.
                Node(
                    package='f1tenth_lidar',
                    executable='hokuyo_scip_driver_node',
                    name='hokuyo_scip_driver',
                    output='screen',
                    parameters=[hokuyo_config, {
                        'ip_address': lidar_ip_address_arg,
                        'skip': 0,
                        'cluster': 1,
                        'scan_topic': '/scan_full',
                    }],
                    condition=IfCondition(PythonExpression([
                        "'", use_lidar_arg, "' == 'true' and '",
                        mapping_mode_arg, "' != 'true' and '",
                        full_scan_arg, "' == 'true'"
                    ])),
                ),
                Node(
                    package='f1tenth_lidar',
                    executable='scan_downsampler_node',
                    name='scan_downsampler',
                    output='screen',
                    parameters=[{
                        'input_topic': '/scan_full',
                        'output_topic': '/scan',
                        'cluster': lidar_cluster_arg,
                    }],
                    condition=IfCondition(PythonExpression([
                        "'", use_lidar_arg, "' == 'true' and '",
                        mapping_mode_arg, "' != 'true' and '",
                        full_scan_arg, "' == 'true'"
                    ])),
                ),
                # Mapping mode: 1080 beams @ 40 Hz (cluster=1, skip=0 — full resolution)
                Node(
                    package='f1tenth_lidar',
                    executable='hokuyo_scip_driver_node',
                    name='hokuyo_scip_driver',
                    output='screen',
                    parameters=[hokuyo_config, {
                        'ip_address': lidar_ip_address_arg,
                        'skip': 0,
                        'cluster': 1,
                    }],
                    condition=IfCondition(PythonExpression([
                        "'", use_lidar_arg, "' == 'true' and '",
                        mapping_mode_arg, "' == 'true'"
                    ])),
                ),

                # ══════════════════════
                #  Scan Splitter — /scan → /scan_walls + /scan_obstacles
                # ══════════════════════
                # Only launched in racing mode (mapping_mode=false).
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(lidar_pkg_dir, 'launch', 'scan_splitter.launch.py')
                    ),
                    condition=UnlessCondition(mapping_mode_arg),
                ),

                # ══════════════════════
                #  Lateral Planner — opponent avoidance → /local_raceline
                # ══════════════════════
                # Only launched in racing mode (mapping_mode=false).
                TimerAction(
                    period=lateral_planner_delay_sec_arg,
                    actions=[
                        IncludeLaunchDescription(
                            PythonLaunchDescriptionSource(
                                os.path.join(lateral_planner_pkg_dir, 'launch', 'lateral_planner.launch.py')
                            ),
                            launch_arguments={
                                'avoidance_enabled': lateral_planner_avoidance_enabled_arg,
                            }.items(),
                        ),
                    ],
                    condition=IfCondition(PythonExpression([
                        "'", mapping_mode_arg, "' != 'true' and '",
                        use_lateral_planner_arg, "' == 'true'"
                    ])),
                ),

                # ══════════════════════
                #  Static transforms
                # ══════════════════════
                # Defaults come from the promoted vehicle_geometry section in
                # vesc.yaml.  Explicit launch arguments support alternate
                # configurations without changing the promoted values.
                # ego_racecar/base_link → ego_racecar/laser
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_baselink_to_laser',
                    arguments=[
                        '--x', LaunchConfiguration('laser_to_base_x_m'),
                        '--y', LaunchConfiguration('laser_to_base_y_m'),
                        '--z', LaunchConfiguration('laser_to_base_z_m'),
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', LaunchConfiguration('laser_to_base_yaw_rad'),
                        '--frame-id', 'ego_racecar/base_link',
                        '--child-frame-id', 'ego_racecar/laser',
                    ],
                ),

                # ego_racecar/base_link → ego_racecar/imu
                # Firmware's upside-down compensation remains separate from the
                # measured planar mounting transform published here.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_baselink_to_imu',
                    arguments=[
                        '--x', LaunchConfiguration('imu_to_base_x_m'),
                        '--y', LaunchConfiguration('imu_to_base_y_m'),
                        '--z', LaunchConfiguration('imu_to_base_z_m'),
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', LaunchConfiguration('imu_to_base_yaw_rad'),
                        '--frame-id', 'ego_racecar/base_link',
                        '--child-frame-id', 'ego_racecar/imu',
                    ],
                ),
            ],
        ),

    ])
