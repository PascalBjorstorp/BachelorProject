"""

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue




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
    # ── Default map path ──
    default_map = os.path.join(planner_pkg_dir, 'maps', 'my_track_map.yaml')

    # --------------- Launch argument variables ----------
    localization_params_file_arg = LaunchConfiguration('localization_params_file')
    use_sim_time_arg = LaunchConfiguration('use_sim_time')
    vesc_config_arg = LaunchConfiguration('vesc_config')
    mux_config_arg = LaunchConfiguration('mux_config')
    use_teleop_arg = LaunchConfiguration('use_teleop')
    use_lidar_arg = LaunchConfiguration('use_lidar')
    mapping_mode_arg = LaunchConfiguration('mapping_mode')
    lidar_cluster_arg = LaunchConfiguration('lidar_cluster')
    lateral_planner_avoidance_enabled_arg = LaunchConfiguration('lateral_planner_avoidance_enabled')
    lateral_planner_delay_sec_arg = LaunchConfiguration('lateral_planner_delay_sec')
    map_file_arg = LaunchConfiguration('map_file')
    bringup_delay_sec_arg = LaunchConfiguration('bringup_delay_sec')
    use_dynamic_bicycle_model_arg = LaunchConfiguration('use_dynamic_bicycle_model')
    old_odom_arg = LaunchConfiguration('oldOdom')
    use_localization_arg = LaunchConfiguration('use_localization')
    amcl_num_particles_arg = LaunchConfiguration('amcl_num_particles')
    amcl_min_particles_arg = LaunchConfiguration('amcl_min_particles')
    amcl_max_particles_arg = LaunchConfiguration('amcl_max_particles')
    amcl_max_beams_arg = LaunchConfiguration('amcl_max_beams')
    amcl_use_kld_arg = LaunchConfiguration('amcl_use_kld')
    amcl_global_initialization_arg = LaunchConfiguration('amcl_global_initialization')
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
        
        DeclareLaunchArgument(  'mux_config', 
                                default_value=mux_config,
                                description='Path to ackermann_mux configuration file'),

        DeclareLaunchArgument(  'joy_config', 
                                default_value=joy_teleop_config,
                                description='Path to joystick configuration file'),

        DeclareLaunchArgument(  'use_teleop', 
                                default_value='true',
                                description='Launch joystick teleop and mux'),
        
        DeclareLaunchArgument(  'use_lidar', 
                                default_value='true',
                                description='Launch LiDAR driver (Hokuyo SCIP 2.0, 40 Hz)'),
        
        DeclareLaunchArgument(  'mapping_mode', 
                                default_value='false',
                                description='Mapping mode: 270 beams @ 20 Hz, no scan splitter or lateral planner'),

        DeclareLaunchArgument(  'lidar_cluster',
                                default_value='4',
                                description='LiDAR clustering in racing mode: 1=1080 beams, 2=540, 4=270'),

        DeclareLaunchArgument(  'lateral_planner_avoidance_enabled',
                    default_value='true',
                    description='Enable lateral planner obstacle avoidance (false publishes baseline raceline)'),

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
            'amcl_num_particles',
            default_value='1000',
            description='GPU AMCL initial/fixed particle count'),

        DeclareLaunchArgument(
            'amcl_min_particles',
            default_value='1000',
            description='GPU AMCL minimum particle count'),

        DeclareLaunchArgument(
            'amcl_max_particles',
            default_value='1000',
            description='GPU AMCL maximum particle count'),

        DeclareLaunchArgument(
            'amcl_max_beams',
            default_value='270',
            description='GPU AMCL max beams sampled from each scan'),

        DeclareLaunchArgument(
            'amcl_use_kld',
            default_value='false',
            description='Enable GPU AMCL KLD adaptive particle sampling'),

        DeclareLaunchArgument(
            'amcl_global_initialization',
            default_value='true',
            description='Seed GPU AMCL particles globally along raceline with heading cone'),

        DeclareLaunchArgument(
            'use_system_monitor',
            default_value='true',
            description='Monitor VESC telemetry and /drive heartbeat'),

        DeclareLaunchArgument(
            'monitor_vesc_timeout_sec',
            default_value='0.50',
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
                    'num_particles': ParameterValue(amcl_num_particles_arg, value_type=int),
                    'min_particles': ParameterValue(amcl_min_particles_arg, value_type=int),
                    'max_particles': ParameterValue(amcl_max_particles_arg, value_type=int),
                    'max_beams': ParameterValue(amcl_max_beams_arg, value_type=int),
                    'use_kld_sampling': ParameterValue(amcl_use_kld_arg, value_type=bool),
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
                    'num_particles': ParameterValue(amcl_num_particles_arg, value_type=int),
                    'min_particles': ParameterValue(amcl_min_particles_arg, value_type=int),
                    'max_particles': ParameterValue(amcl_max_particles_arg, value_type=int),
                    'max_beams': ParameterValue(amcl_max_beams_arg, value_type=int),
                    'use_kld_sampling': ParameterValue(amcl_use_kld_arg, value_type=bool),
                    'global_initialization': ParameterValue(
                        amcl_global_initialization_arg, value_type=bool),
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
                #  VESC nodes (single process, zero-copy intra-process comms)
                # ══════════════════════
                ComposableNodeContainer(
                    name='vesc_container',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='vesc_driver',
                            plugin='vesc_driver::VescDriver',
                            name='vesc_driver_node',
                            parameters=[vesc_config_arg],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        ComposableNode(
                            package='vesc_ackermann',
                            plugin='vesc_ackermann::VescToOdom',
                            name='vesc_to_odom_node',
                            parameters=[
                                vesc_config_arg,
                                {'use_dynamic_bicycle_model': use_dynamic_bicycle_model_arg},
                            ],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        ComposableNode(
                            package='vesc_ackermann',
                            plugin='vesc_ackermann::AckermannToVesc',
                            name='ackermann_to_vesc_node',
                            parameters=[vesc_config_arg],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                    output='screen',
                    condition=UnlessCondition(old_odom_arg),
                ),

                ComposableNodeContainer(
                    name='vesc_container_old',
                    namespace='',
                    package='rclcpp_components',
                    executable='component_container',
                    composable_node_descriptions=[
                        ComposableNode(
                            package='vesc_driver',
                            plugin='vesc_driver::VescDriver',
                            name='vesc_driver_node',
                            parameters=[vesc_config_arg],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        ComposableNode(
                            package='vesc_ackermann',
                            plugin='vesc_ackermann::VescToOdomOld',
                            name='vesc_to_odom_old_node',
                            parameters=[vesc_config_arg],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                        ComposableNode(
                            package='vesc_ackermann',
                            plugin='vesc_ackermann::AckermannToVesc',
                            name='ackermann_to_vesc_node',
                            parameters=[vesc_config_arg],
                            extra_arguments=[{'use_intra_process_comms': True}],
                        ),
                    ],
                    output='screen',
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
                    condition=IfCondition(use_teleop_arg),
                ),


                # ══════════════════════
                #  LiDAR — Custom SCIP 2.0 driver
                # ══════════════════════
                # Normal mode: 270 beams @ 40 Hz (cluster=4, skip=0)
                Node(
                    package='f1tenth_lidar',
                    executable='hokuyo_scip_driver_node',
                    name='hokuyo_scip_driver',
                    output='screen',
                    parameters=[hokuyo_config, {'skip': 0, 'cluster': lidar_cluster_arg}],
                    condition=IfCondition(PythonExpression([
                        "'", use_lidar_arg, "' == 'true' and '",
                        mapping_mode_arg, "' != 'true'"
                    ])),
                ),
                # Mapping mode: 1080 beams @ 40 Hz (cluster=1, skip=0 — full resolution)
                Node(
                    package='f1tenth_lidar',
                    executable='hokuyo_scip_driver_node',
                    name='hokuyo_scip_driver',
                    output='screen',
                    parameters=[hokuyo_config, {'skip': 0, 'cluster': 1}],
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
                    condition=UnlessCondition(mapping_mode_arg),
                ),

                # ══════════════════════
                #  Static transforms
                # ══════════════════════
                # ego_racecar/base_link → ego_racecar/laser
                # Update x, y, z to match your LiDAR mounting position.
                # Use update_vehicle_params.py in the workspace root to set all values.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_baselink_to_laser',
                    arguments=[
                        '--x', '0.265',
                        '--y', '0.0',
                        '--z', '0.05',
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'ego_racecar/base_link',
                        '--child-frame-id', 'ego_racecar/laser',
                    ],
                ),

                # ego_racecar/base_link → ego_racecar/imu
                # VESC firmware already compensates for upside-down mounting — identity rotation.
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_baselink_to_imu',
                    arguments=[
                        '--x', '0.11',
                        '--y', '0.0',
                        '--z', '0.0',
                        '--roll', '0.0',
                        '--pitch', '0.0',
                        '--yaw', '0.0',
                        '--frame-id', 'ego_racecar/base_link',
                        '--child-frame-id', 'ego_racecar/imu',
                    ],
                ),
            ],
        ),

    ])
