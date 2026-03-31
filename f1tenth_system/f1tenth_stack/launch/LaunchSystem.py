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
    use_lidar_arg = LaunchConfiguration('use_lidar')
    mapping_mode_arg = LaunchConfiguration('mapping_mode')
    trajectory_file_arg = LaunchConfiguration('trajectory_file')
    map_file_arg = LaunchConfiguration('map_file')
    bringup_delay_sec_arg = LaunchConfiguration('bringup_delay_sec')

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
        
        DeclareLaunchArgument(  'use_lidar', 
                                default_value='true',
                                description='Launch LiDAR driver (Hokuyo SCIP 2.0, 40 Hz)'),
        
        DeclareLaunchArgument(  'mapping_mode', 
                                default_value='false',
                                description='Mapping mode: 270 beams @ 20 Hz, no scan splitter or lateral planner'),
        
        DeclareLaunchArgument(  'trajectory_file', 
                                default_value='__from_yaml__',
                                description='Optional override for lateral planner trajectory_file (default: YAML value)'),
        
        DeclareLaunchArgument(  'map_file', 
                                default_value=default_map,
                                description='Path to the map YAML file for map_server'),

        DeclareLaunchArgument(  'bringup_delay_sec',
                    default_value='2.0',
                    description='Delay before bringup starts (seconds)'),


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
            ],
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
            ],
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
                ),

                # ══════════════════════
                #  Ackermann_Mux
                # ══════════════════════
                Node(
                    package='ackermann_mux',
                    executable='ackermann_mux',
                    name='ackermann_mux',
                    parameters=[mux_config_arg],
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
                    parameters=[hokuyo_config, {'skip': 0}],
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
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(lateral_planner_pkg_dir, 'launch', 'lateral_planner.launch.py')
                    ),
                    launch_arguments={
                        'trajectory_file': trajectory_file_arg,
                    }.items(),
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


