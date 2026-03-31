"""
Launch file for the C++ GPU AMCL localization stack.

Launches three nodes:
  1. gpu_amcl_cpp      — GPU particle-filter AMCL (40 Hz, subscribes to /scan_walls)
  2. odom_fused        — IMU + wheel odom fusion (200 Hz)
  3. ekf_localization  — EKF sensor fusion + TF broadcast

The map_server is expected to be running from bringup_launch.py.
The scan splitter (/scan → /scan_walls) is expected to be running
from the bringup_launch.py driver stack.

All parameters are loaded from config/gpu_amcl_cpp_params.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, LifecycleNode, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode




def generate_launch_description():
    localization_pkg_dir = get_package_share_directory('f1tenth_localization')
    localization_params_file = os.path.join(localization_pkg_dir, 'config', 'gpu_amcl_cpp_params.yaml')

    stack_pkg_share = get_package_share_directory('f1tenth_stack')
    lidar_pkg_share = get_package_share_directory('f1tenth_lidar')

    # ── Config file paths ──
    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')
    hokuyo_config = os.path.join(lidar_pkg_share, 'config', 'hokuyo_ust10lx.yaml')


    return LaunchDescription([
        # ------------------------------- LOCALIZATION NODES -------------------------------

        # ── Arguments ──────────────────────────────────────────────
        DeclareLaunchArgument(
            'localization params_file',
            default_value=localization_params_file,
            description='Path to the unified YAML parameter file for the localization stack'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        # NOTE: map_server is now launched from bringup_launch.py
        # so the map is available before localization starts.

        # ── Odom fusion node (must start first) ───────────────────
        Node(
            package='f1tenth_localization',
            executable='odom_fused_node',
            name='odom_fused',
            output='screen',
            parameters=[
                LaunchConfiguration('localization params_file'),
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
                LaunchConfiguration('localization params_file'),
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
                LaunchConfiguration('localization params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),


        # ------------------------------- BRINGUP NODES -------------------------------

    ])








def generate_launch_description():
    # ── Config file paths ──
    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    sensors_config = os.path.join(pkg_share, 'config', 'sensors.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')
    hokuyo_config = os.path.join(lidar_pkg_share, 'config', 'hokuyo_ust10lx.yaml')

    # ── Package directories for included launch files ──
    lidar_pkg_dir = get_package_share_directory('f1tenth_lidar')
    lateral_planner_pkg_dir = get_package_share_directory('f1tenth_lateral_planner')

    # ── Default map path ──
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share))))
    default_map = os.path.join(workspace_root, 'f1tenth_sim', 'maps', 'my_track_map.yaml')

    # ── Launch arguments ──
    ld = LaunchDescription([
        DeclareLaunchArgument('joy_config', default_value=joy_teleop_config,
                              description='Path to joystick configuration file'),
        DeclareLaunchArgument('vesc_config', default_value=vesc_config,
                              description='Path to VESC configuration file'),
        DeclareLaunchArgument('sensors_config', default_value=sensors_config,
                              description='Path to sensors configuration file'),
        DeclareLaunchArgument('mux_config', default_value=mux_config,
                              description='Path to ackermann_mux configuration file'),
        DeclareLaunchArgument('use_teleop', default_value='true',
                              description='Launch joystick teleop and mux'),
        DeclareLaunchArgument('use_lidar', default_value='true',
                              description='Launch LiDAR driver (Hokuyo SCIP 2.0, 40 Hz)'),
        DeclareLaunchArgument('mapping_mode', default_value='false',
                              description='Mapping mode: 270 beams @ 20 Hz, no scan splitter or lateral planner'),
        DeclareLaunchArgument('trajectory_file', default_value='__from_yaml__',
                      description='Optional override for lateral planner trajectory_file (default: YAML value)'),
        DeclareLaunchArgument('map_file', default_value=default_map,
                              description='Path to the map YAML file for map_server'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use /clock for simulation time'),
    ])

    use_teleop = LaunchConfiguration('use_teleop')
    use_lidar = LaunchConfiguration('use_lidar')
    mapping_mode = LaunchConfiguration('mapping_mode')

    # ══════════════════════
    #  Map Server (always)
    # ══════════════════════
    # Serves the static occupancy-grid map on /map with transient_local QoS.
    # Started early so the map is available before localization and scan_splitter.
    ld.add_action(LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='/',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': LaunchConfiguration('map_file'),
        }],
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }],
    ))

    # ══════════════════════
    #  VESC nodes (single process, zero-copy intra-process comms)
    # ══════════════════════
    ld.add_action(ComposableNodeContainer(
        name='vesc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='vesc_driver',
                plugin='vesc_driver::VescDriver',
                name='vesc_driver_node',
                parameters=[LaunchConfiguration('vesc_config')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='vesc_ackermann',
                plugin='vesc_ackermann::VescToOdom',
                name='vesc_to_odom_node',
                parameters=[LaunchConfiguration('vesc_config')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='vesc_ackermann',
                plugin='vesc_ackermann::AckermannToVesc',
                name='ackermann_to_vesc_node',
                parameters=[LaunchConfiguration('vesc_config')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    ))

    # ══════════════════════
    #  Joystick + Mux
    # ══════════════════════
    ld.add_action(Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')],
        condition=IfCondition(use_teleop),
    ))

    ld.add_action(Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')],
        condition=IfCondition(use_teleop),
    ))

    ld.add_action(Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        condition=IfCondition(use_teleop),
    ))

    # ══════════════════════
    #  LiDAR — Custom SCIP 2.0 driver
    # ══════════════════════
    # Normal mode: 270 beams @ 40 Hz (cluster=4, skip=0)
    ld.add_action(Node(
        package='f1tenth_lidar',
        executable='hokuyo_scip_driver_node',
        name='hokuyo_scip_driver',
        output='screen',
        parameters=[hokuyo_config, {'skip': 0}],
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and '", mapping_mode, "' != 'true'"
        ])),
    ))

    # Mapping mode: 1080 beams @ 40 Hz (cluster=1, skip=0 — full resolution)
    ld.add_action(Node(
        package='f1tenth_lidar',
        executable='hokuyo_scip_driver_node',
        name='hokuyo_scip_driver',
        output='screen',
        parameters=[hokuyo_config, {'skip': 0, 'cluster': 1}],
        condition=IfCondition(PythonExpression([
            "'", use_lidar, "' == 'true' and '", mapping_mode, "' == 'true'"
        ])),
    ))

    # ══════════════════════
    #  Scan Splitter — /scan → /scan_walls + /scan_obstacles
    # ══════════════════════
    # Only launched in racing mode (mapping_mode=false).
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_pkg_dir, 'launch', 'scan_splitter.launch.py')
        ),
        condition=UnlessCondition(mapping_mode),
    ))

    # ══════════════════════
    #  Lateral Planner — opponent avoidance → /local_raceline
    # ══════════════════════
    # Only launched in racing mode (mapping_mode=false).
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lateral_planner_pkg_dir, 'launch', 'lateral_planner.launch.py')
        ),
        launch_arguments={
            'trajectory_file': LaunchConfiguration('trajectory_file'),
        }.items(),
        condition=UnlessCondition(mapping_mode),
    ))

    # ══════════════════════
    #  Static transforms
    # ══════════════════════
    # ego_racecar/base_link → ego_racecar/laser
    # Update x, y, z to match your LiDAR mounting position.
    # Use update_vehicle_params.py in the workspace root to set all values.
    ld.add_action(Node(
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
    ))

    # ego_racecar/base_link → ego_racecar/imu
    # VESC firmware already compensates for upside-down mounting — identity rotation.
    ld.add_action(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_imu',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'ego_racecar/base_link',
            '--child-frame-id', 'ego_racecar/imu',
        ],
    ))

    return ld
