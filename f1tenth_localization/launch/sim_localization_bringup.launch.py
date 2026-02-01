

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
import yaml


def generate_launch_description():
    # ========== PACKAGE DIRECTORIES ==========
    f1tenth_sim_pkg = get_package_share_directory('f1tenth_gym_ros')
    f1tenth_localization_pkg = get_package_share_directory('f1tenth_localization')
    f1tenth_control_pkg = get_package_share_directory('f1tenth_control')

    # ========== LOAD SIM CONFIG ==========
    sim_config_path = os.path.join(f1tenth_sim_pkg, 'config', 'sim.yaml')
    with open(sim_config_path, 'r') as config_file:
        sim_config = yaml.safe_load(config_file)
    
    use_sim_time = sim_config['bridge']['ros__parameters'].get('use_sim_time', False)
    
    # ========== LAUNCH ARGUMENTS ==========
    declare_use_amcl = DeclareLaunchArgument(
        'use_amcl',
        default_value='true',
        description='Whether to launch AMCL for localization'
    )

    declare_launch_control = DeclareLaunchArgument(
        'launch_control',
        default_value='false',
        description='Whether to launch the control algorithm (FTG)'
    )

    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz'
    )

    declare_min_particles = DeclareLaunchArgument(
        'min_particles',
        default_value='500',
        description='Minimum number of AMCL particles'
    )

    declare_max_particles = DeclareLaunchArgument(
        'max_particles',
        default_value='2000',
        description='Maximum number of AMCL particles'
    )

    declare_enable_perf_monitor = DeclareLaunchArgument(
        'enable_perf_monitor',
        default_value='false',
        description='Whether to launch the performance monitor'
    )

    declare_perf_output_dir = DeclareLaunchArgument(
        'perf_output_dir',
        default_value='/tmp/f1tenth_performance',
        description='Directory to save performance CSV files'
    )

    # ========== SIMULATION LAUNCH (includes map_server) ==========
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_sim_pkg, 'launch', 'gym_bridge_launch.py')
        )
    )

    # ========== AMCL NODE ==========
    # NOTE: The simulation publishes map→base_link directly (no odom frame).
    # For AMCL to work properly with this setup, we configure it to use 'map' as odom_frame
    # This is a simulation-specific workaround. On real hardware, you'd have proper odom.
    # 
    # In simulation, AMCL will essentially just monitor localization quality,
    # since the sim provides ground-truth pose. On real hardware, AMCL provides
    # the map→odom correction.
    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            # Frame IDs - adapted for simulation where map→base_link is direct
            'base_frame_id': 'ego_racecar/base_link',
            'odom_frame_id': 'map',  # Use 'map' since sim doesn't have separate odom frame
            'global_frame_id': 'map',
            # Topic configuration
            'scan_topic': 'scan',
            # Particle filter parameters
            'min_particles': LaunchConfiguration('min_particles'),
            'max_particles': LaunchConfiguration('max_particles'),
            # Update thresholds - increased to reduce CPU load
            'update_min_d': 0.1,       # Min distance (m) before filter update
            'update_min_a': 0.2,       # Min rotation (rad) before filter update
            # Transform tolerance - handle timestamp jitter
            'transform_tolerance': 1.0,
            'tf_broadcast': True,
            # Recovery behavior
            'recovery_alpha_slow': 0.001,
            'recovery_alpha_fast': 0.1,
            # Laser model parameters
            'laser_model_type': 'likelihood_field',
            'laser_likelihood_max_dist': 2.0,
            'laser_max_range': 10.0,
            'laser_min_range': 0.1,
            'max_beams': 60,  # Reduced from 1080 to help AMCL keep up
            # Motion model (diff-corrected for ackermann-like)
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'alpha1': 0.2,
            'alpha2': 0.2,
            'alpha3': 0.2,
            'alpha4': 0.2,
            'alpha5': 0.1,
            # Initial pose
            'set_initial_pose': True,
            'initial_pose': {
                'x': sim_config['bridge']['ros__parameters'].get('sx', 0.0),
                'y': sim_config['bridge']['ros__parameters'].get('sy', 0.0),
                'z': 0.0,
                'yaw': sim_config['bridge']['ros__parameters'].get('stheta', 0.0),
            },
        }]
    )

    # Lifecycle manager for AMCL
    amcl_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['amcl'],
            'bond_timeout': 0.0,
        }]
    )

    # Delay AMCL startup to let simulation establish TF tree first
    delayed_amcl = TimerAction(
        period=3.0,
        actions=[amcl_node],
    )

    delayed_lifecycle_manager = TimerAction(
        period=5.0,  # Start after AMCL has time to initialize
        actions=[amcl_lifecycle_manager],
    )

    # ========== CONTROL LAUNCH (OPTIONAL) ==========
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_control_pkg, 'launch', 'ftg_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('launch_control'))
    )

    # ========== PERFORMANCE MONITOR (OPTIONAL) ==========
    perf_monitor_node = Node(
        package='f1tenth_localization',
        executable='performance_monitor.py',
        name='performance_monitor',
        output='screen',
        parameters=[{
            'output_dir': LaunchConfiguration('perf_output_dir'),
            'sample_rate_hz': 10.0,
            'scan_topic': '/scan',
            'amcl_pose_topic': '/amcl_pose',
        }],
        condition=IfCondition(LaunchConfiguration('enable_perf_monitor'))
    )

    # ========== LAUNCH DESCRIPTION ==========
    return LaunchDescription([
        # Declare arguments
        declare_use_amcl,
        declare_launch_control,
        declare_use_rviz,
        declare_min_particles,
        declare_max_particles,
        declare_enable_perf_monitor,
        declare_perf_output_dir,
        
        # Launch simulation (includes map_server, rviz, bridge)
        sim_launch,
        
        # Launch AMCL with delay (only if use_amcl:=true)
        # The TimerActions handle the delay, conditions are checked via the arguments
        delayed_amcl,
        delayed_lifecycle_manager,
        
        # Optionally launch control
        control_launch,
        
        # Optionally launch performance monitor
        perf_monitor_node,
    ])
