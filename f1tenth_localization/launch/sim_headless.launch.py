"""
F1TENTH Simulation Headless Launch (for Jetson)

Runs the F1TENTH gym simulation without RViz.
Use this on headless systems like Jetson.

Features:
- Ground truth mode: map -> base_link (no AMCL needed)
- 40Hz scan rate to match real hardware
- No RViz (headless)

Usage:
    ros2 launch f1tenth_localization sim_headless.launch.py
    
    # With AMCL mode (requires AMCL to be running):
    ros2 launch f1tenth_localization sim_headless.launch.py ground_truth:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    # Get resolved ground_truth argument
    ground_truth_str = LaunchConfiguration('ground_truth').perform(context)
    ground_truth = ground_truth_str.lower() in ('true', '1', 'yes')
    
    # Get package directories
    sim_pkg = get_package_share_directory('f1tenth_gym_ros')
    
    # Load config
    config_path = os.path.join(sim_pkg, 'config', 'sim.yaml')
    with open(config_path, 'r') as f:
        config_dict = yaml.safe_load(f)
    
    use_sim_time = config_dict['bridge']['ros__parameters'].get('use_sim_time', False)
    
    # Map paths
    map_base = os.path.basename(config_dict['bridge']['ros__parameters']['map_path'])
    map_yaml_path = os.path.join(sim_pkg, 'maps', map_base + '.yaml')
    
    # Determine TF frames based on ground_truth setting
    if ground_truth:
        tf_frame_id = 'map'
        odom_frame_id = 'map'
        mode_msg = 'GROUND TRUTH mode: map -> base_link (no AMCL needed)'
    else:
        tf_frame_id = 'odom'
        odom_frame_id = 'odom'
        mode_msg = 'AMCL mode: odom -> base_link (requires AMCL for map -> odom)'
    
    # Info
    info_msg = LogInfo(msg=f'Starting F1TENTH simulation (headless) - {mode_msg}')
    
    # Simulation bridge node with 40Hz scan rate, ground truth TF, and headless mode
    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        output='screen',
        parameters=[
            config_path,
            {
                'use_sim_time': False,
                'use_sim_time_bridge': use_sim_time,
                'tf_frame_id': tf_frame_id,
                'odom_frame_id': odom_frame_id,
                'scan_publish_rate': 40.0,  # Match real hardware (40Hz LiDAR)
                'headless': True,  # Disable rendering (no PyQt6 needed)
            }
        ]
    )
    
    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_yaml_path,
            'topic': 'map',
            'frame_id': 'map',
            'use_sim_time': use_sim_time,
        }]
    )
    
    # Map server lifecycle manager
    map_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server'],
            'bond_timeout': 0.0,
        }]
    )
    
    # Robot state publisher
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(
                os.path.join(sim_pkg, 'launch', 'ego_racecar.urdf')
            ).read() if os.path.exists(os.path.join(sim_pkg, 'launch', 'ego_racecar.urdf')) else '',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    
    return [
        info_msg,
        bridge_node,
        map_server_node,
        map_lifecycle,
        # robot_state_pub,  # Enable if you have the URDF
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'ground_truth',
            default_value='true',
            description='Use ground truth pose (map->base_link) instead of odom frame'
        ),
        OpaqueFunction(function=launch_setup)
    ])
