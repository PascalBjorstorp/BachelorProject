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

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo, OpaqueFunction
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
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
    
    # Always ground truth: map -> base_link (no AMCL needed)
    tf_frame_id = 'map'
    odom_frame_id = 'map'
    
    info_msg = LogInfo(msg='Starting F1TENTH simulation (headless) - GROUND TRUTH mode')
    
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
    
    return [
        info_msg,
        bridge_node,
        map_server_node,
        map_lifecycle,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
