"""
GPU AMCL Launch File

Launches the GPU-accelerated AMCL node with map server.

Usage:
    # Default settings
    ros2 launch gpu_amcl gpu_amcl.launch.py
    
    # Custom particle count
    ros2 launch gpu_amcl gpu_amcl.launch.py num_particles:=1000
    
    # CPU mode (for debugging)
    ros2 launch gpu_amcl gpu_amcl.launch.py use_gpu:=false
    
    # With custom map
    ros2 launch gpu_amcl gpu_amcl.launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, LifecycleNode
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    gpu_amcl_dir = get_package_share_directory('gpu_amcl')
    
    # Config file path
    default_params_file = os.path.join(gpu_amcl_dir, 'config', 'amcl_params.yaml')
    
    # =========================================================================
    # Launch Arguments
    # =========================================================================
    
    declare_use_gpu = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Enable GPU acceleration'
    )
    
    declare_num_particles = DeclareLaunchArgument(
        'num_particles',
        default_value='2000',
        description='Number of particles'
    )
    
    declare_max_beams = DeclareLaunchArgument(
        'max_beams',
        default_value='60',
        description='Maximum laser beams to use'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to parameter file'
    )
    
    declare_map = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Path to map YAML file (optional, can subscribe to /map)'
    )
    
    declare_initial_pose_x = DeclareLaunchArgument(
        'initial_pose_x',
        default_value='0.0',
        description='Initial pose X coordinate'
    )
    
    declare_initial_pose_y = DeclareLaunchArgument(
        'initial_pose_y',
        default_value='0.0',
        description='Initial pose Y coordinate'
    )
    
    declare_initial_pose_a = DeclareLaunchArgument(
        'initial_pose_a',
        default_value='0.0',
        description='Initial pose yaw angle'
    )
    
    # =========================================================================
    # Info Message
    # =========================================================================
    
    info_msg = LogInfo(
        msg=['Starting GPU AMCL with ', LaunchConfiguration('num_particles'), ' particles']
    )
    
    # =========================================================================
    # GPU AMCL Node
    # =========================================================================
    
    gpu_amcl_node = Node(
        package='gpu_amcl',
        executable='gpu_amcl_node.py',
        name='gpu_amcl',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'use_gpu': LaunchConfiguration('use_gpu'),
                'num_particles': LaunchConfiguration('num_particles'),
                'max_beams': LaunchConfiguration('max_beams'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_a': LaunchConfiguration('initial_pose_a'),
            }
        ],
    )
    
    # =========================================================================
    # Map Server (optional - only if map path provided)
    # =========================================================================
    
    # Note: Map server is only needed if not subscribing to /map from elsewhere
    # For simulation, the gym_bridge already publishes /map
    
    # =========================================================================
    # Return Launch Description
    # =========================================================================
    
    return LaunchDescription([
        # Arguments
        declare_use_gpu,
        declare_num_particles,
        declare_max_beams,
        declare_params_file,
        declare_map,
        declare_initial_pose_x,
        declare_initial_pose_y,
        declare_initial_pose_a,
        
        # Info
        info_msg,
        
        # Nodes
        gpu_amcl_node,
    ])
