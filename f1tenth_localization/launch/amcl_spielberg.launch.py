

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
import yaml


def generate_launch_description():
    # Load sim config to get use_sim_time setting for consistency
    f1tenth_sim_pkg = get_package_share_directory('f1tenth_gym_ros')
    sim_config_path = os.path.join(f1tenth_sim_pkg, 'config', 'sim.yaml')
    
    use_sim_time = False  # Default
    try:
        with open(sim_config_path, 'r') as config_file:
            sim_config = yaml.safe_load(config_file)
            use_sim_time = sim_config['bridge']['ros__parameters'].get('use_sim_time', False)
    except Exception:
        pass  # Use default if config not found
    
    # Declare arguments
    declare_min_particles = DeclareLaunchArgument(
        'min_particles',
        default_value='500',
        description='Minimum number of particles'
    )
    
    declare_max_particles = DeclareLaunchArgument(
        'max_particles',
        default_value='2000',
        description='Maximum number of particles'
    )

    amcl_node = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='/',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                # Frame IDs - adapted for simulation where map→base_link is direct
                'base_frame_id': 'ego_racecar/base_link',
                'odom_frame_id': 'map',  # Use 'map' since sim doesn't have separate odom frame
                'global_frame_id': 'map',
                # Topics
                'scan_topic': 'scan',
                # Update thresholds
                'update_min_d': 0.1,
                'update_min_a': 0.2,
                'transform_tolerance': 1.0,
                # Particles
                'min_particles': LaunchConfiguration('min_particles'),
                'max_particles': LaunchConfiguration('max_particles'),
                # Laser model
                'laser_model_type': 'likelihood_field',
                'laser_likelihood_max_dist': 2.0,
                'laser_max_range': 10.0,
                'laser_min_range': 0.1,
                'max_beams': 60,
                # Motion model
                'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
                'alpha1': 0.2,
                'alpha2': 0.2,
                'alpha3': 0.2,
                'alpha4': 0.2,
                'alpha5': 0.1,
            }
        ]
    )

    # Use lifecycle manager instead of timer-based approach
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

    return LaunchDescription([
        declare_min_particles,
        declare_max_particles,
        amcl_node,
        amcl_lifecycle_manager,
    ])