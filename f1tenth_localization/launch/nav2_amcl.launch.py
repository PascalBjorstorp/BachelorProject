"""
Launch Nav2 AMCL for the F1TENTH localization comparison.

Run the hardware stack without localization first:
  ros2 launch f1tenth_stack System_no_localization.launch.py

Then launch this file in another terminal. The base stack provides /map,
/scan, /ego_racecar/odom, and odom->base_link TF. Nav2 AMCL publishes
/amcl_pose, /particle_cloud, and map->odom TF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('f1tenth_localization')
    params_file = os.path.join(pkg_dir, 'config', 'nav2_amcl_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=params_file,
            description='Path to the Nav2 AMCL YAML parameter file'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock for simulation time'),

        DeclareLaunchArgument(
            'min_particles',
            default_value='1000',
            description='Nav2 AMCL minimum particle count'),

        DeclareLaunchArgument(
            'max_particles',
            default_value='1000',
            description='Nav2 AMCL maximum particle count'),

        DeclareLaunchArgument(
            'max_beams',
            default_value='270',
            description='Number of scan beams used by Nav2 AMCL'),

        DeclareLaunchArgument(
            'update_min_d',
            default_value='0.0',
            description='Minimum translation before a filter update'),

        DeclareLaunchArgument(
            'update_min_a',
            default_value='0.0',
            description='Minimum rotation before a filter update'),

        LifecycleNode(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            namespace='/',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'use_sim_time': ParameterValue(
                        LaunchConfiguration('use_sim_time'), value_type=bool),
                    'min_particles': ParameterValue(
                        LaunchConfiguration('min_particles'), value_type=int),
                    'max_particles': ParameterValue(
                        LaunchConfiguration('max_particles'), value_type=int),
                    'max_beams': ParameterValue(
                        LaunchConfiguration('max_beams'), value_type=int),
                    'update_min_d': ParameterValue(
                        LaunchConfiguration('update_min_d'), value_type=float),
                    'update_min_a': ParameterValue(
                        LaunchConfiguration('update_min_a'), value_type=float),
                },
            ],
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2_amcl',
            output='screen',
            parameters=[{
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'autostart': True,
                'node_names': ['amcl'],
                'bond_timeout': 0.0,
            }],
        ),
    ])
