"""
Launch state publisher node for MPC communication.
Runs on Jetson to publish vehicle state + waypoint index.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Use params file from config directory
    params_file = PathJoinSubstitution([
        FindPackageShare('state_publisher'),
        'config',
        'params.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='state_publisher',
            executable='state_publisher_node',
            name='state_publisher',
            output='screen',
            parameters=[params_file],
        ),
    ])
