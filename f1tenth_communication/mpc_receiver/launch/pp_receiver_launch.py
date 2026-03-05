"""
Launch pp receiver node.
Runs on Ultra96 to receive state and prepare pp reference.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('pp_receiver'),
        'config',
        'params.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='pp_receiver',
            executable='pp_receiver_node',
            name='pp_receiver',
            output='screen',
            parameters=[params_file],
        ),
    ])
