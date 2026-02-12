"""
Launch MPC receiver node.
Runs on Ultra96 to receive state and prepare MPC reference.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = PathJoinSubstitution([
        FindPackageShare('mpc_receiver'),
        'config',
        'params.yaml'
    ])
    
    return LaunchDescription([
        Node(
            package='mpc_receiver',
            executable='mpc_receiver_node',
            name='mpc_receiver',
            output='screen',
            parameters=[params_file],
        ),
    ])
