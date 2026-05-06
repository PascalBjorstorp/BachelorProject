"""
@file state_publisher_launch.py
@brief Launch description for Jetson-side MPC state publisher.
@details Starts state_publisher which subscribes to /local_raceline (nav_msgs/Path).
@dependencies launch, launch_ros, ament_index_python, state_publisher package
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    @brief Build launch description for the Jetson state publisher node.

    @return LaunchDescription with configured node action.
    """
    return LaunchDescription([
        Node(
            package='state_publisher',
            executable='state_publisher_node',
            name='state_publisher',
            output='screen',
        ),
    ])
