"""
@file udp_control_bridge_launch.py
@brief Launch description for the Jetson-side UDP control bridge.
@dependencies launch, launch_ros, state_publisher_udp package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'drive_topic',
            default_value='/drive',
            description='Topic to publish drive commands',
        ),
        DeclareLaunchArgument(
            'listen_port',
            default_value='49001',
            description='UDP listen port for control packets',
        ),
        DeclareLaunchArgument(
            'watchdog_timeout_ms',
            default_value='100.0',
            description='Watchdog timeout in milliseconds',
        ),
        Node(
            package='state_publisher_udp',
            executable='udp_control_bridge_node',
            name='udp_control_bridge',
            output='screen',
            parameters=[{
                'drive_topic': LaunchConfiguration('drive_topic'),
                'listen_port': LaunchConfiguration('listen_port'),
                'watchdog_timeout_ms': LaunchConfiguration('watchdog_timeout_ms'),
            }],
        ),
    ])
