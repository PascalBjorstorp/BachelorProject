"""
@file ros2_udp_sender_launch.py
@brief Launch description for the Jetson-side UDP sender.
@dependencies launch, launch_ros, ament_index_python, state_publisher_udp package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/ego_racecar/odom',
            description='Odometry topic',
        ),
        DeclareLaunchArgument(
            'pose_topic',
            default_value='/ekf_pose',
            description='Pose topic',
        ),
        DeclareLaunchArgument(
            'raceline_topic',
            default_value='/local_raceline',
            description='Local raceline path topic',
        ),
        DeclareLaunchArgument(
            'servo_topic',
            default_value='/sensors/servo_position_command',
            description='Servo feedback topic',
        ),
        DeclareLaunchArgument(
            'dest_ip',
            default_value='10.23.0.120',
            description='UDP destination IP',
        ),
        DeclareLaunchArgument(
            'dest_port',
            default_value='49000',
            description='UDP destination port',
        ),
        Node(
            package='state_publisher_udp',
            executable='ros2_udp_sender_node',
            name='ros2_udp_sender',
            output='screen',
            parameters=[{
                'odom_topic': LaunchConfiguration('odom_topic'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'raceline_topic': LaunchConfiguration('raceline_topic'),
                'servo_topic': LaunchConfiguration('servo_topic'),
                'dest_ip': LaunchConfiguration('dest_ip'),
                'dest_port': LaunchConfiguration('dest_port'),
            }],
        ),
    ])
