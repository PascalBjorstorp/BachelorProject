# F1TENTH VESC-only Launch File
# ==============================
# Minimal launch for testing VESC communication only
# Does not launch LiDAR or joystick

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')

    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Path to VESC configuration file'
    )

    ld = LaunchDescription([vesc_la])

    # VESC Driver - communicates with VESC hardware
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )
    
    # VESC to Odom - computes odometry from VESC telemetry
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )
    
    # Ackermann to VESC - converts ackermann commands to VESC commands
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )

    ld.add_action(vesc_driver_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(ackermann_to_vesc_node)

    return ld
