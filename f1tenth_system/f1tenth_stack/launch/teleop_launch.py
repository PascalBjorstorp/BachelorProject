# F1TENTH Teleop-only Launch File
# ================================
# Launch joystick teleop with VESC
# Useful for initial car testing and manual driving

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('f1tenth_stack')
    
    joy_teleop_config = os.path.join(pkg_share, 'config', 'joy_teleop.yaml')
    vesc_config = os.path.join(pkg_share, 'config', 'vesc.yaml')
    mux_config = os.path.join(pkg_share, 'config', 'mux.yaml')

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Path to joystick configuration file'
    )
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Path to VESC configuration file'
    )
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Path to ackermann_mux configuration file'
    )

    ld = LaunchDescription([joy_la, vesc_la, mux_la])

    # Joystick driver
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')],
        output='screen'
    )
    
    # Joystick to ackermann converter
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')],
        output='screen'
    )

    # Ackermann mux (prioritizes joystick over autonomous)
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')],
        output='screen'
    )

    # VESC nodes
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )
    
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )
    
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')],
        output='screen'
    )

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)

    return ld
