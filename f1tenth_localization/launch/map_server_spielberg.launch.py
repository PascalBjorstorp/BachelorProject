import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    # Get package share directory for portable path
    f1tenth_sim_pkg = get_package_share_directory('f1tenth_gym_ros')
    
    # Declare map name argument for flexibility
    declare_map_name = DeclareLaunchArgument(
        'map_name',
        default_value='Spielberg_map',
        description='Name of the map file (without extension)'
    )
    
    # Build map path from package share directory
    map_yaml_path = os.path.join(f1tenth_sim_pkg, 'maps', 'Spielberg_map.yaml')
    
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='/',
        output='screen',
        parameters=[
            {
                'yaml_filename': map_yaml_path,
                'frame_id': 'map',
                'topic_name': 'map',
                'use_sim_time': False  # Set to True if using sim time
            }
        ]
    )

    configure_map_server = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg="Configuring map_server (running ros2 lifecycle set /map_server configure)..."),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/map_server', 'configure'], output='screen')
        ]
    )

    activate_map_server = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg="Activating map_server (running ros2 lifecycle set /map_server activate)..."),
            ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'], output='screen')
        ]
    )

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription([
        map_server_node,
        configure_map_server,
        activate_map_server,
        static_tf_map_to_odom
    ])