# Copyright 2020 Hongrui Zheng
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""
F1TENTH Gym ROS2 Bridge Launch File

Usage:
  # Ground truth mode (default) - map -> base_link, no AMCL needed
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py ground_truth:=true
  
  # AMCL mode - odom -> base_link, requires AMCL for map -> odom
  ros2 launch f1tenth_gym_ros gym_bridge_launch.py ground_truth:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import yaml


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    # Get resolved ground_truth argument
    ground_truth_str = LaunchConfiguration('ground_truth').perform(context)
    ground_truth = ground_truth_str.lower() in ('true', '1', 'yes')
    
    nodes = []
    
    config = os.path.join(
        get_package_share_directory('f1tenth_gym_ros'),
        'config',
        'sim.yaml'
    )
    with open(config, 'r') as config_file:
        config_dict = yaml.safe_load(config_file)
    has_opp = config_dict['bridge']['ros__parameters']['num_agent'] > 1
    use_sim_time = config_dict['bridge']['ros__parameters']['use_sim_time']
    
    # Determine TF frames based on ground_truth setting
    if ground_truth:
        tf_frame_id = 'map'
        odom_frame_id = 'map'
        print('[gym_bridge] Running in GROUND TRUTH mode: map -> base_link (no AMCL needed)')
    else:
        tf_frame_id = 'ego_racecar/odom'
        odom_frame_id = 'ego_racecar/odom'
        print('[gym_bridge] Running in AMCL mode: ego_racecar/odom -> base_link (requires AMCL for map -> ego_racecar/odom)')

    bridge_node = Node(
        package='f1tenth_gym_ros',
        executable='gym_bridge',
        name='bridge',
        parameters=[config, {
            'use_sim_time': False,
            'use_sim_time_bridge': use_sim_time,
            'tf_frame_id': tf_frame_id,
            'odom_frame_id': odom_frame_id,
        }],
    )
    nodes.append(bridge_node)
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(
            get_package_share_directory('f1tenth_gym_ros'),
            'launch',
            'gym_bridge.rviz'
        )],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    nodes.append(rviz_node)

    # Get the map base name from config (e.g., 'Spielberg_map')
    map_base = os.path.basename(config_dict['bridge']['ros__parameters']['map_path'])
    map_dir = os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'maps')
    map_yaml_path = os.path.join(map_dir, map_base + '.yaml')
    map_image_path = os.path.join(map_dir, map_base + config_dict['bridge']['ros__parameters']['map_img_ext'])

    with open(map_yaml_path, 'r') as file:
        map_yaml = yaml.safe_load(file)
    map_yaml['resolution'] *= config_dict['bridge']['ros__parameters']['scale']
    origin = map_yaml['origin']
    scaled_origin = (
        origin[0] * config_dict['bridge']['ros__parameters']['scale'],
        origin[1] * config_dict['bridge']['ros__parameters']['scale'],
        origin[2],
    )
    map_yaml['origin'] = scaled_origin
    map_yaml['image'] = 'scaled_map' + config_dict['bridge']['ros__parameters']['map_img_ext']

    script_dir = os.path.dirname(os.path.abspath(__file__))
    temp_dir = os.path.join(script_dir, 'temp')
    os.makedirs(temp_dir, exist_ok=True)

    temp_yaml_path = os.path.join(temp_dir, 'scaled_map.yaml')
    temp_img_path = os.path.join(
        temp_dir,
        'scaled_map' + config_dict['bridge']['ros__parameters']['map_img_ext'])

    with open(temp_yaml_path, 'w') as file:
        yaml.dump(map_yaml, file)

    with open(map_image_path, 'rb') as img_file:
        with open(temp_img_path, 'wb') as file:
            file.write(img_file.read())

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{
            'yaml_filename': temp_yaml_path,
            'topic': 'map',
            'frame_id': 'map',
            'output': 'screen',
            'use_sim_time': use_sim_time
        }],
    )
    nodes.append(map_server_node)

    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server']
        }]
    )
    nodes.append(nav_lifecycle_node)

    # Determine vehicle xacro
    vehicle_params = config_dict['bridge']['ros__parameters']['vehicle_params']
    xacro_map = {
        'f1tenth': 'ego_racecar.xacro',
        'fullscale': 'ego_racecar_fullscale.xacro',
        'f1fifth': 'ego_racecar_f1fifth.xacro',
    }
    if vehicle_params not in xacro_map:
        raise ValueError(f'vehicle_params should be one of: {list(xacro_map.keys())}')
    ego_xacro = xacro_map[vehicle_params]

    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', ego_xacro)
            ]),
            'use_sim_time': use_sim_time,
        }],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    nodes.append(ego_robot_publisher)

    if has_opp:
        opp_robot_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='opp_robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    'xacro ',
                    os.path.join(get_package_share_directory('f1tenth_gym_ros'), 'launch', 'opp_racecar.xacro')
                ]),
                'use_sim_time': use_sim_time,
            }],
            remappings=[('/robot_description', 'opp_robot_description')]
        )
        nodes.append(opp_robot_publisher)

    return nodes


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'ground_truth',
            default_value='true',
            description='If true, sim publishes map->base_link (ground truth, no AMCL needed). '
                        'If false, sim publishes odom->base_link (requires AMCL for map->odom).'
        ),
        OpaqueFunction(function=launch_setup),
    ])
