"""
AMCL Bag Visualization Launch File

This launch file plays back a recorded AMCL output bag and visualizes it in RViz2.
Shows: Map, LiDAR scan, AMCL pose, particle cloud, ground truth odometry, and TF.

Usage:
  # Play bag and open RViz
  ros2 launch f1tenth_localization amcl_bag_visualization.launch.py \
    bag_path:=/path/to/bag

  # With custom playback rate
  ros2 launch f1tenth_localization amcl_bag_visualization.launch.py \
    bag_path:=/path/to/bag playback_rate:=0.5

  # Loop playback
  ros2 launch f1tenth_localization amcl_bag_visualization.launch.py \
    bag_path:=/path/to/bag loop:=true

Example:
  ros2 launch f1tenth_localization amcl_bag_visualization.launch.py \
    bag_path:=/home/pascal/Documents/BachelorProject/benchmark_results/amcl_output_gpu_amcl_p500-3000_b270_20260211_221758
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction, 
    LogInfo, OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    """Setup function called at launch time with resolved arguments."""
    # Get resolved arguments
    bag_path = LaunchConfiguration('bag_path').perform(context)
    playback_rate = LaunchConfiguration('playback_rate').perform(context)
    loop = LaunchConfiguration('loop').perform(context).lower() == 'true'
    
    # Expand ~ in bag path
    bag_path = os.path.expanduser(bag_path)
    
    # Validate bag path exists
    if not os.path.exists(bag_path):
        raise RuntimeError(f"Bag path does not exist: {bag_path}")
    
    # Get RViz config path
    localization_pkg = get_package_share_directory('f1tenth_localization')
    rviz_config = os.path.join(localization_pkg, 'launch', 'amcl_bag_visualization.rviz')
    
    # Fallback to source path if installed path doesn't exist
    if not os.path.exists(rviz_config):
        source_path = os.path.dirname(os.path.abspath(__file__))
        rviz_config = os.path.join(source_path, 'amcl_bag_visualization.rviz')
    
    # Build bag play command
    bag_play_cmd = ['ros2', 'bag', 'play', bag_path, '--clock', '--rate', playback_rate]
    if loop:
        bag_play_cmd.append('--loop')
    
    # Bag player
    bag_player = ExecuteProcess(
        cmd=bag_play_cmd,
        output='screen',
        name='bag_player'
    )
    
    # RViz2 with custom config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
    )
    
    # Start RViz first, then bag player after a short delay
    return [
        LogInfo(msg=f'Launching AMCL bag visualization...'),
        LogInfo(msg=f'Bag: {bag_path}'),
        LogInfo(msg=f'Rate: {playback_rate}x, Loop: {loop}'),
        rviz_node,
        TimerAction(
            period=2.0,  # Wait 2s for RViz to initialize
            actions=[
                LogInfo(msg='Starting bag playback...'),
                bag_player,
            ]
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'bag_path',
            description='Path to the ROS 2 bag to play back'
        ),
        DeclareLaunchArgument(
            'playback_rate',
            default_value='1.0',
            description='Playback rate multiplier (0.5 = half speed, 2.0 = double speed)'
        ),
        DeclareLaunchArgument(
            'loop',
            default_value='false',
            description='Loop bag playback continuously'
        ),
        
        # Setup function
        OpaqueFunction(function=launch_setup),
    ])
