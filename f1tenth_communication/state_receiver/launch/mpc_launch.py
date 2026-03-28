"""
@file mpc_launch.py
@brief Launch configuration for the FPGA-backed MPC receiver node.
@details Declares launch-time topic arguments and starts the state receiver
         node with those topic values passed as ROS parameters.
@dependencies launch, launch.actions.DeclareLaunchArgument,
              launch.substitutions.LaunchConfiguration,
              launch_ros.actions.Node
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    @brief Build launch description for FPGA-backed MPC receiver node.
    @param None.
    @return LaunchDescription with topic arguments and receiver node action.
    """
    drive_topic_arg = DeclareLaunchArgument(
        'drive_topic',
        default_value='/drive',
        description='Topic to publish drive commands'
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/mpc_state',
        description='Topic to receive MPC state from'
    )

    mpc_fpga_node = Node(
        package='state_receiver',
        executable='mpc_receiver_node',
        name='mpc_receiver',
        output='screen',
        parameters=[
            {
                'drive_topic': LaunchConfiguration('drive_topic'),
                'input_topic': LaunchConfiguration('input_topic'),
            }
        ],
    )

    return LaunchDescription([
        drive_topic_arg,
        input_topic_arg,
        mpc_fpga_node,
    ])
