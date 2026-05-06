"""
@file kria_udp_receiver_launch.py
@brief Launch description for the Kria-side UDP receiver.
@dependencies launch, launch_ros, state_receiver_udp package
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'state_port',
            default_value='49000',
            description='UDP port to receive state packets on',
        ),
        DeclareLaunchArgument(
            'control_port',
            default_value='49001',
            description='UDP port to send control packets to',
        ),
        DeclareLaunchArgument(
            'xclbin_path',
            default_value='/lib/firmware/xilinx/kr260_mpc_app/mpc_fpga_top_opencl.xclbin',
            description='Path to FPGA xclbin',
        ),
        DeclareLaunchArgument(
            'kernel_name',
            default_value='mpc_fpga_top_opencl',
            description='OpenCL kernel name',
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='0',
            description='Xilinx device index',
        ),
        Node(
            package='state_receiver_udp',
            executable='kria_udp_receiver_node',
            name='kria_udp_receiver',
            output='screen',
            parameters=[{
                'state_port': LaunchConfiguration('state_port'),
                'control_port': LaunchConfiguration('control_port'),
                'xclbin_path': LaunchConfiguration('xclbin_path'),
                'kernel_name': LaunchConfiguration('kernel_name'),
                'device_index': LaunchConfiguration('device_index'),
            }],
        ),
    ])
