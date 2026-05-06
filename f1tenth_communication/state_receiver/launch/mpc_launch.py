from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'debug_gdb',
            default_value='false',
            description='If true, run node under gdb (no xterm)'
        ),
        DeclareLaunchArgument(
            'drive_topic',
            default_value='/drive',
            description='Topic to publish drive commands'
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value='/mpc_state',
            description='Topic to receive MPC state from'
        ),
        DeclareLaunchArgument(
            'xclbin_path',
            default_value='/lib/firmware/xilinx/MPC_FPGA/mpc_fpga_top_opencl.xclbin',
            description='Path to xclbin on KR260'
        ),
        DeclareLaunchArgument(
            'kernel_name',
            default_value='mpc_fpga_top_opencl',
            description='OpenCL kernel name'
        ),
        DeclareLaunchArgument(
            'device_index',
            default_value='0',
            description='Xilinx device index'
        ),
        Node(
            condition=UnlessCondition(LaunchConfiguration('debug_gdb')),
            package='state_receiver',
            executable='mpc_receiver_node',
            name='mpc_receiver',
            output='screen',
            parameters=[{
                'drive_topic': LaunchConfiguration('drive_topic'),
                'input_topic': LaunchConfiguration('input_topic'),
                'xclbin_path': LaunchConfiguration('xclbin_path'),
                'kernel_name': LaunchConfiguration('kernel_name'),
                'device_index': LaunchConfiguration('device_index'),
            }],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration('debug_gdb')),
            package='state_receiver',
            executable='mpc_receiver_node',
            name='mpc_receiver',
            output='screen',
            prefix=['gdb', '-ex', 'run', '--args'],
            parameters=[{
                'drive_topic': LaunchConfiguration('drive_topic'),
                'input_topic': LaunchConfiguration('input_topic'),
                'xclbin_path': LaunchConfiguration('xclbin_path'),
                'kernel_name': LaunchConfiguration('kernel_name'),
                'device_index': LaunchConfiguration('device_index'),
            }],
        ),
    ])
