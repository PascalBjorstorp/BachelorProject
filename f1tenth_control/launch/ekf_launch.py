"""
Launch file for EKF state estimation node.

The EKF fuses:
- Wheel odometry from VESC (velocity, angular velocity)
- IMU data (angular velocity from gyroscope)
- (Optional) MCL pose updates from LiDAR localization

Usage:
  ros2 launch f1tenth_control ekf_launch.py
  
  # With custom parameters:
  ros2 launch f1tenth_control ekf_launch.py publish_tf:=false predict_rate:=100.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Base link frame ID'
    )
    
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Whether to publish TF odom->base_link'
    )
    
    predict_rate_arg = DeclareLaunchArgument(
        'predict_rate',
        default_value='200.0',
        description='EKF prediction rate [Hz]'
    )
    
    # EKF Node
    ekf_node = Node(
        package='f1tenth_control',
        executable='ekf_node_exe',
        name='ekf_node',
        output='screen',
        parameters=[{
            'odom_frame': LaunchConfiguration('odom_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'publish_tf': LaunchConfiguration('publish_tf'),
            'predict_rate': LaunchConfiguration('predict_rate'),
            
            # Process noise (state uncertainty growth per second)
            'process_noise_x': 0.01,        # [m²/s²]
            'process_noise_y': 0.01,
            'process_noise_theta': 0.01,    # [rad²/s²]
            'process_noise_v': 0.1,         # [(m/s)²/s²]
            'process_noise_omega': 0.1,     # [(rad/s)²/s²]
            
            # Measurement noise - wheel odometry
            'odom_velocity_variance': 0.04,  # [m²/s²] - trust wheel encoder
            'odom_omega_variance': 0.01,     # [rad²/s²] - from steering+velocity
            
            # Measurement noise - IMU
            'imu_omega_variance': 0.001,     # [rad²/s²] - gyro is very accurate
            
            # Measurement noise - MCL/localization (when available)
            'mcl_x_variance': 0.05,          # [m²]
            'mcl_y_variance': 0.05,
            'mcl_theta_variance': 0.01,      # [rad²]
            
            # Vehicle parameters
            'wheelbase': 0.3302,
        }],
        remappings=[
            # Input topics
            ('/odom', '/vesc/odom'),          # Wheel odometry from VESC
            ('/imu', '/vesc/sensors/imu/raw'), # IMU from VESC
            ('/mcl_pose', '/mcl_pose'),       # MCL pose (when available)
            # Output topic
            ('/ekf_odom', '/ekf_odom'),
        ]
    )
    
    return LaunchDescription([
        odom_frame_arg,
        base_frame_arg,
        publish_tf_arg,
        predict_rate_arg,
        ekf_node,
    ])
