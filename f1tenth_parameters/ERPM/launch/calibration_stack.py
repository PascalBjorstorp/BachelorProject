#!/usr/bin/env python3
"""Dedicated, exclusive ERPM/current calibration stack.

No MPC, MPCC, planner, teleoperation or normal bringup is launched. Steering is
normal Ackermann steering at zero angle; it is assumed to have been calibrated
in the preceding steering campaign. AckermannToVesc motor outputs are remapped
through `motor_selector.py`, which is the only publisher to VESC motor topics.
"""
from __future__ import annotations
import argparse, os
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def _hardware(path: Path) -> dict:
    cfg = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    hw = cfg.get('hardware', {})
    required = ['lidar_ip_address','base_frame_id','laser_frame_id','imu_frame_id',
                'laser_to_base_x_m','laser_to_base_y_m','laser_to_base_z_m','laser_to_base_yaw_rad']
    missing=[x for x in required if x not in hw]
    if missing: raise ValueError(f'missing hardware fields: {missing}')
    return hw

def generate_description(hw: dict) -> LaunchDescription:
    root=Path(__file__).resolve().parents[1]
    stack_share=get_package_share_directory('f1tenth_stack')
    lidar_share=get_package_share_directory('f1tenth_lidar')
    vesc_config=os.path.join(stack_share,'config','vesc.yaml')
    mux_config=os.path.join(stack_share,'config','mux.yaml')
    lidar_config=os.path.join(lidar_share,'config','hokuyo_ust10lx.yaml')
    selector=ExecuteProcess(cmd=['python3',str(root/'erpm_calibration'/'motor_selector.py')],output='screen')
    return LaunchDescription([
      ComposableNodeContainer(name='erpm_calibration_vesc', namespace='', package='rclcpp_components', executable='component_container',
        composable_node_descriptions=[
          ComposableNode(package='vesc_driver', plugin='vesc_driver::VescDriver', name='vesc_driver_node', parameters=[vesc_config], extra_arguments=[{'use_intra_process_comms':True}]),
          ComposableNode(package='vesc_ackermann', plugin='vesc_ackermann::VescToOdom', name='vesc_to_odom_node', parameters=[vesc_config], extra_arguments=[{'use_intra_process_comms':True}]),
          ComposableNode(package='vesc_ackermann', plugin='vesc_ackermann::AckermannToVesc', name='ackermann_to_vesc_node', parameters=[vesc_config],
            remappings=[('/commands/motor/speed','/erpm_calibration/motor_from_ackermann/speed'),
                        ('/commands/motor/current','/erpm_calibration/motor_from_ackermann/current'),
                        ('/commands/motor/brake','/erpm_calibration/motor_from_ackermann/brake')],
            extra_arguments=[{'use_intra_process_comms':True}]),
        ], output='screen'),
      selector,
      Node(package='ackermann_mux', executable='ackermann_mux', name='ackermann_mux', parameters=[mux_config], output='screen'),
      Node(package='f1tenth_lidar', executable='hokuyo_scip_driver_node', name='hokuyo_scip_driver', parameters=[lidar_config,{'ip_address':str(hw['lidar_ip_address']),'skip':0}], output='screen'),
      Node(package='tf2_ros', executable='static_transform_publisher', name='static_baselink_to_laser',
        arguments=['--x',str(float(hw['laser_to_base_x_m'])),'--y',str(float(hw['laser_to_base_y_m'])),'--z',str(float(hw['laser_to_base_z_m'])),
          '--roll','0.0','--pitch','0.0','--yaw',str(float(hw['laser_to_base_yaw_rad'])),'--frame-id',str(hw['base_frame_id']),'--child-frame-id',str(hw['laser_frame_id'])], output='screen'),
      Node(package='tf2_ros', executable='static_transform_publisher', name='static_baselink_to_imu',
        arguments=['--x','0.0','--y','0.0','--z','0.0','--roll','0.0','--pitch','0.0','--yaw','0.0','--frame-id',str(hw['base_frame_id']),'--child-frame-id',str(hw['imu_frame_id'])], output='screen'),
    ])

def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument('--config',type=Path,required=True); a=p.parse_args()
    service=LaunchService(); service.include_launch_description(generate_description(_hardware(a.config.resolve()))); return service.run()
if __name__=='__main__': raise SystemExit(main())
