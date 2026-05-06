1. 
nvidia-smi
nvcc --version

2. 
cd /home/pascal/Documents/BachelorProject
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select f1tenth_localization f1tenth_lidar f1tenth_lateral_planner f1tenth_planning
source install/setup.bash

3.
ros2 pkg executables f1tenth_localization | grep gpu_amcl_cpp_node


4.
./f1tenth_system/f1tenth_stack/scripts/replay_lateral_planner_bag.sh --rviz

Start rviz2