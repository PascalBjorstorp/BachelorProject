cmake_minimum_required(VERSION 3.5)
project(teleop_axis_merging)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_executable(teleop_axis_merging_node src/teleop_axis_merging_node.cpp)
ament_target_dependencies(teleop_axis_merging_node rclcpp sensor_msgs)

install(TARGETS
  teleop_axis_merging_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()