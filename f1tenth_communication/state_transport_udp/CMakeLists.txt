cmake_minimum_required(VERSION 3.5)
project(state_transport_udp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

set(FPGA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../FPGA_Implementations/MPC_FPGA_Ultra96/include)

add_executable(ros2_udp_sender src/ros2_udp_sender.cpp)
target_include_directories(ros2_udp_sender PRIVATE
  include
  ${FPGA_INCLUDE_DIR}
)
ament_target_dependencies(ros2_udp_sender
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(udp_control_bridge src/udp_control_bridge.cpp)
target_include_directories(udp_control_bridge PRIVATE
  include
  ${FPGA_INCLUDE_DIR}
)
ament_target_dependencies(udp_control_bridge
  rclcpp
  ackermann_msgs
)

add_executable(ultra96_udp_receiver src/ultra96_udp_receiver.cpp)
target_include_directories(ultra96_udp_receiver PRIVATE
  include
  ${FPGA_INCLUDE_DIR}
)

install(TARGETS
  ros2_udp_sender
  udp_control_bridge
  ultra96_udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config include
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
