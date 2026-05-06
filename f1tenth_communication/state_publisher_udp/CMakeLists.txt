cmake_minimum_required(VERSION 3.8)
project(state_publisher_udp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

add_executable(ros2_udp_sender_node src/ros2_udp_sender.cpp)
target_include_directories(ros2_udp_sender_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(ros2_udp_sender_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(udp_control_bridge_node src/udp_control_bridge.cpp)
target_include_directories(udp_control_bridge_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(udp_control_bridge_node
  rclcpp
  ackermann_msgs
)

install(TARGETS
  ros2_udp_sender_node
  udp_control_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
