cmake_minimum_required(VERSION 3.8)
project(f1tenth_lateral_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(DEPENDENCIES
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Include directories
include_directories(include)

# ── LateralPlanner algorithm library ─────────────────────────────────────────
add_library(lateral_planner_lib SHARED
  src/lateral_planner.cpp
)
ament_target_dependencies(lateral_planner_lib ${DEPENDENCIES})

# ── LateralPlanner ROS2 node executable ──────────────────────────────────────
add_executable(lateral_planner_node
  src/lateral_planner_node.cpp
)
target_link_libraries(lateral_planner_node lateral_planner_lib)
ament_target_dependencies(lateral_planner_node ${DEPENDENCIES})

# ── Install targets ───────────────────────────────────────────────────────────
install(
  TARGETS lateral_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS lateral_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
