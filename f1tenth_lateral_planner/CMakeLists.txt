cmake_minimum_required(VERSION 3.8)
project(f1tenth_lateral_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ─────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)

set(DEPS
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2
)

# ── Library: lateral_planner core logic ──────────────────────────────
add_library(lateral_planner_lib
  src/lateral_planner.cpp
)
target_include_directories(lateral_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(lateral_planner_lib ${DEPS})

# ── Executable: ROS2 node ────────────────────────────────────────────
add_executable(lateral_planner_node
  src/lateral_planner_node.cpp
)
target_link_libraries(lateral_planner_node lateral_planner_lib)
ament_target_dependencies(lateral_planner_node ${DEPS})

# ── Install ──────────────────────────────────────────────────────────
install(TARGETS lateral_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS lateral_planner_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
