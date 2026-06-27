cmake_minimum_required(VERSION 3.8)
project(f1tenth_lidar)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3 -march=native)
endif()

# ── Dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

# ── C++ Scan Splitter Node ───────────────────────────────────────────
add_executable(scan_splitter_node
  src/scan_splitter_node.cpp
)
target_include_directories(scan_splitter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/config>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(scan_splitter_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  tf2
  tf2_ros
)

# ── C++ Hokuyo SCIP 2.0 Driver Node ─────────────────────────────────
add_executable(hokuyo_scip_driver_node
  src/hokuyo_scip_driver.cpp
)
target_include_directories(hokuyo_scip_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hokuyo_scip_driver_node
  rclcpp
  sensor_msgs
)

# ── Scan downsampler (full-res /scan_full -> reduced /scan for the pipeline) ──
add_executable(scan_downsampler_node
  src/scan_downsampler_node.cpp
)
ament_target_dependencies(scan_downsampler_node
  rclcpp
  sensor_msgs
)

install(TARGETS scan_splitter_node hokuyo_scip_driver_node scan_downsampler_node
  DESTINATION lib/${PROJECT_NAME}
)

# ── Python diagnostic script ──────────────────────────────────────────
install(PROGRAMS
  src/hokuyo_lidar_diagnostic.py
  DESTINATION lib/${PROJECT_NAME}
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
