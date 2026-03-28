cmake_minimum_required(VERSION 3.8)
project(f1tenth_control)

# Use a consistent language standard for all package targets
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Enable warnings and release-only optimization flags for supported compilers
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  add_compile_options($<$<CONFIG:Release>:-O3;-DNDEBUG>)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(F1TENTH_PUBLIC_INCLUDE_DIRS
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ===================
# Common Library (Reusable components)
# ===================
add_library(f1tenth_common SHARED
  src/common/math_utils.cpp
  src/common/lidar_processor.cpp
)
ament_target_dependencies(f1tenth_common ${DEPENDENCIES})

# ===================
# FTG Algorithm Library
# ===================
add_library(ftg_algorithm SHARED
  src/algorithms/follow_the_gap.cpp
)
target_link_libraries(ftg_algorithm f1tenth_common)
ament_target_dependencies(ftg_algorithm ${DEPENDENCIES})

# ===================
# Pure Pursuit Algorithm Library
# ===================
add_library(pure_pursuit_algorithm SHARED
  src/algorithms/pure_pursuit.cpp
)
ament_target_dependencies(pure_pursuit_algorithm ${DEPENDENCIES})

# ===================
# Stanley Algorithm Library
# ===================
add_library(stanley_algorithm SHARED
  src/algorithms/stanley.cpp
)
ament_target_dependencies(stanley_algorithm ${DEPENDENCIES})

# ===================
# FTG Node Component (standardized runtime path)
# ===================
add_library(ftg_node_component SHARED
  src/nodes/ftg_node.cpp
)
target_link_libraries(ftg_node_component ftg_algorithm f1tenth_common)
ament_target_dependencies(ftg_node_component ${DEPENDENCIES})
rclcpp_components_register_nodes(ftg_node_component "f1tenth_control::FTGNode")

# ===================
# Pure Pursuit Node Component
# ===================
add_library(pure_pursuit_node SHARED
  src/nodes/pure_pursuit_node.cpp
)
target_link_libraries(pure_pursuit_node pure_pursuit_algorithm)
ament_target_dependencies(pure_pursuit_node ${DEPENDENCIES})
rclcpp_components_register_nodes(pure_pursuit_node "f1tenth_control::PurePursuitNode")

# ===================
# Stanley Node Component
# ===================
add_library(stanley_node SHARED
  src/nodes/stanley_node.cpp
)
target_link_libraries(stanley_node stanley_algorithm)
ament_target_dependencies(stanley_node ${DEPENDENCIES})
rclcpp_components_register_nodes(stanley_node "f1tenth_control::StanleyNode")

foreach(control_target
  f1tenth_common
  ftg_algorithm
  pure_pursuit_algorithm
  stanley_algorithm
  ftg_node_component
  pure_pursuit_node
  stanley_node
)
  target_include_directories(${control_target}
    PUBLIC
      ${F1TENTH_PUBLIC_INCLUDE_DIRS}
  )
endforeach()

# ===================
# Install targets
# ===================
install(
  TARGETS f1tenth_common ftg_algorithm pure_pursuit_algorithm stanley_algorithm pure_pursuit_node stanley_node ftg_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ===================
# Testing
# ===================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Keep lint checks focused on the subset currently used by this package.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_pep257_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Math utilities tests
  ament_add_gtest(test_math_utils test/test_math_utils.cpp)
  target_link_libraries(test_math_utils f1tenth_common)
  ament_target_dependencies(test_math_utils ${DEPENDENCIES})

  # LiDAR processor tests
  ament_add_gtest(test_lidar_processor test/test_lidar_processor.cpp)
  target_link_libraries(test_lidar_processor f1tenth_common)
  ament_target_dependencies(test_lidar_processor ${DEPENDENCIES})

  # Follow The Gap algorithm tests
  ament_add_gtest(test_follow_the_gap test/test_follow_the_gap.cpp)
  target_link_libraries(test_follow_the_gap ftg_algorithm f1tenth_common)
  ament_target_dependencies(test_follow_the_gap ${DEPENDENCIES})

  # Pure Pursuit closed-loop simulation sweep (standalone executable)
  add_executable(test_sim_drive_pure_pursuit test/test_sim_drive_pure_pursuit.cpp)
  target_link_libraries(test_sim_drive_pure_pursuit pure_pursuit_algorithm)
  ament_target_dependencies(test_sim_drive_pure_pursuit ${DEPENDENCIES})
endif()

# Export
ament_export_include_directories(include)
ament_export_libraries(f1tenth_common ftg_algorithm pure_pursuit_algorithm stanley_algorithm pure_pursuit_node stanley_node ftg_node_component)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
