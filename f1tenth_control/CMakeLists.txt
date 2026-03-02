cmake_minimum_required(VERSION 3.8)
project(f1tenth_control)

# Use C++17 for better performance and features
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler optimizations for Jetson Orin
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
  # Release optimizations
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG -march=native")
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
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

set(DEPENDENCIES
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  ackermann_msgs
  visualization_msgs
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
# FTG Node Executable
# ===================
add_executable(ftg_node
  src/nodes/ftg_node.cpp
)
target_link_libraries(ftg_node ftg_algorithm f1tenth_common)
ament_target_dependencies(ftg_node ${DEPENDENCIES})

# ===================
# Pure Pursuit Node Component
# ===================
add_library(pure_pursuit_node SHARED
  src/nodes/pure_pursuit_node.cpp
)
target_link_libraries(pure_pursuit_node pure_pursuit_algorithm)
ament_target_dependencies(pure_pursuit_node ${DEPENDENCIES})
rclcpp_components_register_nodes(pure_pursuit_node "f1tenth_control::PurePursuitNode")

# Pure Pursuit Node Executable (standalone)
add_executable(pure_pursuit_node_exe
  src/nodes/pure_pursuit_node.cpp
)
target_link_libraries(pure_pursuit_node_exe pure_pursuit_algorithm)
ament_target_dependencies(pure_pursuit_node_exe ${DEPENDENCIES})

# ===================
# Stanley Node Component
# ===================
add_library(stanley_node SHARED
  src/nodes/stanley_node.cpp
)
target_link_libraries(stanley_node stanley_algorithm)
ament_target_dependencies(stanley_node ${DEPENDENCIES})
rclcpp_components_register_nodes(stanley_node "f1tenth_control::StanleyNode")

# Stanley Node Executable (standalone)
add_executable(stanley_node_exe
  src/nodes/stanley_node.cpp
)
target_link_libraries(stanley_node_exe stanley_algorithm)
ament_target_dependencies(stanley_node_exe ${DEPENDENCIES})

# ===================
# Install targets
# ===================
install(
  TARGETS f1tenth_common ftg_algorithm pure_pursuit_algorithm stanley_algorithm pure_pursuit_node stanley_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS ftg_node pure_pursuit_node_exe stanley_node_exe
  DESTINATION lib/${PROJECT_NAME}
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
  # Disable strict linting checks that are too pedantic for this project
  # Only run essential linters: xmllint, cppcheck
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
endif()

# Export
ament_export_include_directories(include)
ament_export_libraries(f1tenth_common ftg_algorithm pure_pursuit_algorithm stanley_algorithm pure_pursuit_node stanley_node)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
