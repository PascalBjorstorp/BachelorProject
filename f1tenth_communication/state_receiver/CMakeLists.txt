cmake_minimum_required(VERSION 3.5)
project(state_receiver)

# C++17 standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(f1tenth_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# ===========================================================================
# MPC Riccati-ADMM — FPGA (mpc_receiver.cpp)
# ===========================================================================
add_executable(mpc_receiver_node src/mpc_receiver.cpp)
target_include_directories(mpc_receiver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../FPGA_Implementations/MPC_FPGA/include
)
ament_target_dependencies(mpc_receiver_node
  rclcpp
  f1tenth_msgs
  ackermann_msgs
)
install(TARGETS mpc_receiver_node DESTINATION lib/${PROJECT_NAME})

# ===========================================================================
# Install
# ===========================================================================
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
