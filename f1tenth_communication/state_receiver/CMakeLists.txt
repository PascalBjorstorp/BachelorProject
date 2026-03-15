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
# Pure Pursuit — FPGA (pp_receiver.cpp)
# ===========================================================================
set(PP_FPGA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../FPGA_Implementations/PP_FPGA/include")
if(EXISTS "${PP_FPGA_INCLUDE_DIR}/fpga_interface.h")
  message(STATUS "PP FPGA headers found — building pp_receiver_node")
  add_executable(pp_receiver_node src/pp_receiver.cpp)
  target_include_directories(pp_receiver_node PRIVATE ${PP_FPGA_INCLUDE_DIR})
  ament_target_dependencies(pp_receiver_node
    rclcpp
    f1tenth_msgs
    ackermann_msgs
  )
  install(TARGETS pp_receiver_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "PP FPGA headers NOT found at ${PP_FPGA_INCLUDE_DIR} — skipping pp_receiver_node")
endif()

# ===========================================================================
# MPC Riccati-ADMM — FPGA (mpc_receiver.cpp)
# ===========================================================================
set(MPC_FPGA_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../FPGA_Implementations/MPC_FPGA/include")
if(EXISTS "${MPC_FPGA_INCLUDE_DIR}/mpc_fpga_interface.h")
  message(STATUS "MPC FPGA headers found — building mpc_receiver_node")
  add_executable(mpc_receiver_node src/mpc_receiver.cpp)
  target_include_directories(mpc_receiver_node PRIVATE ${MPC_FPGA_INCLUDE_DIR})
  ament_target_dependencies(mpc_receiver_node
    rclcpp
    f1tenth_msgs
    ackermann_msgs
  )
  install(TARGETS mpc_receiver_node DESTINATION lib/${PROJECT_NAME})
else()
  message(STATUS "MPC FPGA headers NOT found at ${MPC_FPGA_INCLUDE_DIR} — skipping mpc_receiver_node")
endif()

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
