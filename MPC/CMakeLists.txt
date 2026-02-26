#===========================================================================
# MPC F1/10th - CMake Build Configuration
#===========================================================================
#
# Project Structure:
#   include/           - Public headers (FPGA-compatible)
#   src/               - Core MPC library implementation
#   sim/               - ROS2 integration for simulation
#   tests/             - Unit tests
#
# Build Targets:
#   mpc_core           - Core MPC library (pure C, no ROS)
#   mpc_node           - ROS2 node for simulator integration
#   test_*             - Unit tests
#
#===========================================================================

cmake_minimum_required(VERSION 3.16)
project(mpc_f1_10th C)

#===========================================================================
# Build Configuration
#===========================================================================

# C99 standard for FPGA compatibility (no C++ features)
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler warnings for code quality
if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#===========================================================================
# ROS2 Dependencies (for simulation only)
#===========================================================================

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(rcl REQUIRED)
find_package(rcutils REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

#===========================================================================
# MPC Core Library (FPGA-Compatible, No ROS Dependencies)
#===========================================================================
#
# This library contains all MPC functionality in pure C.
# It can be compiled for JETSON or converted to HSL for FPGA.
#
# Modules:
#   fp_math.c          - Q16.16 fixed-point arithmetic and linear algebra
#   qp_solver.c        - Quadratic programming solver
#   vehicle_model.c    - Kinematic bicycle model
#   mpc.c              - Main MPC controller
#

set(MPC_CORE_SOURCES
    src/fp_math.c
    src/qp_solver.c
    src/vehicle_model.c
    src/mpc.c
)

add_library(mpc_core STATIC ${MPC_CORE_SOURCES})

target_include_directories(mpc_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

#===========================================================================
# ROS2 MPC Node (Simulation Integration)
#===========================================================================
#
# Wraps the MPC core library in a ROS2 node for simulator testing.
# Subscribes to odometry, publishes control commands.
#

add_executable(mpc_node sim/mpc_ros2_node.c)

target_compile_definitions(mpc_core PRIVATE MPC_DEBUG_PRINT)

target_link_libraries(mpc_node
    mpc_core
    m  # Math library for atan2 in quaternion conversion
)

target_include_directories(mpc_node
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(mpc_node
    rclc
    rcl
    rcutils
    nav_msgs
    geometry_msgs
    ackermann_msgs
)

#===========================================================================
# Installation
#===========================================================================

install(TARGETS mpc_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

#===========================================================================
# ROS2 Ament Package Export
#===========================================================================

ament_package()

