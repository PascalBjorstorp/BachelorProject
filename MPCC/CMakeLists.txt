#===========================================================================
# MPCC F1/10th — CMake Build Configuration
#===========================================================================
#
# Project Structure:
#   include/           - Public headers (FPGA-compatible)
#   src/               - Core MPCC library implementation
#   sim/               - ROS2 integration for simulation
#   test/              - Unit tests
#   config/            - Runtime parameters (YAML)
#
# Build Targets:
#   mpcc_core          - Core MPCC library (pure C, no ROS)
#   mpcc_node          - ROS2 node for simulator integration
#
# Dependencies:
#   MPC package        - Reuses fp_math and vehicle_model from ../MPC
#
#===========================================================================

cmake_minimum_required(VERSION 3.16)
project(mpcc_f1_10th C)

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

# Path to MPC package (for shared sources: fp_math, vehicle_model)
set(MPC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../MPC)

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
# MPCC Core Library (FPGA-Compatible, No ROS Dependencies)
#===========================================================================
#
# Pure C implementation of the MPCC controller and ADMM+Riccati solver.
# Can be compiled for JETSON or converted to HLS for FPGA.
#
# Modules:
#   mpcc.c                - Main MPCC controller (QP construction, cost)
#   mpcc_vehicle_model.c  - Dynamics linearization (Lifted ODE, linear tires)
#   qp_solver_mpcc.c      - ADMM + Riccati QP solver
#   fp_math_mpcc.c        - Q16.16 fixed-point arithmetic (self-contained)
#

set(MPCC_CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpcc_vehicle_model.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/qp_solver_mpcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/fp_math_mpcc.c
)

add_library(mpcc_core STATIC ${MPCC_CORE_SOURCES})

target_include_directories(mpcc_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${MPC_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# Enable debug prints (disable for FPGA/release builds)
target_compile_definitions(mpcc_core PRIVATE MPCC_DEBUG_PRINT)

#===========================================================================
# ROS2 MPCC Node (Simulation Integration)
#===========================================================================
#
# Wraps the MPCC core library in a ROS2 node for simulator testing.
# Subscribes to:
#   - /odom (nav_msgs/Odometry) — vehicle state
#   - /reference_path (custom) — racing line
# Publishes:
#   - /drive (ackermann_msgs/AckermannDriveStamped) — control commands
#   - /mpcc/predicted_path (nav_msgs/Path) — predicted trajectory
#

add_executable(mpcc_node sim/mpcc_ros2_node.c)

target_link_libraries(mpcc_node
    mpcc_core
    m  # Math library for atan2 in quaternion conversion
)

target_include_directories(mpcc_node
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${MPC_DIR}/include>
)

ament_target_dependencies(mpcc_node
    rclc
    rcl
    rcutils
    nav_msgs
    geometry_msgs
    ackermann_msgs
)

#===========================================================================
# Tests (optional, colcon test)
#===========================================================================
#
# Unit tests for MPCC components.
# Build with: colcon build --packages-select mpcc_f1_10th
# Run with:   colcon test --packages-select mpcc_f1_10th
#

if(BUILD_TESTING)
    add_executable(test_mpcc_solver test/test_admm_riccati.c)

    target_link_libraries(test_mpcc_solver
        mpcc_core
        m
    )

    target_include_directories(test_mpcc_solver
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
            $<BUILD_INTERFACE:${MPC_DIR}/include>
    )

    install(TARGETS test_mpcc_solver
        DESTINATION lib/${PROJECT_NAME}
    )
endif()

#===========================================================================
# Installation
#===========================================================================

install(TARGETS mpcc_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

#===========================================================================
# ROS2 Ament Package Export
#===========================================================================

ament_package()
