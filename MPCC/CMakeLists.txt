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

option(MPCC_ENABLE_DEBUG_PRINTS "Enable verbose MPCC debug prints" OFF)
option(MPCC_USE_OSQP "Build MPCC with the OSQP backend instead of the in-tree ADMM solver" OFF)

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
find_package(std_msgs REQUIRED)
if(MPCC_USE_OSQP)
    find_package(osqp REQUIRED)

    find_path(OSQP_INCLUDE_ROOT
        NAMES osqp/osqp.h
        PATHS
            /opt/ros/jazzy/include
            /usr/local/include
            /usr/include
    )
    if(NOT OSQP_INCLUDE_ROOT)
        message(FATAL_ERROR "Could not find osqp/osqp.h. Install/source osqp_vendor or OSQP.")
    endif()
endif()

#===========================================================================
# MPCC Core Library (FPGA-Compatible, No ROS Dependencies)
#===========================================================================
#
# Pure C implementation of the MPCC controller. The default build uses the
# in-tree ADMM+Riccati solver. OSQP remains available as an explicit opt-in
# backend for comparison or experimentation.
#
# Modules:
#   mpcc.c                - Main MPCC controller (QP construction, cost)
#   mpcc_vehicle_model.c  - Dynamics linearization (Lifted ODE, linear tires)
#   qp_solver_mpcc.c      - In-tree ADMM+Riccati solver
#   qp_solver_osqp.c      - Optional OSQP solver bridge
#

if(MPCC_USE_OSQP)
    set(MPCC_SOLVER_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/qp_solver_osqp.c)
else()
    set(MPCC_SOLVER_SOURCE ${CMAKE_CURRENT_SOURCE_DIR}/src/qp_solver_mpcc.c)
endif()

set(MPCC_CORE_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpcc.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/mpcc_vehicle_model.c
    ${MPCC_SOLVER_SOURCE}
)

add_library(mpcc_core STATIC ${MPCC_CORE_SOURCES})

target_include_directories(mpcc_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${MPC_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(MPCC_USE_OSQP)
    target_include_directories(mpcc_core
        PUBLIC
            $<BUILD_INTERFACE:${OSQP_INCLUDE_ROOT}>
    )

    target_compile_definitions(mpcc_core
        PUBLIC
            USE_OSQP
    )
endif()

if(MPCC_ENABLE_DEBUG_PRINTS)
    target_compile_definitions(mpcc_core PRIVATE MPCC_DEBUG_PRINT)
endif()

if(MPCC_USE_OSQP)
    target_link_libraries(mpcc_core
        PUBLIC
            osqp::osqp
    )
endif()

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

add_executable(mpcc_hardware_node src/mpcc_hardware_node.c)

target_link_libraries(mpcc_node
    mpcc_core
    m  # Math library for atan2 in quaternion conversion
)

target_link_libraries(mpcc_hardware_node
    mpcc_core
    m
)

target_include_directories(mpcc_node
    PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${MPC_DIR}/include>
)

target_include_directories(mpcc_hardware_node
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
    std_msgs
)

ament_target_dependencies(mpcc_hardware_node
    rclc
    rcl
    rcutils
    nav_msgs
    geometry_msgs
    ackermann_msgs
    std_msgs
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
    add_executable(test_mpcc_solver
        test/test_admm_riccati.c
    )

    if(MPCC_USE_OSQP)
        target_sources(test_mpcc_solver PRIVATE src/qp_solver_mpcc.c)
    endif()

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

install(TARGETS mpcc_node mpcc_hardware_node
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
