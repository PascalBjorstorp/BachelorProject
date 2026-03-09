#===========================================================================
# MPC FPGA (HLS) — CMake Build Configuration
#===========================================================================
#
# Project Structure:
#   include/           - FPGA MPC headers (HLS types, fixed-point math)
#   src/               - FPGA MPC core (HLS-compatible C sources)
#   sim/               - ROS2 integration for simulation (C-sim of FPGA)
#   testbench/         - Standalone tests
#
# Build Targets:
#   mpc_fpga_core      - Core FPGA MPC library (pure C, no ROS)
#   mpc_fpga_node      - ROS2 node for simulator integration
#
# The FPGA sources are compiled as regular C for software simulation.
# HLS pragmas are ignored by GCC (-Wno-unknown-pragmas).
#
#===========================================================================

cmake_minimum_required(VERSION 3.16)
project(mpc_fpga C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas -O2)
endif()

#===========================================================================
# MPC FPGA Core Library (Pure C, No ROS Dependencies)
#===========================================================================
# These are the HLS sources compiled as regular C for software simulation.
# HLS pragmas (#pragma HLS ...) are silently ignored by GCC.

set(MPC_FPGA_CORE_SOURCES
    src/fp_math_hls.c
    src/vehicle_model_hls.c
    src/riccati_solver_hls.c
    src/mpc_riccati_hls.c
    src/mpc_fpga_top.c
)

add_library(mpc_fpga_core STATIC ${MPC_FPGA_CORE_SOURCES})

target_include_directories(mpc_fpga_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mpc_fpga_core PUBLIC m)

#===========================================================================
# ROS2 Dependencies (optional — only when building as a ROS2 package)
#===========================================================================

find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    find_package(rclc REQUIRED)
    find_package(rcl REQUIRED)
    find_package(rcutils REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(ackermann_msgs REQUIRED)
    find_package(std_msgs REQUIRED)

    #===================================================================
    # ROS2 FPGA MPC Node
    #===================================================================

    add_executable(mpc_fpga_node sim/mpc_fpga_ros2_node.c)

    target_link_libraries(mpc_fpga_node
        mpc_fpga_core
        m
    )

    target_include_directories(mpc_fpga_node
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    ament_target_dependencies(mpc_fpga_node
        rclc
        rcl
        rcutils
        nav_msgs
        geometry_msgs
        ackermann_msgs
        std_msgs
    )

    install(TARGETS mpc_fpga_node
        DESTINATION lib/${PROJECT_NAME}
    )

    install(DIRECTORY include/
        DESTINATION include
    )

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
        install(DIRECTORY launch/
            DESTINATION share/${PROJECT_NAME}/launch
        )
    endif()

    ament_package()
endif()
