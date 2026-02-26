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
project(mpc_fpga C)

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

option(MPC_FPGA_BUILD_TESTS "Build MPC FPGA verification tests" ON)
option(MPC_FPGA_HLS_PREPARE "Compile with HLS-friendly options" ON)

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

set(MPC_FPGA_CORE_SOURCES
    src/fp_math.c
    src/qp_solver.c
    src/vehicle_model.c
    src/mpc.c
)

add_library(mpc_fpga_core STATIC ${MPC_FPGA_CORE_SOURCES})

target_include_directories(mpc_fpga_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(MPC_FPGA_HLS_PREPARE)
    target_compile_definitions(mpc_fpga_core PUBLIC MPC_HLS_TARGET)
endif()

if(MPC_FPGA_BUILD_TESTS)
    enable_testing()

    add_executable(test_mpc_raceline
        test/test_mpc_raceline.c)
    target_include_directories(test_mpc_raceline PRIVATE include)
    target_link_libraries(test_mpc_raceline mpc_fpga_core m)
    add_test(NAME test_mpc_raceline COMMAND test_mpc_raceline)

    add_executable(test_mpc_comprehensive
        test/test_mpc_comprehensive.c)
    target_include_directories(test_mpc_comprehensive PRIVATE include)
    target_link_libraries(test_mpc_comprehensive mpc_fpga_core m)
    add_test(NAME test_mpc_comprehensive COMMAND test_mpc_comprehensive)

    add_executable(test_qp_solver_fpga
        test/test_qp_solver_fpga.c)
    target_include_directories(test_qp_solver_fpga PRIVATE include)
    target_link_libraries(test_qp_solver_fpga mpc_fpga_core m)
    add_test(NAME test_qp_solver_fpga COMMAND test_qp_solver_fpga)
endif()

install(TARGETS mpc_fpga_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)

