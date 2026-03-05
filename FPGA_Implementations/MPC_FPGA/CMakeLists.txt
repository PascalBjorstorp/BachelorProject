#===========================================================================
# MPC FPGA - Standalone CMake Build (No ROS Dependencies)
#===========================================================================
#
# Build Targets:
#   mpc_fpga_core       - Core MPC library (pure C, FPGA-compatible)
#   test_qp_solver_fpga - QP solver test
#   test_mpc_accuracy   - MPC accuracy test
#   test_mpc_comprehensive - Comprehensive MPC test
#   test_mpc_raceline   - Raceline test
#   bench_mpc           - Benchmark
#   bench_mpc_timing    - Timing benchmark
#
# Usage:
#   cmake -S . -B build -DMPC_FPGA_BUILD_TESTS=ON
#   cmake --build build -j
#   ctest --test-dir build --output-on-failure
#
#===========================================================================

cmake_minimum_required(VERSION 3.16)
project(mpc_fpga_hls C)

# C99 standard for FPGA compatibility
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Compiler warnings
if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Options
option(MPC_FPGA_BUILD_TESTS "Build test executables" ON)
option(MPC_FPGA_HLS_PREPARE "Add MPC_HLS_TARGET define" OFF)

#===========================================================================
# MPC Core Library
#===========================================================================

set(MPC_CORE_SOURCES
    src/fp_math.c
    src/qp_solver.c
    src/vehicle_model.c
    src/mpc.c
)

add_library(mpc_fpga_core STATIC ${MPC_CORE_SOURCES})

target_include_directories(mpc_fpga_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(MPC_FPGA_HLS_PREPARE)
    target_compile_definitions(mpc_fpga_core PUBLIC MPC_HLS_TARGET)
endif()

# Link math library
target_link_libraries(mpc_fpga_core PUBLIC m)

#===========================================================================
# Tests
#===========================================================================

if(MPC_FPGA_BUILD_TESTS)
    enable_testing()

    # Test executables
    set(TEST_SOURCES
        test/test_qp_solver_fpga.c
        test/test_mpc_accuracy.c
        test/test_mpc_comprehensive.c
        test/test_mpc_raceline.c
    )

    foreach(test_src ${TEST_SOURCES})
        get_filename_component(test_name ${test_src} NAME_WE)
        add_executable(${test_name} ${test_src})
        target_link_libraries(${test_name} mpc_fpga_core m)
        add_test(NAME ${test_name} COMMAND ${test_name})
    endforeach()

    # Benchmarks
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/bench_mpc.c)
        add_executable(bench_mpc test/bench_mpc.c)
        target_link_libraries(bench_mpc mpc_fpga_core m)
    endif()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/bench_mpc_timing.c)
        add_executable(bench_mpc_timing test/bench_mpc_timing.c)
        target_link_libraries(bench_mpc_timing mpc_fpga_core m)
    endif()
endif()
