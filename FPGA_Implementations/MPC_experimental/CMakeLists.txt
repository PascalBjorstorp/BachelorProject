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
project(mpc_fpga_admm C)

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
# MPC Core Library (Original: projected gradient QP solver)
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
# MPC ADMM Library (Dense ADMM QP solver, drop-in for projected gradient)
#===========================================================================

set(MPC_ADMM_SOURCES
    src/fp_math.c
    src/qp_solver.c
    src/qp_solver_admm.c
    src/vehicle_model.c
    src/mpc.c
)

add_library(mpc_admm_core STATIC ${MPC_ADMM_SOURCES})

target_include_directories(mpc_admm_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# MPC_SOLVER_ADMM switches mpc.c to use admm_solve() instead of qp_solver_solve()
target_compile_definitions(mpc_admm_core PUBLIC MPC_SOLVER_ADMM)

if(MPC_FPGA_HLS_PREPARE)
    target_compile_definitions(mpc_admm_core PUBLIC MPC_HLS_TARGET)
endif()

target_link_libraries(mpc_admm_core PUBLIC m)

#===========================================================================
# MPC Riccati Library (Riccati-ADMM sparse solver, non-condensed formulation)
#===========================================================================

set(MPC_RICCATI_SOURCES
    src/fp_math.c
    src/vehicle_model.c
    src/riccati_solver.c
    src/mpc_riccati.c
    src/mpc.c
    src/qp_solver.c
)

add_library(mpc_riccati_core STATIC ${MPC_RICCATI_SOURCES})

target_include_directories(mpc_riccati_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

if(MPC_FPGA_HLS_PREPARE)
    target_compile_definitions(mpc_riccati_core PUBLIC MPC_HLS_TARGET)
endif()

target_link_libraries(mpc_riccati_core PUBLIC m)

#===========================================================================
# Tests
#===========================================================================

if(MPC_FPGA_BUILD_TESTS)
    enable_testing()

    # --- Original solver tests ---
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

    # --- ADMM solver test ---
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_admm_solver.c)
        add_executable(test_admm_solver test/test_admm_solver.c)
        target_link_libraries(test_admm_solver mpc_admm_core m)
        add_test(NAME test_admm_solver COMMAND test_admm_solver)
    endif()

    # --- Riccati solver test ---
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_riccati_solver.c)
        add_executable(test_riccati_solver test/test_riccati_solver.c)
        target_link_libraries(test_riccati_solver mpc_riccati_core m)
        add_test(NAME test_riccati_solver COMMAND test_riccati_solver)
    endif()

    # Benchmarks (original solver)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/bench_mpc.c)
        add_executable(bench_mpc test/bench_mpc.c)
        target_link_libraries(bench_mpc mpc_fpga_core m)
    endif()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/bench_mpc_timing.c)
        add_executable(bench_mpc_timing test/bench_mpc_timing.c)
        target_link_libraries(bench_mpc_timing mpc_fpga_core m)
    endif()

    # Comparison benchmark: Projected Gradient vs Riccati-ADMM
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/bench_comparison.c)
        add_executable(bench_comparison test/bench_comparison.c)
        target_link_libraries(bench_comparison mpc_riccati_core m)
    endif()

    # ADMM benchmark (uses MPC_SOLVER_ADMM flag)
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/bench_comparison_admm.c)
        add_executable(bench_comparison_admm test/bench_comparison_admm.c)
        target_link_libraries(bench_comparison_admm mpc_admm_core m)
    endif()

    # Riccati convergence diagnostic
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_riccati_diagnostic.c)
        add_executable(test_riccati_diagnostic test/test_riccati_diagnostic.c)
        target_link_libraries(test_riccati_diagnostic mpc_riccati_core m)
    endif()

    # N=20 curve convergence diagnostic
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_riccati_n20_diag.c)
        add_executable(test_riccati_n20_diag test/test_riccati_n20_diag.c)
        target_link_libraries(test_riccati_n20_diag mpc_riccati_core mpc_fpga_core m)
    endif()
endif()
