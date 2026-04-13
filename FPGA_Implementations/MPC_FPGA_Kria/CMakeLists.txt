#===========================================================================
# MPC FPGA (HLS) — CMake Build Configuration
#===========================================================================
#
# Project Structure:
#   include/           - FPGA MPC headers (HLS types, fixed-point math)
#   src/               - FPGA MPC core (HLS-compatible C sources)
#   testbench/         - Standalone tests
#
# Build Targets:
#   mpc_fpga_core      - Core FPGA MPC library
#
# The FPGA sources are compiled as regular C for software simulation.
# HLS pragmas are ignored by GCC (-Wno-unknown-pragmas).
#
#===========================================================================

cmake_minimum_required(VERSION 3.16)
project(MPC_FPGA C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas -O2)
endif()

#===========================================================================
# MPC FPGA Core Library - HLS-Compatible C Sources
#===========================================================================
# These are the HLS sources compiled as regular C for software simulation.
# HLS pragmas (#pragma HLS ...) are silently ignored by GCC.

set(MPC_FPGA_CORE_SOURCES
    src/fp_math_hls.cpp
    src/vehicle_model_hls.cpp
    src/riccati_solver_hls.cpp
    src/mpc_riccati_hls.cpp
    src/mpc_fpga_top.cpp
)

add_library(mpc_fpga_core STATIC ${MPC_FPGA_CORE_SOURCES})

target_include_directories(mpc_fpga_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mpc_fpga_core PUBLIC m)

install(TARGETS mpc_fpga_core
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY include/
    DESTINATION include
)