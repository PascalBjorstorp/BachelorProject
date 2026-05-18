cmake_minimum_required(VERSION 3.16)
project(arm_benchmark LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT MSVC)
    add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

add_executable(arm_benchmark
    test/benchmark_vehicle_model_arm.c
    src/vehicle_model.c
)

target_include_directories(arm_benchmark PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(arm_benchmark PRIVATE m)

find_package(OpenCL QUIET)

if(OpenCL_FOUND)
    add_executable(replay_fpga_opencl
        test/replay_fpga_opencl.cpp
    )

    target_link_libraries(replay_fpga_opencl PRIVATE
        OpenCL::OpenCL
    )
else()
    message(STATUS "OpenCL not found: skipping replay_fpga_opencl target")
endif()
