cmake_minimum_required(VERSION 3.16)
project(arm_benchmark C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

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
