#===========================================================================
# MPC Riccati-ADMM — CMake Build Configuration
#===========================================================================
#
# Project Structure:
#   include/           - Public headers
#   src/               - Core MPC library (Riccati-ADMM solver)
#   sim/               - ROS2 integration for simulation
#   test/              - Tests
#
# Build Targets:
#   mpc_riccati_core   - Core MPC library (pure C, no ROS)
#   mpc_riccati_node   - ROS2 node for simulator integration
#   test_sim_drive     - Standalone simulation test
#
# Standalone test build (no ROS2 required):
#   cmake -S . -B build -DMPC_BUILD_TESTS=ON
#   cmake --build build -j
#   ./build/test_sim_drive
#
#===========================================================================

cmake_minimum_required(VERSION 3.16)
project(mpc_riccati C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

option(MPC_BUILD_TESTS "Build test executables" OFF)

#===========================================================================
# MPC Riccati-ADMM Core Library (Pure C, No ROS Dependencies)
#===========================================================================

set(MPC_CORE_SOURCES
    src/fp_math.c
    src/vehicle_model.c
    src/riccati_solver.c
    src/mpc_riccati.c
)

add_library(mpc_riccati_core STATIC ${MPC_CORE_SOURCES})

target_include_directories(mpc_riccati_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(mpc_riccati_core PUBLIC m)

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
    # ROS2 MPC Riccati Node
    #===================================================================

    add_executable(mpc_riccati_node sim/mpc_ros2_node.c)

    target_link_libraries(mpc_riccati_node
        mpc_riccati_core
        m
    )

    target_include_directories(mpc_riccati_node
        PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )

    ament_target_dependencies(mpc_riccati_node
        rclc
        rcl
        rcutils
        nav_msgs
        geometry_msgs
        ackermann_msgs
        std_msgs
    )

    install(TARGETS mpc_riccati_node
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

#===========================================================================
# Tests (standalone, no ROS2 needed)
#===========================================================================

if(MPC_BUILD_TESTS)
    enable_testing()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_drive.c)
        add_executable(test_sim_drive test/test_sim_drive.c)
        target_link_libraries(test_sim_drive mpc_riccati_core m)
        add_test(NAME test_sim_drive COMMAND test_sim_drive)
    endif()
endif()
