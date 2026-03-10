cmake_minimum_required(VERSION 3.16)
project(mpc_riccati C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT MSVC)
    add_compile_options(-Wall -Wextra -Wpedantic -O3 -ffast-math)
endif()

# ===========================================================================
# MPC Core Library (float32, platform-independent)
# ===========================================================================
add_library(mpc_riccati_core STATIC
    src/fp_math.c
    src/vehicle_model.c
    src/riccati_solver.c
    src/mpc_riccati.c
)
target_include_directories(mpc_riccati_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(mpc_riccati_core PUBLIC m)

# ===========================================================================
# ROS2 Dependencies
# ===========================================================================
find_package(ament_cmake QUIET)

if(ament_cmake_FOUND)
    find_package(rclc REQUIRED)
    find_package(rcl REQUIRED)
    find_package(rcutils REQUIRED)
    find_package(nav_msgs REQUIRED)
    find_package(geometry_msgs REQUIRED)
    find_package(ackermann_msgs REQUIRED)
    find_package(std_msgs REQUIRED)

    # ===================================================================
    # ROS2 MPC Simulation Node (gym_bridge / simulator)
    # ===================================================================
    add_executable(mpc_riccati_node sim/mpc_ros2_node.c)
    target_link_libraries(mpc_riccati_node mpc_riccati_core m)
    target_include_directories(mpc_riccati_node PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    ament_target_dependencies(mpc_riccati_node
        rclc rcl rcutils nav_msgs geometry_msgs ackermann_msgs std_msgs
    )

    # ===================================================================
    # ROS2 MPC Hardware Node (real car)
    # ===================================================================
    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/src/mpc_hardware_node.c)
        add_executable(mpc_hardware_node src/mpc_hardware_node.c)
        target_link_libraries(mpc_hardware_node mpc_riccati_core m)
        target_include_directories(mpc_hardware_node PRIVATE
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        )
        target_compile_definitions(mpc_hardware_node PRIVATE _GNU_SOURCE)
        ament_target_dependencies(mpc_hardware_node
            rclc rcl rcutils nav_msgs geometry_msgs ackermann_msgs std_msgs
        )
        install(TARGETS mpc_hardware_node DESTINATION lib/${PROJECT_NAME})
    endif()

    # ===================================================================
    # Install
    # ===================================================================
    install(TARGETS mpc_riccati_node DESTINATION lib/${PROJECT_NAME})
    install(DIRECTORY include/ DESTINATION include)

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
        install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
    endif()

    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
        install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
    endif()

    ament_package()
endif()
