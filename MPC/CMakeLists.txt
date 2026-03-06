cmake_minimum_required(VERSION 3.8)
project(mpc_hardware C)

# ROS2 ament_cmake build system
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rcutils REQUIRED)

# ===========================================================================
# MPC Core Library (Q16.16 fixed-point, platform-independent)
# ===========================================================================
add_library(mpc_core STATIC
    src/fp_math.c
    src/vehicle_model.c
    src/riccati_solver.c
    src/mpc_riccati.c
)
target_include_directories(mpc_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# Aggressive optimization for real-time performance on Jetson
# NOTE: -ffast-math is intentionally omitted — it can alter rounding behavior
# of the Q16.16 fixed-point math (integer-based), causing subtle bugs.
# -flto enables Link-Time Optimization for cross-TU inlining of fp_mul/fp_add
# into the hardware node (~5-15% solver speedup on ARM).
target_compile_options(mpc_core PRIVATE
    -O3
    -march=native
    -funroll-loops
    -flto
    -Wall -Wextra -Wpedantic
    -std=c99
)
target_compile_definitions(mpc_core PRIVATE
    _POSIX_C_SOURCE=200809L
)

# ===========================================================================
# MPC Hardware Node (ROS2 rclc executable)
# ===========================================================================
add_executable(mpc_hardware_node
    src/mpc_hardware_node.c
)
target_include_directories(mpc_hardware_node PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(mpc_hardware_node
    mpc_core
    m  # libm for math functions (sin, cos, atan2, sqrt)
)
target_compile_options(mpc_hardware_node PRIVATE
    -O3
    -march=native
    -flto
    -Wall -Wextra
    -std=c99
)
target_link_options(mpc_hardware_node PRIVATE -flto)
target_compile_definitions(mpc_hardware_node PRIVATE
    _GNU_SOURCE
    _POSIX_C_SOURCE=200809L
)

# Link ROS2 dependencies
ament_target_dependencies(mpc_hardware_node
    rcl
    rclc
    nav_msgs
    ackermann_msgs
    std_msgs
    geometry_msgs
    rosidl_runtime_c
    rcutils
)

# ===========================================================================
# Install
# ===========================================================================
install(TARGETS mpc_hardware_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()
