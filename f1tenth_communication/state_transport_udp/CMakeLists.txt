cmake_minimum_required(VERSION 3.5)
project(state_transport_udp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

set(FPGA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../FPGA_Implementations/MPC_FPGA_Kria/include)

set(LOCAL_OPENCL_INCLUDE_DIR "$ENV{HOME}/.local/opencl-dev/usr/include")
set(LOCAL_OPENCL_LIB_DIR "$ENV{HOME}/.local/opencl-dev/usr/lib/x86_64-linux-gnu")
set(LOCAL_VITIS_COMMON_INCLUDE_DIR "$ENV{HOME}/.local/vitis_common/include")
set(LOCAL_VITIS_COMMON_LIB_DIR "$ENV{HOME}/.local/vitis_common/lib")

find_path(OPENCL_INCLUDE_DIR
  NAMES CL/cl2.hpp CL/opencl.hpp
  PATHS
    ${LOCAL_OPENCL_INCLUDE_DIR}
    /usr/include
    /usr/local/include
)

find_library(OPENCL_LIBRARY
  NAMES OpenCL OpenCL.so.1 libOpenCL.so.1
  PATHS
    ${LOCAL_OPENCL_LIB_DIR}
    /lib/x86_64-linux-gnu
    /usr/lib/x86_64-linux-gnu
    /usr/local/lib
)

if(NOT OPENCL_INCLUDE_DIR OR NOT OPENCL_LIBRARY)
  message(FATAL_ERROR "OpenCL headers/library not found. Expected local install under ~/.local/opencl-dev or system OpenCL dev packages.")
endif()

find_path(VITIS_COMMON_INCLUDE_DIR
  NAMES vitis_common/common/ros_opencl_120.hpp
  PATHS
    ${LOCAL_VITIS_COMMON_INCLUDE_DIR}
    /home/akselmo/.local/vitis_common/include
    /usr/include
    /usr/local/include
    /opt/xilinx
    /tools/Xilinx
  PATH_SUFFIXES include
)

if(NOT VITIS_COMMON_INCLUDE_DIR)
  message(FATAL_ERROR "vitis_common headers not found (need vitis_common/common/ros_opencl_120.hpp)")
endif()

find_library(VITIS_COMMON_LIBRARY
  NAMES vitis_common
  PATHS
    ${LOCAL_VITIS_COMMON_LIB_DIR}
    /home/akselmo/.local/vitis_common/lib
    /usr/lib
    /usr/local/lib
)

if(NOT VITIS_COMMON_LIBRARY)
  message(FATAL_ERROR "vitis_common library not found (need libvitis_common.so)")
endif()

add_executable(ros2_udp_sender src/ros2_udp_sender.cpp)
target_include_directories(ros2_udp_sender PRIVATE
  include
  ${FPGA_INCLUDE_DIR}
)
ament_target_dependencies(ros2_udp_sender
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

add_executable(udp_control_bridge src/udp_control_bridge.cpp)
target_include_directories(udp_control_bridge PRIVATE
  include
  ${FPGA_INCLUDE_DIR}
)
ament_target_dependencies(udp_control_bridge
  rclcpp
  ackermann_msgs
)

add_executable(kria_udp_receiver src/kria_udp_receiver.cpp)
target_include_directories(kria_udp_receiver PRIVATE
  include
  ${FPGA_INCLUDE_DIR}
  ${VITIS_COMMON_INCLUDE_DIR}
  ${OPENCL_INCLUDE_DIR}
)
target_link_libraries(kria_udp_receiver ${VITIS_COMMON_LIBRARY} ${OPENCL_LIBRARY})

install(TARGETS
  ros2_udp_sender
  udp_control_bridge
  kria_udp_receiver
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config include
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
