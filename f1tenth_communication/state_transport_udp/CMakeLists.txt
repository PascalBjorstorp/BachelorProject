cmake_minimum_required(VERSION 3.5)
project(state_transport_udp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(FPGA_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../FPGA_Implementations/MPC_FPGA_Kria/include)

set(STATE_TRANSPORT_UDP_TARGET "all" CACHE STRING "Build target: all, jetson, or kria")
set_property(CACHE STATE_TRANSPORT_UDP_TARGET PROPERTY STRINGS all jetson kria)

if(STATE_TRANSPORT_UDP_TARGET STREQUAL "all" OR STATE_TRANSPORT_UDP_TARGET STREQUAL "jetson")
  find_package(rclcpp REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(ackermann_msgs REQUIRED)

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
--- stderr: state_transport_udp                                 
In file included from /home/akselmo/.local/vitis_common/include/vitis_common/common/ros_opencl_120.hpp:41,
                 from /home/akselmo/Documents/GitHub/BachelorProject/f1tenth_communication/state_transport_udp/src/kria_udp_receiver.cpp:32:
/home/akselmo/.local/opencl-dev/usr/include/CL/cl2.hpp:18:151: note: ‘#pragma message: cl2.hpp has been renamed to opencl.hpp to make it clear that it supports all versions of OpenCL. Please include opencl.hpp directly.’
   18 | #pragma message("cl2.hpp has been renamed to opencl.hpp to make it clear that it supports all versions of OpenCL. Please include opencl.hpp directly.")
      |                                                                                                                                                       ^
In file included from /home/akselmo/.local/opencl-dev/usr/include/CL/cl2.hpp:17:
/home/akselmo/.local/opencl-dev/usr/include/CL/opencl.hpp:2136:16: warning: ‘cl_uint cl::detail::getContextPlatformVersion(cl_context)’ defined but not used [-Wunused-function]
 2136 | static cl_uint getContextPlatformVersion(cl_context context)
      |                ^~~~~~~~~~~~~~~~~~~~~~~~~
/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_communication/state_transport_udp/src/udp_control_bridge.cpp: In constructor ‘state_transport_udp::UdpControlBridge::UdpControlBridge()’:
/home/akselmo/Documents/GitHub/BachelorProject/f1tenth_communication/state_transport_udp/src/udp_control_bridge.cpp:58:9: error: ‘stats_start_speed_mps_’ was not declared in this scope
   58 |         stats_start_speed_mps_ = static_cast<double>(MPC_FPGA_BRIDGE_STATS_START_SPEED_MPS);
      |         ^~~~~~~~~~~~~~~~~~~~~~
gmake[2]: *** [CMakeFiles/udp_control_bridge.dir/build.make:76: CMakeFiles/udp_control_bridge.dir/src/udp_control_bridge.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:167: CMakeFiles/udp_control_bridge.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< state_transport_udp [15.8s, exited with code 2]

Summary: 22 packages finished [16.1s]
  1 package failed: state_transport_udp
  1 package had stderr output: state_transport_udp
  add_executable(udp_control_bridge src/udp_control_bridge.cpp)
  target_include_directories(udp_control_bridge PRIVATE
    include
    ${FPGA_INCLUDE_DIR}
  )
  ament_target_dependencies(udp_control_bridge
    rclcpp
    ackermann_msgs
  )
endif()

if(STATE_TRANSPORT_UDP_TARGET STREQUAL "all" OR STATE_TRANSPORT_UDP_TARGET STREQUAL "kria")
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

  add_executable(kria_udp_receiver src/kria_udp_receiver.cpp)
  target_include_directories(kria_udp_receiver PRIVATE
    include
    ${FPGA_INCLUDE_DIR}
    ${VITIS_COMMON_INCLUDE_DIR}
    ${OPENCL_INCLUDE_DIR}
  )
  target_link_libraries(kria_udp_receiver ${VITIS_COMMON_LIBRARY} ${OPENCL_LIBRARY})
endif()

if(TARGET ros2_udp_sender)
  install(TARGETS ros2_udp_sender
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TARGET udp_control_bridge)
  install(TARGETS udp_control_bridge
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

if(TARGET kria_udp_receiver)
  install(TARGETS kria_udp_receiver
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

install(DIRECTORY config include
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
