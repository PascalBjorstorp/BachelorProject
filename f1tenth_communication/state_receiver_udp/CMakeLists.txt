cmake_minimum_required(VERSION 3.8)
project(state_receiver_udp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCL QUIET)

if(NOT OpenCL_FOUND)
  set(LOCAL_OPENCL_INCLUDE_DIR "$ENV{HOME}/.local/opencl-dev/usr/include")
  set(LOCAL_OPENCL_LIB_DIR "$ENV{HOME}/.local/opencl-dev/usr/lib/x86_64-linux-gnu")

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
    message(FATAL_ERROR "OpenCL not found via find_package and local fallback search")
  endif()
endif()

find_package(vitis_common QUIET)

if(NOT vitis_common_FOUND)
  set(LOCAL_VITIS_COMMON_INCLUDE_DIR "$ENV{HOME}/.local/vitis_common/include")
  set(LOCAL_VITIS_COMMON_LIB_DIR "$ENV{HOME}/.local/vitis_common/lib")

  find_path(VITIS_COMMON_INCLUDE_DIR
    NAMES vitis_common/common/ros_opencl_120.hpp
    PATHS
      ${LOCAL_VITIS_COMMON_INCLUDE_DIR}
      /usr/include
      /usr/local/include
      /opt/xilinx
      /tools/Xilinx
    PATH_SUFFIXES include
  )

  find_library(VITIS_COMMON_LIBRARY
    NAMES vitis_common
    PATHS
      ${LOCAL_VITIS_COMMON_LIB_DIR}
      /usr/lib
      /usr/local/lib
  )

  if(NOT VITIS_COMMON_INCLUDE_DIR OR NOT VITIS_COMMON_LIBRARY)
    message(FATAL_ERROR "vitis_common not found via find_package and local fallback search")
  endif()
endif()

add_executable(kria_udp_receiver_node src/kria_udp_receiver.cpp)

target_include_directories(kria_udp_receiver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

if(NOT vitis_common_FOUND)
  target_include_directories(kria_udp_receiver_node PRIVATE
    ${VITIS_COMMON_INCLUDE_DIR}
  )
endif()

if(NOT OpenCL_FOUND)
  target_include_directories(kria_udp_receiver_node PRIVATE
    ${OPENCL_INCLUDE_DIR}
  )
endif()

if(vitis_common_FOUND)
  ament_target_dependencies(kria_udp_receiver_node
    rclcpp
    vitis_common
  )
else()
  target_link_libraries(kria_udp_receiver_node
    ${VITIS_COMMON_LIBRARY}
  )
endif()

if(NOT vitis_common_FOUND)
  ament_target_dependencies(kria_udp_receiver_node
    rclcpp
  )
endif()

if(OpenCL_FOUND)
  target_link_libraries(kria_udp_receiver_node
    OpenCL::OpenCL
  )
else()
  target_link_libraries(kria_udp_receiver_node
    ${OPENCL_LIBRARY}
  )
endif()

install(TARGETS
  kria_udp_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
