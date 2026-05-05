cmake_minimum_required(VERSION 3.8)
project(state_receiver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(f1tenth_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vitis_common REQUIRED)
find_package(OpenCL REQUIRED)

add_executable(mpc_receiver_node src/mpc_receiver.cpp)

target_include_directories(mpc_receiver_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(mpc_receiver_node
  rclcpp
  f1tenth_msgs
  ackermann_msgs
  vitis_common
)

target_link_libraries(mpc_receiver_node
  OpenCL::OpenCL
)

install(TARGETS
  mpc_receiver_node
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