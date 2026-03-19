cmake_minimum_required(VERSION 3.5)
project(f1tenth_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(f1tenth_msgs
  "msg/MpcState.msg"
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
