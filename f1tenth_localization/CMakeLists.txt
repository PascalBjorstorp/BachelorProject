cmake_minimum_required(VERSION 3.8)
project(f1tenth_localization)

# Basic setup for Python test script and nav2 AMCL dependency
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_amcl REQUIRED)

# Install Python scripts (if any)
install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
