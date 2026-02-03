cmake_minimum_required(VERSION 3.8)
project(f1tenth_localization)

# Basic setup for Python test script and nav2 AMCL dependency
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_amcl REQUIRED)

# Install Python scripts as executables
install(PROGRAMS
  scripts/performance_monitor.py
  scripts/odom_tf_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
