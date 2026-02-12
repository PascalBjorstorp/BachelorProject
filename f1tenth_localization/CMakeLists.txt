cmake_minimum_required(VERSION 3.8)
project(f1tenth_localization)

# Basic setup for Python test script and nav2 AMCL dependency
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_amcl REQUIRED)

# Install Python scripts as executables
install(PROGRAMS
  Benchmark/JetsonFiles/performance_monitor.py
  Benchmark/JetsonFiles/odom_tf_publisher.py
  RealRobot/JetsonFiles/hokuyo_lidar_diagnostic.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files from all subdirectories into a single share/launch/ dir
install(FILES
  Benchmark/JetsonFiles/amcl_bag_benchmark.launch.py
  Benchmark/JetsonFiles/sim_headless.launch.py
  Simulation/JetsonFiles/sim_localization.launch.py
  Simulation/PCFiles/amcl_bag_visualization.launch.py
  Simulation/PCFiles/nav2_amcl.launch.py
  RealRobot/JetsonFiles/hokuyo_lidar.launch.py
  RealRobot/JetsonFiles/real_localization.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config alongside launch files
install(FILES
  Simulation/PCFiles/amcl_bag_visualization.rviz
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files from both config directories into a single share/config/ dir
install(FILES
  config/nav2_amcl_params.yaml
  RealRobot/config/hokuyo_ust10lx.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
