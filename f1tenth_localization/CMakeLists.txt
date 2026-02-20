cmake_minimum_required(VERSION 3.8)
project(f1tenth_localization)

# Hybrid ament_cmake + Python package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_amcl REQUIRED)

# Install gpu_amcl Python module (importable as 'gpu_amcl')
ament_python_install_package(gpu_amcl)

# Install Python scripts as executables
install(PROGRAMS
  Benchmark/JetsonFiles/performance_monitor.py
  Benchmark/JetsonFiles/odom_tf_publisher.py
  gpu_amcl/scripts/gpu_amcl_node.py
  RealRobot/JetsonFiles/hokuyo_lidar_diagnostic.py
  RealRobot/JetsonFiles/hokuyo_scip_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files from all subdirectories into a single share/launch/ dir
install(FILES
  Benchmark/JetsonFiles/amcl_bag_benchmark.launch.py
  Benchmark/JetsonFiles/sim_headless.launch.py
  Benchmark/JetsonFiles/record_bag.launch.py
  Simulation/JetsonFiles/sim_localization.launch.py
  Simulation/PCFiles/amcl_bag_visualization.launch.py
  RealRobot/JetsonFiles/hokuyo_lidar.launch.py
  RealRobot/JetsonFiles/real_localization.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config alongside launch files
install(FILES
  Simulation/PCFiles/amcl_bag_visualization.rviz
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files from all config directories into a single share/config/ dir
install(FILES
  config/nav2_amcl_params.yaml
  config/gpu_amcl_params.yaml
  Benchmark/config/benchmark_configs.yaml
  RealRobot/config/hokuyo_ust10lx.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install benchmark sweep script
install(PROGRAMS
  Benchmark/JetsonFiles/run_benchmark_sweep.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
