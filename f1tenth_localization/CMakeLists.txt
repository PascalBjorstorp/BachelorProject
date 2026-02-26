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
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Also install benchmark launch files
install(FILES
  Benchmark/JetsonFiles/amcl_bag_benchmark.launch.py
  Benchmark/JetsonFiles/sim_headless.launch.py
  Benchmark/JetsonFiles/record_bag.launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install config files (YAML params + RViz configs)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Also install benchmark config
install(FILES
  Benchmark/config/benchmark_configs.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

# Install benchmark sweep script
install(PROGRAMS
  Benchmark/JetsonFiles/run_benchmark_sweep.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
