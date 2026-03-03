cmake_minimum_required(VERSION 3.18)
project(f1tenth_localization LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================================================
#  Dependencies
# ================================================================
# Hybrid ament_cmake + Python package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_amcl REQUIRED)

# C++ dependencies for gpu_amcl_cpp
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ── Optional CUDA detection ──────────────────────────────────────
# On Jetson / GPU machines: builds full AMCL with CUDA kernels.
# On laptops without CUDA: builds only odom_fused and ekf nodes.
include(CheckLanguage)
check_language(CUDA)
if(CMAKE_CUDA_COMPILER)
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 17)
  set(CMAKE_CUDA_STANDARD_REQUIRED ON)
  find_package(CUDAToolkit REQUIRED)
  set(HAS_CUDA TRUE)
  message(STATUS "CUDA found — building full gpu_amcl_cpp (AMCL + Odom + EKF)")
else()
  set(HAS_CUDA FALSE)
  message(STATUS "CUDA NOT found — building Odom + EKF nodes only (no AMCL)")
endif()

# ================================================================
#  gpu_amcl_cpp — CUDA-dependent targets (AMCL)
# ================================================================
if(HAS_CUDA)
  add_library(gpu_amcl_cuda STATIC
    gpu_amcl_cpp/src/cuda/motion_model_kernels.cu
    gpu_amcl_cpp/src/cuda/sensor_model_kernels.cu
    gpu_amcl_cpp/src/cuda/resampling_kernels.cu
  )
  target_include_directories(gpu_amcl_cuda PUBLIC
    gpu_amcl_cpp/include
  )
  target_link_libraries(gpu_amcl_cuda
    CUDA::cudart
    CUDA::curand
  )
  set_target_properties(gpu_amcl_cuda PROPERTIES
    CUDA_SEPARABLE_COMPILATION OFF
    POSITION_INDEPENDENT_CODE  ON
    CUDA_RESOLVE_DEVICE_SYMBOLS ON
  )

  # Full C++ core library (includes AMCL)
  add_library(gpu_amcl_cpp_lib STATIC
    gpu_amcl_cpp/src/helpers/map_utils.cpp
    gpu_amcl_cpp/src/core/motion_model.cpp
    gpu_amcl_cpp/src/core/sensor_model.cpp
    gpu_amcl_cpp/src/core/resampling.cpp
    gpu_amcl_cpp/src/core/particle_filter.cpp
    gpu_amcl_cpp/src/core/amcl_node.cpp
    gpu_amcl_cpp/src/core/odom_node.cpp
    gpu_amcl_cpp/src/core/ekf_node.cpp
  )
  target_include_directories(gpu_amcl_cpp_lib PUBLIC
    gpu_amcl_cpp/include
  )
  target_link_libraries(gpu_amcl_cpp_lib
    gpu_amcl_cuda
    Eigen3::Eigen
    CUDA::cudart
    CUDA::curand
  )
  ament_target_dependencies(gpu_amcl_cpp_lib
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs std_msgs
  )

  # AMCL executable
  add_executable(gpu_amcl_cpp_node
    gpu_amcl_cpp/src/main_amcl.cpp
  )
  target_link_libraries(gpu_amcl_cpp_node gpu_amcl_cpp_lib)
  ament_target_dependencies(gpu_amcl_cpp_node rclcpp)

  # Odom + EKF link against full lib
  add_executable(odom_fused_node
    gpu_amcl_cpp/src/main_odom.cpp
  )
  target_link_libraries(odom_fused_node gpu_amcl_cpp_lib)
  ament_target_dependencies(odom_fused_node rclcpp)

  add_executable(ekf_localization_node
    gpu_amcl_cpp/src/main_ekf.cpp
  )
  target_link_libraries(ekf_localization_node gpu_amcl_cpp_lib)
  ament_target_dependencies(ekf_localization_node rclcpp)

  install(TARGETS
    gpu_amcl_cpp_node
    odom_fused_node
    ekf_localization_node
    DESTINATION lib/${PROJECT_NAME}
  )

else()
  # ============================================================
  #  No CUDA — build only Odom and EKF (CPU-only, no AMCL)
  # ============================================================
  # Minimal library without CUDA dependencies
  add_library(gpu_amcl_cpp_cpu_lib STATIC
    gpu_amcl_cpp/src/helpers/map_utils.cpp
    gpu_amcl_cpp/src/core/odom_node.cpp
    gpu_amcl_cpp/src/core/ekf_node.cpp
  )
  target_include_directories(gpu_amcl_cpp_cpu_lib PUBLIC
    gpu_amcl_cpp/include
  )
  target_link_libraries(gpu_amcl_cpp_cpu_lib
    Eigen3::Eigen
  )
  ament_target_dependencies(gpu_amcl_cpp_cpu_lib
    rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs std_msgs
  )

  add_executable(odom_fused_node
    gpu_amcl_cpp/src/main_odom.cpp
  )
  target_link_libraries(odom_fused_node gpu_amcl_cpp_cpu_lib)
  ament_target_dependencies(odom_fused_node rclcpp)

  add_executable(ekf_localization_node
    gpu_amcl_cpp/src/main_ekf.cpp
  )
  target_link_libraries(ekf_localization_node gpu_amcl_cpp_cpu_lib)
  ament_target_dependencies(ekf_localization_node rclcpp)

  install(TARGETS
    odom_fused_node
    ekf_localization_node
    DESTINATION lib/${PROJECT_NAME}
  )

endif()

# ================================================================
#  Pipeline Latency Monitor (CPU-only, no CUDA dependency)
# ================================================================
add_executable(pipeline_latency_monitor
  gpu_amcl_cpp/src/core/pipeline_latency_monitor.cpp
)
target_include_directories(pipeline_latency_monitor PUBLIC
  gpu_amcl_cpp/include
)
ament_target_dependencies(pipeline_latency_monitor
  rclcpp sensor_msgs geometry_msgs
)
install(TARGETS
  pipeline_latency_monitor
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================================
#  Python gpu_amcl (existing)
# ================================================================
# Install gpu_amcl Python module (importable as 'gpu_amcl')
ament_python_install_package(gpu_amcl)

# Install Python scripts as executables
install(PROGRAMS
  Benchmark/localization_benchmark.py
  Benchmark/JetsonFiles/performance_monitor.py
  Benchmark/JetsonFiles/odom_tf_publisher.py
  gpu_amcl/scripts/gpu_amcl_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================================
#  Shared assets (launch, config, benchmark)
# ================================================================
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
