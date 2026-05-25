cmake_minimum_required(VERSION 3.18)
project(f1tenth_localization LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    $<$<COMPILE_LANGUAGE:CXX>:-Wall>
    $<$<COMPILE_LANGUAGE:CXX>:-Wextra>
    $<$<COMPILE_LANGUAGE:CXX>:-Wpedantic>
  )
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================================================
#  Dependencies
# ================================================================
# Hybrid ament_cmake + Python package
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_amcl REQUIRED)

# C++ dependencies for gpu_amcl_cpp
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
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
    gpu_amcl_cpp/src/cuda/weight_kernels.cu
    gpu_amcl_cpp/src/cuda/estimate_kernels.cu
    gpu_amcl_cpp/src/cuda/cluster_kernels.cu
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gpu_amcl_cuda PRIVATE
      $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler=-Wno-pedantic>
    )
  endif()
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
    ament_index_cpp rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros tf2_geometry_msgs std_msgs
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
  src/core/pipeline_latency_monitor.cpp
)
target_include_directories(pipeline_latency_monitor PUBLIC
  include
)
ament_target_dependencies(pipeline_latency_monitor
  rclcpp sensor_msgs geometry_msgs std_msgs ackermann_msgs
)

add_executable(performance_monitor_cpp
  src/core/performance_monitor.cpp
)
target_include_directories(performance_monitor_cpp PUBLIC
  include
  config
)
ament_target_dependencies(performance_monitor_cpp
  rclcpp
)

install(TARGETS
  pipeline_latency_monitor
  performance_monitor_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================================
#  Python gpu_amcl (existing)
# ================================================================
# Install gpu_amcl Python module (importable as 'gpu_amcl') if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gpu_amcl/__init__.py")
  ament_python_install_package(gpu_amcl)
else()
  message(STATUS "gpu_amcl package missing __init__.py; skipping ament_python_install_package(gpu_amcl)")
endif()

# Install Python scripts as executables (skip optional legacy node if absent)
set(LOCALIZATION_PYTHON_SCRIPTS
  optitrackBenchmark/optitrack_map_calibrator.py
  scripts/sim_benchmark_logger.py
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gpu_amcl/scripts/gpu_amcl_node.py")
  list(APPEND LOCALIZATION_PYTHON_SCRIPTS gpu_amcl/scripts/gpu_amcl_node.py)
else()
  message(STATUS "gpu_amcl/scripts/gpu_amcl_node.py not found; skipping install")
endif()

if(LOCALIZATION_PYTHON_SCRIPTS)
  install(PROGRAMS
    ${LOCALIZATION_PYTHON_SCRIPTS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# ================================================================
#  Shared assets (launch, config, benchmark)
# ================================================================
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files (YAML params + RViz configs)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install OptiTrack calibration/benchmark helper files.
install(DIRECTORY
  optitrackBenchmark
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  optitrackBenchmark/record_optitrack_benchmark_bag.sh
  DESTINATION share/${PROJECT_NAME}/optitrackBenchmark
)

install(PROGRAMS
  Benchmark/run_sim_amcl_benchmark.py
  DESTINATION share/${PROJECT_NAME}/Benchmark
)

ament_package()
