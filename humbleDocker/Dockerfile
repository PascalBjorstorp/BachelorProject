# =============================================================================
# BachelorProject - ROS2 Humble Docker Image (Ubuntu 22.04)
# =============================================================================
# Build:  docker build -t f1tenth-humble .
# Run:    docker compose up
# Shell:  docker compose run --rm f1tenth bash
# =============================================================================

FROM ros:humble-ros-base-jammy

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# ---------- System Dependencies ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
    # Build tools
    build-essential \
    cmake \
    gcc \
    g++ \
    python3-pip \
    python3-venv \
    python3-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-numpy \
    python3-tk \
    python3-pytest \
    git \
    wget \
    curl \
    # ROS2 Humble packages
    ros-humble-ackermann-msgs \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-control-msgs \
    ros-humble-trajectory-msgs \
    ros-humble-rclcpp \
    ros-humble-rclcpp-components \
    ros-humble-rclpy \
    ros-humble-rclc \
    ros-humble-rcl \
    ros-humble-rcutils \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-nav2-amcl \
    ros-humble-nav2-map-server \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-teleop-twist-keyboard \
    ros-humble-joy \
    ros-humble-diagnostic-updater \
    ros-humble-serial-driver \
    ros-humble-asio-cmake-module \
    ros-humble-launch-testing \
    ros-humble-launch-testing-ros \
    ros-humble-launch-testing-ament-cmake \
    # Linear algebra
    libeigen3-dev \
    # X11 / OpenGL for GUI (rviz2)
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libx11-6 \
    libxrender1 \
    libxext6 \
    x11-apps \
    # Linting / testing
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

# ---------- Python Dependencies ----------
# Upgrade pip
RUN pip3 install --no-cache-dir --upgrade pip setuptools wheel

# Install Python packages needed by the project
COPY f1tenth_sim/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt && rm /tmp/requirements.txt

# ---------- Workspace Setup ----------
ENV WS=/ros2_ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}

# Copy project sources into the workspace
COPY f1tenth_communication/    ${WS}/src/f1tenth_communication/
COPY f1tenth_control/          ${WS}/src/f1tenth_control/
COPY f1tenth_localization/     ${WS}/src/f1tenth_localization/
COPY f1tenth_planning/         ${WS}/src/f1tenth_planning/
COPY f1tenth_sim/              ${WS}/src/f1tenth_sim/
COPY f1tenth_system/           ${WS}/src/f1tenth_system/
COPY f1tenth_parameters/       ${WS}/src/f1tenth_parameters/
COPY MPC/                      ${WS}/src/MPC/

# Copy config / parameters / trajectories that may be referenced at runtime
COPY f1tenth_parameters/vehicle_params.yaml ${WS}/src/f1tenth_parameters/vehicle_params.yaml

# ---------- Initialize rosdep ----------
RUN rosdep update --rosdistro=humble || true
RUN cd ${WS} && \
    rosdep install --from-paths src --ignore-src -r -y --rosdistro=humble || true

# ---------- Build ----------
# Limit parallelism to avoid OOM during compilation in Docker
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${WS} && \
    MAKEFLAGS='-j2' colcon build --symlink-install \
        --parallel-workers 2 \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"

# ---------- Entrypoint ----------
COPY humbleDocker/docker-entrypoint.sh /docker-entrypoint.sh
RUN chmod +x /docker-entrypoint.sh

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
