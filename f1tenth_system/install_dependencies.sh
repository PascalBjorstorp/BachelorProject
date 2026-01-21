#!/bin/bash
# F1Tenth System Dependency Installation Script
# Supports ROS2 Jazzy and Humble distributions
#
# Usage: ./install_dependencies.sh
#
# This script will:
# 1. Detect the current ROS2 distribution
# 2. Install all required ROS2 packages
# 3. Install rosdep dependencies
# 4. Handle distribution-specific package names

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Print colored output
info() { echo -e "${BLUE}[INFO]${NC} $1"; }
success() { echo -e "${GREEN}[SUCCESS]${NC} $1"; }
warn() { echo -e "${YELLOW}[WARNING]${NC} $1"; }
error() { echo -e "${RED}[ERROR]${NC} $1"; }

# Detect ROS2 distribution
detect_ros_distro() {
    if [ -n "$ROS_DISTRO" ]; then
        echo "$ROS_DISTRO"
        return
    fi

    # Try to detect from installed ROS2
    if [ -d "/opt/ros/jazzy" ]; then
        echo "jazzy"
    elif [ -d "/opt/ros/humble" ]; then
        echo "humble"
    else
        echo ""
    fi
}

# Validate ROS2 distribution
validate_distro() {
    local distro=$1
    case $distro in
        jazzy|humble)
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

# Core ROS2 packages needed by f1tenth_system
# These are common across both Jazzy and Humble
COMMON_PACKAGES=(
    # Build tools
    "ros-dev-tools"
    "python3-colcon-common-extensions"
    "python3-rosdep"
    "python3-vcstool"

    # Core ROS2 packages
    "ros-\${ROS_DISTRO}-ackermann-msgs"
    "ros-\${ROS_DISTRO}-rclcpp"
    "ros-\${ROS_DISTRO}-rclcpp-components"
    "ros-\${ROS_DISTRO}-rclpy"
    "ros-\${ROS_DISTRO}-std-msgs"
    "ros-\${ROS_DISTRO}-geometry-msgs"
    "ros-\${ROS_DISTRO}-sensor-msgs"
    "ros-\${ROS_DISTRO}-nav-msgs"
    "ros-\${ROS_DISTRO}-builtin-interfaces"
    "ros-\${ROS_DISTRO}-action-msgs"

    # TF2 packages
    "ros-\${ROS_DISTRO}-tf2"
    "ros-\${ROS_DISTRO}-tf2-ros"
    "ros-\${ROS_DISTRO}-tf2-geometry-msgs"

    # Message generation
    "ros-\${ROS_DISTRO}-rosidl-default-generators"
    "ros-\${ROS_DISTRO}-rosidl-default-runtime"
    "ros-\${ROS_DISTRO}-rosidl-runtime-py"

    # Control and teleoperation
    "ros-\${ROS_DISTRO}-control-msgs"
    "ros-\${ROS_DISTRO}-trajectory-msgs"
    "ros-\${ROS_DISTRO}-joy"
    "ros-\${ROS_DISTRO}-diagnostic-updater"

    # Serial communication (for VESC driver)
    "ros-\${ROS_DISTRO}-serial-driver"

    # Testing and linting
    "ros-\${ROS_DISTRO}-ament-lint-auto"
    "ros-\${ROS_DISTRO}-ament-lint-common"
    "ros-\${ROS_DISTRO}-ament-cmake-xmllint"
    "ros-\${ROS_DISTRO}-ament-copyright"
    "ros-\${ROS_DISTRO}-ament-flake8"
    "ros-\${ROS_DISTRO}-ament-pep257"
    "ros-\${ROS_DISTRO}-ament-xmllint"
    "ros-\${ROS_DISTRO}-launch"
    "ros-\${ROS_DISTRO}-launch-ros"
    "ros-\${ROS_DISTRO}-launch-testing"
    "ros-\${ROS_DISTRO}-launch-testing-ros"
    "ros-\${ROS_DISTRO}-launch-testing-ament-cmake"

    # Test interfaces
    "ros-\${ROS_DISTRO}-example-interfaces"
    "ros-\${ROS_DISTRO}-test-msgs"
    "ros-\${ROS_DISTRO}-action-tutorials-interfaces"
    "ros-\${ROS_DISTRO}-std-srvs"

    # Python dependencies
    "python3-pytest"
    "python3-numpy"
    "python3-tk"
)

# Distribution-specific packages
# These packages may have different names or availability
declare -A JAZZY_SPECIFIC
JAZZY_SPECIFIC=(
    ["urg_node"]="ros-jazzy-urg-node"
)

declare -A HUMBLE_SPECIFIC
HUMBLE_SPECIFIC=(
    ["urg_node"]="ros-humble-urg-node"
)

# Check if running as root (needed for apt)
check_sudo() {
    if [ "$EUID" -ne 0 ]; then
        if ! command -v sudo &> /dev/null; then
            error "This script requires root privileges. Please run as root or install sudo."
            exit 1
        fi
        SUDO="sudo"
    else
        SUDO=""
    fi
}

# Install packages using apt
install_apt_packages() {
    local packages=("$@")
    local to_install=()

    for pkg in "${packages[@]}"; do
        # Expand ROS_DISTRO in package name
        expanded_pkg=$(echo "$pkg" | envsubst)
        
        # Check if package is available
        if apt-cache show "$expanded_pkg" &> /dev/null; then
            if ! dpkg -s "$expanded_pkg" &> /dev/null 2>&1; then
                to_install+=("$expanded_pkg")
            fi
        else
            warn "Package not found in apt: $expanded_pkg (skipping)"
        fi
    done

    if [ ${#to_install[@]} -gt 0 ]; then
        info "Installing ${#to_install[@]} packages..."
        $SUDO apt-get install -y "${to_install[@]}"
    else
        info "All packages already installed"
    fi
}

# Initialize rosdep if needed
init_rosdep() {
    if [ ! -f "/etc/ros/rosdep/sources.list.d/20-default.list" ]; then
        info "Initializing rosdep..."
        $SUDO rosdep init || warn "rosdep init failed (may already be initialized)"
    fi
    
    info "Updating rosdep..."
    rosdep update --rosdistro="$ROS_DISTRO" || warn "rosdep update had some issues"
}

# Install dependencies using rosdep
install_rosdep_deps() {
    local workspace_dir="$1"
    
    if [ -d "$workspace_dir" ]; then
        info "Installing rosdep dependencies for workspace: $workspace_dir"
        cd "$workspace_dir"
        rosdep install --from-paths . --ignore-src -y --rosdistro="$ROS_DISTRO" || \
            warn "Some rosdep dependencies may not be available"
    fi
}

# Main installation function
main() {
    echo "========================================"
    echo "F1Tenth System Dependency Installer"
    echo "========================================"
    echo ""

    # Check for sudo
    check_sudo

    # Detect ROS2 distribution
    ROS_DISTRO=$(detect_ros_distro)
    
    if [ -z "$ROS_DISTRO" ]; then
        error "Could not detect ROS2 distribution."
        echo "Please set the ROS_DISTRO environment variable:"
        echo "  export ROS_DISTRO=jazzy"
        echo "  or"
        echo "  export ROS_DISTRO=humble"
        exit 1
    fi

    if ! validate_distro "$ROS_DISTRO"; then
        error "Unsupported ROS2 distribution: $ROS_DISTRO"
        echo "This script supports: jazzy, humble"
        exit 1
    fi

    export ROS_DISTRO
    info "Detected ROS2 distribution: $ROS_DISTRO"
    echo ""

    # Update apt cache
    info "Updating package cache..."
    $SUDO apt-get update

    # Install common packages
    info "Installing common ROS2 packages..."
    install_apt_packages "${COMMON_PACKAGES[@]}"

    # Install distribution-specific packages
    info "Installing $ROS_DISTRO-specific packages..."
    if [ "$ROS_DISTRO" = "jazzy" ]; then
        for key in "${!JAZZY_SPECIFIC[@]}"; do
            install_apt_packages "${JAZZY_SPECIFIC[$key]}"
        done
    elif [ "$ROS_DISTRO" = "humble" ]; then
        for key in "${!HUMBLE_SPECIFIC[@]}"; do
            install_apt_packages "${HUMBLE_SPECIFIC[$key]}"
        done
    fi

    # Initialize and run rosdep
    init_rosdep

    # Get the directory where this script is located
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    
    # Install rosdep dependencies for the workspace
    install_rosdep_deps "$SCRIPT_DIR"

    echo ""
    success "Dependency installation complete!"
    echo ""
    echo "To build the f1tenth_system workspace:"
    echo "  cd $SCRIPT_DIR"
    echo "  source /opt/ros/$ROS_DISTRO/setup.bash"
    echo "  colcon build"
    echo ""
}

# Run main
main "$@"
