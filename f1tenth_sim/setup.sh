#!/bin/bash
# F1Tenth Gym ROS2 Bridge - Setup Script
# This script sets up a virtual environment and installs all dependencies

set -e  # Exit on error

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$SCRIPT_DIR/.venv"

echo "================================================"
echo "  F1Tenth Gym ROS2 Bridge Setup"
echo "================================================"
echo ""

# Check for ROS2 Jazzy
if [ ! -f "/opt/ros/jazzy/setup.bash" ]; then
    echo "ERROR: ROS2 Jazzy not found at /opt/ros/jazzy"
    echo "Please install ROS2 Jazzy first:"
    echo "  https://docs.ros.org/en/jazzy/Installation.html"
    exit 1
fi
echo "✓ ROS2 Jazzy found"

# Check for python3-venv
if ! python3 -c "import venv" 2>/dev/null; then
    echo "ERROR: python3-venv is not installed"
    echo "Please install it with: sudo apt install python3-venv"
    exit 1
fi
echo "✓ python3-venv found"

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2 | cut -d'.' -f1,2)
PYTHON_MAJOR=$(echo "$PYTHON_VERSION" | cut -d'.' -f1)
PYTHON_MINOR=$(echo "$PYTHON_VERSION" | cut -d'.' -f2)
if [ "$PYTHON_MAJOR" -lt 3 ] || ([ "$PYTHON_MAJOR" -eq 3 ] && [ "$PYTHON_MINOR" -lt 10 ]); then
    echo "ERROR: Python 3.10+ required (found $PYTHON_VERSION)"
    exit 1
fi
echo "✓ Python $PYTHON_VERSION found"

# Create virtual environment if it doesn't exist (without system site packages)
if [ ! -d "$VENV_DIR" ]; then
    echo ""
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
    echo "✓ Virtual environment created at $VENV_DIR"
else
    echo "✓ Virtual environment already exists"
fi

# Activate virtual environment
source "$VENV_DIR/bin/activate"

# Upgrade pip
echo ""
echo "Upgrading pip..."
pip install --upgrade pip wheel setuptools > /dev/null

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip install -r "$SCRIPT_DIR/requirements.txt"
echo "✓ Python dependencies installed"

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install ROS2 dependencies
echo ""
echo "Installing ROS2 dependencies..."
cd "$SCRIPT_DIR"
rosdep update > /dev/null 2>&1 || true
rosdep install -i --from-path . --rosdistro jazzy -y > /dev/null 2>&1 || echo "Note: Some rosdep packages may need manual installation"
echo "✓ ROS2 dependencies checked"

# Build the workspace
echo ""
echo "Building the ROS2 package..."
colcon build --symlink-install
echo "✓ Package built successfully"

# Configure map path in sim.yaml (just the map name, bridge resolves the full path)
echo ""
echo "Configuring map path..."
SIM_YAML="$SCRIPT_DIR/config/sim.yaml"
DEFAULT_MAP="my_track_map"
sed -i "s|map_path:.*|map_path: '$DEFAULT_MAP'|" "$SIM_YAML"
echo "✓ Map path set to: $DEFAULT_MAP"

echo ""
echo "================================================"
echo "  Setup Complete!"
echo "================================================"
echo ""
echo "To run the simulation:"
echo ""
echo "  ./run.sh"
echo ""
echo "Available maps in $SCRIPT_DIR/maps:"
ls -1 "$SCRIPT_DIR/maps/"*.yaml 2>/dev/null | xargs -I{} basename {} .yaml | sed 's/^/  - /'
echo ""
echo "To change the map, edit config/sim.yaml and update 'map_path'"
echo ""
