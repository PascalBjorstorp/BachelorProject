#!/bin/bash
# MPC F1/10th Setup and Build Script

set -e  # Exit on error

echo "================================"
echo "MPC F1/10th - Setup & Build"
echo "================================"
echo ""

# Step 1: Source ROS2 Jazzy
echo "[1/5] Sourcing ROS2 Jazzy..."
source /opt/ros/jazzy/setup.bash
echo "✓ ROS2 Jazzy sourced"
echo ""

# Step 2: Navigate to workspace
echo "[2/5] Setting up workspace..."
WORKSPACE_DIR="${HOME}/sim_ws"
if [ ! -d "$WORKSPACE_DIR" ]; then
    echo "Creating workspace: $WORKSPACE_DIR"
    mkdir -p "$WORKSPACE_DIR/src"
else
    echo "Workspace exists: $WORKSPACE_DIR"
fi
echo "✓ Workspace ready"
echo ""

# Step 3: Check if MPC package exists
echo "[3/5] Checking MPC package..."
if [ -f "$WORKSPACE_DIR/src/mpc_f1_10th/CMakeLists.txt" ]; then
    echo "✓ MPC package found at $WORKSPACE_DIR/src/mpc_f1_10th"
else
    echo "⚠ MPC package not found at $WORKSPACE_DIR/src/mpc_f1_10th"
    echo "  Option 1: Copy/move MPC to workspace:"
    echo "    cp -r /home/jonathan/MPC $WORKSPACE_DIR/src/mpc_f1_10th"
    echo "  Option 2: Symlink:"
    echo "    ln -s /home/jonathan/MPC $WORKSPACE_DIR/src/mpc_f1_10th"
    echo ""
    read -p "Copy MPC to workspace? (y/n) " -n 1 -r
    echo ""
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        cp -r /home/jonathan/MPC "$WORKSPACE_DIR/src/mpc_f1_10th"
        echo "✓ MPC copied to workspace"
    else
        echo "⚠ Skipping copy - make sure MPC is in workspace/src/"
    fi
fi
echo ""

# Step 4: Install ROS dependencies
echo "[4/5] Installing ROS dependencies..."
cd "$WORKSPACE_DIR"
rosdep install -i --from-path src --rosdistro jazzy -y 2>/dev/null || echo "⚠ Some dependencies may need manual installation"
echo "✓ Dependencies resolved"
echo ""

# Step 5: Build with colcon
echo "[5/5] Building with colcon..."
cd "$WORKSPACE_DIR"
colcon build --symlink-install
echo "✓ Build complete!"
echo ""

# Source the workspace
source "$WORKSPACE_DIR/install/local_setup.bash"
echo ""
echo "================================"
echo "Setup Complete!"
echo "================================"
echo ""
echo "To use the MPC package in a new terminal:"
echo "  1. source /opt/ros/jazzy/setup.bash"
echo "  2. source ~/sim_ws/install/local_setup.bash"
echo "  3. ros2 launch mpc_f1_10th mpc_launch.py"
echo ""
echo "Or use the provided run scripts:"
echo "  ./run_simulator.sh   # Terminal 1 - Run simulator"
echo "  ./run_mpc.sh         # Terminal 2 - Run MPC node"
echo ""
