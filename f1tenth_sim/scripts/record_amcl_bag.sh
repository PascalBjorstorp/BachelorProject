#!/bin/bash
# =============================================================================
# AMCL Benchmark Bag Recording Script
# =============================================================================
#
# This script records a ROS 2 bag with all topics needed for AMCL benchmarking.
# Run this on the PC where the simulation is running.
#
# Usage:
#   ./record_amcl_bag.sh                    # Default 60 seconds
#   ./record_amcl_bag.sh 120                # Record for 120 seconds
#   ./record_amcl_bag.sh 0                  # Record until Ctrl+C
#
# Prerequisites:
#   1. Simulation must be running:
#      ros2 launch f1tenth_gym_ros gym_bridge_launch.py
#   
#   2. A controller should be running so the car moves:
#      ros2 launch f1tenth_control ftg_launch.py
#      (or keyboard teleop, pure pursuit, etc.)
#
#   3. sim.yaml must have: tf_frame_id: 'odom' and odom_frame_id: 'odom'
#
# Output:
#   - Bag saved to: ~/f1tenth_bags/amcl_benchmark_<timestamp>/
#
# =============================================================================

set -e

# Configuration
BAG_DIR="${HOME}/f1tenth_bags"
DURATION="${1:-60}"  # Default 60 seconds, or first argument
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG_NAME="amcl_benchmark_${TIMESTAMP}"
BAG_PATH="${BAG_DIR}/${BAG_NAME}"

# Topics to record
TOPICS=(
    "/scan"                    # LiDAR data (primary input for AMCL)
    "/map"                     # Occupancy grid map
    "/tf"                      # Dynamic transforms (odom -> base_link)
    "/tf_static"               # Static transforms (laser frame)
    "/ego_racecar/odom"        # Ground truth odometry for comparison
)

# Create output directory
mkdir -p "${BAG_DIR}"

echo "=============================================="
echo "  AMCL Benchmark Bag Recording"
echo "=============================================="
echo ""
echo "Output: ${BAG_PATH}"
echo "Topics: ${TOPICS[*]}"
echo ""

# Check if simulation topics are available
echo "Checking for required topics..."
MISSING_TOPICS=()
for topic in "${TOPICS[@]}"; do
    if ! ros2 topic list | grep -q "^${topic}$"; then
        MISSING_TOPICS+=("${topic}")
    fi
done

if [ ${#MISSING_TOPICS[@]} -gt 0 ]; then
    echo ""
    echo "WARNING: The following topics are not available:"
    for topic in "${MISSING_TOPICS[@]}"; do
        echo "  - ${topic}"
    done
    echo ""
    echo "Make sure the simulation is running:"
    echo "  ros2 launch f1tenth_gym_ros gym_bridge_launch.py"
    echo ""
    read -p "Continue anyway? [y/N] " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Show current scan rate
echo ""
echo "Current scan rate:"
timeout 3 ros2 topic hz /scan --window 10 2>/dev/null || echo "  (Could not measure)"
echo ""

# Build ros2 bag record command
RECORD_CMD="ros2 bag record -o ${BAG_PATH} --topics"
for topic in "${TOPICS[@]}"; do
    RECORD_CMD="${RECORD_CMD} ${topic}"
done

echo "Recording until Ctrl+C..."
if [ "${DURATION}" != "0" ]; then
    echo "(Suggested duration: ${DURATION} seconds)"
fi
echo ""

# Start recording
echo "Starting recording..."
echo "Command: ${RECORD_CMD}"
echo ""
echo "Press Ctrl+C to stop recording when ready!"
echo ""
eval "${RECORD_CMD}"

# Show results
echo ""
echo "=============================================="
echo "  Recording Complete!"
echo "=============================================="
echo ""
echo "Bag saved to: ${BAG_PATH}"
echo ""
echo "Bag info:"
ros2 bag info "${BAG_PATH}"
echo ""
echo "=============================================="
echo "  Next Steps"
echo "=============================================="
echo ""
echo "1. Copy bag to Jetson:"
echo "   scp -r ${BAG_PATH} jetson@<jetson-ip>:~/f1tenth_bags/"
echo ""
echo "2. On Jetson, run benchmark:"
echo "   ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \\"
echo "     bag_path:=~/f1tenth_bags/${BAG_NAME}"
echo ""
echo "3. With different particle counts:"
echo "   ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \\"
echo "     bag_path:=~/f1tenth_bags/${BAG_NAME} \\"
echo "     min_particles:=1000 max_particles:=5000"
echo ""
