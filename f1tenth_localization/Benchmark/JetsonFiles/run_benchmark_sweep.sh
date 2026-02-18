#!/bin/bash
# ============================================================
# run_benchmark_sweep.sh
#
# Reads benchmark_configs.yaml and runs amcl_bag_benchmark
# for each configuration × repeat count.
#
# Prerequisites:
#   - ROS 2 workspace sourced (source install/setup.bash)
#   - f1tenth_localization package built and installed
#   - A recorded lap bag exists in Benchmark/bags/lapBags/
#
# Usage:
#   ./run_benchmark_sweep.sh
#   ./run_benchmark_sweep.sh /path/to/bag
#   ./run_benchmark_sweep.sh /path/to/bag /path/to/configs.yaml
# ============================================================
set -euo pipefail

# ---- Resolve paths relative to this script ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCALIZATION_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
WORKSPACE_ROOT="$(cd "$LOCALIZATION_DIR/.." && pwd)"

DEFAULT_BAG_DIR="$LOCALIZATION_DIR/Benchmark/bags/lapBags"
DEFAULT_CONFIG="$LOCALIZATION_DIR/Benchmark/config/benchmark_configs.yaml"

BAG_PATH="${1:-}"
CONFIG_FILE="${2:-$DEFAULT_CONFIG}"

# ---- Resolve bag path ----
if [[ -z "$BAG_PATH" ]]; then
    # Auto-detect: use the newest bag in lapBags/
    if [[ -d "$DEFAULT_BAG_DIR" ]]; then
        BAG_PATH="$(ls -td "$DEFAULT_BAG_DIR"/*/ 2>/dev/null | head -1)"
    fi
    if [[ -z "$BAG_PATH" ]]; then
        echo "ERROR: No bag path provided and no bags found in $DEFAULT_BAG_DIR"
        echo "Usage: $0 [bag_path] [config_file]"
        exit 1
    fi
    echo "Auto-detected bag: $BAG_PATH"
fi

if [[ ! -d "$BAG_PATH" ]]; then
    echo "ERROR: Bag directory does not exist: $BAG_PATH"
    exit 1
fi

if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "ERROR: Config file not found: $CONFIG_FILE"
    exit 1
fi

# ---- Parse YAML with Python (avoids yq dependency) ----
read_config() {
    python3 -c "
import yaml, json, sys
with open('$CONFIG_FILE') as f:
    cfg = yaml.safe_load(f)
repeats = cfg.get('repeats', 1)
configs = cfg.get('configs', [])
# Output as JSON lines for easy bash parsing
print(json.dumps({'repeats': repeats, 'count': len(configs)}))
for c in configs:
    print(json.dumps(c))
"
}

CONFIG_OUTPUT=$(read_config)
HEADER=$(echo "$CONFIG_OUTPUT" | head -1)
REPEATS=$(echo "$HEADER" | python3 -c "import json,sys; print(json.load(sys.stdin)['repeats'])")
NUM_CONFIGS=$(echo "$HEADER" | python3 -c "import json,sys; print(json.load(sys.stdin)['count'])")
TOTAL_RUNS=$((NUM_CONFIGS * REPEATS))

echo "============================================================"
echo " AMCL Benchmark Sweep"
echo "============================================================"
echo " Bag:        $BAG_PATH"
echo " Config:     $CONFIG_FILE"
echo " Configs:    $NUM_CONFIGS"
echo " Repeats:    $REPEATS"
echo " Total runs: $TOTAL_RUNS"
echo "============================================================"
echo ""

# ---- Cooldown between runs (seconds) ----
COOLDOWN=10

RUN_NUMBER=0

# Read config lines (skip header)
CONFIG_LINES=$(echo "$CONFIG_OUTPUT" | tail -n +2)

for REPEAT in $(seq 1 "$REPEATS"); do
    CONFIG_INDEX=0
    while IFS= read -r CONFIG_JSON; do
        CONFIG_INDEX=$((CONFIG_INDEX + 1))
        RUN_NUMBER=$((RUN_NUMBER + 1))

        # Extract parameters from JSON
        AMCL_TYPE=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('amcl_type','nav2_amcl'))")
        MIN_P=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('min_particles',500))")
        MAX_P=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('max_particles',2000))")
        MAX_B=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('max_beams',120))")
        UPDATE_D=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('update_min_d','0.001'))")
        UPDATE_A=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('update_min_a','0.001'))")
        USE_KLD=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('use_kld','false'))")
        RECORD=$(echo "$CONFIG_JSON" | python3 -c "import json,sys; print(json.load(sys.stdin).get('record_output','false'))")

        echo "------------------------------------------------------------"
        echo " Run $RUN_NUMBER/$TOTAL_RUNS  (config $CONFIG_INDEX/$NUM_CONFIGS, repeat $REPEAT/$REPEATS)"
        echo " amcl_type=$AMCL_TYPE  particles=$MIN_P-$MAX_P  beams=$MAX_B  kld=$USE_KLD"
        echo "------------------------------------------------------------"

        ros2 launch f1tenth_localization amcl_bag_benchmark.launch.py \
            bag_path:="$BAG_PATH" \
            amcl_type:="$AMCL_TYPE" \
            min_particles:="$MIN_P" \
            max_particles:="$MAX_P" \
            max_beams:="$MAX_B" \
            update_min_d:="$UPDATE_D" \
            update_min_a:="$UPDATE_A" \
            use_kld:="$USE_KLD" \
            record_output:="$RECORD"

        echo ""
        echo " Run $RUN_NUMBER/$TOTAL_RUNS completed."

        if [[ $RUN_NUMBER -lt $TOTAL_RUNS ]]; then
            echo " Cooldown ${COOLDOWN}s before next run..."
            sleep "$COOLDOWN"
        fi

    done <<< "$CONFIG_LINES"
done

echo ""
echo "============================================================"
echo " Benchmark sweep complete: $TOTAL_RUNS runs finished."
echo " CSV results in: $LOCALIZATION_DIR/Benchmark/Matlab/csv/"
echo "============================================================"
