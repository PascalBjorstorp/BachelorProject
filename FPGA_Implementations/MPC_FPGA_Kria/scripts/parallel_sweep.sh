#!/bin/bash
# Parallel pragma and bit-width sweep for MPC FPGA HLS optimization
# Usage: ./parallel_sweep.sh [num_cores]

set -e

NUM_CORES=${1:-12}
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SWEEP_DIR="${BASE_DIR}/sweep_runs"
RESULTS_CSV="${BASE_DIR}/sweep_results_$(date +%Y%m%d_%H%M%S).csv"
TIMESTAMP=$(date +%s)

echo "=== MPC FPGA HLS Parallel Sweep ===" 
echo "Base directory: $BASE_DIR"
echo "Sweep output: $SWEEP_DIR"
echo "Results CSV: $RESULTS_CSV"
echo "Using $NUM_CORES cores"
echo ""

# Create sweep directory
mkdir -p "$SWEEP_DIR"

# Source Xilinx tools
source /tools/Xilinx/2025.1/Vitis/settings64.sh

# Define sweep parameters grid
# Format: CANDIDATE_ID STATE_ZY_II CTRL_ZY_II ACC_WIDTH ACC_INT ACC_GUARD
declare -a CANDIDATES=(
    # Baseline (current working state)
    "base_6_4_24_12_20 6 4 24 12 20"
    
    # State path II reduction (keeping ctrl at 4)
    "state_5_4_24_12_20 5 4 24 12 20"
    "state_4_4_24_12_20 4 4 24 12 20"
    "state_3_4_24_12_20 3 4 24 12 20"
    
    # Ctrl path II reduction (keeping state at 6)
    "ctrl_6_3_24_12_20 6 3 24 12 20"
    "ctrl_6_2_24_12_20 6 2 24 12 20"
    
    # Aggressive combined II reductions
    "comb_5_3_24_12_20 5 3 24 12 20"
    "comb_4_3_24_12_20 4 3 24 12 20"
    "comb_4_2_24_12_20 4 2 24 12 20"
    
    # Bit-width variations (with good II)
    "width_5_3_26_12_20 5 3 26 12 20"
    "width_5_3_26_13_20 5 3 26 13 20"
    "width_5_3_28_13_20 5 3 28 13 20"
    "width_4_3_26_12_20 4 3 26 12 20"
)

# Write CSV header
cat > "$RESULTS_CSV" << EOF
CANDIDATE_ID,STATE_ZY_II,CTRL_ZY_II,ACC_WIDTH,ACC_INT,ACC_GUARD,CSIM_PASS,CSIM_PASS_RATE,AVG_ITERS,MIN_CYCLES,FMAX_MHZ,STATUS
EOF

# Function to run single candidate
run_candidate() {
    local cand_id="$1"
    local state_ii="$2"
    local ctrl_ii="$3"
    local acc_width="$4"
    local acc_int="$5"
    local acc_guard="$6"
    
    local work_dir="${SWEEP_DIR}/${cand_id}_${TIMESTAMP}"
    mkdir -p "$work_dir"
    
    # Set unique temp directory for this run to avoid collisions
    export TMPDIR="${work_dir}/tmp"
    mkdir -p "$TMPDIR"
    
    echo "[$(date '+%H:%M:%S')] Starting: $cand_id (STATE_II=$state_ii, CTRL_II=$ctrl_ii, WIDTH=$acc_width/$acc_int/$acc_guard)"
    
    # Run csim first (faster validation)
    local csim_log="${work_dir}/csim.log"
    export HLS_RUN_MODE=csim
    export HLS_ACC_WIDTH="$acc_width"
    export HLS_ACC_INT_BITS="$acc_int"
    export HLS_RAW_ACC_GUARD_BITS="$acc_guard"
    export HLS_STATE_ZY_II="$state_ii"
    export HLS_CTRL_ZY_II="$ctrl_ii"
    
    local csim_pass=0
    local pass_rate="N/A"
    local avg_iters="N/A"
    
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        csim_pass=1
        pass_rate=$(grep -oP 'success: \d+, \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
        avg_iters=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
    fi
    
    # Run synth to get latency/Fmax (only if csim passed)
    local min_cycles="N/A"
    local fmax="N/A"
    if [ "$csim_pass" -eq 1 ]; then
        local synth_log="${work_dir}/synth.log"
        export HLS_RUN_MODE=synth
        if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$synth_log" 2>&1); then
            min_cycles=$(grep -oP 'min_latency: \K[0-9]+' "$synth_log" | head -1 || echo "N/A")
            fmax=$(grep -oP 'Target frequency: \K[0-9.]+' "$synth_log" | head -1 || echo "N/A")
        fi
    fi
    
    # Append result to CSV (atomic)
    echo "${cand_id},${state_ii},${ctrl_ii},${acc_width},${acc_int},${acc_guard},${csim_pass},${pass_rate},${avg_iters},${min_cycles},${fmax},done" >> "$RESULTS_CSV"
    
    echo "[$(date '+%H:%M:%S')] Completed: $cand_id (CSIM=$csim_pass, CYCLES=$min_cycles, FMAX=$fmax)"
    
    # Clean up to save space
    rm -rf "$work_dir"
}

export -f run_candidate
export SWEEP_DIR BASE_DIR RESULTS_CSV TIMESTAMP

# Launch all candidates in parallel
echo "Launching ${#CANDIDATES[@]} candidates across $NUM_CORES cores..."
printf '%s\n' "${CANDIDATES[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r id s_ii c_ii acc_w acc_i acc_g <<< "{}"; run_candidate "$id" "$s_ii" "$c_ii" "$acc_w" "$acc_i" "$acc_g"'

echo ""
echo "=== Sweep Complete ==="
echo "Results saved to: $RESULTS_CSV"
echo ""
echo "Top 5 candidates (by latency, csim_pass=1):"
(head -1 "$RESULTS_CSV"; grep "^.*,1," "$RESULTS_CSV" | sort -t',' -k10 -n | head -5) | column -t -s','
#!/bin/bash
# Parallel pragma and bit-width sweep for MPC FPGA HLS optimization
# Usage: ./parallel_sweep.sh [num_cores]

set -e

NUM_CORES=${1:-12}
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SWEEP_DIR="${BASE_DIR}/sweep_runs"
RESULTS_CSV="${BASE_DIR}/sweep_results_$(date +%Y%m%d_%H%M%S).csv"

echo "=== MPC FPGA HLS Parallel Sweep ===" 
echo "Base directory: $BASE_DIR"
echo "Sweep output: $SWEEP_DIR"
echo "Results CSV: $RESULTS_CSV"
echo "Using $NUM_CORES cores"
echo ""

# Clean old sweep directory
rm -rf "$SWEEP_DIR"
mkdir -p "$SWEEP_DIR"

# Source Xilinx tools
source /tools/Xilinx/2025.1/Vitis/settings64.sh

# Define sweep parameters grid
# Format: CANDIDATE_ID STATE_ZY_II CTRL_ZY_II ACC_WIDTH ACC_INT ACC_GUARD
declare -a CANDIDATES=(
    # Baseline (current working state)
    "base_6_4_24_12_20 6 4 24 12 20"
    
    # State path II reduction (keeping ctrl at 4)
    "state_5_4_24_12_20 5 4 24 12 20"
    "state_4_4_24_12_20 4 4 24 12 20"
    "state_3_4_24_12_20 3 4 24 12 20"
    
    # Ctrl path II reduction (keeping state at 6)
    "ctrl_6_3_24_12_20 6 3 24 12 20"
    "ctrl_6_2_24_12_20 6 2 24 12 20"
    
    # Aggressive combined II reductions
    "comb_5_3_24_12_20 5 3 24 12 20"
    "comb_4_3_24_12_20 4 3 24 12 20"
    "comb_4_2_24_12_20 4 2 24 12 20"
    
    # Bit-width variations (with good II)
    "width_5_3_26_12_20 5 3 26 12 20"
    "width_5_3_26_13_20 5 3 26 13 20"
    "width_5_3_28_13_20 5 3 28 13 20"
    "width_4_3_26_12_20 4 3 26 12 20"
)

# Write CSV header
cat > "$RESULTS_CSV" << EOF
CANDIDATE_ID,STATE_ZY_II,CTRL_ZY_II,ACC_WIDTH,ACC_INT,ACC_GUARD,CSIM_PASS,CSIM_PASS_RATE,AVG_ITERS,MIN_CYCLES,FMAX_MHZ,STATUS
EOF

# Function to run single candidate
run_candidate() {
    local cand_id="$1"
    local state_ii="$2"
    local ctrl_ii="$3"
    local acc_width="$4"
    local acc_int="$5"
    local acc_guard="$6"
    
    local cand_dir="${SWEEP_DIR}/${cand_id}"
    mkdir -p "$cand_dir"
    
    echo "[$(date '+%H:%M:%S')] Starting: $cand_id (STATE_II=$state_ii, CTRL_II=$ctrl_ii, WIDTH=$acc_width/$acc_int/$acc_guard)"
    
    # Copy HLS project to candidate directory
    cp -r "${BASE_DIR}/mpc_fpga_hls" "${cand_dir}/"
    
    # Run csim + synth in the cloned project
    cd "${cand_dir}"
    
    # Run csim first (faster validation)
    CSIM_LOG="${cand_dir}/csim.log"
    export HLS_RUN_MODE=csim
    export HLS_ACC_WIDTH="$acc_width"
    export HLS_ACC_INT_BITS="$acc_int"
    export HLS_RAW_ACC_GUARD_BITS="$acc_guard"
    export HLS_STATE_ZY_II="$state_ii"
    export HLS_CTRL_ZY_II="$ctrl_ii"
    
    if vitis-run --mode hls --tcl "${BASE_DIR}/scripts/run_hls.tcl" > "$CSIM_LOG" 2>&1; then
        CSIM_PASS=1
        PASS_RATE=$(grep -oP 'success: \d+, \K[0-9.]+' "$CSIM_LOG" | head -1 || echo "N/A")
        AVG_ITERS=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$CSIM_LOG" | head -1 || echo "N/A")
    else
        CSIM_PASS=0
        PASS_RATE="0.0"
        AVG_ITERS="N/A"
    fi
    
    # Run synth to get latency/Fmax
    SYNTH_LOG="${cand_dir}/synth.log"
    export HLS_RUN_MODE=synth
    
    MIN_CYCLES="N/A"
    FMAX="N/A"
    if vitis-run --mode hls --tcl "${BASE_DIR}/scripts/run_hls.tcl" > "$SYNTH_LOG" 2>&1; then
        MIN_CYCLES=$(grep -oP 'mpc_fpga_top.*?min: \K[0-9]+' "$SYNTH_LOG" | head -1 || echo "N/A")
        FMAX=$(grep -oP 'Target frequency: \K[0-9.]+' "$SYNTH_LOG" | head -1 || echo "N/A")
    fi
    
    # Append result to CSV
    echo "${cand_id},${state_ii},${ctrl_ii},${acc_width},${acc_int},${acc_guard},${CSIM_PASS},${PASS_RATE},${AVG_ITERS},${MIN_CYCLES},${FMAX},done" >> "$RESULTS_CSV"
    
    echo "[$(date '+%H:%M:%S')] Completed: $cand_id (CSIM=$CSIM_PASS, CYCLES=$MIN_CYCLES, FMAX=$FMAX)"
    
    # Clean up to save space
    rm -rf "${cand_dir}/mpc_fpga_hls"
}

export -f run_candidate
export SWEEP_DIR BASE_DIR RESULTS_CSV

# Launch all candidates in parallel using GNU parallel or xargs
echo "Launching $NUM_CORES parallel jobs..."
parallel --jobs "$NUM_CORES" --line-buffer run_candidate ::: "${CANDIDATES[@]}"

echo ""
echo "=== Sweep Complete ==="
echo "Results saved to: $RESULTS_CSV"
echo ""
echo "Top 5 candidates (by latency, csim_pass=1):"
(head -1 "$RESULTS_CSV"; grep "^.*,1," "$RESULTS_CSV" | sort -t',' -k9 -n | head -5) | column -t -s','

