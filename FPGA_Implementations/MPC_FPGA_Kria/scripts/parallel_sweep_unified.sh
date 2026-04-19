#!/bin/bash
# Unified two-stage sweep: Stage 1 finds minimal bit-width, Stage 2 optimizes pragmas
# Usage: ./parallel_sweep_unified.sh [num_cores]
# Runs fully autonomously: ~30min stage1 + ~2hr stage2 = ~2.5-3 hours total

set -e

NUM_CORES=${1:-12}
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SWEEP_DIR="${BASE_DIR}/sweep_runs"
RESULTS_BASE="${BASE_DIR}/sweep_results_unified_$(date +%Y%m%d_%H%M%S)"
RESULTS_STAGE1="${RESULTS_BASE}_stage1.csv"
RESULTS_STAGE2="${RESULTS_BASE}_stage2.csv"
TIMESTAMP=$(date +%s)

echo "=== MPC FPGA HLS Unified Two-Stage Sweep ==="
echo "Stage 1 (20-30min): Find minimal feasible bit-width (baseline pragmas)"
echo "Stage 2 (90-120min): Optimize pragmas with minimal width found"
echo "Total runtime: ~2.5-3 hours at $NUM_CORES cores"
echo ""
echo "Base directory: $BASE_DIR"
echo "Results Stage 1: $RESULTS_STAGE1"
echo "Results Stage 2: $RESULTS_STAGE2"
echo ""

mkdir -p "$SWEEP_DIR"
source /tools/Xilinx/2025.1/Vitis/settings64.sh

# ============================================================================
# STAGE 1: FIND MINIMAL BIT-WIDTH (baseline pragmas 6/4)
# ============================================================================

echo "=== STAGE 1: BIT-WIDTH SWEEP (Baseline Pragmas 6/4) ==="
echo "Testing accumulator widths, guard bits, state width, K width combinations..."
echo ""

# Stage 1 candidates: All bit-width combinations with baseline pragmas
declare -a STAGE1_CANDIDATES=(
    # Baseline
    "base_24_12_20_32_32 24 12 20 32 32"
    
    # Reduce guard bits
    "guard_24_12_18_32_32 24 12 18 32 32"
    "guard_24_12_16_32_32 24 12 16 32 32"
    "guard_24_12_14_32_32 24 12 14 32 32"
    "guard_24_12_12_32_32 24 12 12 32 32"
    
    # Reduce accumulator width
    "acc_22_12_20_32_32 22 12 20 32 32"
    "acc_22_12_18_32_32 22 12 18 32 32"
    "acc_20_12_20_32_32 20 12 20 32 32"
    "acc_20_12_16_32_32 20 12 16 32 32"
    "acc_20_11_16_32_32 20 11 16 32 32"
    "acc_18_10_14_32_32 18 10 14 32 32"
    "acc_18_9_12_32_32 18 9 12 32 32"
    "acc_16_8_10_32_32 16 8 10 32 32"
    
    # Reduce state width
    "state_w_24_12_20_30_32 24 12 20 30 32"
    "state_w_24_12_20_28_32 24 12 20 28 32"
    "state_w_20_11_16_26_32 20 11 16 26 32"
    "state_w_18_9_12_24_32 18 9 12 24 32"
    
    # Reduce K width
    "k_w_24_12_20_32_28 24 12 20 32 28"
    "k_w_20_11_16_32_26 20 11 16 32 26"
    "k_w_18_9_12_28_24 18 9 12 28 24"
    
    # Extreme narrow
    "extreme_16_8_10_24_24 16 8 10 24 24"
    "extreme_14_7_8_22_20 14 7 8 22 20"
)

# Write Stage 1 CSV header
cat > "$RESULTS_STAGE1" << EOF
CAND_ID,ACC_WIDTH,ACC_INT,ACC_GUARD,STATE_W,K_W,CSIM_PASS,PASS_RATE,AVG_ITERS,STATUS
EOF

run_stage1_candidate() {
    local cand_id="$1"
    local acc_w="$2"
    local acc_i="$3"
    local acc_g="$4"
    local state_w="$5"
    local k_w="$6"
    
    local work_dir="${SWEEP_DIR}/s1_${cand_id}_${TIMESTAMP}"
    mkdir -p "$work_dir"
    export TMPDIR="${work_dir}/tmp"
    mkdir -p "$TMPDIR"
    
    echo "[$(date '+%H:%M:%S')] S1: Testing $cand_id (ACC=$acc_w/$acc_i/$acc_g STATE_W=$state_w K_W=$k_w)"
    
    local csim_log="${work_dir}/csim.log"
    export HLS_RUN_MODE=csim
    export HLS_ACC_WIDTH="$acc_w"
    export HLS_ACC_INT_BITS="$acc_i"
    export HLS_RAW_ACC_GUARD_BITS="$acc_g"
    export HLS_STATE_ZY_II=6
    export HLS_CTRL_ZY_II=4
    export HLS_STATE_WIDTH="$state_w"
    export HLS_K_WIDTH="$k_w"
    
    local csim_pass=0
    local pass_rate="N/A"
    local avg_iters="N/A"
    
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        csim_pass=1
        pass_rate=$(grep -oP 'success: \d+, \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
        avg_iters=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
    fi
    
    echo "$cand_id,$acc_w,$acc_i,$acc_g,$state_w,$k_w,$csim_pass,$pass_rate,$avg_iters,done" >> "$RESULTS_STAGE1"
    echo "[$(date '+%H:%M:%S')] S1: Done $cand_id (PASS=$csim_pass)"
    
    rm -rf "$work_dir"
}

export -f run_stage1_candidate
export SWEEP_DIR BASE_DIR RESULTS_STAGE1 TIMESTAMP

# Run Stage 1
printf '%s\n' "${STAGE1_CANDIDATES[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r id acc_w acc_i acc_g state_w k_w <<< "{}"; run_stage1_candidate "$id" "$acc_w" "$acc_i" "$acc_g" "$state_w" "$k_w"'

echo ""
echo "=== STAGE 1 COMPLETE ==="
echo "Stage 1 results: $RESULTS_STAGE1"
echo ""

# Find minimal working configuration (lowest bit-width that passes)
echo "Analyzing Stage 1 results..."
MINIMAL_CONFIG=$(awk -F',' 'NR>1 && $7==1 {print $2","$3","$4","$5","$6}' "$RESULTS_STAGE1" | \
    awk -F',' 'NR==1 {sum=$1+$2+$3+$4+$5; best=$0; best_sum=sum; next} {sum=$1+$2+$3+$4+$5; if(sum<best_sum) {best=$0; best_sum=sum}} END {print best}')

if [ -z "$MINIMAL_CONFIG" ]; then
    echo "ERROR: No configurations passed Stage 1!"
    exit 1
fi

# Parse minimal config
IFS=',' read -r MIN_ACC_W MIN_ACC_I MIN_ACC_G MIN_STATE_W MIN_K_W <<< "$MINIMAL_CONFIG"
echo "Minimal feasible configuration (lowest total bits): $MIN_ACC_W/$MIN_ACC_I/$MIN_ACC_G STATE_W=$MIN_STATE_W K_W=$MIN_K_W"
echo ""

# ============================================================================
# STAGE 2: OPTIMIZE PRAGMAS WITH MINIMAL WIDTH
# ============================================================================

echo "=== STAGE 2: PRAGMA SWEEP (With Minimal Width: $MINIMAL_CONFIG) ==="
echo "Testing II reductions and combinations with minimal bit-width..."
echo ""

# Stage 2 candidates: Pragma variations with minimal width from Stage 1
declare -a STAGE2_CANDIDATES=(
    # Baseline (6/4)
    "pragma_6_4 6 4"
    
    # State II reduction
    "pragma_5_4 5 4"
    "pragma_4_4 4 4"
    "pragma_3_4 3 4"
    
    # Control II reduction
    "pragma_6_3 6 3"
    "pragma_6_2 6 2"
    "pragma_6_1 6 1"
    
    # Aggressive combinations
    "pragma_5_3 5 3"
    "pragma_4_3 4 3"
    "pragma_4_2 4 2"
    "pragma_3_3 3 3"
    "pragma_3_2 3 2"
    "pragma_2_2 2 2"
    "pragma_2_1 2 1"
)

# Write Stage 2 CSV header
cat > "$RESULTS_STAGE2" << EOF
CAND_ID,STATE_ZY_II,CTRL_ZY_II,ACC_WIDTH,ACC_INT,ACC_GUARD,STATE_W,K_W,CSIM_PASS,PASS_RATE,AVG_ITERS,MIN_CYCLES,FMAX_MHZ,STATUS
EOF

run_stage2_candidate() {
    local cand_id="$1"
    local state_ii="$2"
    local ctrl_ii="$3"
    
    local work_dir="${SWEEP_DIR}/s2_${cand_id}_${TIMESTAMP}"
    mkdir -p "$work_dir"
    export TMPDIR="${work_dir}/tmp"
    mkdir -p "$TMPDIR"
    
    echo "[$(date '+%H:%M:%S')] S2: Testing $cand_id (STATE_II=$state_ii CTRL_II=$ctrl_ii)"
    
    local csim_log="${work_dir}/csim.log"
    export HLS_RUN_MODE=csim
    export HLS_ACC_WIDTH="$MIN_ACC_W"
    export HLS_ACC_INT_BITS="$MIN_ACC_I"
    export HLS_RAW_ACC_GUARD_BITS="$MIN_ACC_G"
    export HLS_STATE_ZY_II="$state_ii"
    export HLS_CTRL_ZY_II="$ctrl_ii"
    export HLS_STATE_WIDTH="$MIN_STATE_W"
    export HLS_K_WIDTH="$MIN_K_W"
    
    local csim_pass=0
    local pass_rate="N/A"
    local avg_iters="N/A"
    
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        csim_pass=1
        pass_rate=$(grep -oP 'success: \d+, \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
        avg_iters=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
    fi
    
    # Run synth (only if csim passed)
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
    
    echo "$cand_id,$state_ii,$ctrl_ii,$MIN_ACC_W,$MIN_ACC_I,$MIN_ACC_G,$MIN_STATE_W,$MIN_K_W,$csim_pass,$pass_rate,$avg_iters,$min_cycles,$fmax,done" >> "$RESULTS_STAGE2"
    echo "[$(date '+%H:%M:%S')] S2: Done $cand_id (PASS=$csim_pass CYCLES=$min_cycles FMAX=$fmax)"
    
    rm -rf "$work_dir"
}

export -f run_stage2_candidate
export MIN_ACC_W MIN_ACC_I MIN_ACC_G MIN_STATE_W MIN_K_W

# Run Stage 2
printf '%s\n' "${STAGE2_CANDIDATES[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r id state_ii ctrl_ii <<< "{}"; run_stage2_candidate "$id" "$state_ii" "$ctrl_ii"'

echo ""
echo "=== SWEEP COMPLETE ==="
echo ""
echo "Stage 1 results (bit-width optimization): $RESULTS_STAGE1"
echo "Stage 2 results (pragma optimization): $RESULTS_STAGE2"
echo ""
echo "Summary:"
echo "  Minimal feasible width: $MINIMAL_CONFIG"
echo ""
echo "Top 5 Stage 2 results (by latency, csim_pass=1):"
(head -1 "$RESULTS_STAGE2"; grep "^.*,1," "$RESULTS_STAGE2" | sort -t',' -k12 -n | head -5) | column -t -s','
echo ""
echo "Recommended configuration:"
BEST=$(grep "^.*,1," "$RESULTS_STAGE2" | sort -t',' -k12 -n | head -1)
if [ -n "$BEST" ]; then
    echo "$BEST"
else
    echo "No passing Stage 2 configurations found!"
fi
