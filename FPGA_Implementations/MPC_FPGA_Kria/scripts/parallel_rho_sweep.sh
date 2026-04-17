#!/bin/bash
# Parallel RHO tuning sweep across all precision configurations
# Uses xargs for parallelization (same pattern as parallel_sweep_3stage.sh)
# Usage: ./parallel_rho_sweep.sh [num_cores]
# Example: ./parallel_rho_sweep.sh 16
# Sweeps Q32.16, Q30.15, Q28.14, Q26.13 with RHO tuning
# Total configurations: 4 precisions × 30 RHO values = 120 runs

set -e

NUM_CORES=${1:-16}
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SWEEP_DIR="${BASE_DIR}/sweep_runs"
RESULTS_FILE="${BASE_DIR}/sweep_results_rho_$(date +%Y%m%d_%H%M%S).csv"
TIMESTAMP=$(date +%s)

echo "=== MPC FPGA HLS RHO Tuning Sweep ==="
echo "Testing RHO across precisions Q32.16, Q30.15, Q28.14, Q26.13"
echo "RHO range: 10.0 to 25.0 in 0.5 steps = 30 values per precision"
echo "Total configurations: 4 precisions × 30 RHO values = 120 runs"
echo "Parallelization: $NUM_CORES cores using xargs"
echo ""
echo "Results file: $RESULTS_FILE"
echo ""

mkdir -p "$SWEEP_DIR"

# Source Xilinx environment once at top level
source /tools/Xilinx/2025.1/Vitis/settings64.sh 2>/dev/null || true

# Optional acceleration for sweeps (e.g., SWEEP_SIM_DURATION=20).
# If unset, testbench default SIM_DURATION is used.
if [ -n "${SWEEP_SIM_DURATION:-}" ]; then
    export SIM_DURATION="$SWEEP_SIM_DURATION"
fi

# ============================================================================
# RHO SWEEP CONFIGURATION
# ============================================================================

# Precision configurations: WIDTH:INT_BITS
declare -a PRECISIONS=(
    "32:16"   # Q32.16 - baseline
    "30:15"   # Q30.15
    "28:14"   # Q28.14
    "26:13"   # Q26.13
)

PRECISION_NAMES=("Q32.16" "Q30.15" "Q28.14" "Q26.13")

# RHO values to test (10.0 to 25.0 in 0.5 increments)
declare -a RHO_VALUES=(
    "10.0" "10.5" "11.0" "11.5" "12.0" "12.5" "13.0" "13.5" "14.0" "14.5"
    "15.0" "15.5" "16.0" "16.5" "17.0" "17.5" "18.0" "18.5" "19.0" "19.5"
    "20.0" "20.5" "21.0" "21.5" "22.0" "22.5" "23.0" "23.5" "24.0" "24.5" "25.0"
)

# RHO_U/RHO ratio from baseline (24/18 = 1.333...)
RHO_U_RATIO=1.33333

# ============================================================================
# CSV HEADER
# ============================================================================

cat > "$RESULTS_FILE" << EOF
precision,width,int_bits,rho,rho_u,csim_passed,csim_failed,avg_iters,max_iters,fpga_time_us,testbench_status
EOF

echo "CSV header written to $RESULTS_FILE"
echo ""

# ============================================================================
# PARALLEL EXECUTION FUNCTION (same as parallel_sweep_3stage.sh pattern)
# ============================================================================

run_rho_candidate() {
    # Takes: name width int_bits rho rho_u
    local precision_name="$1"
    local width="$2"
    local int_bits="$3"
    local rho_value="$4"
    local rho_u="$5"
    
    # Create work directory with PID for parallelization safety
    local work_dir="${SWEEP_DIR}/rho_${precision_name}_rho${rho_value}_${TIMESTAMP}_$$"
    mkdir -p "$work_dir"
    export TMPDIR="${work_dir}/tmp"
    mkdir -p "$TMPDIR"
    
    # Run csim with this configuration
    local csim_log="${work_dir}/csim.log"
    
    export HLS_RUN_MODE=csim
    export HLS_RICCATI_WIDTH="$width"
    export HLS_RICCATI_INT_BITS="$int_bits"
    export MPC_FPGA_ADMM_RHO="$rho_value"
    export MPC_FPGA_ADMM_RHO_U="$rho_u"
    
    # Run HLS simulation
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        # Parse testbench output
        local passed=0
        local failed=0
        local avg_iters=""
        local max_iters=""
        local fpga_time=""
        local status="COMPLETE"
        
        # Extract results from testbench output
        if grep -q "=== RESULTS:" "$csim_log"; then
            passed=$(grep "=== RESULTS:" "$csim_log" | grep -oP '\d+(?= passed)' | tail -1)
            failed=$(grep "=== RESULTS:" "$csim_log" | grep -oP '\d+(?= failed)' | tail -1)
        fi
        
        # Try to extract iteration statistics
        if grep -q "Avg iterations:" "$csim_log"; then
            avg_iters=$(grep "Avg iterations:" "$csim_log" | grep -oP '[0-9]+\.[0-9]+|[0-9]+' | tail -1)
            max_iters=$(grep "Max iterations:" "$csim_log" | grep -oP '\d+' | tail -1)
        fi
        
        # Try to extract FPGA time
        if grep -q "FPGA time:" "$csim_log"; then
            fpga_time=$(grep "FPGA time:" "$csim_log" | grep -oP '\d+' | tail -1)
        fi
        
        # Determine status
        if [[ $passed -ge 5 ]]; then
            status="PASS"
        elif [[ $passed -ge 4 ]]; then
            status="MARGINAL"
        else
            status="FAIL"
        fi
        
        # Write result to CSV
        echo "$precision_name,$width,$int_bits,$rho_value,$rho_u,$passed,$failed,$avg_iters,$max_iters,$fpga_time,$status" >> "$RESULTS_FILE"
        
        echo "[$(date '+%H:%M:%S')] $precision_name RHO=$rho_value: $passed/$failed, avg_iter=$avg_iters, status=$status"
    else
        echo "[$(date '+%H:%M:%S')] $precision_name RHO=$rho_value: COMPILATION FAILED"
        echo "$precision_name,$width,$int_bits,$rho_value,$rho_u,0,6,,,ERROR" >> "$RESULTS_FILE"
    fi
    
    # Cleanup temp directory
    rm -rf "$work_dir"
}

export -f run_rho_candidate
export BASE_DIR SWEEP_DIR RHO_U_RATIO PRECISIONS PRECISION_NAMES TIMESTAMP RESULTS_FILE

# ============================================================================
# GENERATE ALL TEST CONFIGURATIONS
# ============================================================================

echo "=== Generating test configurations ==="

declare -a TEST_CONFIGS=()

for prec_idx in "${!PRECISIONS[@]}"; do
    # Get precision from array
    precision_info="${PRECISIONS[$prec_idx]}"
    IFS=':' read -r width int_bits <<< "$precision_info"
    precision_name="${PRECISION_NAMES[$prec_idx]}"
    
    for rho in "${RHO_VALUES[@]}"; do
        # Calculate RHO_U
        rho_u=$(printf "%.2f" "$(echo "$rho * $RHO_U_RATIO" | bc -l)")
        # Store: "name width int_bits rho rho_u"
        TEST_CONFIGS+=("$precision_name $width $int_bits $rho $rho_u")
    done
done

TOTAL_CONFIGS=${#TEST_CONFIGS[@]}
echo "Total configurations to test: $TOTAL_CONFIGS"
echo ""

# ============================================================================
# RUN SWEEP WITH XARGS PARALLELIZATION
# ============================================================================

echo "=== Starting parallel sweep ($NUM_CORES cores) ==="
echo "Estimated time: ~$((TOTAL_CONFIGS / NUM_CORES)) seconds"
echo ""

START_TIME=$(date +%s)

# Use xargs to run sweeps in parallel (same pattern as parallel_sweep_3stage.sh)
printf '%s\n' "${TEST_CONFIGS[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r name w i r ru <<< "{}"; run_rho_candidate "$name" "$w" "$i" "$r" "$ru"'

END_TIME=$(date +%s)
ELAPSED=$((END_TIME - START_TIME))

echo ""
echo "=== Sweep Complete ==="
echo "Total time: $ELAPSED seconds (~$((ELAPSED / 60)) minutes)"
echo "Results written to: $RESULTS_FILE"
echo ""

# ============================================================================
# ANALYSIS
# ============================================================================

echo "=== Analysis ==="
echo ""

# Count results by status
echo "Results summary:"
grep -v "precision," "$RESULTS_FILE" | awk -F',' '{print $NF}' | sort | uniq -c
echo ""

# Find best RHO for each precision (highest passed tests)
echo "Best configurations by precision (highest passed):"
for prec in "${PRECISION_NAMES[@]}"; do
    best_line=$(grep "^$prec," "$RESULTS_FILE" | sort -t',' -k6 -nr | head -1)
    if [[ -n "$best_line" ]]; then
        echo "$best_line"
    fi
done
echo ""

# Show Q32 configurations
echo "Q32.16 sweep results:"
grep "^Q32.16," "$RESULTS_FILE" | grep "PASS," | sort -t',' -k8 -n | head -5
echo ""

# Identify top performers (PASS status, lowest avg_iters)
echo "Top 10 best configurations (PASS + lowest avg_iters):"
grep "PASS," "$RESULTS_FILE" | grep -v ",,$" | sort -t',' -k8 -n | head -10
echo ""

echo "Full results: cat $RESULTS_FILE"
