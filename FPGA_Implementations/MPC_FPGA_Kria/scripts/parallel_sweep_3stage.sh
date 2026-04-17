#!/bin/bash
# THREE-STAGE COMPREHENSIVE SWEEP: Widths → Pragmas → Refinement
# Stage 1: Find minimal Riccati+Accum (35 candidates, ~1 hour)
# Stage 2: Optimize pragmas with minimal widths (includes M_BT_P_II)
# Stage 3: Refine around Stage-2 winner with guard/II neighborhood
# Total: ~60 candidates, 3-4 hours @ 16 cores

set -e

NUM_CORES=${1:-16}
BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SWEEP_DIR="${BASE_DIR}/sweep_runs"
RESULTS_BASE="${BASE_DIR}/sweep_results_3stage_$(date +%Y%m%d_%H%M%S)"
RESULTS_S1="${RESULTS_BASE}_stage1_widths.csv"
RESULTS_S2="${RESULTS_BASE}_stage2_pragmas.csv"
RESULTS_S3="${RESULTS_BASE}_stage3_stateK.csv"
TIMESTAMP=$(date +%s)

echo "=== MPC FPGA HLS THREE-STAGE COMPREHENSIVE SWEEP ==="
echo "Stage 1 (60min): Minimal Riccati+Accum widths"
echo "Stage 2 (60min): Pragma optimization"
echo "Stage 3 (30min): Local refinement"
echo "Total: ~3-4 hours @ $NUM_CORES cores"
echo ""

mkdir -p "$SWEEP_DIR"
source /tools/Xilinx/2025.1/Vitis/settings64.sh

# Optional acceleration for sweeps (e.g., SWEEP_SIM_DURATION=20).
# If unset, testbench default SIM_DURATION is used.
if [ -n "${SWEEP_SIM_DURATION:-}" ]; then
    export SIM_DURATION="$SWEEP_SIM_DURATION"
fi

# ============================================================================
# STAGE 1: RICCATI + ACCUMULATOR WIDTH SWEEP (with rational constraints)
# ============================================================================

echo "=== STAGE 1: FIND MINIMAL RICCATI + ACCUMULATOR WIDTHS ==="
echo "Generating exhaustive rational grid (target: multi-hour run on 16 cores)"
echo ""

declare -a STAGE1_CANDIDATES=()
if [ -n "${STAGE1_RICCATI_PAIRS:-}" ]; then
    IFS=';' read -r -a riccati_pairs <<< "$STAGE1_RICCATI_PAIRS"
else
    riccati_pairs=("32 16" "30 15" "28 14" "26 13" "24 12" "22 11" "20 10")
fi
if [ -n "${STAGE1_ACCUM_PAIRS:-}" ]; then
    IFS=';' read -r -a accum_pairs <<< "$STAGE1_ACCUM_PAIRS"
else
    accum_pairs=("24 12" "22 11" "20 10" "18 9" "16 8" "14 7")
fi
if [ -n "${STAGE1_GUARD_BITS:-}" ]; then
    read -r -a guard_bits <<< "$STAGE1_GUARD_BITS"
else
    guard_bits=(20 18 16 14 12)
fi

for rp in "${riccati_pairs[@]}"; do
    read -r rw ri <<< "$rp"
    for ap in "${accum_pairs[@]}"; do
        read -r aw ai <<< "$ap"
        for g in "${guard_bits[@]}"; do
            id="r${rw}i${ri}_a${aw}i${ai}_g${g}"
            STAGE1_CANDIDATES+=("$id $rw $ri $aw $ai $g")
        done
    done
done

echo "Stage 1 candidate count: ${#STAGE1_CANDIDATES[@]}"

cat > "$RESULTS_S1" << EOF
CAND_ID,RICCATI_W,RICCATI_I,ACCUM_W,ACCUM_I,GUARD,CSIM_PASS,PASS_RATE,AVG_ITERS,STATUS
EOF

run_stage1_candidate() {
    local cand_id="$1" riccati_w="$2" riccati_i="$3" accum_w="$4" accum_i="$5" guard="$6"
    local work_dir="${SWEEP_DIR}/s1_${cand_id}_${TIMESTAMP}"
    mkdir -p "$work_dir"; export TMPDIR="${work_dir}/tmp"; mkdir -p "$TMPDIR"
    
    echo "[$(date '+%H:%M:%S')] S1: $cand_id"
    
    export HLS_RUN_MODE=csim HLS_RICCATI_WIDTH="$riccati_w" HLS_RICCATI_INT_BITS="$riccati_i"
    export HLS_ACCUM_WIDTH="$accum_w" HLS_ACCUM_INT_BITS="$accum_i" HLS_RAW_ACC_GUARD_BITS="$guard"
    export HLS_STATE_ZY_II=6 HLS_CTRL_ZY_II=4 HLS_M_BT_P_II=3
    
    local csim_log="${work_dir}/csim.log" csim_pass=0 pass_rate="N/A" avg_iters="N/A"
    # Run vitis in the subshell and capture the output
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        csim_pass=1
    else
        # Require full testbench success when exit code is nonzero.
        if [ -f "$csim_log" ] && grep -q "=== RESULTS: 6 passed, 0 failed ===" "$csim_log"; then
            csim_pass=1
        else
            # Log failed - save debug info
            {
                echo "DEBUG: vitis-run failed for $cand_id"
                echo "csim_log path: $csim_log"
                echo "csim_log exists: $([ -f "$csim_log" ] && echo yes || echo no)"
                if [ -f "$csim_log" ]; then
                    echo "csim_log size: $(wc -c < "$csim_log")"
                    echo "=== LAST 20 LINES OF csim_log ==="
                    tail -20 "$csim_log"
                fi
            } >> "${work_dir}/DEBUG.log"
        fi
    fi
    if [ "$csim_pass" -eq 1 ]; then
        pass_rate=$(grep -oP 'success: \d+, \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
        avg_iters=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
    fi
    
    echo "$cand_id,$riccati_w,$riccati_i,$accum_w,$accum_i,$guard,$csim_pass,$pass_rate,$avg_iters,done" >> "$RESULTS_S1"
    rm -rf "$work_dir"
}

export -f run_stage1_candidate
export SWEEP_DIR BASE_DIR RESULTS_S1 TIMESTAMP

printf '%s\n' "${STAGE1_CANDIDATES[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r id r_w r_i a_w a_i g <<< "{}"; run_stage1_candidate "$id" "$r_w" "$r_i" "$a_w" "$a_i" "$g"'

# Find minimal configuration
MINIMAL=$(awk -F',' 'NR>1 && $7==1 {print $2","$3","$4","$5","$6}' "$RESULTS_S1" | \
    awk -F',' 'NR==1 {sum=$1+$2+$3+$4+$5; best=$0; best_sum=sum; next} {sum=$1+$2+$3+$4+$5; if(sum<best_sum) {best=$0; best_sum=sum}} END {print best}')

if [ -z "$MINIMAL" ]; then echo "ERROR: No Stage 1 passes!"; exit 1; fi
IFS=',' read -r MIN_RW MIN_RI MIN_AW MIN_AI MIN_G <<< "$MINIMAL"

echo ""; echo "Stage 1 Complete: Minimal widths found"
echo "  Riccati: $MIN_RW/$MIN_RI, Accum: $MIN_AW/$MIN_AI, Guard: $MIN_G"
echo ""

if [ -n "${SWEEP_STOP_AFTER_STAGE1:-}" ]; then
    echo "SWEEP_STOP_AFTER_STAGE1 is set; stopping after Stage 1."
    exit 0
fi

# ============================================================================
# STAGE 2: PRAGMA (II) SWEEP with minimal widths
# ============================================================================

echo "=== STAGE 2: PRAGMA OPTIMIZATION ==="
echo "Sweeping full II grid (state/control/M_BT_P)"
echo ""

declare -a STAGE2_CANDIDATES=()
for s_ii in 6 5 4 3 2; do
    for c_ii in 4 3 2 1; do
        for m_ii in 4 3 2; do
            id="pragma_s${s_ii}_c${c_ii}_m${m_ii}"
            STAGE2_CANDIDATES+=("$id $s_ii $c_ii $m_ii")
        done
    done
done

echo "Stage 2 candidate count: ${#STAGE2_CANDIDATES[@]}"

cat > "$RESULTS_S2" << EOF
CAND_ID,STATE_II,CTRL_II,M_BTP_II,RICCATI_W,RICCATI_I,ACCUM_W,ACCUM_I,GUARD,CSIM_PASS,PASS_RATE,AVG_ITERS,MIN_CYCLES,FMAX,STATUS
EOF

run_stage2_candidate() {
    local cand_id="$1" state_ii="$2" ctrl_ii="$3" m_btp_ii="$4"
    local work_dir="${SWEEP_DIR}/s2_${cand_id}_${TIMESTAMP}"
    mkdir -p "$work_dir"; export TMPDIR="${work_dir}/tmp"; mkdir -p "$TMPDIR"
    
    echo "[$(date '+%H:%M:%S')] S2: $cand_id"
    
    export HLS_RUN_MODE=csim HLS_STATE_ZY_II="$state_ii" HLS_CTRL_ZY_II="$ctrl_ii" HLS_M_BT_P_II="$m_btp_ii"
    export HLS_RICCATI_WIDTH="$MIN_RW" HLS_RICCATI_INT_BITS="$MIN_RI"
    export HLS_ACCUM_WIDTH="$MIN_AW" HLS_ACCUM_INT_BITS="$MIN_AI" HLS_RAW_ACC_GUARD_BITS="$MIN_G"
    
    local csim_log="${work_dir}/csim.log" csim_pass=0 pass_rate="N/A" avg_iters="N/A"
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        csim_pass=1
    else
        if [ -f "$csim_log" ] && grep -q "=== RESULTS: 6 passed, 0 failed ===" "$csim_log"; then
            csim_pass=1
        else
            { echo "DEBUG S2: $cand_id failed"; tail -10 "$csim_log" 2>/dev/null; } >> "${work_dir}/DEBUG.log"
        fi
    fi
    if [ "$csim_pass" -eq 1 ]; then
        pass_rate=$(grep -oP 'success: \d+, \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
        avg_iters=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
    fi
    
    local min_cycles="N/A" fmax="N/A"
    if [ "$csim_pass" -eq 1 ]; then
        export HLS_RUN_MODE=synth
        local synth_log="${work_dir}/synth.log"
        if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$synth_log" 2>&1); then
            min_cycles=$(grep -oP 'min_latency: \K[0-9]+' "$synth_log" | head -1 || echo "N/A")
            fmax=$(grep -oP 'Target frequency: \K[0-9.]+' "$synth_log" | head -1 || echo "N/A")
        fi
    fi
    
    echo "$cand_id,$state_ii,$ctrl_ii,$m_btp_ii,$MIN_RW,$MIN_RI,$MIN_AW,$MIN_AI,$MIN_G,$csim_pass,$pass_rate,$avg_iters,$min_cycles,$fmax,done" >> "$RESULTS_S2"
    rm -rf "$work_dir"
}

export -f run_stage2_candidate
export MIN_RW MIN_RI MIN_AW MIN_AI MIN_G

printf '%s\n' "${STAGE2_CANDIDATES[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r id s_ii c_ii m_ii <<< "{}"; run_stage2_candidate "$id" "$s_ii" "$c_ii" "$m_ii"'

# Find best pragma combo by minimum synthesized latency among csim passes
BEST_PRAGMA=$(awk -F',' 'NR>1 && $10==1 && $13 != "N/A" {print $2","$3","$4","$13}' "$RESULTS_S2" | \
    awk -F',' 'NR==1 {best=$1","$2","$3; best_cycles=$4+0; next} {c=$4+0; if(c<best_cycles){best=$1","$2","$3; best_cycles=c}} END {print best}')

if [ -z "$BEST_PRAGMA" ]; then
    echo "WARNING: No Stage 2 passes, using baseline 6/4"
    BEST_PRAGMA="6,4,3"
fi

IFS=',' read -r BEST_S_II BEST_C_II BEST_M_BTP_II <<< "$BEST_PRAGMA"

echo ""; echo "Stage 2 Complete: Best pragma found"
echo "  STATE_II=$BEST_S_II, CTRL_II=$BEST_C_II, M_BTP_II=$BEST_M_BTP_II"
echo ""

# ============================================================================
# STAGE 3: LOCAL REFINEMENT around best pragma point
# ============================================================================

echo "=== STAGE 3: LOCAL REFINEMENT ==="
echo "Guard-bit and II neighborhood refinement around Stage-2 winner"
echo ""

declare -a STAGE3_CANDIDATES=()
for dg in -4 -2 0 2 4; do
    for ds in -1 0 1; do
        for dc in -1 0 1; do
            m_ii=${BEST_M_BTP_II}
            g=$((MIN_G+dg))
            s=$((BEST_S_II+ds))
            c=$((BEST_C_II+dc))
            id="ref_s${s}_c${c}_m${m_ii}_g${g}"
            STAGE3_CANDIDATES+=("$id $s $c $m_ii $g")
        done
    done
done

echo "Stage 3 candidate count: ${#STAGE3_CANDIDATES[@]}"

cat > "$RESULTS_S3" << EOF
CAND_ID,STATE_II,CTRL_II,M_BTP_II,GUARD,RICCATI_W,RICCATI_I,ACCUM_W,ACCUM_I,CSIM_PASS,PASS_RATE,AVG_ITERS,MIN_CYCLES,FMAX,STATUS
EOF

run_stage3_candidate() {
    local cand_id="$1" state_ii="$2" ctrl_ii="$3" m_btp_ii="$4" guard="$5"
    local work_dir="${SWEEP_DIR}/s3_${cand_id}_${TIMESTAMP}"
    mkdir -p "$work_dir"; export TMPDIR="${work_dir}/tmp"; mkdir -p "$TMPDIR"
    
    if [ "$state_ii" -lt 1 ] || [ "$ctrl_ii" -lt 1 ] || [ "$m_btp_ii" -lt 1 ] || [ "$guard" -lt 8 ]; then
        echo "$cand_id,$state_ii,$ctrl_ii,$m_btp_ii,$guard,$MIN_RW,$MIN_RI,$MIN_AW,$MIN_AI,0,N/A,N/A,N/A,N/A,skipped_invalid" >> "$RESULTS_S3"
        return
    fi

    echo "[$(date '+%H:%M:%S')] S3: $cand_id"
    
    export HLS_RUN_MODE=csim HLS_STATE_ZY_II="$state_ii" HLS_CTRL_ZY_II="$ctrl_ii" HLS_M_BT_P_II="$m_btp_ii"
    export HLS_RICCATI_WIDTH="$MIN_RW" HLS_RICCATI_INT_BITS="$MIN_RI"
    export HLS_ACCUM_WIDTH="$MIN_AW" HLS_ACCUM_INT_BITS="$MIN_AI" HLS_RAW_ACC_GUARD_BITS="$guard"
    
    local csim_log="${work_dir}/csim.log" csim_pass=0 pass_rate="N/A" avg_iters="N/A"
    if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$csim_log" 2>&1); then
        csim_pass=1
    else
        if [ -f "$csim_log" ] && grep -q "=== RESULTS: 6 passed, 0 failed ===" "$csim_log"; then
            csim_pass=1
        else
            { echo "DEBUG S3: $cand_id failed"; tail -10 "$csim_log" 2>/dev/null; } >> "${work_dir}/DEBUG.log"
        fi
    fi
    if [ "$csim_pass" -eq 1 ]; then
        pass_rate=$(grep -oP 'success: \d+, \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
        avg_iters=$(grep -oP 'Avg iterations/call: \K[0-9.]+' "$csim_log" | head -1 || echo "N/A")
    fi
    
    local min_cycles="N/A" fmax="N/A"
    if [ "$csim_pass" -eq 1 ]; then
        export HLS_RUN_MODE=synth
        local synth_log="${work_dir}/synth.log"
        if (cd "$BASE_DIR" && vitis-run --mode hls --tcl scripts/run_hls.tcl > "$synth_log" 2>&1); then
            min_cycles=$(grep -oP 'min_latency: \K[0-9]+' "$synth_log" | head -1 || echo "N/A")
            fmax=$(grep -oP 'Target frequency: \K[0-9.]+' "$synth_log" | head -1 || echo "N/A")
        fi
    fi
    
    echo "$cand_id,$state_ii,$ctrl_ii,$m_btp_ii,$guard,$MIN_RW,$MIN_RI,$MIN_AW,$MIN_AI,$csim_pass,$pass_rate,$avg_iters,$min_cycles,$fmax,done" >> "$RESULTS_S3"
    rm -rf "$work_dir"
}

export -f run_stage3_candidate
export BEST_S_II BEST_C_II BEST_M_BTP_II MIN_RW MIN_RI MIN_AW MIN_AI MIN_G

printf '%s\n' "${STAGE3_CANDIDATES[@]}" | \
    xargs -I {} --max-procs="$NUM_CORES" \
    bash -c 'IFS=" " read -r id s_ii c_ii m_ii g <<< "{}"; run_stage3_candidate "$id" "$s_ii" "$c_ii" "$m_ii" "$g"'

echo ""
echo "=== THREE-STAGE SWEEP COMPLETE ==="
echo ""
echo "Results saved:"
echo "  Stage 1 (Widths): $RESULTS_S1"
echo "  Stage 2 (Pragmas): $RESULTS_S2"
echo "  Stage 3 (State/K): $RESULTS_S3"
echo ""
echo "Recommended configuration:"
echo "  Riccati: $MIN_RW/$MIN_RI, Accum: $MIN_AW/$MIN_AI, Guard: $MIN_G"
echo "  Pragmas: STATE_II=$BEST_S_II, CTRL_II=$BEST_C_II, M_BTP_II=$BEST_M_BTP_II"
echo ""
echo "Top Stage 3 results (by latency, csim_pass=1):"
(head -1 "$RESULTS_S3"; awk -F',' 'NR>1 && $10==1 {print}' "$RESULTS_S3" | sort -t',' -k13 -n | head -5) | column -t -s','
