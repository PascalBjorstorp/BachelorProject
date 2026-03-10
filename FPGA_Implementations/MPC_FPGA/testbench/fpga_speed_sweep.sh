#!/usr/bin/env bash
#===============================================================
# FPGA MPC Parameter Sweep — find optimal dt, weights, wall constraints
#
# The FPGA solver uses compile-time constants (no env-var override).
# This script uses sed to modify mpc_fpga_types.h, recompiles,
# runs the test, and collects CSV results for each configuration.
#
# Sweeps:
#   Phase 1: dt × WALL_END × WALL_MARGIN  (horizon & constraint tuning)
#   Phase 2: Q_LAT × Q_HDG × Q_VEL       (primary tracking weights)
#   Phase 3: Q_LAT_VEL × Q_YAW × R_STEER × W_JERK (secondary weights)
#   Phase 4: ADMM rho × cross_call_scale  (solver parameters)
#
# Usage: bash fpga_speed_sweep.sh [phase]
#   No argument = run all phases
#   phase = 1,2,3,4 to run specific phase only
#===============================================================
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_DIR="${SCRIPT_DIR}/.."
HEADER="${PROJECT_DIR}/include/mpc_fpga_types.h"
HEADER_BAK="${HEADER}.sweep_backup"

# Source files for compilation
SRC_FILES=(
    "${PROJECT_DIR}/testbench/test_fpga_sim_drive.c"
    "${PROJECT_DIR}/src/riccati_solver_hls.c"
    "${PROJECT_DIR}/src/mpc_riccati_hls.c"
    "${PROJECT_DIR}/src/mpc_fpga_top.c"
    "${PROJECT_DIR}/src/vehicle_model_hls.c"
    "${PROJECT_DIR}/src/fp_math_hls.c"
)
BINARY="${PROJECT_DIR}/test_fpga_sweep"
CC_FLAGS="-D_GNU_SOURCE -O2 -std=c99 -Wall -Wno-unknown-pragmas -I${PROJECT_DIR}/include"

OUTFILE="${PROJECT_DIR}/testbench/fpga_sweep_$(date +%Y%m%d_%H%M%S).csv"

# ---- Backup original header ----
cp "${HEADER}" "${HEADER_BAK}"

cleanup() {
    echo ""
    echo "Restoring original header..."
    cp "${HEADER_BAK}" "${HEADER}"
    rm -f "${HEADER_BAK}" "${BINARY}"
    echo "Done."
}
trap cleanup EXIT

# ---- Helper: modify a #define in the header ----
set_define() {
    local name="$1" value="$2"
    # Handles both FP_CONST(x.x) and plain integer values
    if [[ "${value}" == *"."* ]]; then
        # Float value → use FP_CONST
        sed -i "s|#define ${name} .*|#define ${name}         FP_CONST(${value})|" "${HEADER}"
    elif [[ "${name}" == "MPC_DT" ]]; then
        # Special: MPC_DT uses raw fixed-point value with comment
        sed -i "s|#define MPC_DT .*|#define MPC_DT              ((fixed_point_t)${value})   /* in Q16.16 */|" "${HEADER}"
    else
        # Integer value
        sed -i "s|#define ${name} .*|#define ${name}            ${value}|" "${HEADER}"
    fi
}

# ---- Compute MPC_DT and cross_call_scale for a given dt ----
compute_dt_values() {
    local dt_float="$1"
    # MPC_DT in Q16.16 = dt * 65536
    MPC_DT_FP=$(python3 -c "print(round(${dt_float} * 65536))")
    # cross_call_scale = control_dt / prediction_dt = 0.005 / dt
    CROSS_CALL_SCALE=$(python3 -c "print(round(0.005 / ${dt_float}, 4))")
}

# ---- Compile and run one test ----
run_one() {
    local label="$1"

    # Compile
    if ! gcc ${CC_FLAGS} "${SRC_FILES[@]}" -o "${BINARY}" -lm 2>/dev/null; then
        echo "${label},999,0,6,,,,,,,,,,,,,COMPILE_FAIL" >> "${OUTFILE}"
        printf "  %-50s → COMPILE FAIL\n" "${label}"
        return
    fi

    # Run with CSV output
    local output
    output=$(MPC_TUNING_CSV=1 timeout 120 "${BINARY}" 2>/dev/null) || true

    local csv_line
    csv_line=$(echo "${output}" | grep '^CSV,' | head -1) || true

    if [[ -z "${csv_line}" ]]; then
        echo "${label},999,0,6,,,,,,,,,,,,,TIMEOUT" >> "${OUTFILE}"
        printf "  %-50s → TIMEOUT/FAIL\n" "${label}"
        return
    fi

    # Parse: CSV,passed,failed,max_lat,avg_lat,max_hdg,avg_hdg,max_vx,avg_solve,max_solve,wall_cols,time_5ms,max_vel_err,avg_vel_err,avg_iters
    local p f ml al mh ah mv as ms wc t5 mve ave ai
    IFS=',' read -r _ p f ml al mh ah mv as ms wc t5 mve ave ai <<< "${csv_line}"

    # Score: lower = better. Heavily penalize wall collisions.
    local score
    if [[ "${wc}" != "0" ]]; then
        score="500"
    elif (( f > 0 )); then
        score="999"
    else
        score=$(python3 -c "
t5=${t5}; ave=${ave}; mve=${mve}; al=${al}; ml=${ml}; ah=${ah}; mv=${mv}; ai=${ai}; asolve=${as}
score = (
    ave * 20.0 +
    mve * 4.0 +
    max(0, 30 - t5) * 3.0 +
    max(0, 12.0 - mv) * 8.0 +
    al * 5.0 +
    ml * 1.5 +
    ah * 2.0 +
    ai * 0.5 +
    asolve * 0.003
)
print(f'{score:.3f}')
" 2>/dev/null || echo "999")
    fi

    echo "${label},${score},${p},${f},${ml},${al},${mh},${ah},${mv},${as},${ms},${wc},${t5},${mve},${ave},${ai}" >> "${OUTFILE}"
    printf "  %-50s → sc=%-7s vmax=%-6s t5=%-5s wc=%s lat=%-5s p/f=%s/%s\n" \
           "${label}" "${score}" "${mv}" "${t5}" "${wc}" "${al}" "${p}" "${f}"
}

# ---- Restore defaults before each test ----
restore_defaults() {
    cp "${HEADER_BAK}" "${HEADER}"
}

# ---- CSV header ----
echo "label,score,passed,failed,max_lat,avg_lat,max_hdg,avg_hdg,max_vx,avg_solve,max_solve,wall_cols,time_above_5,max_vel_err,avg_vel_err,avg_iters" > "${OUTFILE}"

echo "============================================================"
echo "  FPGA MPC Parameter Sweep"
echo "  Output: ${OUTFILE}"
echo "============================================================"
echo ""

PHASE="${1:-all}"

# ================================================================
# PHASE 1: dt × WALL_END × WALL_MARGIN  (horizon & constraint)
# ================================================================
if [[ "${PHASE}" == "all" || "${PHASE}" == "1" ]]; then
    echo "--- Phase 1: dt × WALL_END × WALL_MARGIN ---"
    count=0
    for dt in 0.050 0.060 0.075 0.100; do
        compute_dt_values "${dt}"
        for wall_end in 8 10 12 14 16 18; do
            for wall_margin in 0.15 0.19 0.22 0.25; do
                restore_defaults
                set_define "MPC_DT" "${MPC_DT_FP}"
                set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
                set_define "WALL_END" "${wall_end}"
                set_define "WALL_MARGIN" "${wall_margin}"
                run_one "dt=${dt}+WE=${wall_end}+WM=${wall_margin}"
                count=$((count + 1))
            done
        done
    done
    echo ""
    echo "Phase 1 complete: ${count} configs tested"
    echo "--- Phase 1 Top 10 (no wall collisions) ---"
    (grep -v '^label' "${OUTFILE}" | grep -v ',500,' | sort -t',' -k2 -n | head -10 | column -t -s',') || echo "  (no passing configs)"
    echo ""
fi

# ================================================================
# PHASE 2: Q_LAT × Q_HDG × Q_VEL (primary tracking weights)
# Uses best dt/WALL_END/WALL_MARGIN from Phase 1 (or defaults below)
# ================================================================
if [[ "${PHASE}" == "all" || "${PHASE}" == "2" ]]; then
    echo "--- Phase 2: Q_LAT × Q_HDG × Q_VEL ---"

    # Best Phase 1 config (update after Phase 1 results)
    BEST_DT="${BEST_DT:-0.075}"
    BEST_WE="${BEST_WE:-14}"
    BEST_WM="${BEST_WM:-0.19}"

    compute_dt_values "${BEST_DT}"
    count=0
    for q_lat in 100 150 200 300 400 500; do
        for q_hdg in 200 300 400 500 700; do
            for q_vel in 10 20 30 40 60 80; do
                restore_defaults
                set_define "MPC_DT" "${MPC_DT_FP}"
                set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
                set_define "WALL_END" "${BEST_WE}"
                set_define "WALL_MARGIN" "${BEST_WM}"
                set_define "MPC_W_LAT_ERROR" "${q_lat}.0"
                set_define "MPC_W_HEADING" "${q_hdg}.0"
                set_define "MPC_W_VELOCITY" "${q_vel}.0"
                run_one "QL=${q_lat}+QH=${q_hdg}+QV=${q_vel}"
                count=$((count + 1))
            done
        done
    done
    echo ""
    echo "Phase 2 complete: ${count} configs tested"
    echo "--- Phase 2 Top 10 ---"
    (grep '^Q' "${OUTFILE}" | sort -t',' -k2 -n | head -10 | column -t -s',') || echo "  (no matching configs)"
    echo ""
fi

# ================================================================
# PHASE 3: Secondary weights around best Phase 2 config
# ================================================================
if [[ "${PHASE}" == "all" || "${PHASE}" == "3" ]]; then
    echo "--- Phase 3: Secondary weights ---"

    # Best Phase 2 config (update after Phase 2 results)
    BEST_DT="${BEST_DT:-0.075}"
    BEST_WE="${BEST_WE:-14}"
    BEST_WM="${BEST_WM:-0.19}"
    BEST_QL="${BEST_QL:-200}"
    BEST_QH="${BEST_QH:-500}"
    BEST_QV="${BEST_QV:-25}"

    compute_dt_values "${BEST_DT}"
    count=0

    # 3a: Q_LAT_VEL sweep
    for qlv in 10 20 30 50 80; do
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "MPC_W_LAT_VEL" "${qlv}.0"
        run_one "best+QLV=${qlv}"
        count=$((count + 1))
    done

    # 3b: Q_YAW sweep
    for qy in 3 5 10 20 40; do
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "MPC_W_YAW_RATE" "${qy}.0"
        run_one "best+QY=${qy}"
        count=$((count + 1))
    done

    # 3c: R_STEER sweep
    for rs in 0.05 0.1 0.15 0.2 0.35 0.5; do
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "MPC_W_STEER_EFF" "${rs}"
        run_one "best+RS=${rs}"
        count=$((count + 1))
    done

    # 3d: W_JERK sweep
    for wj in 0.05 0.1 0.2 0.3 0.5 1.0; do
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "MPC_W_STEER_JERK" "${wj}"
        run_one "best+WJ=${wj}"
        count=$((count + 1))
    done

    # 3e: ACCEL rate sweep
    for ar in 0.005 0.01 0.05 0.1; do
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "MPC_W_ACCEL_RATE" "${ar}"
        run_one "best+AR=${ar}"
        count=$((count + 1))
    done

    # 3f: Delta actuator weight sweep
    for da in 0.05 0.1 0.2 0.5 1.0; do
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "MPC_W_DELTA_ACT" "${da}"
        run_one "best+DA=${da}"
        count=$((count + 1))
    done

    echo ""
    echo "Phase 3 complete: ${count} configs tested"
    echo "--- Phase 3 Top 10 ---"
    (grep '^best' "${OUTFILE}" | sort -t',' -k2 -n | head -10 | column -t -s',') || echo "  (no matching configs)"
    echo ""
fi

# ================================================================
# PHASE 4: ADMM solver parameters
# ================================================================
if [[ "${PHASE}" == "all" || "${PHASE}" == "4" ]]; then
    echo "--- Phase 4: ADMM solver parameters ---"

    BEST_DT="${BEST_DT:-0.075}"
    BEST_WE="${BEST_WE:-14}"
    BEST_WM="${BEST_WM:-0.19}"
    BEST_QL="${BEST_QL:-200}"
    BEST_QH="${BEST_QH:-500}"
    BEST_QV="${BEST_QV:-25}"

    compute_dt_values "${BEST_DT}"
    count=0

    for rho in 20 30 40 60 80 100; do
        for rho_u in 5 10 20; do
            restore_defaults
            set_define "MPC_DT" "${MPC_DT_FP}"
            set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
            set_define "WALL_END" "${BEST_WE}"
            set_define "WALL_MARGIN" "${BEST_WM}"
            set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
            set_define "MPC_W_HEADING" "${BEST_QH}.0"
            set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
            set_define "ADMM_RHO_DEFAULT" "${rho}.0"
            set_define "ADMM_RHO_U_DEFAULT" "${rho_u}.0"
            run_one "rho=${rho}+rho_u=${rho_u}"
            count=$((count + 1))
        done
    done

    # Over-relaxation alpha sweep
    for alpha in 1.0 1.2 1.4 1.6 1.8; do
        alpha_m1=$(python3 -c "print(round(${alpha} - 1, 1))")
        alpha_m1_fp=$(python3 -c "print(round((${alpha} - 1) * 65536))")
        restore_defaults
        set_define "MPC_DT" "${MPC_DT_FP}"
        set_define "MPC_CROSS_CALL_SCALE" "${CROSS_CALL_SCALE}"
        set_define "WALL_END" "${BEST_WE}"
        set_define "WALL_MARGIN" "${BEST_WM}"
        set_define "MPC_W_LAT_ERROR" "${BEST_QL}.0"
        set_define "MPC_W_HEADING" "${BEST_QH}.0"
        set_define "MPC_W_VELOCITY" "${BEST_QV}.0"
        set_define "ADMM_OVER_RELAX" "${alpha}"
        # Also update ADMM_OVER_RELAX_MINUS1 (raw Q16.16)
        sed -i "s|#define ADMM_OVER_RELAX_MINUS1.*|#define ADMM_OVER_RELAX_MINUS1      ${alpha_m1_fp}           /* alpha - 1 = ${alpha_m1} in Q16.16 */|" "${HEADER}"
        run_one "alpha=${alpha}"
        count=$((count + 1))
    done

    echo ""
    echo "Phase 4 complete: ${count} configs tested"
    echo "--- Phase 4 Top 10 ---"
    (grep -E '^(rho|alpha)' "${OUTFILE}" | sort -t',' -k2 -n | head -10 | column -t -s',') || echo "  (no matching configs)"
    echo ""
fi

echo "============================================================"
echo "  Sweep Complete!"
echo "  Full results: ${OUTFILE}"
echo ""
echo "--- Overall Top 20 (all phases, no wall collisions) ---"
(grep -v '^label' "${OUTFILE}" | grep -v ',500,' | grep -v ',999,' | sort -t',' -k2 -n | head -20 | column -t -s',') || echo "  (no passing configs)"
echo "============================================================"
