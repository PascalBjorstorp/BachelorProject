#!/usr/bin/env bash
#===============================================================
# Targeted MPC weight sweep — maximize speed on new tighter raceline
# Baseline: Q_LAT=150, Q_HDG=500, Q_VEL=6, Q_LAT_VEL=60, Q_YAW=10
#           R_STEER=0.1, W_JERK=0.2
# Problem:  avg speed only 5.14 m/s (56% above 5 m/s)
# Strategy: ↑Q_VEL, ↓Q_HDG while keeping Q_LAT for wall safety
#===============================================================
set -euo pipefail

cd "$(dirname "$0")/.."
BINARY="./test_sim_drive"

if [[ ! -x "${BINARY}" ]]; then
    echo "ERROR: build test_sim_drive first" >&2
    exit 1
fi

OUTFILE="test/speed_sweep_$(date +%Y%m%d_%H%M%S).csv"
echo "label,score,passed,failed,max_lat,avg_lat,max_hdg,avg_hdg,max_vx,avg_solve_us,max_solve_us,time_above_5,max_vel_err,avg_vel_err,avg_iters,status,Q_LAT,Q_HDG,Q_VEL,Q_LAT_VEL,Q_YAW,R_STEER,W_JERK" > "${OUTFILE}"

run_one() {
    local label="$1"
    local ql="$2" qh="$3" qv="$4" qlv="$5" qy="$6" rs="$7" wj="$8"

    local output
    output=$(Q_LAT="${ql}" Q_HDG="${qh}" Q_VEL="${qv}" Q_LAT_VEL="${qlv}" Q_YAW="${qy}" R_STEER="${rs}" W_JERK="${wj}" MPC_TUNING_CSV=1 timeout 90 "${BINARY}" 2>/dev/null) || true

    local csv_line
    csv_line=$(echo "${output}" | grep '^CSV,' | head -1) || true

    if [[ -z "${csv_line}" ]]; then
        echo "${label},999,0,6,,,,,,,,,,,,,${ql},${qh},${qv},${qlv},${qy},${rs},${wj}" >> "${OUTFILE}"
        printf "  %-45s → TIMEOUT/FAIL\n" "${label}"
        return
    fi

    # Parse: CSV,passed,failed,max_lat,avg_lat,max_hdg,avg_hdg,max_vx,avg_solve,max_solve,wall_cols,time_5ms,max_vel_err,avg_vel_err,avg_iters
    local p f ml al mh ah mv as ms wc t5 mve ave ai
    IFS=',' read -r _ p f ml al mh ah mv as ms wc t5 mve ave ai <<< "${csv_line}"

    # Score: lower = better. Prioritize speed while penalizing wall hits
    local score
    if (( f > 0 )); then
        score="999"
    elif (( wc > 0 )); then
        score="500"
    else
        # Primary: speed (time above 5 m/s, avg velocity error)
        # Secondary: safety (lateral error)
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
")
    fi

    echo "${label},${score},${p},${f},${ml},${al},${mh},${ah},${mv},${as},${ms},${t5},${mve},${ave},${ai},OK,${ql},${qh},${qv},${qlv},${qy},${rs},${wj}" >> "${OUTFILE}"
    printf "  %-45s → score=%-7s vmax=%-6s t5=%-5s lat=%-5s p/f=%s/%s\n" "${label}" "${score}" "${mv}" "${t5}" "${al}" "${p}" "${f}"
}

echo "=== Speed-Focused Weight Sweep ==="
echo "Output: ${OUTFILE}"
echo ""

# Phase 1: Vary Q_HDG and Q_VEL (most impactful)
echo "--- Phase 1: Primary sweep (Q_HDG × Q_VEL) ---"
for qh in 100 150 200 300 400 500; do
    for qv in 6 15 30 50 80 120; do
        run_one "HDG=${qh}+VEL=${qv}" 150 "${qh}" "${qv}" 60 10 0.1 0.2
    done
done

# Phase 2: Best Q_HDG/Q_VEL combos with varied Q_LAT
echo ""
echo "--- Phase 2: Q_LAT variations ---"
for ql in 75 100 125 150 200; do
    for qh in 100 200 300; do
        for qv in 30 50 80; do
            run_one "LAT=${ql}+HDG=${qh}+VEL=${qv}" "${ql}" "${qh}" "${qv}" 60 10 0.1 0.2
        done
    done
done

# Phase 3: Secondary weight tuning around promising configs
echo ""
echo "--- Phase 3: Secondary tweaks ---"
for qlv in 20 40 60 100; do
    run_one "best+LAT_VEL=${qlv}" 150 200 50 "${qlv}" 10 0.1 0.2
done
for qy in 3 5 10 20 30; do
    run_one "best+YAW=${qy}" 150 200 50 60 "${qy}" 0.1 0.2
done
for rs in 0.05 0.1 0.2 0.35 0.5; do
    run_one "best+R_STEER=${rs}" 150 200 50 60 10 "${rs}" 0.2
done
for wj in 0.1 0.2 0.5 1.0 2.0; do
    run_one "best+W_JERK=${wj}" 150 200 50 60 10 0.1 "${wj}"
done

echo ""
echo "--- Top 10 results ---"
sort -t',' -k2 -n "${OUTFILE}" | head -11 | column -t -s','
echo ""
echo "Done. Full results: ${OUTFILE}"
