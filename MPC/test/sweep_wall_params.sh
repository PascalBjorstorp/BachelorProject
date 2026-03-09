#!/usr/bin/env bash
# sweep_wall_params.sh — Sweep WALL_MARGIN x WALL_CONSTRAINT_STRIDE combinations
# and report a results table. Restores best-passing combo at the end.
set -euo pipefail

cd "$(dirname "$0")/.."  # MPC root

SRC=src/mpc_riccati.c

# Save original values
ORIG_MARGIN="0.25"
ORIG_STRIDE="1"

MARGINS=(0.10 0.15 0.20 0.25)
STRIDES=(1 2 3 5)

BUILD_CMD="gcc -D_GNU_SOURCE -O3 -std=c99 -Wall -ffast-math \
  -Wno-unused-variable -Wno-unused-but-set-variable \
  -Iinclude test/test_sim_drive.c src/mpc_riccati.c src/riccati_solver.c \
  src/vehicle_model.c src/fp_math.c -o test_sim_drive -lm"

# ---------- helpers ----------
set_margin() {
    sed -i "s/#define WALL_MARGIN .*/#define WALL_MARGIN FP_CONST($1)/" "$SRC"
}
set_stride() {
    sed -i "s/#define WALL_CONSTRAINT_STRIDE .*/#define WALL_CONSTRAINT_STRIDE $1/" "$SRC"
}
restore_original() {
    set_margin "$ORIG_MARGIN"
    set_stride "$ORIG_STRIDE"
}
trap restore_original EXIT   # always restore on exit

# ---------- results storage ----------
declare -a R_MARGIN R_STRIDE R_PASS R_FAIL R_AVG_ITER R_MAX_LAT R_AVG_LAT R_WALL R_T5
IDX=0

# Header
printf "\n%-8s %-8s %-6s %-6s %-10s %-10s %-10s %-12s %-12s\n" \
       "MARGIN" "STRIDE" "PASS" "FAIL" "AVG_ITER" "MAX_LAT" "AVG_LAT" "WALL_COLL" "PCT>5m/s"
printf '%.0s-' {1..88}; echo

SIM_DURATION=30.0   # must match test_sim_drive.c

for margin in "${MARGINS[@]}"; do
  for stride in "${STRIDES[@]}"; do
    set_margin "$margin"
    set_stride "$stride"

    # Build
    if ! eval $BUILD_CMD 2>/dev/null; then
        printf "%-8s %-8s BUILD FAILED\n" "$margin" "$stride"
        continue
    fi

    # Run with CSV output
    OUTPUT=$(MPC_TUNING_CSV=1 ./test_sim_drive 2>&1 || true)

    # Parse CSV line: CSV,pass,fail,max_lat,avg_lat,max_hdg,avg_hdg,max_vx,avg_solve,max_solve,wall_coll,time_above_5,max_vel_err,avg_vel
    CSV_LINE=$(echo "$OUTPUT" | grep '^CSV,' | tail -1)
    if [[ -z "$CSV_LINE" ]]; then
        printf "%-8s %-8s NO CSV OUTPUT\n" "$margin" "$stride"
        continue
    fi

    IFS=',' read -r _ pass fail max_lat avg_lat max_hdg avg_hdg max_vx avg_solve max_solve wall_coll time5 _ _ <<< "$CSV_LINE"

    # Avg iterations
    avg_iter=$(echo "$OUTPUT" | grep 'Avg iterations/call' | awk '{print $NF}')

    # Percent above 5 m/s
    pct5=$(awk "BEGIN{printf \"%.1f\", 100*${time5}/${SIM_DURATION}}")

    printf "%-8s %-8s %-6s %-6s %-10s %-10s %-10s %-12s %-12s\n" \
           "$margin" "$stride" "$pass" "$fail" "$avg_iter" "$max_lat" "$avg_lat" "$wall_coll" "${pct5}%"

    # Store for best-selection
    R_MARGIN[$IDX]=$margin
    R_STRIDE[$IDX]=$stride
    R_PASS[$IDX]=$pass
    R_FAIL[$IDX]=$fail
    R_AVG_ITER[$IDX]=$avg_iter
    R_MAX_LAT[$IDX]=$max_lat
    R_AVG_LAT[$IDX]=$avg_lat
    R_WALL[$IDX]=$wall_coll
    R_T5[$IDX]=$pct5
    IDX=$((IDX+1))
  done
done

printf '%.0s-' {1..88}; echo

# ---------- Find best passing combination ----------
# "Best" = all tests pass (fail==0), then lowest avg_lat, then lowest max_lat
BEST_IDX=-1
BEST_AVG_LAT=999
BEST_MAX_LAT=999

for ((i=0; i<IDX; i++)); do
    if [[ "${R_FAIL[$i]}" == "0" ]]; then
        # Compare avg_lat (use awk for float comparison)
        IS_BETTER=$(awk "BEGIN{print (${R_AVG_LAT[$i]} < $BEST_AVG_LAT) || (${R_AVG_LAT[$i]} == $BEST_AVG_LAT && ${R_MAX_LAT[$i]} < $BEST_MAX_LAT)}")
        if [[ "$IS_BETTER" == "1" ]]; then
            BEST_IDX=$i
            BEST_AVG_LAT=${R_AVG_LAT[$i]}
            BEST_MAX_LAT=${R_MAX_LAT[$i]}
        fi
    fi
done

if [[ $BEST_IDX -ge 0 ]]; then
    echo ""
    echo "=== BEST passing combination ==="
    echo "  WALL_MARGIN = ${R_MARGIN[$BEST_IDX]}"
    echo "  WALL_CONSTRAINT_STRIDE = ${R_STRIDE[$BEST_IDX]}"
    echo "  Avg lateral error = ${R_AVG_LAT[$BEST_IDX]} m"
    echo "  Max lateral error = ${R_MAX_LAT[$BEST_IDX]} m"
    echo "  Wall collisions = ${R_WALL[$BEST_IDX]}"
    echo "  Avg iterations = ${R_AVG_ITER[$BEST_IDX]}"
    echo ""
    echo "Setting source to best combination..."
    # The trap will fire, but we override first
    trap - EXIT
    set_margin "${R_MARGIN[$BEST_IDX]}"
    set_stride "${R_STRIDE[$BEST_IDX]}"
    echo "Done. mpc_riccati.c now has WALL_MARGIN=${R_MARGIN[$BEST_IDX]}, STRIDE=${R_STRIDE[$BEST_IDX]}"
else
    echo ""
    echo "No combination passed all tests! Restoring original values."
    # trap will restore
fi
