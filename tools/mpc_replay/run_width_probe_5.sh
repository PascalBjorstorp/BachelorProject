#!/usr/bin/env bash
# Focused width probe over the 5 datasets requested for the Q12.14 sizing:
#   FPGA_ROS2, FPGA_UDP, MPC_10Laps, MPC10LapBaseline, LateralPlanner_*.mcap
# Builds replay_fpga_scalar with -DFP_WIDTH_PROBE and WIDE measurement typedefs
# (so single products are not truncated before measurement), exports each bag
# to a state CSV, runs the probe accumulating into one CSV, then aggregates a
# report with the production (Q12.14) widths.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
KRIA="${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria"
HELPER="${REPO_ROOT}/tools/mpc_replay/helper"
INPUT="${REPO_ROOT}/tools/input"
OUT_DIR="${REPO_ROOT}/tools/mpc_replay/width_probe5_out"
ACCUM_CSV="${OUT_DIR}/width_samples.csv"
BIN="${HELPER}/replay_fpga_scalar_wprobe5"
VITIS_INCLUDE="${VITIS_INCLUDE:-/home/akselmo/Vivado_program/2025.2/Vitis/include}"
HORIZON=20

# dataset spec:  "label|path|mode"   mode = direct | synth
DATASETS=(
  "FPGA_ROS2|${INPUT}/FPGA_ROS2|direct"
  "FPGA_UDP|${INPUT}/FPGA_UDP|synth"
  "MPC_10Laps|${INPUT}/MPC_10Laps|synth"
  "MPC10LapBaseline|${INPUT}/MPC10LapBaseline|synth"
  "LateralPlanner|${INPUT}/LateralPlanner_20260511_180450_0.mcap|synth"
)

mkdir -p "${OUT_DIR}"; rm -f "${ACCUM_CSV}"
set +u; source /opt/ros/jazzy/setup.bash; [ -f "${REPO_ROOT}/install/local_setup.bash" ] && source "${REPO_ROOT}/install/local_setup.bash"; set -u
[[ -d "${VITIS_INCLUDE}" ]] || { echo "ERROR: VITIS_INCLUDE not found: ${VITIS_INCLUDE}" >&2; exit 2; }

echo "[1/4] Build probe (FP_WIDTH_PROBE, wide measurement typedefs, Q12.14 config)"
WIDE="-DMPC_HLS_SUM6_QP_MUL_WIDTH=64 -DMPC_HLS_SUM6_P_QP_WIDTH=64 \
-DMPC_HLS_SUM2_QP_RAW_WIDTH=64 -DMPC_HLS_SUM4_QP_RAW_WIDTH=64 \
-DMPC_HLS_SUM8_QP_RAW_WIDTH=64 -DMPC_HLS_SUM2_P_RAW_WIDTH=64 \
-DMPC_HLS_SUM2_P_QP_WIDTH=64 -DMPC_HLS_SUM4_P_QP_WIDTH=64 \
-DMPC_HLS_P_MIX_ITEM_WIDTH=64 -DMPC_HLS_SUM2_P_MIX_WIDTH=64 \
-DMPC_HLS_SUM4_P_MIX_WIDTH=64 -DMPC_HLS_SUM8_P_MIX_WIDTH=64 \
-DMPC_HLS_SUM8_P_MIX_PUP_WIDTH=64 -DMPC_HLS_SUM2_MG_RAW_WIDTH=64 \
-DMPC_HLS_SUM6_MG_QP_WIDTH=64 -DMPC_HLS_SUM2_MG_QP_WIDTH=64 \
-DMPC_HLS_SUM4_MG_QP_WIDTH=64 -DMPC_HLS_SUM2_QP_MG_WIDTH=64 \
-DMPC_HLS_SUM2_MG_K_WIDTH=64 -DMPC_HLS_K_QP_ITEM_WIDTH=64 \
-DMPC_HLS_SUM2_K_QP_WIDTH=64 -DMPC_HLS_SUM4_K_QP_WIDTH=64 \
-DMPC_HLS_SUM8_K_QP_WIDTH=64 -DMPC_HLS_QP_RECIP_SHIFT_WIDTH=64 \
-DMPC_HLS_FN_RECIP_SHIFT_WIDTH=64 -DMPC_HLS_QP_DET_MUL_WIDTH=64 \
-DMPC_HLS_QP_GUARD=38 -DMPC_HLS_FN_GUARD=38 -DMPC_HLS_P_QP_GUARD=38 \
-DMPC_HLS_MG_QP_GUARD=38 -DMPC_HLS_MG_K_GUARD=38 -DMPC_HLS_K_QP_GUARD=38 \
-DMPC_HLS_P_WIDTH=48 -DMPC_HLS_MG_WIDTH=48 -DMPC_HLS_K_WIDTH=48 -DMPC_HLS_FN_WIDTH=40"
g++ -O2 -Wno-unknown-pragmas -DFP_WIDTH_PROBE ${WIDE} \
  -I"${KRIA}/include" -I"${VITIS_INCLUDE}" \
  "${HELPER}/replay_fpga_scalar.cpp" \
  "${KRIA}/src/fp_math_hls.cpp" "${KRIA}/src/vehicle_model_hls.cpp" \
  "${KRIA}/src/riccati_solver_hls.cpp" "${KRIA}/src/mpc_riccati_hls.cpp" \
  "${KRIA}/src/mpc_fpga_top.cpp" -lm -o "${BIN}"

echo "[2/4] Export + probe each dataset"
for spec in "${DATASETS[@]}"; do
  IFS='|' read -r label path mode <<< "${spec}"
  state_csv="${OUT_DIR}/state_${label}.csv"
  echo "  -> ${label} (${mode})  ${path}"
  if [[ "${mode}" == "direct" ]]; then
    python3 "${HELPER}/export_mpc_state_csv.py" --bag "${path}" --out "${state_csv}" --horizon "${HORIZON}" \
      >"${OUT_DIR}/export_${label}.log" 2>&1 || { echo "     export FAILED"; continue; }
  else
    python3 "${HELPER}/export_mpc_state_csv_from_cpu_bag.py" --bag "${path}" --out "${state_csv}" \
      --horizon "${HORIZON}" --steering-source auto >"${OUT_DIR}/export_${label}.log" 2>&1 || { echo "     export FAILED"; continue; }
  fi
  [[ -s "${state_csv}" ]] || { echo "     empty state CSV"; continue; }
  echo "     rows: $(($(wc -l < "${state_csv}")-1))"
  FP_WPROBE_LABEL="${label}" FP_WPROBE_CSV="${ACCUM_CSV}" \
    "${BIN}" "${state_csv}" "${OUT_DIR}/replay_${label}.csv" >"${OUT_DIR}/probe_${label}.log" 2>&1 || true
done

echo "[3/4] Emit production (Q12.14) widths"
g++ -O0 -w -I"${KRIA}/include" -I"${VITIS_INCLUDE}" "${HELPER}/dump_widths.cpp" -o "${HELPER}/dump_widths"
"${HELPER}/dump_widths" > "${OUT_DIR}/prod_widths.csv"

echo "[4/4] Aggregate report"
python3 "${HELPER}/aggregate_width_report.py" \
  --samples "${ACCUM_CSV}" --prod-widths "${OUT_DIR}/prod_widths.csv" \
  --md "${OUT_DIR}/width_report.md" --csv "${OUT_DIR}/width_report.csv"
echo "Report: ${OUT_DIR}/width_report.md"
