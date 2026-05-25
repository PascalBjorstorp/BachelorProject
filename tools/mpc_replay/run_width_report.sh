#!/usr/bin/env bash
#
# run_width_report.sh — REPORT-GRADE fixed-point sizing evidence.
#
# For every configurable width/guard in fp_types_hls.hpp, measures the
# observed max bit-width and |value| at the canonical chokepoints
# (products -> *_GUARD, sums -> fp_sum*_*, stores -> family WIDTH/INT_BITS)
# across MULTIPLE recorded bags, then emits a per-variable table comparing:
#
#     algebraic worst case   vs   observed max (per bag + combined)   vs
#     chosen width   vs   margin
#
# so the report can argue exactly why the worst case is not used and why
# each chosen width is correct.
#
# The probe is built with WIDE measurement typedefs (the #ifndef guards in
# fp_types_hls.hpp let us override) so single cast products are never
# truncated before measurement — production widths stay untouched.
#
# Usage:  tools/mpc_replay/run_width_report.sh
#         (auto-discovers every rosbag2 under tools/input/)
# Optional profile override:
#         MPC_HLS_WIDTH_PROFILE_OVERRIDE=4   tools/mpc_replay/run_width_report.sh
#         MPC_HLS_WIDTH_PROFILE_OVERRIDE=8   tools/mpc_replay/run_width_report.sh
#         MPC_HLS_WIDTH_PROFILE_OVERRIDE=-1  tools/mpc_replay/run_width_report.sh
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
KRIA="${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria"
HELPER="${REPO_ROOT}/tools/mpc_replay/helper"
INPUT_DIR="${REPO_ROOT}/tools/input"
OUT_DIR="${WIDTH_REPORT_OUT_DIR:-${REPO_ROOT}/tools/mpc_replay/width_report_out}"
BIN="${HELPER}/replay_fpga_scalar_wreport"
ACCUM_CSV="${OUT_DIR}/width_samples.csv"
VITIS_INCLUDE="${VITIS_INCLUDE:-/home/akselmo/Vivado_program/2025.2/Vitis/include}"
HORIZON="${HORIZON:-20}"
STEERING_SOURCE="${STEERING_SOURCE:-auto}"
PROFILE_OVERRIDE="${MPC_HLS_WIDTH_PROFILE_OVERRIDE:-}"

mkdir -p "${OUT_DIR}"
rm -f "${ACCUM_CSV}"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u                                # ROS setup.bash uses unbound vars
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # repo colcon install provides custom msgs (f1tenth_msgs) some bags need
  if [[ -f "${REPO_ROOT}/install/local_setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/install/local_setup.bash"
  fi
  set -u
fi
[[ -d "${VITIS_INCLUDE}" ]] || { echo "ERROR: VITIS_INCLUDE not found: ${VITIS_INCLUDE}" >&2; exit 2; }

echo "[1/4] Building probe (FP_WIDTH_PROBE, wide measurement typedefs)"
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
-DMPC_HLS_FN_RECIP_SHIFT_WIDTH=64 -DMPC_HLS_QP_DET_MUL_WIDTH=64"
PROFILE_FLAG=""
if [[ -n "${PROFILE_OVERRIDE}" ]]; then
  PROFILE_FLAG="-DMPC_HLS_WIDTH_PROFILE=${PROFILE_OVERRIDE}"
fi
echo "    width profile override: ${PROFILE_OVERRIDE:-default-from-fp_width_profile_config.hpp}"
g++ -O2 -Wno-unknown-pragmas -DFP_WIDTH_PROBE ${PROFILE_FLAG} ${WIDE} \
  -I"${KRIA}/include" -I"${VITIS_INCLUDE}" \
  "${HELPER}/replay_fpga_scalar.cpp" \
  "${KRIA}/src/fp_math_hls.cpp" "${KRIA}/src/vehicle_model_hls.cpp" \
  "${KRIA}/src/riccati_solver_hls.cpp" "${KRIA}/src/mpc_riccati_hls.cpp" \
  "${KRIA}/src/mpc_fpga_top.cpp" \
  -lm -o "${BIN}"

# Discover every rosbag2 (a directory containing metadata.yaml).
mapfile -t BAGS < <(find "${INPUT_DIR}" -name metadata.yaml -printf '%h\n' | sort -u)
[[ ${#BAGS[@]} -gt 0 ]] || { echo "ERROR: no rosbag2 under ${INPUT_DIR}" >&2; exit 2; }
echo "[2/4] Found ${#BAGS[@]} bag(s):"; printf '   %s\n' "${BAGS[@]}"

echo "[3/4] Export + probe each bag"
for bag in "${BAGS[@]}"; do
  label="$(basename "${bag}")"
  [[ "${bag}" == "${INPUT_DIR}" ]] && label="input_root"
  state_csv="${OUT_DIR}/state_${label}.csv"
  echo "  -> ${label}"
  info="${OUT_DIR}/baginfo_${label}.txt"
  ros2 bag info "${bag}" >"${info}" 2>/dev/null || true
  if grep -q "Topic: /mpc_state |" "${info}"; then
    python3 "${HELPER}/export_mpc_state_csv.py" \
      --bag "${bag}" --out "${state_csv}" --horizon "${HORIZON}" \
      >"${OUT_DIR}/export_${label}.log" 2>&1 || { echo "     export failed, skipping"; continue; }
  else
    python3 "${HELPER}/export_mpc_state_csv_from_cpu_bag.py" \
      --bag "${bag}" --out "${state_csv}" --horizon "${HORIZON}" \
      --steering-source "${STEERING_SOURCE}" \
      >"${OUT_DIR}/export_${label}.log" 2>&1 || { echo "     export failed, skipping"; continue; }
  fi
  if [[ ! -s "${state_csv}" ]]; then echo "     empty state CSV, skipping"; continue; fi
  FP_WPROBE_LABEL="${label}" FP_WPROBE_CSV="${ACCUM_CSV}" \
    "${BIN}" "${state_csv}" "${OUT_DIR}/replay_${label}.csv" \
    >"${OUT_DIR}/probe_${label}.log" 2>&1 || true
done

echo "[4/4] Emit production widths + aggregate -> report"
# Built WITHOUT the wide overrides => the real typedef widths.
g++ -O0 -w ${PROFILE_FLAG} -I"${KRIA}/include" -I"${VITIS_INCLUDE}" \
  "${HELPER}/dump_widths.cpp" -o "${HELPER}/dump_widths"
"${HELPER}/dump_widths" > "${OUT_DIR}/prod_widths.csv"

python3 "${HELPER}/aggregate_width_report.py" \
  --samples "${ACCUM_CSV}" \
  --prod-widths "${OUT_DIR}/prod_widths.csv" \
  --md "${OUT_DIR}/width_report.md" \
  --csv "${OUT_DIR}/width_report.csv"

echo
echo "Report written:"
echo "  ${OUT_DIR}/width_report.md   (paste into the thesis)"
echo "  ${OUT_DIR}/width_report.csv  (raw per-variable + per-bag data)"
