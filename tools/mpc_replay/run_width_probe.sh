#!/usr/bin/env bash
#
# run_width_probe.sh — measure the TRUE bit-widths the QP product/sum families
# need, from real recorded state, so guard #defines can be set from data
# instead of guessed worst case.
#
# It builds replay_fpga_scalar with -DFP_WIDTH_PROBE and runs it over one or
# more mpc_state CSVs. The probe (fp_width_probe.hpp) tracks the maximum
# untruncated signed bit-width seen at each instrumented site and prints a
# "FP WIDTH PROBE SUMMARY" table at exit. Read the need_guard column, add a
# small margin, and set MPC_HLS_QP_GUARD accordingly in fp_types_hls.hpp.
#
# Usage:
#   tools/mpc_replay/run_width_probe.sh [state_csv ...]
# Optional profile override:
#   MPC_HLS_WIDTH_PROFILE_OVERRIDE=4   tools/mpc_replay/run_width_probe.sh
#   MPC_HLS_WIDTH_PROFILE_OVERRIDE=8   tools/mpc_replay/run_width_probe.sh
#   MPC_HLS_WIDTH_PROFILE_OVERRIDE=-1  tools/mpc_replay/run_width_probe.sh
# Default state CSV: ARM_Benchmark/input/state_replay.csv
#
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
KRIA="${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria"
HELPER="${REPO_ROOT}/tools/mpc_replay/helper"
BIN="${HELPER}/replay_fpga_scalar_wprobe"
VITIS_INCLUDE="${VITIS_INCLUDE:-/home/akselmo/Vivado_program/2025.2/Vitis/include}"
PROFILE_OVERRIDE="${MPC_HLS_WIDTH_PROFILE_OVERRIDE:-}"

CSVS=("$@")
if [[ ${#CSVS[@]} -eq 0 ]]; then
  CSVS=("${REPO_ROOT}/ARM_Benchmark/input/state_replay.csv")
fi

if [[ ! -d "${VITIS_INCLUDE}" ]]; then
  echo "ERROR: VITIS include path not found: ${VITIS_INCLUDE}" >&2
  echo "Set VITIS_INCLUDE=/path/to/Vitis/include" >&2
  exit 2
fi

echo "[1/2] Building replay_fpga_scalar with -DFP_WIDTH_PROBE (wide measurement types)"
# Force the sum-accumulator typedefs WIDE during measurement (the #ifndef
# guards in fp_types_hls.hpp let us override). Otherwise the production-tight
# widths would truncate single cast products BEFORE the per-input probe sees
# them, hiding the very thing we are trying to measure. 64 >= every algebraic
# single-product width (max is P/MG ~61b) and <= int64 probe range.
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
  -I"${KRIA}/include" \
  -I"${VITIS_INCLUDE}" \
  "${HELPER}/replay_fpga_scalar.cpp" \
  "${KRIA}/src/fp_math_hls.cpp" \
  "${KRIA}/src/vehicle_model_hls.cpp" \
  "${KRIA}/src/riccati_solver_hls.cpp" \
  "${KRIA}/src/mpc_riccati_hls.cpp" \
  "${KRIA}/src/mpc_fpga_top.cpp" \
  -lm -o "${BIN}"

echo "[2/2] Running probe over ${#CSVS[@]} state CSV(s)"
TMP_OUT="$(mktemp)"
trap 'rm -f "${TMP_OUT}"' EXIT
for csv in "${CSVS[@]}"; do
  if [[ ! -f "${csv}" ]]; then
    echo "  SKIP (missing): ${csv}" >&2
    continue
  fi
  echo "  -> ${csv}"
  # Probe summary goes to stderr at exit; solver stdout is discarded.
  "${BIN}" "${csv}" "${TMP_OUT}" >/dev/null
done

echo
echo "Done. Use the need_guard values above (+ margin) to set the guard"
echo "#defines in FPGA_Implementations/MPC_FPGA_Kria/include/fp_types_hls.hpp."
