#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

if [[ -z "${XILINX_HLS:-}" ]]; then
  if [[ -f "/tools/Xilinx/2025.1/Vitis/settings64.sh" ]]; then
    # shellcheck disable=SC1091
    source /tools/Xilinx/2025.1/Vitis/settings64.sh
  fi
fi

if [[ -z "${XILINX_HLS:-}" ]]; then
  echo "XILINX_HLS is not set. Source Vitis settings first."
  exit 1
fi

HLS_INCLUDE="${XILINX_HLS}/include"
BUILD_DIR="${ROOT_DIR}/build/mixed_precision_tests"
mkdir -p "${BUILD_DIR}"

COMMON_FLAGS=(
  -std=c++17
  -O2
  -DMPC_HLS_TARGET
  -DMPC_USE_AP_FIXED
  -DMPC_HLS_AP_CORE_ARITH=0
  -DMPC_HLS_AP_RECIP=1
  -DMPC_HLS_AP_RECIP_ITERS=2
  -DMPC_HLS_AP_VM_MUL=0
  -I"${ROOT_DIR}/include"
  -I"${HLS_INCLUDE}"
)

g++ "${COMMON_FLAGS[@]}" -x c++ \
  "${ROOT_DIR}/src/fp_math_hls.cpp" \
  "${ROOT_DIR}/testbench/test_fp_math_accuracy.cpp" \
  -o "${BUILD_DIR}/test_fp_math_accuracy"

g++ "${COMMON_FLAGS[@]}" -x c++ \
  "${ROOT_DIR}/src/fp_math_hls.cpp" \
  "${ROOT_DIR}/testbench/test_fp_backend_consistency.cpp" \
  -o "${BUILD_DIR}/test_fp_backend_consistency"

"${BUILD_DIR}/test_fp_math_accuracy"
"${BUILD_DIR}/test_fp_backend_consistency"

echo "All mixed-precision tests passed."
