#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG="${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include/fp_width_profile_config.hpp"

usage() {
  cat <<'EOF'
Usage:
  tools/mpc_replay/set_store_width_profile.sh current
  tools/mpc_replay/set_store_width_profile.sh exact
  tools/mpc_replay/set_store_width_profile.sh +4

This updates the default store widening used by builds that do not pass
-DMPC_HLS_STORE_WIDTH_PAD explicitly.

Only FN/P/MG/K stores are affected.
QP store/transport remains fixed to the external 32-bit payload format.
EOF
}

[[ $# -eq 1 ]] || { usage; exit 2; }

case "$1" in
  current)
    current="$(sed -n 's/^#define MPC_HLS_STORE_WIDTH_PAD //p' "${CFG}")"
    case "${current}" in
      MPC_HLS_STORE_PAD_EXACT) echo "exact" ;;
      MPC_HLS_STORE_PAD_PLUS4) echo "+4" ;;
      *) echo "unknown (${current})" ;;
    esac
    exit 0
    ;;
  exact) value="MPC_HLS_STORE_PAD_EXACT" ;;
  +4) value="MPC_HLS_STORE_PAD_PLUS4" ;;
  *) usage; exit 2 ;;
esac

tmp="$(mktemp)"
sed "s/^#define MPC_HLS_STORE_WIDTH_PAD .*/#define MPC_HLS_STORE_WIDTH_PAD ${value}/" "${CFG}" > "${tmp}"
mv "${tmp}" "${CFG}"

echo "Set default store width profile to ${1} in ${CFG}"
