#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
CFG="${REPO_ROOT}/FPGA_Implementations/MPC_FPGA_Kria/include/fp_width_profile_config.hpp"

usage() {
  cat <<'EOF'
Usage:
  tools/mpc_replay/set_width_profile.sh current
  tools/mpc_replay/set_width_profile.sh +1
  tools/mpc_replay/set_width_profile.sh +4
  tools/mpc_replay/set_width_profile.sh +8
  tools/mpc_replay/set_width_profile.sh max

This updates the default profile used by builds that do not pass
-DMPC_HLS_WIDTH_PROFILE explicitly.
EOF
}

[[ $# -eq 1 ]] || { usage; exit 2; }

case "$1" in
  current)
    current="$(sed -n 's/^#define MPC_HLS_WIDTH_PROFILE //p' "${CFG}")"
    case "${current}" in
      MPC_HLS_PROFILE_PLUS1) echo "+1" ;;
      MPC_HLS_PROFILE_PLUS4) echo "+4" ;;
      MPC_HLS_PROFILE_PLUS8) echo "+8" ;;
      MPC_HLS_PROFILE_WORST) echo "max" ;;
      *) echo "unknown (${current})" ;;
    esac
    exit 0
    ;;
  +1) value="MPC_HLS_PROFILE_PLUS1" ;;
  +4) value="MPC_HLS_PROFILE_PLUS4" ;;
  +8) value="MPC_HLS_PROFILE_PLUS8" ;;
  max) value="MPC_HLS_PROFILE_WORST" ;;
  *) usage; exit 2 ;;
esac

tmp="$(mktemp)"
sed "s/^#define MPC_HLS_WIDTH_PROFILE .*/#define MPC_HLS_WIDTH_PROFILE ${value}/" "${CFG}" > "${tmp}"
mv "${tmp}" "${CFG}"

echo "Set default width profile to ${1} in ${CFG}"
