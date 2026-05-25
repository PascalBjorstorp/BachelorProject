#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="${REPO_ROOT}/tools/mpc_replay/run_width_probe.sh"

usage() {
  cat <<'EOF'
Usage:
  tools/mpc_replay/run_width_probe_profile.sh +1 [state_csv ...]
  tools/mpc_replay/run_width_probe_profile.sh +4 [state_csv ...]
  tools/mpc_replay/run_width_probe_profile.sh +8 [state_csv ...]
  tools/mpc_replay/run_width_probe_profile.sh max [state_csv ...]

Runs the width-probe flow with the requested profile override.
EOF
}

[[ $# -ge 1 ]] || { usage; exit 2; }

case "$1" in
  +1) override="1" ;;
  +4) override="4" ;;
  +8) override="8" ;;
  max) override="-1" ;;
  *) usage; exit 2 ;;
esac
shift

echo "Running width probe profile $([[ ${override} == -1 ]] && echo max || echo +${override})"
MPC_HLS_WIDTH_PROFILE_OVERRIDE="${override}" "${RUNNER}" "$@"
