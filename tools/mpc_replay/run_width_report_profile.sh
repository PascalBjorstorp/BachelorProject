#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
RUNNER="${REPO_ROOT}/tools/mpc_replay/run_width_report.sh"
BASE_OUT="${REPO_ROOT}/tools/mpc_replay/width_report_out"

usage() {
  cat <<'EOF'
Usage:
  tools/mpc_replay/run_width_report_profile.sh +1
  tools/mpc_replay/run_width_report_profile.sh +4
  tools/mpc_replay/run_width_report_profile.sh +8
  tools/mpc_replay/run_width_report_profile.sh max

Runs the width-report flow with the requested profile override and writes
artifacts to a dedicated subdirectory under tools/mpc_replay/width_report_out/.
EOF
}

[[ $# -eq 1 ]] || { usage; exit 2; }

case "$1" in
  +1) override="1"; tag="plus1" ;;
  +4) override="4"; tag="plus4" ;;
  +8) override="8"; tag="plus8" ;;
  max) override="-1"; tag="max" ;;
  *) usage; exit 2 ;;
esac

out_dir="${BASE_OUT}/${tag}"
echo "Running width report profile $1 -> ${out_dir}"

MPC_HLS_WIDTH_PROFILE_OVERRIDE="${override}" \
WIDTH_REPORT_OUT_DIR="${out_dir}" \
"${RUNNER}"
