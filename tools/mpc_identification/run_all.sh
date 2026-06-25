#!/usr/bin/env bash
# Full hardware-to-simulation identification run.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 /path/to/run.mcap /path/to/raceline.csv [output-directory]" >&2
  exit 2
fi
"$ROOT/run_prepare.sh" "$1" "$2" "${3:-$ROOT/output}"
# Keep raceline available to plot script even when output is outside the bundle.
export RACELINE_PATH="$2"
"$ROOT/run_fit.sh" "${3:-$ROOT/output}"
