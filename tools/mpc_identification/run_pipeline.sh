#!/usr/bin/env bash
# Backwards-compatible alias for the preparation phase.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$ROOT/run_prepare.sh" "$@"
