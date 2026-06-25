#!/usr/bin/env bash
# Create a compact archive containing outputs needed for review. It omits the
# raw MCAP and retained raw scan matrices because the original bag is already known.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -gt 2 ]]; then
  echo "Usage: $0 [output-directory] [archive-path]" >&2
  exit 2
fi
OUT="${1:-$ROOT/output}"
ARCHIVE="${2:-$ROOT/identification_results.zip}"
ARCHIVE="$(python3 -c 'import os, sys; print(os.path.abspath(sys.argv[1]))' "$ARCHIVE")"
[[ -d "$OUT" ]] || { echo "Missing output directory: $OUT" >&2; exit 2; }
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/identification_results"
DEST="$TMP/identification_results"
for FILE in \
  "$OUT"/*.json "$OUT"/*.log "$OUT"/*.txt "$OUT"/*.csv "$OUT"/fitted_sim_env.sh \
  "$OUT"/plots "$OUT"/icp_map/icp_map_preview.png "$OUT"/extracted/manifest.json \
  "$OUT"/extracted/tf_static.csv "$OUT"/extracted/local_raceline_snapshots.csv; do
  [[ -e "$FILE" ]] || continue
  if [[ -d "$FILE" ]]; then cp -R "$FILE" "$DEST/"; else cp "$FILE" "$DEST/"; fi
done
(
  cd "$TMP"
  zip -qr "$ARCHIVE" identification_results
)
echo "Wrote $ARCHIVE"
