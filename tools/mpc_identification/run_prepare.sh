#!/usr/bin/env bash
# Extract a ROS 2 MCAP bag, localise scan poses against its recorded map, and
# produce the time-aligned identification table. No ROS installation is needed.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INCLUDE_LOCAL_RACELINE=0
if [[ "${1:-}" == "--include-local-raceline" ]]; then
  INCLUDE_LOCAL_RACELINE=1
  shift
fi
if [[ $# -lt 2 || $# -gt 3 ]]; then
  echo "Usage: $0 [--include-local-raceline] /path/to/run.mcap /path/to/raceline.csv [output-directory]" >&2
  exit 2
fi
BAG="$1"
RACELINE="$2"
OUT="${3:-$ROOT/output}"
for FILE in "$BAG" "$RACELINE"; do
  [[ -f "$FILE" ]] || { echo "Missing input: $FILE" >&2; exit 2; }
done

export MPLBACKEND=Agg
export PYTHONPATH="$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$OUT"

EXTRACT_ARGS=("$BAG" --output "$OUT/extracted")
if [[ "$INCLUDE_LOCAL_RACELINE" -eq 1 ]]; then
  EXTRACT_ARGS+=(--include-local-raceline)
fi
python3 "$ROOT/scripts/extract_mcap.py" "${EXTRACT_ARGS[@]}" | tee "$OUT/01_extract.log"
python3 "$ROOT/scripts/build_icp_map.py" "$OUT/extracted/map.npz" --output "$OUT/icp_map" | tee "$OUT/02_build_icp_map.log"
python3 "$ROOT/scripts/localize_scans_icp.py" \
  --scan-index "$OUT/extracted/scan_index.csv" --scans "$OUT/extracted/scans.npz" \
  --map-points "$OUT/icp_map/icp_map_points.npz" --prior "$OUT/extracted/ekf_pose.csv" \
  --output "$OUT/icp_poses.csv" | tee "$OUT/03_icp.log"
python3 "$ROOT/scripts/align_signals.py" \
  --icp "$OUT/icp_poses.csv" --odom "$OUT/extracted/odom.csv" --drive "$OUT/extracted/drive.csv" \
  --servo-feedback "$OUT/extracted/servo_feedback.csv" \
  --imu-yaw-rate "$OUT/extracted/imu_filtered_yaw_rate.csv" --raceline "$RACELINE" \
  --output "$OUT/observed_run.csv" | tee "$OUT/04_align.log"
python3 "$ROOT/scripts/make_identification_segments.py" "$OUT/observed_run.csv" \
  --output "$OUT/segments.csv" | tee "$OUT/05_segments.log"
python3 "$ROOT/scripts/fit_actuator.py" \
  --drive "$OUT/extracted/drive.csv" --servo-feedback "$OUT/extracted/servo_feedback.csv" \
  --output "$OUT/actuator_fit.json" | tee "$OUT/06_actuator_fit.log"

cat > "$OUT/PREPARE_COMPLETE.txt" <<EOF
Prepared identification data from:
  bag: $BAG
  raceline: $RACELINE

Next command:
  $ROOT/run_fit.sh $OUT

Optional local-raceline export (slow; only needed for closed-loop historical reference replay):
  $ROOT/run_prepare.sh --include-local-raceline "$BAG" "$RACELINE" "$OUT"
EOF

echo "Preparation complete. Run: $ROOT/run_fit.sh $OUT"
