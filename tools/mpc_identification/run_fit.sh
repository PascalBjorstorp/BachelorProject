#!/usr/bin/env bash
# Fit longitudinal and lateral plant parameters, validate on the held-out lap,
# and write a shell file for test_sim_drive.c.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -gt 1 ]]; then
  echo "Usage: $0 [prepared-output-directory]" >&2
  exit 2
fi
OUT="${1:-$ROOT/output}"
for FILE in "$OUT/observed_run.csv" "$OUT/segments.csv" "$OUT/extracted/drive.csv" "$OUT/actuator_fit.json"; do
  [[ -f "$FILE" ]] || { echo "Missing prerequisite: $FILE. Run run_prepare.sh first." >&2; exit 2; }
done

export MPLBACKEND=Agg
export PYTHONPATH="$ROOT/scripts${PYTHONPATH:+:$PYTHONPATH}"
mkdir -p "$OUT/plots"

python3 "$ROOT/scripts/merge_actuator_fit.py" \
  --base "$ROOT/config/initial_parameters.json" --actuator-fit "$OUT/actuator_fit.json" \
  --output "$OUT/actuator_seed_parameters.json" | tee "$OUT/07_merge_actuator.log"
python3 "$ROOT/scripts/fit_plant.py" longitudinal \
  --observed "$OUT/observed_run.csv" --segments "$OUT/segments.csv" --drive "$OUT/extracted/drive.csv" \
  --parameters "$OUT/actuator_seed_parameters.json" --output "$OUT/longitudinal_fit.json" \
  --trace-output "$OUT/longitudinal_trace.csv" --metrics-output "$OUT/longitudinal_metrics.json" \
  | tee "$OUT/08_longitudinal_fit.log"
python3 "$ROOT/scripts/fit_plant.py" lateral \
  --observed "$OUT/observed_run.csv" --segments "$OUT/segments.csv" --drive "$OUT/extracted/drive.csv" \
  --parameters "$OUT/longitudinal_fit.json" --output "$OUT/lateral_fit.json" \
  --trace-output "$OUT/lateral_trace.csv" --metrics-output "$OUT/lateral_metrics.json" \
  | tee "$OUT/09_lateral_fit.log"
python3 "$ROOT/scripts/replay_open_loop.py" \
  --observed "$OUT/observed_run.csv" --segments "$OUT/segments.csv" --drive "$OUT/extracted/drive.csv" \
  --parameters "$OUT/lateral_fit.json" --split test --output "$OUT/test_predictions.csv" \
  --metrics "$OUT/test_metrics.json" | tee "$OUT/10_test_replay.log"
python3 "$ROOT/scripts/plot_results.py" \
  --observed "$OUT/observed_run.csv" --predictions "$OUT/test_predictions.csv" \
  --raceline "${RACELINE_PATH:-$ROOT/raceline.csv}" --output "$OUT/plots" \
  | tee "$OUT/11_plots.log"
python3 "$ROOT/scripts/export_sim_env.py" "$OUT/lateral_fit.json" \
  --output "$OUT/fitted_sim_env.sh" | tee "$OUT/12_export_sim_env.log"

cat > "$OUT/FIT_COMPLETE.txt" <<EOF
Fitting and held-out replay complete.

Primary results:
  actuator_fit.json
  longitudinal_metrics.json
  lateral_metrics.json
  test_metrics.json
  plots/
  fitted_sim_env.sh
EOF

echo "Fit complete. Package outputs with: $ROOT/package_results.sh $OUT"
