#!/usr/bin/env bash
set -euo pipefail

# Aggressive Kria HLS sweep helper.
# Runs multiple synth profiles and writes a compact CSV summary.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPORT_PATH="$PROJECT_DIR/mpc_fpga_hls/kria_kv260/syn/report/mpc_fpga_top_csynth.rpt"
SWEEP_LOG_DIR="$PROJECT_DIR/logs/hls_sweep"
SUMMARY_CSV="$SWEEP_LOG_DIR/hls_sweep_summary.csv"

mkdir -p "$SWEEP_LOG_DIR"

if ! command -v vitis-run >/dev/null 2>&1; then
  echo "ERROR: vitis-run not found in PATH."
  echo "Run: source /tools/Xilinx/2025.1/Vitis/settings64.sh"
  exit 1
fi

trim() {
  local s="$1"
  # shellcheck disable=SC2001
  echo "$(echo "$s" | sed 's/^ *//; s/ *$//')"
}

extract_field() {
  local line="$1"
  local idx="$2"
  local val
  val="$(echo "$line" | awk -F'|' -v i="$idx" '{print $i}')"
  trim "$val"
}

extract_metrics() {
  local report="$1"

  local timing_line
  timing_line="$(awk '/\|ap_clk/{print; exit}' "$report")"

  local latency_line
  latency_line="$(awk '
    /\+ Latency:/ {in_lat=1; next}
    in_lat && /^[[:space:]]*\|[[:space:]]*[0-9]+[[:space:]]*\|[[:space:]]*[0-9]+/ {print; exit}
  ' "$report")"

  local total_line
  total_line="$(awk '/^[[:space:]]*\|Total[[:space:]]*\|/{print; exit}' "$report")"

  local util_line
  util_line="$(awk '/^[[:space:]]*\|Utilization \(%\)[[:space:]]*\|/{print; exit}' "$report")"

  local est_clk max_lat_cycles max_lat_abs interval_max
  local bram dsp ff lut bram_pct dsp_pct ff_pct lut_pct

  est_clk="$(extract_field "$timing_line" 4)"
  max_lat_cycles="$(extract_field "$latency_line" 3)"
  max_lat_abs="$(extract_field "$latency_line" 5)"
  interval_max="$(extract_field "$latency_line" 7)"

  bram="$(extract_field "$total_line" 3)"
  dsp="$(extract_field "$total_line" 4)"
  ff="$(extract_field "$total_line" 5)"
  lut="$(extract_field "$total_line" 6)"

  bram_pct="$(extract_field "$util_line" 3)"
  dsp_pct="$(extract_field "$util_line" 4)"
  ff_pct="$(extract_field "$util_line" 5)"
  lut_pct="$(extract_field "$util_line" 6)"

  echo "$est_clk,$max_lat_cycles,$max_lat_abs,$interval_max,$bram,$dsp,$ff,$lut,$bram_pct,$dsp_pct,$ff_pct,$lut_pct"
}

# profile_name,clock_period_ns,pipeline_loops,recip_ii,state_update_ii,unroll_gma,unroll_pa,unroll_atpa,unroll_forward,unroll_state,k_partition_mode,k_partition_factor
PROFILES=(
  "balanced_base,10,0,2,2,3,4,4,2,2,0,4"
  "balanced_kcyc,10,0,2,2,3,4,4,2,2,1,4"
  "balanced_recip1,10,0,1,2,3,4,4,2,2,0,4"
  "balanced_stateii1,10,0,1,1,3,4,4,2,2,0,4"
  "mid_parallel,10,0,1,1,4,4,4,3,3,0,4"
  "mid_parallel_kcyc,10,0,1,1,4,4,4,3,3,1,4"
  "high_parallel,10,0,1,1,5,5,5,4,4,0,4"
  "max_dsp,10,64,1,1,6,6,6,8,8,0,4"
  "high_parallel_clk9,9,0,1,1,5,5,5,4,4,0,4"
  "high_parallel_clk8p5,8.5,0,1,1,5,5,5,4,4,0,4"
)

echo "profile,clock_target_ns,pipeline_loops,recip_ii,state_update_ii,unroll_gma,unroll_pa,unroll_atpa,unroll_forward,unroll_state,k_partition_mode,k_partition_factor,estimated_clock_ns,max_latency_cycles,max_latency_abs,interval_max,bram18k,dsp,ff,lut,bram_pct,dsp_pct,ff_pct,lut_pct" > "$SUMMARY_CSV"

for profile in "${PROFILES[@]}"; do
  IFS=',' read -r name clk loops recip_ii state_ii ug up ua uf us k_mode k_factor <<< "$profile"

  echo ""
  echo "=== Running profile: $name (clock=$clk ns, pipeline_loops=$loops, recip_ii=$recip_ii, state_ii=$state_ii, ug=$ug, up=$up, ua=$ua, uf=$uf, us=$us, k_mode=$k_mode, k_factor=$k_factor) ==="

  (
    cd "$PROJECT_DIR"
    HLS_RUN_MODE=synth \
    HLS_CLOCK_PERIOD_NS="$clk" \
    HLS_PIPELINE_LOOPS="$loops" \
    HLS_RECIP_II="$recip_ii" \
    HLS_STATE_UPDATE_II="$state_ii" \
    HLS_UNROLL_GMA_FACTOR="$ug" \
    HLS_UNROLL_PA_FACTOR="$up" \
    HLS_UNROLL_ATPA_FACTOR="$ua" \
    HLS_UNROLL_FORWARD_FACTOR="$uf" \
    HLS_UNROLL_STATE_FACTOR="$us" \
    HLS_K_PARTITION_MODE="$k_mode" \
    HLS_K_PARTITION_FACTOR="$k_factor" \
    vitis-run --mode hls --tcl scripts/run_hls.tcl
  )

  if [[ ! -f "$REPORT_PATH" ]]; then
    echo "ERROR: report not found at $REPORT_PATH"
    exit 1
  fi

  cp "$REPORT_PATH" "$SWEEP_LOG_DIR/${name}_mpc_fpga_top_csynth.rpt"

  metrics="$(extract_metrics "$REPORT_PATH")"
  echo "$name,$clk,$loops,$recip_ii,$state_ii,$ug,$up,$ua,$uf,$us,$k_mode,$k_factor,$metrics" >> "$SUMMARY_CSV"

done

echo ""
echo "Sweep complete."
echo "Summary CSV: $SUMMARY_CSV"
echo "Reports: $SWEEP_LOG_DIR"
